module IncrementAndCompare  #(
                            parameter STEP_SIZE = 1,
                            parameter INC_TERM_DW_INTEGER = 16,
                            parameter N_DW_INTEGER = 13,
                            parameter ERROR_DW_INTEGER = 14,
                            parameter A_DW_INTEGER = 4,
                            parameter DW_FRACTIONAL = 4
                            )(
                            input                                                   clk,            // Clock signal
                            input                                                   rst,            // Reset signal
                            // Input values
                            input                                                   initiate,       // Initiates a new delay caclulation, locks input values
                            input                                                   ack,            // Acknowledges output has been read
                            input           [N_DW_INTEGER+DW_FRACTIONAL:0]          n_prev,         // Delay from previous calculation
                            input signed    [A_DW_INTEGER+DW_FRACTIONAL:0]          a_prev,         // Compensated delay from previous calculation
                            input           [2*A_DW_INTEGER+DW_FRACTIONAL:0]        a_prev_sq,      // Compensated squared delay from previous calculation
                            input signed    [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]   comp_term,      // Comparator term going into comparator
                            input signed    [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]   comp_term_prev, // Previous comparator term
                            input signed    [ERROR_DW_INTEGER+DW_FRACTIONAL:0]      error_prev,     // Error from previous delay calculation
                            // Output values
                            output          [N_DW_INTEGER+DW_FRACTIONAL:0]          n_next,         // Delay output
                            output signed   [ERROR_DW_INTEGER+DW_FRACTIONAL:0]      error_next,     // Error output
                            output signed   [A_DW_INTEGER+DW_FRACTIONAL:0]          a_next,         // Compensated delay
                            output          [2*A_DW_INTEGER+DW_FRACTIONAL:0]        a_next_sq,      // Compensated squared delay
                            output signed   [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]   comp_term_next, // Comparator term propagated to next
                            output                                                  ready,          // Result ready signal
                            output                                                  available
                            );

    // Constant step size parameter
    localparam signed [3:0] stepsize = 4'b0100; 
    localparam signed [3:0] increment = 4'b0001; 

    // State machine variables
    enum {LOAD, RUN, WAIT, IDLE} state, nextState;

    // Algorithm registers
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      comp_term_cur_reg;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      comp_term_next_reg;
    logic signed [N_DW_INTEGER+DW_FRACTIONAL:0]             n_cur_reg;
    logic signed [ERROR_DW_INTEGER+DW_FRACTIONAL:0]         error_cur_reg;
    logic signed [A_DW_INTEGER+DW_FRACTIONAL:0]             a_cur_reg;
    logic signed [A_DW_INTEGER+DW_FRACTIONAL:0]             a_prev_reg;
    logic signed [2*A_DW_INTEGER+DW_FRACTIONAL:0]           a_prev_sq_reg;
    logic signed [2*A_DW_INTEGER+DW_FRACTIONAL:0]           a_cur_sq_reg;
    logic                                                   sign_bit;
    logic signed [21:0]                                     a_mul_n_integer_reg;        
    logic signed [19:0]                                     a_mul_n_fractional_reg;     // Additional sign bit and addition extension
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      inc_term_reg;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      comparator_sum_prev_reg;
    logic [4:0]				    		                    counter;

    // Ready signal
    logic   ready_reg;

    // Assigning regs to output
    assign n_next           = (state == WAIT) ? n_cur_reg           : '0;
    assign error_next       = (state == WAIT) ? error_cur_reg       : '0;
    assign a_next           = (state == WAIT) ? a_cur_reg           : '0;
    assign a_next_sq        = (state == WAIT) ? a_cur_sq_reg        : '0;
    assign comp_term_next   = (state == WAIT) ? comp_term_next_reg  : '0;
    assign ready            = (state == WAIT) ? ready_reg           : '0;
    assign available        = (state == IDLE);

    // Sum of increment term in comparator (should work as a wire)
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      comparator_sum;
    logic signed [A_DW_INTEGER+DW_FRACTIONAL:0]             a_prev_shifted, a_cur_shifted;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      delta_increment_term_pos,delta_increment_term_neg;

    assign comparator_sum           = a_mul_n_integer_reg + a_mul_n_fractional_reg + a_prev_sq_reg + inc_term_reg; // current 2*a*N + a^2
    assign a_prev_shifted           = (a_prev_reg >>> 1);
    assign a_cur_shifted            = (a_cur_reg >>> 1);
    assign delta_increment_term_neg = - (n_cur_reg >> 1) - a_prev_shifted + increment - signed'({counter,4'd0} >> 3);
    assign delta_increment_term_pos = (n_cur_reg >> 1) + a_prev_shifted + increment + signed'({counter,4'd0} >> 3);


    // Locking next state
    always @(posedge clk) begin
        if(rst) 
            state <= IDLE;
        else
            state <= nextState;
    end

    // FSM functionality
    always_comb begin : FSM
        if(rst)
            nextState = IDLE;
        else begin
            case (state)
                IDLE: if(initiate)      nextState = LOAD; else nextState = IDLE;
                LOAD:                   nextState = RUN;
                RUN:    if ((comparator_sum < comp_term_cur_reg && sign_bit == 1) || (comparator_sum > comp_term_cur_reg && sign_bit == 0))
                            nextState = WAIT; 
                        else 
                            nextState = RUN;
                WAIT: if(ack)           nextState = IDLE; else nextState = WAIT;
            endcase 
        end
    end

    // Algorithm functionality
    always @(posedge clk) begin
        if (rst) begin
            comp_term_cur_reg           <= '0;
            comp_term_next_reg          <= '0;
            n_cur_reg                   <= '0;
            error_cur_reg               <= '0;
            a_cur_reg                   <= '0;
            a_prev_reg                  <= '0;
            a_cur_sq_reg                <= '0;
            a_prev_sq_reg               <= '0;
            sign_bit                    <= '0;
            a_mul_n_integer_reg         <= '0;
            a_mul_n_fractional_reg      <= '0;
            inc_term_reg                <= '0;
            comparator_sum_prev_reg     <= '0;
            counter                     <= '0;
            ready_reg                   <= '0;
        end
        else begin
            case(state)
                IDLE: begin
                    comp_term_cur_reg           <= '0;
                    comp_term_next_reg          <= '0;
                    n_cur_reg                   <= '0;
                    error_cur_reg               <= '0;
                    a_cur_reg                   <= '0;
                    a_prev_reg                  <= '0;
                    a_cur_sq_reg                <= '0;
                    a_prev_sq_reg               <= '0;
                    sign_bit                    <= '0;
                    a_mul_n_integer_reg         <= '0;
                    a_mul_n_fractional_reg      <= '0;
                    inc_term_reg                <= '0;
                    comparator_sum_prev_reg     <= '0;
                    counter                     <= '0;
                    ready_reg                   <= '0;
                end
                LOAD: begin
                    n_cur_reg           <= n_prev;
                    a_cur_reg           <= a_prev;
                    a_prev_reg          <= a_prev;
                    a_cur_sq_reg        <= a_prev_sq;
                    a_prev_sq_reg       <= a_prev_sq;
                    comp_term_cur_reg   <= comp_term + error_prev;
                    case(a_prev[6:4])    // Calculating integer part of 2*N_prev*a_prev
                        3'b000:     a_mul_n_integer_reg <= 0;     
                        3'b001:     a_mul_n_integer_reg <= (n_prev << 1);                   //2*N_prev
                        3'b010:     a_mul_n_integer_reg <= (n_prev << 2);                   //4*N_prev           
                        3'b011:     a_mul_n_integer_reg <= (n_prev << 1) + (n_prev << 2);   //6*N_prev     
                        3'b100: begin
                            if(a_prev[A_DW_INTEGER+DW_FRACTIONAL] == 1'b1)  a_mul_n_integer_reg <= -(n_prev << 3);  // -8*N_prev = -2^3*N_prev
                            else                                            a_mul_n_integer_reg <= (n_prev << 3);   // 8*N_prev = 2^3*N_prev
                        end
                        3'b101:     a_mul_n_integer_reg <= -(n_prev << 1) - (n_prev << 2);  //-6*N_prev
                        3'b110:     a_mul_n_integer_reg <= -(n_prev << 2);                  //-4*N_prev
                        3'b111:     a_mul_n_integer_reg <= -(n_prev << 1);                  //-2*N_prev
                        default:;  // Should never happen
                    endcase
                    case(a_prev[3:2])               // Calculating fractional part of 2*N_prev*a_prev (step size = 1/4)
                        2'b00:   a_mul_n_fractional_reg <= 0;
                        2'b01: begin
                            if(a_prev[A_DW_INTEGER+DW_FRACTIONAL] == 1'b1)  a_mul_n_fractional_reg <= n_prev + (n_prev >> 1);     // 0.75*2*N_prev
                            else                                            a_mul_n_fractional_reg <= (n_prev >> 1);                // 0.25*2*N_prev
                        end
                        2'b10: begin
                            if(a_prev[A_DW_INTEGER+DW_FRACTIONAL] == 1'b1)  a_mul_n_fractional_reg <= n_prev;                      // 0.5*2*N_prev
                            else                                            a_mul_n_fractional_reg <= n_prev;                       // 0.5*2*N_prev
                        end
                        2'b11: begin
                            if(a_prev[A_DW_INTEGER+DW_FRACTIONAL] == 1'b1)  a_mul_n_fractional_reg <= (n_prev >> 1);               // 0.25*2*N_prev
                            else                                            a_mul_n_fractional_reg <= n_prev + (n_prev >> 1);       // 0.75*2*N_prev
                        end  
                    endcase
                    if (comp_term + error_prev < comp_term_prev) begin
                        sign_bit                    <= 1;      // -1
                    end
                end
                RUN: begin
		            counter <= counter + 1;
                    // Comparator: 2*a*N_prev+a^2 < comp_term_w_error
                    if (sign_bit == 1)
                        if(comparator_sum < comp_term_cur_reg) begin
                            n_cur_reg           <= n_cur_reg + a_cur_reg + stepsize;
                            a_cur_reg           <= a_cur_reg + stepsize;
                            a_cur_sq_reg        <= a_cur_sq_reg + a_cur_shifted + increment;
                            if (counter == 0) begin
                                error_cur_reg       <= comp_term_cur_reg - (comparator_sum + delta_increment_term_pos);
                                comp_term_next_reg  <= comparator_sum + delta_increment_term_pos;
                            end
                            else begin
                                error_cur_reg       <= comp_term_cur_reg - comparator_sum_prev_reg;
                                comp_term_next_reg  <= comparator_sum_prev_reg;
                            end
                            ready_reg           <= 1'b1;
                        end
                        else begin   
                            // Updating incremented terms
                            comparator_sum_prev_reg <= comparator_sum;
                            a_cur_reg    <= a_cur_reg - stepsize;
                            inc_term_reg <= inc_term_reg + delta_increment_term_neg;
                            a_cur_sq_reg <= a_cur_sq_reg - a_cur_shifted + increment;
                        end

                    // Comparator: 2*a*N_prev+a^2 > comp_term_w_error
                    else if (sign_bit == 0)
                        if(comparator_sum > comp_term_cur_reg) begin
                            n_cur_reg           <= n_cur_reg + a_cur_reg - stepsize;
                            a_cur_reg           <= a_cur_reg - stepsize;
                            a_cur_sq_reg        <= a_cur_sq_reg - a_cur_shifted + increment;
                            if (counter == 0) begin
                                error_cur_reg       <= comp_term_cur_reg - (comparator_sum + delta_increment_term_neg);
                                comp_term_next_reg  <= comparator_sum + delta_increment_term_neg;
                            end
                            else begin
                                error_cur_reg       <= comp_term_cur_reg - comparator_sum_prev_reg;
                                comp_term_next_reg  <= comparator_sum_prev_reg;
                            end
                            ready_reg           <= 1'b1;
                        end
                        else begin   
                            // Updating incremented terms
			                comparator_sum_prev_reg <= comparator_sum;
                            a_cur_reg    <= a_cur_reg + stepsize;
                            inc_term_reg <= inc_term_reg + delta_increment_term_pos;
                            a_cur_sq_reg <= a_cur_sq_reg + a_cur_shifted + increment;
                        end
                end
            endcase
        end
    end
endmodule