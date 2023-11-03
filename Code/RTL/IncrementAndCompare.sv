module IncrementAndCompare  #(
                            parameter STEP_SIZE = 1/4,
                            parameter INC_TERM_DW_INTEGER = 16,
                            parameter N_DW_INTEGER = 13,
                            parameter ERROR_DW_INTEGER = 14,
                            parameter A_DW_INTEGER = 3,
                            parameter DW_FRACTIONAL = 1/STEP_SIZE
                            )(
                            input                                                   clk,            // Clock signal
                            input                                                   rst,            // Reset signal
                            // Input values
                            input                                                   initiate,       // Initiates a new delay caclulation, locks input values
                            input           [N_DW_INTEGER+DW_FRACTIONAL:0]          n_prev,         // Delay from previous calculation
                            input signed    [A_DW_INTEGER+DW_FRACTIONAL:0]          a_prev,         // Compensated delay from previous calculation
                            input           [2*A_DW_INTEGER+DW_FRACTIONAL:0]        a_prev_sq,      // Compensated squared delay from previous calculation
                            input signed    [DW_INTEGER+DW_FRACTIONAL:0]            comp_term,      // Comparator term going into comparator
                            input signed    [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]   comp_term_prev  // Previous comparator term
                            input signed    [ERROR_DW_INTEGER+DW_FRACTIONAL:0]      error_prev,     // Error from previous delay calculation
                            // Output values
                            output          [N_DW_INTEGER+DW_FRACTIONAL:0]          n_next,         // Delay output
                            output signed   [ERROR_DW_INTEGER+DW_FRACTIONAL:0]      error_next,     // Error output
                            output signed   [A_DW_INTEGER+DW_FRACTIONAL:0]          a_next,         // Compensated delay
                            output          [2*A_DW_INTEGER+DW_FRACTIONAL:0]        a_next_sq,      // Compensated squared delay
                            output signed   [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]   comp_term_next, // Comparator term propagated to next
                            output                                                  ready           // Result ready signal
                            );

    // Constant step size parameter
    localparam [1:0] stepsize = 0.25; 

    // State machine variables
    enum {LOAD, RUN, IDLE} state, nextState;

    // Algorithm registers
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      comp_term_cur_reg;
    logic [N_DW_INTEGER+DW_FRACTIONAL:0]                    n_cur_reg;
    logic signed [ERROR_DW_INTEGER+DW_FRACTIONAL:0]         error_cur_reg;
    logic signed [A_DW_INTEGER+DW_FRACTIONAL:0]             a_cur_reg;
    logic signed [A_DW_INTEGER+DW_FRACTIONAL:0]             a_prev_reg;
    logic [2*A_DW_INTEGER+DW_FRACTIONAL:0]                  a_cur_sq_reg;
    logic                                                   sign_bit;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      a_mul_n_integer_reg;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      a_mul_n_fractional_reg;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      inc_term_reg;

    // Ready signal
    logic   ready_reg;

    // Assigning regs to output
    assign n_next           = n_cur_reg && ready_reg;
    assign error_next       = error_cur_reg && ready_reg;
    assign a_next           = a_cur_reg && ready_reg;
    assign a_next_sq        = a_cur_sq_reg && ready_reg;
    assign comp_term_next   = comp_term_cur_reg && ready_reg;
    assign ready            = ready_reg;

    // Sum of increment term in comparator (should work as a wire)
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]      comparator_sum;
    assign comparator_sum = a_mul_n_integer_reg + a_mul_n_fractional_reg + a_prev_sq_reg + inc_term_reg

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
            nextState = IDLE
        else begin
            case (state)
                IDLE: if(initiate) nextState = LOAD;
                LOAD: nextState = RUN;
                RUN: if (ready_reg) nextState = IDLE;
            endcase
        end
    end

    // Algorithm functionality
    always @(posedge clk) begin
        if (rst) begin
            comp_term_cur_reg           <= '0;
            n_cur_reg                   <= '0;
            error_cur_reg               <= '0;
            a_cur_reg                   <= '0;
            a_prev_reg                  <= '0;
            a_cur_sq_reg                <= '0;
            a_mul_n_integer_reg         <= '0;
            a_mul_n_fractional_reg      <= '0;
            inc_term_reg                <= '0;
            ready_reg                   <= 1'b0;
            sign_bit                    <= 1'b0;
        end
        else begin
            case(state)
                IDLE: begin
                    comp_term_cur_reg           <= '0;
                    n_cur_reg                   <= '0;
                    error_cur_reg               <= '0;
                    a_cur_reg                   <= '0;
                    a_prev_reg                  <= '0;
                    a_cur_sq_reg                <= '0;
                    a_mul_n_integer_reg         <= '0;
                    a_mul_n_fractional_reg      <= '0;
                    inc_term_reg                <= '0;
                    ready_reg                   <= 1'b0;
                    sign_bit                    <= 1'b0;
                end
                LOAD: begin
                    n_cur_reg           <= n_prev;
                    a_cur_reg           <= a_prev;
                    a_prev_reg          <= a_prev;
                    a_cur_sq_reg        <= a_prev_sq;
                    a_prev_sq_reg       <= a_prev_sq;
                    comp_term_cur_reg   <= comp_term + {error_prev[ERROR_DW_INTEGER+DW_FRACTIONAL], 2'd0, error_prev[ERROR_DW_INTEGER+DW_FRACTIONAL-1:0};
                    case(a_prev[A_DW_INTEGER+DW_FRACTIONAL-1:DW_FRACTIONAL])    // Calculating integer part of 2*N_prev*a_prev
                        "00":   a_mul_n_integer_reg <= 0;
                        "01":   a_mul_n_integer_reg <= {a_prev[A_DW_INTEGER+DW_FRACTIONAL], n_prev << 1};               // 2*N_prev
                        "10":   a_mul_n_integer_reg <= {a_prev[A_DW_INTEGER+DW_FRACTIONAL], n_prev << 2};               // 4*N_prev = 2^2*N_prev
                        "11":   a_mul_n_integer_reg <= {a_prev[A_DW_INTEGER+DW_FRACTIONAL], n_prev << 1 + n_prev << 2}; // 6*N_prev = 2^2*N_prev + 2*N_prev
                    endcase
                    case(a_prev[DW_FRACTIONAL-1:DW_FRACTIONAL-2])               // Calculating fractional part of 2*N_prev*a_prev (step size = 1/4)
                        "00":   a_mul_n_fractional_reg <= 0;
                        "01":   a_mul_n_fractional_reg <= {a_prev[A_DW_INTEGER+DW_FRACTIONAL], 13'd0, n_prev};                  // N_prev
                        "10":   a_mul_n_fractional_reg <= {a_prev[A_DW_INTEGER+DW_FRACTIONAL], 13'd0, n_prev >> 1};             // (1/2)*N_prev
                        "11":   a_mul_n_fractional_reg <= {a_prev[A_DW_INTEGER+DW_FRACTIONAL], 13'd0, n_prev + n_prev >> 1};    // N_prev + (1/2)*N_prev
                    endcase
                    if (comp_term + {error_prev[ERROR_DW_INTEGER+DW_FRACTIONAL], 2'd0, error_prev[ERROR_DW_INTEGER+DW_FRACTIONAL-1:0} < comp_term_prev)
                        sign_bit <= 1;      // -1
                end
                RUN: begin
                    // Comparator: 2*a*N_prev+a^2 < comp_term_w_error
                    if (sign_bit == 1)
                        if(comparator_sum < comp_term_cur_reg) begin
                            n_cur_reg           <= n_cur_reg + a_cur_reg - {sign_bit,stepsize};
                            a_cur_reg           <= a_cur_reg - {sign_bit,stepsize};
                            a_cur_sq_reg        <= a_cur_sq_reg - (a_cur_reg >> 1)  - (4'b0001);
                            error_cur_reg       <= comp_term_cur_reg - comparator_sum;
                            ready_reg           <= 1'b1;
                        end
                        else begin   
                            // Updating incremented terms
                            inc_term_reg <= inc_term_reg + {sign_bit,(n_cur_reg >> 1)} + {sign_bit,(a_prev_reg >> 1)} + (4'b0001);
                            a_cur_reg    <= a_cur_reg + {sign_bit,stepsize};
                            a_cur_sq_reg <= a_cur_sq_reg + (a_cur_reg >> 1)  + (4'b0001);
                        end

                    // Comparator: 2*a*N_prev+a^2 > comp_term_w_error
                    else if (sign_bit == 0)
                        if(comparator_sum > comp_term_cur_reg) begin
                            n_cur_reg           <= n_cur_reg + a_cur_reg - {sign_bit,stepsize};
                            a_cur_reg           <= a_cur_reg - {sign_bit,stepsize};
                            a_cur_sq_reg        <= a_cur_sq_reg - (a_cur_reg >> 1)  - (4'b0001);
                            error_cur_reg       <= comp_term_cur_reg - comparator_sum;
                            ready_reg           <= 1'b1;
                        end
                        else begin   
                            // Updating incremented terms
                            inc_term_reg <= inc_term_reg + {sign_bit,(n_cur_reg >> 1)} + {sign_bit,(a_prev_reg >> 1)} + (4'b0001);
                            a_cur_reg    <= a_cur_reg + {sign_bit,stepsize};
                            a_cur_sq_reg <= a_cur_sq_reg + (a_cur_reg >> 1)  + (4'b0001);
                        end
                end
    end
endmodule