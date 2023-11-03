/* Module calculating the term K_n = A_0(2n+1) +/- C_0 for all n elements*/
module NextElementIncrementTermCalculator  #(
                            parameter DW_INTEGER   = 18,
                            parameter DW_FRACTION  = 3,
                            parameter ANGLE_DW              = 8,

                            )(
                            input                                                   clk,            // Clock signal
                            input                                                   rst,            // Reset signal
                            // Input values
                            input                                                   initiate,       // Initiates a new delay caclulation, locks input values
                            input                                                   ack,            // Acknowledges output has been read
                            input [R_DW_INTEGER+R_DW_FRACTION:0]                    r_0,            // R_0 input
                            input [ANGLE_DW:0]                                      angle,          // angle input
                            // Output values
                            output signed   [DW_INTEGER+DW_FRACTIONAL:0]   output_term,    // Comparator term propagated to next
                            output                                                  ready           // Result ready signal
                            );

    // Term A_0 = (f_s*p/v_s)^2 = 16.47094788 stored in memory/LUT
    localparam [DW_INTEGER+DW_FRACTION:0] a_0 = 16.5; // Quantized value

    // State machine variables
    enum {LOAD, RUN_INIT, WAIT, RUN_NEXT, IDLE} state, nextState;

    logic [DW_FRACTION+DW_INTEGER:0]    cordic_in_reg;
    logic [DW_FRACTION+DW_INTEGER:0]    output_term_reg;

    logic [5:0]                         counter;
    logic                               cordic_initiate;
    logic                               last_element;
    logic                               done;


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
                IDLE:       if(initiate)        nextState = LOAD;
                LOAD:                           nextState = RUN1;
                RUN_INIT:   if (done)           nextState = WAIT;
                WAIT: begin
                            if (ack && last_element)    nextState = IDLE;
                            else if (ack)               nextState = RUN_NEXT;
                end
                RUN_NEXT:   if (done)           nextState = WAIT;
            endcase
        end
    end

    // Calculation functionality
    always @(posedge clk) begin
        if (rst) begin
            cordic_in_reg   <= '0;
            output_term_reg <= '0;
            counter         <= '0;
            cordic_initiate <= 1'b0;
            last_element    <= 1'b0;
            done            <= 1'b0;
        end
        else
            case(state) begin
                IDLE: begin     // RESET ALL VALUES
                    cordic_in_reg       <= '0;
                    output_term_reg     <= '0;
                    counter             <= '0;
                    cordic_initiate     <= 1'b0;
                    last_element        <= 1'b0;
                    done                <= 1'b0;
                end
                LOAD: begin     // CONST MULTIPLIER
                    // NOTE: CAN BE DONE ITERATIVELY USING A SINGLE 2-ADDER
                    cordic_in_reg       <= (r_0 << 17) + (r_0 << 9) + (r_0 << 7) + (r_0 << 5) + (r_0 << 4) + (r_0 << 2) + (r_0 << 1) + r_0;
                    cordic_initiate     <= 1'b1;
                end
                RUN_INIT: begin // CORDIC (angle,input)
                    if (cordic_done) begin
                        output_term_reg <= a_0 - cordic_out;    // FROM CORDIC MODULE
                        done            <= cordic_done;         // FROM CORDIC MODULE
                        counter         <= 5'd1;
                    end
                end
                WAIT:           // DO NOTHING
                RUN_NEXT: begin // INCREMENT K_N
                    output_term_reg     <= output_term_reg + (a_0 << 1);
                    if (counter == 5'd31)
                        last_element    <= 1'b1;
                    else
                        counter         <= counter + 1;
                end
            endcase
    end
endmodule