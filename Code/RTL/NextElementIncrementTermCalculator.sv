/* Module calculating the term K_n = A_0(2n+1) +/- C_0 for all n elements*/
module NextElementIncrementTermCalculator  #(
                            parameter DW_INTEGER    = 18,
                            parameter DW_INPUT      = 8,
                            parameter DW_FRACTION   = 6,
                            parameter ANGLE_DW      = 8
                            )(
                            input                                                   clk,                // Clock signal
                            input                                                   rst,                // Reset signal
                            // Input values
                            input                                                   initiate,           // Initiates a new delay caclulation for all n, locks input values
                            input                                                   ack,                // Acknowledges output has been read
                            input [DW_INPUT-1:0]                                    r_0,                // R_0 input
                            input [ANGLE_DW-1:0]                                    angle,              // angle input
                            // Output values
                            output signed   [DW_INTEGER+DW_FRACTION:0]              output_term_pos_n,  // Output comparator term (K_n)
                            output signed   [DW_INTEGER+DW_FRACTION:0]              output_term_neg_n,  // Output comparator term (K_n)
                            output                                                  ready               // Result ready signal
                            );

    // Term A_0 = (f_s*p/v_s)^2 = 16.47094788 stored in memory/LUT
    localparam [5+DW_FRACTION:0] a_0 = 12'b010000_011110; // Quantized value = 16.46875

    // State machine variables
    enum {LOAD, RUN1, WAIT, RUN2, IDLE} state, nextState;

    // Registers holding results
    logic [ANGLE_DW:0]                          angle_reg;
    logic signed [DW_FRACTION+DW_INTEGER:0]     output_term_pos_n_reg;
    logic signed [DW_FRACTION+DW_INTEGER:0]     output_term_neg_n_reg;

    // Control signals
    logic [5:0]                         counter;
    logic                               last_element;
    logic                               ready_reg;

    // Cordic signals
    logic signed [16+DW_FRACTION:0]     cordic_result;
    logic [DW_FRACTION+DW_INTEGER:0]    cordic_x_scale;
    logic                               cordic_initiate;
    logic                               cordic_ack;
    logic                               cordic_ready;

    // Assigning outputs
    assign ready                = (state == WAIT) ? ready_reg : '0;
    assign output_term_pos_n    = (state == WAIT) ? output_term_pos_n_reg : '0;
    assign output_term_neg_n    = (state == WAIT) ? output_term_neg_n_reg : '0;

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
                IDLE:   if(initiate)            nextState = LOAD;
                LOAD:                           nextState = RUN1;
                RUN1:   if(cordic_ready)        nextState = WAIT;
                WAIT: begin
                    if (ack && last_element)    nextState = IDLE;
                    else if (ack)               nextState = RUN2;
                end
                RUN2:                           nextState = WAIT;
            endcase
        end
    end

    // Calculation functionality
    always @(posedge clk) begin
        if (rst) begin
            cordic_x_scale          <= '0;
            output_term_pos_n_reg   <= '0;
            output_term_neg_n_reg   <= '0;
            angle_reg               <= '0;
            counter                 <= '0;
            cordic_initiate         <= 1'b0;
            last_element            <= 1'b0;
            ready_reg               <= 1'b0;
            cordic_ack              <= 1'b0;
        end
        else begin
            case(state)
                IDLE: begin     // RESET ALL VALUES
                    cordic_x_scale          <= '0;
                    output_term_pos_n_reg   <= '0;
                    output_term_neg_n_reg   <= '0;
                    angle_reg               <= '0;
                    counter                 <= '0;
                    cordic_initiate         <= 1'b0;
                    last_element            <= 1'b0;
                    ready_reg               <= 1'b0;
                    cordic_ack              <= 1'b0;
                end
                LOAD: begin // CONST MULTIPLIER
                    // NOTE: CAN BE DONE ITERATIVELY USING A SINGLE 2-ADDER
                    cordic_x_scale      <= (r_0 << 7+6) + (r_0 << 1+6) + (r_0 << 6) + (r_0 << 6-1) + (r_0 << 6-2) + (r_0 << 6-6) + (r_0 >>> 9-6);     // (2*p*(f_s/v_s)^2) * 10^-3 = 131.767
                    angle_reg           <= angle;
                    cordic_initiate     <= 1'b1;
                end
                RUN1: begin // CORDIC (angle,input)
                    cordic_initiate     <= 1'b0;
                    if (cordic_ready) begin
                        output_term_pos_n_reg   <= signed'(a_0) - cordic_result;      // A_0 - C_0
                        output_term_neg_n_reg   <= signed'(a_0) + cordic_result;      // A_0 + C_0
                        ready_reg               <= cordic_ready;
                        cordic_ack              <=  1'b1;
                        counter                 <= 5'd1;
                    end
                end
                WAIT: if(ack) begin
                        ready_reg   <= 1'b0;
                    end
                RUN2: begin // INCREMENT K_N
                    output_term_pos_n_reg   <= output_term_pos_n_reg + (a_0 << 1);    // A_0 * (2*n + 1) - C_0
                    output_term_neg_n_reg   <= output_term_neg_n_reg + (a_0 << 1);    // A_0 * (2*n + 1) + C_0
                    ready_reg               <= 1'b1;
                    if (counter == 5'd31)
                        last_element    <= 1'b1;
                    else
                        counter         <= counter + 1;
                end
            endcase
		end
    end


    CordicCosine # (
        .DW_ANGLE(7),
        .DW_FRACTION(6),
        .DW_CALCULATION_TERMS(16)
    )   cordic_inst  (
        .clk(clk),
        .rst(rst),
        .initiate(cordic_initiate),
        .angle(angle_reg),
        .x_scale(cordic_x_scale),
        .ack(cordic_ack),

        .result(cordic_result),
        .ready(cordic_ready)
    );
endmodule