/* Module calculating the term K_n = A_0(2n+1) +/- C_0 for all n elements*/
module NextPointIncrementTermCalculator  #(
                            parameter DW_INTEGER    = 18,
                            parameter DW_INPUT      = 8,
                            parameter DW_FRACTION   = 8,
                            parameter ANGLE_DW      = 8
                            )(
                            input                                       clk,            // Clock signal
                            input                                       rst,            // Reset signal
                            // Input values
                            input                                       configure,      // Signal initiating first calculation and configures values
                            input                                       ack,            // Acknowledges output has been read
                            input [DW_INPUT-1:0]                        r_0,            // R_0 input
                            input [ANGLE_DW-1:0]                        angle,          // angle input
                            // Output values
                            output signed   [DW_INTEGER+DW_FRACTION:0]  output_term_pos_n,  // Comparator term output for pos n
                            output signed   [DW_INTEGER+DW_FRACTION:0]  output_term_neg_n,  // Comparator term output for neg n
                            output                                      ready               // Result ready signal
                            );

    // Term 2* p * f_s / v_s = 16.11688 stored in memory/LUT
    localparam [4+DW_FRACTION:0] cordic_const_in = 13'b01000_00011111; // Quantized value for 2 * p * f_s / v_s

    // State machine variables
    enum {LOAD, CONFIGURE, WAIT_CONFIGURE, WAIT, RUN1, RUN0 , IDLE} state, nextState;

    // Registers holding results
    logic [ANGLE_DW:0]                          angle_reg;
    logic signed [DW_FRACTION+DW_INTEGER:0]     output_term_pos_n_reg;
    logic signed [DW_FRACTION+DW_INTEGER:0]     output_term_neg_n_reg;
    logic signed [DW_FRACTION+DW_INTEGER:0]     const_R_0_term;
    logic signed [DW_FRACTION+DW_INTEGER:0]     value_N0;
    logic signed [DW_FRACTION+DW_INTEGER:0]     cordic_out;

    // Control signals
    logic [5:0]                         counter_elements;
    logic                               last_element;
    logic                               ready_reg;

    // Cordic signals
    logic signed [16+DW_FRACTION:0]     cordic_result;
    logic [DW_FRACTION+DW_INTEGER:0]    cordic_x_scale;
    logic                               cordic_initiate;
    logic                               cordic_ack;
    logic                               cordic_ready;

    // Assigning outputs
    assign ready                = (state == WAIT || state == WAIT_CONFIGURE) ? ready_reg : '0;
    assign output_term_pos_n    = (state == WAIT || state == WAIT_CONFIGURE) ? output_term_pos_n_reg : '0;
    assign output_term_neg_n    = (state == WAIT || state == WAIT_CONFIGURE) ? output_term_neg_n_reg : '0;

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
                IDLE:   if(configure)           nextState = CONFIGURE;
                CONFIGURE: if(cordic_ready)     nextState = WAIT_CONFIGURE;
                WAIT_CONFIGURE: if (ack)        nextState = RUN1;
                RUN0:                           nextState = WAIT;   // Calculating for element n = 0
                RUN1:                           nextState = WAIT;   // Calculating for element except n = 0
                WAIT: begin
                    if (ack && last_element)    nextState = RUN0;
                    else if (ack)               nextState = RUN1;
                end
            endcase
        end
    end

    // Calculation functionality
    always @(posedge clk) begin
        if (rst) begin
            cordic_x_scale   <= '0;
            output_term_pos_n_reg <= '0;
            output_term_neg_n_reg <= '0;
            angle_reg       <= '0;
            counter_elements<= '0;
            const_R_0_term  <= '0;
            value_N0        <= '0;
            cordic_out      <= '0;
            cordic_initiate <= 1'b0;
            last_element    <= 1'b0;
            ready_reg       <= 1'b0;
            cordic_ack      <= 1'b0;
        end
        else begin
            case(state)
                IDLE: begin     // RESET ALL VALUES
                    cordic_x_scale      <= '0;
                    output_term_pos_n_reg   <= '0;
                    output_term_neg_n_reg   <= '0;
                    angle_reg           <= '0;
                    counter_elements    <= '0;
                    const_R_0_term      <= '0;
                    value_N0            <= '0;
                    cordic_out          <= '0;
                    cordic_initiate     <= 1'b0;
                    last_element        <= 1'b0;
                    ready_reg           <= 1'b0;
                    cordic_ack          <= 1'b0;
                end
                CONFIGURE: begin // CONST MULTIPLIER
                    // NOTE: CAN BE DONE ITERATIVELY USING A SINGLE 2-ADDER
                    const_R_0_term      <= (r_0 << 8+5) + (r_0 << 8-1) - (r_0 << 8-5) - (r_0 >>> 10-8) - (r_0 >>> 12-8) + (r_0 >>> 17-8);     // R_0 * 2 * f_s / v_s
                    angle_reg           <= angle;
                    cordic_x_scale      <= {cordic_const_in};
                    cordic_initiate     <= 1'b1;
                end
                WAIT_CONFIGURE: begin // Making output for element n = 0 in point k = 1 ready
                    cordic_initiate     <= 1'b0;
                    if (cordic_ready) begin
                        output_term_pos_n_reg <= 10'b01_00000000 + signed'(const_R_0_term);           // 1 + 2*R_0*f_s/v_s
                        output_term_neg_n_reg <= 10'b01_00000000 + signed'(const_R_0_term);           // 1 + 2*R_0*f_s/v_s
                        value_N0        <= 10'b01_00000000 + signed'(const_R_0_term);           // 1 + 2*R_0*f_s/v_s
                        cordic_out      <= cordic_result;                                       // 2*p*f_s/v_s * cos(angle)
                        ready_reg       <= cordic_ready;
                        cordic_ack      <= 1'b1;
                    end
                end
                RUN0: begin // Calculating value for element n = 0
                    value_N0                <= value_N0 + 11'b010_00000000;        // 1 + 2*R_0*f_s/v_s + 2k
                    output_term_pos_n_reg   <= value_N0 + 11'b010_00000000;
                    output_term_neg_n_reg   <= value_N0 + 11'b010_00000000;
                    ready_reg               <= 1'b1;
                end
                RUN1: begin // Calculating value for element n+1
                    output_term_pos_n_reg       <= output_term_pos_n_reg - cordic_out;     // (1 + 2*R_0*f_s/v_s + 2k) - n(2*p*f_s/v_s * cos(angle))
                    output_term_neg_n_reg       <= output_term_neg_n_reg + cordic_out;
                    ready_reg                   <= 1'b1;
                    if (counter_elements == 5'd30) begin
                        last_element        <= 1'b1;
                        counter_elements    <= 5'd0;
                    end
                    else
                        counter_elements    <= counter_elements + 1;
                end
                WAIT: if(ack) begin
                        ready_reg       <= 1'b0;
                        last_element    <= 1'b0;
                    end
            endcase
		end
    end


    CordicCosine # (
        .DW_ANGLE(7),
        .DW_FRACTION(DW_FRACTION),
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