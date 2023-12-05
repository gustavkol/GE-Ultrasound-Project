/* Module calculating the term L_kn = 2k + 1 + B_n for all n elements*/
module NextPointIncrementTermCalculator  #(
                            parameter DW_INTEGER    = 16,
                            parameter DW_INPUT      = 8,
                            parameter DW_FRACTION   = 8,
                            parameter DW_ANGLE      = 8,
                            parameter NUM_ELEMENTS  = 64
                            )(
                            input                                       clk,                                // Clock signal
                            input                                       rst,                                // Reset signal
                            // Input values
                            input                                       configure,                          // Signal initiating first calculation and configures values
                            input                                       ack,                                // Acknowledges output has been read
                            input                                       final_scanpoint,                    // Signal indicating last point of scanline
                            input [DW_INPUT-1:0]                        r_0,                                // R_0 input
                            input [DW_ANGLE-1:0]                        angle,                              // angle input
                            // Output values
                            output signed [DW_INTEGER+DW_FRACTION-4:0]  output_terms    [NUM_ELEMENTS-1:0], // Comparator term outputs
                            output                                      ready,                              // Result ready signal
                            output                                      done_configuring                    // Signal going high after configuring is done
                            );

    localparam [5+8-1:0] cordic_const_in = 13'b01000_00100110; // Quantized value for 2 * p * f_s / v_s

    // State machine variables
    enum {LOAD, CONFIGURE, WAIT, RUN_NEXT, IDLE} state, nextState;

    // Registers holding results
    logic [DW_ANGLE-1:0]                        angle_reg;
    logic signed [DW_FRACTION+DW_INTEGER:0]     output_term_neg_n_reg [NUM_ELEMENTS-1:0];
    logic signed [DW_FRACTION+DW_INTEGER:0]     output_term_pos_n_reg [NUM_ELEMENTS-1:0];
    logic signed [DW_FRACTION+DW_INTEGER:0]     const_R_0_term;
    logic signed [16+6-1:0]                     cordic_out;

    // Control signals
    logic [5:0]                         counter_elements;
    logic                               ready_reg;

    // Cordic signals
    logic signed [16+8-1:0]             cordic_result;
    logic [16+8-1:0]                    cordic_x_scale;
    logic                               cordic_initiate;
    logic                               cordic_ack;
    logic                               cordic_ready;
    logic                               done_configuring_reg;

    // Assigning outputs
    generate
        genvar i;
        for (i = 0; i < 32; i = i + 1) begin : COMP_INST
            assign output_terms[32+i]    = output_term_pos_n_reg[i+1][DW_INTEGER+DW_FRACTION:4];
            assign output_terms[31-i]    = output_term_neg_n_reg[i][DW_INTEGER+DW_FRACTION:4];
        end
    endgenerate

    assign ready                = (state == WAIT) ? ready_reg : '0;
    assign done_configuring     = done_configuring_reg;


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
                IDLE:           if(configure)                   nextState = LOAD;
                LOAD:           if(cordic_ready || angle == 90) nextState = CONFIGURE;
                CONFIGURE:      if(ready_reg)                   nextState = WAIT;
                RUN_NEXT:                                       nextState = WAIT;
                WAIT: begin
                    if (ack && final_scanpoint)                 nextState = IDLE;
                    else if (ack)                               nextState = RUN_NEXT;
                end
            endcase
        end
    end

    // Calculation functionality
    always @(posedge clk) begin
        if (rst) begin
            cordic_x_scale          <= '0;
            angle_reg               <= '0;
            counter_elements        <= '0;
            const_R_0_term          <= '0;
            cordic_out              <= '0;
            cordic_initiate         <= 1'b0;
            ready_reg               <= 1'b0;
            cordic_ack              <= 1'b0;
            done_configuring_reg    <= 1'b0;
            for (int i = 0; i <= 32; i = i + 1) begin
                output_term_neg_n_reg[i] = '0;
                output_term_pos_n_reg[i] = '0;
            end
        end
        else begin
            case(state)
                IDLE: begin
                    cordic_x_scale          <= '0;
                    angle_reg               <= '0;
                    counter_elements        <= '0;
                    const_R_0_term          <= '0;
                    cordic_out              <= '0;
                    cordic_initiate         <= 1'b0;
                    ready_reg               <= 1'b0;
                    cordic_ack              <= 1'b0;
                    done_configuring_reg    <= 1'b0;
                    for (int i = 0; i <= 32; i = i + 1) begin
                        output_term_neg_n_reg[i] = '0;
                        output_term_pos_n_reg[i] = '0;
                    end
                end
                LOAD: begin
                    const_R_0_term      <= (r_0 << DW_FRACTION+5)       // R_0 * 2 * f_s / v_s
                                            + (r_0 << DW_FRACTION-1) 
                                            - (r_0 << DW_FRACTION-5) 
                                            - (r_0 >>> 10-DW_FRACTION); 
                                            //- (r_0 >>> 12-DW_FRACTION) 
                                            //+ (r_0 >>> 17-DW_FRACTION);
                    angle_reg           <= angle;
                    if (~(angle == 90)) begin
                        cordic_x_scale      <= cordic_const_in;
                        cordic_initiate     <= 1'b1;
                    end
                end
                CONFIGURE: begin
                    cordic_initiate     <= 1'b0;
                    if (cordic_ready || angle == 90) begin
                        if (counter_elements == 0) begin        // Calculating L_kn for n = 0, k = 0
                            output_term_pos_n_reg[0]    <= 10'b01_00000000 + signed'(const_R_0_term);           // B_0 = 1 + 2*R_0*f_s/v_s
                            output_term_neg_n_reg[0]    <= 10'b01_00000000 + signed'(const_R_0_term);           // B_0 = 1 + 2*R_0*f_s/v_s
                            cordic_out                  <= cordic_result;                                       // 2*p*f_s/v_s * cos(angle)
                            cordic_ack                  <= 1'b1;
                            counter_elements            <= counter_elements + 1;
                        end
                    end
                    if(counter_elements > 0 && counter_elements <= 32) begin    // Calculating L_kn for all n, k = 0
                        output_term_pos_n_reg[counter_elements]   <= output_term_pos_n_reg[counter_elements-1] - cordic_out;       // (1 + 2*R_0*f_s/v_s + 2*1) - n(2*p*f_s/v_s * cos(angle))
                        output_term_neg_n_reg[counter_elements]   <= output_term_neg_n_reg[counter_elements-1] + cordic_out;       // (1 + 2*R_0*f_s/v_s + 2*1) + n(2*p*f_s/v_s * cos(angle))
                        if (counter_elements == 32) begin
                            ready_reg               <= 1'b1;
                            done_configuring_reg    <= 1'b1;
                        end
                        counter_elements    <= counter_elements + 1;
                    end
                end
                RUN_NEXT: begin // Calculating L_(k+1)n for all n
                    for (int i = 0; i <= 32; i = i + 1) begin
                        output_term_neg_n_reg[i] = output_term_neg_n_reg[i] + 11'b010_00000000;      // +2 for each point k
                        output_term_pos_n_reg[i] = output_term_pos_n_reg[i] + 11'b010_00000000;      // +2 for each point k
                    end
                    ready_reg   <= 1'b1;
                end
                WAIT: if (ack)  ready_reg   <= 1'b0;
            endcase
		end
    end


    CordicCosine # (
        .DW_ANGLE(DW_ANGLE),
        .DW_FRACTION(8),
        .DW_CALCULATION_TERMS(16)
    )   cordic_inst  (
        // Input
        .clk(clk),
        .rst(rst),
        .initiate(cordic_initiate),
        .angle(angle_reg),
        .x_scale(cordic_x_scale),
        .ack(cordic_ack),
        // Output
        .result(cordic_result),
        .ready(cordic_ready)
    );
endmodule