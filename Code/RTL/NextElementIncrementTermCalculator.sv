/* Module calculating the term K_n = A_0(2n+1) +/- C_0 for all n elements*/
module NextElementIncrementTermCalculator  #(
                            parameter DW_INTEGER    = 18,
                            parameter DW_INPUT      = 8,
                            parameter NUM_ELEMENTS  = 64,
                            parameter DW_FRACTION   = 6,
                            parameter DW_ANGLE      = 8
                            )(
                            input                                                   clk,                // Clock signal
                            input                                                   rst,                // Reset signal
                            // Input values
                            input                                                   initiate,           // Initiates a new delay caclulation for all n, locks input values
                            input                                                   ack,                // Acknowledges output has been read
                            input [DW_INPUT+4-1:0]                                  r_0,                // R_0 input
                            input [DW_ANGLE-1:0]                                    angle,              // angle input
                            // Output values
                            output signed   [DW_INTEGER+DW_FRACTION-1:0]            output_term_pos_n,  // Output comparator term (K_n)
                            output signed   [DW_INTEGER+DW_FRACTION-1:0]            output_term_neg_n,  // Output comparator term (K_n)
                            output                                                  last_element,
                            output                                                  ready               // Result ready signal
                            );

    localparam HALF_ELEMENTS                        = NUM_ELEMENTS/2;
    localparam DW_CONSTANT                          = 6;
    localparam [DW_CONSTANT+DW_FRACTION-1:0] a_0    = 'b010000_011110; // Quantized value A_0 = 16.46875
    localparam DW_COUNTER                           = 7;

    // State machine variables
    enum {LOAD, RUN1, WAIT, RUN2, IDLE} state, nextState;

    // Registers holding results
    logic [DW_ANGLE-1:0]                                        angle_reg;
    logic signed [DW_FRACTION+DW_INTEGER-1:0]                   output_term_pos_n_reg;
    logic signed [DW_FRACTION+DW_INTEGER-1:0]                   output_term_neg_n_reg;

    // Control signals
    logic [DW_COUNTER-1:0]                                      counter;
    logic                                                       last_element_reg;
    logic                                                       ready_reg;

    // Cordic signals
    logic signed [DW_INTEGER+DW_FRACTION-1:0]                   cordic_result;
    logic [DW_FRACTION+DW_INTEGER-1:0]                          cordic_x_scale;
    logic                                                       cordic_initiate;
    logic                                                       cordic_ack;
    logic                                                       cordic_ready;

    // Assigning outputs
    assign ready                = (state == WAIT) ? ready_reg : '0;
    assign output_term_pos_n    = (state == WAIT) ? output_term_pos_n_reg : '0;
    assign output_term_neg_n    = (state == WAIT) ? output_term_neg_n_reg : '0;
    assign last_element         = last_element_reg;

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
                    if (ack && last_element_reg)nextState = IDLE;
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
            last_element_reg        <= 1'b0;
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
                    last_element_reg        <= 1'b0;
                    ready_reg               <= 1'b0;
                    cordic_ack              <= 1'b0;
                end
                LOAD: begin // CONST MULTIPLIER
                    // NOTE: -4 because of input fraction in area optimized solution
                    cordic_x_scale      <= (r_0 << 7+(DW_FRACTION-4))           // (2*p*(f_s/v_s)^2) * 10^-3 = 131.767
                                            + (r_0 << 1+(DW_FRACTION-4)) 
                                            + (r_0 << (DW_FRACTION-4)) 
                                            + (r_0 << (DW_FRACTION-4)-1) 
                                            + (r_0 << (DW_FRACTION-4)-2) 
                                            + (r_0 >> 6-(DW_FRACTION-4)) 
                                            + (r_0 >>> 9-(DW_FRACTION-4));     
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
                        counter                 <= {DW_COUNTER{'0}};
                    end
                end
                WAIT: if(ack) begin
                        ready_reg   <= 1'b0;
                    end
                RUN2: begin // INCREMENT K_N
                    output_term_pos_n_reg   <= output_term_pos_n_reg + (a_0 << 1);    // A_0 * (2*n + 1) - C_0
                    output_term_neg_n_reg   <= output_term_neg_n_reg + (a_0 << 1);    // A_0 * (2*n + 1) + C_0
                    ready_reg               <= 1'b1;
                    if (counter == HALF_ELEMENTS-2)
                        last_element_reg    <= 1'b1;
                    else
                        counter         <= counter + 1;
                end
            endcase
		end
    end


    CordicCosine # (
        .DW_ANGLE(DW_ANGLE),
        .DW_FRACTION(DW_FRACTION),
        .DW_CALCULATION_TERMS(DW_INTEGER)
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