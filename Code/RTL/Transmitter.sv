module Transmitter #(
                    parameter DW_INPUT                  = 8,
                    parameter DW_ANGLE                  = 8,
                    parameter DW_SCANLINE_COUNT         = 13,
                    parameter NUM_ELEMENTS              = 64
                    )(
                    input                                       clk,                // Clock signal
                    input                                       rst,                // Reset signal
                    // Input values
                    input                                       initiate,           // Initiates calculation for next scan point in scanline
                    input [DW_INPUT-1:0]                        r_0,                // R_0 input
                    input [DW_ANGLE-1:0]                        angle,              // angle input
                    input [DW_SCANLINE_COUNT-1:0]               num_points,         // Number of points in scanline
                    // Output values
                    output [NUM_ELEMENTS-1:0]                   txArray,            // Transmit signal for each element
                    output                                      done                // Transmission for complete scanline done signal
                    );

    // Calculator parameters
    localparam DW_CALC_INTEGER              = 18;
    localparam DW_CALC_FRACTION             = 6;
    
    // IncrementAndCompare parameters
    localparam DW_INC_TERM_INTEGER          = 16;
    localparam DW_N_INTEGER                 = 13;
    localparam DW_ERROR_INTEGER             = 14;
    localparam DW_A_INTEGER                 = 4;
    localparam DW_FRACTION_INCANDCOMPARE    = 4;

    // General parameters
    localparam DW_ELEMENT_COUNTER           = 6;
    localparam DW_POINT_COUNTER             = 13;
    localparam HALF_ELEMENTS                = NUM_ELEMENTS/2;

    // Delay values
    logic [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]    delayArray  [NUM_ELEMENTS-1:0]; // Array holding all delay values
    logic [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]    delayMin;                       // Holding smallest delay value
    logic [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]    delayMax;                       // Holding largest delay value
    logic [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]    n_0_reg;                        // Holding value for element n=0
    logic [DW_INPUT+DW_FRACTION_INCANDCOMPARE-1:0]      r_0_reg;
    logic [DW_INPUT+DW_FRACTION_INCANDCOMPARE-1:0]      r_0_calc;
    logic [DW_ANGLE-1:0]                                angle_reg;
    logic [DW_ANGLE-1:0]                                angle_calc;

    // NextElementIncrementTermCalculator inputs and outputs
    logic signed [DW_CALC_FRACTION+DW_CALC_INTEGER-1:0]                     reference_output_term_pos_n;
    logic signed [DW_CALC_FRACTION+DW_CALC_INTEGER-1:0]                     reference_output_term_neg_n;

    //  ** IncrementAndCompare inputs and outputs **  //
    // Positive indexed elements
    logic           [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              n_prev_pos_n;
    logic signed    [DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              a_prev_pos_n;
    logic           [2*DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]            a_prev_sq_pos_n;
    logic signed    [DW_INC_TERM_INTEGER+DW_FRACTION_INCANDCOMPARE:0]       comp_term_pos_n;
    logic signed    [DW_INC_TERM_INTEGER+DW_FRACTION_INCANDCOMPARE:0]       comp_term_prev_pos_n;
    logic signed    [DW_ERROR_INTEGER+DW_FRACTION_INCANDCOMPARE:0]          error_prev_pos_n;

    logic           [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              n_next_pos_n;
    logic signed    [DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              a_next_pos_n;
    logic           [2*DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]            a_next_sq_pos_n;
    logic signed    [DW_INC_TERM_INTEGER+DW_FRACTION_INCANDCOMPARE:0]       comp_term_next_pos_n;
    logic signed    [DW_ERROR_INTEGER+DW_FRACTION_INCANDCOMPARE:0]          error_next_pos_n;

    // Negative indexed elements
    logic           [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              n_prev_neg_n;
    logic signed    [DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              a_prev_neg_n;
    logic           [2*DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]            a_prev_sq_neg_n;
    logic signed    [DW_INC_TERM_INTEGER+DW_FRACTION_INCANDCOMPARE:0]       comp_term_neg_n;
    logic signed    [DW_INC_TERM_INTEGER+DW_FRACTION_INCANDCOMPARE:0]       comp_term_prev_neg_n;
    logic signed    [DW_ERROR_INTEGER+DW_FRACTION_INCANDCOMPARE:0]          error_prev_neg_n;

    logic           [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              n_next_neg_n;
    logic signed    [DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]              a_next_neg_n;
    logic           [2*DW_A_INTEGER+DW_FRACTION_INCANDCOMPARE:0]            a_next_sq_neg_n;
    logic signed    [DW_INC_TERM_INTEGER+DW_FRACTION_INCANDCOMPARE:0]       comp_term_next_neg_n;
    logic signed    [DW_ERROR_INTEGER+DW_FRACTION_INCANDCOMPARE:0]          error_next_neg_n;

    

    // ** Control signals ** //
    logic [DW_ELEMENT_COUNTER-1:0]  element_counter;
    logic [DW_POINT_COUNTER-1:0]    point_counter;
    logic                           point_counter_prev_index_7;
    logic [DW_POINT_COUNTER-1:0]    num_points_reg;
    logic                           final_scanpoint;
    // reference calculator
    logic       reference_ack;
    logic       reference_calculation_done;
    logic       reference_ready;
    logic       initiate_calc;
    logic       initiate_calc_reg;
    logic       rst_refCalculator;
    // increment and compare
    logic       initiate_incrementAndCompare;
    logic       ack_incrementAndCompare;

    logic       incrementAndCompare_available;
    logic       incrementAndCompare_available_pos_n;
    logic       incrementAndCompare_available_neg_n;

    logic       delay_ready;
    logic       delay_ready_neg_n;
    logic       delay_ready_pos_n;

    // transmit
    logic       initiate_transmit;
    logic       transmit_done;

    enum {LOAD, NEXT_ELEMENT_CALC, LOAD_NEXT, TRANSMIT, IDLE} state, nextState;

    assign delay_ready                      = delay_ready_neg_n && delay_ready_pos_n;
    assign incrementAndCompare_available    = incrementAndCompare_available_neg_n && incrementAndCompare_available_pos_n;

    assign final_scanpoint                  = (point_counter == num_points_reg && state == TRANSMIT) ? 1'b1 : 1'b0;
    assign done                             = transmit_done && final_scanpoint;
    assign rst_refCalculator                = rst || done;

    assign initiate_calc                    = (state == IDLE && initiate) || initiate_calc_reg;
    assign r_0_calc                         = (state == LOAD) ? {r_0,4'd0} : r_0_reg;
    assign angle_calc                       = (state == LOAD) ? angle : angle_reg;

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
                IDLE:                   if (initiate)                           nextState = LOAD;
                LOAD:                                                           nextState = NEXT_ELEMENT_CALC;
                NEXT_ELEMENT_CALC:      if (initiate_transmit)                  nextState = TRANSMIT;
                LOAD_NEXT:                                                      nextState = NEXT_ELEMENT_CALC;
                TRANSMIT:               if (transmit_done && final_scanpoint)   nextState = IDLE;
                                        else if (transmit_done)                 nextState = LOAD_NEXT;
            endcase
        end
    end

    // System functionality
    always @(posedge clk) begin
        if (rst) begin
            delayMin                        <= '0;
            delayMax                        <= '0;
            reference_ack                   <= '0;
            n_prev_pos_n                    <= '0;
            a_prev_pos_n                    <= '0;
            a_prev_sq_pos_n                 <= '0;
            comp_term_pos_n                 <= '0;
            comp_term_prev_pos_n            <= '0;
            error_prev_pos_n                <= '0;
            n_prev_neg_n                    <= '0;
            a_prev_neg_n                    <= '0;
            a_prev_sq_neg_n                 <= '0;
            comp_term_neg_n                 <= '0;
            comp_term_prev_neg_n            <= '0;
            error_prev_neg_n                <= '0;
            element_counter                 <= '0;
            point_counter                   <= '0;
            initiate_transmit               <= '0;
            ack_incrementAndCompare         <= '0;
            initiate_incrementAndCompare    <= '0;
            initiate_calc_reg               <= '0;
            angle_reg                       <= '0;
            r_0_reg                         <= '0;
            num_points_reg                  <= '0;
            n_0_reg                         <= '0;
            point_counter_prev_index_7      <= '0;
            for (int i = 0; i < NUM_ELEMENTS; i = i + 1) begin
                delayArray[i] = '0;
            end
        end
        else begin
            case(state)
                IDLE: begin     // RESET ALL VALUES
                    delayMin                        <= '0;
                    delayMax                        <= '0;
                    reference_ack                   <= '0;
                    n_prev_pos_n                    <= '0;
                    a_prev_pos_n                    <= '0;
                    a_prev_sq_pos_n                 <= '0;
                    comp_term_pos_n                 <= '0;
                    comp_term_prev_pos_n            <= '0;
                    error_prev_pos_n                <= '0;
                    n_prev_neg_n                    <= '0;
                    a_prev_neg_n                    <= '0;
                    a_prev_sq_neg_n                 <= '0;
                    comp_term_neg_n                 <= '0;
                    comp_term_prev_neg_n            <= '0;
                    error_prev_neg_n                <= '0;
                    element_counter                 <= '0;
                    point_counter                   <= '0;
                    initiate_transmit               <= '0;
                    ack_incrementAndCompare         <= '0;
                    initiate_incrementAndCompare    <= '0;
                    initiate_calc_reg               <= '0;
                    angle_reg                       <= '0;
                    r_0_reg                         <= '0;
                    num_points_reg                  <= '0;
                    n_0_reg                         <= '0;
                    point_counter_prev_index_7      <= 1'b1;
                    for (int i = 0; i < NUM_ELEMENTS; i = i + 1) begin
                        delayArray[i] = '0;
                    end
                end
                LOAD: begin // CONST MULTIPLIER
                    n_prev_pos_n                <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);  // N_{0,0} = f_s/v_s * R_0 = 16.234375 * R_0
                    n_prev_neg_n                <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayArray[HALF_ELEMENTS-1] <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayMin                    <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayMax                    <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    n_0_reg                     <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    r_0_reg                     <= {r_0,4'd0};
                    angle_reg                   <= angle;
                    num_points_reg              <= num_points;
                    element_counter             <= 1;
                end
                NEXT_ELEMENT_CALC: begin
                    initiate_transmit                   <= 1'b0;
                    // Synchronizing comparator term calculator and IncrementAndCompare module
                    if (reference_ready && incrementAndCompare_available && initiate_incrementAndCompare == 1'b0) begin  // K_n ready
                        comp_term_pos_n                 <= reference_output_term_pos_n >>> 2;        // Compensating for DW in fraction
                        comp_term_neg_n                 <= reference_output_term_neg_n >>> 2;
                        initiate_incrementAndCompare    <= 1'b1;
                        reference_ack                   <= 1'b1;
                    end
                    else begin
                        reference_ack                   <= 1'b0;
                        initiate_incrementAndCompare    <= 1'b0;
                    end

                    // When IncrementAndCompare is done
                    if (delay_ready && ack_incrementAndCompare == 1'b0) begin
                        // Storing delay value
                        delayArray[HALF_ELEMENTS-1+element_counter]  <= n_next_pos_n;
                        if (element_counter < HALF_ELEMENTS)
                            delayArray[HALF_ELEMENTS-1-element_counter]  <= n_next_neg_n;

                        if (n_next_pos_n < delayMin || n_next_neg_n < delayMin)
                            if (n_next_pos_n < n_next_neg_n)
                                delayMin            <= n_next_pos_n;
                            else if (element_counter < HALF_ELEMENTS)
                                delayMin            <= n_next_neg_n;
                        if (n_next_pos_n > delayMax || n_next_neg_n > delayMax)
                            if (n_next_pos_n > n_next_neg_n)
                                delayMax            <= n_next_pos_n;
                            else if (element_counter < HALF_ELEMENTS)
                                delayMax            <= n_next_neg_n;
                        // Feedback to IncrementAndCompare
                        n_prev_pos_n                  <= n_next_pos_n;
                        a_prev_pos_n                  <= a_next_pos_n;
                        a_prev_sq_pos_n               <= a_next_sq_pos_n;
                        comp_term_prev_pos_n          <= comp_term_next_pos_n;
                        error_prev_pos_n              <= error_next_pos_n;

                        n_prev_neg_n                  <= n_next_neg_n;
                        a_prev_neg_n                  <= a_next_neg_n;
                        a_prev_sq_neg_n               <= a_next_sq_neg_n;
                        comp_term_prev_neg_n          <= comp_term_next_neg_n;
                        error_prev_neg_n              <= error_next_neg_n;
                        // Updating control signals
                        ack_incrementAndCompare <= 1'b1;
                        reference_ack           <= 1'b1;
                        if (reference_calculation_done == 1'b1) begin
                            initiate_transmit               <= 1'b1;
                            element_counter                 <= '0;
                            point_counter                   <= point_counter + 1;
                            n_prev_pos_n                    <= '0;
                            a_prev_pos_n                    <= '0;
                            a_prev_sq_pos_n                 <= '0;
                            comp_term_prev_pos_n            <= '0;
                            error_prev_pos_n                <= '0;
                            n_prev_neg_n                    <= '0;
                            a_prev_neg_n                    <= '0;
                            a_prev_sq_neg_n                 <= '0;
                            comp_term_prev_neg_n            <= '0;
                            error_prev_neg_n                <= '0;
                        end
                        else begin
                            element_counter     <= element_counter + 1;
                        end
                    end
                    else begin
                        ack_incrementAndCompare         <= 1'b0;
                        reference_ack                   <= 1'b0;
                    end
                end
                LOAD_NEXT: begin
                    n_prev_pos_n        <= n_0_reg + signed'(6'b010000);
                    n_prev_neg_n        <= n_0_reg + signed'(6'b010000);
                    delayArray[31]      <= n_0_reg + signed'(6'b010000);
                    delayMin            <= n_0_reg + signed'(6'b010000);
                    delayMax            <= n_0_reg + signed'(6'b010000);
                    n_0_reg             <= n_0_reg + signed'(6'b010000);
                    element_counter     <= 1;
                    initiate_calc_reg   <= 1'b0;
                    if (point_counter[7] == point_counter_prev_index_7 && point_counter[3:1] == 3'b101) begin // Correction of quantized error (2^6+2^2+2 = 69 approx 69.44)
                        point_counter_prev_index_7      <= ~point_counter[7];
                        r_0_reg                         <= r_0_reg - 5'b0_0001;
                    end
                    else
                        r_0_reg                         <= r_0_reg + 5'b0_0001;
                end
                TRANSMIT: begin
                    initiate_transmit   <= 1'b0;
                    if (transmit_done && ~final_scanpoint)
                        initiate_calc_reg   <= 1'b1;
                end

            endcase
		end
    end




    // Instantiating the reference calculator module
    NextElementIncrementTermCalculator #(
        .DW_INTEGER(DW_CALC_INTEGER),
		.DW_FRACTION(DW_CALC_FRACTION),
        .DW_ANGLE(DW_ANGLE),
        .DW_INPUT(DW_INPUT)
    ) calcElement (
        .clk(clk),
        .rst(rst_refCalculator),
        // Input
        .initiate(initiate_calc),
        .ack(reference_ack),
        .r_0(r_0_calc),
        .angle(angle_calc),
        // Output
        .output_term_pos_n(reference_output_term_pos_n),
        .output_term_neg_n(reference_output_term_neg_n),
        .last_element(reference_calculation_done),
        .ready(reference_ready)
    );

    // Instantiating the incrementAndCompare module
    IncrementAndCompare #(
        .DW_INC_TERM_INTEGER(DW_INC_TERM_INTEGER),
        .DW_N_INTEGER(DW_N_INTEGER),
        .DW_ERROR_INTEGER(DW_ERROR_INTEGER),
        .DW_A_INTEGER(DW_A_INTEGER),
        .DW_FRACTION(DW_FRACTION_INCANDCOMPARE)
    ) incAndComp_pos_n (
        .clk(clk),
        .rst(rst),
        // Input
        .initiate(initiate_incrementAndCompare),
        .ack(ack_incrementAndCompare),
        .n_prev(n_prev_pos_n),
        .a_prev(a_prev_pos_n),
        .a_prev_sq(a_prev_sq_pos_n),
        .comp_term(comp_term_pos_n),
        .comp_term_prev(comp_term_prev_pos_n),
        .error_prev(error_prev_pos_n),
        // Output
        .n_next(n_next_pos_n),
        .error_next(error_next_pos_n),
        .a_next(a_next_pos_n),
        .a_next_sq(a_next_sq_pos_n),
        .comp_term_next(comp_term_next_pos_n),
        .ready(delay_ready_pos_n),
        .available(incrementAndCompare_available_pos_n)
    );

        // Instantiating the incrementAndCompare module
    IncrementAndCompare #(
        .DW_INC_TERM_INTEGER(DW_INC_TERM_INTEGER),
        .DW_N_INTEGER(DW_N_INTEGER),
        .DW_ERROR_INTEGER(DW_ERROR_INTEGER),
        .DW_A_INTEGER(DW_A_INTEGER),
        .DW_FRACTION(DW_FRACTION_INCANDCOMPARE)
    ) incAndComp_neg_n (
        .clk(clk),
        .rst(rst),
        // Input
        .initiate(initiate_incrementAndCompare),
        .ack(ack_incrementAndCompare),
        .n_prev(n_prev_neg_n),
        .a_prev(a_prev_neg_n),
        .a_prev_sq(a_prev_sq_neg_n),
        .comp_term(comp_term_neg_n),
        .comp_term_prev(comp_term_prev_neg_n),
        .error_prev(error_prev_neg_n),
        // Output
        .n_next(n_next_neg_n),
        .error_next(error_next_neg_n),
        .a_next(a_next_neg_n),
        .a_next_sq(a_next_sq_neg_n),
        .comp_term_next(comp_term_next_neg_n),
        .ready(delay_ready_neg_n),
        .available(incrementAndCompare_available_neg_n)
    );

    CountdownTransmit  #(
        .DW_N_INTEGER(DW_N_INTEGER),
        .DW_FRACTION_INCANDCOMPARE(DW_FRACTION_INCANDCOMPARE)                        
    ) inst_countdown(
        .clk(clk),
        .rst(rst),
        // Input values
        .initiate(initiate_transmit),
        .delayArray(delayArray),
        .minValue(delayMin),
        .maxValue(delayMax),
        // Output values
        .txArray(txArray),
        .done(transmit_done)
    );

endmodule