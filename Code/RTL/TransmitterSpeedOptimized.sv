module TransmitterSpeedOptimized #(
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
    localparam DW_CALC_FRACTION             = 7;
    
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

    // Delay value signals/regs
    logic [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                delayArray  [NUM_ELEMENTS-1:0]; // Array holding all delay values
    logic signed [DW_ERROR_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     errorArray  [NUM_ELEMENTS-1:0]; // Error outputs
    logic [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                delayMin;                       // Holding smallest delay value
    logic [DW_ELEMENT_COUNTER:0]                                    delayMinIndex;
    logic [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                delayMax;                       // Holding largest delay value
    logic [DW_ELEMENT_COUNTER:0]                                    delayMaxIndex;

    // NextElementIncrementTermCalculator inputs and outputs
    logic       reference_calculation_done;
    logic       reference_ack;
    logic       reference_ready;
    logic       initiate_calc;
    logic       rst_refCalculator;
    logic       incrementAndCompare_available;
    logic       incrementAndCompare_available_pos_n;
    logic       incrementAndCompare_available_neg_n;
    logic       delay_ready;
    logic       delay_ready_neg_n;
    logic       delay_ready_pos_n;
    logic        [DW_INPUT+DW_FRACTION_INCANDCOMPARE-1:0]                   r_0_calc;
    logic        [DW_ANGLE-1:0]                                             angle_calc;
    logic signed [DW_CALC_FRACTION+DW_CALC_INTEGER-1:0]                     reference_output_term_pos_n;
    logic signed [DW_CALC_FRACTION+DW_CALC_INTEGER-1:0]                     reference_output_term_neg_n;

    // IncrementAndCompare next element inputs and outputs
    logic       initiate_incrementAndCompare;
    logic       ack_incrementAndCompare;
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


    // Array inputs/outputs
    logic       rst_array;
    logic       ack_array;
    logic       array_ready;
    logic       done_configuring_array;
    logic        [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0]         delay_array_calc        [NUM_ELEMENTS-1:0]; // Output delays for all elements
    logic signed [DW_ERROR_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     error_array_calc        [NUM_ELEMENTS-1:0]; // Error outputs

    // FSM signals
    enum {LOAD, NEXT_ELEMENT_CALC, LOAD_NEXT, TRANSMIT, IDLE} state, nextState;
    logic                           final_scanpoint;
    logic [DW_ELEMENT_COUNTER-1:0]  element_counter;
    logic [DW_POINT_COUNTER-1:0]    point_counter;
    logic [DW_POINT_COUNTER-1:0]    num_points_reg;

    // Transmit signals
    logic       initiate_transmit;
    logic       transmit_done;


    assign rst_refCalculator                = rst || done;                                                                  // Resets referece calculator when scanline completed
    assign initiate_calc                    = (state == IDLE && initiate);                                                  // Initiates calculator to start scanline
    assign incrementAndCompare_available    = incrementAndCompare_available_neg_n && incrementAndCompare_available_pos_n;   // Indicating incrementAndCompare available for pos and neg index
    assign delay_ready                      = delay_ready_neg_n && delay_ready_pos_n;                                       // Indicating both pos and neg indexed delay values ready
    
    assign angle_calc                       = (state == LOAD) ? angle : '0;                                                 // Input to reference calculator
    assign r_0_calc                         = (state == LOAD) ? {r_0,4'd0} : '0;                                            // Input to reference calculator

    assign rst_array                        = rst || done;                                                                  // Resets array calculation when scanline completed

    assign final_scanpoint                  = (point_counter == num_points_reg && state == TRANSMIT) ? 1'b1 : 1'b0;         // Indicating final point of scanline
    assign done                             = transmit_done && final_scanpoint;                                             // Indicates when a scanline has completed

    // Locking next state
    always_ff @(posedge clk) begin
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
                IDLE:                   if (initiate)                                       nextState = LOAD;
                LOAD:                                                                       nextState = NEXT_ELEMENT_CALC;
                NEXT_ELEMENT_CALC:      if (initiate_transmit && done_configuring_array)    nextState = TRANSMIT;
                LOAD_NEXT:              if (initiate_transmit)                              nextState = TRANSMIT;
                TRANSMIT:               if (transmit_done && final_scanpoint)               nextState = IDLE;
                                        else if (transmit_done)                             nextState = LOAD_NEXT;
            endcase
        end
    end

    // System functionality
    always_ff @(posedge clk) begin
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
            num_points_reg                  <= '0;
            ack_array                       <= '0;
            delayMinIndex                   <= '0;
            delayMaxIndex                   <= '0;
            for (int i = 0; i < NUM_ELEMENTS; i = i + 1) begin
                delayArray[i] <= '0;
                errorArray[i] <= '0;
            end
        end
        else begin
            case(state)
                IDLE: begin
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
                    num_points_reg                  <= '0;
                    ack_array                       <= '0;
                    delayMinIndex                   <= '0;
                    delayMaxIndex                   <= '0;
                    for (int i = 0; i < NUM_ELEMENTS; i = i + 1) begin
                        delayArray[i] <= '0;
                        errorArray[i] <= '0;
                    end
                end
                LOAD: begin // CONST MULTIPLIER
                    n_prev_pos_n                <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);  // N_{0,0} = f_s/v_s * R_0 = 16.234375 * R_0
                    n_prev_neg_n                <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayArray[HALF_ELEMENTS-1] <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayMin                    <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayMax                    <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayMinIndex               <= HALF_ELEMENTS-1;
                    delayMaxIndex               <= HALF_ELEMENTS-1;
                    num_points_reg              <= num_points;
                    element_counter             <= 1;
                end
                NEXT_ELEMENT_CALC: begin
                    initiate_transmit                   <= 1'b0;
                    // Synchronizing comparator term calculator and IncrementAndCompare module
                    if (reference_ready && incrementAndCompare_available && initiate_incrementAndCompare == 1'b0) begin  // K_n ready
                        comp_term_pos_n                 <= reference_output_term_pos_n >>> (DW_CALC_FRACTION - DW_FRACTION_INCANDCOMPARE);        // Compensating for DW in fraction
                        comp_term_neg_n                 <= reference_output_term_neg_n >>> (DW_CALC_FRACTION - DW_FRACTION_INCANDCOMPARE);
                        initiate_incrementAndCompare    <= 1'b1;
                        reference_ack                   <= 1'b1;
                    end
                    else begin
                        reference_ack                   <= 1'b0;
                        initiate_incrementAndCompare    <= 1'b0;
                    end

                    // When IncrementAndCompare is done, store delay pos and neg indexed delay values
                    if (delay_ready && ack_incrementAndCompare == 1'b0) begin
                        // Storing delay value
                        delayArray[HALF_ELEMENTS-1+element_counter]  <= n_next_pos_n;
                        errorArray[HALF_ELEMENTS-1+element_counter]  <= error_next_pos_n;
                        if (element_counter < HALF_ELEMENTS) begin
                            delayArray[HALF_ELEMENTS-1-element_counter]  <= n_next_neg_n;
                            errorArray[HALF_ELEMENTS-1-element_counter]  <= error_next_neg_n;
                        end

                        if (n_next_pos_n < delayMin || n_next_neg_n < delayMin)
                            if (n_next_pos_n < n_next_neg_n) begin
                                delayMin            <= n_next_pos_n;
                                delayMinIndex       <= HALF_ELEMENTS - 1 + element_counter;
                            end
                            else if (element_counter < HALF_ELEMENTS) begin
                                delayMin            <= n_next_neg_n;
                                delayMinIndex       <= HALF_ELEMENTS - 1 - signed'(element_counter);
                            end

                        if (n_next_pos_n > delayMax || n_next_neg_n > delayMax)
                            if (n_next_pos_n > n_next_neg_n) begin
                                delayMax            <= n_next_pos_n;
                                delayMaxIndex       <= HALF_ELEMENTS - 1 + element_counter;
                            end
                            else if (element_counter < HALF_ELEMENTS) begin
                                delayMax            <= n_next_neg_n;
                                delayMaxIndex       <= HALF_ELEMENTS - 1 - signed'(element_counter);
                            end

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
                    ack_array           <= 1'b0;
                    if (array_ready) begin
                        delayArray          <= delay_array_calc;
                        errorArray          <= error_array_calc;

                        // Logic for storing max and min delay values
                        case(delayMinIndex)
                            NUM_ELEMENTS-1: begin
                                if (delay_array_calc[delayMinIndex] <= delay_array_calc[delayMinIndex-1]) begin
                                    delayMin        <= delay_array_calc[delayMinIndex];
                                end
                                else begin
                                    delayMin        <= delay_array_calc[delayMinIndex-1];
                                    delayMinIndex   <= delayMinIndex - 1;
                                end
                            end
                            0: begin
                                if (delay_array_calc[delayMinIndex] <= delay_array_calc[delayMinIndex+1]) begin
                                    delayMin        <= delay_array_calc[delayMinIndex];
                                end
                                else begin
                                    delayMin        <= delay_array_calc[delayMinIndex+1];
                                    delayMinIndex   <= delayMinIndex + 1;
                                end
                            end
                            default: begin
                                if (delay_array_calc[delayMinIndex] <= delay_array_calc[delayMinIndex+1] && delay_array_calc[delayMinIndex] <= delay_array_calc[delayMinIndex-1] ) begin
                                    delayMin        <= delay_array_calc[delayMinIndex];
                                end
                                else if (delay_array_calc[delayMinIndex+1] <= delay_array_calc[delayMinIndex-1]) begin
                                    delayMin        <= delay_array_calc[delayMinIndex+1];
                                    delayMinIndex   <= delayMinIndex + 1;
                                end
                                else begin
                                    delayMin        <= delay_array_calc[delayMinIndex-1];
                                    delayMinIndex   <= delayMinIndex - 1;
                                end
                            end
                        endcase

                        case(delayMaxIndex)
                            NUM_ELEMENTS-1: begin
                                if (delay_array_calc[delayMaxIndex] >= delay_array_calc[delayMaxIndex-1]) begin
                                    delayMax        <= delay_array_calc[delayMaxIndex];
                                end
                                else begin
                                    delayMax        <= delay_array_calc[delayMaxIndex-1];
                                    delayMaxIndex   <= delayMaxIndex - 1;
                                end
                            end
                            0: begin
                                if (delay_array_calc[delayMaxIndex] >= delay_array_calc[delayMaxIndex+1]) begin
                                    delayMax        <= delay_array_calc[delayMaxIndex];
                                end
                                else begin
                                    delayMax        <= delay_array_calc[delayMaxIndex+1];
                                    delayMaxIndex   <= delayMaxIndex + 1;
                                end
                            end
                            default: begin
                                if (delay_array_calc[delayMaxIndex] >= delay_array_calc[delayMaxIndex+1] && delay_array_calc[delayMaxIndex] > delay_array_calc[delayMaxIndex-1] ) begin
                                    delayMax        <= delay_array_calc[delayMaxIndex];
                                end
                                else if (delay_array_calc[delayMaxIndex+1] >= delay_array_calc[delayMaxIndex-1]) begin
                                    delayMax        <= delay_array_calc[delayMaxIndex+1];
                                    delayMaxIndex   <= delayMaxIndex + 1;
                                end
                                else begin
                                    delayMax        <= delay_array_calc[delayMaxIndex-1];
                                    delayMaxIndex   <= delayMaxIndex - 1;
                                end
                            end
                        endcase

                        initiate_transmit   <= 1'b1;
                        point_counter       <= point_counter + 1;
                    end
                end
                TRANSMIT: begin
                    initiate_transmit   <= 1'b0;
                    if (transmit_done && ~final_scanpoint) begin
                        ack_array           <= 1'b1;
                    end
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

    // Instantiating the incrementAndCompare modules for reference point
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

    // Instantiating array module computing delays for points k > 0
    IncrementAndCompareArray  #(
        .DW_INC_TERM_INTEGER(DW_INC_TERM_INTEGER),
        .DW_N_INTEGER(DW_N_INTEGER),
        .DW_ERROR_INTEGER(DW_ERROR_INTEGER),
        .DW_A_INTEGER(DW_A_INTEGER),
        .DW_FRACTION(DW_FRACTION_INCANDCOMPARE),
        .NUM_ELEMENTS(NUM_ELEMENTS)
    ) inst_array (
        .clk(clk),
        .rst(rst_array),
        // Input values
        .r_0(r_0),
        .angle(angle),
        .configure(initiate),
        .transmit_done(transmit_done),
        .n_prev(delayArray),
        .error_prev(errorArray),
        .ack(ack_array),
        .final_scanpoint(final_scanpoint),
        // Output values
        .n_out(delay_array_calc),
        .error_out(error_array_calc),
        .ready(array_ready),
        .done_configuring(done_configuring_array)
    );

    // Instantiation module generating transmit enable outputs
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