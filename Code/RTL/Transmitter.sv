module Transmitter #(
                    parameter DW_INTEGER                = 18,
                    parameter DW_INPUT                  = 8,
                    parameter DW_FRACTION               = 8,
                    parameter ANGLE_DW                  = 8,
                    parameter INC_TERM_DW_INTEGER       = 16,
                    parameter N_DW_INTEGER              = 13,
                    parameter ERROR_DW_INTEGER          = 14,
                    parameter A_DW_INTEGER              = 4,
                    parameter DW_FRACTION_INCANDCOMPARE = 4
                    )(
                    input                                       clk,                // Clock signal
                    input                                       rst,                // Reset signal
                    // Input values
                    input                                       initiate,           // Initiates calculation for next scan point in scanline
                    input [DW_INPUT-1:0]                        r_0,                // R_0 input
                    input [ANGLE_DW-1:0]                        angle,              // angle input
                    input [12:0]                                num_points,         // Number of points in scanline
                    // Output values
                    output [63:0]                               txArray,            // Transmit signal for each element
                    output                                      done                // Transmittion for scanline done signal
                    );

    // Delay values
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    delayArray[63:0];               // Array holding all delay values
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    delayMin;                       // Holding smallest delay value
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    delayMax;                       // Holding largest delay value
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_0_reg;                        // Holding value for element n=0
    logic [DW_INPUT-1:0]                                                r_0_reg;
    logic [DW_INPUT-1:0]                                                r_0_calc;
    logic [ANGLE_DW-1:0]                                                angle_reg;
    logic [ANGLE_DW-1:0]                                                angle_calc;

    // NextElementIncrementTermCalculator inputs and outputs
    logic signed [DW_FRACTION+DW_INTEGER-2:0]                           reference_output_term_pos_n;
    logic signed [DW_FRACTION+DW_INTEGER-2:0]                           reference_output_term_neg_n;

    //  ** IncrementAndCompare inputs and outputs **  //
    // Positive indexed elements
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_prev_pos_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_prev_pos_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_prev_sq_pos_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_pos_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_prev_pos_n;
    logic signed [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]         error_prev_pos_n;

    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_next_pos_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_next_pos_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_next_sq_pos_n;
    logic signed  [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     comp_term_next_pos_n;
    logic signed  [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]        error_next_pos_n;

    // Negative indexed elements
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_prev_neg_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_prev_neg_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_prev_sq_neg_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_neg_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_prev_neg_n;
    logic signed [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]         error_prev_neg_n;

    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_next_neg_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_next_neg_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_next_sq_neg_n;
    logic signed  [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     comp_term_next_neg_n;
    logic signed  [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]        error_next_neg_n;

    

    // ** Control signals ** //
    logic [5:0]     element_counter;
    logic [12:0]    point_counter;
    logic [12:0]    num_points_reg;
    logic           final_scanpoint;
    // reference calculator
    logic       reference_ack;
    logic       reference_calculation_done;
    logic       reference_ready;
    logic       initiate_calc;
    logic       initiate_calc_reg;
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

    assign final_scanpoint                  = (point_counter == num_points_reg) ? 1'b1 : 1'b0;
    assign done                             = transmit_done && final_scanpoint;

    assign initiate_calc                    = initiate || initiate_calc_reg;
    assign r_0_calc                         = (state == LOAD) ? r_0 : r_0_reg;
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
            //delayArray              <= '0;
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
            for (int i = 0; i < 64; i = i + 1) begin
                delayArray[i] = '0;
            end
        end
        else begin
            case(state)
                IDLE: begin     // RESET ALL VALUES
                    //delayArray              <= '0;
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
                    for (int i = 0; i < 64; i = i + 1) begin
                        delayArray[i] = '0;
                    end
                end
                LOAD: begin // CONST MULTIPLIER
                    n_prev_pos_n        <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);  // N_{0,0} = f_s/v_s * R_0 = 16.234375 * R_0
                    n_prev_neg_n        <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayArray[31]      <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayMin            <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    delayMax            <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    n_0_reg             <= (r_0 << 4+DW_FRACTION_INCANDCOMPARE) + (r_0 << DW_FRACTION_INCANDCOMPARE-2) - (r_0 << DW_FRACTION_INCANDCOMPARE-6);
                    r_0_reg             <= r_0;
                    angle_reg           <= angle;
                    num_points_reg      <= num_points;
                    element_counter     <= 5'd1;
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
                        delayArray[31+element_counter]  <= n_next_pos_n;
                        if (element_counter < 32)
                            delayArray[31-element_counter]  <= n_next_neg_n;

                        if (n_next_pos_n < delayMin || n_next_neg_n < delayMin)
                            if (n_next_pos_n < n_next_neg_n)
                                delayMin            <= n_next_pos_n;
                            else if (element_counter < 32)
                                delayMin            <= n_next_neg_n;
                        if (n_next_pos_n > delayMax || n_next_neg_n > delayMax)
                            if (n_next_pos_n > n_next_neg_n)
                                delayMax            <= n_next_pos_n;
                            else if (element_counter < 32)
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
                            element_counter                 <= 5'd0;
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
                    element_counter     <= 5'd1;
                    if(point_counter[5:0] == 5'b10000)
                        r_0_reg         <= r_0_reg + 1;
                end
                TRANSMIT: begin
                    initiate_transmit   <= 1'b0;
                    if (initiate_calc_reg == 0)
                        initiate_calc_reg   <= 1'b1;
                    else
                        initiate_calc_reg   <= 1'b0;
                end

            endcase
		end
    end




    // Instantiating the reference calculator module
    NextElementIncrementTermCalculator #(
        .DW_INTEGER(18),
		.DW_FRACTION(6),
        .ANGLE_DW(8),
        .DW_INPUT(8)
    ) calcElement (
        .clk(clk),
        .rst(rst),

        .initiate(initiate_calc),
        .ack(reference_ack),
        .r_0(r_0_calc),
        .angle(angle_calc),

        .output_term_pos_n(reference_output_term_pos_n),
        .output_term_neg_n(reference_output_term_neg_n),
        .last_element(reference_calculation_done),
        .ready(reference_ready)
    );

    // Instantiating the incrementAndCompare module
    IncrementAndCompare #(
        .STEP_SIZE(1),
        .INC_TERM_DW_INTEGER(16),
        .N_DW_INTEGER(13),
        .ERROR_DW_INTEGER(14),
        .A_DW_INTEGER(4),
        .DW_FRACTIONAL(DW_FRACTION_INCANDCOMPARE)
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
        .STEP_SIZE(1),
        .INC_TERM_DW_INTEGER(16),
        .N_DW_INTEGER(13),
        .ERROR_DW_INTEGER(14),
        .A_DW_INTEGER(4),
        .DW_FRACTIONAL(DW_FRACTION_INCANDCOMPARE)
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
        .N_DW_INTEGER(N_DW_INTEGER),
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