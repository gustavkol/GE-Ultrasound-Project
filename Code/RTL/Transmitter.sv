module Transmitter #(
                    parameter DW_INTEGER    = 18,
                    parameter DW_INPUT      = 8,
                    parameter DW_FRACTION   = 8,
                    parameter ANGLE_DW      = 8,
                    parameter INC_TERM_DW_INTEGER = 16,
                    parameter N_DW_INTEGER = 13,
                    parameter ERROR_DW_INTEGER = 14,
                    parameter A_DW_INTEGER = 4,
                    parameter DW_FRACTION_INCANDCOMPARE = 4
                    )(
                    input                                       clk,                // Clock signal
                    input                                       rst,                // Reset signal
                    // Input values
                    input                                       initiate,           // Initiates calculation for next scan point in scanline
                    input [DW_INPUT-1:0]                        r_0,                // R_0 input
                    input [ANGLE_DW-1:0]                        angle,              // angle input
                    // Output values
                    output [63:0]                               txArray,            // Transmit signal for each element
                    output                                      done                // Transmittion for scanline done signal

                    // TODO: Add max k/scanline length
                    );

    // Value signals
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     delayArray[63:0];               // Array holding all delay values
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     delayMin;                       // Holding smallest delay value
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     delayMax;                       // Holding largest delay value

    // NextElementIncrementTermCalculator inputs and outputs
    logic                                       reference_ack;

    logic signed [DW_FRACTION+DW_INTEGER-2:0]     reference_output_term_pos_n;
    logic signed [DW_FRACTION+DW_INTEGER-2:0]     reference_output_term_neg_n;

    // NextPointIncrementTermCalculator inputs and outputs
    logic                                       configure_scanline;
    logic [DW_INPUT-1:0]                        r_0_reg;
    logic [ANGLE_DW-1:0]                        angle_reg;
    logic                                       ack_element;

    logic signed [DW_FRACTION+DW_INTEGER:0]     scanline_output_term_pos_n;
    logic signed [DW_FRACTION+DW_INTEGER:0]     scanline_output_term_neg_n;

    // IncrementAndCompare inputs and outputs
    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_prev_pos_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_prev_pos_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_prev_sq_pos_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_pos_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_prev_pos_n;
    logic signed [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]         error_prev_pos_n;

    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_prev_neg_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_prev_neg_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_prev_sq_neg_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_neg_n;
    logic signed [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]      comp_term_prev_neg_n;
    logic signed [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]         error_prev_neg_n;

    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_next_pos_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_next_pos_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_next_sq_pos_n;
    logic signed  [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     comp_term_next_pos_n;
    logic signed  [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]        error_next_pos_n;

    logic [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                    n_next_neg_n;
    logic signed [A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]             a_next_neg_n;
    logic [2*A_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]                  a_next_sq_neg_n;
    logic signed  [INC_TERM_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]     comp_term_next_neg_n;
    logic signed  [ERROR_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0]        error_next_neg_n;
    

    // ** Control signals ** //
    logic [5:0]     element_counter;
    logic [12:0]    point_counter;
    logic           final_scanpoint;
    // reference calculator
    logic       reference_calculation_done;
    logic       reference_ready;
    // scanline point calculator
    logic       scanline_done;
    logic       element_ready;
    // increment and compare
    logic       initiate_incrementAndCompare;
    logic       ack_incrementAndCompare;

    logic       incrementAndCompare_available;
    logic       incrementAndCompare_available_pos_n;
    logic       incrementAndCompare_available_neg_n;

    logic       delay_ready;
    logic       delay_ready_neg_n;
    logic       delay_ready_pos_n;

    assign delay_ready                      = delay_ready_neg_n && delay_ready_pos_n;
    assign incrementAndCompare_available    = incrementAndCompare_available_neg_n && incrementAndCompare_available_pos_n;
    // transmit
    logic       initiate_transmit;
    logic       transmit_done;

    enum {LOAD, REFERENCE_POINT_CALC, SCANLINE_CALC, TRANSMIT, WAIT, IDLE} state, nextState;

    // TODO: Assign output
    assign done     = 0;

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
                LOAD:                                                           nextState = REFERENCE_POINT_CALC;
                REFERENCE_POINT_CALC:   if (initiate_transmit)                  nextState = TRANSMIT;
                SCANLINE_CALC:          if (initiate_transmit)                  nextState = TRANSMIT;
                TRANSMIT:               if (transmit_done && scanline_done)     nextState = IDLE;
                                        else if (transmit_done)                 nextState = SCANLINE_CALC;
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
            configure_scanline              <= '0;
            r_0_reg                         <= '0;
            angle_reg                       <= '0;
            ack_element                     <= '0;
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
            final_scanpoint                 <= '0;
            ack_incrementAndCompare         <= '0;
            initiate_incrementAndCompare    <= '0;
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
                    configure_scanline              <= '0;
                    r_0_reg                         <= '0;
                    angle_reg                       <= '0;
                    ack_element                     <= '0;
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
                    final_scanpoint                 <= '0;
                    ack_incrementAndCompare         <= '0;
                    initiate_incrementAndCompare    <= '0;
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
                    r_0_reg             <= r_0;
                    angle_reg           <= angle;
                    element_counter     <= 5'd1;
                    configure_scanline  <= 1'b1;
                end
                REFERENCE_POINT_CALC: begin
                    initiate_transmit               <= 1'b0;
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
                        if (reference_calculation_done == 1'b1) begin
                            initiate_transmit               <= 1'b1;
                            element_counter                 <= 5'd0;
                            point_counter                   <= 13'd1;
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
                    end
                end
                SCANLINE_CALC: begin
                    if(element_ready && incrementAndCompare_available && initiate_incrementAndCompare == 1'b0) begin
                        comp_term_pos_n                 <= scanline_output_term_pos_n >>> 4;        // Compensating for DW difference in fraction
                        comp_term_neg_n                 <= scanline_output_term_neg_n >>> 4;
                        initiate_incrementAndCompare    <= 1'b1;
                        ack_element                     <= 1'b1;
                    end
                    else begin
                        initiate_incrementAndCompare    <= 1'b0;
                        ack_element                     <= 1'b0;
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
                        if (scanline_done == 1'b1) begin
                            initiate_transmit               <= 1'b1;
                            element_counter                 <= 5'd0;
                            point_counter                   <= point_counter + 1;
                        end
                        else begin
                            element_counter     <= element_counter + 1;
                        end
                    end
                    else begin
                        ack_incrementAndCompare         <= 1'b0;
                    end
                end
                TRANSMIT: begin
                    initiate_transmit   <= 1'b0;
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

        .initiate(initiate),
        .ack(reference_ack),
        .r_0(r_0),
        .angle(angle),

        .output_term_pos_n(reference_output_term_pos_n),
        .output_term_neg_n(reference_output_term_neg_n),
        .last_element(reference_calculation_done),
        .ready(reference_ready)
    );

    // Instantiating the scanline calculator module
    NextPointIncrementTermCalculator #(
        .DW_INTEGER(18),
		.DW_FRACTION(8),
        .ANGLE_DW(8),
        .DW_INPUT(8)
    ) calcPoint (
        .clk(clk),
        .rst(rst),

        .configure(configure_scanline),
        .ack(ack_element),
        .r_0(r_0_reg),
        .angle(angle_reg),

        .output_term_pos_n(scanline_output_term_pos_n),
        .output_term_neg_n(scanline_output_term_neg_n),
        .last_element(scanline_done),
        .ready(element_ready)
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