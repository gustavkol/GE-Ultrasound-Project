/* Module calculating the term L_kn = A_0(2n+1) +/- C_0 for all n elements*/
module IncrementAndCompareArray  #(
                            parameter DW_INC_TERM_INTEGER = 16,
                            parameter DW_N_INTEGER = 13,
                            parameter DW_ERROR_INTEGER = 14,
                            parameter DW_A_INTEGER = 4,
                            parameter DW_FRACTION = 4,
                            parameter NUM_ELEMENTS  = 64,
                            parameter DW_INPUT = 8,
                            parameter DW_ANGLE = 8
                            )(
                            input                                               clk,                                // Clock signal
                            input                                               rst,                                // Reset signal
                            // Input values
                            input [DW_INPUT-1:0]                                r_0,                // R_0 input
                            input [DW_ANGLE-1:0]                                angle,              // angle input
                            input                                               configure,
                            input          [DW_N_INTEGER+DW_FRACTION:0]         n_prev      [NUM_ELEMENTS-1:0],      // Output delays for all elements
                            input signed   [DW_ERROR_INTEGER+DW_FRACTION:0]     error_prev  [NUM_ELEMENTS-1:0],
                            input                                               ack,                               // Acknowledges output has been read
                            input                                               transmit_done,
                            input                                               final_scanpoint,
                            // Output values
                            output          [DW_N_INTEGER+DW_FRACTION:0]        n_out      [NUM_ELEMENTS-1:0],      // Output delays for all elements
                            output signed   [DW_ERROR_INTEGER+DW_FRACTION:0]    error_out  [NUM_ELEMENTS-1:0],
                            output                                              ready,                               // Result ready signal
                            output                                              done_configuring
                            );



    // IncrementAndCompare inputs/outputs
    logic signed   [DW_A_INTEGER+DW_FRACTION:0]             a_next           [NUM_ELEMENTS-1:0];     // Compensated delays
    logic          [2*DW_A_INTEGER+DW_FRACTION:0]           a_next_sq        [NUM_ELEMENTS-1:0];     // Compensated squared delays
    logic signed   [DW_INC_TERM_INTEGER+DW_FRACTION:0]      comp_term_next   [NUM_ELEMENTS-1:0];     // Comparator term propagated to next
    logic signed   [DW_INC_TERM_INTEGER+DW_FRACTION:0]      comp_terms       [NUM_ELEMENTS-1:0];     // Comparator term propagated to next

    logic                       calc_ready;                     // Calculator ready signal
    logic [NUM_ELEMENTS-1:0]    ready_reg;                      // Delay ready signals
    logic                       initiate_incrementAndCompare;
    logic                       rst_calc;

    // Assigning outputs
    assign ready            = (ready_reg == '1);
    assign rst_calc         = rst || final_scanpoint;

    always_ff @(posedge clk) begin
        if (rst) begin
            initiate_incrementAndCompare    <= '0;
        end
        else begin
            if (calc_ready)
                initiate_incrementAndCompare    <= ack;
        end
    end


    // Generating one incrementAndCompare for each element
    genvar i;
    generate
        for(i = 0; i < NUM_ELEMENTS; i = i + 1) begin
            IncrementAndCompare #(
                .DW_INC_TERM_INTEGER(DW_INC_TERM_INTEGER),
                .DW_N_INTEGER(DW_N_INTEGER),
                .DW_ERROR_INTEGER(DW_ERROR_INTEGER),
                .DW_A_INTEGER(DW_A_INTEGER),
                .DW_FRACTION(DW_FRACTION)
            ) incAndCompInst (
                .clk(clk),
                .rst(rst),
                // Input
                .initiate(initiate_incrementAndCompare),
                .ack(ack),
                .n_prev(n_prev[i]),
                .a_prev(a_next[i]),
                .a_prev_sq(a_next_sq[i]),
                .comp_term(comp_terms[i]),
                .comp_term_prev(comp_term_next[i]),
                .error_prev(error_prev[i]),
                // Output
                .n_next(n_out[i]),
                .a_next(a_next[i]),
                .a_next_sq(a_next_sq[i]),
                .comp_term_next(comp_term_next[i]),
                .error_next(error_out[i]),
                .ready(ready_reg[i]),
                .available()
            );
        end
    endgenerate

    NextPointIncrementTermCalculator  #(
        .DW_INPUT(DW_INPUT),
        .DW_ANGLE(DW_ANGLE)
    ) calc_inst (
        .clk(clk),                          // Clock signal
        .rst(rst_calc),                     // Reset signal
    // Input values
        .configure(configure),              // Signal initiating first calculation and configures values
        .ack(ack),                          // Acknowledges output has been read
        .final_scanpoint(final_scanpoint),  // Signal indicating last point of scanline
        .r_0(r_0),                          // R_0 input
        .angle(angle),                      // angle input
    // Output values
        .output_terms(comp_terms),          // Comparator term output for pos n
        .done_configuring(done_configuring),
        .ready(calc_ready)                  // Result ready signal
    );



endmodule