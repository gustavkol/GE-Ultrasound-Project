`timescale 1 ns / 1 ps
module tb_IncrementAndCompare;
    
    parameter   STEP_SIZE = 1,
                INC_TERM_DW_INTEGER = 16,
                N_DW_INTEGER = 13,
                ERROR_DW_INTEGER = 14,
                A_DW_INTEGER = 4,
                DW_FRACTIONAL = 4;

    // input signals
    reg                                                 clk, rst, initiate, ack;
    reg [N_DW_INTEGER+DW_FRACTIONAL:0]                  n_prev;
    reg signed [A_DW_INTEGER+DW_FRACTIONAL:0]           a_prev;
    reg [2*A_DW_INTEGER+DW_FRACTIONAL:0]                a_prev_sq;
    reg signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]    comp_term;
    reg signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]    comp_term_prev;
    reg signed [ERROR_DW_INTEGER+DW_FRACTIONAL:0]       error_prev;

    reg signed [31:0][INC_TERM_DW_INTEGER+DW_FRACTIONAL:0] comp_term_array_pos = {      // comparator terms for pos n when angle = 135 and R_0 = 1mm
        {21'b00000000001101101_1010},       // 109.625
        {21'b00000000010001110_1001},       // 142.5625
        {21'b00000000010101111_1000},       // 175.5
        {21'b00000000011010000_0111},       // 208.4375
        {21'b00000000011110001_0110},       // 241.375
        {21'b00000000100010010_0101},       // 274.3125
        {21'b00000000100110011_0101},       // 307.3125
        {21'b00000000101010100_0100},       // 340.25
        {21'b00000000101110101_0011},       // 373.1875
        {21'b00000000110010110_0010},       // 406.125
        {21'b00000000110110111_0001},       // 439.0625
        {21'b00000000111010111_1111},       // 471.9375
        {21'b00000000111111000_1111},       // 504.9375
        {21'b00000001000011001_1110},       // 537.875
        {21'b00000001000111010_1100},       // 570.75
        {21'b00000001001011011_1100},       // 603.75
        {21'b00000001001111100_1100},       // 636.75
        {21'b00000001010011101_1010},       // 669.625
        {21'b00000001010111110_1001},       // 702.5625
        {21'b00000001011011111_1000},       // 735.5
        {21'b00000001100000000_1000},       // 768.5
        {21'b00000001100100001_0110},       // 801.375
        {21'b00000001101000010_0110},       // 834.375
        {21'b00000001101100011_0101},       // 867.3125
        {21'b00000001110000100_0100},       // 900.25
        {21'b00000001110100101_0011},       // 933.1875
        {21'b00000001111000110_0010},       // 966.125
        {21'b00000001111100111_0001},       // 999.0625
        {21'b00000010000001000_0000},       // 1032.0
        {21'b00000010000101000_1111},       // 1064.9375
        {21'b00000010001001001_1110},       // 1097.875
        {21'b00000010001101010_1110}        // 1130.875
    };

    reg signed [30:0][INC_TERM_DW_INTEGER+DW_FRACTIONAL:0] comp_term_array_neg = {      // comparator terms for neg n when angle = 135 and R_0 = 1mm
        {21'b11111111110110011_0101},       // -76.6875
        {21'b11111111111010100_0100},       // -43.75
        {21'b11111111111110101_0011},       // -10.8125
        {21'b00000000000010110_0010},       // 22.125
        {21'b00000000000110111_0001},       // 55.0625
        {21'b00000000001011000_0000},       // 88.0
        {21'b00000000001111000_1111},       // 120.9375
        {21'b00000000010011001_1111},       // 153.9375
        {21'b00000000010111010_1110},       // 186.875
        {21'b00000000011011011_1100},       // 219.75
        {21'b00000000011111100_1100},       // 252.75
        {21'b00000000100011101_1010},       // 285.625
        {21'b00000000100111110_1010},       // 318.625
        {21'b00000000101011111_1001},       // 351.5625
        {21'b00000000110000000_1000},       // 384.5
        {21'b00000000110100001_0111},       // 417.4375
        {21'b00000000111000010_0110},       // 450.375
        {21'b00000000111100011_0101},       // 483.3125
        {21'b00000001000000100_0100},       // 516.25
        {21'b00000001000100101_0011},       // 549.1875
        {21'b00000001001000110_0010},       // 582.125
        {21'b00000001001100111_0001},       // 615.0625
        {21'b00000001010001000_0001},       // 648.0625
        {21'b00000001010101000_0000},       // 680.0
        {21'b00000001011001001_1111},       // 713.9375
        {21'b00000001011101010_1110},       // 746.875
        {21'b00000001100001011_1101},       // 779.8125
        {21'b00000001100101100_1100},       // 812.75
        {21'b00000001101001101_1011},       // 845.6875
        {21'b00000001101101110_1010},       // 878.625
        {21'b00000001110001111_1001}       // 911.5625
    };

    // output signals
    wire [N_DW_INTEGER+DW_FRACTIONAL:0]                 n_next;
    wire signed [ERROR_DW_INTEGER+DW_FRACTIONAL:0]      error_next;
    wire signed [A_DW_INTEGER+DW_FRACTIONAL:0]          a_next;
    wire [2*A_DW_INTEGER+DW_FRACTIONAL:0]               a_next_sq;
    wire signed [INC_TERM_DW_INTEGER+DW_FRACTIONAL:0]   comp_term_next;
    wire                                                ready;

    integer i;

    // generation of the clock signal
    always  #5 clk = ~clk;

    // Stimuli
    initial begin
        clk = 0; rst = 0; initiate = 0; ack = 0;

        // Initial input pos n
        n_prev           = 17'b0000000010000_0100;  //  16.25
        a_prev           = 8'b0010_0000;            //  2
        a_prev_sq        = 10'b000100_0000;         //  4
        comp_term        = comp_term_array_pos[31];
        comp_term_prev   = '0;                      //  0
        error_prev       = '0;                      //  0
        #5 rst           = 1;
        #40 rst          = 0;
        #70 initiate     = 1;
        #20  initiate    = 0;

        // Looping outputs to input of next calculation
        for (i = 30; i >= 0; i = i - 1) begin
            wait(ready == 1'b1);
            n_prev          = n_next;
            a_prev          = a_next;
            a_prev_sq       = a_next_sq;
            comp_term_prev  = comp_term_next;
            error_prev      = error_next;

            ack             = 1;
            #20  ack        = 0;

            comp_term       = comp_term_array_pos[i];

            #20 initiate    = 1;
            #20 initiate    = 0;
        end

        #200 rst            = 1;
        #40 rst             = 0;

        // Initial input neg n
        n_prev           = 17'b0000000010000_0100;  //  16.25
        a_prev           = 9'b11110_0000;            //  -2
        a_prev_sq        = 10'b000100_0000;         //  4
        comp_term        = comp_term_array_neg[30];
        comp_term_prev   = '0;                      //  0
        error_prev       = '0;                      //  0
        #20 initiate     = 1;
        #20  initiate    = 0;

                // Looping outputs to input of next calculation
        for (i = 29; i >= 0; i = i - 1) begin
            wait(ready == 1'b1);
            n_prev          = n_next;
            a_prev          = a_next;
            a_prev_sq       = a_next_sq;
            comp_term_prev  = comp_term_next;
            error_prev      = error_next;

            ack             = 1;
            #20  ack        = 0;

            comp_term       = comp_term_array_neg[i];

            #20 initiate    = 1;
            #20 initiate    = 0;
        end
    end

    // Input values

    // Instantiating the calculator module
    IncrementAndCompare #(
        .STEP_SIZE(STEP_SIZE),
        .INC_TERM_DW_INTEGER(INC_TERM_DW_INTEGER),
        .N_DW_INTEGER(N_DW_INTEGER),
        .ERROR_DW_INTEGER(ERROR_DW_INTEGER),
        .A_DW_INTEGER(A_DW_INTEGER),
        .DW_FRACTIONAL(DW_FRACTIONAL)
    ) inst1 (
        .clk(clk),
        .rst(rst),
        // Input
        .initiate(initiate),
        .ack(ack),
        .n_prev(n_prev),
        .a_prev(a_prev),
        .a_prev_sq(a_prev_sq),
        .comp_term(comp_term),
        .comp_term_prev(comp_term_prev),
        .error_prev(error_prev),
        // Output
        .n_next(n_next),
        .error_next(error_next),
        .a_next(a_next),
        .a_next_sq(a_next_sq),
        .comp_term_next(comp_term_next),
        .ready(ready)
    );
endmodule
