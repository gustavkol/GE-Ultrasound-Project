`timescale 1 ns / 1 ps
module tb_calculator;
    parameter DW_INTEGER    = 18;
    parameter DW_FRACTION   = 6;
    parameter ANGLE_DW      = 8;
    parameter DW_INPUT      = 8;

    // input signals
    reg                             clk, rst, initiate, ack;
    reg [DW_INPUT-1:0]              r_0;
    reg [ANGLE_DW-1:0]              angle;

    // output signals
    wire [DW_INTEGER+DW_FRACTION:0] output_term_pos_n, output_term_neg_n;
    wire                                ready;

    integer i;

    // Input values
    assign angle    = 8'd60;
    assign r_0      = 8'd10;

    // generation of the clock signal
    always  #5 clk = ~clk;


    // Stimuli
    initial begin
        clk = 0; rst = 0; initiate = 0; ack = 0;

        #5 rst           = 1;
        #40 rst          = 0;
        #70 initiate     = 1;
        #10 initiate     = 0;
        
        for (i = 0; i < 32; i = i + 1) begin
            wait(ready)
            #10 ack             = 1;
            #10 ack             = 0;
            if (i < 31) begin
                initiate            = 1;
                #10 initiate        = 0;
            end
        end
    end

    // Instantiating the calculator module
    NextElementIncrementTermCalculator #(
        .DW_INTEGER(DW_INTEGER),
		.DW_FRACTION(DW_FRACTION),
        .ANGLE_DW(ANGLE_DW),
        .DW_INPUT(DW_INPUT)
		  ) calc (
        .clk(clk),
        .rst(rst),

        .initiate(initiate),
        .ack(ack),
        .r_0(r_0),
        .angle(angle),

        .output_term_pos_n(output_term_pos_n),
        .output_term_neg_n(output_term_neg_n),
        .ready(ready)
    );
endmodule
