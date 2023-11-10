`timescale 1 ns / 1 ps
module tb_CordicCosine;
    
    parameter   DW_ANGLE = 7,
                DW_FRACTION = 6,
                DW_CALCULATION_TERMS = 16;

    // input signals
    reg                                         clk, rst, initiate, ack;
    reg [DW_ANGLE:0]                            angle;
    reg [DW_CALCULATION_TERMS+DW_FRACTION:0]    x_scale;

    // output signals
    wire [DW_CALCULATION_TERMS+DW_FRACTION:0]   result;
    wire                                        ready;

    // generation of the clock signal
    always  #5 clk = ~clk;

    // Stimuli
    initial begin
        clk = 0; rst = 0; initiate = 0; ack = 0;

        // CORDIC input
        angle           = 60;
        x_scale         = 24'b000000000000000001_000000;

        #5 rst           = 1;
        #40 rst          = 0;

        #70 initiate     = 1;
        #20  initiate    = 0;

        wait(ready == 1'b1);
        #20 ack = 1;
        #20 ack = 0;

    end

    // Instantiating the calculator module
    CordicCosine #(
        .DW_ANGLE(DW_ANGLE),
        .DW_FRACTION(DW_FRACTION),
        .DW_CALCULATION_TERMS(DW_CALCULATION_TERMS)
    ) cordicInstance (
        .clk(clk),
        .rst(rst),
        .initiate(initiate),
        .angle(angle),
        .x_scale(x_scale),
        .ack(ack),
        .result(result),
        .ready(ready)
    );
endmodule
