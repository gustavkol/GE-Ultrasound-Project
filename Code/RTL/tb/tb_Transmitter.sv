`timescale 1 ns / 1 ps
module tb_Transmitter;

    parameter DW_ANGLE      = 8;
    parameter DW_INPUT      = 8;
    parameter NUM_ELEMENTS  = 64;

    // input signals
    reg                             clk, rst, initiate;
    reg [DW_INPUT-1:0]              r_0;
    reg [DW_ANGLE-1:0]              angle;
    reg [12:0]                      num_points;

    // output signals
    wire [NUM_ELEMENTS-1:0]             txArray;
    wire                                done;

    integer i;

    // Input values
    assign angle        = 8'd60;
    assign r_0          = 8'd10;
    assign num_points   = 13'd50;

    // generation of the clock signal
    always  #5 clk = ~clk;


    // Stimuli
    initial begin
        clk = 0; rst = 0; initiate = 0;

        #5 rst           = 1;
        #40 rst          = 0;
        #70 initiate     = 1;
        #10 initiate     = 0;

    end


    Transmitter #(
                    .DW_INPUT(DW_INPUT),
                    .DW_ANGLE(DW_ANGLE),
                    .NUM_ELEMENTS(NUM_ELEMENTS)
    ) transmitter_instance (
                    .clk(clk),                // Clock signal
                    .rst(rst),                // Reset signal
                    // Input values
                    .initiate(initiate),           // Initiates calculation for next scan point in scanline
                    .r_0(r_0),                // R_0 input
                    .angle(angle),              // angle input
                    .num_points(num_points),
                    // Output values
                    .txArray(txArray),            // Transmit signal for each element
                    .done(done)                // Transmittion for scanline done signal
                    );
endmodule
