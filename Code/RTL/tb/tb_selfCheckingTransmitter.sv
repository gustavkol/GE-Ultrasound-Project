`timescale 1 ns / 1 ps
module tb_Transmitter;
    parameter DW_INTEGER    = 18;
    parameter DW_FRACTION   = 8;
    parameter ANGLE_DW      = 8;
    parameter DW_INPUT      = 8;

    // input signals
    reg                             clk, rst, initiate;
    reg [DW_INPUT-1:0]              r_0;
    reg [ANGLE_DW-1:0]              angle;
    reg [12:0]                      num_points;

    // output signals
    wire [63:0]                     txArray;
    wire                            done;

    // Input generation and vectors
    localparam NUM_INPUTS                                   = 5;
    localparam [DW_INPUT-1:0] R_0_INPUT_VECTOR    [NUM_INPUTS-1:0]  = {{8'd30},{8'd50},{8'd70},{8'd90},{8'd110}};
    localparam [ANGLE_DW-1:0] ANGLE_INPUT_VECTOR  [NUM_INPUTS-1:0]  = {{8'd70},{8'd110},{8'd50},{8'd130},{8'd90}};
    real R_0_REAL  [NUM_INPUTS-1:0]                       = {30.0*10**-3.0,50.0*10**-3.0,70.0*10**-3.0,90.0*10**-3.0,110.0*10**-3.0};
    real ANGLE_REAL [NUM_INPUTS-1:0]                       = {70,110,50,130,90};
    
    // Reference values
    real                            reference_delay, diff;

    typedef enum {LOAD, NEXT_ELEMENT_CALC, LOAD_NEXT, TRANSMIT, IDLE} states;
    states transmit = TRANSMIT;

    localparam  PI              = 3.14159;
    localparam  F_S             = 25*10**6.0;
    localparam  V_S             = 1540;
    localparam  PITCH           = 250*10**-6.0;
    localparam  DELTA_L         = V_S/F_S;
    localparam  SF              = 2.0**-4.0;

    integer                      scanline_index;
    integer                      num_assertions;
    real                         point_index;
    real                         cur_index;




    // generation of the clock signal
    always  #5 clk = ~clk;


    // Stimuli
    initial begin
        clk = 0; rst = 1; initiate = 0; r_0 = 0; angle = 0;
        num_points = 0; point_index = 0.0; scanline_index = 0; cur_index = 0.0; num_assertions = 0;

        #40 rst         = 0;

        #20
        for (int i = 0; i < NUM_INPUTS; i++) begin
            angle        = ANGLE_INPUT_VECTOR[i];
            r_0          = R_0_INPUT_VECTOR[i];
            num_points   = 13'd3;
            #70 initiate     = 1;
            #10 initiate     = 0;
            wait(done == 1'b1);
        end
    end


// **** SELF CHECKING FUNCTIONALITY **** //

    always @(posedge clk) begin
        if (rst) begin
            scanline_index  = 0;
            point_index     = 0;
        end
        else if (done == 1'b1) begin
            scanline_index++;
            point_index = 0;
        end
    end

    always @(posedge clk) begin
        if (~rst)
            if (transmitter_instance.state == transmit && $past(transmitter_instance.state) != transmit) begin
                for (int n = 0; n < 64; n++) begin      // Calculating and comparing reference delay
                    cur_index   = n - 31.0;
                    reference_delay = (F_S / V_S) * $sqrt( (cur_index * PITCH)**2.0 - 2.0*cur_index*PITCH*(R_0_REAL[scanline_index] + point_index * DELTA_L) * $cos(ANGLE_REAL[scanline_index] * PI / 180.0) + (R_0_REAL[scanline_index] + point_index * DELTA_L)**2.0);
                    assert (reference_delay + 1.0 > SF*transmitter_instance.delayArray[n] && reference_delay - 1.0 < SF*transmitter_instance.delayArray[n]) 
                        else begin 
                            $display ("Over 1 error in delay %f , %f", SF*transmitter_instance.delayArray[n], reference_delay);
                            num_assertions++;
                        end

                end
                point_index = point_index + 1.0;
            end
    end


    Transmitter #(
                    .DW_INTEGER(DW_INTEGER),
                    .DW_INPUT(DW_INPUT),
                    .DW_FRACTION(DW_FRACTION),
                    .ANGLE_DW(ANGLE_DW)
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
