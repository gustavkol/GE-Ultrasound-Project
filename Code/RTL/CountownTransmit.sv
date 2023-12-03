module CountdownTransmit  #(
                            parameter DW_N_INTEGER = 13,
                            parameter DW_FRACTION_INCANDCOMPARE = 4,
                            parameter NUM_ELEMENTS  = 64
                            )(
                            input                                                   clk,                // Clock signal
                            input                                                   rst,                // Reset signal
                            // Input values
                            input                                                   initiate,           // Initiates countdown
                            input signed [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0] delayArray[NUM_ELEMENTS-1:0],
                            input signed [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0] minValue,
                            input signed [DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:0] maxValue,
                            // Output values
                            output [NUM_ELEMENTS-1:0]                               txArray,
                            output                                                  done               // Result ready signal
                            );

    logic[DW_N_INTEGER+2:0]   counter;

    enum {LOAD, TRANSMIT, IDLE} state, nextState;

    // Assign output
    generate
        genvar i;
        for (i = 0; i < NUM_ELEMENTS; i = i + 1) begin : COMP_INST
            assign txArray[i] = delayArray[i][DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:DW_FRACTION_INCANDCOMPARE-2] == counter[DW_N_INTEGER+2:0] ? (state == TRANSMIT) : 1'b0;
        end
    endgenerate
    assign done     = state == TRANSMIT && (counter[DW_N_INTEGER+2:0] == maxValue[DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:DW_FRACTION_INCANDCOMPARE-2]);


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
                IDLE:       if (initiate)                               nextState = LOAD;
                LOAD:                                                   nextState = TRANSMIT;
                TRANSMIT:   if (done)                                   nextState = IDLE;
            endcase
        end
    end


    always @(posedge clk) begin
        if(rst) begin
            counter         <= '0;
        end
        else begin
            case (state)
                IDLE: begin
                    counter         <= '0;
                end
                LOAD: begin
                    counter[DW_N_INTEGER+2:0]   <= minValue[DW_N_INTEGER+DW_FRACTION_INCANDCOMPARE:DW_FRACTION_INCANDCOMPARE-2];
                end
                TRANSMIT: begin
                    counter     <= counter + 1;
                end
            endcase
        end
    end

endmodule