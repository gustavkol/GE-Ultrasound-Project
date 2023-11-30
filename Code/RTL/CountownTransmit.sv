module CountdownTransmit  #(
                            parameter N_DW_INTEGER = 13,
                            parameter DW_FRACTION_INCANDCOMPARE = 4
                            )(
                            input                                                   clk,                // Clock signal
                            input                                                   rst,                // Reset signal
                            // Input values
                            input                                                   initiate,           // Initiates countdown
                            input signed [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0] delayArray[63:0],
                            input signed [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0] minValue,
                            input signed [N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:0] maxValue,
                            // Output values
                            output [63:0]                                           txArray,
                            output                                                  done               // Result ready signal
                            );

    logic[N_DW_INTEGER+3:0]   counter;      // TODO: REMOVE FRACTION BEFORE SYNTHESIZINGss

    enum {LOAD, TRANSMIT, IDLE} state, nextState;

    // Assign output
    generate
        genvar i;
        for (i = 0; i < 64; i = i + 1) begin : COMP_INST
            assign txArray[i] = delayArray[i][N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:DW_FRACTION_INCANDCOMPARE-2] == counter[N_DW_INTEGER+3:1] ? (state == TRANSMIT) : 1'b0;
        end
    endgenerate
    assign done     = state == TRANSMIT && (counter[N_DW_INTEGER+3:1] == maxValue[N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:DW_FRACTION_INCANDCOMPARE-2]);


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
                    counter[N_DW_INTEGER+3:1]   <= minValue[N_DW_INTEGER+DW_FRACTION_INCANDCOMPARE:DW_FRACTION_INCANDCOMPARE-2];
                end
                TRANSMIT: begin
                    counter     <= counter + 2'b10;
                end
            endcase
        end
    end

endmodule