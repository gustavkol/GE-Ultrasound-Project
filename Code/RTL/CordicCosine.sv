module CordicCosine # (
                    DW_ANGLE = 7,
                    DW_FRACTION = 6,
                    DW_CALCULATION_TERMS = 16
                )(
                    input                                           clk,
                    input                                           rst,
                    input                                           initiate,
                    input [DW_ANGLE:0]                              angle,
                    input [DW_CALCULATION_TERMS+DW_FRACTION:0]      x_scale,
                    input                                           ack,

                    output signed [DW_CALCULATION_TERMS+DW_FRACTION:0]      result,
                    output                                                  ready
                );

    // State machine variables
    enum {LOAD, RUN, WAIT, IDLE} state, nextState;

    // Internal registers
    logic signed [DW_CALCULATION_TERMS+DW_FRACTION:0]   x_cur_reg;            // NOTE: may not have to be signed
    logic signed [DW_CALCULATION_TERMS+DW_FRACTION:0]   y_cur_reg;            // NOTE: may not have to be signed
    logic signed [DW_ANGLE+DW_FRACTION:0]               angle_cur_reg;        // NOTE: may not have to be signed
    logic                                               sign_bit;
    logic                                               sign_increment;
    logic                                               done_reg;
    logic [3:0]                                         counter;

    localparam signed [6+6:0] ROTATED_ANGLE_ARRAY [8:0] = {      // rotated angle for each iteration
        {13'b0000000_010000},          // 0.25
        {13'b0000000_011100},          // 0.4375
        {13'b0000000_111000},          // 0.875
        {13'b0000001_110000},          // 1.75
        {13'b0000011_100100},          // 3.5625
        {13'b0000111_001000},          // 7.125
        {13'b0001110_001000},          // 14.125
        {13'b0011010_100100},          // 26.5625
        {13'b0101101_000000}            // 45
    };
    localparam [3:0] num_iterations = 4'd9;

    logic signed [DW_ANGLE+DW_FRACTION+1:0] next_angle;
    assign next_angle = (sign_increment == 1'b0) ? angle_cur_reg - ROTATED_ANGLE_ARRAY[counter]  :   angle_cur_reg + ROTATED_ANGLE_ARRAY[counter];

    // assigning outputs
    assign result   = (state == WAIT) ? ((sign_bit == 1'b0 ? x_cur_reg : -x_cur_reg)) : '0;
    assign ready    = (state == WAIT) ? done_reg : '0;


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
                IDLE:       if (initiate)                           nextState = LOAD;
                LOAD:                                               nextState = RUN;
                RUN:        if (counter == num_iterations - 1'd1)   nextState = WAIT;
                WAIT:       if (ack)                                nextState = IDLE;
            endcase
        end
    end

    // Algorithm
    always @(posedge clk) begin
        if(rst) begin
            x_cur_reg       <= '0;
            y_cur_reg       <= '0;
            angle_cur_reg   <= '0;
            sign_bit        <= '0;
            sign_increment  <= '0;
            done_reg        <= '0;
            counter         <= '0;
        end
        else begin
            case(state)
                IDLE: begin
                    x_cur_reg       <= '0;
                    y_cur_reg       <= '0;
                    angle_cur_reg   <= '0;
                    sign_bit        <= '0;
                    sign_increment  <= '0;
                    done_reg        <= '0;
                    counter         <= '0;
                end
                LOAD: begin
                    x_cur_reg       <= (x_scale >> 1) + (x_scale >> 4) + (x_scale >> 5) + (x_scale >> 6);               // K_n = 0.6088 ~ 0.609375 = 2^-1 + 2^-4 + 2^-5 + 2^-6
                    if (angle >= 90) begin
                        angle_cur_reg   <= {180 - angle, 6'b0000};
                        sign_bit        <= 1'b1;
                    end
                    else begin
                        angle_cur_reg   <= {angle, 6'b0000};
                        sign_bit        <= 1'b0;
                    end
                end
                RUN: begin
                    if (counter == num_iterations-1'b1) begin
                        done_reg        <= 1'b1;
                        x_cur_reg       <= (sign_increment == 1'b0) ? (x_cur_reg - (y_cur_reg >>> counter)) : (x_cur_reg + (y_cur_reg >>> counter));
                    end
                    else if (done_reg == 1'b0) begin
                        if (sign_increment == 1'b0) begin   // +
                            x_cur_reg       <= x_cur_reg - (y_cur_reg >>> counter);
                            y_cur_reg       <= y_cur_reg + (x_cur_reg >>> counter);
                        end
                        else begin                          //-
                            x_cur_reg       <= x_cur_reg + (y_cur_reg >>> counter);
                            y_cur_reg       <= y_cur_reg - (x_cur_reg >>> counter);
                        end

                        angle_cur_reg   <= next_angle;
                        if (next_angle >= 0)
                            sign_increment    <= 1'b0;
                        else
                            sign_increment    <= 1'b1;

                        counter <= counter + 1;
                    end
                end
            endcase
        end
    end

endmodule