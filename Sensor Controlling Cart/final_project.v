module clock_divider #(parameter n = 26)(
    input clk, 
    input en,
    input pause,
    output clk_div
);
    reg [n:0] num = 0;
    wire [n:0] next_num;

    always@(posedge clk) begin
        if(!en) begin
            num <= 0;
        end else begin
            if(!pause) begin
                num <= next_num;
            end
        end
    end

    assign next_num = num + 1;
    //assign clk_div = num[n];
    assign clk_div = pause ? 0 : num[n];
endmodule

module final_project(
    input clk,
    input rst,
    input echo,
    input RxD,
    input [1:0] sw,
    input IRSenseL,
    input IRSenseR,
    input IRSenseBL,
    input IRSenseBR,
    output trig,
    output IN1,
    output IN2,
    output IN3, 
    output IN4,
    output left_pwm,
    output right_pwm,
    output [15:0] led,
    output [3:0] DIGIT,
    output [6:0] DISPLAY
);
    reg [2:0] mode;
    reg [1:0] type, next_type;
    reg clk_turn_en, clk_u_turn_en, clk_rotate_en;
    reg clk_blink_en;
    reg sevenSeg;
    reg turnControl;
    reg [10:0] blink_;

    wire [7:0] RxData;
    wire [19:0] distance;
    wire [19:0] distance1;
    wire [15:0] BCD;
    wire [3:0] num0, num1, num2, num3;
    wire IRSignL, IRSignR;
    wire IRSignBL, IRSignBR;
    wire clk_turn, clk_u_turn, clk_rotate;
    wire clk_blink;
    wire pause, pause1;

    motor A(
        .clk(clk),
        .rst(rst),
        // .mode(3'b000),
        .mode(mode),
        .pwm({left_pwm, right_pwm}),
        .l_IN({IN1, IN2}),
        .r_IN({IN3, IN4})
    );

    sonic_top B0(
        .clk(clk), 
        .rst(rst), 
        .Echo(echo), 
        .Trig(trig),
        .distance(distance)
    );

    bluetooth C(
        .clk(clk),
        .rst(rst),
        .RxD(RxD),
        .RxData(RxData)
    );

    SevenSegment D(
	    .clk(clk),
	    .rst(rst),
	    .nums(BCD),
        .display(DISPLAY),
	    .digit(DIGIT)
    );
    
    IRSensor E(
        .clk(clk),
        .rst(rst),
        .Sense_(IRSenseL),
        .obstacles_(IRSignL)
    );

    IRSensor F(
        .clk(clk),
        .rst(rst),
        .Sense_(IRSenseR),
        .obstacles_(IRSignR)
    );

    IRSensor E1(
        .clk(clk),
        .rst(rst),
        .Sense_(IRSenseBL),
        .obstacles_(IRSignBL)
    );

    IRSensor F1(
        .clk(clk),
        .rst(rst),
        .Sense_(IRSenseBR),
        .obstacles_(IRSignBR)
    );

    clock_divider #(.n(26)) G ( // turn_duration
        .clk(clk),
        .en(clk_turn_en), // add distance pause
        .pause(pause),  // pause the duration from keep counting if there's obstacle
        .clk_div(clk_turn)
    );

    clock_divider #(.n(28)) H ( // u_turn_duration
        .clk(clk),
        .en(clk_u_turn_en), // add distance pause
        .pause(pause),  // pause the duration from keep counting if there's obstacle
        .clk_div(clk_u_turn)
    );

    clock_divider #(.n(29)) I ( // rotate_duration
        .clk(clk),
        .en(clk_rotate_en), // add distance pause
        .pause(pause),  // pause the duration from keep counting if there's obstacle
        .clk_div(clk_rotate)
    );

    clock_divider #(.n(25)) J ( // obstacles
        .clk(clk),
        .en(clk_blink_en),
        .pause(0),
        .clk_div(clk_blink)
    );

    assign led = {mode, (!clk_blink && clk_blink_en) ? 11'b1111_1111_111 : 11'd0, type};
    assign pause = (distance < 20) ? 1'b1 : 1'b0;
    // assign pause1 = (IRSignBL || IRSignBR) ? 1'b1 : 1'b0;
    
    assign num0 = RxData % 10;
    assign num1 = (RxData/10) % 10;
    assign num2 = (RxData/100) % 10;
    assign num3 = (RxData/1000) % 10;

    // assign BCD = 1'b1 ? {num3, num2, num1, num0} : 16'd0;
    assign BCD = !clk_blink_en ? (sevenSeg ? {num3, num2, num1, num0} : 16'd0) : 
                                (!clk_blink ? {4'd10, 4'd10, 4'd10, 4'd10} : {4'd11, 4'd11, 4'd11, 4'd11});
    
    always@(posedge clk, posedge rst) begin
        if(rst) type <= 2'b01;
        else type <= next_type;
    end

    always@* begin
        next_type = type;
        if(sw[0] && !sw[1]) begin   // follow
            next_type = 2'b01;
        end else if(!sw[0] && sw[1]) begin // speech
            next_type = 2'b10;
        end else if((num2 == 0) && (num1 == 4) && (num0 == 9)) begin   // "one" follow
            next_type = 2'b01;
        end else if((num2 == 0) && (num1 == 5) && (num0 == 0)) begin   // "two" speech
            next_type = 2'b10;
        end
    end

    ///////////////////////////////
    // speech:
    //      go
    //      move backward
    //      turn left
    //      right
    //      stop
    //      u-turn
    //      hi
    //      goodbye
    ////////////////////////////////

    always@(posedge clk, posedge rst) begin
        if(rst) begin
            mode <= 3'b000;
            clk_turn_en <= 0;
            clk_u_turn_en <= 0;
            clk_rotate_en <= 0;
            sevenSeg <= 0;
            turnControl <= 0;
        end else begin
            mode <= 3'b000;
            case(type)
                2'b01 : begin
                    if((distance > 15) && (distance < 60) && IRSignL && IRSignR) begin
                        mode <= 3'b011; // forward
                    end else if((distance > 15) && (distance < 60) && IRSignL) begin
                        mode <= 3'b010; // left
                    end else if((distance > 15) && (distance < 60) && IRSignR) begin
                        mode <= 3'b001; // right
                    end else begin
                        mode <= 3'b000; // stop
                    end
                end
                2'b10 : begin
                    if((num2 == 1) && (num1 == 1) && (num0 == 1)) begin  // forward
                        mode <= 3'b011;  // forward
                        sevenSeg <= 1;
                        turnControl <= 0;
                        clk_turn_en <= 0;
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                    end else if((num2 == 2) && (num1 == 4) && (num0 == 7)) begin    // turn left
                        clk_turn_en <= 1;        // start turn duration
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b010;  // left
                        end else begin
                            mode <= 3'b000; //stop
                            // mode <= 3'b011; //forward
                        end
                        if(clk_turn) begin
                            clk_turn_en <= 0;
                            turnControl <= 1;       // to turn or not
                            //mode <= 3'b011; // forward
                        end
                    // end else if((num2 == 2) && (num1 == 5) && (num0 == 5)) begin    // turn right
                    end else if((num2 == 1) && (num1 == 8) && (num0 == 6)) begin    // turn right
                        clk_turn_en <= 1;        // start turn duration
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b001;  // right
                        end else begin
                            mode <= 3'b000; //stop
                            // mode <= 3'b011;  // forward
                        end
                        if(clk_turn) begin
                            clk_turn_en <= 0;
                            turnControl <= 1;       // to turn or not
                            //mode <= 3'b011; // forward
                        end
                    end else if((num2 == 2) && (num1 == 5) && (num0 == 1)) begin // backward
                    // end else if((num2 == 2) && (num1 == 5) && (num0 == 1) && !IRSignBL && !IRSignBR) begin // backward
                        mode <= 3'b100;  // backward 
                        sevenSeg <= 1;
                        turnControl <= 0;
                        clk_turn_en <= 0;
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                    end else if((num2 == 1) && (num1 == 8) && (num0 == 3)) begin // u-turn
                        clk_turn_en <= 0;        // start turn duration
                        clk_u_turn_en <= 1;     // start u-turn duration
                        clk_rotate_en <= 0;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b010;  // left
                        end else begin
                            mode <= 3'b000; //stop
                            // mode <= 3'b011;  // forward
                        end
                        if(clk_u_turn) begin
                            clk_u_turn_en <= 1'b0;
                            turnControl <= 1'b1;       // to turn or not
                            //mode <= 3'b011; // forward
                        end
                    // end else if((num2 == 2) && (num1 == 1) && (num0 == 7)) begin    // turn left 90 degree
                    //     clk_turn_en <= 1;        // start turn duration
                    //     clk_u_turn_en <= 0;
                    //     clk_rotate_en <= 0;
                    //     sevenSeg <= 1;      // show BCD
                    //     if(turnControl == 0) begin
                    //         mode <= 3'b010;  // left
                    //     end
                    //     if(clk_turn) begin
                    //         clk_turn_en <= 0;
                    //         turnControl <= 1;       // to turn or not
                    //     end
                    // end else if((num2 == 2) && (num1 == 4) && (num0 == 6)) begin    // turn right 90 degree
                    //     clk_turn_en <= 1;        // start turn duration
                    //     clk_u_turn_en <= 0;
                    //     clk_rotate_en <= 0;
                    //     sevenSeg <= 1;      // show BCD
                    //     if(turnControl == 0) begin
                    //         mode <= 3'b001;  // right
                    //     end
                    //     if(clk_turn) begin
                    //         clk_turn_en <= 0;
                    //         turnControl <= 1;       // to turn or not
                    //     end
                    end else if((num2 == 1) && (num1 == 0) && (num0 == 5)) begin    // rotate left  hi
                        clk_turn_en <= 0;        // start turn duration
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 1;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b010;  // left
                        end else begin
                            mode <= 3'b000; //stop
                        end
                        if(clk_rotate) begin
                            clk_rotate_en <= 0;
                            turnControl <= 1;       // to turn or not
                        end
                    end else if((num2 == 2) && (num1 == 1) && (num0 == 7)) begin    // rotate right goodbye
                        clk_turn_en <= 0;        // start turn duration
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 1;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b001;  // right
                        end else begin
                            mode <= 3'b000; //stop
                        end
                        if(clk_rotate) begin
                            clk_rotate_en <= 0;
                            turnControl <= 1;       // to turn or not
                        end
                    end else begin
                        mode <= 3'b000;  // stop
                        sevenSeg <= 0;
                        turnControl <= 0;
                        clk_turn_en <= 0;
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                    end
                end
            endcase
            if(pause) begin
                // if(mode != 3'b011 && mode != 3'b000 && turnControl != 1'b1 && !IRSignBR && !IRSignBL) begin
                if(mode != 3'b011 && mode != 3'b000 && turnControl != 1'b1) begin
                    mode <= 3'b100; // backward
                end else begin
                    mode <= 3'b000; //forward
                end
            end
            // if(pause1)begin
            //     mode <= 3'b000; //backward
            // end
        end
    end

    always@(*)begin
    //   if((pause || pause1) && type == 2'b10)begin
      if(pause && type == 2'b10)begin
        clk_blink_en <= 1;
      end else begin
        clk_blink_en <= 0;
      end
    end

endmodule

// 158, 263, 364, 356-358
