module clock_divider #(parameter n= 26)(
    input clk, 
    input en,
    output clk_div
);
    reg [n:0] num = 0;
    wire [n:0] next_num;
    always@(posedge clk)begin
      if(!en)begin
        num = 0;
      end else begin
        num = next_num;
      end
    end
    assign next_num = num + 1;
    assign clk_div = num[n];
endmodule

module final_project(
    input clk,
    input rst,
    input echo,
    input RxD,
    input [1:0] sw,
    input IRSenseL,
    input IRSenseR,
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
    reg clk20_en;
    reg sevenSeg;
    reg turnControl;

    // wire primaryRxD;
    wire [7:0] RxData;
    wire [19:0] distance;
    wire [15:0] BCD;
    wire IRSignL, IRSignR;
    wire clk20;
    wire [3:0] num0, num1, num2, num3;
    reg [3:0] num00, num11, num22, num33;
    
    clock_divider #(.n(23)) clk_20(clk, clk20_en, clk20);

    motor A(
        .clk(clk),
        .rst(rst),
        // .mode(3'b000),
        .mode(mode),
        .pwm({left_pwm, right_pwm}),
        .l_IN({IN1, IN2}),
        .r_IN({IN3, IN4})
    );

    sonic_top B(
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

    assign led = {mode, 11'd0, type};
    // assign primaryRxD = !rst && !clk20 ? RxD : 8'd0;
    // assign primaryRxD = RxD;
    
    assign num0 = RxData % 10;
    assign num1 = (RxData/10) % 10;
    assign num2 = (RxData/100) % 10;
    assign num3 = (RxData/1000) % 10;

    // assign BCD = 1'b1 ? {num3, num2, num1, num0} : 16'd0;
    assign BCD = sevenSeg ? {num3, num2, num1, num0} : 16'd0;
    
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

    // speech:
    // go
    // move backward
    // turn left
    // turn right
    // stop

    always@(posedge clk, posedge rst) begin
        if(rst) begin
            mode <= 3'b000;
            clk20_en <= 0;
            sevenSeg <= 0;
            turnControl <= 0;
        end else begin
            mode <= 3'b000;
            case(type)
                2'b01 : begin
                    if((distance > 15) && (distance < 60) && IRSignL && IRSignR) begin
                        mode <= 3'b011; // forward
                    end else if((distance > 15) && (distance < 60) && IRSignL)begin
                        mode <= 3'b001; // left
                    end else if((distance > 15) && (distance < 60) && IRSignR)begin
                        mode <= 3'b010; // right
                    end else begin
                        mode <= 3'b000; // stop
                    end
                end
                2'b10 : begin
                    if((num2 == 1) && (num1 == 1) && (num0 == 1) && (distance > 20)) begin
                        mode <= 3'b011;  // forward
                        sevenSeg <= 1;
                        turnControl <= 0;
                        clk20_en <= 0;
                    end else if((num2 == 2) && (num1 == 4) && (num0 == 7)) begin

                        clk20_en <= 1;        // start turn duration
                        if(turnControl == 0)begin
                            mode <= 3'b010;  // left
                        end
                        sevenSeg <= 1;      // show BCD
                        if(clk20)begin
                            clk20_en <= 0;
                            turnControl <= 1;       // to turn or not
                        end

                    end else if((num2 == 2) && (num1 == 5) && (num0 == 5)) begin
                        clk20_en <= 1;        // start turn duration
                        if(turnControl == 0)begin
                            mode <= 3'b001;  // right
                        end
                        sevenSeg <= 1;      // show BCD
                        if(clk20)begin
                            clk20_en <= 0;
                            turnControl <= 1;       // to turn or not
                        end

                    end else if((num2 == 2) && (num1 == 5) && (num0 == 1) && (distance > 20)) begin
                        mode <= 3'b100;  // backward 
                        sevenSeg <= 1;
                        turnControl <= 0;
                        clk20_en <= 0;
                    end else begin
                        mode <= 3'b000;  // stop
                        sevenSeg <= 0;
                        turnControl <= 0;
                        clk20_en <= 0;
                    end
                end
            endcase
        end
    end

endmodule
