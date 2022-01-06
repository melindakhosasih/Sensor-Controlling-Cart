module final_project(
    input clk,
    input rst,
    input echo,
    input RxD,
    input [1:0] sw,
    output trig,
    output IN1,
    output IN2,
    output IN3, 
    output IN4,
    output left_pwm,
    output right_pwm,
    output [1:0] led,
    output [3:0] DIGIT,
    output [6:0] DISPLAY
);
    reg [2:0] mode;
    reg [1:0] type, next_type;
    wire [7:0] RxData;
    wire [19:0] distance;
    wire [15:0] BCD;
    
    motor A(
        .clk(clk),
        .rst(rst),
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

    assign BCD[3:0] = RxData % 10;
    assign BCD[7:4] = (RxData/10) % 10;
    assign BCD[11:8] = (RxData/100) % 10;
    assign BCD[15:12] = (RxData/1000) % 10;

    assign led = type;
    
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
        end else if((BCD[11:8] == 0) && (BCD[7:4] == 4) && (BCD[3:0] == 9)) begin   // "one" follow
            next_type = 2'b01;
        end else if((BCD[11:8] == 0) && (BCD[7:4] == 5) && (BCD[3:0] == 0)) begin   // "two" speech
            next_type = 2'b10;
        end
    end

    always@(posedge clk, posedge rst) begin
        if(rst) begin
            mode <= 3'b000;
        end else begin
            mode <= 3'b000;
            case(type)
                2'b01 : begin
                    if(distance > 20 && distance < 50) begin
                        mode <= 3'b011;
                    end else begin
                        mode <= 3'b000;
                    end
                end
                2'b10 : begin
                    if((BCD[11:8] == 1) && (BCD[7:4] == 1) && (BCD[3:0] == 1)) begin
                        mode <= 3'b011;  // forward
                    end else if((BCD[11:8] == 2) && (BCD[7:4] == 4) && (BCD[3:0] == 7)) begin
                        mode <= 3'b001;  // left
                    end else if((BCD[11:8] == 2) && (BCD[7:4] == 5) && (BCD[3:0] == 5)) begin
                        mode <= 3'b010;  // right
                    end else if((BCD[11:8] == 2) && (BCD[7:4] == 5) && (BCD[3:0] == 1)) begin
                        mode <= 3'b100;  // backward 
                    end else begin
                        mode <= 3'b000;  // stop
                    end
                end
            endcase
        end
    end

endmodule
