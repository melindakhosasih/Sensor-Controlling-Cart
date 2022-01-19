module IRSensor (
    input Sense_,
    output wire obstacles_
);
    reg [19:0] counter;
    assign obstacles_ = !Sense_;

endmodule

module SevenSegment(
	output reg [6:0] display,
	output reg [3:0] digit,
	input wire [15:0] nums,
	input wire rst,
	input wire clk
    );
    
    reg [15:0] clk_divider;
    reg [3:0] display_num;
    
    always @ (posedge clk, posedge rst) begin
		if(rst)begin
		  clk_divider <= 0;
		end else begin
			clk_divider <= clk_divider + 15'b1;
		end
    end
    
    always @ (posedge clk_divider[15]) begin
		case (digit)
			4'b1110 : begin
					display_num <= nums[7:4];
					digit <= 4'b1101;
				end
			4'b1101 : begin
					display_num <= nums[11:8];
					digit <= 4'b1011;
				end
			4'b1011 : begin
					display_num <= nums[15:12];
					digit <= 4'b0111;
				end
			4'b0111 : begin
					display_num <= nums[3:0];
					digit <= 4'b1110;
				end
			default : begin
					display_num <= nums[3:0];
					digit <= 4'b1110;
				end				
		endcase
    end
    
    always @ (*) begin
    	case (display_num)
    		0 : display = 7'b1000000;	//0000
			1 : display = 7'b1111001;   //0001                                                
			2 : display = 7'b0100100;   //0010                                                
			3 : display = 7'b0110000;   //0011                                             
			4 : display = 7'b0011001;   //0100                                               
			5 : display = 7'b0010010;   //0101                                               
			6 : display = 7'b0000010;   //0110
			7 : display = 7'b1111000;   //0111
			8 : display = 7'b0000000;   //1000
			9 : display = 7'b0010000;	//1001
			10 : display = 7'b0111111;	//- - - - 
			11 : display = 7'b1111111;	//_ _ _ _
			default : display = 7'b1111111;
    	endcase
    end
    
endmodule

module bluetooth (
    input clk,
	input rst,
	input RxD,
	output [7:0] RxData
    ); 

	parameter bit_div = (100_000_000 / 9_600); // read the bit every bit_div
	parameter mid = (100_000_000 / 9_600) >> 1;	// divided by 2
	parameter IDLE = 1'b0;
	parameter READ = 1'b1;

    reg [13:0] cnt;
	reg [9:0] data;
	reg [3:0] bit_cnt;
	reg state;

	assign RxData = data[7:0];

	always@(posedge clk, posedge rst) begin
		if(rst) begin
			state <= IDLE;
			cnt <= 0;
			data <= 0;
			bit_cnt <= 0;
		end else begin
			case(state)
				IDLE : begin
					if(RxD == 1'b0) begin
						cnt <= cnt + 1;
						if(cnt == mid) begin
							state <= READ;
							cnt <= 0;
							data <= 0;
							bit_cnt <= 0;
						end
					end
				end
				READ : begin
					cnt <= cnt + 1;
					if(bit_cnt == 10) begin
						state <= IDLE;
						cnt <= 0;
						bit_cnt <= 0;
					end else begin
						if(cnt == bit_div) begin
							cnt <= 0;
							data <= {RxD, data[9:1]};
							bit_cnt <= bit_cnt + 1;
						end
					end
				end
			endcase
		end
	end
endmodule

module motor(
    input clk,
    input rst,
    input [2:0] mode,
    output [1:0] pwm,
    output [1:0] r_IN,
    output [1:0] l_IN
);
    reg [9:0] next_left_motor, next_right_motor;
    reg [9:0] left_motor, right_motor;
    reg [1:0] r_temp, l_temp; // direction
    wire left_pwm, right_pwm;

    motor_pwm m0(clk, rst, left_motor, left_pwm);
    motor_pwm m1(clk, rst, right_motor, right_pwm);

    assign pwm = {left_pwm, right_pwm};
    assign r_IN = r_temp;
    assign l_IN = l_temp;

    // TODO: trace the rest of motor.v and control the speed and direction of the two motors
    always@(posedge clk, posedge rst) begin
        if(rst) begin
            left_motor <= 10'd0;
            right_motor <= 10'd0;
        end else begin
            left_motor <= next_left_motor;
            right_motor <= next_right_motor;
        end
    end

    always@(*)begin
        case(mode)
            3'b000 : begin   // stop
                next_left_motor = 10'd0;
                next_right_motor = 10'd0;
                l_temp = 2'b00; // off
                r_temp = 2'b00; // off
            end
            3'b001 : begin   // turn left
                next_left_motor = 10'd750;
                next_right_motor = 10'd750;
                l_temp = 2'b01; // backward
                r_temp = 2'b10; // forward
            end
            3'b010 : begin   // turn right
                next_left_motor = 10'd750;
                next_right_motor = 10'd750;
                l_temp = 2'b10; // forward
                r_temp = 2'b01; // backward
            end
            3'b011 : begin   // go forward
                next_left_motor = 10'd750;
                next_right_motor = 10'd750;
                l_temp = 2'b10; // forward
                r_temp = 2'b10; // forward
            end
            3'b100 : begin  // move backward
                next_left_motor = 10'd750;
                next_right_motor = 10'd750;
                l_temp = 2'b01; // backward
                r_temp = 2'b01; // backward
            end
        endcase
    end

    
endmodule

module motor_pwm (
    input clk,
    input reset,
    input [9:0] duty,
	output pmod_1 //PWM
);
    PWM_gen pwm_0 ( 
        .clk(clk), 
        .reset(reset), 
        .freq(32'd25000),
        .duty(duty), 
        .PWM(pmod_1)
    );

endmodule

//generate PWM by input frequency & duty cycle
module PWM_gen (
    input wire clk,
    input wire reset,
	input [31:0] freq,
    input [9:0] duty,
    output reg PWM
);
    wire [31:0] count_max = 100_000_000 / freq;
    wire [31:0] count_duty = count_max * duty / 1024;
    reg [31:0] count;
        
    always @(posedge clk, posedge reset) begin
        if (reset) begin
            count <= 0;
            PWM <= 0;
        end else if (count < count_max) begin
            count <= count + 1;
            if(count < count_duty)
                PWM <= 1;
            else
                PWM <= 0;
        end else begin
            count <= 0;
            PWM <= 0;
        end
    end
endmodule


`define c   32'd262   // C3
`define g   32'd392   // G3
`define b   32'd494   // B3
`define hc  32'd524   // C4
`define hd  32'd588   // D4
`define he  32'd660   // E4
`define hf  32'd698   // F4
`define hg  32'd784   // G4
`define hb   32'd988  // B5

`define sil   32'd50000000 // silence

module sound (
	input [11:0] ibeatNum,
	input en,
	output reg [31:0] toneL,
    output reg [31:0] toneR
);

    always @* begin
        if(en == 1) begin
            case(ibeatNum)
                // --- Measure 1 ---
                12'd0: toneR = `hb;      12'd1: toneR = `hb;
                12'd2: toneR = `hb;      12'd3: toneR = `hb;
                12'd4: toneR = `hb;      12'd5: toneR = `hb;
                12'd6: toneR = `hb;      12'd7: toneR = `hb;
                12'd8: toneR = `sil;      12'd9: toneR = `sil;
                12'd10: toneR = `sil;     12'd11: toneR = `sil;
                12'd12: toneR = `sil;     12'd13: toneR = `sil;
                12'd14: toneR = `sil;     12'd15: toneR = `sil;

                12'd16: toneR = `hb;     12'd17: toneR = `hb;
                12'd18: toneR = `hb;     12'd19: toneR = `hb;
                12'd20: toneR = `hb;     12'd21: toneR = `hb;
                12'd22: toneR = `hb;     12'd23: toneR = `hb;
                12'd24: toneR = `sil;     12'd25: toneR = `sil;
                12'd26: toneR = `sil;     12'd27: toneR = `sil;
                12'd28: toneR = `sil;     12'd29: toneR = `sil;
                12'd30: toneR = `sil;

                default: toneR = `sil;
            endcase
        end else begin
            toneR = `sil;
        end
    end

    always @(*) begin
        if(en == 1)begin
            case(ibeatNum)
                12'd0: toneL = `hb;  	12'd1: toneL = `hb;
                12'd2: toneL = `hb;  	12'd3: toneL = `hb;
                12'd4: toneL = `hb;	    12'd5: toneL = `hb;
                12'd6: toneL = `hb;  	12'd7: toneL = `hb;
                12'd8: toneL = `sil;	12'd9: toneL = `sil;
                12'd10: toneL = `sil;	12'd11: toneL = `sil;
                12'd12: toneL = `sil;	12'd13: toneL = `sil;
                12'd14: toneL = `sil;	12'd15: toneL = `sil;

                12'd16: toneL = `hb;     12'd17: toneL = `hb;
                12'd18: toneL = `hb;     12'd19: toneL = `hb;
                12'd20: toneL = `hb;     12'd21: toneL = `hb;
                12'd22: toneL = `hb;     12'd23: toneL = `hb;
                12'd24: toneL = `sil;     12'd25: toneL = `sil;
                12'd26: toneL = `sil;     12'd27: toneL = `sil;
                12'd28: toneL = `sil;     12'd29: toneL = `sil;
                12'd30: toneL = `sil;

                default : toneL = `sil;
            endcase
        end
        else begin
            toneL = `sil;
        end
    end
endmodule

module note_gen(
    clk, // clock from crystal
    rst, // active high reset
    note_div_left, // div for note generation
    note_div_right,
    audio_left,
    audio_right
);
    input clk; // clock from crystal
    input rst; // active low reset
    input [21:0] note_div_left, note_div_right; // div for note generation
    output [15:0] audio_left, audio_right;

    reg [21:0] clk_cnt_next, clk_cnt;
    reg [21:0] clk_cnt_2_next, clk_cnt_2;
    reg b_clk, b_clk_next;
    reg c_clk, c_clk_next;

    always @(posedge clk or posedge rst)
        if (rst == 1'b1)
            begin
                clk_cnt <= 22'd0;
                clk_cnt_2 <= 22'd0;
                b_clk <= 1'b0;
                c_clk <= 1'b0;
            end
        else
            begin
                clk_cnt <= clk_cnt_next;
                clk_cnt_2 <= clk_cnt_2_next;
                b_clk <= b_clk_next;
                c_clk <= c_clk_next;
            end
    
    always @*
        if (clk_cnt == note_div_left)
            begin
                clk_cnt_next = 22'd0;
                b_clk_next = ~b_clk;
            end
        else
            begin
                clk_cnt_next = clk_cnt + 1'b1;
                b_clk_next = b_clk;
            end

    always @*
        if (clk_cnt_2 == note_div_right)
            begin
                clk_cnt_2_next = 22'd0;
                c_clk_next = ~c_clk;
            end
        else
            begin
                clk_cnt_2_next = clk_cnt_2 + 1'b1;
                c_clk_next = c_clk;
            end

    assign audio_left = (note_div_left == 22'd1) ? 16'h0000 : 
                                (b_clk == 1'b0) ? 16'hEEE0 : 16'h0020;
    assign audio_right = (note_div_right == 22'd1) ? 16'h0000 : 
                                (c_clk == 1'b0) ? 16'hEEE0 : 16'h0020;
endmodule

module player_control (
	input clk, 
	input reset, 
	input en, 
	output reg [11:0] ibeat
);
	parameter LEN = 30;
    reg [11:0] next_ibeat;

	always @(posedge clk, posedge reset) begin
		if (reset || !en) begin
			ibeat <= 0;
		end else begin
			if(en)begin
				ibeat <= next_ibeat;
			end
		end
	end

    always @* begin
        next_ibeat = (ibeat + 1 < LEN) ? (ibeat + 1) : 0;
    end

endmodule

module sonic_top(clk, rst, Echo, Trig, distance);
	input clk, rst, Echo;
	output Trig;
    output [19:0] distance;

	wire [19:0] dis;
    wire clk1M;
	wire clk_2_17;

    assign distance = dis;

    div clk1(clk, clk1M);   // clock divider for 1us clock (10^-6 second)
	TrigSignal u1(.clk(clk), .rst(rst), .trig(Trig));   // every 1.000 the signal is '1' but '0' for 9.999.000 (one pulse for 10us clock for 100MHz)
	PosCounter u2(.clk(clk1M), .rst(rst), .echo(Echo), .distance_count(dis));   // count the distance every 1us second
 
endmodule

module PosCounter(clk, rst, echo, distance_count); 
    input clk, rst, echo;
    output[19:0] distance_count;

    parameter S0 = 2'b00;
    parameter S1 = 2'b01; 
    parameter S2 = 2'b10;
    
    wire start, finish;
    reg[1:0] curr_state, next_state;
    reg echo_reg1, echo_reg2;
    reg[19:0] count, distance_register;
    wire[19:0] distance_count; 

    always@(posedge clk) begin
        if(rst) begin
            echo_reg1 <= 0;
            echo_reg2 <= 0;
            count <= 0;
            distance_register  <= 0;
            curr_state <= S0;
        end
        else begin
            echo_reg1 <= echo;   
            echo_reg2 <= echo_reg1; 
            case(curr_state)
                S0:begin
                    if (start) curr_state <= next_state; //S1
                    else count <= 0;
                end
                S1:begin
                    if (finish) curr_state <= next_state; //S2
                    else count <= count + 1;
                end
                S2:begin
                    distance_register <= count;
                    count <= 0;
                    curr_state <= next_state; //S0
                end
            endcase
        end
    end

    always @(*) begin
        case(curr_state)
            S0:next_state = S1;
            S1:next_state = S2;
            S2:next_state = S0;
            default:next_state = S0;
        endcase
    end

    assign start = echo_reg1 & ~echo_reg2;  // start receiving the reflected signal
    assign finish = ~echo_reg1 & echo_reg2; // no more reflected signal

    // TODO: trace the code and calculate the distance, output it to <distance_count>
    assign distance_count = distance_register >> 6;
    
endmodule

// send trigger signal to sensor
module TrigSignal(clk, rst, trig);
    input clk, rst;
    output trig;

    reg trig, next_trig;
    reg[23:0] count, next_count;

    always @(posedge clk, posedge rst) begin
        if (rst) begin
            count <= 0;
            trig <= 0;
        end
        else begin
            count <= next_count;
            trig <= next_trig;
        end
    end
    // count 10us to set trig high and wait for 100ms
    always @(*) begin
        next_trig = trig;
        next_count = count + 1;
        if(count == 999)
            next_trig = 0;
        else if(count == 24'd9999999) begin
            next_trig = 1;
            next_count = 0;
        end
    end
endmodule

// clock divider for T = 1us clock
module div(clk, out_clk);
    input clk;
    output out_clk;
    reg out_clk;
    reg [6:0]cnt;
    
    always @(posedge clk) begin   
        if(cnt < 7'd50) begin
            cnt <= cnt + 1'b1;
            out_clk <= 1'b1;
        end 
        else if(cnt < 7'd100) begin
	        cnt <= cnt + 1'b1;
	        out_clk <= 1'b0;
        end
        else if(cnt == 7'd100) begin
            cnt <= 0;
            out_clk <= 1'b1;
        end
    end
endmodule

module speaker_control(
    clk,  // clock from the crystal
    rst,  // active high reset
    audio_in_left, // left channel audio data input
    audio_in_right, // right channel audio data input
    audio_mclk, // master clock
    audio_lrck, // left-right clock, Word Select clock, or sample rate clock
    audio_sck, // serial clock
    audio_sdin // serial audio data input
);
    input clk;  // clock from the crystal
    input rst;  // active high reset
    input [15:0] audio_in_left; // left channel audio data input
    input [15:0] audio_in_right; // right channel audio data input
    output audio_mclk; // master clock
    output audio_lrck; // left-right clock
    output audio_sck; // serial clock
    output audio_sdin; // serial audio data input
    reg audio_sdin;

    wire [8:0] clk_cnt_next;
    reg [8:0] clk_cnt;
    reg [15:0] audio_left, audio_right;

    assign clk_cnt_next = clk_cnt + 1'b1;

    always @(posedge clk or posedge rst)
        if (rst == 1'b1)
            clk_cnt <= 9'd0;
        else
            clk_cnt <= clk_cnt_next;

    assign audio_mclk = clk_cnt[1];
    assign audio_lrck = clk_cnt[8];
    assign audio_sck = 1'b1; // use internal serial clock mode

    always @(posedge clk_cnt[8] or posedge rst)
        if (rst == 1'b1)
            begin
                audio_left <= 16'd0;
                audio_right <= 16'd0;
            end
        else
            begin
                audio_left <= audio_in_left;
                audio_right <= audio_in_right;
            end

    always @*
        case (clk_cnt[8:4])
            5'b00000: audio_sdin = audio_right[0];
            5'b00001: audio_sdin = audio_left[15];
            5'b00010: audio_sdin = audio_left[14];
            5'b00011: audio_sdin = audio_left[13];
            5'b00100: audio_sdin = audio_left[12];
            5'b00101: audio_sdin = audio_left[11];
            5'b00110: audio_sdin = audio_left[10];
            5'b00111: audio_sdin = audio_left[9];
            5'b01000: audio_sdin = audio_left[8];
            5'b01001: audio_sdin = audio_left[7];
            5'b01010: audio_sdin = audio_left[6];
            5'b01011: audio_sdin = audio_left[5];
            5'b01100: audio_sdin = audio_left[4];
            5'b01101: audio_sdin = audio_left[3];
            5'b01110: audio_sdin = audio_left[2];
            5'b01111: audio_sdin = audio_left[1];
            5'b10000: audio_sdin = audio_left[0];
            5'b10001: audio_sdin = audio_right[15];
            5'b10010: audio_sdin = audio_right[14];
            5'b10011: audio_sdin = audio_right[13];
            5'b10100: audio_sdin = audio_right[12];
            5'b10101: audio_sdin = audio_right[11];
            5'b10110: audio_sdin = audio_right[10];
            5'b10111: audio_sdin = audio_right[9];
            5'b11000: audio_sdin = audio_right[8];
            5'b11001: audio_sdin = audio_right[7];
            5'b11010: audio_sdin = audio_right[6];
            5'b11011: audio_sdin = audio_right[5];
            5'b11100: audio_sdin = audio_right[4];
            5'b11101: audio_sdin = audio_right[3];
            5'b11110: audio_sdin = audio_right[2];
            5'b11111: audio_sdin = audio_right[1];
            default: audio_sdin = 1'b0;
        endcase

endmodule

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
    assign clk_div = pause ? 0 : num[n];
endmodule

module clock_divider_rotate #(parameter n = 26)(
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

    assign next_num = (num != 300_000_000) ? (num + 1) : 0;
    assign clk_div = pause ? 0 : (num < 150_000_000) ? 0 : 1;
endmodule

module clock_divider_turn #(parameter n = 26)(
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

    assign next_num = (num != 100_000_000) ? (num + 1) : 0;
    assign clk_div = pause ? 0 : (num < 50_000_000) ? 0 : 1;
endmodule

module clock_divider_u_turn #(parameter n = 27)(
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
    assign next_num = (num != 160_000_000) ? (num + 1) : 0;
    assign clk_div = pause ? 0 : (num < 80_000_000) ? 0 : 1;
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

    output audio_mclk, 
    output audio_lrck, 
    output audio_sck, 
    output audio_sdin,

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
    reg play_sound;
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
    wire clk21;

    wire [15:0] audio_in_left, audio_in_right; // Internal Signal
    wire [11:0] ibeatNum;               // Beat counter
    wire [31:0] freqL, freqR;           // Raw frequency
    wire [21:0] freq_outL, freq_outR;    // Processed frequency, adapted to the clock rate of Basys3

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
        .Sense_(IRSenseL),
        .obstacles_(IRSignL)
    );

    IRSensor F(
        .Sense_(IRSenseR),
        .obstacles_(IRSignR)
    );

    IRSensor E1(
        .Sense_(IRSenseBL),
        .obstacles_(IRSignBL)
    );

    IRSensor F1(
        .Sense_(IRSenseBR),
        .obstacles_(IRSignBR)
    );

    clock_divider_turn #(.n(26)) G ( // turn_duration
        .clk(clk),
        .en(clk_turn_en), // add distance pause
        .pause(pause),  // pause the duration from keep counting if there's obstacle
        .clk_div(clk_turn)
    );

    clock_divider_u_turn #(.n(27)) H ( // u_turn_duration
        .clk(clk),
        .en(clk_u_turn_en), // add distance pause
        .pause(pause),  // pause the duration from keep counting if there's obstacle
        .clk_div(clk_u_turn)
    );

    clock_divider_rotate #(.n(28)) I ( // rotate_duration
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

    clock_divider #(.n(22)) K ( // speaker
        .clk(clk),
        .en(play_sound),
        .pause(0),
        .clk_div(clk21)
    );

    player_control #(.LEN(30)) L ( 
        .clk(clk21),
        .reset(rst),
        .en(play_sound),
        .ibeat(ibeatNum)
    );

    assign freq_outL = 50000000 / freqL;
    assign freq_outR = 50000000 / freqR;

    note_gen M(
        .clk(clk), 
        .rst(rst), 
        .note_div_left(freq_outL), 
        .note_div_right(freq_outR), 
        .audio_left(audio_in_left),     // left sound audio
        .audio_right(audio_in_right)    // right sound audio
    );

    speaker_control N(
        .clk(clk), 
        .rst(rst), 
        .audio_in_left(audio_in_left),      // left channel audio data input
        .audio_in_right(audio_in_right),    // right channel audio data input
        .audio_mclk(audio_mclk),            // master clock
        .audio_lrck(audio_lrck),            // left-right clock
        .audio_sck(audio_sck),              // serial clock
        .audio_sdin(audio_sdin)             // serial audio data input
    );

    sound O (
        .ibeatNum(ibeatNum),
        .en(play_sound),
        .toneL(freqL),
        .toneR(freqR)
    );

    assign led = {mode, (!clk_blink && clk_blink_en) ? 11'b1111_1111_111 : 11'd0, type};
    assign pause = (distance < 20) ? 1'b1 : 1'b0;
    assign pause1 = (IRSignBR) ? 1'b1 : 1'b0;

    assign num0 = RxData % 10;
    assign num1 = (RxData/10) % 10;
    assign num2 = (RxData/100) % 10;
    assign num3 = (RxData/1000) % 10;

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
            play_sound <= 0;
            turnControl <= 0;
            clk_blink_en <= 0;
        end else begin
            mode <= 3'b000;
            clk_blink_en <= 0;
            play_sound <= 0;
            case(type)
                2'b01 : begin
                    if((distance > 15) && (distance < 60) && IRSignL && IRSignR) begin
                        mode <= 3'b011; // forward
                    end else if(((distance > 15) && (distance < 60) && IRSignL) || (IRSignL && distance > 15)) begin
                        mode <= 3'b010; // left
                    end else if(((distance > 15) && (distance < 60) && IRSignR) || (IRSignR && distance > 15)) begin
                        mode <= 3'b001; // right
                    end else begin
                        mode <= 3'b000; // stop
                    end
                end
                2'b10 : begin
                    if(RxData == 111) begin  // forward
                        mode <= 3'b011;  // forward
                        sevenSeg <= 1;
                        turnControl <= 0;
                        clk_turn_en <= 0;
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                    end else if(RxData == 247) begin    // turn left          
                        clk_turn_en <= 1;        // start turn duration
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b010;  // left
                        end else begin
                            mode <= 3'b000; //stop
                        end
                        if(clk_turn) begin
                            clk_turn_en <= 0;
                            turnControl <= 1;       // to turn or not
                        end
                    end else if(RxData == 186) begin    // turn right
                        clk_turn_en <= 1;        // start turn duration
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b001;  // right
                        end else begin
                            mode <= 3'b000; //stop
                        end
                        if(clk_turn) begin
                            clk_turn_en <= 0;
                            turnControl <= 1;       // to turn or not
                        end
                    end else if(RxData == 251) begin // backward
                        mode <= 3'b100;  // backward 
                        sevenSeg <= 1;
                        turnControl <= 0;
                        clk_turn_en <= 0;
                        clk_u_turn_en <= 0;
                        clk_rotate_en <= 0;
                        if(pause1) begin
                            mode <= 3'b000;
                            clk_blink_en <= 1;
                            play_sound <= 1;
                        end
                    end else if(RxData == 183) begin // u-turn
                        clk_turn_en <= 0;        // start turn duration
                        clk_u_turn_en <= 1;     // start u-turn duration
                        clk_rotate_en <= 0;
                        sevenSeg <= 1;      // show BCD
                        if(turnControl == 0) begin
                            mode <= 3'b010;  // left
                        end else begin
                            mode <= 3'b000; //stop
                        end
                        if(clk_u_turn) begin
                            clk_u_turn_en <= 1'b0;
                            turnControl <= 1'b1;       // to turn or not
                        end
                    end else if(RxData == 105) begin    // rotate left  hi
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
                    end else if(RxData == 217) begin    // rotate right goodbye
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
                    if(pause || pause1) begin
                        if(mode == 3'b011) begin
                            if(!pause1 && pause) begin
                                mode <= 3'b100; // backward
                            end else begin
                                mode <= 3'b000;
                            end
                        end else if(mode == 3'b100)begin          
                            if(pause1) begin
                                mode <= 3'b000;
                            end 
                        end
                        clk_blink_en <= 1;
                        play_sound <= 1;
                    end
                end
            endcase
        end
    end

endmodule
