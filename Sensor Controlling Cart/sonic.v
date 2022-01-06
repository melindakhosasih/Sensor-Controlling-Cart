// sonic_top is the module to interface with sonic sensors
// clk = 100MHz
// <Trig> and <Echo> should connect to the sensor
// <distance> is the output distance in cm
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