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
