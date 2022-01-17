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
