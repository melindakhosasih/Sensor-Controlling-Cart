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
