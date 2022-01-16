module IRSensor (
    input clk,
    input rst,
    input Sense_,
    // output reg obstacles_
    output wire obstacles_
);
    reg [19:0] counter;
    // always@(posedge clk, posedge rst)begin
    //   if(!rst)begin
    //     counter <= 0;
    //     obstacles_ <= 0;
    //   end else begin
    //     if(counter < 5000)begin
    //       counter <= counter + 1;
    //       obstacles_ <= Sense_;
    //     end else begin
    //       counter <= 0;
    //     end
    //   end
    // end
    assign obstacles_ = !Sense_;

endmodule