
module Rst_Reg #(parameter size = 32)(
    clk,
    rst,
    data_i,
    data_o
);

  //I/O ports
  input clk;
  input rst;
  input [size-1:0] data_i;
  output reg [size-1:0] data_o;

  //Internal Signals
  //Main function

  always @(posedge clk or posedge rst) begin
    if (rst) data_o <= `PC_RESET;
    else data_o <= data_i;
  end

endmodule
