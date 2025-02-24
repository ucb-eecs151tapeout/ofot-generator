module Pipe_Reg #(parameter size = 32)(
  clk,
  rst,
  data_i,
  data_o,
  stall
);

  //I/O ports
  input clk;
  input rst;
  input [size-1:0] data_i;

  output [size-1:0] data_o;
  input stall;

  //Internal Signals
  reg [size-1:0] data_o;

  //Main function
  /*your code here*/
  always @(posedge clk ) begin
    if (rst) data_o <= 0;
    else if (!stall) data_o <= data_i;
  end
  
  

endmodule
