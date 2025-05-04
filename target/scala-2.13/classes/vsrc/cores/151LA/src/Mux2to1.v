module Mux2to1 #(parameter size = 32) (
    data0_i,
    data1_i,
    select_i,
    data_o
);

  //I/O ports
  input [size-1:0] data0_i;
  input [size-1:0] data1_i;
  input select_i;

  output [size-1:0] data_o;

  //Internal Signals
  wire [size-1:0] data_o;

  //Main function
  assign data_o = select_i ? data1_i : data0_i;

endmodule
