
module Program_Counter #(parameter size = 32)(
    clk,
    rst,
    PC_in,
    PC_out,
    stall
);

  //I/O ports
  input clk;
  input rst;
  input [size-1:0] PC_in;
  output reg [size-1:0] PC_out;
  input stall;

  //Internal Signals
  reg [size-1:0] current_PC;
  //assertion:
  // assert property(@(posedge clk) rst && (PC_out ==`PC_RESET)) begin
  // end
  //Main function

  // always @(posedge clk or posedge rst) begin
  //   if (rst) current_PC <= `PC_RESET;
  //   else current_PC <= PC_in;
  // end

  always @(posedge clk or posedge rst) begin
    if (rst) PC_out <= `PC_RESET;
    else if (!stall) PC_out <= PC_in;
  end

endmodule
