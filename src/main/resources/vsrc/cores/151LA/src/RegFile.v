module RegFile #(parameter size = 32)(
    //input
    clk,
    rst,
    RS1_addr_i,
    RS2_addr_i,
    RDaddr_i,
    RDdata_i,
    RegWrite_en,
    stall,

    //output
    RS1_data_o,
    RS2_data_o
);
  parameter N = 32; // number of reg


  //I/O ports
  input clk;
  input rst;
  input RegWrite_en;
  input [$clog2(N)-1:0] RS1_addr_i;
  input [$clog2(N)-1:0] RS2_addr_i;
  input [$clog2(N)-1:0] RDaddr_i;
  input [size-1:0] RDdata_i;
  input stall;

  output [size-1:0] RS1_data_o;
  output [size-1:0] RS2_data_o;

  //Internal signals/registers
  reg signed [size-1:0] Reg_File [0:N-1];  //32 word registers

  //assertion
  // assert (Reg_File[0] == 0) 
  //   else $error("Error: x0 != 0");

  //Read the data
  assign RS1_data_o = Reg_File[RS1_addr_i];
  assign RS2_data_o = Reg_File[RS2_addr_i];

  //clk are just for writing
  //Writing data when postive edge clk_i and RegWrite_i was set.
  always @(posedge rst or posedge clk) begin
    if (rst) begin
      Reg_File[0]  <= 0;
      Reg_File[1]  <= 0;
      Reg_File[2]  <= 0;
      Reg_File[3]  <= 0;
      Reg_File[4]  <= 0;
      Reg_File[5]  <= 0;
      Reg_File[6]  <= 0;
      Reg_File[7]  <= 0;
      Reg_File[8]  <= 0;
      Reg_File[9]  <= 0;
      Reg_File[10] <= 0;
      Reg_File[11] <= 0;
      Reg_File[12] <= 0;
      Reg_File[13] <= 0;
      Reg_File[14] <= 0;
      Reg_File[15] <= 0;
      Reg_File[16] <= 0;
      Reg_File[17] <= 0;
      Reg_File[18] <= 0;
      Reg_File[19] <= 0;
      Reg_File[20] <= 0;
      Reg_File[21] <= 0;
      Reg_File[22] <= 0;
      Reg_File[23] <= 0;
      Reg_File[24] <= 0;
      Reg_File[25] <= 0;
      Reg_File[26] <= 0;
      Reg_File[27] <= 0;
      Reg_File[28] <= 0;
      Reg_File[29] <= 0;
      Reg_File[30] <= 0;
      Reg_File[31] <= 0;
    end else if (!stall) begin
      if (RegWrite_en && RDaddr_i != 0) Reg_File[RDaddr_i] <= RDdata_i;
      else Reg_File[RDaddr_i] <= Reg_File[RDaddr_i];
    end
  end

    // assign Reg_File[RDaddr_i] = (rst) ? 0 :
    //                             (RegWrite_en && RDaddr_i != 0) ? RDdata_i : 0;
    

endmodule






