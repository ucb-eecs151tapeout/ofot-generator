`include "const.vh"

module Riscv151(
    input clk,
    input reset,

    // Memory system ports
    output [31:0] dcache_addr,
    output [31:0] icache_addr,
    output [3:0] dcache_we, //store
    output dcache_re, // load
    output icache_re, // always be high
    output [31:0] dcache_din,
    input [31:0] dcache_dout,
    input [31:0] icache_dout,
    input stall, // reg remain the value when stall is 1, do nothing if stall is 0
    output [31:0] csr // parallel to dmem

);

  // Implement your core here, then delete this comment


  //internal signals
  wire [`CPU_ADDR_BITS-1:0] PC_in, PC_out, PC_add_4_X, PC_add_4_M, PC_I;
  wire [`CPU_ADDR_BITS-1:0] ALU_out_X, ALU_out_M;
  wire [`CPU_INST_BITS-1:0] instr_X, instr_M;
  wire [`CPU_INST_BITS-1:0] instr_in;
  wire [`CPU_INST_BITS-1:0] instr_Imm, instr_Imm_M;
  wire [`CPU_DATA_BITS-1:0] RegWrite_data;
  wire [`CPU_DATA_BITS-1:0] LoadMask_out;
  wire [`CPU_DATA_BITS-1:0] RS1_data, RS2_data;  
  wire [`CPU_DATA_BITS-1:0] RS1_data_fw, RS2_data_fw;
  wire [3:0] ALUop;
  wire [`CPU_DATA_BITS-1:0] A_out, B_out;
  wire [`CPU_DATA_BITS-1:0] MemWrite_data;
  wire [31:0] tohost;

  //control signal
  wire [1:0] PC_sel_X, PC_sel_M;
  wire [1:0] PC_sel;
  wire RegWEn, RegWEn_X;
  wire [1:0] WBSel, WBSel_X;
  wire [2:0] ImmSel;
  wire fw_RS1, fw_RS2;
  wire BrEq, BrLT, BrUn;
  wire A_sel, B_sel;
  wire MemRW; //MemRW_X;
  wire NOP;
  wire NOP_X;
  wire [1:0] CSR_RI;
  wire [3:0] dcache_mask;

  //assign wires
  assign dcache_re = (instr_X[6:0] == `OPC_LOAD) ? 1 : 0;
  assign dcache_we = (MemRW) ? dcache_mask : 0;
  assign icache_re = 1;
  assign csr = tohost;
  // assign PC_sel = (reset) ? 2 : PC_sel_M;

  //blocks

  //I
  Mux3to1 #(.size(`CPU_ADDR_BITS)) Mux_PC(
    .data0_i(PC_add_4_X),
    .data1_i(ALU_out_M),
    .data2_i(`PC_RESET),
    .select_i(PC_sel),
    .data_o(PC_in)
  );

  //reset PC
  // Rst_Reg #(.size(`CPU_DATA_BITS)) PC_reset(
  //     .clk(clk),
  //     .rst(reset),
  //     .data_i(PC_in),
  //     .data_o(PC_I)
  // );

  //Pipe_reg between I/X
  
  Program_Counter #(.size(`CPU_ADDR_BITS)) PC(
      .clk(clk),
      .rst(reset),
      .PC_in(PC_in),
      // .PC_in(PC_I),
      .PC_out(PC_out),
      .stall(stall)
  );

  assign icache_addr = PC_in; // imem syn read takes 1 cycle
  // assign icache_addr = PC_I; // imem syn read takes 1 cycle

  assign PC_add_4_X = PC_out + 4;

  //X
  CSdec #(.size(`CPU_INST_BITS)) U_CSdec(
    //input
    .instr_X(instr_X),
    .instr_M(instr_M),
    .BrEq(BrEq),
    .BrLT(BrLT),
    .rst(reset),

    //output
    //Xstage
    .fw_RS1(fw_RS1), 
    .fw_RS2(fw_RS2),
    .BrUn(BrUn),
    .ImmSel(ImmSel),
    .A_sel(A_sel),
    .B_sel(B_sel),
    .MemRW(MemRW),
    .CSR_RI(CSR_RI),

    //Mstage
    .RegWEn(RegWEn_X),
    .WBSel(WBSel_X),
    .PC_sel(PC_sel_X),
    .NOP(NOP_X)
  );

  assign instr_in = (PC_sel != 2) ? icache_dout : 0;

  Mux2to1 #(.size(`CPU_INST_BITS)) Mux_NOP(
    .data0_i(instr_in), 
    .data1_i(`INSTR_NOP),
    .select_i(NOP),
    .data_o(instr_X)
  );

  RegFile #(.size(`CPU_INST_BITS)) Reg_File(
      //input
      .clk(clk),
      .rst(reset),
      .RS1_addr_i(instr_X[19:15]),
      .RS2_addr_i(instr_X[24:20]),
      .RDaddr_i(instr_M[11:7]), // from instr of M stage (but need a cycle)
      .RDdata_i(RegWrite_data),
      .RegWrite_en(RegWEn),
      .stall(stall),

      //output
      .RS1_data_o(RS1_data),
      .RS2_data_o(RS2_data)
  );
  
  ImmGen #(.size(`CPU_INST_BITS)) U_ImmGen(
    .ImmSel(ImmSel),
    .instr_i(instr_X[31:7]),
    .instr_o(instr_Imm)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_fw_RS1(
    .data0_i(RS1_data), 
    // .data1_i(ALU_out_M),
    .data1_i(RegWrite_data),
    .select_i(fw_RS1),
    .data_o(RS1_data_fw)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_fw_RS2(
    .data0_i(RS2_data),
    .data1_i(RegWrite_data),
    .select_i(fw_RS2),
    .data_o(RS2_data_fw)
  );

  BranchComp #(.size(`CPU_DATA_BITS)) U_BranchComp (
    //input
    .RS1_data(RS1_data_fw),
    .RS2_data(RS2_data_fw),
    .BrUn(BrUn),

    //output
    .BrEq(BrEq),
    .BrLT(BrLT)
  );

  ALUdec U_ALUdec(
    .opcode(instr_X[6:0]),
    .funct(instr_X[14:12]),
    .add_rshift_type(instr_X[30]),
    .ALUop(ALUop)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_ALU_A(
    .data0_i(RS1_data_fw),
    .data1_i(PC_out),
    .select_i(A_sel),
    .data_o(A_out)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_ALU_B(
    .data0_i(RS2_data_fw),
    .data1_i(instr_Imm),
    .select_i(B_sel),
    .data_o(B_out)
  ); 

  ALU U_ALU(
    .A(A_out),
    .B(B_out),
    .ALUop(ALUop),
    .Out(ALU_out_X)
  );

  StoreMask #(.size(`CPU_DATA_BITS)) U_StoreMask(
    .instr_func3_X(instr_X[14:12]),
    .instr_op_X(instr_X[6:0]),
    .RS2_data_fw(RS2_data_fw),
    .MemWrite_data(MemWrite_data),
    .mask(dcache_mask),
    .addr(ALU_out_X[1:0])
  );


  //Pipe_reg between X/M

  assign dcache_addr = ALU_out_X; // dmem syn read takes 1 cycle
  assign dcache_din = MemWrite_data;

  CSR #(.size(`CPU_DATA_BITS)) X_M_CSR(
      .clk(clk),
      .rst(reset),
      .CSR_RI(CSR_RI),
      .RS1_data(RS1_data_fw),
      .Zimm(instr_Imm),
      .csr_o(tohost),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_DATA_BITS)) X_M_ALU(
      .clk(clk),
      .rst(reset),
      .data_i(ALU_out_X),
      .data_o(ALU_out_M),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_DATA_BITS)) X_M_PCplus4(
      .clk(clk),
      .rst(reset),
      .data_i(PC_add_4_X),
      .data_o(PC_add_4_M),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_INST_BITS)) X_M_instr(
      .clk(clk),
      .rst(reset),
      .data_i(instr_X),
      .data_o(instr_M),
      .stall(stall)
  );

  Pipe_Reg #(.size(6)) X_M_CS(
      .clk(clk),  
      .rst(1'b0),
      .data_i({RegWEn_X, WBSel_X, PC_sel_X, NOP_X}),
      .data_o({RegWEn, WBSel, PC_sel, NOP}),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_INST_BITS)) X_M_imm(
      .clk(clk),
      .rst(reset),
      .data_i(instr_Imm),
      .data_o(instr_Imm_M),
      .stall(stall)
  );
  //M

  LoadMask #(.size(`CPU_DATA_BITS)) U_LoadMask(
    .instr_func3_M(instr_M[14:12]),
    .instr_op_M(instr_M[6:0]),
    .DMEM_out(dcache_dout),
    .LoadMask_out(LoadMask_out),
    .imm(instr_Imm_M),
    .addr(ALU_out_M[1:0])
  );

  Mux3to1 #(.size(`CPU_DATA_BITS)) Mux_WB(
    .data0_i(LoadMask_out),
    .data1_i(ALU_out_M),
    .data2_i(PC_add_4_M),
    .select_i(WBSel),
    .data_o(RegWrite_data)
  );

  // cache Cache(
  //   .clk(clk), 
  //   .reset(rst),

  //   .cpu_req_valid(cpu_req_valid), //input
  //   .cpu_req_ready(cpu_req_ready), //output // connects to stall signal
  //   .cpu_req_addr(cpu_req_addr), //input
  //   .cpu_req_data(cpu_req_data), //input
  //   .cpu_req_write(cpu_req_write), //input

  //   .cpu_resp_valid(cpu_resp_valid), //output
  //   .cpu_resp_data(cpu_resp_data), //output

  //   .mem_req_valid(mem_req_valid), //output
  //   .mem_req_ready(mem_req_ready), //input
  //   .mem_req_addr(mem_req_addr), //output
  //   .mem_req_rw(mem_req_rw), //output
  //   .mem_req_data_valid(mem_req_data_valid), //output
  //   .mem_req_data_ready(mem_req_data_ready), //input
  //   .mem_req_data_bits(mem_req_data_bits), //output

  //   .mem_req_data_mask(mem_req_data_mask), //output // byte level masking

  //   .mem_resp_valid(mem_resp_valid), //input
  //   .mem_resp_data(mem_resp_data) //input
  // );

endmodule
