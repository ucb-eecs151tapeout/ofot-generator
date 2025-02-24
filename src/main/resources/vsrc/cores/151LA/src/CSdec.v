module CSdec #(
    parameter size = 32
) (
    //input
    instr_X,
    instr_M,
    BrEq,
    BrLT,
    rst,

    //output
    fw_RS1, 
    fw_RS2,
    BrUn,
    ImmSel,
    A_sel,
    B_sel,
    MemRW,
    RegWEn,
    WBSel,
    PC_sel,
    NOP,
    CSR_RI,
);

  //I/O ports
  input [size-1:0] instr_X;
  input [size-1:0] instr_M;
  input BrEq, BrLT;
  input rst;
  
  output fw_RS1;
  output fw_RS2;
  output BrUn;
  output [2:0] ImmSel;
  output A_sel;
  output B_sel;
  output MemRW;
  output RegWEn;
  output [1:0] WBSel;
  output [1:0] PC_sel;
  output NOP;
  output [1:0] CSR_RI;

  //Internal Signals
  wire [6:0] instr_op_X, instr_op_M;
  wire [2:0] instr_func3_X;
  wire [5:0] instr_rd_M;
  wire [5:0] instr_rs1_X, instr_rs2_X;

  //Main function
  assign instr_op_X = instr_X[6:0];
  assign instr_op_M = instr_M[6:0];
  assign instr_func3_X = instr_X[14:12];
  assign instr_rd_M = instr_M[11:7];
  assign instr_rs1_X = instr_X[19:15];
  assign instr_rs2_X = instr_X[24:20];

  //NOP and PC_sel

  //NOP = 1 at M stage when br taken or jump
  assign NOP = ((instr_op_X == (`OPC_BRANCH)) && ((BrEq && instr_func3_X == `FNC_BEQ) || (~BrEq && instr_func3_X == `FNC_BNE) || (BrLT && (instr_func3_X == `FNC_BLT || instr_func3_X == `FNC_BLTU)) || (~BrLT && (instr_func3_X == `FNC_BGE || instr_func3_X == `FNC_BGEU)))) ? 1 :
               (instr_op_X == (`OPC_JAL) || instr_op_X == (`OPC_JALR) ) ? 1 : 0;

  //PCSel = 1 at M stage when br taken or jump
  assign PC_sel = (rst) ? 2 :
                  (instr_op_X == (`OPC_JAL) || instr_op_X == (`OPC_JALR) ) ? 1 : 
                  ((instr_op_X == (`OPC_BRANCH)) && ((BrEq && instr_func3_X == `FNC_BEQ) || (~BrEq && instr_func3_X == `FNC_BNE) || (BrLT && (instr_func3_X == `FNC_BLT || instr_func3_X == `FNC_BLTU)) || (~BrLT && (instr_func3_X == `FNC_BGE || instr_func3_X == `FNC_BGEU)))) ? 1 : 0 ;

  //RegWEn at M is 1 when R-type, I-type, lw, jalr, jal
  assign RegWEn = ((instr_op_X == `OPC_BRANCH) || (instr_op_X == `OPC_STORE) || (instr_X == `INSTR_NOP)) ? 0 : 1;

  //MemRW at X is 1 when sw
  assign MemRW = (instr_op_X == `OPC_STORE);

  //WBSel at M is 1(ALU) when R-type, I-type, auipc, lui; 2(PC+4) when jalr, jal
  assign WBSel = ((instr_op_X == `OPC_ARI_RTYPE) || (instr_op_X == `OPC_ARI_ITYPE) || (instr_op_X == `OPC_AUIPC) || (instr_op_X == `OPC_LUI)) ? 1 : 
                   ((instr_op_X == `OPC_JAL) || (instr_op_X == `OPC_JALR)) ? 2 : 0;

  //fw_RS1 is 1 when previous WBrd == current rs1 
  assign fw_RS1 = (instr_rd_M == instr_rs1_X) && ((instr_op_M != `OPC_BRANCH) && (instr_op_M != `OPC_STORE)) && (instr_rs1_X != 0);

  //fw_RS2 is 1 when previous WBrd == current rs1 
  assign fw_RS2 = (instr_rd_M == instr_rs2_X) && ((instr_op_M != `OPC_BRANCH) && (instr_op_M != `OPC_STORE)) && (instr_rs2_X != 0);

  //ImmSel at X 
  assign ImmSel = ((instr_op_X == `OPC_JAL)) ? `Imm_J :
                  (instr_op_X == `OPC_JALR) ? `Imm_JR :
                  ((instr_op_X == `OPC_ARI_ITYPE) || (instr_op_X == `OPC_LOAD)) ? `Imm_I :
                  (instr_op_X == `OPC_STORE) ? `Imm_S :
                  (instr_op_X == `OPC_BRANCH) ? `Imm_B :
                  ((instr_op_X == `OPC_AUIPC) || (instr_op_X == `OPC_LUI)) ? `Imm_U : 
                  (instr_op_X == `OPC_CSR) ? `Imm_csr : 0;

  //BrUn at X is 1 when bltu
  assign BrUn = ((instr_op_X == `OPC_BRANCH) && ((instr_func3_X == `FNC_BGEU) || (instr_func3_X == `FNC_BLTU)));

  //A_sel at X
  assign A_sel = (instr_op_X == `OPC_BRANCH) || (instr_op_X == `OPC_JAL) || (instr_op_X == `OPC_AUIPC);
  
  //B_sel at X is 0 iff r-type
  assign B_sel = (instr_op_X == `OPC_ARI_RTYPE) ? 0: 1;

  //CSR st X
  assign CSR_RI = ((instr_func3_X == `FNC_RWI) && (instr_op_X == `OPC_CSR)) ? 2 : //csrwi
                  ((instr_func3_X == `FNC_RW) && (instr_op_X == `OPC_CSR)) ? 1 : 0; //csrw


endmodule
