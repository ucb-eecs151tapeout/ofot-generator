`include "const.vh"
`include "Opcode.vh"
`include "ALUop.vh"

module Riscv151(
    input clk,
    input reset,

    // Memory system ports
    output reg [31:0] dcache_addr,
    output reg [31:0] icache_addr,
    output reg [3:0] dcache_we,
    output dcache_re,
    output icache_re,
    output reg [31:0] dcache_din,
    input [31:0] dcache_dout,
    input [31:0] icache_dout,
    input stall,
    output reg [31:0] csr
);

//Program counter and pipeline instruction registers
reg [31:0] queried_pc;
reg [31:0] inst_id;
reg [31:0] inst_ex;


//Decode Stage
reg Asel_id;
reg [2:0] Bsel_id;
reg [31:0] pc_id;
reg [1:0] PCSel_id;
reg [1:0] WBSel_id;
reg [4:0] rd_id;
reg [3:0] memW_id;
reg [3:0] memR_id;   
reg signed [31:0] non_forward_data_id;
reg [2:0] fn3_id;
reg [6:0] opcode_id;


//ALU/Execution
reg signed [31:0] op_1, op_2, rs1_data_ex, rs2_data_ex; // synchronous
reg Asel_ex, add_rshift_type;
reg [2:0] Bsel_ex;
reg [31:0] pc_ex;
reg [1:0] PCSel_ex;
reg [1:0] PCsel_tmp;
reg [1:0] WBSel_ex;
reg [4:0] rd_ex;
reg [3:0] memW_ex;
reg [3:0] memR_ex;   
reg [2:0] fn3_ex;
reg [6:0] opcode_ex;
reg signed [31:0] ALU_out_ex;
reg signed [31:0] adder_out_ex;
reg [31:0] pc_ex_out;



//Memory stage
reg [2:0] Bsel_mem;
reg [31:0] pc_mem, rs2_data_mem;
reg [1:0] WBSel_mem;
reg [4:0] rd_mem;
reg [3:0] memW_mem;
reg [3:0] memR_mem;   
reg [2:0] fn3_mem;
reg [6:0] opcode_mem;

//Writeback stage
reg signed[31:0] ALU_out_mem;
reg WEn_wb;
reg [4:0] rd_wb;


// Registers from imm_gen
reg signed [31:0] s_type;
reg signed [31:0] i_type;
reg signed [31:0] u_type;
reg signed [31:0] uj_type;
reg signed [31:0] b_type;

reg [4:0] rs1;
reg [4:0] rs2;
reg signed [31:0] rs1_data_id;
reg signed [31:0] rs2_data_id;
reg signed [31:0] rs1_data_mem;
reg signed [31:0] ALU_out_wb;

// Registers for forward_logic from_ex
reg signed [31:0] s_type_ex;
reg signed [31:0] i_type_ex;
reg signed [31:0] s_type_mem;
reg signed [31:0] i_type_mem;
reg signed [31:0] data_in_ex;
reg use_forward_on_mux1_ex;
reg use_forward_on_mux2_ex;
reg signed [31:0] forward_data_ex;

// Registers for forward_logic from_mem
reg signed [31:0] data_in_mem;
reg use_forward_on_mux1_mem;
reg use_forward_on_mux2_mem;
reg signed [31:0] forward_data_mem;
reg signed [31:0] ALU_forward_mem; // used for forwarding and writing to ALU_wb_mem

// Registers for forward_logic from_wb
reg [4:0] rs1_decode_wb;
reg [4:0] rs2_decode_wb;
reg use_forward_on_mux1_wb;
reg use_forward_on_mux2_wb;
reg signed [31:0] forward_data_wb;
reg signed [31:0] csr_res_wb;
reg [2:0] funct3_ex; 
reg [2:0] funct3_id; 
reg [31:0] data1_ex; 
reg [31:0] data2_ex; 
reg branch_ex; 
reg wait_branch;
reg inter_stall;

reg [31:0] inst_id_tmp;
reg initialize; // to initialize the pipeline with nop instruction
reg just_stalled;
reg read_stall;

// IF Stage---------------------------------------------------------------------
always @(posedge clk) begin
  just_stalled <= stall;
  if (!just_stalled && stall) queried_pc <= queried_pc - 4;
end

always @(posedge clk) begin
  if (!stall) begin
    if (!inter_stall && PCsel_tmp == 0 && PCSel_id == 0) begin  //No stall and no branch 
      // Reset the pipeline
      read_stall <= 0;
      if (reset) begin 
        queried_pc <= `PC_RESET;
        inst_id <=  `INSTR_NOP;
        initialize <= 0;
        wait_branch <= 0;
        inst_id_tmp <= 0;
        // just_stalled <= 0;
      end else if (|inst_id_tmp) begin
        inst_id <= inst_id_tmp;
        inst_id_tmp <= 0;
        queried_pc <= queried_pc;
      end else if (wait_branch == 1) begin
        pc_id <= queried_pc;
        queried_pc <= queried_pc + 4;
        inst_id <= `INSTR_NOP;
        wait_branch <= 0;
      end else if ( opcode_id == `OPC_LOAD || opcode_id == `OPC_STORE) begin
        queried_pc <= queried_pc;
        inst_id <= `INSTR_NOP;
        read_stall <= 1;
      end else begin
        if (!initialize) begin
          inst_id <=  `INSTR_NOP;
          initialize <= 1;
        end else if (just_stalled) begin
          // queried_pc = queried_pc;
          inst_id <=  `INSTR_NOP;
        end else begin
        // Logic for muxing the next PC value
        //  jump and link register
        pc_id <= queried_pc;
        // branch instruction
        queried_pc <= queried_pc + 4;
        // set the instruction to the output of the instruction cache
        inst_id <= icache_dout;
        // just_stalled <= 0;
        end 
      end

      
    end else if (inter_stall) begin // if the memory stage is stalled, then the pipeline should be stalled
      // if (!just_stalled) begin
      //   just_stalled <= 1;
      //   queried_pc <= queried_pc - 4;
      // end else begin
        queried_pc <= queried_pc;
        // pc_id <= queried_pc;
      // end
    end else begin
      // branch instruction
      if (PCSel_id == 0 )  begin
        // queried_pc <= queried_pc + 4;
        inst_id <= `INSTR_NOP;

        // if the instruction is a jump instruction, set the queried pc to the output of the ALU
        if (opcode_ex == `OPC_JALR) begin
          queried_pc <= adder_out_ex;
          wait_branch <= 1;
        end
        else if (PCSel_ex == `JUMP_INST && PCsel_tmp == `JUMP_INST) begin
          queried_pc <= pc_ex_out;
          wait_branch <= 1;
        end
        else if (PCSel_ex == `JUMP_INST && PCsel_tmp == `COND_BRANCH) begin 
          queried_pc <= ALU_out_ex;
          wait_branch <= 1;
        end
        else queried_pc <= queried_pc + 4;
      
      end else begin  
        inst_id <= `INSTR_NOP;
        pc_id <= queried_pc - 4;
        queried_pc <= queried_pc - 4;
      end

    end
  end
end

// ICache 
always @(*) begin
  icache_addr = queried_pc;
end
// DCache
assign icache_re = !(stall || reset || inter_stall || (opcode_id == `OPC_LOAD || opcode_id == `OPC_STORE));

// ID (Decode) Stage -----------------------------------------------------------
// Decode logic
decoder_logic decoder(
    .inst(inst_id),
    .reset(reset),
    .ASel(Asel_id),
    .Bsel(Bsel_id),
    .PCSel(PCSel_id),
    .WBSel(WBSel_id),
    .rd(rd_id),
    .rs1(rs1),
    .rs2(rs2),
    .fn3(fn3_id),
    .memW(memW_id),
    .memR(memR_id),
    .opcode(opcode_id)
);


// Imm gen
imm_gen generator (
    .inst(inst_id),
    .s_type(s_type),
    .i_type(i_type),
    .u_type(u_type),
    .uj_type(uj_type),
    .b_type(b_type)
);

wire reg_clk = (clk && !inter_stall && !stall); // clock for the register file
reg_file #(
    .NUM_REGS(32),
    .ADDR_WIDTH(5)
) registers (
    .WEn(WEn_wb),     
    .clk(reg_clk),         
    .reset(reset),  
    .addr1(rs1),     
    .addr2(rs2),     
    .write_addr(rd_wb), 
    .d_in(ALU_out_wb),
    .rs1(rs1_data_id),  
    .rs2(rs2_data_id)  
);


// Multiplexing input
forward_logic from_ex (
    .data_in(ALU_out_ex),
    .rs1_decode(rs1),
    .rs2_decode(rs2),
    .rd(rd_ex), 
    .use_forward_on_mux1(use_forward_on_mux1_ex),
    .use_forward_on_mux2(use_forward_on_mux2_ex),
    .data_out(forward_data_ex)
);

forward_logic from_mem (
    .data_in(ALU_forward_mem),
    .rs1_decode(rs1),
    .rs2_decode(rs2),
    .rd(rd_mem),
    .use_forward_on_mux1(use_forward_on_mux1_mem),
    .use_forward_on_mux2(use_forward_on_mux2_mem),
    .data_out(forward_data_mem)
);

forward_logic from_wb (
    .data_in(ALU_out_wb),
    .rs1_decode(rs1),
    .rs2_decode(rs2),
    .rd(rd_wb),
    .use_forward_on_mux1(use_forward_on_mux1_wb),
    .use_forward_on_mux2(use_forward_on_mux2_wb),
    .data_out(forward_data_wb)
);


//Imm gen logic
always @(*) begin
  case (Bsel_id) 
    `REG_BSEL: non_forward_data_id = rs2_data_id;
    `IMM_I_BSEL: non_forward_data_id = i_type;
    `IMM_S_BSEL: non_forward_data_id = s_type;
    `IMM_B_BSEL: non_forward_data_id = b_type;
    `IMM_U_BSEL: non_forward_data_id = u_type;
    `IMM_UJ_BSEL: non_forward_data_id = uj_type;
  endcase
end

wire probe_clk;
assign probe_clk = clk && !inter_stall && !stall; 

//Mux for ALU
always @(posedge clk or posedge reset) begin
  // Reset the pipeline
  if (reset) begin
    op_1 <= 32'b0;
    op_2 <= 32'b0;
    pc_ex <= 32'b0;
    PCsel_tmp <= 2'b0;
    WBSel_ex <= 2'b0;
    rd_ex <= 5'b0;
    memW_ex <= 4'b0;
    memR_ex <= 4'b0;
    opcode_ex <= 0;
    rs2_data_ex <= 0;
    s_type_ex <= 0;
    i_type_ex <= 0;
  end else if (!inter_stall && !stall) begin

  // ALU_A Mux
    if (use_forward_on_mux1_ex) rs1_data_ex <= forward_data_ex;
    else if (!use_forward_on_mux1_ex && use_forward_on_mux1_mem) rs1_data_ex <= forward_data_mem;
    else if (!use_forward_on_mux1_ex && !use_forward_on_mux1_mem && use_forward_on_mux1_wb) rs1_data_ex <= forward_data_wb;
    else rs1_data_ex <= rs1_data_id;

    if (Asel_id == `PC_ASEL) op_1 <= pc_id - 4;
    else if (use_forward_on_mux1_ex) op_1 <= forward_data_ex;
    else if (!use_forward_on_mux1_ex && use_forward_on_mux1_mem) op_1 <= forward_data_mem;
    else if (!use_forward_on_mux1_ex && !use_forward_on_mux1_mem && use_forward_on_mux1_wb) op_1 <= forward_data_wb;
    else op_1 <= rs1_data_id;

  // ALU_B Mux
    if (Bsel_id != `REG_BSEL) op_2 <= non_forward_data_id;
    else if (use_forward_on_mux2_ex) op_2 <= forward_data_ex;
    else if (!use_forward_on_mux2_ex && use_forward_on_mux2_mem) op_2 <= forward_data_mem;
    else if (!use_forward_on_mux2_ex && !use_forward_on_mux2_mem && use_forward_on_mux2_wb) op_2 <= forward_data_wb;
    else op_2 <= rs2_data_id;


  // rs2 Mux
    if (use_forward_on_mux2_ex) rs2_data_ex <= forward_data_ex;
    else if (!use_forward_on_mux2_ex && use_forward_on_mux2_mem) rs2_data_ex <= forward_data_mem;
    else if (!use_forward_on_mux2_ex && !use_forward_on_mux2_mem && use_forward_on_mux2_wb) rs2_data_ex <= forward_data_wb;
    else rs2_data_ex <= rs2_data_id;

  // rs2
    s_type_ex <= s_type;
    i_type_ex <= i_type;
    inst_ex <= inst_id;
    opcode_ex <= opcode_id;
    pc_ex <= pc_id;
    PCsel_tmp <= PCSel_id;
    WBSel_ex <= WBSel_id;
    rd_ex <= rd_id;
    memW_ex <= memW_id;
    memR_ex <= memR_id;
    fn3_ex <= fn3_id;
  end
end

//add_rshift_type logic 
always @(*) begin
    add_rshift_type = ((inst_ex[30] && (fn3_ex == `FNC_ADD_SUB))|| (inst_ex[30] && (fn3_ex == `FNC_SRL_SRA))) ? 1: 0;
    data1_ex = rs1_data_ex;
    data2_ex = rs2_data_ex;
end

// Exexcute Stage ---------------------------------------------------------------------
branch_comp branch_comparator (
   .funct3(fn3_ex), 
   .data1(data1_ex), 
   .data2(data2_ex), 
   .branch(branch_ex)
);

// ALUop decoder
reg [3:0] ALUop;
ALUdec ALUdec1(
  .opcode(opcode_ex),
  .funct(fn3_ex),
  .add_rshift_type(add_rshift_type),
  .ALUop(ALUop)
);

// ALU 
ALU ALU1(
  .A(op_1),
  .B(op_2),
  .ALUop(ALUop),
  .Out(ALU_out_ex),
  .adder(adder_out_ex)
);

//calculating pc related stuff
always @(*) begin
  pc_ex_out = ALU_out_ex;
  if ((PCsel_tmp == 1 && branch_ex) || PCsel_tmp == 2) PCSel_ex = 2;
  else PCSel_ex = 0;
end


// move to the next register
reg [1:0] stall_counter;
reg [31:0] addr;
reg [5:0] shifted_by_read;
reg [5:0] shifted_by_write;
reg [2:0] funct3_mem;
reg [2:0] shift_mask_read;


always @(posedge clk) begin
 if (reset) begin
  ALU_out_mem <= 0;
  rs2_data_mem <= 32'b0;
  pc_mem <= 32'b0;
  WBSel_mem <= 2'b0;
  rd_mem <= 5'b0;
  memW_mem <= 4'b0;
  memR_mem <= 4'b0;
  fn3_mem <= 0;
  opcode_mem <= 0;
  s_type_ex <= 0;
  i_type_ex <= 0;
  shifted_by_read <= 5'b0; 
  shifted_by_write <= 5'b0;
  shift_mask_read <= 0;
  // inter_stall <= 0;
  addr <= 0;
  funct3_mem <= 0;

 end else if (clk && !inter_stall && !stall) begin
  ALU_out_mem <= (WBSel_ex == `PC_WB)? (pc_ex) : ALU_out_ex;
  rs2_data_mem <= rs2_data_ex;
  rs1_data_mem <= op_1;
  s_type_ex <= s_type_mem;
  i_type_ex <= i_type_mem;
  funct3_mem <= funct3_ex;

  if (memR_ex == 15) begin // Read: lw (Load Word)
      shifted_by_read <= 5'b00000;   // Word starts at bit 0
      memR_mem <= memR_ex << 0;
  end else if (memR_ex == 3) begin // Read: lh (Load Halfword)
      shifted_by_read <= {ALU_out_ex[1], 4'b0000}; // Start bit based on bit 1
      memR_mem <= memR_ex << {ALU_out_ex[1], 1'b0};
  end else if (memR_ex == 1) begin // Read: lb (Load Byte)
      shifted_by_read <= {ALU_out_ex[1:0], 3'b000}; // Start bit based on bits [1:0]
      memR_mem <= memR_ex << ALU_out_ex[1:0];
  end else begin
      shifted_by_read <= 5'b0; 
      memR_mem <= 0;
  end

  if (memW_ex == 15) begin // Write: sw (Store Word)
      shifted_by_write <= 5'b00000;  // Word starts at bit 0
      memW_mem <= memW_ex << 0;
  end else if (memW_ex == 3) begin // Write: sh (Store Halfword)
      shifted_by_write <= {ALU_out_ex[1], 4'b0000}; // Start bit based on bit 1
      memW_mem <= memW_ex << {ALU_out_ex[1], 1'b0};
  end else if (memW_ex == 1) begin // Write: sb (Store Byte)
      shifted_by_write <= {ALU_out_ex[1:0], 3'b000}; // Start bit based on bits [1:0]
      memW_mem <= memW_ex << ALU_out_ex[1:0];
  end else begin
      shifted_by_write <= 5'b0;
      memW_mem <= 0;
  end

  pc_mem <= pc_ex;
  WBSel_mem <= WBSel_ex;
  rd_mem <= rd_ex;
  // memW_mem <= memW_ex;
  fn3_mem <= fn3_ex;
  opcode_mem <= opcode_ex;
 end 
end


// Mem Stage ---------------------------------------------------------------------
reg [31:0] load_map, store_map;
reg signed [31:0] tmp_dout1;
reg signed [31:0] tmp_dout2;
reg signed [31:0] tmp_dout3;
reg [31:0] tmp_din1;





assign dcache_re = |memR_mem; // if  any of the memory mask bits are 1, it means that we should be reading
always @(*) begin
  dcache_addr = ALU_out_mem; // address to read from
  dcache_we = memW_mem; // Write enable 

  load_map = {{8{memR_mem[3]}}, {8{memR_mem[2]}}, {8{memR_mem[1]}}, {8{memR_mem[0]}}}; // mask for reading
  store_map = {{8{memW_mem[3]}}, {8{memW_mem[2]}}, {8{memW_mem[1]}}, {8{memW_mem[0]}}}; // mask for reading

  tmp_dout1 = dcache_dout & load_map; // load the data and mask
  tmp_dout2 = tmp_dout1 >> shifted_by_read; // load unsigned

  if (!fn3_mem[2]) begin
    case (memR_mem[3] + memR_mem[2] + memR_mem[1] + memR_mem[0])  // Load unsigned
      4: tmp_dout3 = tmp_dout2; // Load word
      2: tmp_dout3 = $signed({{16{tmp_dout2[15]}} , tmp_dout2[15:0]}); // Load halfword
      1: tmp_dout3 = $signed({{24{tmp_dout2[7]}} , tmp_dout2[7:0]}); // Load byte
    endcase
  end else begin
    tmp_dout3 = tmp_dout2;
  end
  
  if (|memW_mem) begin
    tmp_din1 = (rs2_data_mem << shifted_by_write) & store_map; 
    dcache_din = tmp_din1; // data to write
  end

  ALU_forward_mem = (dcache_re)? tmp_dout3 : ALU_out_mem;
end

always @(negedge clk) begin
  if (!reset)
    inter_stall <= (|memR_mem || |memW_mem) && !inter_stall; // if any of the memory mask bits are 1 and we are not already stalled, 
     //then we should stall
  else inter_stall <= 0;
end

// // if the instruction is a load instruction and the destination register is the same as
// assign load_hazard = (|memR_mem) && ((rd_mem == rs1 && rs1 != 0)|| (rd_mem == rs2 && rs2 != 0));


// WB Stage ---------------------------------------------------------------------
always @(posedge clk) begin
  if (!stall && !inter_stall && !reset) begin
    ALU_out_wb <= ALU_forward_mem;
    csr_res_wb <= (WBSel_mem == `CSR_WB)? rs1_data_mem: 0;
    rd_wb  <= rd_mem;
    WEn_wb <= rd_mem != 0;
  end 
  if (reset) csr_res_wb <= 0;
end

// Set csr
always @(*) begin
  csr = csr_res_wb;
end


endmodule
