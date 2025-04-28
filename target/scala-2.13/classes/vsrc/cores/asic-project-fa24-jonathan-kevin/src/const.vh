`ifndef CONST
`define CONST

//Memory constants
`define MEM_DATA_BITS 128
`define MEM_TAG_BITS 5
`define MEM_ADDR_BITS 28
`define MEM_DATA_CYCLES 4

//CPU constants
`define CPU_ADDR_BITS 32
`define CPU_INST_BITS 32
`define CPU_DATA_BITS 32
`define CPU_OP_BITS 4
`define CPU_WMASK_BITS 16
`define CPU_TAG_BITS 15

// PC address on reset
`define PC_RESET 32'h00002000

// NOP instruction 
`define INSTR_NOP {12'd0, 5'd0, `FNC_ADD_SUB, 5'd0, `OPC_ARI_ITYPE}

// CSR addresses
`define CSR_TOHOST 12'h51E
`define CSR_HARTID 12'h50B
`define CSR_STATUS 12'h50A

// Control logic signal enums
`define PC_ASEL 1'b0 
`define REG_ASEL 1'b1 
`define REG_BSEL 3'd0 

//Immediate signal enums
`define IMM_S_BSEL 3'd1
`define IMM_I_BSEL 3'd2
`define IMM_U_BSEL 3'd3
`define IMM_UJ_BSEL 3'd4
`define IMM_B_BSEL 3'd5

// Write Back logic signal enums
`define PC_WB 2'd0
`define ALU_WB 2'd1
`define MEM_WB 2'd2
`define CSR_WB 2'd3

// Branching logic signal enums
`define NO_BRANCH 2'd0
`define COND_BRANCH 2'd1
`define JUMP_INST 2'd2


`endif //CONST
