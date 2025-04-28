`include "Opcode.vh"
`include "const.vh"

module decoder_logic (
    input [31:0] inst,
    input reset,
    output reg ASel, //reg for I-type, R-type, load & store, J, csr; pc for b type, 0 for alu, 1 for pc
    output reg [2:0] Bsel, //corresponding types, or reg if r type; // 0 if register, else is the correspoding immediate
    
    output reg [1:0] PCSel, // inst = jump || branch && cond is true // 0 if no branching, 1 if branch, 2 if jump
    output reg [1:0] WBSel, // 0 for pc, 1 for alu, 2 for mem
    output reg [4:0] rd,
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output [2:0] fn3,
    output reg [3:0] memW,
    output reg [3:0] memR,
    output reg [6:0] opcode 
);


assign opcode = reset? 0 : inst[6:0];
assign fn3 = reset? 0 : inst[14:12];
assign rs1 = reset? 0 : inst[19:15];
assign rs2 = reset? 0 : inst[24:20];

//assigning the values of the control signals
always @(*) begin
    if (!reset) begin
        case(opcode)
            `OPC_ARI_ITYPE: begin
                ASel = `REG_ASEL;
                Bsel = `IMM_I_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                memR = 0;
            end
            `OPC_ARI_RTYPE: begin
                ASel = `REG_ASEL;
                Bsel = `REG_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_LOAD: begin
                ASel = `REG_ASEL;
                Bsel = `IMM_I_BSEL; // same format
                WBSel = `MEM_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                case (fn3)
                    `FNC_LB: memR = 4'b0001;
                    `FNC_LH: memR = 4'b0011;
                    `FNC_LW: memR = 4'hf;
                    `FNC_LHU: memR = 4'b0011;
                    `FNC_LBU: memR = 4'b0001;
                endcase
            end

            `OPC_STORE : begin
                ASel = `REG_ASEL;
                Bsel = `IMM_S_BSEL;
                WBSel = `ALU_WB;
                case (fn3)
                    `FNC_SB: memW = 4'b0001;
                    `FNC_SH: memW = 4'b0011;
                    `FNC_SW: memW = 4'hf;
                endcase
                PCSel = 0;
                rd = 0;
                memR = 0;
            end

            `OPC_JAL : begin
                ASel = `PC_ASEL;
                Bsel = `IMM_UJ_BSEL;
                WBSel = `PC_WB;
                memW = 0;
                PCSel = `JUMP_INST;
                rd = inst[11:7];
                memR = 0;
            end
            `OPC_BRANCH : begin
                ASel = `PC_ASEL;
                Bsel = `IMM_B_BSEL;
                WBSel = 0;
                memW = 0;
                PCSel = `COND_BRANCH;
                rd = 0;
                memR = 0;
            end

            `OPC_LUI : begin
                ASel = `REG_ASEL;
                Bsel = `IMM_U_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                memR = 0;
            end
            `OPC_JALR : begin
                ASel = `REG_ASEL;
                Bsel = `IMM_I_BSEL;
                WBSel = `PC_WB;
                memW = 0;
                PCSel = `JUMP_INST;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_CSR: begin
                ASel = `REG_ASEL;
                Bsel = `REG_BSEL;
                WBSel = `CSR_WB;
                memW = 0;
                PCSel = `NO_BRANCH;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_AUIPC: begin
                ASel = `PC_ASEL;
                Bsel = `IMM_U_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = `NO_BRANCH;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_NOOP: begin
                ASel = `REG_ASEL;
                Bsel = `REG_BSEL;
                WBSel = `CSR_WB;
                memW = 0;
                PCSel = `NO_BRANCH;
                rd = 0;
                memR = 0;
            end

        endcase
    end
    else begin
        ASel = 0;
        Bsel = 0;
        WBSel = 0;
        memW = 0;
        PCSel = 0;
        rd = 0;
        memR = 0;
    end
end

endmodule