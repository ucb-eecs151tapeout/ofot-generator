module ImmGen #(
    parameter size = 32
) (
    ImmSel,
    instr_i,
    instr_o
);

    input [2:0] ImmSel;
    input [31:7] instr_i;
    output [size-1:0] instr_o;

    assign instr_o = (ImmSel == `Imm_J) ? {{11{instr_i[31]}}, instr_i[31], instr_i[19:12], instr_i[20], instr_i[30:21], 1'b0} :
                     (ImmSel == `Imm_JR) ? {{20{instr_i[31]}}, instr_i[31:20]} :
                     (ImmSel == `Imm_I) ? {{20{instr_i[31]}}, instr_i[31:20]} :
                     (ImmSel == `Imm_U) ? {instr_i[31:12], {12'd0}} :
                     (ImmSel == `Imm_B) ? {{19{instr_i[31]}}, instr_i[31], instr_i[7], instr_i[30:25], instr_i[11:8], 1'b0} :
                     (ImmSel == `Imm_S) ? {{20{instr_i[31]}}, instr_i[31:25], instr_i[11:7]} : 
                     (ImmSel == `Imm_csr) ? {27'd0, instr_i[19:15]} : 0;  

    
endmodule