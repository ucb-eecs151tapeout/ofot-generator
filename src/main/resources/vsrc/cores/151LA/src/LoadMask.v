module LoadMask #(
    parameter size = 32
) (
    instr_func3_M,
    instr_op_M,
    DMEM_out,
    LoadMask_out,
    imm,
    addr
);
    //I/O
    input [2:0] instr_func3_M;
    input [6:0] instr_op_M;
    input [size-1:0] DMEM_out;
    input [size-1:0] imm;
    input [1:0] addr;
    output [size-1:0] LoadMask_out;


    //Main Function
    assign LoadMask_out = ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 0) ? {{24{DMEM_out[7]}},DMEM_out[7:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 1) ? {{24{DMEM_out[15]}},DMEM_out[15:8]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 2) ? {{24{DMEM_out[23]}},DMEM_out[23:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 3) ? {{24{DMEM_out[31]}},DMEM_out[31:24]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 0) ? {24'd0, DMEM_out[7:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 1) ? {24'd0, DMEM_out[15:8]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 2) ? {24'd0, DMEM_out[23:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 3) ? {24'd0, DMEM_out[31:24]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 0) ? {{16{DMEM_out[15]}},DMEM_out[15:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 1) ? {{16{DMEM_out[15]}},DMEM_out[15:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 2) ? {{16{DMEM_out[31]}},DMEM_out[31:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 3) ? {{16{DMEM_out[31]}},DMEM_out[31:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 0) ? {16'd0, DMEM_out[15:0]} : 
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 1) ? {16'd0, DMEM_out[15:0]} : 
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 2) ? {16'd0, DMEM_out[31:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 3) ? {16'd0, DMEM_out[31:16]} : DMEM_out;

endmodule