module StoreMask #(
    parameter size = 32
) (
    instr_func3_X,
    instr_op_X,
    RS2_data_fw,
    MemWrite_data,
    addr,
    mask
);
    //I/O
    input [2:0] instr_func3_X;
    input [6:0] instr_op_X;
    input [size-1:0] RS2_data_fw;
    input [1:0] addr;
    output [size-1:0] MemWrite_data;
    output [3:0] mask;


    //Main Function
    assign mask = ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 0) ? 4'b0001 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 1) ? 4'b0010 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 2) ? 4'b0100 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 3) ? 4'b1000 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 0) ? 4'b0011 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 1) ? 4'b0011 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 2) ? 4'b1100 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 3) ? 4'b1100 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SW)) ? 4'b1111 : 0;

    assign MemWrite_data = ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB)) ? {4{RS2_data_fw[7:0]}} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 1) ? {24'd0,RS2_data_fw[15:8]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 2) ? {24'd0,RS2_data_fw[23:16]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 3) ? {24'd0,RS2_data_fw[31:24]} :
                           ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH)) ? {2{RS2_data_fw[15:0]}} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 1) ? {16'd0,RS2_data_fw[15:0]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 2) ? {16'd0,RS2_data_fw[31:16]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 3) ? {16'd0,RS2_data_fw[31:16]} :
                           ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SW) ) ? RS2_data_fw : 0;
endmodule