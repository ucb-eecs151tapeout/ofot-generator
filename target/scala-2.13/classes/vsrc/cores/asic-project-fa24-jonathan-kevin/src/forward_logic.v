module forward_logic (
    input [31:0] data_in,
    input [4:0] rs1_decode,
    input [4:0] rs2_decode,
    input [4:0] rd, // make sure that this is null when the rd is not used
    output use_forward_on_mux1,
    output use_forward_on_mux2, 
    output [31:0] data_out
);
//if rd is not used, use_forward_on_mux1 and use_forward_on_mux2 should be 0
assign use_forward_on_mux1 = (rd == rs1_decode && rd != 0); //rd can't be 0
assign use_forward_on_mux2 = (rd == rs2_decode && rd != 0);

assign data_out = data_in; //default value

endmodule