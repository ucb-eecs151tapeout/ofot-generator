module imm_gen (
    input [31:0] inst,
    output [31:0] s_type,
    output [31:0] i_type,
    output [31:0] u_type,
    output [31:0] uj_type,
    output [31:0] b_type
);
    //Defining the extensions
    wire [19:0] extension_s;
    wire [19:0] extension_i;
    wire [19:0]  extension_uj;
    assign extension_s = {20{inst[31]}};
    assign extension_i = {20{inst[31]}};
    assign extension_uj = {12{inst[31]}};
    
    //Assigning the types with extensions
    assign s_type = {extension_s, inst[31:25], inst[11:7]};
    assign i_type = {extension_i, inst[31:20]};
    assign u_type = {inst[31:12], 12'b0};
    assign uj_type = {extension_uj, inst[19:12], inst[20], inst[30:21], 1'b0};
    assign b_type =  {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};

endmodule