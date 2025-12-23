`include "alu.v"
module Top(input [3:0] a,b, output [3:0] sum);
    ALU alu_inst(.a(a),.b(b),.sum(sum));
endmodule