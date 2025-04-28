`include "Opcode.vh"
module branch_comp (
    input [2:0] funct3,
    input [31:0] data1,
    input [31:0] data2,
    output reg branch
);
reg eq, lt, ltu;
always @(*) begin
    // Branch comparison logic
    eq = data1 == data2;
    lt = $signed(data1) < $signed(data2);
    ltu = (data1 < data2);

    // For each funct3, set the corresponding branch signal
    case (funct3) 
        `FNC_BEQ:  branch = eq;
        `FNC_BNE:  branch = !eq;
        `FNC_BGE:  branch = !lt;
        `FNC_BGEU: branch = !ltu;
        `FNC_BLT:  branch = lt;
        `FNC_BLTU: branch = ltu;
        default:  branch = 1'b0;
    endcase
end

endmodule