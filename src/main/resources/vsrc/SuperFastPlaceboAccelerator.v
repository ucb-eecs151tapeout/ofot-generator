module SuperFastPlaceboAccelerator(
    input [31:0] A,B,
    input [3:0] ALUop,
    output reg [31:0] Out
);

always @(*) begin
    case (ALUop)
        default: Out = 0;
    endcase
end

endmodule