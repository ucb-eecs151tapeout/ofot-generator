// Module: ALUdecoder
// Desc:   Sets the ALU operation
// Inputs: opcode: the top 6 bits of the instruction
//         funct: the funct, in the case of r-type instructions
//         add_rshift_type: selects whether an ADD vs SUB, or an SRA vs SRL
// Outputs: ALUop: Selects the ALU's operation
//

`include "Opcode.vh"
`include "ALUop.vh"

module ALUdec(
  input [6:0]       opcode,
  input [2:0]       funct,
  input             add_rshift_type,
  output reg [3:0]  ALUop
);

  // Implement your ALU decoder here, then delete this comment
  always @* begin
    case(opcode)
    // No operation (kill) : 7'b0000000
    `OPC_NOOP: ALUop = `ALU_XXX;       

    // Special immediate instructions
    `OPC_LUI: ALUop = `ALU_COPY_B;       //7'b0110111
    `OPC_AUIPC: ALUop = `ALU_ADD;      //7'b0010111

    // // Jump instructions
    `OPC_JAL: ALUop = `ALU_ADD;          //7'b1101111
    `OPC_JALR: ALUop = `ALU_ADD;         //7'b1100111

    // // Branch instructions
    `OPC_BRANCH: ALUop = `ALU_ADD;       //7'b1100011

    // // Load and store instructions
    `OPC_STORE: ALUop = `ALU_ADD;       //7'b0100011
    `OPC_LOAD: ALUop = `ALU_ADD;        //7'b0000011

    // Arithmetic instructions
    `OPC_ARI_RTYPE: begin  //7'b0110011
      if (funct == 3'b000) begin
        if(add_rshift_type) ALUop = `ALU_SUB;
        else ALUop = `ALU_ADD;
      end
      else if (funct == 3'b001) ALUop = `ALU_SLL;
      else if (funct == 3'b010) ALUop = `ALU_SLT;
      else if (funct == 3'b011) ALUop = `ALU_SLTU;
      else if (funct == 3'b100) ALUop = `ALU_XOR;
      else if (funct == 3'b101) begin
        if(add_rshift_type) ALUop = `ALU_SRA;
        else ALUop = `ALU_SRL;
      end
      else if (funct == 3'b110) ALUop = `ALU_OR;
      else if (funct == 3'b111) ALUop = `ALU_AND;
      else ALUop = `ALU_XXX;      
    end
    `OPC_ARI_ITYPE: begin  //7'b0010011
      if (funct == 3'b000) ALUop = `ALU_ADD;
      else if (funct == 3'b001) ALUop = `ALU_SLL;
      else if (funct == 3'b010) ALUop = `ALU_SLT;
      else if (funct == 3'b011) ALUop = `ALU_SLTU;
      else if (funct == 3'b100) ALUop = `ALU_XOR;
      else if (funct == 3'b101) begin
        if(add_rshift_type) ALUop = `ALU_SRA;
        else ALUop = `ALU_SRL;
      end
      else if (funct == 3'b110) ALUop = `ALU_OR;
      else if (funct == 3'b111) ALUop = `ALU_AND;
      else ALUop = `ALU_XXX;
    end
    // Control status register
    `OPC_CSR: ALUop = `ALU_ADD;        //7'b1110011
    default: ALUop = `ALU_XXX;
    endcase
  end



endmodule
