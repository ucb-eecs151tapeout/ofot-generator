module reg_file #(
    parameter NUM_REGS = 32, 
    parameter ADDR_WIDTH = 5
) (
    input WEn,         
    input clk,         
    input reset,      
    input [ADDR_WIDTH-1:0] addr1, 
    input [ADDR_WIDTH-1:0] addr2,
    //Write back data
    input [ADDR_WIDTH-1:0] write_addr, 
    input [31:0] d_in,  
    //rs1 and rs2
    output [31:0] rs1,
    output [31:0] rs2
);

//NUM_REGS total 32 bit registers
reg [31:0] registers [NUM_REGS-1:0];
genvar i;

//Reset Registers
generate
    for (i = 0; i < NUM_REGS; i = i + 1) begin
        always @(posedge clk or posedge reset) begin
            if (reset) begin
                registers[i] <= 32'b0;
            end
        end
    end
endgenerate

//Write to registers
always @(posedge clk) begin
    if (!reset) begin
        if (write_addr != 0 && WEn) begin
            registers[write_addr] <= d_in;
        end
    end
end

//Assign rs1 and rs2
assign rs1 = registers[addr1];
assign rs2 = registers[addr2];
endmodule