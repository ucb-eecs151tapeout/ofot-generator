module CSR #(
    parameter size = 32
) (
    clk,
    rst,
    CSR_RI,
    RS1_data,
    Zimm,
    csr_o,
    stall
);

    //I/O

    input clk;
    input rst;
    input [1:0] CSR_RI;
    input [size-1:0] RS1_data;
    input [size-1:0] Zimm;
    output reg [size-1:0] csr_o;
    input stall;

    //Main Function
    always @(posedge clk or posedge rst) begin
        if (rst)
            csr_o <= 0;
        else if (!stall) begin
            case (CSR_RI)
                2'd2: csr_o <= Zimm;
                2'd1: csr_o <= RS1_data;
                default: csr_o <= 0;
            endcase
        end
    end

    // always @(posedge clk or posedge rst) begin
    // $display("At time %0d: clk=%b, rst=%b, CSR_RI=%b, RS1_data=%h, Zimm=%h, csr_o=%h", 
    //          $time, clk, rst, CSR_RI, RS1_data, Zimm, csr_o);
    // end
endmodule