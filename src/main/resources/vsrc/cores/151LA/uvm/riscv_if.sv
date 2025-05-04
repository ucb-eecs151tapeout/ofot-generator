`ifndef RISCV_IF_SV
`define RISCV_IF_SV

`include "const.vh"

interface riscv_if(input logic clk, input logic reset);

  logic                      mem_req_valid;
  logic                      mem_req_ready;
  logic                      mem_req_rw;
  logic [`MEM_ADDR_BITS-1:0] mem_req_addr;
  logic [`MEM_TAG_BITS-1:0]  mem_req_tag;

  logic                      mem_req_data_valid;
  logic                      mem_req_data_ready;
  logic [`MEM_DATA_BITS-1:0] mem_req_data_bits;
  logic [(`MEM_DATA_BITS/8)-1:0] mem_req_data_mask;

  logic                      mem_resp_valid;
  logic [`MEM_TAG_BITS-1:0]  mem_resp_tag;
  logic [`MEM_DATA_BITS-1:0] mem_resp_data;

  logic [31:0]               csr;

endinterface

`endif

