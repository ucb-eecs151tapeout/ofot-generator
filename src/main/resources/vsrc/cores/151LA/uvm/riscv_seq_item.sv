`ifndef RISCV_SEQ_ITEM_SV
`define RISCV_SEQ_ITEM_SV

package riscv_seq_item_pkg;

  import uvm_pkg::*;

  class riscv_seq_item extends uvm_sequence_item;
    `uvm_object_utils(riscv_seq_item)

    // NOT rand, just logic for now
    // logic        mem_req_rw;
    // logic [31:0] mem_req_addr;
    // logic [127:0] mem_req_data_bits;
    // logic [15:0] mem_req_data_mask;
    // logic [4:0]  mem_req_tag;

    logic        mem_req_valid;
    logic        mem_req_ready;
    logic        mem_req_rw;
    logic [`MEM_ADDR_BITS-1:0] mem_req_addr;
    logic [`MEM_TAG_BITS-1:0]  mem_req_tag;
    logic        mem_req_data_valid;
    logic        mem_req_data_ready;
    logic [`MEM_DATA_BITS-1:0] mem_req_data_bits;
    logic [(`MEM_DATA_BITS/8)-1:0] mem_req_data_mask;

    logic                       mem_resp_valid;
    logic [`MEM_TAG_BITS-1:0]   mem_resp_tag;
    logic [`MEM_DATA_BITS-1:0]  mem_resp_data;
    logic [31:0]               csr;

    function new(string name = "riscv_seq_item");
      super.new(name);
    endfunction
  endclass

endpackage : riscv_seq_item_pkg

`endif
