`ifndef RISCV_SEQ_SV
`define RISCV_SEQ_SV

// import uvm_pkg::*;
// import riscv_seq_item_pkg::*;

class riscv_seq extends uvm_sequence #(riscv_seq_item);
  `uvm_object_utils(riscv_seq)

  int unsigned num_transactions = 10;

  function new(string name = "riscv_seq");
    super.new(name);
  endfunction

  virtual task body();
    riscv_seq_item item;
    for (int i = 0; i < num_transactions; i++) begin
      item = riscv_seq_item::type_id::create("item"); //從factory `create()` 建立一個新的 `riscv_seq_item`，名叫item。
      start_item(item); //告訴 UVM driver：「我要送一筆新的 item，請準備接收！」
          item.mem_req_ready = 1; 
          item.mem_req_data_ready = 1;
          item.mem_resp_valid = 1;
          item.mem_resp_data = 0;
          item.mem_resp_tag = 0;
      finish_item(item);
    end
  endtask

endclass

`endif

// Sequence = 產生 transaction
// Sequencer = 幫 transaction 排隊交給 Driver。

// 在這裡做好item等driver用

