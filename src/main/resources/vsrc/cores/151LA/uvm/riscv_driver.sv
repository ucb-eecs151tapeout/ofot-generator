`ifndef RISCV_DRIVER_SV //如果還沒定義 RISCV_DRIVER_SV
`define RISCV_DRIVER_SV //定義 RISCV_DRIVER_SV

class riscv_driver extends uvm_driver #(riscv_seq_item); //繼承自uvm_driver(from UVM Lib)，處理的資料型態是 riscv_seq_item
  `uvm_component_utils(riscv_driver) //register "riscv_driver" to UVM factory, factory records classes' names

  // This is standard code for all components
  function new(string name = "riscv_driver", uvm_component parent = null);
    super.new(name, parent);
  endfunction
  // 定義一個function的名字叫"new" 他有個自己的name 和 parent，然後當這個new被呼叫時
  // 他會執行super.new，也就是把當前的name、parent丟給自己的parent--uvm_driver (super = parent)
  
  //virtual interface
  virtual riscv_if vif; //建立一個virtual interface vif

  function void build_phase(uvm_phase phase); //standard code, uvm_phase: a class in uvm_lib. phase: 想成state
    super.build_phase(phase);
    if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif)) 
    //從 config_db 拿資料(型態為 virtual riscv_if)
    // this: 從"自己(driver)"的 config 往上(parent)找
    // "vif"：key 名字叫做 "vif"
    // vif：找到的結果要存到 vif 這個變數裡
      `uvm_fatal("NOVIF", "Virtual interface must be set for driver") //報錯
  endfunction

  task run_phase(uvm_phase phase);
    super.run_phase(phase);

    forever begin
      riscv_seq_item item; //就只是宣告一個變數 item，型態是 riscv_seq_item (一種class)，但還沒有 new（沒有真正 allocate 記憶體）。
      seq_item_port.get_next_item(item);
      //seq_item_port: uvm內建的port，between seq & driver
      //get_next_item(item): 等待 sequence 送一個 transaction ，存到item 

      // drive data to DUT
      @(posedge vif.clk);
      if (!vif.reset) begin
        // vif.mem_req_valid <= 1;
        vif.mem_req_ready <= 1;
        // vif.mem_req_rw <= item.mem_req_rw;
        // vif.mem_req_addr <= item.mem_req_addr;
        // vif.mem_req_tag <= item.mem_req_tag;
        // vif.mem_req_data_valid <= 1;
        vif.mem_req_data_ready <= 1;
        // vif.mem_req_data_bits <= item.mem_req_data_bits;
        // vif.mem_req_data_mask <= item.mem_req_data_mask;
        vif.mem_resp_valid <= 1;
        vif.mem_resp_tag <= item.mem_resp_tag;
        vif.mem_resp_data <= item.mem_resp_data;
      end
      @(posedge vif.clk);
      // vif.mem_req_valid <= 0;
      // vif.mem_req_data_valid <= 0;
      vif.mem_req_ready <= 0;
      vif.mem_req_data_ready <= 0;
      vif.mem_resp_valid <= 0;

      seq_item_port.item_done(); //告訴 sequence：這一個 transaction（item）已經處理完了，可以給下一筆了
    end
  endtask

endclass

`endif

// new：你自己去泡一杯固定口味的珍奶。

// factory：你有一整本菜單（工廠登記了很多飲料類型）。

// create：你去點餐（create）：「我要一杯珍奶！」，櫃台（factory）根據菜單決定給你什麼。

// build_phase	建立所有 component，做 new，抓 config_db 的東西
// run_phase	主要執行測試行為（driver/monitor真的動作）

