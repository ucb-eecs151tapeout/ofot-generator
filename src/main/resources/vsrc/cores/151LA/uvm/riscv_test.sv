// `ifndef RISCV_TEST_SV
// `define RISCV_TEST_SV

// class riscv_test extends uvm_test;
//   `uvm_component_utils(riscv_test)

//   virtual riscv_if vif;
//   string loadmem_file;

//   riscv_env env;
//   riscv_seq seq; // <- 宣告在class內（不是run_phase內）

//   function new(string name = "riscv_test", uvm_component parent = null);
//     super.new(name, parent);
//   endfunction

//   function void build_phase(uvm_phase phase);
//     super.build_phase(phase);
    
//     // Get virtual interface
//     if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif))
//       `uvm_fatal("NOVIF", "No virtual interface set for test.")
    
//     // 拿loadmem plusarg
//     if (!$value$plusargs("loadmem=%s", loadmem_file)) begin
//       loadmem_file = "";
//     end

//     env = riscv_env::type_id::create("env", this);
//     seq = riscv_seq::type_id::create("seq"); // 這裡先create好
//   endfunction


//   task run_phase(uvm_phase phase);
//     super.run_phase(phase);
//     phase.raise_objection(this);

//     if (loadmem_file != "") begin
//       `uvm_info("LOADMEM", $sformatf("Loading memory from %s...", loadmem_file), UVM_LOW)

//       // 小技巧: 在 vif 或 memory model array上做 readmemh
//       $readmemh(loadmem_file, vif.mem_storage); 
//       // ⇨ 這個 mem_storage 是你 memory model裡的 array，必須在 interface定義好

//       `uvm_info("LOADMEM", "Memory load completed.", UVM_LOW)
//     end
//     else begin
//       `uvm_warning("LOADMEM", "No memory file specified. Proceeding without loading.")
//     end

//     // 可以reset = 0，啟動CPU（這時memory已經準備好）
//     @(posedge vif.clk);
//     vif.reset <= 0; // 你可以選擇這時候讓CPU開始動

//     phase.drop_objection(this);
//   endtask


// endclass

// `endif


`ifndef RISCV_TEST_SV
`define RISCV_TEST_SV

class riscv_test extends uvm_test;
  `uvm_component_utils(riscv_test)

  riscv_env env;
  riscv_seq seq; // <- 宣告在class內（不是run_phase內）

  function new(string name = "riscv_test", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    env = riscv_env::type_id::create("env", this);
    seq = riscv_seq::type_id::create("seq"); // 這裡先create好
  endfunction

task run_phase(uvm_phase phase);
  super.run_phase(phase);
  phase.raise_objection(this);

  seq.start(env.agent.sequencer);

  // 等一段時間，讓 DUT 有時間送資料
  repeat (10000) @(posedge env.agent.vif.clk);

  phase.drop_objection(this);
endtask

endclass

`endif
