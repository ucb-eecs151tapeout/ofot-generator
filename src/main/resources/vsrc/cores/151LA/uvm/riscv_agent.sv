`ifndef RISCV_AGENT_SV
`define RISCV_AGENT_SV

class riscv_agent extends uvm_agent;
  `uvm_component_utils(riscv_agent)

  virtual riscv_if vif;

  riscv_driver driver;
  riscv_monitor monitor;
  uvm_sequencer #(riscv_seq_item) sequencer;

  function new(string name = "riscv_agent", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    // Always create monitor
    monitor = riscv_monitor::type_id::create("monitor", this);

    // Create driver + sequencer
    driver = riscv_driver::type_id::create("driver", this);
    sequencer = uvm_sequencer#(riscv_seq_item)::type_id::create("sequencer", this);

    // Connect sequencer ->  (port.connect(export))
    driver.seq_item_port.connect(sequencer.seq_item_export);
  endfunction

  function void connect_phase(uvm_phase phase);
    super.connect_phase(phase);

    // Pass virtual interface to sub-components
    uvm_config_db#(virtual riscv_if)::set(this, "driver", "vif", vif);
    uvm_config_db#(virtual riscv_if)::set(this, "monitor", "vif", vif);
    //在 agent 的 build_phase() 把 vif 存到 config_db，指定 driver和monitor底下都可以拿到
    //之後在 driver / monitor 的 build_phase() 裡可拿到
  endfunction

endclass

`endif
