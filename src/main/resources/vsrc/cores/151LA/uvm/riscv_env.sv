`ifndef RISCV_ENV_SV
`define RISCV_ENV_SV

class riscv_env extends uvm_env;
  `uvm_component_utils(riscv_env)

  riscv_agent agent;
  riscv_scoreboard scoreboard;

  function new(string name = "riscv_env", uvm_component parent = null);
    super.new(name, parent);
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    agent = riscv_agent::type_id::create("agent", this);
    scoreboard = riscv_scoreboard::type_id::create("scoreboard", this);
  endfunction


function void connect_phase(uvm_phase phase);
  super.connect_phase(phase);

  if (agent == null) `uvm_fatal("ENV", "agent is null!")
  if (agent.monitor == null) `uvm_fatal("ENV", "agent.monitor is null!")
  if (scoreboard == null) `uvm_fatal("ENV", "scoreboard is null!")

  // 直接接
  agent.monitor.csr_ap.connect(scoreboard.csr_export);

  `uvm_info("ENV", "Monitor connected to Scoreboard!", UVM_LOW)
endfunction


endclass

`endif
