`ifndef RISCV_UVM_PKG_SV
`define RISCV_UVM_PKG_SV



package riscv_uvm_pkg;
  `include "uvm_macros.svh"
  import uvm_pkg::*;
  import riscv_seq_item_pkg::*;

  // `include "riscv_if.sv"
  // `include "riscv_seq_item.sv"
  `include "riscv_seq.sv"
  `include "riscv_driver.sv"
  `include "riscv_monitor.sv"
  `include "riscv_agent.sv"
  `include "riscv_scoreboard.sv"
  `include "riscv_env.sv"
  `include "riscv_test.sv"

endpackage

`endif
