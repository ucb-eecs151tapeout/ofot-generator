`timescale 1ns/1ps

`include "riscv_if.sv"  // interface一定要include
`include "uvm_macros.svh"
import uvm_pkg::*;
import riscv_uvm_pkg::*;

module riscv_top_tb;

  logic clk;
  logic reset;

  // interface instance
  riscv_if rif(clk, reset);

  // DUT instance
  riscv_top dut (
    .clk(clk),
    .reset(reset),
    .mem_req_valid(rif.mem_req_valid),
    .mem_req_ready(rif.mem_req_ready),
    .mem_req_rw(rif.mem_req_rw),
    .mem_req_addr(rif.mem_req_addr),
    .mem_req_tag(rif.mem_req_tag),
    .mem_req_data_valid(rif.mem_req_data_valid),
    .mem_req_data_ready(rif.mem_req_data_ready),
    .mem_req_data_bits(rif.mem_req_data_bits),
    .mem_req_data_mask(rif.mem_req_data_mask),
    .mem_resp_valid(rif.mem_resp_valid),
    .mem_resp_tag(rif.mem_resp_tag),
    .mem_resp_data(rif.mem_resp_data),
    .csr(rif.csr)
  );

  //test
// initial begin
//   $display("[TB] Simulation started at time %t", $time);
//   clk = 0;
//   forever #5 clk = ~clk;
// end

// initial begin
//   $display("[TB] Assert reset at time %t", $time);
//   reset = 1;
//   repeat (2) @(posedge clk);
//   reset = 0;
//   $display("[TB] Deassert reset at time %t", $time);
// end

// // 加上這段檢查
// initial begin
//   #1000;
//   $display("[TB] Checkpoint 1000ns reached, simulation still alive");
// end

// // 最後還是有 run_test
// initial begin
//   uvm_config_db#(virtual riscv_if)::set(null, "*", "vif", rif);
//   run_test();
// end


  // Clock generation
  initial begin
    clk = 0;
    forever #5 clk = ~clk;
  end

  // Reset generation
  initial begin
    reset = 1;
    repeat (2) @(posedge clk); // Reset active for 2 cycles
    reset = 0;
  end

  // Load memory
  initial begin
    string loadmem_path;
    if ($value$plusargs("loadmem=%s", loadmem_path)) begin
      $display("[TB] Loading memory from %s", loadmem_path);
      dut.mem.load_memory(loadmem_path);  // call task (推薦方式)
    end
  end

  // UVM environment hook
  initial begin
    uvm_config_db#(virtual riscv_if)::set(null, "*", "vif", rif);
    $display("[TB DEBUG] run_test about to start at time %t", $time);
    run_test();
  end

endmodule
