`ifndef RISCV_MONITOR_SV
`define RISCV_MONITOR_SV

class riscv_monitor extends uvm_monitor;
  `uvm_component_utils(riscv_monitor)

  virtual riscv_if vif;

  uvm_analysis_port #(riscv_seq_item) csr_ap; //建立一個分析端口 (analysis port) csr_ap 之後要把觀察到的 riscv_seq_item 傳出去給 scoreboard

  function new(string name = "riscv_monitor", uvm_component parent = null);
    super.new(name, parent);
    csr_ap = new("csr_ap", this); //new 一個 analysis port（csr_ap），名字叫 "csr_ap"
  endfunction

  function void build_phase(uvm_phase phase);
    super.build_phase(phase);

    if (!uvm_config_db#(virtual riscv_if)::get(this, "", "vif", vif)) begin
      `uvm_fatal("MON_GET_VIF", "Failed to get virtual interface vif!")
    end
  endfunction

  task run_phase(uvm_phase phase);
  riscv_seq_item item;
  
  forever begin
    @(posedge vif.clk);
    if (!vif.reset) begin
      if (vif.mem_req_valid && vif.mem_req_data_valid) begin
        item = riscv_seq_item::type_id::create("item");

        // item.mem_req_valid     = vif.mem_req_valid;
        item.mem_req_rw        = vif.mem_req_rw;
        item.mem_req_addr      = vif.mem_req_addr;
        item.mem_req_tag       = vif.mem_req_tag;
        // item.mem_req_data_valid= vif.mem_req_data_valid;
        item.mem_req_data_bits = vif.mem_req_data_bits;
        item.mem_req_data_mask = vif.mem_req_data_mask;
        item.mem_req_tag       = vif.mem_req_tag;
        item.csr               = vif.csr;

        // **只有當有連結才送**
        if (csr_ap.size() > 0) begin
          csr_ap.write(item);
        end

        `uvm_info("MONITOR", $sformatf(
          "Monitor captured: rw=%0d addr=0x%h data=0x%h mask=0x%h tag=0x%h",
          item.mem_req_rw, item.mem_req_addr, item.mem_req_data_bits, item.mem_req_data_mask, item.mem_req_tag
        ), UVM_MEDIUM)
      end
    end
  end
endtask


endclass

`endif
