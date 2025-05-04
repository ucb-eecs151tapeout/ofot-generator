`ifndef RISCV_SCOREBOARD_SV
`define RISCV_SCOREBOARD_SV

class riscv_scoreboard extends uvm_scoreboard;
  `uvm_component_utils(riscv_scoreboard)

  uvm_analysis_export #(riscv_seq_item) csr_export;

  function new(string name = "riscv_scoreboard", uvm_component parent = null);
    super.new(name, parent);
    csr_export = new("csr_export", this);
  endfunction


  function logic [127:0] get_expected_data(logic [31:0] addr);
    case (addr)
      32'h80000000: return 32'h2A; // example expected
      default: return '0;
    endcase
  endfunction

  function void write(riscv_seq_item item);
    logic [127:0] expected = get_expected_data(item.mem_req_addr);

    if (item.mem_req_data_bits === expected) begin
      `uvm_info("SCOREBOARD", $sformatf(
        "[PASS] addr=0x%h data=0x%h",
        item.mem_req_addr, item.mem_req_data_bits
      ), UVM_LOW)
    end else begin
      `uvm_error("SCOREBOARD", $sformatf(
        "[FAIL] addr=0x%h expected=0x%h actual=0x%h",
        item.mem_req_addr, expected, item.mem_req_data_bits
      ))
    end
  endfunction

endclass

`endif
