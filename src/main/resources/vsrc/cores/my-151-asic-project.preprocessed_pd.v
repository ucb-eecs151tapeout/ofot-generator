// Module:  Cache
/* Desc:    

    Direct Mapped Cache - 32-bit, 4 KiB total size, 512 bits line size
    T - 20, I - 6, O - 6
    (NOTE: In the cache the Offset bits are only 4 wide since the 2 LSB are dropped to go from byte granularity to word granularity of the address - Feels wrong...)

  NOTE:
  Think word (32-bit) wise!
  The cache will always receive and send a single word (32-bit) of data to and from the CPU.
  
  BUT! 

  This is not the case between the cache and the memory! 
  Instead the cache sends: 32 bit (TODO: check)
  While the memory sends: 128 bit (TODO: check)

  Additionally the memory address lengths of these transactions between Cache <-> MEM are different!!!

*/

// Inputs:  clk : clk
//          reset: reset
//          ...
// Outputs: ...

`ifndef UTIL
`define UTIL

`define ceilLog2(x) ( \
(x) > 2**30 ? 31 : \
(x) > 2**29 ? 30 : \
(x) > 2**28 ? 29 : \
(x) > 2**27 ? 28 : \
(x) > 2**26 ? 27 : \
(x) > 2**25 ? 26 : \
(x) > 2**24 ? 25 : \
(x) > 2**23 ? 24 : \
(x) > 2**22 ? 23 : \
(x) > 2**21 ? 22 : \
(x) > 2**20 ? 21 : \
(x) > 2**19 ? 20 : \
(x) > 2**18 ? 19 : \
(x) > 2**17 ? 18 : \
(x) > 2**16 ? 17 : \
(x) > 2**15 ? 16 : \
(x) > 2**14 ? 15 : \
(x) > 2**13 ? 14 : \
(x) > 2**12 ? 13 : \
(x) > 2**11 ? 12 : \
(x) > 2**10 ? 11 : \
(x) > 2**9 ? 10 : \
(x) > 2**8 ? 9 : \
(x) > 2**7 ? 8 : \
(x) > 2**6 ? 7 : \
(x) > 2**5 ? 6 : \
(x) > 2**4 ? 5 : \
(x) > 2**3 ? 4 : \
(x) > 2**2 ? 3 : \
(x) > 2**1 ? 2 : \
(x) > 2**0 ? 1 : 0)

`endif // UTIL
`ifndef CONST
`define CONST

`define MEM_DATA_BITS 128
`define MEM_TAG_BITS 5
`define MEM_ADDR_BITS 28
`define MEM_DATA_CYCLES 4

`define CPU_ADDR_BITS 32
`define CPU_INST_BITS 32
`define CPU_DATA_BITS 32
`define CPU_OP_BITS 4
`define CPU_WMASK_BITS 16
`define CPU_TAG_BITS 15

// PC address on reset
`define PC_RESET 32'h00002000

// The NOP instruction
`define INSTR_NOP {12'd0, 5'd0, `FNC_ADD_SUB, 5'd0, `OPC_ARI_ITYPE}

`define CSR_TOHOST 12'h51E
`define CSR_HARTID 12'h50B
`define CSR_STATUS 12'h50A

`endif //CONST

module cache #
(
  parameter LINES = 64,
  parameter CPU_WIDTH = `CPU_INST_BITS, // Default: 32 bits
  
  // is without the bits 1, and 0 of the total address 
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8) // Default: 32 - 2 = 30
)
(
  input clk,
  input reset,

  input                       cpu_req_valid,
  output reg                  cpu_req_ready,

  // NOTE: the bottom two bits are removed because we don't need byte granularity but word granularity. So we drop the 2 LSB because memory is byte addressable and 4 bytes make a word so we can correspondingly drop 2 bits of address since 2^2 = 4. 
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,   // [29:0] - icache_addr[31:2] the 2 LSB are removed! (30 bits)

  input [CPU_WIDTH-1:0]       cpu_req_data,   // [31:0] - Data from cpu to write to both cache and memory 
  input [3:0]                 cpu_req_write,  // [3:0] - For masking word/half/byte memory operations

  output                  cpu_resp_valid,
  output [CPU_WIDTH-1:0]  cpu_resp_data,  // [31:0]

  // Memory address input:

  output reg                  mem_req_valid,
  input                       mem_req_ready,
  output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr, // Is 28 bits [29:2] long... WHY?
  output reg                          mem_req_rw, // HIGH: Write, LOW: Read
  
  // Memory data input: 

  output                          mem_req_data_valid,
  input                            mem_req_data_ready,  // Should be high-z for ICache!!!
  output [`MEM_DATA_BITS-1:0]      mem_req_data_bits,
  // [127:0] byte level masking
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask, // [15:0] byte level masking

  // Memory data output:

  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data     // [127:0]
);
  // TIO wires

  wire [19:0] tag_bits;
  wire [5:0] index_bits;
  wire [3:0] offset_bits;

  reg [29:0] reg_cpu_req_addr; // FIX for missing first instruction: On any cache access we register the incoming address on the rising edge!!!

  assign tag_bits = reg_cpu_req_addr[29:10];
  assign index_bits = reg_cpu_req_addr[9:4];
  assign offset_bits = reg_cpu_req_addr[3:0];

  // SRAM regs and wires:
  // The following wires, except the data_addr wire which is shared among all 4 SRAMs, are partioned 4 ways. 
  // For example data_we is 4 bits where the ith bit corresponds to the ith SRAM from 0 - 3 for all 4 SRAMs.
  
  // Cache data SRAMs:
  reg [3:0] data_we;  // One hot encoding!
  reg [3:0] data_wmask;   // We are sharing (duplicating) this signal between all 4 SRAMs.
  reg [7:0] data_addr;
  

  reg [31:0] data_din [0:3];  // 32-bit wide vector with depth=4 (2D Array) - Contains the input to all 4 SRAMS in series
  
  reg [31:0] data_dout [0:3];  // 32-bit wide vector with depth=4 (2D Array) - Contains the output of all 4 SRAMS in series

  // Tells you which of the 4 words within the SRAM to choose of your corresponding cache line/set.
  wire [1:0] sram_line_word_sel;

  assign sram_line_word_sel = offset_bits[3:2]; // WHY DOES THIS WORK??? IS THE TIO THINKING INCORRECT THIS WHOLE TIME???
  // Is it rather: T - 20, I - 6, O - 
  
  // FIXME: this below does not work properly!!!
  // assign sram_line_word_sel = offset_bits[5:4]; // There is 16 bytes between sequential (i % 4) words within the same SRAM and cache line/set  

  // Tells you which of the 4 SRAMs to select in binary
  wire [1:0] sram_sel;
  assign sram_sel = offset_bits % 4;

  // Cache metadata SRAM:
  reg meta_we;
  reg [3:0] meta_wmask;
  reg [5:0] meta_addr;
  reg [31:0] meta_din;
  wire [31:0] meta_dout;

  // Always looking up metadata
  assign meta_addr = index_bits;


  // Memory signal wires:

  // Recall our external is 2^21 x 2^7 bits so 128 bit entries which is also 2^4 = 16 bytes
  // So we drop the 2 LSB of our cpu_req_addr since we want 16 byte addresable granularity! 
  wire [27:0] upper_addr;
  wire [1:0] lower_addr;

  assign upper_addr = reg_cpu_req_addr[29:2];
  assign lower_addr = reg_cpu_req_addr[1:0];

  // This is for relating the inputted CPU address to what fetch cycle (0, 1, 2, 3) will its the associated word be available (cacheline, word). For example, PC 0x2004 will be available in the 0th fetch cycle because the [5:2] bits of the PC is 0b0001 = 1. 1 is between 0-3 and we know that in the 0th cycle of the fetch the memory will send the cache the 0-3 first words of the cache line.

  reg [1:0] fetch_assoc_cycle; // TODO: weird ah name!
  // wire [3:0] ith_word_in_cache_line;  // Every cache line has 16 words!
  // assign ith_word_in_cache_line = upper_addr % 16;  // FIXME this is wrong!!!

  always @* begin
    if (offset_bits < 4)       fetch_assoc_cycle = 2'b00;
    else if (offset_bits < 8)  fetch_assoc_cycle = 2'b01;
    else if (offset_bits < 12) fetch_assoc_cycle = 2'b10;
    else                       fetch_assoc_cycle = 2'b11;
  end
  
  // There is 4 fetches we need to make since a cache line is 526 bits
  // All of external memory is chunked into 4  

  wire [25:0] upper_addr_cache_line_sub_block;  
  assign upper_addr_cache_line_sub_block = upper_addr[27:2];

  
  assign mem_req_addr = upper_addr; // Is fine to always be set to upper_addr since ExtMemModel will deal with giving us 4 x 128 bits over 4 cycles.

    
  // Describe the control part as a finite state machine

  // Define state bits
  parameter IDLE = 2'b00;     // Only time when cache sends a high ready signal. Ready to receive a CPU request
  parameter READ = 2'b01;     // Determines if we get a hit or miss
  parameter STORE = 2'b10;    // Write-through: Writes to both cache and memory
  parameter FETCH = 2'b11;    // Fetch for 4 cycles 512 bits from memory

  wire [1:0] state;
  reg [1:0] nextstate;


  // Fetch counter
  
  wire [1:0] fetch_count;
  wire [1:0] fetched_count;
  reg [1:0] next_fetch_count;

  assign fetched_count = fetch_count - 1;

  // READ state - ready valid signals + stall

  wire valid_bit;
  assign valid_bit = meta_dout[0];
  
  wire [17:0] ret_meta_tag;
  assign ret_meta_tag = meta_dout[31:14];

  // STORE state - Ready/valid signals for writing to MEM
  wire store_stall;
   assign store_stall = ~mem_req_ready; // only stall to wait for the address port of memory to be ready!!! Since the extmemmodel is a FSM where it takes in 1 then the other????
  // assign store_stall = ~mem_req_ready || ~mem_req_data_ready;  // Both the address and data input ports must be ready

  // FETCH state - Stall if only the memory input address port and not ready (not the output data port since we know it will be good later)
  wire fetch_stall;
  assign fetch_stall = ~mem_req_ready;
  // assign fetch_stall = mem_req_ready && mem_req_data_ready && mem_resp_valid;

  
  // Registered cache outputs to CPU
  reg [31:0] next_cpu_resp_data;
  reg next_cpu_resp_valid;
  
  // Registered cache output to memory for WRITES~
  reg next_mem_req_data_valid;
  reg [127:0] next_mem_req_data_bits;
  reg [15:0] next_mem_req_data_mask;


  always @* begin

    // Default values:

    // CPU ready/valid signals
    cpu_req_ready = 0;  
    next_cpu_resp_valid = 0;
    next_cpu_resp_data = cpu_resp_data; // Default: stay the same unless changed

    // External memory ready/valid signals:

    // Address input port
    mem_req_valid = 0;
    mem_req_rw = 0; // LOW: read
    // mem_req_addr = 28'b0;  Don't need to!

    // Data input port
    next_mem_req_data_valid = 0;
    next_mem_req_data_bits = 128'b0;
    next_mem_req_data_mask = 16'b0;

    // Cache memory SRAM signals

    data_we = 4'b0;
    data_wmask = 4'b0; 
    data_addr = 8'b0;

    data_din[0] = 32'b0;
    data_din[1] = 32'b0;
    data_din[2] = 32'b0;
    data_din[3] = 32'b0;

    // Cache metadata SRAM signals
    
    meta_we = 0;
    meta_wmask = 4'b0;
    meta_din = 32'b0;


    // If not overriden by later code default state is IDLE
    nextstate = IDLE;

    case (state)
      
      // TODO: check IDLE on its own!

      //IDLE STATE
      IDLE : begin
        cpu_req_ready = 1;  // Only time the cache is ready to do stuff!

        // First ensure the CPU is giving a valid response!
        if (cpu_req_valid) begin
          reg_cpu_req_addr = cpu_req_addr;
          
          // Writes
          if (cpu_req_write) begin
            nextstate = STORE;
          end

          // Reads
          else begin
            // Prepare cache output
            data_addr = {index_bits, sram_line_word_sel};

            nextstate = READ;
          end
        end
      end

      
      // TODO: check READ on its own!

      //READ STATE
      READ : begin
        // Check valid bit and if tags match 

        // HIT - Return cache output and go back to idle
        if (valid_bit && tag_bits == ret_meta_tag) begin
          nextstate = IDLE;
          next_cpu_resp_valid = 1;
          next_cpu_resp_data = data_dout[sram_sel];  // FIXME: this is not working!
        end 

        // MISS - Go fetch from memory
        else begin
          
          // Ready valid for memory - wait to FETCH from memory until memory is ready
          if (!fetch_stall) begin
            nextstate = FETCH;

            // Begin fetching from memory to fill cache
            mem_req_valid = 1;
            mem_req_rw = 0;   // LOW: read

            next_fetch_count = 2'b01;

            // Update cache metadata sram
            meta_we = 1;
            meta_wmask = 4'b1111;
            
            // Valid bit is high!
            meta_din = {tag_bits, 13'b0, 1'b1}; // TODO: contemplate about when and how the valid bit is reset to 0???
          end
        end
      end
      
      // Maybe TODO: I noticed that if we are stalling waiting to write to memory then we will continually be writing to the cache. Maybe put memory ready valid in the IDLE stage to reduce what is on? I think this is not a worry since in order to save power we need to implement power/clock gating! Too much.

      // TODO: check STORE on its own!

      //STORE STATE
      STORE : begin

        // Write to cache:

        if (valid_bit && tag_bits == ret_meta_tag) begin

          // Data sram:

          // One-hot encoding: where the ith bit represents which of the 4 SRAM you are writing to!
          data_we = 4'b0001 << sram_sel;
    
          // data_wmask = {{{((sram_sel+1)*1)-1}{1'b0}}, cpu_req_write, {(sram_sel*1){1'b0}}}  // TODO: check for correctness!
          data_wmask = cpu_req_write;

          // Old probably wrong thinking:
          // assign sram_line_word_sel = offset_bits[5:2] % 4; // To select word wise not byte wise! (1 byte * 4 = 1 32-bit word)

          data_addr = {index_bits, sram_line_word_sel};
          data_din[sram_sel] =  cpu_req_data;
          // data_din = {{{((sram_sel+1)*32)-1}{1'b0}}, cpu_req_data, {(sram_sel*32){1'b0}}};

          // Metadata sram:

          meta_we = 1;
          meta_wmask = 4'b1111;
          
          // Valid bit is high!
          meta_din = {tag_bits, 13'b0, 1'b1}; // TODO: contemplate about when and how the valid bit is reset to 0???

        end

        // Write to memory as well (write-through policy):

        /* FIXME: Does STORE state take more than 1 cycle due to write-through policy??? Read the following excerpt from the checkpoint spec:

        "(you can use a write-back or a write-through policy). Both of these transactions will take multiple cycles."

        */ 

        // Address input port:
        mem_req_valid = 1;
        mem_req_rw = 1; // HIGH: Write

        // Data input port

        // We need to register these values!

        next_mem_req_data_valid = 1;
        next_mem_req_data_bits = {{{((lower_addr+1)*32)-1}{1'b0}}, cpu_req_data, {(lower_addr*32){1'b0}}};

        
        next_mem_req_data_mask = {{{((lower_addr+1)*4)-1}{1'b0}}, cpu_req_write, {(lower_addr*4){1'b0}}};;

        

        // TODO: check if this logic makes sense!

        if (!store_stall) begin
          // Finished writing to memory
          nextstate = IDLE;
        end 
        else begin
          // Stall in the STORE state if memory is not ready to write
          nextstate = STORE;
        end

      end

      //FETCH STATE
      FETCH : begin
        nextstate = FETCH;
        
        // Write to SRAM data caches:

        // One-hot encoding: where the ith bit represents which of the 4 SRAM you are writing to!
        data_we = 4'b1111;  // Writing to all 4 SRAMs at once!
        data_wmask = 4'b1111; // Writing everything to SRAMs!
        data_addr = {index_bits, fetched_count};  // FIXME: this concatenation is weird!!!
      
        data_din[0] = mem_resp_data[31:0];
        data_din[1] = mem_resp_data[63:32];
        data_din[2] = mem_resp_data[95:64];
        data_din[3] = mem_resp_data[127:96];
              
        // Fetch from memory
        mem_req_valid = 1;
        mem_req_rw = 0;   // LOW: read
        // mem_req_addr = {upper_addr_cache_line_sub_block, fetch_count}; - don't need to do :D

        // Latch the memory data that should be returned later
        if (fetched_count == fetch_assoc_cycle) begin
          case (lower_addr) 
            2'd0: next_cpu_resp_data = mem_resp_data[31:0];
            2'd1: next_cpu_resp_data = mem_resp_data[63:32];
            2'd2: next_cpu_resp_data = mem_resp_data[95:64];
            2'd3: next_cpu_resp_data = mem_resp_data[127:96];
          endcase
        end

        // Update FETCH counter
        next_fetch_count = fetch_count + 1;

        // Equal to 0 means we looped back to 0 and are in the 4th FETCH cycle (1 -> 2 -> 3 -> 0, fetch_count)
        // DONE FETCHING 
        if (fetch_count == 0) begin
          // Finished FETCH state return to IDLE state
          nextstate = IDLE;

          // Reset fetch count back to 0
          next_fetch_count = 0;

          // Return fetched output
          next_cpu_resp_valid = 1;

          // Do this to tell memory we no longer want to read from memory!
          mem_req_valid = 0;
        end
      end
    endcase


  end

  // Stage register
  // Default state: IDLE state

  REGISTER_R #(.N(2), .INIT(2'b00)) state_reg
  (.q(state), .d(nextstate), .rst(reset), .clk(clk));

  // Fetch counter

  REGISTER_R #(.N(2), .INIT(2'b00)) fetch_counter_reg
  (.q(fetch_count), .d(next_fetch_count), .rst(reset), .clk(clk));

  // Registered output of cache output

  REGISTER_R #(.N(32), .INIT(32'b00)) cache_resp__data_reg
  (.q(cpu_resp_data), .d(next_cpu_resp_data), .rst(reset), .clk(clk));

  // Registered output of cache valid signal

  REGISTER_R #(.N(1), .INIT(1'b0)) cache_resp_valid_reg
  (.q(cpu_resp_valid), .d(next_cpu_resp_valid), .rst(reset), .clk(clk));

  // Registered outputs of cache to memory for writes!!
  REGISTER_R #(.N(1), .INIT(1'b0)) mem_req_data_valid_reg
  (.q(mem_req_data_valid), .d(next_mem_req_data_valid), .rst(reset), .clk(clk));
  REGISTER_R #(.N(128), .INIT(128'b0)) mem_req_data_bits_reg
  (.q(mem_req_data_bits), .d(next_mem_req_data_bits), .rst(reset), .clk(clk));
  REGISTER_R #(.N(16), .INIT(16'b0)) mem_req_data_mask_reg
  (.q(mem_req_data_mask), .d(next_mem_req_data_mask), .rst(reset), .clk(clk));


  // SRAM memory modules:

  // 4 x SRAM cache memory:

  // SRAM specifics: 

  // localparam DATA_WIDTH = 32;
  // localparam ADDR_WIDTH = 8;
  // localparam WMASK_WIDTH = 4;
  // localparam RAM_DEPTH = 1 << ADDR_WIDTH;

  genvar i;
  generate
    for (i=0; i<4; i=i+1) begin : sram_data_gen_block
      sram22_256x32m4w8 sram_data (
        .clk(clk),
        .we(data_we[i:i]),  // One-hot encoding
        .wmask(data_wmask), // All share the same wmask
        .addr(data_addr),   // All share the same addr
        .din(data_din[i]),  // 2D array (32 x 4) like regfile
        .dout(data_dout[i]) // 2D array (32 x 4) like regfile
      );
    end
  endgenerate

  // TODO: maybe somehow make sure that our SRAMs are initialized to 0?

  // 1 x SRAM cache metadata:

  // SRAM specifics: 

  // localparam DATA_WIDTH = 32;
  // localparam ADDR_WIDTH = 6;
  // localparam WMASK_WIDTH = 4;
  // localparam RAM_DEPTH = 1 << ADDR_WIDTH;

  sram22_64x32m4w8 sram_meta (
    .clk(clk),
    .we(meta_we),
    .wmask(meta_wmask),
    .addr(meta_addr),
    .din(meta_din),
    .dout(meta_dout)
  );

  // TODO: maybe somehow make sure that our SRAMs are initialized to 0? Ever more important for metadata valid bit!!!

endmodule


// Module:  Control Logic
// Desc:    Calculates all the control logic to feed into pipelines
// Inputs:  clk : clk
//          reset: rest
//          BrUn : branch unsigned
//          ...
// Outputs: ...

// what files to include ??
/**
 * List of RISC-V opcodes.
 * This file was completely rewritten from the file version that was used for MIPS. 
 * RISC-V uses far fewer opcodes than MIPS, but many more function codes.
 */

`ifndef OPCODE
`define OPCODE

// ***** Constants *****
`define BITS            32  // TODO: check if this is correct syntax then maybe consider using throughout the project!! - LJ 4/11

// ***** Opcodes *****

// No operation (kill)
`define OPC_NOOP        7'b0000000

// Special immediate instructions
`define OPC_LUI         7'b0110111
`define OPC_AUIPC       7'b0010111

// Jump instructions
`define OPC_JAL         7'b1101111
`define OPC_JALR        7'b1100111

// Branch instructions
`define OPC_BRANCH      7'b1100011

// Load and store instructions
`define OPC_STORE       7'b0100011
`define OPC_LOAD        7'b0000011

// Arithmetic instructions
`define OPC_ARI_RTYPE   7'b0110011
`define OPC_ARI_ITYPE   7'b0010011

// Control status register
`define OPC_CSR         7'b1110011


// ***** Function codes *****

// Branch function codes
`define FNC_BEQ         3'b000
`define FNC_BNE         3'b001
`define FNC_BLT         3'b100
`define FNC_BGE         3'b101
`define FNC_BLTU        3'b110
`define FNC_BGEU        3'b111

// Load and store function codes
`define FNC_LB          3'b000
`define FNC_LH          3'b001
`define FNC_LW          3'b010
`define FNC_LBU         3'b100
`define FNC_LHU         3'b101
`define FNC_SB          3'b000
`define FNC_SH          3'b001
`define FNC_SW          3'b010

// Arithmetic R-type and I-type functions codes
`define FNC_ADD_SUB     3'b000
`define FNC_SLL         3'b001
`define FNC_SLT         3'b010
`define FNC_SLTU        3'b011
`define FNC_XOR         3'b100
`define FNC_OR          3'b110
`define FNC_AND         3'b111
`define FNC_SRL_SRA     3'b101

// Control status function codes
`define FNC_RW          3'b001
`define FNC_RWI         3'b101

// ADD and SUB use the same opcode + function code
// SRA and SRL also use the same opcode + function code
// For these operations, we also need to look at bit 30 of the instruction
`define FNC2_ADD        1'b0
`define FNC2_SUB        1'b1
`define FNC2_SRL        1'b0
`define FNC2_SRA        1'b1

// R type instructions
`define RTYPE 3'b000
// I type instructions
`define ITYPE 3'b001
// I* type instructions
`define ISTARTYPE 3'b010
// S type instructions
`define STYPE 3'b100
// B type instructions
`define BTYPE 3'b101
// U type instructions
`define UTYPE 3'b111
// J type instructions
`define JTYPE 3'b110


`endif //OPCODE
/* Standard include file for EECS151.

 The "no flip-flop inference" policy.  Instead of using flip-flop and
 register inference, all EECS151/251A Verilog specifications will use
 explicit instantiation of register modules (defined below).  This
 policy will apply to lecture, discussion, lab, project, and problem
 sets.  This way of specification matches our RTL model of circuit,
 i.e., all specifications are nothing but a set of interconnected
 combinational logic blocks and state elements.  The goal is to
 simplify the use of Verilog and avoid mistakes that arise from
 specifying sequential logic.  Also, we can eliminate the explicit use
 of the non-blocking assignment "<=", and the associated confusion
 about blocking versus non-blocking.

 Here is a draft set of standard registers for EECS151.  All are
 positive edge triggered.  R and CE represent synchronous reset and
 clock enable, respectively. Both are active high.

 REGISTER 
 REGISTER_CE
 REGISTER_R
 REGISTER_R_CE
*/
`ifndef EECS151_V
`define EECS151_V

`timescale 1ns/1ns

// Register of D-Type Flip-flops
module REGISTER(q, d, clk);
  parameter N = 1;
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	     clk;
  initial q = {N{1'b0}};
  always @(posedge clk)
    q <= d;
endmodule // REGISTER

// Register with clock enable
module REGISTER_CE(q, d, ce, clk);
  parameter N = 1;
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      ce, clk;
  initial q = {N{1'b0}};
  always @(posedge clk)
    if (ce) q <= d;
endmodule // REGISTER_CE

// Register with reset value
module REGISTER_R(q, d, rst, clk);
  parameter N = 1;
  parameter INIT = {N{1'b0}};
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      rst, clk;
  initial q = INIT;
  always @(posedge clk)
    if (rst) q <= INIT;
    else q <= d;
endmodule // REGISTER_R

// Register with reset and clock enable
//  Reset works independently of clock enable
module REGISTER_R_CE(q, d, rst, ce, clk);
  parameter N = 1;
  parameter INIT = {N{1'b0}};
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      rst, ce, clk;
  initial q = INIT;
  always @(posedge clk)
    if (rst) q <= INIT;
    else if (ce) q <= d;
endmodule // REGISTER_R_CE


/* 
 Memory Blocks.
*/

// Single-port ROM with asynchronous read
module ASYNC_ROM(q, addr);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input  [AWIDTH-1:0] addr; // address
  output [DWIDTH-1:0] q;    // read data

  (* rom_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  assign q = mem[addr];
endmodule // ASYNC_ROM

// Single-port RAM with asynchronous read
module ASYNC_RAM(q, d, addr, we, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr; // address
  input 	            we;   // write-enable
  input  [DWIDTH-1:0] d;    // write data
  output [DWIDTH-1:0] q;    // read data

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we)
      mem[addr] <= d;
  end

  assign q = mem[addr];
endmodule // ASYNC_RAM

// Single-port ROM with synchronous read
module SYNC_ROM(q, addr, en, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input 	            clk;
  input               en;   // ram-enable
  input  [AWIDTH-1:0] addr; // address
  output [DWIDTH-1:0] q;    // read data

  (* rom_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_ROM

// Single-port RAM with synchronous read
module SYNC_RAM(q, d, addr, we, en, clk);
  parameter DWIDTH = 8;           // Data width
  parameter AWIDTH = 8;           // Address width
  parameter DEPTH  = 1 << AWIDTH; // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr; // address
  input 	            we;   // write-enable
  input               en;   // ram-enable
  input  [DWIDTH-1:0] d;    // write data
  output [DWIDTH-1:0] q;    // read data

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      if (we)
        mem[addr] <= d;
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_RAM

// Dual-port ROM with synchronous read
module SYNC_ROM_DP(q0, addr0, en0, q1, addr1, en1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input 	            clk;
  input               en0, en1;     // ram-enable
  input  [AWIDTH-1:0] addr0, addr1; // address
  output [DWIDTH-1:0] q0, q1;       // read data

  (* rom_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data0_reg;
  reg [DWIDTH-1:0] read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;
endmodule // SYNC_ROM_DP

// Dual-port RAM with asynchronous read
module ASYNC_RAM_DP(q0, d0, addr0, we0, q1, d1, addr1, we1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr0, addr1; // address
  input 	            we0, we1;     // write-enable
  input  [DWIDTH-1:0] d0, d1;       // write data
  output [DWIDTH-1:0] q0, q1;       // read data

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we0)
      mem[addr0] <= d0;
  end

  always @(posedge clk) begin
    if (we1)
      mem[addr1] <= d1;
  end

  assign q0 = mem[addr0];
  assign q1 = mem[addr1];

endmodule // ASYNC_RAM_DP

// Dual-port RAM with synchronous read
module SYNC_RAM_DP(q0, d0, addr0, we0, en0, q1, d1, addr1, we1, en1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr0, addr1; // address
  input 	            we0, we1;     // write-enable
  input               en0, en1;     // ram-enable
  input  [DWIDTH-1:0] d0, d1;       // write data
  output [DWIDTH-1:0] q0, q1;       // read data

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data0_reg, read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      if (we0)
        mem[addr0] <= d0;
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      if (we1)
        mem[addr1] <= d1;
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;

endmodule // SYNC_RAM_DP

// Single-port RAM with synchronous read with write byte-enable
module SYNC_RAM_WBE(q, d, addr, en, wbe, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input [DWIDTH-1:0]   d;    // Data input
  input [AWIDTH-1:0]   addr; // Address input
  input [DWIDTH/8-1:0] wbe;  // write-byte-enable
  input en;
  input clk;
  output [DWIDTH-1:0] q;

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      for (i = 0; i < DWIDTH/8; i = i+1) begin
        if (wbe[i])
          mem[addr][i*8 +: 8] <= d[i*8 +: 8];
        end
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_RAM_WBE

// Dual-port RAM with synchronous read with write byte-enable
module SYNC_RAM_DP_WBE(q0, d0, addr0, en0, wbe0, q1, d1, addr1, en1, wbe1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input clk;
  input [DWIDTH-1:0]   d0;    // Data input
  input [AWIDTH-1:0]   addr0; // Address input
  input [DWIDTH/8-1:0] wbe0;  // write-byte-enable
  input                en0;
  output [DWIDTH-1:0]  q0;

  input [DWIDTH-1:0]   d1;    // Data input
  input [AWIDTH-1:0]   addr1; // Address input
  input [DWIDTH/8-1:0] wbe1;  // write-byte-enable
  input                en1;
  output [DWIDTH-1:0]  q1;

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end


  reg [DWIDTH-1:0] read_data0_reg;
  reg [DWIDTH-1:0] read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      for (i = 0; i < 4; i = i+1) begin
        if (wbe0[i])
          mem[addr0][i*8 +: 8] <= d0[i*8 +: 8];
      end
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      for (i = 0; i < 4; i = i+1) begin
        if (wbe1[i])
          mem[addr1][i*8 +: 8] <= d1[i*8 +: 8];
        end
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;

endmodule // SYNC_RAM_DP_WBE

// Multi-port RAM with two asynchronous-read ports, one synchronous-write port
module ASYNC_RAM_1W2R(d0, addr0, we0, q1, addr1, q2, addr2, clk);
  parameter DWIDTH = 8;  // Data width
  parameter AWIDTH = 8;  // Address width
  parameter DEPTH = 256; // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";
  input clk;

  input [DWIDTH-1:0] d0;    // Data input
  input [AWIDTH-1:0] addr0; // Address input
  input              we0;   // Write enable

  input [AWIDTH-1:0] addr1; // Address input
  output [DWIDTH-1:0] q1;

  input [AWIDTH-1:0] addr2; // Address input
  output [DWIDTH-1:0] q2;

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we0)
      mem[addr0] <= d0;
  end

  assign q1 = mem[addr1];
  assign q2 = mem[addr2];

endmodule // ASYNC_RAM_1W2R

`endif
module control_logic(
    input clk,
    input reset,
    input stall,

    // IF/ID Stage
    output icache_re,
    input [31:0] inst_if,
    output reg [1:0] rdata1Sel,
    output reg [1:0] rdata2Sel,
    output reg icache_sel,

    // EX Stage
    output reg flush_ex,
    output BrUn,
    input BrEq,
    input BrLt,
    output reg ASel,
    output reg BSel,
    output [6:0] opcode_ex,
    output [2:0] funct,
    output add_rshift_type,
    input [31:0] inst_ex,
    output MemWEn,
    output reg dcache_din_Sel,
    output reg dcache_addr_Sel,

    // MEM/WB Stage
    output dcache_re,
    output [3:0] dcache_we,
    output reg flush_mem,
    output RegWEn,
    output reg [1:0] PCSel,
    output reg [1:0] WBSel,
    input [31:0] inst_mem,

    // Can keep for now, will NOT be used for our "single stage"
    // Extra fun instruction register for forwarding! :D
    input [31:0] inst_wb
);
    // Decoding of the instructions for each stage:

    // IF/ID stage decoding

    wire [6:0] if_opcode;
    wire [2:0] if_f3;
    wire [6:0] if_f7;
    wire [4:0] if_rd, if_rs1, if_rs2;

    assign if_opcode = inst_if[6:0];
    assign if_f3 = inst_if[14:12];
    assign if_f7 = inst_if[31:25];  // Only for R, I* type instructions
    assign if_rd = inst_if[11:7];   // Only for R, I, I*, U, and J type instructions
    assign if_rs1 = inst_if[19:15]; // Only for R, I, I*, S, and B type instructions
    assign if_rs2 = inst_if[24:20]; // Only for R, S and B type instructions

    // EX stage decoding

    wire [6:0] ex_opcode;
    wire [2:0] ex_f3;
    wire [4:0] ex_rd, ex_rs1, ex_rs2;
    wire [6:0] ex_f7;

    assign ex_opcode = inst_ex[6:0];
    assign ex_f3 = inst_ex[14:12];
    assign ex_f7 = inst_ex[31:25];  // Only for R, I* type instructions
    assign ex_rd = inst_ex[11:7];   // Only for R, I, I*, U, and J type instructions
    assign ex_rs1 = inst_ex[19:15]; // Only for R, I, I*, S, and B type instructions
    assign ex_rs2 = inst_ex[24:20]; // Only for R, S and B type instructions

    // MEM stage decoding

    wire [6:0] mem_opcode;
    wire [2:0] mem_f3;
    wire [6:0] mem_f7;
    wire [4:0] mem_rd, mem_rs1, mem_rs2;

    assign mem_opcode = inst_mem[6:0];
    assign mem_f3 = inst_mem[14:12];
    assign mem_f7 = inst_mem[31:25];  // Only for R, I* type instructions
    assign mem_rd = inst_mem[11:7];   // Only for R, I, I*, U, and J type instructions
    assign mem_rs1 = inst_mem[19:15]; // Only for R, I, I*, S, and B type instructions
    assign mem_rs2 = inst_mem[24:20]; // Only for R, S and B type instructions

    // "WB" Stage decoding

    wire [6:0] wb_opcode;
    wire [2:0] wb_f3;
    wire [4:0] wb_rd, wb_rs1, wb_rs2;
    wire [6:0] wb_f7;

    assign wb_opcode = inst_wb[6:0];
    assign wb_f3 = inst_wb[14:12];
    assign wb_f7 = inst_wb[31:25];  // Only for R, I* type instructions
    assign wb_rd = inst_wb[11:7];   // Only for R, I, I*, U, and J type instructions
    assign wb_rs1 = inst_wb[19:15]; // Only for R, I, I*, S, and B type instructions
    assign wb_rs2 = inst_wb[24:20]; // Only for R, S and B type instructions
    
    // ===============================================================
    // ================ IF/ID Stage - Control Signals ================
    // ===============================================================

    // We need to pipeline if a branch should be taken from the EX to the MEM/WB stage
    wire taken_ex, taken_mem;
    
    taken_not_taken taken_not_taken(
        // Inputs
        .clk(clk), .reset(reset), .stall(stall),
        .inst_ex(inst_ex),
        .BrEq(BrEq),
        .BrLt(BrLt),

        // Outputs
        .taken_ex(taken_ex),
        .taken_mem(taken_mem)
    );    
// when to NOP // how to correctly set PC after a NOP ?
    reg PC_stall;

    // Needed to fix the MEM -> ALU data forwarding stalling control logic TIMING

    // PCSel logic:
    // if (reset) ==> 2 (pc_wire)
    // else if (branch/JAL/JALR) ==> 1 (wb_ALU_wire)
    // else ==> 0


    wire reset_reg_out;

    REGISTER_R_CE #(.N(1), .INIT(1'b0)) reset_reg 
        (.q(reset_reg_out), .d(reset), .ce(!stall), .rst(1'b0), .clk(clk));

    always @* begin

        if (reset_reg_out || stall || PC_stall || 
            // LW
            inst_ex == `OPC_LOAD || 
            // branches
            inst_ex == `OPC_BRANCH ||
            // jumps 
            inst_ex == `OPC_JAL ||
            inst_ex == `OPC_JALR
            ) begin
            PCSel = 2'b10;      // stall
            icache_sel = 1'b1;  // insert NOP
        end else if (mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR || mem_opcode == `OPC_BRANCH) begin
            PCSel = 2'b01;      // jump next cycle
            icache_sel = 1'b1;  // insert NOP immidiately
        end else begin
            PCSel = 2'b0;
            icache_sel = 1'b0;
        end
    end

    // IMEM ready/valid signal
    
    assign icache_re = (reset || PC_stall || stall) ? 1'b0 : 1'b1;

    // Data forwarding logic for muxes

    // rdata1Sel:
    // 00 - regfile
    // 01 - ALU_out
    // 10 - wdata_wb
    // 11 - (unknown)

    always @* begin
        // Always initialize with no data forwarding: regfile
        rdata1Sel = 2'b00;

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin

            // ALU -> ALU
            // Check if a data hazard might exist. Requirements:
            // 2nd instruction (IF/ID stage) rs1 and 1st instruction (EX stage) rd registers match.
            // 2nd instruction must not be a JAL/AUIPC/LUI instruction.
            // 1st instruction must have a rd register but cannot be a load instruction,
            // so only R, U, and I type instructions (excluding load instructions).

            if (if_rs1 == ex_rd &&
                if_rs1 != 5'b0 &&
                if_opcode != `OPC_JAL &&
                if_opcode != `OPC_AUIPC &&
                if_opcode != `OPC_LUI &&
                (ex_opcode == `OPC_ARI_RTYPE ||
                ex_opcode == `OPC_ARI_ITYPE ||
                ex_opcode == `OPC_AUIPC ||
                ex_opcode == `OPC_LUI)) begin
                    rdata1Sel = 2'b01; 
            end 
            // MEM -> ALU
            // 1 cycle stall + forward, for load words
            // or
            // 2 cycle apart hazards so be more general!

            else if (if_rs1 == mem_rd &&
                    if_rs1 != 5'b0 &&
                    if_opcode != `OPC_JAL &&
                    if_opcode != `OPC_AUIPC &&
                    if_opcode != `OPC_LUI &&
                    (mem_opcode == `OPC_LOAD || 
                    mem_opcode == `OPC_ARI_RTYPE ||
                    mem_opcode == `OPC_ARI_ITYPE ||
                    mem_opcode == `OPC_AUIPC ||
                    mem_opcode == `OPC_LUI)) begin 
                    rdata1Sel = 2'b10;
            end
        end
    end 

    
    // rdata1Sel:
    // 00 - regfile
    // 01 - ALU_out
    // 10 - wdata_wb
    // 11 - (unknown)
    
    always @* begin
        // Always initialize with no data forwarding: regfile
        rdata2Sel = 2'b00;

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin

            // ALU -> ALU
            // Check if a data hazard might exist. Requirements:
            // 2nd instruction (IF/ID stage) rs1 and 1st instruction (EX stage) rd registers match.
            // 2nd instruction must not be a JAL/AUIPC/LUI instruction.
            // 1st instruction must have a rd register but cannot be a load instruction,
            // so only R, U, and I type instructions (excluding load instructions).

            if (if_rs2 == ex_rd &&
                if_rs2 != 5'b0 &&
                if_opcode != `OPC_JAL &&
                if_opcode != `OPC_JALR &&
                if_opcode != `OPC_AUIPC &&
                if_opcode != `OPC_LUI &&
                if_opcode != `OPC_ARI_ITYPE &&
                if_opcode != `OPC_LOAD && 
                (ex_opcode == `OPC_ARI_RTYPE ||
                ex_opcode == `OPC_ARI_ITYPE ||
                ex_opcode == `OPC_AUIPC ||
                ex_opcode == `OPC_LUI)) begin
                    rdata2Sel = 2'b01; 
            end 

            // MEM -> ALU
            // 1 cycle stall + forward, for load words
            // or
            // 2 cycle apart hazards so be more general!

            else if (if_rs2 == mem_rd &&
                    if_rs2 != 5'b0 &&
                    if_opcode != `OPC_JAL &&
                    if_opcode != `OPC_JALR &&
                    if_opcode != `OPC_AUIPC &&
                    if_opcode != `OPC_LUI &&
                    if_opcode != `OPC_ARI_ITYPE &&
                    if_opcode != `OPC_LOAD && 
                    (mem_opcode == `OPC_LOAD || 
                    mem_opcode == `OPC_ARI_RTYPE ||
                    mem_opcode == `OPC_ARI_ITYPE ||
                    mem_opcode == `OPC_AUIPC ||
                    mem_opcode == `OPC_LUI)) begin 
                    rdata2Sel = 2'b10;
            end
        end
    end

    // ===============================================================
    // ================== EX Stage - Control Signals =================
    // ===============================================================

    // 1 if the instruction is (sltu/sltiu/bltu/bgeu) * (doesn't matter) otherwise
    assign BrUn =   (reset) ? 1'b0 : 
                    (ex_opcode == `OPC_ARI_RTYPE && ex_f3 == `FNC_SLTU) ||
                    (ex_opcode == `OPC_ARI_ITYPE && ex_f3 == `FNC_SLTU) ||  // TODO: check if `FNC_SLTU is the correct Funct3 code for sltiu???
                    (ex_opcode == `OPC_BRANCH && ex_f3 == `FNC_BLTU) ||
                    (ex_opcode == `OPC_BRANCH && ex_f3 == `FNC_BGEU);

    /* 
    ASel    := 1 ==> A := rdata1, 
            := 0 ==> A := PC  

    ASel := 1 if 
        instruction type == r type, i type, I* type, store type 
        instruction is a JALR

    ASel := 0 if
        instruction type = b type
        instruction is a JAL or AUIPC instruction
    */

    assign ASel = (ex_opcode == `OPC_ARI_RTYPE) ||
                    (ex_opcode == `OPC_ARI_ITYPE) ||
                    (ex_opcode == `OPC_LOAD) ||
                    // (ex_opcode == `OPC_BRANCH) ||
                    (ex_opcode == `OPC_JALR) || 
                    (ex_opcode == `OPC_CSR) || 
                    (ex_opcode == `OPC_STORE);

    /* 
    BSel    := 1 ==> B := imm, 
            := 0 ==> B := rdata2

    BSel := 1 if 
        instruction type is not an r type instruction
    */
    
    assign BSel = (ex_opcode == `OPC_ARI_RTYPE) ? 0 : 1;

    // ALU decoder control signals
    assign opcode_ex = ex_opcode;
    assign funct = ex_f3;
    // assign add_rshift_type = (ex_opcode == `OPC_ARI_RTYPE) ? ex_f7[5] : 1'b0;   
    assign add_rshift_type = ex_f7[5]; // should always be the inst[30] bit

    // Partial Store signal
    assign MemWEn = (ex_opcode == `OPC_STORE && 
                        ~(mem_opcode == `OPC_BRANCH && taken_mem) &&
                        ~(mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR));


    // Bypass path control signals
    
    always @* begin
        // By default: Don't forward!
        
        dcache_din_Sel = 1'b0;

        // MEM -> MEM (dcache_din, rs2)

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin
                
            if (ex_rs2 == mem_rd &&
                ex_rs2 != 5'b0 &&
                mem_opcode == `OPC_LOAD &&
                (ex_opcode == `OPC_LOAD || 
                ex_opcode == `OPC_STORE)) begin
                dcache_din_Sel = 1'b1;
            end
        end

        
        // MEM -> MEM (address input, rs1)

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin
            
            dcache_addr_Sel = 1'b0;

            if (ex_rs1 == mem_rd &&
                ex_rs1 != 5'b0 &&
                mem_opcode == `OPC_LOAD &&
                (ex_opcode == `OPC_LOAD || 
                ex_opcode == `OPC_STORE)) begin
                dcache_addr_Sel = 1'b1;
            end
        end
    end

    // ===============================================================
    // ================ MEM/WB Stage - Control Signals ===============
    // ===============================================================

    // DMEM ready/valid signal

    // dcache_re := 1 ==> signal ready
    //           := 0 ==> signal not ready
    // dcache_re := 1 ==> if
    //                     instruction is ??????? always leave on ??? 
    assign dcache_re = (reset) ? 4'b0 : (ex_opcode == `OPC_LOAD && !stall) ? 1'b1: 1'b0;

    // dcache_we := 1 ==> write enabled
    //           := 0 ==> write not enabled
    // dcahce_we := 1 ==> if
    //                     instruction is store type
    assign dcache_we = (reset) ? 4'b0 : (mem_opcode == `OPC_STORE && 
                        ~(mem_opcode == `OPC_BRANCH && taken_mem) &&
                        ~(mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR)) ? 4'b1111 : 4'b0;

    // RegWEn := 1 ==> RegWEn := write enabled
    //        := 0 ==> RegWEn := write not enabled
    // RegWEn := 1 if
    //              instruction type == R, I, I*, Jal, Jalr, LUI, AUIPC, Loads???
    assign RegWEn = (reset || stall) ?  0 : (mem_opcode == `OPC_ARI_RTYPE ||
                                    mem_opcode == `OPC_ARI_ITYPE ||
                                    mem_opcode == `OPC_JAL ||
                                    mem_opcode == `OPC_JALR ||
                                    mem_opcode == `OPC_LUI ||
                                    mem_opcode == `OPC_AUIPC ||
                                    mem_opcode == `OPC_LOAD) ? 1'b1: 1'b0;

    // WBSel := 00 == > ALU_out
    //       := 01 == > PC + 4
    //       := 10 == > Mem_outPC_stall
    //       := 11 == > ??
    // WBSel := 01 if
    //              instruction is JAL
    //              instruction is JALR
    // WBSel := 00 if 
    //              instruction anything else (R, I, U)
    // WBSel := 10 if 
    //              instruction is load type

    assign WBSel = (mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR) ? 2'b01 : 
                   (mem_opcode == `OPC_LOAD) ? 2'b10 : 
                   2'b00;

    // Flushing and stalling control signals

    always @* begin
        PC_stall = 1'b0;
        flush_ex = 1'b0;
        flush_mem = 1'b0;

        if (!stall) begin
            // MEM -> ALU insert bubble
            // For when we need to insert a bubble for a MEM -> ALU (lw) Data hazard!!!
            if (ex_opcode == `OPC_LOAD &&
                if_opcode != `OPC_JAL &&
                if_opcode != `OPC_AUIPC &&
                if_opcode != `OPC_LUI  && 
                ~(mem_opcode == `OPC_BRANCH && taken_mem) &&
                ~(mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR)
            ) begin

                if (if_rs1 == ex_rd && if_rs1 != 5'b0) begin
                    PC_stall = 1'b1;
                    flush_ex = 1'b1;
                end
                // For the edge case of jalr which has only rs1 and not rs2
                else if (if_rs2 == ex_rd && 
                        if_rs2 != 5'b0 &&
                        if_opcode != `OPC_JALR) begin
                    PC_stall = 1'b1;
                    flush_ex = 1'b1;
                end
            end

            
            // Control Hazards (Branch, Jumps)
            
            // Branch
            else if (mem_opcode == `OPC_BRANCH && taken_mem) begin   
                flush_ex = 1;
                flush_mem = 1;

                
            end

            // Jumps
            else if (mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR) begin
                flush_ex = 1;
                flush_mem = 1;
            end 
        end
    end

    
endmodule 


// This module pipelines if a branch should be taken or not taken from the EX to the MEM stage

module taken_not_taken (
    input clk, reset, stall,
    input [31:0] inst_ex,
    input BrEq,
    input BrLt,

    output taken_ex, taken_mem
);
    wire [6:0] ex_opcode;
    wire [2:0] ex_f3;

    assign ex_opcode = inst_ex[6:0];
    assign ex_f3 = inst_ex[14:12];

    reg taken_ex; 
    wire taken_mem;    

    always @* begin
        if (ex_opcode == `OPC_BRANCH) begin
            if ((ex_f3 == `FNC_BEQ && BrEq) ||
                (ex_f3 == `FNC_BNE && !BrEq) ||
                (ex_f3 == `FNC_BLT && BrLt) ||
                (ex_f3 == `FNC_BGE && !BrLt) ||
                (ex_f3 == `FNC_BLTU && BrLt) ||
                (ex_f3 == `FNC_BGEU && !BrLt)) begin 
                taken_ex = 1;
            end
            else begin
                taken_ex = 0;
            end
        end
        else begin
            taken_ex = 0;
        end
    end

    REGISTER_R_CE #(.N(1), .INIT(1'b0)) taken_reg_mem
    (.q(taken_mem), .d(taken_ex), .ce(!stall), .rst(reset), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)    
endmodule
/* Standard include file for EECS151.

 The "no flip-flop inference" policy.  Instead of using flip-flop and
 register inference, all EECS151/251A Verilog specifications will use
 explicit instantiation of register modules (defined below).  This
 policy will apply to lecture, discussion, lab, project, and problem
 sets.  This way of specification matches our RTL model of circuit,
 i.e., all specifications are nothing but a set of interconnected
 combinational logic blocks and state elements.  The goal is to
 simplify the use of Verilog and avoid mistakes that arise from
 specifying sequential logic.  Also, we can eliminate the explicit use
 of the non-blocking assignment "<=", and the associated confusion
 about blocking versus non-blocking.

 Here is a draft set of standard registers for EECS151.  All are
 positive edge triggered.  R and CE represent synchronous reset and
 clock enable, respectively. Both are active high.

 REGISTER 
 REGISTER_CE
 REGISTER_R
 REGISTER_R_CE
*/
`ifndef EECS151_V
`define EECS151_V

`timescale 1ns/1ns

// Register of D-Type Flip-flops
module REGISTER(q, d, clk);
  parameter N = 1;
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	     clk;
  initial q = {N{1'b0}};
  always @(posedge clk)
    q <= d;
endmodule // REGISTER

// Register with clock enable
module REGISTER_CE(q, d, ce, clk);
  parameter N = 1;
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      ce, clk;
  initial q = {N{1'b0}};
  always @(posedge clk)
    if (ce) q <= d;
endmodule // REGISTER_CE

// Register with reset value
module REGISTER_R(q, d, rst, clk);
  parameter N = 1;
  parameter INIT = {N{1'b0}};
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      rst, clk;
  initial q = INIT;
  always @(posedge clk)
    if (rst) q <= INIT;
    else q <= d;
endmodule // REGISTER_R

// Register with reset and clock enable
//  Reset works independently of clock enable
module REGISTER_R_CE(q, d, rst, ce, clk);
  parameter N = 1;
  parameter INIT = {N{1'b0}};
  output reg [N-1:0] q;
  input [N-1:0]      d;
  input 	      rst, ce, clk;
  initial q = INIT;
  always @(posedge clk)
    if (rst) q <= INIT;
    else if (ce) q <= d;
endmodule // REGISTER_R_CE


/* 
 Memory Blocks.
*/

// Single-port ROM with asynchronous read
module ASYNC_ROM(q, addr);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input  [AWIDTH-1:0] addr; // address
  output [DWIDTH-1:0] q;    // read data

  (* rom_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  assign q = mem[addr];
endmodule // ASYNC_ROM

// Single-port RAM with asynchronous read
module ASYNC_RAM(q, d, addr, we, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr; // address
  input 	            we;   // write-enable
  input  [DWIDTH-1:0] d;    // write data
  output [DWIDTH-1:0] q;    // read data

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we)
      mem[addr] <= d;
  end

  assign q = mem[addr];
endmodule // ASYNC_RAM

// Single-port ROM with synchronous read
module SYNC_ROM(q, addr, en, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input 	            clk;
  input               en;   // ram-enable
  input  [AWIDTH-1:0] addr; // address
  output [DWIDTH-1:0] q;    // read data

  (* rom_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_ROM

// Single-port RAM with synchronous read
module SYNC_RAM(q, d, addr, we, en, clk);
  parameter DWIDTH = 8;           // Data width
  parameter AWIDTH = 8;           // Address width
  parameter DEPTH  = 1 << AWIDTH; // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr; // address
  input 	            we;   // write-enable
  input               en;   // ram-enable
  input  [DWIDTH-1:0] d;    // write data
  output [DWIDTH-1:0] q;    // read data

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      if (we)
        mem[addr] <= d;
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_RAM

// Dual-port ROM with synchronous read
module SYNC_ROM_DP(q0, addr0, en0, q1, addr1, en1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input 	            clk;
  input               en0, en1;     // ram-enable
  input  [AWIDTH-1:0] addr0, addr1; // address
  output [DWIDTH-1:0] q0, q1;       // read data

  (* rom_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data0_reg;
  reg [DWIDTH-1:0] read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;
endmodule // SYNC_ROM_DP

// Dual-port RAM with asynchronous read
module ASYNC_RAM_DP(q0, d0, addr0, we0, q1, d1, addr1, we1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr0, addr1; // address
  input 	            we0, we1;     // write-enable
  input  [DWIDTH-1:0] d0, d1;       // write data
  output [DWIDTH-1:0] q0, q1;       // read data

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we0)
      mem[addr0] <= d0;
  end

  always @(posedge clk) begin
    if (we1)
      mem[addr1] <= d1;
  end

  assign q0 = mem[addr0];
  assign q1 = mem[addr1];

endmodule // ASYNC_RAM_DP

// Dual-port RAM with synchronous read
module SYNC_RAM_DP(q0, d0, addr0, we0, en0, q1, d1, addr1, we1, en1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input               clk;
  input  [AWIDTH-1:0] addr0, addr1; // address
  input 	            we0, we1;     // write-enable
  input               en0, en1;     // ram-enable
  input  [DWIDTH-1:0] d0, d1;       // write data
  output [DWIDTH-1:0] q0, q1;       // read data

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data0_reg, read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      if (we0)
        mem[addr0] <= d0;
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      if (we1)
        mem[addr1] <= d1;
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;

endmodule // SYNC_RAM_DP

// Single-port RAM with synchronous read with write byte-enable
module SYNC_RAM_WBE(q, d, addr, en, wbe, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input [DWIDTH-1:0]   d;    // Data input
  input [AWIDTH-1:0]   addr; // Address input
  input [DWIDTH/8-1:0] wbe;  // write-byte-enable
  input en;
  input clk;
  output [DWIDTH-1:0] q;

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  reg [DWIDTH-1:0] read_data_reg;
  always @(posedge clk) begin
    if (en) begin
      for (i = 0; i < DWIDTH/8; i = i+1) begin
        if (wbe[i])
          mem[addr][i*8 +: 8] <= d[i*8 +: 8];
        end
      read_data_reg <= mem[addr];
    end
  end

  assign q = read_data_reg;
endmodule // SYNC_RAM_WBE

// Dual-port RAM with synchronous read with write byte-enable
module SYNC_RAM_DP_WBE(q0, d0, addr0, en0, wbe0, q1, d1, addr1, en1, wbe1, clk);
  parameter DWIDTH = 8;             // Data width
  parameter AWIDTH = 8;             // Address width
  parameter DEPTH  = (1 << AWIDTH); // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";

  input clk;
  input [DWIDTH-1:0]   d0;    // Data input
  input [AWIDTH-1:0]   addr0; // Address input
  input [DWIDTH/8-1:0] wbe0;  // write-byte-enable
  input                en0;
  output [DWIDTH-1:0]  q0;

  input [DWIDTH-1:0]   d1;    // Data input
  input [AWIDTH-1:0]   addr1; // Address input
  input [DWIDTH/8-1:0] wbe1;  // write-byte-enable
  input                en1;
  output [DWIDTH-1:0]  q1;

  (* ram_style = "block" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end


  reg [DWIDTH-1:0] read_data0_reg;
  reg [DWIDTH-1:0] read_data1_reg;

  always @(posedge clk) begin
    if (en0) begin
      for (i = 0; i < 4; i = i+1) begin
        if (wbe0[i])
          mem[addr0][i*8 +: 8] <= d0[i*8 +: 8];
      end
      read_data0_reg <= mem[addr0];
    end
  end

  always @(posedge clk) begin
    if (en1) begin
      for (i = 0; i < 4; i = i+1) begin
        if (wbe1[i])
          mem[addr1][i*8 +: 8] <= d1[i*8 +: 8];
        end
      read_data1_reg <= mem[addr1];
    end
  end

  assign q0 = read_data0_reg;
  assign q1 = read_data1_reg;

endmodule // SYNC_RAM_DP_WBE

// Multi-port RAM with two asynchronous-read ports, one synchronous-write port
module ASYNC_RAM_1W2R(d0, addr0, we0, q1, addr1, q2, addr2, clk);
  parameter DWIDTH = 8;  // Data width
  parameter AWIDTH = 8;  // Address width
  parameter DEPTH = 256; // Memory depth
  parameter MIF_HEX = "";
  parameter MIF_BIN = "";
  input clk;

  input [DWIDTH-1:0] d0;    // Data input
  input [AWIDTH-1:0] addr0; // Address input
  input              we0;   // Write enable

  input [AWIDTH-1:0] addr1; // Address input
  output [DWIDTH-1:0] q1;

  input [AWIDTH-1:0] addr2; // Address input
  output [DWIDTH-1:0] q2;

  (* ram_style = "distributed" *) reg [DWIDTH-1:0] mem [0:DEPTH-1];

  integer i;
  initial begin
    if (MIF_HEX != "") begin
      $readmemh(MIF_HEX, mem);
    end
    else if (MIF_BIN != "") begin
      $readmemb(MIF_BIN, mem);
    end
    else begin
      for (i = 0; i < DEPTH; i = i + 1) begin
        mem[i] = 0;
      end
    end
  end

  always @(posedge clk) begin
    if (we0)
      mem[addr0] <= d0;
  end

  assign q1 = mem[addr1];
  assign q2 = mem[addr2];

endmodule // ASYNC_RAM_1W2R

`endif

module ExtMemModel
(
  input         clk,
  input         reset,

  // Read/Write Address (input Address port) request from CPU
  input                      mem_req_valid,
  output                     mem_req_ready,
  input                      mem_req_rw, // HIGH: Write, LOW: Read
  input [`MEM_ADDR_BITS-1:0] mem_req_addr,
  input [`MEM_TAG_BITS-1:0]  mem_req_tag,

  // Write data (input Data port) request from CPU
  input                          mem_req_data_valid,
  output                         mem_req_data_ready,
  input [`MEM_DATA_BITS-1:0]     mem_req_data_bits,
  input [(`MEM_DATA_BITS/8)-1:0] mem_req_data_mask,

  // Read data (output Data port) response to CPU
  output reg                      mem_resp_valid,
  output reg [`MEM_DATA_BITS-1:0] mem_resp_data,
  output reg [`MEM_TAG_BITS-1:0]  mem_resp_tag
);

  // Memory read takes 4 consecutive cycles of 128-bit each
  localparam DATA_CYCLES = 4;
  localparam DEPTH = 2*1024*1024; // 2*1024*1024 entries of 128-bit (2M x 16B)

  reg [`ceilLog2(DATA_CYCLES)-1:0] cnt;
  reg [`MEM_TAG_BITS-1:0] tag;
  reg state_busy, state_rw;
  reg [`MEM_ADDR_BITS-1:0] addr;

  reg [`MEM_DATA_BITS-1:0] ram [DEPTH-1:0];
  // Ignore lower 2 bits and count ourselves if read, otherwise if write use the 
  wire do_write = mem_req_data_valid && mem_req_data_ready;
  // exact address delivered
  wire [`ceilLog2(DEPTH)-1:0] ram_addr = state_busy  ? ( do_write ? addr[`ceilLog2(DEPTH)-1:0] :  {addr[`ceilLog2(DEPTH)-1:`ceilLog2(DATA_CYCLES)], cnt} )
                                                     : {mem_req_addr[`ceilLog2(DEPTH)-1:`ceilLog2(DATA_CYCLES)], cnt};
  wire do_read = mem_req_valid && mem_req_ready && !mem_req_rw || state_busy && !state_rw;

  initial
  begin : zero
    integer i;
    for (i = 0; i < DEPTH; i = i+1)
      ram[i] = 1'b0;
  end

  wire [`MEM_DATA_BITS-1:0] masked_din;

  generate
    genvar i;
    for (i = 0; i < `MEM_DATA_BITS; i=i+1) begin: MASKED_DIN
      assign masked_din[i] = mem_req_data_mask[i/8] ? mem_req_data_bits[i] : ram[ram_addr][i];
    end
  endgenerate

  always @(posedge clk)
  begin
    if (reset)
      state_busy <= 1'b0;
    else if ((do_read && cnt == DATA_CYCLES-1 || do_write))
      state_busy <= 1'b0;
    else if (mem_req_valid && mem_req_ready)
      state_busy <= 1'b1;

    if (!state_busy && mem_req_valid)
    begin
      state_rw <= mem_req_rw;
      tag <= mem_req_tag;
      addr <= mem_req_addr;
    end

    if (reset)
      cnt <= 1'b0;
    else if(do_read)
      cnt <= cnt + 1'b1;

    if (do_write)
      ram[ram_addr] <= masked_din;
    else
      mem_resp_data <= ram[ram_addr];

    if (reset)
      mem_resp_valid <= 1'b0;
    else
      mem_resp_valid <= do_read;

    mem_resp_tag <= state_busy ? tag : mem_req_tag;
  end

  assign mem_req_ready = !state_busy;
  assign mem_req_data_ready = state_busy && state_rw;

endmodule


module Memory151( 
  input clk,
  input reset,

  // Cache <=> CPU interface
  input  [31:0] dcache_addr,
  input  [31:0] icache_addr,
  input  [3:0]  dcache_we,
  input         dcache_re,
  input         icache_re,
  input  [31:0] dcache_din,
  output [31:0] dcache_dout,
  output [31:0] icache_dout,
  output        stall,

  // Arbiter <=> Main memory interface
  output                       mem_req_valid,
  input                        mem_req_ready,
  output                       mem_req_rw,
  output [`MEM_ADDR_BITS-1:0]  mem_req_addr,
  output [`MEM_TAG_BITS-1:0]   mem_req_tag,

  output                       mem_req_data_valid,
  input                        mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0]  mem_req_data_bits,
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

  input                        mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]   mem_resp_data,
  input [`MEM_TAG_BITS-1:0]    mem_resp_tag

);

wire i_stall_n;
wire d_stall_n;

wire ic_mem_req_valid;
wire ic_mem_req_ready;
wire [`MEM_ADDR_BITS-1:0]  ic_mem_req_addr;
wire ic_mem_resp_valid;

wire dc_mem_req_valid;
wire dc_mem_req_ready;
wire dc_mem_req_rw;
wire [`MEM_ADDR_BITS-1:0]  dc_mem_req_addr;
wire dc_mem_resp_valid;

wire [(`MEM_DATA_BITS/8)-1:0]  dc_mem_req_mask;

`ifdef no_cache_mem
no_cache_mem icache (
  .clk(clk),
  .reset(reset),
  .cpu_req_valid(icache_re),
  .cpu_req_ready(i_stall_n),
  .cpu_req_addr(icache_addr[31:2]),
  .cpu_req_data(), // core does not write to icache
  .cpu_req_write(4'b0), // never write
  .cpu_resp_valid(),
  .cpu_resp_data(icache_dout)
);

no_cache_mem dcache (
  .clk(clk),
  .reset(reset),
  .cpu_req_valid((| dcache_we) || dcache_re),
  .cpu_req_ready(d_stall_n),
  .cpu_req_addr(dcache_addr[31:2]),
  .cpu_req_data(dcache_din),
  .cpu_req_write(dcache_we),
  .cpu_resp_valid(),
  .cpu_resp_data(dcache_dout)
);
assign stall =  ~i_stall_n || ~d_stall_n;

`else
cache icache (
  .clk(clk),
  .reset(reset),
  .cpu_req_valid(icache_re),
  .cpu_req_ready(i_stall_n),
  .cpu_req_addr(icache_addr[31:2]),
  .cpu_req_data(), // core does not write to icache
  .cpu_req_write(4'b0), // never write
  .cpu_resp_valid(),
  .cpu_resp_data(icache_dout),
  .mem_req_valid(ic_mem_req_valid),
  .mem_req_ready(ic_mem_req_ready),
  .mem_req_addr(ic_mem_req_addr),
  .mem_req_data_valid(),
  .mem_req_data_bits(),
  .mem_req_data_mask(),
  .mem_req_data_ready(),
  .mem_req_rw(),
  .mem_resp_valid(ic_mem_resp_valid),
  .mem_resp_data(mem_resp_data)
);

cache dcache (
  .clk(clk),
  .reset(reset),
  .cpu_req_valid((| dcache_we) || dcache_re),
  .cpu_req_ready(d_stall_n),
  .cpu_req_addr(dcache_addr[31:2]),
  .cpu_req_data(dcache_din),
  .cpu_req_write(dcache_we),
  .cpu_resp_valid(),
  .cpu_resp_data(dcache_dout),
  .mem_req_valid(dc_mem_req_valid),
  .mem_req_ready(dc_mem_req_ready),
  .mem_req_addr(dc_mem_req_addr),
  .mem_req_rw(dc_mem_req_rw),
  .mem_req_data_valid(mem_req_data_valid),
  .mem_req_data_bits(mem_req_data_bits),
  .mem_req_data_mask(mem_req_data_mask),
  .mem_req_data_ready(mem_req_data_ready),
  .mem_resp_valid(dc_mem_resp_valid),
  .mem_resp_data(mem_resp_data)
);
assign stall =  ~i_stall_n || ~d_stall_n;

//                           ICache 
//                         /        \
//   Riscv151 --- Memory151          Arbiter <--> ExtMemModel
//                         \        /
//                           DCache 

riscv_arbiter arbiter (
  .clk(clk),
  .reset(reset),
  .ic_mem_req_valid(ic_mem_req_valid),
  .ic_mem_req_ready(ic_mem_req_ready),
  .ic_mem_req_addr(ic_mem_req_addr),
  .ic_mem_resp_valid(ic_mem_resp_valid),

  .dc_mem_req_valid(dc_mem_req_valid),
  .dc_mem_req_ready(dc_mem_req_ready),
  .dc_mem_req_rw(dc_mem_req_rw),
  .dc_mem_req_addr(dc_mem_req_addr),
  .dc_mem_resp_valid(dc_mem_resp_valid),

  .mem_req_valid(mem_req_valid),
  .mem_req_ready(mem_req_ready),
  .mem_req_rw(mem_req_rw),
  .mem_req_addr(mem_req_addr),
  .mem_req_tag(mem_req_tag),
  .mem_resp_valid(mem_resp_valid),
  .mem_resp_tag(mem_resp_tag)
);
`endif

endmodule


module no_cache_mem #(
  parameter CPU_WIDTH      = `CPU_INST_BITS,                              // 32
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS - `ceilLog2(`CPU_INST_BITS/8) // 32 - ceil(log_2(32/8)) = 32 - 2 = 30
) (
  input clk,
  input reset,

  input                       cpu_req_valid,
  output                      cpu_req_ready,
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,                               // [29:0]   (30 bits)
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,

  output reg                  cpu_resp_valid,
  output reg [CPU_WIDTH-1:0]  cpu_resp_data
);

  localparam DEPTH = 2*1024*1024;                                         // 2^21
  localparam WORDS = `MEM_DATA_BITS/CPU_WIDTH;                            // 128/32 = 4

  reg [`MEM_DATA_BITS-1:0] ram [DEPTH-1:0];

  wire [WORD_ADDR_BITS-`ceilLog2(WORDS)-1:0] upper_addr;                  // [30-2-1:0] = [27:0]  (28 bits)
  assign upper_addr = cpu_req_addr[WORD_ADDR_BITS-1:`ceilLog2(WORDS)];    // cpu_req_addr[29:2]

  wire [`ceilLog2(DEPTH)-1:0] ram_addr;                                   // [21-1:0]             (20 bits)
  assign ram_addr = upper_addr[`ceilLog2(DEPTH)-1:0];                     // upper_addr[20:0]

  wire [`ceilLog2(WORDS)-1:0] lower_addr;
  assign lower_addr = cpu_req_addr[`ceilLog2(WORDS)-1:0];

  wire [`MEM_DATA_BITS-1:0] read_data;
  assign read_data = (ram[ram_addr] >> CPU_WIDTH*lower_addr);

  assign cpu_req_ready = 1'b1;

  wire [CPU_WIDTH-1:0] wmask;
  assign wmask = {{8{cpu_req_write[3]}},
                  {8{cpu_req_write[2]}},
                  {8{cpu_req_write[1]}},
                  {8{cpu_req_write[0]}}};

  wire [`MEM_DATA_BITS-1:0] write_data;
  assign write_data = (ram[ram_addr] & ~({{`MEM_DATA_BITS-CPU_WIDTH{1'b0}},wmask} << CPU_WIDTH*lower_addr)) | ((cpu_req_data & wmask) << CPU_WIDTH*lower_addr);

  always @(posedge clk) begin
    if (reset) 
      cpu_resp_valid <= 1'b0;
    else if (cpu_req_valid && cpu_req_ready) begin
      if (cpu_req_write) begin
        cpu_resp_valid <= 1'b0;
        ram[ram_addr] <= write_data;
      end else begin
        cpu_resp_valid <= 1'b1;
        cpu_resp_data <= read_data[CPU_WIDTH-1:0];
      end
    end else
      cpu_resp_valid <= 1'b0;
  end

  initial
  begin : zero
    integer i;
    for (i = 0; i < DEPTH; i = i + 1)
      ram[i] = 0;
  end

endmodule


module parameterized_4_to_1_mux #(parameter N = 32)
    ( 
    input [N-1:0] a,                 // 4-bit input called a
    input [N-1:0] b,                 // 4-bit input called b
    input [N-1:0] c,                 // 4-bit input called c
    input [N-1:0] d,                 // 4-bit input called d
    input [1:0] sel,               // input sel used to select between a,b,c,d
    output [N-1:0] out);             // 4-bit output based on input sel

   // When sel[1] is 0, (sel[0]? b:a) is selected and when sel[1] is 1, (sel[0] ? d:c) is taken
   // When sel[0] is 0, a is sent to output, else b and when sel[0] is 0, c is sent to output, else d

   // If sel == 0b00 then out := a
   assign out = sel[1] ? (sel[0] ? d : c) : (sel[0] ? b : a);

endmodule
/* Notes on ExtMemModel.v 
* Memory read takes 4 consecutive cycles of 128-bit each
* 
*/

module partialload(
    input [31:0] inst,      // inst_mem
    input [31:0] memaddr,   // wb_ALU_wire
    input [31:0] memdata,   // mem_out == dcache_dout

    output reg [31:0] data_to_reg   // pl_out ==> goes to WBMux
);

    wire [2:0] f3;
    wire [1:0] position;

    assign f3 = inst[14:12];
    assign position = memaddr[1:0];

    always @* begin
        case(f3)
            // lb
            3'b000: begin
                // TODO:
                // Based on IMM not the bottom two bits of the mem_addr!
                // Must grab immediate and grab that!
                // Wait is this actually a problem? Wait nooo!!! Because imm is already taken into account in the EX stage so ... wait... Still feels wrong no?  Still somehow passing all load tests??? HAH
                
                //  CHECK HOW MEMORY IS ORDERED, DOES ADDRESS START AT LOWEST BYTE WITHIN THE 32 BITS OF MEMDATA ? 
                case(position)
                    2'b00: data_to_reg = {{24{memdata[7]}}, memdata[7:0]};
                    2'b01: data_to_reg = {{24{memdata[15]}}, memdata[15:8]};
                    2'b10: data_to_reg = {{24{memdata[23]}}, memdata[23:16]};
                    2'b11: data_to_reg = {{24{memdata[31]}}, memdata[31:24]};
                endcase
            end
            // lbu
            3'b100: begin
                case(position)
                    2'b00: data_to_reg = {{24'b0}, memdata[7:0]};
                    2'b01: data_to_reg = {{24'b0}, memdata[15:8]};
                    2'b10: data_to_reg = {{24'b0}, memdata[23:16]};
                    2'b11: data_to_reg = {{24'b0}, memdata[31:24]};
                endcase
            end
            // lh
            3'b001: begin
                case(position)
                    2'b00: data_to_reg = {{16{memdata[15]}}, memdata[15:0]};
                    2'b01: data_to_reg = {{16{memdata[23]}}, memdata[23:8]};
                    2'b10: data_to_reg = {{16{memdata[31]}}, memdata[31:16]};
                endcase
            end
            // lhu
            3'b101: begin
                case(position)
                    2'b00: data_to_reg = {{16'b0}, memdata[15:0]};
                    2'b01: data_to_reg = {{16'b0}, memdata[23:8]};
                    2'b10: data_to_reg = {{16'b0}, memdata[31:16]};
                endcase
            end
            // lw
            3'b010: data_to_reg = memdata;

        endcase

    end


endmodule
/* 1. correctly extract bits from register: bits are always stored in the lowest position in register 
*  2. correctly store bits into memroy into correct position: use concatenation + a bit mask
*/

module partialstore (
    input [31:0] inst,      // instruction from execute stage
    input [31:0] memaddr,   // dcache_addr
    input [31:0] regdata,   // rdata2_ex input
    input MemWEn,           // Wire from control logic telling partial store to store

    output reg [3:0] bitmask,   // dcache_we
    output reg [31:0] data_to_mem   // dcache_din

);

wire [2:0] f3;
wire [6:0] opcode;
wire [1:0] position;
wire isstore;

assign position = memaddr[1:0];
assign opcode = inst[6:0];
assign f3 = inst[14:12];
assign isstore = (opcode == 7'b0100011 && MemWEn);


always @* begin
    bitmask = 4'b0000;  // Need to make sure the default case is that if it is not a store instruction don't enable write!
    if (isstore) begin
        case(f3)
            // sb
            3'b000: begin
                case(position)
                    2'b00: begin
                        data_to_mem = {{24'b0}, regdata[7:0]};
                        bitmask = 4'b0001;
                    end
                    2'b01: begin
                        data_to_mem = {{16'b0}, regdata[7:0], {8'b0}};
                        bitmask = 4'b0010;
                    end
                    2'b10: begin 
                        data_to_mem = {{8'b0}, regdata[7:0], {16'b0}};
                        bitmask = 4'b0100;
                    end
                    2'b11: begin
                        data_to_mem = {regdata[7:0], {24'b0}};
                        bitmask = 4'b1000;
                    end
                endcase
            end
            // sh
            3'b001: begin
                case(position)
                    2'b00: begin
                        data_to_mem = {{16'b0}, regdata[15:0]};
                        bitmask = 4'b0011;
                    end
                    2'b10: begin
                        data_to_mem = {regdata[15:0], {16'b0}};
                        bitmask = 4'b1100;
                    end
                endcase
            end

            // sw
            3'b010: begin
                data_to_mem = regdata;
                bitmask = 4'b1111;  // Forgot to make sure bitmask was all 1's for sw
            end
        endcase

    end
end

endmodule
// Module: RegFile
// Desc:   Reads from rs1/rs2 and writes wdata to rd based on RegWEn
// Inputs: clk: clk
//         reset: resets register
//         rd: return destination ? 
//         rs1: read 1
//         rs2: read 2
//         RegWEn: controls write, write when high
//         wdata: data to write
//         rdata1: data from rs1
//         rdata2: data from rs2
// Outputs: ALUop: Selects the ALU's operation

// what files to include ??


module RegFile(
    input clk,
    input reset,
    input [4:0] rd,
    input [4:0] rs1,
    input [4:0] rs2,
    input RegWEn,
    input [31:0] wdata,
    output reg [31:0] rdata1,
    output reg [31:0] rdata2
);
    reg [31:0] regfile [31:0];

    assign rdata2 = regfile[rs2];
    assign rdata1 = regfile[rs1];

    integer i;
    
    always @(posedge clk or posedge reset) begin
        if (reset) begin
            for (i = 0; i < 32; i = i + 1) begin
                regfile[i] <= 32'b0;
            end
        end else if (rd == 0) begin
            regfile[rd] <= 32'b0;
        end else if (RegWEn) begin
            regfile[rd] <= wdata;
        end
   end

    
    property assertion_reg_x0;
        @(posedge clk) 
        regfile[0] == 32'b0;
    endproperty

    assert property (assertion_reg_x0);

endmodule 



/*
localparam size = 32 * 32;

reg [size - 1 : 0] bigregin;
wire [size - 1 : 0] bigregout;

REGISTER_R_CE #(.N(size), .INIT({size{1'b0}})) bigregister (.q(bigregout), .d(bigregin), .rst(reset), .ce(RegWEn), .clk(clk));

// reset + write 
always @(posedge clk or posedge reset) begin
    if (reset) begin
        // bigregister resets to INIT
        bigregin <= {size{1'b0}};
    end else if (RegWEn) begin
        bigregin <= {bigregin[0: (rd * 32) - 1], /*bigregin[(rd + 1) * 32 - 1 : rd * 32] <=// wdata, bigregin[((rd + 1) * 32) : size - 1]};
    end
end

assign rdata1 = bigregout[(rs1 + 1) * 32 - 1 : rs1 * 32];  
assign rdata2 = bigregout[(rs2 + 1) * 32 - 1 : rs2 * 32];  */


/*
localparam size = 32;

reg [size - 1 : 0] bigregin;
wire [size - 1 : 0] bigregout;
wire CE;

REGISTER_R_CE #(.N(size), .INIT({size{1'b0}})) bigregister (.q(bigregout), .d(bigregin), .rst(reset), .ce(CE), .clk(clk));

// reset + write 
always @(posedge clk or posedge reset) begin
    if (RegWEn) begin
        bigregister[rd].ce <= RegWEn;
        if (rd == 0) begin
            bigregister[rd].d <= 0;
        end else begin
            bigregister[rd].d <= wdata;
        end
    end
end

always @(posedge clk) begin
    if (rs1 == 0) begin
        rdata1 <= {size{1'b0}};
    end else begin
        rdata1 <= bigregister[rs1].q
    end

    if (rs2 == 0) begin
        rdata2 <= {size{1'b0}};
    end else begin
        rdata2 <= bigregister[rs2].q
    end
end */

// Module:  Control Logic
// Desc:    Calculates all the control logic to feed into pipelines
// Inputs:  clk : clk
//          reset: rest
//          BrUn : branch unsigned
//          ...
// Outputs: ...

// what files to include ??

module control_logic(
    input clk,
    input reset,
    input stall,

    // IF/ID Stage
    output icache_re,
    input [31:0] inst_if,
    output reg [1:0] rdata1Sel,
    output reg [1:0] rdata2Sel,
    output reg icache_sel,

    // EX Stage
    output reg flush_ex,
    output BrUn,
    input BrEq,
    input BrLt,
    output reg ASel,
    output reg BSel,
    output [6:0] opcode_ex,
    output [2:0] funct,
    output add_rshift_type,
    input [31:0] inst_ex,
    output MemWEn,
    output reg dcache_din_Sel,
    output reg dcache_addr_Sel,

    // MEM/WB Stage
    output dcache_re,
    output [3:0] dcache_we,
    output reg flush_mem,
    output RegWEn,
    output reg [1:0] PCSel,
    output reg [1:0] WBSel,
    input [31:0] inst_mem,

    // Can keep for now, will NOT be used for our "single stage"
    // Extra fun instruction register for forwarding! :D
    input [31:0] inst_wb
);
    // Decoding of the instructions for each stage:

    // IF/ID stage decoding

    wire [6:0] if_opcode;
    wire [2:0] if_f3;
    wire [6:0] if_f7;
    wire [4:0] if_rd, if_rs1, if_rs2;

    assign if_opcode = inst_if[6:0];
    assign if_f3 = inst_if[14:12];
    assign if_f7 = inst_if[31:25];  // Only for R, I* type instructions
    assign if_rd = inst_if[11:7];   // Only for R, I, I*, U, and J type instructions
    assign if_rs1 = inst_if[19:15]; // Only for R, I, I*, S, and B type instructions
    assign if_rs2 = inst_if[24:20]; // Only for R, S and B type instructions

    // EX stage decoding

    wire [6:0] ex_opcode;
    wire [2:0] ex_f3;
    wire [4:0] ex_rd, ex_rs1, ex_rs2;
    wire [6:0] ex_f7;

    assign ex_opcode = inst_ex[6:0];
    assign ex_f3 = inst_ex[14:12];
    assign ex_f7 = inst_ex[31:25];  // Only for R, I* type instructions
    assign ex_rd = inst_ex[11:7];   // Only for R, I, I*, U, and J type instructions
    assign ex_rs1 = inst_ex[19:15]; // Only for R, I, I*, S, and B type instructions
    assign ex_rs2 = inst_ex[24:20]; // Only for R, S and B type instructions

    // MEM stage decoding

    wire [6:0] mem_opcode;
    wire [2:0] mem_f3;
    wire [6:0] mem_f7;
    wire [4:0] mem_rd, mem_rs1, mem_rs2;

    assign mem_opcode = inst_mem[6:0];
    assign mem_f3 = inst_mem[14:12];
    assign mem_f7 = inst_mem[31:25];  // Only for R, I* type instructions
    assign mem_rd = inst_mem[11:7];   // Only for R, I, I*, U, and J type instructions
    assign mem_rs1 = inst_mem[19:15]; // Only for R, I, I*, S, and B type instructions
    assign mem_rs2 = inst_mem[24:20]; // Only for R, S and B type instructions

    // "WB" Stage decoding

    wire [6:0] wb_opcode;
    wire [2:0] wb_f3;
    wire [4:0] wb_rd, wb_rs1, wb_rs2;
    wire [6:0] wb_f7;

    assign wb_opcode = inst_wb[6:0];
    assign wb_f3 = inst_wb[14:12];
    assign wb_f7 = inst_wb[31:25];  // Only for R, I* type instructions
    assign wb_rd = inst_wb[11:7];   // Only for R, I, I*, U, and J type instructions
    assign wb_rs1 = inst_wb[19:15]; // Only for R, I, I*, S, and B type instructions
    assign wb_rs2 = inst_wb[24:20]; // Only for R, S and B type instructions
    
    // ===============================================================
    // ================ IF/ID Stage - Control Signals ================
    // ===============================================================

    // We need to pipeline if a branch should be taken from the EX to the MEM/WB stage
    wire taken_ex, taken_mem;
    
    taken_not_taken taken_not_taken(
        // Inputs
        .clk(clk), .reset(reset), .stall(stall),
        .inst_ex(inst_ex),
        .BrEq(BrEq),
        .BrLt(BrLt),

        // Outputs
        .taken_ex(taken_ex),
        .taken_mem(taken_mem)
    );    
// when to NOP // how to correctly set PC after a NOP ?
    reg PC_stall;

    // Needed to fix the MEM -> ALU data forwarding stalling control logic TIMING

    // PCSel logic:
    // if (reset) ==> 2 (pc_wire)
    // else if (branch/JAL/JALR) ==> 1 (wb_ALU_wire)
    // else ==> 0


    wire reset_reg_out;

    REGISTER_R_CE #(.N(1), .INIT(1'b0)) reset_reg 
        (.q(reset_reg_out), .d(reset), .ce(!stall), .rst(1'b0), .clk(clk));

    always @* begin

        if (reset_reg_out || stall || PC_stall || 
            // LW
            inst_ex == `OPC_LOAD || 
            // branches
            inst_ex == `OPC_BRANCH ||
            // jumps 
            inst_ex == `OPC_JAL ||
            inst_ex == `OPC_JALR
            ) begin
            PCSel = 2'b10;      // stall
            icache_sel = 1'b1;  // insert NOP
        end else if (mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR || mem_opcode == `OPC_BRANCH) begin
            PCSel = 2'b01;      // jump next cycle
            icache_sel = 1'b1;  // insert NOP immidiately
        end else begin
            PCSel = 2'b0;
            icache_sel = 1'b0;
        end
    end

    // IMEM ready/valid signal
    
    assign icache_re = (reset || PC_stall || stall) ? 1'b0 : 1'b1;

    // Data forwarding logic for muxes

    // rdata1Sel:
    // 00 - regfile
    // 01 - ALU_out
    // 10 - wdata_wb
    // 11 - (unknown)

    always @* begin
        // Always initialize with no data forwarding: regfile
        rdata1Sel = 2'b00;

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin

            // ALU -> ALU
            // Check if a data hazard might exist. Requirements:
            // 2nd instruction (IF/ID stage) rs1 and 1st instruction (EX stage) rd registers match.
            // 2nd instruction must not be a JAL/AUIPC/LUI instruction.
            // 1st instruction must have a rd register but cannot be a load instruction,
            // so only R, U, and I type instructions (excluding load instructions).

            if (if_rs1 == ex_rd &&
                if_rs1 != 5'b0 &&
                if_opcode != `OPC_JAL &&
                if_opcode != `OPC_AUIPC &&
                if_opcode != `OPC_LUI &&
                (ex_opcode == `OPC_ARI_RTYPE ||
                ex_opcode == `OPC_ARI_ITYPE ||
                ex_opcode == `OPC_AUIPC ||
                ex_opcode == `OPC_LUI)) begin
                    rdata1Sel = 2'b01; 
            end 
            // MEM -> ALU
            // 1 cycle stall + forward, for load words
            // or
            // 2 cycle apart hazards so be more general!

            else if (if_rs1 == mem_rd &&
                    if_rs1 != 5'b0 &&
                    if_opcode != `OPC_JAL &&
                    if_opcode != `OPC_AUIPC &&
                    if_opcode != `OPC_LUI &&
                    (mem_opcode == `OPC_LOAD || 
                    mem_opcode == `OPC_ARI_RTYPE ||
                    mem_opcode == `OPC_ARI_ITYPE ||
                    mem_opcode == `OPC_AUIPC ||
                    mem_opcode == `OPC_LUI)) begin 
                    rdata1Sel = 2'b10;
            end
        end
    end 

    
    // rdata1Sel:
    // 00 - regfile
    // 01 - ALU_out
    // 10 - wdata_wb
    // 11 - (unknown)
    
    always @* begin
        // Always initialize with no data forwarding: regfile
        rdata2Sel = 2'b00;

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin

            // ALU -> ALU
            // Check if a data hazard might exist. Requirements:
            // 2nd instruction (IF/ID stage) rs1 and 1st instruction (EX stage) rd registers match.
            // 2nd instruction must not be a JAL/AUIPC/LUI instruction.
            // 1st instruction must have a rd register but cannot be a load instruction,
            // so only R, U, and I type instructions (excluding load instructions).

            if (if_rs2 == ex_rd &&
                if_rs2 != 5'b0 &&
                if_opcode != `OPC_JAL &&
                if_opcode != `OPC_JALR &&
                if_opcode != `OPC_AUIPC &&
                if_opcode != `OPC_LUI &&
                if_opcode != `OPC_ARI_ITYPE &&
                if_opcode != `OPC_LOAD && 
                (ex_opcode == `OPC_ARI_RTYPE ||
                ex_opcode == `OPC_ARI_ITYPE ||
                ex_opcode == `OPC_AUIPC ||
                ex_opcode == `OPC_LUI)) begin
                    rdata2Sel = 2'b01; 
            end 

            // MEM -> ALU
            // 1 cycle stall + forward, for load words
            // or
            // 2 cycle apart hazards so be more general!

            else if (if_rs2 == mem_rd &&
                    if_rs2 != 5'b0 &&
                    if_opcode != `OPC_JAL &&
                    if_opcode != `OPC_JALR &&
                    if_opcode != `OPC_AUIPC &&
                    if_opcode != `OPC_LUI &&
                    if_opcode != `OPC_ARI_ITYPE &&
                    if_opcode != `OPC_LOAD && 
                    (mem_opcode == `OPC_LOAD || 
                    mem_opcode == `OPC_ARI_RTYPE ||
                    mem_opcode == `OPC_ARI_ITYPE ||
                    mem_opcode == `OPC_AUIPC ||
                    mem_opcode == `OPC_LUI)) begin 
                    rdata2Sel = 2'b10;
            end
        end
    end

    // ===============================================================
    // ================== EX Stage - Control Signals =================
    // ===============================================================

    // 1 if the instruction is (sltu/sltiu/bltu/bgeu) * (doesn't matter) otherwise
    assign BrUn =   (reset) ? 1'b0 : 
                    (ex_opcode == `OPC_ARI_RTYPE && ex_f3 == `FNC_SLTU) ||
                    (ex_opcode == `OPC_ARI_ITYPE && ex_f3 == `FNC_SLTU) ||  // TODO: check if `FNC_SLTU is the correct Funct3 code for sltiu???
                    (ex_opcode == `OPC_BRANCH && ex_f3 == `FNC_BLTU) ||
                    (ex_opcode == `OPC_BRANCH && ex_f3 == `FNC_BGEU);

    /* 
    ASel    := 1 ==> A := rdata1, 
            := 0 ==> A := PC  

    ASel := 1 if 
        instruction type == r type, i type, I* type, store type 
        instruction is a JALR

    ASel := 0 if
        instruction type = b type
        instruction is a JAL or AUIPC instruction
    */

    assign ASel = (ex_opcode == `OPC_ARI_RTYPE) ||
                    (ex_opcode == `OPC_ARI_ITYPE) ||
                    (ex_opcode == `OPC_LOAD) ||
                    // (ex_opcode == `OPC_BRANCH) ||
                    (ex_opcode == `OPC_JALR) || 
                    (ex_opcode == `OPC_CSR) || 
                    (ex_opcode == `OPC_STORE);

    /* 
    BSel    := 1 ==> B := imm, 
            := 0 ==> B := rdata2

    BSel := 1 if 
        instruction type is not an r type instruction
    */
    
    assign BSel = (ex_opcode == `OPC_ARI_RTYPE) ? 0 : 1;

    // ALU decoder control signals
    assign opcode_ex = ex_opcode;
    assign funct = ex_f3;
    // assign add_rshift_type = (ex_opcode == `OPC_ARI_RTYPE) ? ex_f7[5] : 1'b0;   
    assign add_rshift_type = ex_f7[5]; // should always be the inst[30] bit

    // Partial Store signal
    assign MemWEn = (ex_opcode == `OPC_STORE && 
                        ~(mem_opcode == `OPC_BRANCH && taken_mem) &&
                        ~(mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR));


    // Bypass path control signals
    
    always @* begin
        // By default: Don't forward!
        
        dcache_din_Sel = 1'b0;

        // MEM -> MEM (dcache_din, rs2)

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin
                
            if (ex_rs2 == mem_rd &&
                ex_rs2 != 5'b0 &&
                mem_opcode == `OPC_LOAD &&
                (ex_opcode == `OPC_LOAD || 
                ex_opcode == `OPC_STORE)) begin
                dcache_din_Sel = 1'b1;
            end
        end

        
        // MEM -> MEM (address input, rs1)

        // Don't forward if the last two instructions are invalid due to control hazards!
        if (mem_opcode != `OPC_JAL && mem_opcode != `OPC_JALR && 
                !(mem_opcode == `OPC_BRANCH && taken_mem)) begin
            
            dcache_addr_Sel = 1'b0;

            if (ex_rs1 == mem_rd &&
                ex_rs1 != 5'b0 &&
                mem_opcode == `OPC_LOAD &&
                (ex_opcode == `OPC_LOAD || 
                ex_opcode == `OPC_STORE)) begin
                dcache_addr_Sel = 1'b1;
            end
        end
    end

    // ===============================================================
    // ================ MEM/WB Stage - Control Signals ===============
    // ===============================================================

    // DMEM ready/valid signal

    // dcache_re := 1 ==> signal ready
    //           := 0 ==> signal not ready
    // dcache_re := 1 ==> if
    //                     instruction is ??????? always leave on ??? 
    assign dcache_re = (reset) ? 4'b0 : (ex_opcode == `OPC_LOAD && !stall) ? 1'b1: 1'b0;

    // dcache_we := 1 ==> write enabled
    //           := 0 ==> write not enabled
    // dcahce_we := 1 ==> if
    //                     instruction is store type
    assign dcache_we = (reset) ? 4'b0 : (mem_opcode == `OPC_STORE && 
                        ~(mem_opcode == `OPC_BRANCH && taken_mem) &&
                        ~(mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR)) ? 4'b1111 : 4'b0;

    // RegWEn := 1 ==> RegWEn := write enabled
    //        := 0 ==> RegWEn := write not enabled
    // RegWEn := 1 if
    //              instruction type == R, I, I*, Jal, Jalr, LUI, AUIPC, Loads???
    assign RegWEn = (reset || stall) ?  0 : (mem_opcode == `OPC_ARI_RTYPE ||
                                    mem_opcode == `OPC_ARI_ITYPE ||
                                    mem_opcode == `OPC_JAL ||
                                    mem_opcode == `OPC_JALR ||
                                    mem_opcode == `OPC_LUI ||
                                    mem_opcode == `OPC_AUIPC ||
                                    mem_opcode == `OPC_LOAD) ? 1'b1: 1'b0;

    // WBSel := 00 == > ALU_out
    //       := 01 == > PC + 4
    //       := 10 == > Mem_outPC_stall
    //       := 11 == > ??
    // WBSel := 01 if
    //              instruction is JAL
    //              instruction is JALR
    // WBSel := 00 if 
    //              instruction anything else (R, I, U)
    // WBSel := 10 if 
    //              instruction is load type

    assign WBSel = (mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR) ? 2'b01 : 
                   (mem_opcode == `OPC_LOAD) ? 2'b10 : 
                   2'b00;

    // Flushing and stalling control signals

    always @* begin
        PC_stall = 1'b0;
        flush_ex = 1'b0;
        flush_mem = 1'b0;

        if (!stall) begin
            // MEM -> ALU insert bubble
            // For when we need to insert a bubble for a MEM -> ALU (lw) Data hazard!!!
            if (ex_opcode == `OPC_LOAD &&
                if_opcode != `OPC_JAL &&
                if_opcode != `OPC_AUIPC &&
                if_opcode != `OPC_LUI  && 
                ~(mem_opcode == `OPC_BRANCH && taken_mem) &&
                ~(mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR)
            ) begin

                if (if_rs1 == ex_rd && if_rs1 != 5'b0) begin
                    PC_stall = 1'b1;
                    flush_ex = 1'b1;
                end
                // For the edge case of jalr which has only rs1 and not rs2
                else if (if_rs2 == ex_rd && 
                        if_rs2 != 5'b0 &&
                        if_opcode != `OPC_JALR) begin
                    PC_stall = 1'b1;
                    flush_ex = 1'b1;
                end
            end

            
            // Control Hazards (Branch, Jumps)
            
            // Branch
            else if (mem_opcode == `OPC_BRANCH && taken_mem) begin   
                flush_ex = 1;
                flush_mem = 1;

                
            end

            // Jumps
            else if (mem_opcode == `OPC_JAL || mem_opcode == `OPC_JALR) begin
                flush_ex = 1;
                flush_mem = 1;
            end 
        end
    end

    
endmodule 


// This module pipelines if a branch should be taken or not taken from the EX to the MEM stage

module taken_not_taken (
    input clk, reset, stall,
    input [31:0] inst_ex,
    input BrEq,
    input BrLt,

    output taken_ex, taken_mem
);
    wire [6:0] ex_opcode;
    wire [2:0] ex_f3;

    assign ex_opcode = inst_ex[6:0];
    assign ex_f3 = inst_ex[14:12];

    reg taken_ex; 
    wire taken_mem;    

    always @* begin
        if (ex_opcode == `OPC_BRANCH) begin
            if ((ex_f3 == `FNC_BEQ && BrEq) ||
                (ex_f3 == `FNC_BNE && !BrEq) ||
                (ex_f3 == `FNC_BLT && BrLt) ||
                (ex_f3 == `FNC_BGE && !BrLt) ||
                (ex_f3 == `FNC_BLTU && BrLt) ||
                (ex_f3 == `FNC_BGEU && !BrLt)) begin 
                taken_ex = 1;
            end
            else begin
                taken_ex = 0;
            end
        end
        else begin
            taken_ex = 0;
        end
    end

    REGISTER_R_CE #(.N(1), .INIT(1'b0)) taken_reg_mem
    (.q(taken_mem), .d(taken_ex), .ce(!stall), .rst(reset), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)    
endmodule/* 1. correctly extract bits from register: bits are always stored in the lowest position in register 
*  2. correctly store bits into memroy into correct position: use concatenation + a bit mask
*/

module partialstore (
    input [31:0] inst,      // instruction from execute stage
    input [31:0] memaddr,   // dcache_addr
    input [31:0] regdata,   // rdata2_ex input
    input MemWEn,           // Wire from control logic telling partial store to store

    output reg [3:0] bitmask,   // dcache_we
    output reg [31:0] data_to_mem   // dcache_din

);

wire [2:0] f3;
wire [6:0] opcode;
wire [1:0] position;
wire isstore;

assign position = memaddr[1:0];
assign opcode = inst[6:0];
assign f3 = inst[14:12];
assign isstore = (opcode == 7'b0100011 && MemWEn);


always @* begin
    bitmask = 4'b0000;  // Need to make sure the default case is that if it is not a store instruction don't enable write!
    if (isstore) begin
        case(f3)
            // sb
            3'b000: begin
                case(position)
                    2'b00: begin
                        data_to_mem = {{24'b0}, regdata[7:0]};
                        bitmask = 4'b0001;
                    end
                    2'b01: begin
                        data_to_mem = {{16'b0}, regdata[7:0], {8'b0}};
                        bitmask = 4'b0010;
                    end
                    2'b10: begin 
                        data_to_mem = {{8'b0}, regdata[7:0], {16'b0}};
                        bitmask = 4'b0100;
                    end
                    2'b11: begin
                        data_to_mem = {regdata[7:0], {24'b0}};
                        bitmask = 4'b1000;
                    end
                endcase
            end
            // sh
            3'b001: begin
                case(position)
                    2'b00: begin
                        data_to_mem = {{16'b0}, regdata[15:0]};
                        bitmask = 4'b0011;
                    end
                    2'b10: begin
                        data_to_mem = {regdata[15:0], {16'b0}};
                        bitmask = 4'b1100;
                    end
                endcase
            end

            // sw
            3'b010: begin
                data_to_mem = regdata;
                bitmask = 4'b1111;  // Forgot to make sure bitmask was all 1's for sw
            end
        endcase

    end
end

endmodule/* Notes on ExtMemModel.v 
* Memory read takes 4 consecutive cycles of 128-bit each
* 
*/

module partialload(
    input [31:0] inst,      // inst_mem
    input [31:0] memaddr,   // wb_ALU_wire
    input [31:0] memdata,   // mem_out == dcache_dout

    output reg [31:0] data_to_reg   // pl_out ==> goes to WBMux
);

    wire [2:0] f3;
    wire [1:0] position;

    assign f3 = inst[14:12];
    assign position = memaddr[1:0];

    always @* begin
        case(f3)
            // lb
            3'b000: begin
                // TODO:
                // Based on IMM not the bottom two bits of the mem_addr!
                // Must grab immediate and grab that!
                // Wait is this actually a problem? Wait nooo!!! Because imm is already taken into account in the EX stage so ... wait... Still feels wrong no?  Still somehow passing all load tests??? HAH
                
                //  CHECK HOW MEMORY IS ORDERED, DOES ADDRESS START AT LOWEST BYTE WITHIN THE 32 BITS OF MEMDATA ? 
                case(position)
                    2'b00: data_to_reg = {{24{memdata[7]}}, memdata[7:0]};
                    2'b01: data_to_reg = {{24{memdata[15]}}, memdata[15:8]};
                    2'b10: data_to_reg = {{24{memdata[23]}}, memdata[23:16]};
                    2'b11: data_to_reg = {{24{memdata[31]}}, memdata[31:24]};
                endcase
            end
            // lbu
            3'b100: begin
                case(position)
                    2'b00: data_to_reg = {{24'b0}, memdata[7:0]};
                    2'b01: data_to_reg = {{24'b0}, memdata[15:8]};
                    2'b10: data_to_reg = {{24'b0}, memdata[23:16]};
                    2'b11: data_to_reg = {{24'b0}, memdata[31:24]};
                endcase
            end
            // lh
            3'b001: begin
                case(position)
                    2'b00: data_to_reg = {{16{memdata[15]}}, memdata[15:0]};
                    2'b01: data_to_reg = {{16{memdata[23]}}, memdata[23:8]};
                    2'b10: data_to_reg = {{16{memdata[31]}}, memdata[31:16]};
                endcase
            end
            // lhu
            3'b101: begin
                case(position)
                    2'b00: data_to_reg = {{16'b0}, memdata[15:0]};
                    2'b01: data_to_reg = {{16'b0}, memdata[23:8]};
                    2'b10: data_to_reg = {{16'b0}, memdata[31:16]};
                endcase
            end
            // lw
            3'b010: data_to_reg = memdata;

        endcase

    end


endmodule
// TODO: do we need to add more imports like from immgen and etc???

module Riscv151(
    input clk,
    input reset,

    // Memory system ports
    output [31:0] dcache_addr,
    output [31:0] icache_addr,
    output [3:0] dcache_we,
    output dcache_re,
    output icache_re,
    output [31:0] dcache_din,

    input [31:0] dcache_dout,
    input [31:0] icache_dout,
    input stall,              //  ~i_stall_n || ~d_stall_n ==  ~cpu_req_ready (icache) || ~cpu_req_ready (dcache)
    output [31:0] csr
);
  localparam bits = 32; // TODO: replace with `CPU_ADDR_BITS from const.vh !!!!
  localparam reg_bits = $clog2(bits);

  // Control wires

  // IF/ID Stage
  wire [31:0] icache_dout;
  wire icache_re_wire;

  // EX Stage
  wire flush_ex;
  wire BrUn;
  wire BrEq;
  wire BrLt;
  wire ASel;
  wire BSel;
  wire [6:0] opcode_ex;
  wire [2:0] funct;
  wire add_rshift_type;
  wire [31:0] inst_ex;
  wire dcache_addr_Sel_wire, dcache_din_Sel_wire;

  // MEM/WB Stage
  wire dcache_re_wire;
  wire [3:0] dcache_we_wire;
  wire flush_mem;
  wire RegWEn;
  wire [1:0] PCSel;
  wire [1:0] WBSel;
  wire [31:0] inst_mem;

  // Can keep for now, will NOT be used for our "single stage"
  // Extra fun instruction register for forwarding! :D
  wire [31:0] inst_wb;

  // End of control wires

  // ========================================================================
  // ================ Instruction Fetch/Decode Stage (IF/ID) ================
  // ========================================================================

  wire [bits-1:0] PC_4;
  wire [bits-1:0] pc_wire;
  wire [bits-1:0] next_pc;
  wire [bits-1:0] icache_inst;
  wire icache_sel;


  assign PC_4 = pc_wire + 4;

  wire [bits-1:0] wb_ALU_wire;

  parameterized_4_to_1_mux #(.N(bits)) PCMux ( 
    .a(PC_4),
    .b(wb_ALU_wire),
    .c(pc_wire),
    .d(),
    .sel(PCSel),
    .out(next_pc)
  ); 

  REGISTER_R_CE #(.N(bits), .INIT(`PC_RESET)) pc_reg 
  (.q(pc_wire), .d(next_pc), .ce(!stall), .rst(reset), .clk(clk));

  assign icache_inst = (icache_sel) ? 32'h00000013 : icache_dout;
  
  // ==================== IMEM  ====================

  // output [31:0] icache_addr,  // Address of next instruction
  // input [31:0] icache_dout,   // Instruction out
  
  // // Control logic signals
  // output icache_re,           // What does this mean???    
  // input stall,                // What does this mean?

  assign icache_addr = next_pc;
  
  assign icache_re = icache_re_wire;

  // ===============================================

  wire [bits-1:0] rdata1_if, rdata2_if;
  wire [bits-1:0] wdata_wb;

  RegFile regfile (
    .clk(clk),
    .reset(reset),
    .rd(inst_mem[11:7]),
    .rs1(icache_inst[19:15]),
    .rs2(icache_inst[24:20]),
    .RegWEn(RegWEn),
    .wdata(wdata_wb),
    .rdata1(rdata1_if),
    .rdata2(rdata2_if)
  );
  
  wire [2:0] immsel;

  immgendec immgendec (
    .opcode(icache_inst[6:0]), 
    .funct3(icache_inst[14:12]),
    .immsel(immsel)   
  );

  wire [bits-1:0] imm_if;

  immgen immgen(
    .immsel(immsel),
    .inst_31_7(icache_inst[31:7]),
    .imm(imm_if)
  );

  // Data forwarding muxes

  wire [1:0] rdata1SelWire;
  wire [bits-1:0] rdata1NextWire;

  wire [bits-1:0] ALU_out;

  parameterized_4_to_1_mux #(.N(bits)) rdata1Mux ( 
    .a(rdata1_if),
    .b(ALU_out),
    .c(wdata_wb),
    .d(),
    .sel(rdata1SelWire),
    .out(rdata1NextWire)
  ); 

  wire [1:0] rdata2SelWire;
  wire [bits-1:0] rdata2NextWire;

  parameterized_4_to_1_mux #(.N(bits)) rdata2Mux ( 
    .a(rdata2_if),
    .b(ALU_out),
    .c(wdata_wb),
    .d(),
    .sel(rdata2SelWire),
    .out(rdata2NextWire)
  ); 

  // Flushing wires 
  
  wire reset_flush_ex;
  assign reset_flush_ex = reset || flush_ex;

  // IF/ID Pipeline Registers
  
  wire [bits-1:0] pc_wire_ex;
  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) pc_reg_ex
  (.q(pc_wire_ex), .d(pc_wire), .ce(!stall), .rst(reset_flush_ex), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  wire [bits-1:0] rdata1_ex, rdata2_ex;
  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) rdata1_reg
  (.q(rdata1_ex), .d(rdata1NextWire), .ce(!stall), .rst(reset_flush_ex), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)
  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) rdata2_reg
  (.q(rdata2_ex), .d(rdata2NextWire), .ce(!stall), .rst(reset_flush_ex), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  wire [bits-1:0] imm_ex;
  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) imm_reg
  (.q(imm_ex), .d(imm_if), .ce(!stall), .rst(reset_flush_ex), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  wire [bits-1:0] inst_ex;
  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) inst_reg_ex
  (.q(inst_ex), .d(icache_dout), .ce(!stall), .rst(reset_flush_ex), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  // ========================================================================
  // ========================= Execution Stage (EX) =========================
  // ========================================================================

  branchcomp branchcomp (
    .rdata1(rdata1_ex), 
    .rdata2(rdata2_ex),
    .BrUn(BrUn),
    .BrEq(BrEq),
    .BrLt(BrLt)
  );
  
  wire [bits-1:0] a_wire;

  // 0 -> PC, 1 -> rdata1
  assign a_wire = (ASel) ? rdata1_ex : pc_wire_ex;
  
  wire [bits-1:0] b_wire;
  
  // 0 -> rdata2, 1 -> imm
  assign b_wire = (BSel) ? imm_ex : rdata2_ex; 

  wire [3:0] ALUop;

  ALUdec ALUdec(
    .opcode(opcode_ex),
    .funct(funct),
    .add_rshift_type(add_rshift_type),
    .ALUop(ALUop)
  );

  custom_ALU custom_custom_ALU(
    .A(a_wire),
    .B(b_wire),
    .ALUop(ALUop),
    .Out(ALU_out)
  );

  // Partial Store

  wire [3:0] bitmask;
  wire [31:0] ps_out;
  wire MemWEn;

  partialstore partialstore (
    // Inputs
    .inst(inst_ex),
    .memaddr(ALU_out),
    // should be wdata_wire according to bypass pipeline diagram ??? 
    // - For this "simple" pipeline don't go based off bypassed pipeline diagram.
    // - thus regdata should not be b_wire but instead rdata2_ex - LJ

    .regdata(rdata2_ex), 
    .MemWEn(MemWEn), /// FIGURE THIS OUT !!!! ==> CONTROL LOGIC 

    // Outputs
    .bitmask(bitmask),
    .data_to_mem(ps_out)
  );

  // EX Pipeline Registers

  wire reset_flush_mem;
  assign reset_flush_mem = reset || flush_mem;
  
  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) wb_alu_reg
  (.q(wb_ALU_wire), .d(ALU_out), .ce(!stall), .rst(reset_flush_mem), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  wire [bits-1:0] pc_wire_mem;

  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) pc_reg_mem
  (.q(pc_wire_mem), .d(pc_wire_ex), .ce(!stall), .rst(reset_flush_mem), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  // ==================== DMEM  ====================
  // output [31:0] dcache_addr,
  // output [3:0] dcache_we,
  // output dcache_re,
  // output [31:0] dcache_din,

  // input [31:0] dcache_dout,  
  // input stall,                // What does this mean?
  // assign dcache_addr = addr;
  wire [bits-1:0] mem_out;

  assign dcache_addr = (dcache_addr_Sel_wire) ? wdata_wb : ALU_out;       
  assign dcache_we = bitmask;         // From partial store 
  assign dcache_re = dcache_re_wire;  // From control module
  assign dcache_din = (dcache_din_Sel_wire) ? wdata_wb : ps_out;         // From partial store

  assign mem_out = dcache_dout;       // To partial load

  // ===============================================

  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) inst_reg_mem
  (.q(inst_mem), .d(inst_ex), .ce(!stall), .rst(reset_flush_mem), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  // =========================================================================
  // ==================== Memory/Writeback Stage (MEM/WB) ====================
  // =========================================================================

  wire [bits-1:0] wb_PC_4_wire;
  assign wb_PC_4_wire = pc_wire_mem + 4;


  wire [31:0] pl_out;

  partialload partialload(
    .inst(inst_mem),
    .memaddr(wb_ALU_wire),
    .memdata(mem_out),
    .data_to_reg(pl_out)
  );

  parameterized_4_to_1_mux #(.N(bits)) WBMux ( 
    .a(wb_ALU_wire),
    .b(wb_PC_4_wire),
    .c(pl_out),
    .d(),
    .sel(WBSel),
    .out(wdata_wb)
  ); 

  // ================ CSR ================ 
  // TODO: put this into the control logic darn nambit

  wire [6:0] mem_opcode;
  wire [2:0] mem_f3;
  wire [11:0] mem_imm;
  wire [4:0] mem_rs1;

  assign mem_opcode = inst_mem[6:0];
  assign mem_f3 = inst_mem[14:12];
  assign mem_imm = inst_mem[31:20];
  assign mem_rs1 = inst_mem[19:15];

  wire [4:0] ex_rs1;
  assign ex_rs1 = inst_ex[19:15];


  // If funct3 == 3'b001 then CSRRW
  // If funct3 == 3'b101 then CSRRWI

  // TODO: make sure that wb_ALU_wire is the rs1 output! So check if control logic is copying rs1 through ALU!!!
  
  wire [bits-1:0] next_csr;
  assign next_csr = (mem_opcode != `OPC_CSR) ? 32'b0 : (mem_f3 == 3'b001) ? wb_ALU_wire : {{27{1'b0}}, mem_rs1};

  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) csr_reg
  (.q(csr), .d(next_csr), .ce(!stall), .rst(reset), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)



  // ================ CSR ================

  // ===== Below register for 2 stage hazards =====
  
  wire [bits-1:0] inst_wb;

  REGISTER_R_CE #(.N(bits), .INIT({bits{1'b0}})) inst_reg_wb
  (.q(inst_wb), .d(inst_mem), .ce(!stall), .rst(reset), .clk(clk)); // TODO: figure out what signal is the CE??? (Control logic!)

  // ===============================================

  // =========================================================================
  // ============================= Control Logic =============================
  // =========================================================================

  control_logic control(
    .clk(clk),
    .reset(reset),
    .stall(stall),

    // IF/ID Stage
    .icache_re(icache_re_wire),
    .inst_if(icache_dout),
    .rdata1Sel(rdata1SelWire),
    .rdata2Sel(rdata2SelWire),
    .icache_sel(icache_sel),

    // EX Stage
    .flush_ex(flush_ex),
    .BrUn(BrUn),
    .BrEq(BrEq),
    .BrLt(BrLt),
    .ASel(ASel),
    .BSel(BSel),
    .opcode_ex(opcode_ex),
    .funct(funct),
    .add_rshift_type(add_rshift_type),
    .inst_ex(inst_ex),
    .MemWEn(MemWEn),
    .dcache_addr_Sel(dcache_addr_Sel_wire),
    .dcache_din_Sel(dcache_din_Sel_wire),

    // MEM/WB Stage
    .dcache_re(dcache_re_wire),
    .dcache_we(dcache_we_wire),
    .flush_mem(flush_mem),
    .RegWEn(RegWEn),
    .PCSel(PCSel),
    .WBSel(WBSel),
    .inst_mem(inst_mem),

    // Can keep for now, will NOT be used for our "single stage"
    // Extra fun instruction register for forwarding! :D
    .inst_wb(inst_wb)
  );

  // ========================================================================
  // ========================= Cache State Machine ==========================
  // ========================================================================

  /*
  MUST OBEY ALL READY/VALID INTERFACES !!!!


  IDLE: 
    where are we getting addresses from ;-; 
    is dcache address same as cache address ? format ? 

    10 bit addresses 
    each SRAM 8 bit address, line size 4B
    per SRAM:
      tag : 3 bits
      index : 3 bits
      offset: 2 bits

    Direct Mapped Cache:
    32-bit, 4 Kb total size, 512 bits line size
      Tag:    bits
      Index:  bits
      Offset: bits

    checking address: 
    1. go to index in cache
    2. check valid bit (where to put this info ? needs to be saved ... 
    alr part of cache input ?? )
    3. make sure tag in cache matches tag given
    4. if match, hit go to CACHE READ, if not miss go to MEM FETCH



  CACHE READ:
  MEM FETCH:
  STORE:
  
  */

  // =========================================================================
  // ============================== Assertions ===============================
  // =========================================================================


  // When reset signal is high, pc should reset to PC_RESET

  property pc_reset_check;
    @(posedge clk) reset |-> pc_wire == `PC_RESET;
  endproperty

  assert property (pc_reset_check);

  // store instructions , make sure bitmask values correspond to correct instructions (sb, sh, sw)
  property sb_check;
    @(posedge clk) 
    (mem_opcode == `OPC_STORE && mem_f3 == 000) |-> 
      (bitmask == 4'b0001 || bitmask == 4'b0010 || bitmask == 4'b0100 || bitmask == 4'b1000);
  endproperty

  assert property (sb_check);


  property sh_check;
    @(posedge clk) 
    (mem_opcode == `OPC_STORE && mem_f3 == 001) |-> 
      (bitmask == 4'b0011 || bitmask == 4'b1100);
  endproperty

  assert property (sh_check);


  property sw_check;
    @(posedge clk) 
    (mem_opcode == `OPC_STORE && mem_f3 == 010) |-> 
      (bitmask == 4'b1111);
  endproperty

  assert property (sw_check);
  // for lb instructions, upper 24 bits of data written to regfile should all be 0s/1s
  // for lh instructions, upper 16 bits of data written to regfile should all be 0s/1s

  property lb_check;
    @(posedge clk) 
    (mem_opcode == `OPC_LOAD && (mem_f3 == 000 || mem_f3 == 100) ) |-> 
      (pl_out[31:8] == 24'h000000 || pl_out[31:8] == 24'hFFFFFF);
  endproperty

  assert property (lb_check);

  property lh_check;
    @(posedge clk) 
    (mem_opcode == `OPC_LOAD && (mem_f3 == 001 || mem_f3 == 010) ) |-> 
      (pl_out[31:16] == 16'h0000 || pl_out[31:16] == 16'hFFFF);
  endproperty

  assert property (lh_check);

endmodule
module riscv_arbiter
(
  input clk,
  input reset,

  input                       ic_mem_req_valid,
  output                      ic_mem_req_ready,
  input [`MEM_ADDR_BITS-1:0]  ic_mem_req_addr,
  output                      ic_mem_resp_valid,

  input                       dc_mem_req_valid,
  output                      dc_mem_req_ready,
  input                       dc_mem_req_rw,
  input [`MEM_ADDR_BITS-1:0]  dc_mem_req_addr,
  output                      dc_mem_resp_valid,

  output                      mem_req_valid,
  input                       mem_req_ready,
  output                      mem_req_rw,
  output [`MEM_ADDR_BITS-1:0] mem_req_addr,
  output [`MEM_TAG_BITS-1:0]  mem_req_tag,
  input                       mem_resp_valid,
  input [`MEM_TAG_BITS-1:0]   mem_resp_tag
);

  assign ic_mem_req_ready = mem_req_ready;
  assign dc_mem_req_ready = mem_req_ready & ~ic_mem_req_valid;

  assign mem_req_valid = ic_mem_req_valid | dc_mem_req_valid;
  assign mem_req_rw
    = ic_mem_req_valid ? 1'b0 : dc_mem_req_rw;
  assign mem_req_addr
    = ic_mem_req_valid ? ic_mem_req_addr : dc_mem_req_addr;
  assign mem_req_tag
    = ic_mem_req_valid ? 4'd0 : 4'd1;

  assign ic_mem_resp_valid = mem_resp_valid & (mem_resp_tag == 4'd0);
  assign dc_mem_resp_valid = mem_resp_valid & (mem_resp_tag == 4'd1);

endmodule


module OneFiftyOneCoreBlackBox
(
  input clk,
  input reset,

  output                      mem_req_valid,
  input                       mem_req_ready,
  output                      mem_req_rw,
  output [`MEM_ADDR_BITS-1:0] mem_req_addr,
  output [`MEM_TAG_BITS-1:0]  mem_req_tag,

  output                      mem_req_data_valid,
  input                       mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0] mem_req_data_bits,
  output [(`MEM_DATA_BITS/8)-1:0] mem_req_data_mask,

  input                       mem_resp_valid,
  input [`MEM_TAG_BITS-1:0]   mem_resp_tag,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data,
  output [31:0]               csr
);

  wire [31:0]   dcache_addr; // From cpu of Riscv151.v
  wire [31:0]   dcache_din;  // From cpu of Riscv151.v
  wire [31:0]   dcache_dout; // From mem of Memory151.v
  wire          dcache_re;   // From cpu of Riscv151.v
  wire [3:0]    dcache_we;   // From cpu of Riscv151.v
  wire [31:0]   icache_addr; // From cpu of Riscv151.v
  wire [31:0]   icache_dout; // From mem of Memory151.v
  wire          icache_re;   // From cpu of Riscv151.v
  wire          stall;       // From mem of Memory151.v

  Memory151 mem(
    // Outputs
    .dcache_dout(dcache_dout[31:0]),
    .icache_dout(icache_dout[31:0]),
    .stall(stall),
    .mem_req_valid(mem_req_valid),
    .mem_req_rw(mem_req_rw),
    .mem_req_addr(mem_req_addr[`MEM_ADDR_BITS-1:0]),
    .mem_req_tag(mem_req_tag[`MEM_TAG_BITS-1:0]),
    .mem_req_data_valid(mem_req_data_valid),
    .mem_req_data_bits(mem_req_data_bits[`MEM_DATA_BITS-1:0]),
    .mem_req_data_mask(mem_req_data_mask[(`MEM_DATA_BITS/8)-1:0]),
    // Inputs
    .clk(clk),
    .reset(reset),
    .dcache_addr(dcache_addr[31:0]),
    .icache_addr(icache_addr[31:0]),
    .dcache_we(dcache_we[3:0]),
    .dcache_re(dcache_re),
    .icache_re(icache_re),
    .dcache_din(dcache_din[31:0]),
    .mem_req_ready(mem_req_ready),
    .mem_req_data_ready(mem_req_data_ready),
    .mem_resp_valid(mem_resp_valid),
    .mem_resp_data(mem_resp_data[`MEM_DATA_BITS-1:0]),
    .mem_resp_tag(mem_resp_tag[`MEM_TAG_BITS-1:0]));
  
  // RISC-V 151 CPU
  Riscv151 cpu(
      // Outputs
      .dcache_addr(dcache_addr[31:0]),
      .icache_addr(icache_addr[31:0]),
      .dcache_we(dcache_we[3:0]),
      .dcache_re(dcache_re),
      .icache_re(icache_re),
      .dcache_din(dcache_din[31:0]),
      // Inputs
      .clk(clk),
      .reset(reset),
      .dcache_dout(dcache_dout[31:0]),
      .icache_dout(icache_dout[31:0]),
      .csr(csr),
      .stall(stall));

endmodule


