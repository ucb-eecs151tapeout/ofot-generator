// Module: ALUdecoder
// Desc:   Sets the ALU operation
// Inputs: opcode: the top 6 bits of the instruction
//         funct: the funct, in the case of r-type instructions
//         add_rshift_type: selects whether an ADD vs SUB, or an SRA vs SRL
// Outputs: ALUop: Selects the ALU's operation
//

/**
 * List of RISC-V opcodes.
 * This file was completely rewritten from the file version that was used for MIPS. 
 * RISC-V uses far fewer opcodes than MIPS, but many more function codes.
 */

`ifndef OPCODE
`define OPCODE

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



`endif //OPCODE
/**
 * List of ALU operations.
*/
`ifndef ALUOP
`define ALUOP

`define ALU_ADD     4'd0
`define ALU_SUB     4'd1
`define ALU_AND     4'd2
`define ALU_OR      4'd3
`define ALU_XOR     4'd4
`define ALU_SLT     4'd5
`define ALU_SLTU    4'd6
`define ALU_SLL     4'd7
`define ALU_SRA     4'd8
`define ALU_SRL     4'd9
`define ALU_COPY_B  4'd10
`define ALU_XXX     4'd15

`endif //ALUOP

module ALUdec(
  input [6:0]       opcode,
  input [2:0]       funct,
  input             add_rshift_type,
  output reg [3:0]  ALUop
);

always@(*) begin
  case(opcode) 
    `OPC_NOOP: ALUop = `ALU_XXX;
    `OPC_AUIPC: ALUop = `ALU_ADD;
    `OPC_LUI: ALUop = `ALU_COPY_B;
    `OPC_JAL: ALUop = `ALU_ADD;
    `OPC_BRANCH: ALUop = `ALU_ADD;
    `OPC_LOAD: ALUop = `ALU_ADD;
    `OPC_STORE: ALUop = `ALU_ADD;
    `OPC_JALR: ALUop = `ALU_ADD;
    // Arithmetic instructions
    `OPC_ARI_RTYPE: begin
      case(funct)
        `FNC_ADD_SUB: ALUop = add_rshift_type? `ALU_SUB : `ALU_ADD;
        `FNC_SLL: ALUop = `ALU_SLL;
        `FNC_AND: ALUop = `ALU_AND;
        `FNC_SLT: ALUop = `ALU_SLT;
        `FNC_SLTU: ALUop = `ALU_SLTU;
        `FNC_XOR: ALUop = `ALU_XOR;
        `FNC_OR: ALUop = `ALU_OR;
        `FNC_SRL_SRA: ALUop = add_rshift_type? `ALU_SRA: `ALU_SRL;

        default: ALUop = `ALU_XXX;
      endcase
    end
    `OPC_ARI_ITYPE: begin
      case(funct)
        `FNC_ADD_SUB: ALUop = `ALU_ADD;
        `FNC_SLL: ALUop = `ALU_SLL;
        `FNC_AND: ALUop = `ALU_AND;
        `FNC_SLT: ALUop = `ALU_SLT;
        `FNC_SLTU: ALUop = `ALU_SLTU;
        `FNC_XOR: ALUop = `ALU_XOR;
        `FNC_OR: ALUop = `ALU_OR;
        `FNC_SRL_SRA: ALUop = add_rshift_type? `ALU_SRA: `ALU_SRL;
        default: ALUop = `ALU_XXX;
      endcase
    end

    default: ALUop = `ALU_XXX;
  endcase
end

endmodule

// Module: ALU.v
// Desc:   32-bit ALU for the RISC-V Processor
// Inputs: 
//    A: 32-bit value
//    B: 32-bit value
//    ALUop: Selects the ALU's operation 
// 						
// Outputs:// /
//    Out: The chosen function mapped to A and B.


module custom_ALU(
    input  [31:0] A,B,
    input signed [3:0] ALUop,
    output reg signed [31:0] Out,
    output reg [31:0] adder
);

  //  // Implement your ALU here, then delete this comment

always @(*) begin
    adder = A + B;
    case (ALUop)
        `ALU_ADD: Out = A + B;
        `ALU_AND: Out = A & B;
        `ALU_COPY_B: Out = B;
        `ALU_OR: Out = A | B;
        `ALU_SLL: Out = A << B[4:0];
        `ALU_SLT: Out = $signed(A) < $signed(B);
        `ALU_SLTU: Out = A < B;
        `ALU_SRA: Out = $signed(A) >>> B[4:0];
        `ALU_SRL: Out = A >> B[4:0];
        `ALU_SUB: Out = A - B;
        `ALU_XOR: Out = A ^ B;
        `ALU_XXX: Out = 0;
    endcase
end


endmodule

module branch_comp (
    input [2:0] funct3,
    input [31:0] data1,
    input [31:0] data2,
    output reg branch
);
reg eq, lt, ltu;
always @(*) begin
    // Branch comparison logic
    eq = data1 == data2;
    lt = $signed(data1) < $signed(data2);
    ltu = (data1 < data2);

    // For each funct3, set the corresponding branch signal
    case (funct3) 
        `FNC_BEQ:  branch = eq;
        `FNC_BNE:  branch = !eq;
        `FNC_BGE:  branch = !lt;
        `FNC_BGEU: branch = !ltu;
        `FNC_BLT:  branch = lt;
        `FNC_BLTU: branch = ltu;
        default:  branch = 1'b0;
    endcase
end

endmodule
mode
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

//Memory constants
`define MEM_DATA_BITS 128
`define MEM_TAG_BITS 5
`define MEM_ADDR_BITS 28
`define MEM_DATA_CYCLES 4

//CPU constants
`define CPU_ADDR_BITS 32
`define CPU_INST_BITS 32
`define CPU_DATA_BITS 32
`define CPU_OP_BITS 4
`define CPU_WMASK_BITS 16
`define CPU_TAG_BITS 15

// PC address on reset
`define PC_RESET 32'h00002000

// NOP instruction 
`define INSTR_NOP {12'd0, 5'd0, `FNC_ADD_SUB, 5'd0, `OPC_ARI_ITYPE}

// CSR addresses
`define CSR_TOHOST 12'h51E
`define CSR_HARTID 12'h50B
`define CSR_STATUS 12'h50A

// Control logic signal enums
`define PC_ASEL 1'b0 
`define REG_ASEL 1'b1 
`define REG_BSEL 3'd0 

//Immediate signal enums
`define IMM_S_BSEL 3'd1
`define IMM_I_BSEL 3'd2
`define IMM_U_BSEL 3'd3
`define IMM_UJ_BSEL 3'd4
`define IMM_B_BSEL 3'd5

// Write Back logic signal enums
`define PC_WB 2'd0
`define ALU_WB 2'd1
`define MEM_WB 2'd2
`define CSR_WB 2'd3

// Branching logic signal enums
`define NO_BRANCH 2'd0
`define COND_BRANCH 2'd1
`define JUMP_INST 2'd2


`endif //CONST

// `define CPU_ADDR_BITS 32
// `define CPU_INST_BITS 32
// `define CPU_DATA_BITS 32
// `define CPU_OP_BITS 4
// `define CPU_WMASK_BITS 16
// `define CPU_TAG_BITS 15

module cache #
(
  parameter LINES = 64,
  parameter CPU_WIDTH = `CPU_INST_BITS,
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8)
)
(
  input clk,
  input reset,
  // CPU interface - Request
  input                       cpu_req_valid,
  output                      cpu_req_ready,
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,
  
  // CPU interface - Response 
  output                      cpu_resp_valid,
  output [CPU_WIDTH-1:0]      cpu_resp_data,

  // Memory interface - Request
  output                      mem_req_valid,
  input                       mem_req_ready,
  output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr,
  output                           mem_req_rw,
  output                           mem_req_data_valid,
  input                            mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0]      mem_req_data_bits,
  // byte level masking
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

  // Memory interface - Response 
  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data
);


reg [3:0] state;
reg [64:0] valid_addr;
reg [64:0] dirty_addr;
wire [31:0] cpu_used_addr;
assign cpu_used_addr = {cpu_req_addr, 2'b00}; // 2 bits of offset

wire [3:0] wmask;  // wmask is a 4-bit signal based on the number of words (8 bits) for each word of data
wire [7:0] addr;   // address is 8-bits, based on the SRAM size (256 words)
wire [31:0] din;   // data input is 32-bits wide
wire [31:0] dout;  // data output is 32-bits wide

parameter STATE_REQUESTING_MEM = 1;
parameter STATE_REQUESTING_AND_WAITING_REQUEST = 2;
parameter STATE_IDLE = 0;

//Internal state and control registers
reg _cpu_req_ready; // adding _ for all variable that connects to the output => placeholder for reg to wire
reg _cpu_resp_valid;                      // CPU response valid flag
reg [31:0] _cpu_resp_data_cache;                // CPU response data (32 bits)
// Memory Request
reg _mem_req_valid;                       // Memory request valid flag
reg _mem_req_rw;                          // Memory request read/write flag
reg _mem_req_data_valid;                  // Memory request data valid flag
reg [127:0] _mem_req_data_bits;           // Memory request data bits (128 bits)
reg [15:0] _mem_req_data_mask;            // Memory request data mask (128 bits / 8 = 16 bytes)
reg [31:0] processed_addr;



// Memory Request Address
reg [`MEM_ADDR_BITS-1:0] _mem_req_addr;                 // Memory request address range

wire [127:0] dout_cache;
wire [127:0] din_cache;
wire [7:0] addr_cache;
wire [31:0] wmask_cache;
wire we_cache;

reg have_queue;
reg we_meta;
reg [31:0] cpu_processed_data;  


// reg [127:0] _dout_cache;
reg [127:0] _din_cache;
// reg [7:0] _addr_cache;
reg [31:0] _wmask_cache;
reg _we_cache;


// Data out from cache
wire [31:0] dout_cache_3;
wire [31:0] dout_cache_2;
wire [31:0] dout_cache_1;
wire [31:0] dout_cache_0;


sram22_256x32m4w8 cache3 (
    .clk(clk),
    // .we(we_cache || (processed_addr[24:23] == 2'b11 && cpu_req_valid)),
    .we(we_cache),
    .wmask(wmask_cache[15:12]),
    .addr(addr_cache),
    .din(din_cache[127:96]),
    .dout(dout_cache_3)
);

sram22_256x32m4w8 cache2 (
    .clk(clk),
    .we(we_cache),
    // .we(we_cache || (processed_addr[24:23] == 2'b10 && cpu_req_valid)),
    .wmask(wmask_cache[11:8]),
    .addr(addr_cache),
    .din(din_cache[95:64]),
    .dout(dout_cache_2)
);

sram22_256x32m4w8 cache1 (
    .clk(clk),
    .we(we_cache),
    // .we(we_cache || (processed_addr[24:23] == 2'b01 && cpu_req_valid)),
    .wmask(wmask_cache[7:4]),
    .addr(addr_cache),
    .din(din_cache[63:32]),
    .dout(dout_cache_1)
);

sram22_256x32m4w8 cache0 (
    .clk(clk),
    .we(we_cache),
    // .we(we_cache || (processed_addr[24:23] == 2'b00 && cpu_req_valid)),
    .wmask(wmask_cache[3:0]),
    .addr(addr_cache),
    .din(din_cache[31:0]),
    .dout(dout_cache_0)
);




// 7 bit of tag, 23 bit of index, 2 of offset
// 7 bit cache line indexing, 2 bit of sram line indexing, 2 bit of word indexing per SRAM line
reg [4:0] depth_tag;
reg [1:0] cache_tag;
reg [22:0] index;
reg [2:0] offset;
reg [31:0] pipelined_addr;


wire [3:0] wmask_meta;  // wmask is 4 bits wide
wire [5:0] addr_meta;   // addr is 32 bits wide
reg [5:0] _addr_meta;   // addr is 32 bits wide

reg [31:0] din_meta;    // din is 32 bits wide
wire [31:0] dout_meta;   // dout is 32 bits wide
reg [31:0] processed_data;
reg [31:0] processed_write_mask;


// Instantiate the SRAM module and connect the wires
sram22_64x32m4w8 metadata (
  .clk(clk),               // clk is connected directly to the module
  .we(we_meta),            // we_meta connected to we
  .wmask(wmask_meta),      // wmask_meta connected to wmask
  .addr(addr_meta),        // addr_meta connected to addr
  .din(din_meta),          // din_meta connected to din
  .dout(dout_meta)         // dout_meta connected to dout
);

reg [2:0] cycle_count;
reg [31:0] _cpu_resp_data_forward_mem;
reg do_write;
reg hit_write;
reg write_cache;
reg source_cache;
reg [1:0] cache_num;
// we need to modify the metadata
reg pipline_hit;
wire cache_idle_hit ;

// Assign the output signals to the internal signals
// successfully pipelining instruction
assign mem_req_addr = cpu_used_addr[31:4];
assign mem_req_valid = _mem_req_valid;
assign addr_cache = (state == STATE_IDLE) ?  {cpu_used_addr[31:26], cpu_used_addr[5:4]} : {processed_addr[31:26], cycle_count};
// assign addr_cache = {_mem_req_addr[31:26], cycle_count};
assign addr_meta = _addr_meta;
assign din_cache = _din_cache;
assign wmask_cache = _wmask_cache;
assign we_cache = _we_cache;
assign mem_req_data_valid = _mem_req_data_valid;
assign cpu_resp_data = (source_cache || cache_idle_hit) ? _cpu_resp_data_cache : _cpu_resp_data_forward_mem;
assign cpu_req_ready = !have_queue && _cpu_req_ready;
assign cpu_resp_valid = _cpu_resp_valid;
assign wmask_meta = (do_write || write_cache)? 4'hf : 4'h0;
assign mem_req_data_mask = _mem_req_data_mask;
assign cache_idle_hit = (dout_meta[31:9] == cpu_used_addr[31:9]) && valid_addr[cpu_used_addr[31:25]];  // Check if we have hit and the address is valid

// Cobinational Logic
always @(*) begin
    // Cache hit
    pipline_hit = cpu_req_valid && dout_meta[31:4] == cpu_used_addr[31:4] && valid_addr[cpu_used_addr[31:25]];
    // if we don't have the data in the cache, need to request from memory
    _mem_req_valid = cpu_req_valid && !cache_idle_hit;
    //Address Metadata
    _addr_meta = cpu_used_addr[31:25];
    // Metadata data input 
    din_meta = {cpu_used_addr[31:8], 8'b00000000};
    // Cache number
    cache_num = processed_addr[3:2];

    // Metadata write enable, set active on cache miss
    // we_meta = ((state == STATE_IDLE) && mem_req_valid) || ((cycle_count == 0) && mem_resp_valid);
    // we_meta = ((state == STATE_IDLE) && mem_req_valid && !cache_idle_hit) || ((state == STATE_REQUESTING_MEM && cycle_count == 1));
    we_meta = ((state == STATE_REQUESTING_MEM && cycle_count == 1));

    // Cache write enable, active on valid memory access req or cache write
    // _we_cache = (state == STATE_REQUESTING_MEM && mem_resp_valid && cycle_count != 3) || do_write;
    _we_cache = (state == STATE_REQUESTING_MEM && mem_resp_valid && cycle_count != 3);



  //Reading from cache and memory forwarding
  case (cache_num) // Select which SRAM to read froma
      2'b11: _cpu_resp_data_forward_mem = mem_resp_data[127:96]; // From cache1_3
      2'b10: _cpu_resp_data_forward_mem = mem_resp_data[95:64]; // From cache1_2
      2'b01: _cpu_resp_data_forward_mem = mem_resp_data[63:32]; // From cache1_1
      2'b00: _cpu_resp_data_forward_mem = mem_resp_data[31:0]; // From cache1_0
  endcase

  // Which cache we are reading from
  case (cache_num) 
      0: _cpu_resp_data_cache <= dout_cache_0;
      1: _cpu_resp_data_cache <= dout_cache_1;
      2: _cpu_resp_data_cache <= dout_cache_2;
      3: _cpu_resp_data_cache <= dout_cache_3;
  endcase
end


// Modified generate block for handling data writes to individual SRAMs
always @(*) begin
      // Writing to cache from memory
    if (mem_resp_valid) begin
        // Memory to cache write path
            // Select which 32-bit word to write to
            _din_cache[31:0]   = (|cpu_req_write && processed_addr[9:8] == 0) ? 
                                  ((mem_resp_data[31:0] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  mem_resp_data[31:0];
            _din_cache[63:32]  = (|cpu_req_write && processed_addr[9:8] == 1) ? 
                                  ((mem_resp_data[63:32] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  mem_resp_data[63:32];
            _din_cache[95:64]  = (|cpu_req_write && processed_addr[9:8] == 2) ? 
                                  ((mem_resp_data[95:64] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  mem_resp_data[95:64];
            _din_cache[127:96] = (|cpu_req_write && processed_addr[9:8] == 3) ? 
                                  ((mem_resp_data[127:96] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  mem_resp_data[127:96];
    end 
    // Writing to cache from CPU
    else if (cpu_req_valid && do_write) begin
        // CPU to cache write path
            _din_cache[31:0]   = (|cpu_req_write && processed_addr[9:8] == 0) ? 
                                  ((dout_cache[31:0] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  dout_cache_0;
            _din_cache[63:32]  = (|cpu_req_write && processed_addr[9:8] == 1) ? 
                                  ((dout_cache[63:32] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  dout_cache_1;
            _din_cache[95:64]  = (|cpu_req_write && processed_addr[9:8] == 2) ? 
                                  ((dout_cache[95:64] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  dout_cache_2;
            _din_cache[127:96] = (|cpu_req_write && processed_addr[9:8] == 3) ? 
                                  ((dout_cache[127:96] & ~processed_write_mask) | (cpu_processed_data & processed_write_mask)) :
                                  dout_cache_3;
    end
end



// metadata : 28 bits of tag, las 2 bits representing the number of iteration completed in the line

always @(posedge clk) begin
  if (reset) begin
    valid_addr <= 0;
    state <= STATE_IDLE;
    dirty_addr <=0;
    _cpu_req_ready <= 1;
    have_queue <= 0;
    _mem_req_data_mask <= 16'h0000;
    _mem_req_data_valid <= 0;
    // _mem_req_addr <= 0;
    _cpu_resp_valid <= 0;
    _mem_req_valid <= 0;
    _wmask_cache <= 0;
    do_write <= 0;
  end else begin
      case (state)
        STATE_REQUESTING_MEM: begin
          _wmask_cache <= 32'hffff; // write all the data
          have_queue <= 0; // we don't have any queue yet
          if (_cpu_resp_valid) _cpu_resp_valid = 0; // reset the cpu response valid
          
          
          // If we already have data ready, set the resp to valid and write to cache
          if (cycle_count == processed_addr[5:4] + 1 && cpu_req_valid) begin
            _cpu_resp_valid <= 1;
            do_write <= |mem_req_rw;
            source_cache <= 0;
          end    
          
          // _addr_cache <= {_mem_req_addr[31:26], cycle_count};
          cycle_count <= cycle_count + 1;

          // If cycle is 3, we have all the data
          if (cycle_count == `MEM_DATA_CYCLES - 1) begin
            cycle_count <= 0; // reset the cycle count
            _mem_req_data_valid <= 0; // reset the data valid
            valid_addr[processed_addr[31:25]] <= 1; // the cache valid bit is set to 1
            state <= STATE_IDLE; // go back to idle
            write_cache <= 0; // we are not writing to cache
          end
        end
        
        STATE_IDLE: begin
          // if (do_write) do_write = 0;
          _cpu_req_ready = 1; // we are ready to accept new request
          _wmask_cache = 0; //Mask is 0 by default
          // if (do_write) ; // reset do_write
          if (!cpu_req_valid) state <= STATE_IDLE; // if no request, stay idle
          else begin
            // check if valid
            // write_cache <= 1;
            processed_addr <= cpu_used_addr;  // Address for accessing
            processed_data <= cpu_req_data;
            processed_write_mask <= {{8{cpu_req_write[3]}}, {8{cpu_req_write[2]}}, {8{cpu_req_write[1]}}, {8{cpu_req_write[0]}}};
            cpu_processed_data <= cpu_req_data;

            //if we have the data in the cache
            if (cache_idle_hit) begin
              _cpu_req_ready <= 1; // we are ready to accept new request
              state <= STATE_IDLE; // stay idle
              do_write <= |mem_req_rw; // if we have a write request
              _cpu_resp_valid <= 1; // we have the data valid 
              source_cache <= 1; // Cache is the source
               
            end else begin // if we don't have the data in the cache
              _cpu_resp_valid <= (0 == cpu_used_addr[5:4]); // If at cycle 0 its already valid
              do_write <= 0;
              _cpu_req_ready <= 0;
              // _mem_req_valid <= 1;
              _mem_req_addr <= cpu_used_addr[31:4]; // since one address line contains 4 word
              // _addr_cache <= {cpu_used_addr[31:24], cpu_used_addr[8:7]};
              _wmask_cache <= 32'hffff; // write all the data
              // _we_cache <= 1;
              state <= STATE_REQUESTING_MEM;
              cycle_count <= 0; // start the cycle count
              write_cache <= 1; // write to cache
            end
          end 
        end
        
        default: begin
            state <= STATE_IDLE;
            valid_addr <= 0;
            dirty_addr <= 0;
            cycle_count <= 0;
            write_cache <= 0;
            _cpu_resp_valid <= 0;
            _cpu_req_ready <= 0;
            do_write <= 0;

            // _mem_req_valid <= 0;
        end
    endcase
  end
end
endmodule

module decoder_logic (
    input [31:0] inst,
    input reset,
    output reg ASel, //reg for I-type, R-type, load & store, J, csr; pc for b type, 0 for alu, 1 for pc
    output reg [2:0] Bsel, //corresponding types, or reg if r type; // 0 if register, else is the correspoding immediate
    
    output reg [1:0] PCSel, // inst = jump || branch && cond is true // 0 if no branching, 1 if branch, 2 if jump
    output reg [1:0] WBSel, // 0 for pc, 1 for alu, 2 for mem
    output reg [4:0] rd,
    output reg [4:0] rs1,
    output reg [4:0] rs2,
    output [2:0] fn3,
    output reg [3:0] memW,
    output reg [3:0] memR,
    output reg [6:0] opcode 
);


assign opcode = reset? 0 : inst[6:0];
assign fn3 = reset? 0 : inst[14:12];
assign rs1 = reset? 0 : inst[19:15];
assign rs2 = reset? 0 : inst[24:20];

//assigning the values of the control signals
always @(*) begin
    if (!reset) begin
        case(opcode)
            `OPC_ARI_ITYPE: begin
                ASel = `REG_ASEL;
                Bsel = `IMM_I_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                memR = 0;
            end
            `OPC_ARI_RTYPE: begin
                ASel = `REG_ASEL;
                Bsel = `REG_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_LOAD: begin
                ASel = `REG_ASEL;
                Bsel = `IMM_I_BSEL; // same format
                WBSel = `MEM_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                case (fn3)
                    `FNC_LB: memR = 4'b0001;
                    `FNC_LH: memR = 4'b0011;
                    `FNC_LW: memR = 4'hf;
                    `FNC_LHU: memR = 4'b0011;
                    `FNC_LBU: memR = 4'b0001;
                endcase
            end

            `OPC_STORE : begin
                ASel = `REG_ASEL;
                Bsel = `IMM_S_BSEL;
                WBSel = `ALU_WB;
                case (fn3)
                    `FNC_SB: memW = 4'b0001;
                    `FNC_SH: memW = 4'b0011;
                    `FNC_SW: memW = 4'hf;
                endcase
                PCSel = 0;
                rd = 0;
                memR = 0;
            end

            `OPC_JAL : begin
                ASel = `PC_ASEL;
                Bsel = `IMM_UJ_BSEL;
                WBSel = `PC_WB;
                memW = 0;
                PCSel = `JUMP_INST;
                rd = inst[11:7];
                memR = 0;
            end
            `OPC_BRANCH : begin
                ASel = `PC_ASEL;
                Bsel = `IMM_B_BSEL;
                WBSel = 0;
                memW = 0;
                PCSel = `COND_BRANCH;
                rd = 0;
                memR = 0;
            end

            `OPC_LUI : begin
                ASel = `REG_ASEL;
                Bsel = `IMM_U_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = 0;
                rd = inst[11:7];
                memR = 0;
            end
            `OPC_JALR : begin
                ASel = `REG_ASEL;
                Bsel = `IMM_I_BSEL;
                WBSel = `PC_WB;
                memW = 0;
                PCSel = `JUMP_INST;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_CSR: begin
                ASel = `REG_ASEL;
                Bsel = `REG_BSEL;
                WBSel = `CSR_WB;
                memW = 0;
                PCSel = `NO_BRANCH;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_AUIPC: begin
                ASel = `PC_ASEL;
                Bsel = `IMM_U_BSEL;
                WBSel = `ALU_WB;
                memW = 0;
                PCSel = `NO_BRANCH;
                rd = inst[11:7];
                memR = 0;
            end

            `OPC_NOOP: begin
                ASel = `REG_ASEL;
                Bsel = `REG_BSEL;
                WBSel = `CSR_WB;
                memW = 0;
                PCSel = `NO_BRANCH;
                rd = 0;
                memR = 0;
            end

        endcase
    end
    else begin
        ASel = 0;
        Bsel = 0;
        WBSel = 0;
        memW = 0;
        PCSel = 0;
        rd = 0;
        memR = 0;
    end
end

endmodule

module ExtMemModel
(
  input         clk,
  input         reset,

  // Read/Write Address request from CPU
  input                      mem_req_valid,
  output                     mem_req_ready,
  input                      mem_req_rw, // HIGH: Write, LOW: Read
  input [`MEM_ADDR_BITS-1:0] mem_req_addr,
  input [`MEM_TAG_BITS-1:0]  mem_req_tag,

  // Write data request from CPU
  input                          mem_req_data_valid,
  output                         mem_req_data_ready,
  input [`MEM_DATA_BITS-1:0]     mem_req_data_bits,
  input [(`MEM_DATA_BITS/8)-1:0] mem_req_data_mask,

  // Read data response to CPU
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

module forward_logic (
    input [31:0] data_in,
    input [4:0] rs1_decode,
    input [4:0] rs2_decode,
    input [4:0] rd, // make sure that this is null when the rd is not used
    output use_forward_on_mux1,
    output use_forward_on_mux2, 
    output [31:0] data_out
);
//if rd is not used, use_forward_on_mux1 and use_forward_on_mux2 should be 0
assign use_forward_on_mux1 = (rd == rs1_decode && rd != 0); //rd can't be 0
assign use_forward_on_mux2 = (rd == rs2_decode && rd != 0);

assign data_out = data_in; //default value

endmodule
module imm_gen (
    input [31:0] inst,
    output [31:0] s_type,
    output [31:0] i_type,
    output [31:0] u_type,
    output [31:0] uj_type,
    output [31:0] b_type
);
    //Defining the extensions
    wire [19:0] extension_s;
    wire [19:0] extension_i;
    wire [19:0]  extension_uj;
    assign extension_s = {20{inst[31]}};
    assign extension_i = {20{inst[31]}};
    assign extension_uj = {12{inst[31]}};
    
    //Assigning the types with extensions
    assign s_type = {extension_s, inst[31:25], inst[11:7]};
    assign i_type = {extension_i, inst[31:20]};
    assign u_type = {inst[31:12], 12'b0};
    assign uj_type = {extension_uj, inst[19:12], inst[20], inst[30:21], 1'b0};
    assign b_type =  {{20{inst[31]}}, inst[7], inst[30:25], inst[11:8], 1'b0};

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
  parameter CPU_WIDTH      = `CPU_INST_BITS,
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS - `ceilLog2(`CPU_INST_BITS/8)
) (
  input clk,
  input reset,

  input                       cpu_req_valid,
  output                      cpu_req_ready,
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,

  output reg                  cpu_resp_valid,
  output reg [CPU_WIDTH-1:0]  cpu_resp_data
);

  localparam DEPTH = 2*1024*1024;
  localparam WORDS = `MEM_DATA_BITS/CPU_WIDTH;

  reg [`MEM_DATA_BITS-1:0] ram [DEPTH-1:0];

  wire [WORD_ADDR_BITS-`ceilLog2(WORDS)-1:0] upper_addr;
  assign upper_addr = cpu_req_addr[WORD_ADDR_BITS-1:`ceilLog2(WORDS)];

  wire [`ceilLog2(DEPTH)-1:0] ram_addr;
  assign ram_addr = upper_addr[`ceilLog2(DEPTH)-1:0];

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


module reg_file #(
    parameter NUM_REGS = 32, 
    parameter ADDR_WIDTH = 5
) (
    input WEn,         
    input clk,         
    input reset,      
    input [ADDR_WIDTH-1:0] addr1, 
    input [ADDR_WIDTH-1:0] addr2,
    //Write back data
    input [ADDR_WIDTH-1:0] write_addr, 
    input [31:0] d_in,  
    //rs1 and rs2
    output [31:0] rs1,
    output [31:0] rs2
);

//NUM_REGS total 32 bit registers
reg [31:0] registers [NUM_REGS-1:0];
genvar i;

//Reset Registers
generate
    for (i = 0; i < NUM_REGS; i = i + 1) begin
        always @(posedge clk or posedge reset) begin
            if (reset) begin
                registers[i] <= 32'b0;
            end
        end
    end
endgenerate

//Write to registers
always @(posedge clk) begin
    if (!reset) begin
        if (write_addr != 0 && WEn) begin
            registers[write_addr] <= d_in;
        end
    end
end

//Assign rs1 and rs2
assign rs1 = registers[addr1];
assign rs2 = registers[addr2];
endmodule

module Riscv151(
    input clk,
    input reset,

    // Memory system ports
    output reg [31:0] dcache_addr,
    output reg [31:0] icache_addr,
    output reg [3:0] dcache_we,
    output dcache_re,
    output icache_re,
    output reg [31:0] dcache_din,
    input [31:0] dcache_dout,
    input [31:0] icache_dout,
    input stall,
    output reg [31:0] csr
);

//Program counter and pipeline instruction registers
reg [31:0] queried_pc;
reg [31:0] inst_id;
reg [31:0] inst_ex;


//Decode Stage
reg Asel_id;
reg [2:0] Bsel_id;
reg [31:0] pc_id;
reg [1:0] PCSel_id;
reg [1:0] WBSel_id;
reg [4:0] rd_id;
reg [3:0] memW_id;
reg [3:0] memR_id;   
reg signed [31:0] non_forward_data_id;
reg [2:0] fn3_id;
reg [6:0] opcode_id;


//ALU/Execution
reg signed [31:0] op_1, op_2, rs1_data_ex, rs2_data_ex; // synchronous
reg Asel_ex, add_rshift_type;
reg [2:0] Bsel_ex;
reg [31:0] pc_ex;
reg [1:0] PCSel_ex;
reg [1:0] PCsel_tmp;
reg [1:0] WBSel_ex;
reg [4:0] rd_ex;
reg [3:0] memW_ex;
reg [3:0] memR_ex;   
reg [2:0] fn3_ex;
reg [6:0] opcode_ex;
reg signed [31:0] ALU_out_ex;
reg signed [31:0] adder_out_ex;
reg [31:0] pc_ex_out;



//Memory stage
reg [2:0] Bsel_mem;
reg [31:0] pc_mem, rs2_data_mem;
reg [1:0] WBSel_mem;
reg [4:0] rd_mem;
reg [3:0] memW_mem;
reg [3:0] memR_mem;   
reg [2:0] fn3_mem;
reg [6:0] opcode_mem;

//Writeback stage
reg signed[31:0] ALU_out_mem;
reg WEn_wb;
reg [4:0] rd_wb;


// Registers from imm_gen
reg signed [31:0] s_type;
reg signed [31:0] i_type;
reg signed [31:0] u_type;
reg signed [31:0] uj_type;
reg signed [31:0] b_type;

reg [4:0] rs1;
reg [4:0] rs2;
reg signed [31:0] rs1_data_id;
reg signed [31:0] rs2_data_id;
reg signed [31:0] rs1_data_mem;
reg signed [31:0] ALU_out_wb;

// Registers for forward_logic from_ex
reg signed [31:0] s_type_ex;
reg signed [31:0] i_type_ex;
reg signed [31:0] s_type_mem;
reg signed [31:0] i_type_mem;
reg signed [31:0] data_in_ex;
reg use_forward_on_mux1_ex;
reg use_forward_on_mux2_ex;
reg signed [31:0] forward_data_ex;

// Registers for forward_logic from_mem
reg signed [31:0] data_in_mem;
reg use_forward_on_mux1_mem;
reg use_forward_on_mux2_mem;
reg signed [31:0] forward_data_mem;
reg signed [31:0] ALU_forward_mem; // used for forwarding and writing to ALU_wb_mem

// Registers for forward_logic from_wb
reg [4:0] rs1_decode_wb;
reg [4:0] rs2_decode_wb;
reg use_forward_on_mux1_wb;
reg use_forward_on_mux2_wb;
reg signed [31:0] forward_data_wb;
reg signed [31:0] csr_res_wb;
reg [2:0] funct3_ex; 
reg [2:0] funct3_id; 
reg [31:0] data1_ex; 
reg [31:0] data2_ex; 
reg branch_ex; 
reg wait_branch;
reg inter_stall;

reg [31:0] inst_id_tmp;
reg initialize; // to initialize the pipeline with nop instruction
reg just_stalled;
reg read_stall;

// IF Stage---------------------------------------------------------------------
always @(posedge clk) begin
  just_stalled <= stall;
  if (!just_stalled && stall) queried_pc <= queried_pc - 4;
end

always @(posedge clk) begin
  if (!stall) begin
    if (!inter_stall && PCsel_tmp == 0 && PCSel_id == 0) begin  //No stall and no branch 
      // Reset the pipeline
      read_stall <= 0;
      if (reset) begin 
        queried_pc <= `PC_RESET;
        inst_id <=  `INSTR_NOP;
        initialize <= 0;
        wait_branch <= 0;
        inst_id_tmp <= 0;
        // just_stalled <= 0;
      end else if (|inst_id_tmp) begin
        inst_id <= inst_id_tmp;
        inst_id_tmp <= 0;
        queried_pc <= queried_pc;
      end else if (wait_branch == 1) begin
        pc_id <= queried_pc;
        queried_pc <= queried_pc + 4;
        inst_id <= `INSTR_NOP;
        wait_branch <= 0;
      end else if ( opcode_id == `OPC_LOAD || opcode_id == `OPC_STORE) begin
        queried_pc <= queried_pc;
        inst_id <= `INSTR_NOP;
        read_stall <= 1;
      end else begin
        if (!initialize) begin
          inst_id <=  `INSTR_NOP;
          initialize <= 1;
        end else if (just_stalled) begin
          // queried_pc = queried_pc;
          inst_id <=  `INSTR_NOP;
        end else begin
        // Logic for muxing the next PC value
        //  jump and link register
        pc_id <= queried_pc;
        // branch instruction
        queried_pc <= queried_pc + 4;
        // set the instruction to the output of the instruction cache
        inst_id <= icache_dout;
        // just_stalled <= 0;
        end 
      end

      
    end else if (inter_stall) begin // if the memory stage is stalled, then the pipeline should be stalled
      // if (!just_stalled) begin
      //   just_stalled <= 1;
      //   queried_pc <= queried_pc - 4;
      // end else begin
        queried_pc <= queried_pc;
        // pc_id <= queried_pc;
      // end
    end else begin
      // branch instruction
      if (PCSel_id == 0 )  begin
        // queried_pc <= queried_pc + 4;
        inst_id <= `INSTR_NOP;

        // if the instruction is a jump instruction, set the queried pc to the output of the ALU
        if (opcode_ex == `OPC_JALR) begin
          queried_pc <= adder_out_ex;
          wait_branch <= 1;
        end
        else if (PCSel_ex == `JUMP_INST && PCsel_tmp == `JUMP_INST) begin
          queried_pc <= pc_ex_out;
          wait_branch <= 1;
        end
        else if (PCSel_ex == `JUMP_INST && PCsel_tmp == `COND_BRANCH) begin 
          queried_pc <= ALU_out_ex;
          wait_branch <= 1;
        end
        else queried_pc <= queried_pc + 4;
      
      end else begin  
        inst_id <= `INSTR_NOP;
        pc_id <= queried_pc - 4;
        queried_pc <= queried_pc - 4;
      end

    end
  end
end

// ICache 
always @(*) begin
  icache_addr = queried_pc;
end
// DCache
assign icache_re = !(stall || reset || inter_stall || (opcode_id == `OPC_LOAD || opcode_id == `OPC_STORE));

// ID (Decode) Stage -----------------------------------------------------------
// Decode logic
decoder_logic decoder(
    .inst(inst_id),
    .reset(reset),
    .ASel(Asel_id),
    .Bsel(Bsel_id),
    .PCSel(PCSel_id),
    .WBSel(WBSel_id),
    .rd(rd_id),
    .rs1(rs1),
    .rs2(rs2),
    .fn3(fn3_id),
    .memW(memW_id),
    .memR(memR_id),
    .opcode(opcode_id)
);


// Imm gen
imm_gen generator (
    .inst(inst_id),
    .s_type(s_type),
    .i_type(i_type),
    .u_type(u_type),
    .uj_type(uj_type),
    .b_type(b_type)
);

wire reg_clk = (clk && !inter_stall && !stall); // clock for the register file
reg_file #(
    .NUM_REGS(32),
    .ADDR_WIDTH(5)
) registers (
    .WEn(WEn_wb),     
    .clk(reg_clk),         
    .reset(reset),  
    .addr1(rs1),     
    .addr2(rs2),     
    .write_addr(rd_wb), 
    .d_in(ALU_out_wb),
    .rs1(rs1_data_id),  
    .rs2(rs2_data_id)  
);


// Multiplexing input
forward_logic from_ex (
    .data_in(ALU_out_ex),
    .rs1_decode(rs1),
    .rs2_decode(rs2),
    .rd(rd_ex), 
    .use_forward_on_mux1(use_forward_on_mux1_ex),
    .use_forward_on_mux2(use_forward_on_mux2_ex),
    .data_out(forward_data_ex)
);

forward_logic from_mem (
    .data_in(ALU_forward_mem),
    .rs1_decode(rs1),
    .rs2_decode(rs2),
    .rd(rd_mem),
    .use_forward_on_mux1(use_forward_on_mux1_mem),
    .use_forward_on_mux2(use_forward_on_mux2_mem),
    .data_out(forward_data_mem)
);

forward_logic from_wb (
    .data_in(ALU_out_wb),
    .rs1_decode(rs1),
    .rs2_decode(rs2),
    .rd(rd_wb),
    .use_forward_on_mux1(use_forward_on_mux1_wb),
    .use_forward_on_mux2(use_forward_on_mux2_wb),
    .data_out(forward_data_wb)
);


//Imm gen logic
always @(*) begin
  case (Bsel_id) 
    `REG_BSEL: non_forward_data_id = rs2_data_id;
    `IMM_I_BSEL: non_forward_data_id = i_type;
    `IMM_S_BSEL: non_forward_data_id = s_type;
    `IMM_B_BSEL: non_forward_data_id = b_type;
    `IMM_U_BSEL: non_forward_data_id = u_type;
    `IMM_UJ_BSEL: non_forward_data_id = uj_type;
  endcase
end

wire probe_clk;
assign probe_clk = clk && !inter_stall && !stall; 

//Mux for ALU
always @(posedge clk or posedge reset) begin
  // Reset the pipeline
  if (reset) begin
    op_1 <= 32'b0;
    op_2 <= 32'b0;
    pc_ex <= 32'b0;
    PCsel_tmp <= 2'b0;
    WBSel_ex <= 2'b0;
    rd_ex <= 5'b0;
    memW_ex <= 4'b0;
    memR_ex <= 4'b0;
    opcode_ex <= 0;
    rs2_data_ex <= 0;
    s_type_ex <= 0;
    i_type_ex <= 0;
  end else if (!inter_stall && !stall) begin

  // ALU_A Mux
    if (use_forward_on_mux1_ex) rs1_data_ex <= forward_data_ex;
    else if (!use_forward_on_mux1_ex && use_forward_on_mux1_mem) rs1_data_ex <= forward_data_mem;
    else if (!use_forward_on_mux1_ex && !use_forward_on_mux1_mem && use_forward_on_mux1_wb) rs1_data_ex <= forward_data_wb;
    else rs1_data_ex <= rs1_data_id;

    if (Asel_id == `PC_ASEL) op_1 <= pc_id - 4;
    else if (use_forward_on_mux1_ex) op_1 <= forward_data_ex;
    else if (!use_forward_on_mux1_ex && use_forward_on_mux1_mem) op_1 <= forward_data_mem;
    else if (!use_forward_on_mux1_ex && !use_forward_on_mux1_mem && use_forward_on_mux1_wb) op_1 <= forward_data_wb;
    else op_1 <= rs1_data_id;

  // ALU_B Mux
    if (Bsel_id != `REG_BSEL) op_2 <= non_forward_data_id;
    else if (use_forward_on_mux2_ex) op_2 <= forward_data_ex;
    else if (!use_forward_on_mux2_ex && use_forward_on_mux2_mem) op_2 <= forward_data_mem;
    else if (!use_forward_on_mux2_ex && !use_forward_on_mux2_mem && use_forward_on_mux2_wb) op_2 <= forward_data_wb;
    else op_2 <= rs2_data_id;


  // rs2 Mux
    if (use_forward_on_mux2_ex) rs2_data_ex <= forward_data_ex;
    else if (!use_forward_on_mux2_ex && use_forward_on_mux2_mem) rs2_data_ex <= forward_data_mem;
    else if (!use_forward_on_mux2_ex && !use_forward_on_mux2_mem && use_forward_on_mux2_wb) rs2_data_ex <= forward_data_wb;
    else rs2_data_ex <= rs2_data_id;

  // rs2
    s_type_ex <= s_type;
    i_type_ex <= i_type;
    inst_ex <= inst_id;
    opcode_ex <= opcode_id;
    pc_ex <= pc_id;
    PCsel_tmp <= PCSel_id;
    WBSel_ex <= WBSel_id;
    rd_ex <= rd_id;
    memW_ex <= memW_id;
    memR_ex <= memR_id;
    fn3_ex <= fn3_id;
  end
end

//add_rshift_type logic 
always @(*) begin
    add_rshift_type = ((inst_ex[30] && (fn3_ex == `FNC_ADD_SUB))|| (inst_ex[30] && (fn3_ex == `FNC_SRL_SRA))) ? 1: 0;
    data1_ex = rs1_data_ex;
    data2_ex = rs2_data_ex;
end

// Exexcute Stage ---------------------------------------------------------------------
branch_comp branch_comparator (
   .funct3(fn3_ex), 
   .data1(data1_ex), 
   .data2(data2_ex), 
   .branch(branch_ex)
);

// ALUop decoder
reg [3:0] ALUop;
ALUdec ALUdec1(
  .opcode(opcode_ex),
  .funct(fn3_ex),
  .add_rshift_type(add_rshift_type),
  .ALUop(ALUop)
);

// ALU 
custom_ALU custom_ALU1(
  .A(op_1),
  .B(op_2),
  .ALUop(ALUop),
  .Out(ALU_out_ex),
  .adder(adder_out_ex)
);

//calculating pc related stuff
always @(*) begin
  pc_ex_out = ALU_out_ex;
  if ((PCsel_tmp == 1 && branch_ex) || PCsel_tmp == 2) PCSel_ex = 2;
  else PCSel_ex = 0;
end


// move to the next register
reg [1:0] stall_counter;
reg [31:0] addr;
reg [5:0] shifted_by_read;
reg [5:0] shifted_by_write;
reg [2:0] funct3_mem;
reg [2:0] shift_mask_read;


always @(posedge clk) begin
 if (reset) begin
  ALU_out_mem <= 0;
  rs2_data_mem <= 32'b0;
  pc_mem <= 32'b0;
  WBSel_mem <= 2'b0;
  rd_mem <= 5'b0;
  memW_mem <= 4'b0;
  memR_mem <= 4'b0;
  fn3_mem <= 0;
  opcode_mem <= 0;
  s_type_ex <= 0;
  i_type_ex <= 0;
  shifted_by_read <= 5'b0; 
  shifted_by_write <= 5'b0;
  shift_mask_read <= 0;
  // inter_stall <= 0;
  addr <= 0;
  funct3_mem <= 0;

 end else if (clk && !inter_stall && !stall) begin
  ALU_out_mem <= (WBSel_ex == `PC_WB)? (pc_ex) : ALU_out_ex;
  rs2_data_mem <= rs2_data_ex;
  rs1_data_mem <= op_1;
  s_type_ex <= s_type_mem;
  i_type_ex <= i_type_mem;
  funct3_mem <= funct3_ex;

  if (memR_ex == 15) begin // Read: lw (Load Word)
      shifted_by_read <= 5'b00000;   // Word starts at bit 0
      memR_mem <= memR_ex << 0;
  end else if (memR_ex == 3) begin // Read: lh (Load Halfword)
      shifted_by_read <= {ALU_out_ex[1], 4'b0000}; // Start bit based on bit 1
      memR_mem <= memR_ex << {ALU_out_ex[1], 1'b0};
  end else if (memR_ex == 1) begin // Read: lb (Load Byte)
      shifted_by_read <= {ALU_out_ex[1:0], 3'b000}; // Start bit based on bits [1:0]
      memR_mem <= memR_ex << ALU_out_ex[1:0];
  end else begin
      shifted_by_read <= 5'b0; 
      memR_mem <= 0;
  end

  if (memW_ex == 15) begin // Write: sw (Store Word)
      shifted_by_write <= 5'b00000;  // Word starts at bit 0
      memW_mem <= memW_ex << 0;
  end else if (memW_ex == 3) begin // Write: sh (Store Halfword)
      shifted_by_write <= {ALU_out_ex[1], 4'b0000}; // Start bit based on bit 1
      memW_mem <= memW_ex << {ALU_out_ex[1], 1'b0};
  end else if (memW_ex == 1) begin // Write: sb (Store Byte)
      shifted_by_write <= {ALU_out_ex[1:0], 3'b000}; // Start bit based on bits [1:0]
      memW_mem <= memW_ex << ALU_out_ex[1:0];
  end else begin
      shifted_by_write <= 5'b0;
      memW_mem <= 0;
  end

  pc_mem <= pc_ex;
  WBSel_mem <= WBSel_ex;
  rd_mem <= rd_ex;
  // memW_mem <= memW_ex;
  fn3_mem <= fn3_ex;
  opcode_mem <= opcode_ex;
 end 
end


// Mem Stage ---------------------------------------------------------------------
reg [31:0] load_map, store_map;
reg signed [31:0] tmp_dout1;
reg signed [31:0] tmp_dout2;
reg signed [31:0] tmp_dout3;
reg [31:0] tmp_din1;





assign dcache_re = |memR_mem; // if  any of the memory mask bits are 1, it means that we should be reading
always @(*) begin
  dcache_addr = ALU_out_mem; // address to read from
  dcache_we = memW_mem; // Write enable 

  load_map = {{8{memR_mem[3]}}, {8{memR_mem[2]}}, {8{memR_mem[1]}}, {8{memR_mem[0]}}}; // mask for reading
  store_map = {{8{memW_mem[3]}}, {8{memW_mem[2]}}, {8{memW_mem[1]}}, {8{memW_mem[0]}}}; // mask for reading

  tmp_dout1 = dcache_dout & load_map; // load the data and mask
  tmp_dout2 = tmp_dout1 >> shifted_by_read; // load unsigned

  if (!fn3_mem[2]) begin
    case (memR_mem[3] + memR_mem[2] + memR_mem[1] + memR_mem[0])  // Load unsigned
      4: tmp_dout3 = tmp_dout2; // Load word
      2: tmp_dout3 = $signed({{16{tmp_dout2[15]}} , tmp_dout2[15:0]}); // Load halfword
      1: tmp_dout3 = $signed({{24{tmp_dout2[7]}} , tmp_dout2[7:0]}); // Load byte
    endcase
  end else begin
    tmp_dout3 = tmp_dout2;
  end
  
  if (|memW_mem) begin
    tmp_din1 = (rs2_data_mem << shifted_by_write) & store_map; 
    dcache_din = tmp_din1; // data to write
  end

  ALU_forward_mem = (dcache_re)? tmp_dout3 : ALU_out_mem;
end

always @(negedge clk) begin
  if (!reset)
    inter_stall <= (|memR_mem || |memW_mem) && !inter_stall; // if any of the memory mask bits are 1 and we are not already stalled, 
     //then we should stall
  else inter_stall <= 0;
end

// // if the instruction is a load instruction and the destination register is the same as
// assign load_hazard = (|memR_mem) && ((rd_mem == rs1 && rs1 != 0)|| (rd_mem == rs2 && rs2 != 0));


// WB Stage ---------------------------------------------------------------------
always @(posedge clk) begin
  if (!stall && !inter_stall && !reset) begin
    ALU_out_wb <= ALU_forward_mem;
    csr_res_wb <= (WBSel_mem == `CSR_WB)? rs1_data_mem: 0;
    rd_wb  <= rd_mem;
    WEn_wb <= rd_mem != 0;
  end 
  if (reset) csr_res_wb <= 0;
end

// Set csr
always @(*) begin
  csr = csr_res_wb;
end


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


