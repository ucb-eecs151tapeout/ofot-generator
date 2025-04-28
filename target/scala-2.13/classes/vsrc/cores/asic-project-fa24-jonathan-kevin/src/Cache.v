`include "util.vh"
`include "const.vh"

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