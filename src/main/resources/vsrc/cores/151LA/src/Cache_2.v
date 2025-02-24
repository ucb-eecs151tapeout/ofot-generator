`include "util.vh"
`include "const.vh"

module cache #
(
  parameter LINES = 64,
  parameter CPU_WIDTH = `CPU_INST_BITS,
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8) // 32 - log2(32/8) = 30
)
(
  input clk,
  input reset,

  input                       cpu_req_valid, // CPU requesting memory transaction
  output reg                      cpu_req_ready, // cache is ready for CPU memory transaction, added as reg
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr, // address incoming from CPU
  input [CPU_WIDTH-1:0]       cpu_req_data, // write data if writing
  input [3:0]                 cpu_req_write, // if this is not 0, cpu wants to write, 4 bit write mask 

  output reg                      cpu_resp_valid, // added as reg, cache has valid data for CPU after read
  output reg [CPU_WIDTH-1:0]      cpu_resp_data, // giving data to CPU, added as reg

  output reg                      mem_req_valid, // fetching data from the memory, address is valid
  input                       mem_req_ready, // fetching data from the memory, main memory is ready for cache to provide address
  output reg [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr, // fetching data from the memory, log2(128/32), changed to reg, address for memory
  output reg                           mem_req_rw, // saving data to the memory, 1 for write and 0 for read, added reg
  output reg                           mem_req_data_valid, // saving data to the memory, added as reg, cache providing valid data to write memory
  input                            mem_req_data_ready, // saving data to the memory, memory ready for writing
  output reg [`MEM_DATA_BITS-1:0]      mem_req_data_bits, // saving data to the memory, data to write to memory from cache, 128 bits, added as reg
  // byte level masking
  output reg [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask, // saving data to the memory, 16'hFFFF for full write, added reg

  input                       mem_resp_valid, // fetching data from the memory, indicates whether mem_resp_data is valid 
  input [`MEM_DATA_BITS-1:0]  mem_resp_data // fetching data from the memory, contains the data
);

  // Implement your cache here, then delete this comment

  reg [63:0] valids; // array of regs that stores which indices have valid values
  reg [2:0] current_state;
  reg [2:0] next_state;

  reg meta_we;
  reg [3:0] meta_wmask;
  reg [5:0] meta_addr;
  reg [31:0] meta_din;
  wire [31:0] meta_dout;
  reg [31:0] meta_dout_reg;

  reg [3:0] data_we;
  reg [7:0] data_addr;
  reg [3:0][31:0] data_din;
  wire [3:0][31:0] data_dout;
  reg [3:0] data_wmask;

  wire [3:0] offset;
  wire [5:0] index;
  wire [21:0] tag;

  assign offset = cpu_req_addr[3:0];
  assign index = cpu_req_addr[9:4];
  assign tag = cpu_req_addr[WORD_ADDR_BITS-1:10];

  // localparam IDLING = 3'd0; // not doing anything
  localparam META_READ = 3'd1; // idle but also when activated checking for cache hit/miss for tag while also grabbing data from data SRAM (to save a cycle)
  localparam WRITING_FROM_CPU = 3'd3; // writing single word to the sram // update: incorporated into META READ
  localparam WRITING_TO_CACHE = 3'd4; // writing 16 words (entire cache line) to the srams, 4 words at a time
  localparam FETCHING_FROM_MEM = 3'd5; // fetching 16 words (entire cache line) from the mem, 4 words at a time
  localparam WRITING_TO_MEM = 3'd6; //writing through single word to the mem from the CPU

  reg[2:0] cycle_count;

  wire cpu_fire;
  assign cpu_fire = cpu_req_valid && cpu_req_ready; //cpu ready to do stuff
  // assign cpu_resp_data = data_dout[offset];

  wire mem_fetch_fire;
  assign mem_fetch_fire = mem_req_valid && mem_req_ready;
  // assign mem_req_addr = cpu_req_addr[29:2] //only selecting the necessary bits from the address for 

  wire mem_write_fire;
  assign mem_write_fire = mem_req_data_valid && mem_req_data_ready;

// always @(posedge clk or posedge reset) begin
//     if (reset) meta_dout_reg <= 0;
//     else meta_dout_reg <= meta_dout;
// end

//FSM
  always @(posedge clk or posedge reset) begin
    if (reset) current_state <= META_READ;
    else current_state <= next_state;
  end

  always @* begin
    case (current_state)
    META_READ: begin
        if (cpu_fire) begin
            if (cpu_fire && cpu_req_write) next_state = WRITING_FROM_CPU; // if cpu_req_write is not 0 this means the CPU wants us to write something instead of read
            else next_state = FETCHING_FROM_MEM; // if cache miss    
        end
        // if (cpu_fire && cpu_req_write) next_state = WRITING_FROM_CPU; // if cpu_req_write is not 0 this means the CPU wants us to write something instead of read
        // else next_state = FETCHING_FROM_MEM; // if cache miss
    end
    WRITING_FROM_CPU: begin
        next_state = WRITING_TO_MEM;  //time to write through
    end
    WRITING_TO_CACHE: begin
        if (cycle_count == 4) next_state = META_READ;
        else next_state = WRITING_TO_CACHE;
    end
    FETCHING_FROM_MEM: begin
        next_state = WRITING_TO_CACHE;
    end
    WRITING_TO_MEM: next_state = META_READ;
    default: next_state = META_READ;
    endcase
  end

  always @(posedge clk or posedge reset) begin
    if (reset) begin // at reset, set valids reg bits to 0 to invalidate data SRAM values
      valids <= 64'h0000_0000_0000_0000;
      cycle_count <= 1;
      cpu_resp_valid <= 0;
      cpu_resp_data <= 0;
    end
    else begin
      if (current_state == META_READ) begin 
        if (cpu_fire && (meta_dout == tag) && (valids[index] == 1)) begin // if cache hit
          cpu_resp_valid <= 1;
          cpu_resp_data <= data_dout[offset[1:0]];
        end
      end else if (current_state == WRITING_TO_CACHE) begin // apparently the memory automatically increments?
        if (cycle_count == 4) begin
          cycle_count <= 1;
        end else begin
          cycle_count <= cycle_count + 1; // this is probably unsafe but whatever
        end
      end
    end
  end

  always @(*) begin

    cpu_req_ready = 0;
    mem_req_valid = 0;
    mem_req_data_valid = 0;

    meta_we = 0;
    meta_wmask = 4'd0;
    meta_addr = 6'd0;
    meta_din = 32'd0;
    data_we = 4'd0;
    data_wmask = 4'd0;
    data_addr = 8'd0;
    data_din = 128'd0;

    case (current_state)
      META_READ: begin 
        cpu_req_ready = 1; // we're ready to accept input from the cpu
        if (cpu_fire) begin // we read from the SRAM and check to see if its good or not
          meta_we = 0;
          meta_addr = index; // for 64 SRAM lines, need 6 address bits which perfectly fits index for 4 SRAM line groups in the data SRAMS
          data_we = 2'd0;
          data_addr = (index * 4) + offset[3:2]; // offset's most significant 2 bits determine which sram line
        end
      end

      WRITING_FROM_CPU: begin
        meta_we = 1;
        meta_wmask = 4'b1111; // we wanna override everything
        meta_addr = index; // index is 6 bit so fits for 64 lines
        meta_din = {12'b0000_0000_0000, tag}; // zero pad tag to actually fit in the metadata sram

        valids[index] = 1'b1;
        
        data_we = 4'b0001 << offset[1:0]; // the offset is 4 bits, where use the least significant 2 bits to determine which sram block
        // data_we = offset[1:0]; 
        data_wmask = cpu_req_write; //idk what cpu_req_write means
        data_addr = (index * 4) + offset[3:2]; // offset's most significant 2 bits determine which sram line
        data_din[offset[1:0]] = cpu_req_data; // the targeted sram's data_din is changed to be the requested written data
      end

      WRITING_TO_MEM: begin
        mem_req_data_valid = 1; // this will tell the memory to accept our write data
        if (mem_write_fire) begin
          mem_req_data_bits = cpu_req_data << offset[1:0]; // shift over data to fit the mask
          mem_req_addr = cpu_req_addr[29:2]; // i still dont get how this works but itll select 128 bits and if it doesnt ill check this later
          mem_req_rw = 1; // 0 for read
          mem_req_data_mask = 16'h000F << offset[1:0]; // shift over full word's mask
          
        end
      end

      FETCHING_FROM_MEM: begin
        mem_req_valid = 1; // this will tell the memory to accept our request address
        if (mem_fetch_fire) begin
          valids[index] = 1'b1;
          mem_req_addr = cpu_req_addr[29:2]; // i still dont get how this works but itll select 128 bits and if it doesnt ill check this later
          mem_req_rw = 0; // 0 for read
          
        end
      end

      WRITING_TO_CACHE: begin
        if (mem_resp_valid) begin
          meta_we = 1;
          meta_wmask = 4'b1111; // we wanna override everything
          meta_addr = index; // index is 6 bit so fits for 64 lines
          meta_din = {12'b0000_0000_0000, tag}; // zero pad tag to actually fit in the metadata sram
          
          data_we = 4'b1111;
          data_addr = (index * 4) + cycle_count; // index is 6 bit but data_addr is 8 bit because there are 256 sram lines, so each index accomodates 4 sram lines
          data_din = mem_resp_data;
        end
      end

    endcase
  end


  sram22_256x32m4w8 sramData0 (
    .clk(clk),
    .we(data_we[0]),
    .wmask(data_wmask), // state == WRITE_QUERY ? prev_req_write : 4'hF
    .addr(data_addr),
    .din(data_din[0]),
    .dout(data_dout[0])
  );    

  sram22_256x32m4w8 sramData1 (
    .clk(clk),
    .we(data_we[1]),
    .wmask(data_wmask), // state == WRITE_QUERY ? prev_req_write : 4'hF
    .addr(data_addr),
    .din(data_din[1]),
    .dout(data_dout[1])
  );    
  sram22_256x32m4w8 sramData2 (
    .clk(clk),
    .we(data_we[2]),
    .wmask(data_wmask), // state == WRITE_QUERY ? prev_req_write : 4'hF
    .addr(data_addr),
    .din(data_din[2]),
    .dout(data_dout[2])
  );    
  sram22_256x32m4w8 sramData3 (
    .clk(clk),
    .we(data_we[3]),
    .wmask(data_wmask), // state == WRITE_QUERY ? prev_req_write : 4'hF
    .addr(data_addr),
    .din(data_din[3]),
    .dout(data_dout[3])
  );        


  // {valid, tag} // update no, i'm using the large valids reg to store valids instead as they are easier to reset
  sram22_64x32m4w8 sramMeta (
    .clk(clk),
    .we(meta_we),
    .wmask(meta_wmask),
    .addr(meta_addr),
    .din(meta_din),
    .dout(meta_dout)
  );

endmodule
