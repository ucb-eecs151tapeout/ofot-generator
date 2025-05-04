// Module: ALUdecoder
// Desc:   Sets the ALU operation
// Inputs: opcode: the top 6 bits of the instruction
//         funct: the funct, in the case of r-type instructions
//         add_rshift_type: selects whether an ADD vs SUB, or an SRA vs SRL
// Outputs: ALUop: Selects the ALU's operation
//

`include "Opcode.vh"
`include "ALUop.vh"

module ALUdec(
  input [6:0]       opcode,
  input [2:0]       funct,
  input             add_rshift_type,
  output reg [3:0]  ALUop
);

  // Implement your ALU decoder here, then delete this comment
  always @* begin
    case(opcode)
    // No operation (kill) : 7'b0000000
    `OPC_NOOP: ALUop = `ALU_XXX;       

    // Special immediate instructions
    `OPC_LUI: ALUop = `ALU_COPY_B;       //7'b0110111
    `OPC_AUIPC: ALUop = `ALU_ADD;      //7'b0010111

    // // Jump instructions
    `OPC_JAL: ALUop = `ALU_ADD;          //7'b1101111
    `OPC_JALR: ALUop = `ALU_ADD;         //7'b1100111

    // // Branch instructions
    `OPC_BRANCH: ALUop = `ALU_ADD;       //7'b1100011

    // // Load and store instructions
    `OPC_STORE: ALUop = `ALU_ADD;       //7'b0100011
    `OPC_LOAD: ALUop = `ALU_ADD;        //7'b0000011

    // Arithmetic instructions
    `OPC_ARI_RTYPE: begin  //7'b0110011
      if (funct == 3'b000) begin
        if(add_rshift_type) ALUop = `ALU_SUB;
        else ALUop = `ALU_ADD;
      end
      else if (funct == 3'b001) ALUop = `ALU_SLL;
      else if (funct == 3'b010) ALUop = `ALU_SLT;
      else if (funct == 3'b011) ALUop = `ALU_SLTU;
      else if (funct == 3'b100) ALUop = `ALU_XOR;
      else if (funct == 3'b101) begin
        if(add_rshift_type) ALUop = `ALU_SRA;
        else ALUop = `ALU_SRL;
      end
      else if (funct == 3'b110) ALUop = `ALU_OR;
      else if (funct == 3'b111) ALUop = `ALU_AND;
      else ALUop = `ALU_XXX;      
    end
    `OPC_ARI_ITYPE: begin  //7'b0010011
      if (funct == 3'b000) ALUop = `ALU_ADD;
      else if (funct == 3'b001) ALUop = `ALU_SLL;
      else if (funct == 3'b010) ALUop = `ALU_SLT;
      else if (funct == 3'b011) ALUop = `ALU_SLTU;
      else if (funct == 3'b100) ALUop = `ALU_XOR;
      else if (funct == 3'b101) begin
        if(add_rshift_type) ALUop = `ALU_SRA;
        else ALUop = `ALU_SRL;
      end
      else if (funct == 3'b110) ALUop = `ALU_OR;
      else if (funct == 3'b111) ALUop = `ALU_AND;
      else ALUop = `ALU_XXX;
    end
    // Control status register
    `OPC_CSR: ALUop = `ALU_ADD;        //7'b1110011
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
// Outputs:
//    Out: The chosen function mapped to A and B.

`include "Opcode.vh"
`include "ALUop.vh"

module custom_ALU(
    input [31:0] A,B,
    input [3:0] ALUop,
    output reg [31:0] Out
);

    // Implement your ALU here, then delete this comment

always @(*) begin
    case (ALUop)
        `ALU_ADD: Out = A + B;
        `ALU_SUB: Out = A - B;
        `ALU_AND: Out = A & B;
        `ALU_OR: Out = A | B;
        `ALU_XOR: Out = A ^ B;
        `ALU_SLT: Out = ($signed(A) < $signed(B)) ? 32'd1 : 32'd0;
        `ALU_SLTU: Out = (A < B) ? 32'd1: 32'd0;
        `ALU_SLL: Out = A << B[4:0];
        `ALU_SRA: Out = $signed(A) >>> B[4:0];
        `ALU_SRL: Out = A >> B[4:0];
        `ALU_COPY_B: Out = B;
        `ALU_XXX: Out = 32'd0;
    endcase
end

endmodule
module BranchComp #(
    parameter size = 32
) (
    //input
    RS1_data,
    RS2_data,
    BrUn,

    //output
    BrEq,
    BrLT
);

  //I/O ports
  input BrUn;
  input [size-1:0] RS1_data;
  input [size-1:0] RS2_data;
  output BrEq;
  output BrLT;

  //Internal Signals
  wire [size-1:0] RS1_data_m, RS2_data_m;

  //Main function
  assign BrEq = (RS1_data == RS2_data) ? 1 : 0;
  assign RS1_data_m = (BrUn) ? -RS1_data : RS1_data;
  assign RS2_data_m = (BrUn) ? -RS2_data : RS2_data;
  assign BrLT = (~BrUn && ($signed(RS1_data) < $signed(RS2_data))) ? 1 : 
                (BrUn && ((RS1_data) < (RS2_data))) ? 1 : 0;





    
endmodule
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

  reg[1:0] cycle_count;

  wire cpu_fire;
  assign cpu_fire = cpu_req_valid && cpu_req_ready; //cpu ready to do stuff
  // assign cpu_resp_data = data_dout[offset];

  wire mem_fetch_fire;
  assign mem_fetch_fire = mem_req_valid && mem_req_ready;
  // assign mem_req_addr = cpu_req_addr[29:2] //only selecting the necessary bits from the address for 

  wire mem_write_fire;
  assign mem_write_fire = mem_req_data_valid && mem_req_data_ready;

  always @(posedge clk or posedge reset) begin
    if (reset) begin // at reset, set valids reg bits to 0 to invalidate data SRAM values
      valids <= 64'h0000_0000_0000_0000;
      current_state <= META_READ;
      meta_we <= 0;
      data_we <= 0;
    end
    else begin

      if (current_state == META_READ) begin 
        if (cpu_fire) begin
          if (cpu_req_write) begin // if cpu_req_write is not 0 this means the CPU wants us to write something instead of read
            current_state <= WRITING_FROM_CPU; 
          end
        end else if ((meta_dout == tag) && (valids[index] == 1)) begin // if cache hit
          cpu_resp_valid <= 1;
          cpu_resp_data <= data_dout[offset[1:0]];
        end else begin // if cache miss
          current_state <= FETCHING_FROM_MEM;
        end
      end else if (current_state == WRITING_FROM_CPU) begin
        valids[index] <= 1;
        current_state <= WRITING_TO_MEM; //time to write through
      end else if (current_state == FETCHING_FROM_MEM) begin
          current_state <= WRITING_TO_CACHE;
      end else if (current_state == WRITING_TO_CACHE) begin // apparently the memory automatically increments?
        if (cycle_count == 4) begin
          cycle_count <= 0;
          current_state <= META_READ;
        end else begin
          cycle_count <= cycle_count + 1; // this is probably unsafe but whatever
        end
      end else if (current_state == WRITING_TO_MEM) begin
        current_state <= META_READ;
      end else current_state <= META_READ;
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
  sram22_64x32m4w8 sramMeta1 (
    .clk(clk),
    .we(meta_we),
    .wmask(meta_wmask),
    .addr(meta_addr),
    .din(meta_din),
    .dout(meta_dout)
  );

endmodule

module CSdec #(
    parameter size = 32
) (
    //input
    instr_X,
    instr_M,
    BrEq,
    BrLT,
    rst,

    //output
    fw_RS1, 
    fw_RS2,
    BrUn,
    ImmSel,
    A_sel,
    B_sel,
    MemRW,
    RegWEn,
    WBSel,
    PC_sel,
    NOP,
    CSR_RI,
);

  //I/O ports
  input [size-1:0] instr_X;
  input [size-1:0] instr_M;
  input BrEq, BrLT;
  input rst;
  
  output fw_RS1;
  output fw_RS2;
  output BrUn;
  output [2:0] ImmSel;
  output A_sel;
  output B_sel;
  output MemRW;
  output RegWEn;
  output [1:0] WBSel;
  output [1:0] PC_sel;
  output NOP;
  output [1:0] CSR_RI;

  //Internal Signals
  wire [6:0] instr_op_X, instr_op_M;
  wire [2:0] instr_func3_X;
  wire [5:0] instr_rd_M;
  wire [5:0] instr_rs1_X, instr_rs2_X;

  //Main function
  assign instr_op_X = instr_X[6:0];
  assign instr_op_M = instr_M[6:0];
  assign instr_func3_X = instr_X[14:12];
  assign instr_rd_M = instr_M[11:7];
  assign instr_rs1_X = instr_X[19:15];
  assign instr_rs2_X = instr_X[24:20];

  //NOP and PC_sel

  //NOP = 1 at M stage when br taken or jump
  assign NOP = ((instr_op_X == (`OPC_BRANCH)) && ((BrEq && instr_func3_X == `FNC_BEQ) || (~BrEq && instr_func3_X == `FNC_BNE) || (BrLT && (instr_func3_X == `FNC_BLT || instr_func3_X == `FNC_BLTU)) || (~BrLT && (instr_func3_X == `FNC_BGE || instr_func3_X == `FNC_BGEU)))) ? 1 :
               (instr_op_X == (`OPC_JAL) || instr_op_X == (`OPC_JALR) ) ? 1 : 0;

  //PCSel = 1 at M stage when br taken or jump
  assign PC_sel = (rst) ? 2 :
                  (instr_op_X == (`OPC_JAL) || instr_op_X == (`OPC_JALR) ) ? 1 : 
                  ((instr_op_X == (`OPC_BRANCH)) && ((BrEq && instr_func3_X == `FNC_BEQ) || (~BrEq && instr_func3_X == `FNC_BNE) || (BrLT && (instr_func3_X == `FNC_BLT || instr_func3_X == `FNC_BLTU)) || (~BrLT && (instr_func3_X == `FNC_BGE || instr_func3_X == `FNC_BGEU)))) ? 1 : 0 ;

  //RegWEn at M is 1 when R-type, I-type, lw, jalr, jal
  assign RegWEn = ((instr_op_X == `OPC_BRANCH) || (instr_op_X == `OPC_STORE) || (instr_X == `INSTR_NOP)) ? 0 : 1;

  //MemRW at X is 1 when sw
  assign MemRW = (instr_op_X == `OPC_STORE);

  //WBSel at M is 1(ALU) when R-type, I-type, auipc, lui; 2(PC+4) when jalr, jal
  assign WBSel = ((instr_op_X == `OPC_ARI_RTYPE) || (instr_op_X == `OPC_ARI_ITYPE) || (instr_op_X == `OPC_AUIPC) || (instr_op_X == `OPC_LUI)) ? 1 : 
                   ((instr_op_X == `OPC_JAL) || (instr_op_X == `OPC_JALR)) ? 2 : 0;

  //fw_RS1 is 1 when previous WBrd == current rs1 
  assign fw_RS1 = (instr_rd_M == instr_rs1_X) && ((instr_op_M != `OPC_BRANCH) && (instr_op_M != `OPC_STORE)) && (instr_rs1_X != 0);

  //fw_RS2 is 1 when previous WBrd == current rs1 
  assign fw_RS2 = (instr_rd_M == instr_rs2_X) && ((instr_op_M != `OPC_BRANCH) && (instr_op_M != `OPC_STORE)) && (instr_rs2_X != 0);

  //ImmSel at X 
  assign ImmSel = ((instr_op_X == `OPC_JAL)) ? `Imm_J :
                  (instr_op_X == `OPC_JALR) ? `Imm_JR :
                  ((instr_op_X == `OPC_ARI_ITYPE) || (instr_op_X == `OPC_LOAD)) ? `Imm_I :
                  (instr_op_X == `OPC_STORE) ? `Imm_S :
                  (instr_op_X == `OPC_BRANCH) ? `Imm_B :
                  ((instr_op_X == `OPC_AUIPC) || (instr_op_X == `OPC_LUI)) ? `Imm_U : 
                  (instr_op_X == `OPC_CSR) ? `Imm_csr : 0;

  //BrUn at X is 1 when bltu
  assign BrUn = ((instr_op_X == `OPC_BRANCH) && ((instr_func3_X == `FNC_BGEU) || (instr_func3_X == `FNC_BLTU)));

  //A_sel at X
  assign A_sel = (instr_op_X == `OPC_BRANCH) || (instr_op_X == `OPC_JAL) || (instr_op_X == `OPC_AUIPC);
  
  //B_sel at X is 0 iff r-type
  assign B_sel = (instr_op_X == `OPC_ARI_RTYPE) ? 0: 1;

  //CSR st X
  assign CSR_RI = ((instr_func3_X == `FNC_RWI) && (instr_op_X == `OPC_CSR)) ? 2 : //csrwi
                  ((instr_func3_X == `FNC_RW) && (instr_op_X == `OPC_CSR)) ? 1 : 0; //csrw


endmodule

module CSR #(
    parameter size = 32
) (
    clk,
    rst,
    CSR_RI,
    RS1_data,
    Zimm,
    csr_o,
    stall
);

    //I/O

    input clk;
    input rst;
    input [1:0] CSR_RI;
    input [size-1:0] RS1_data;
    input [size-1:0] Zimm;
    output reg [size-1:0] csr_o;
    input stall;

    //Main Function
    always @(posedge clk or posedge rst) begin
        if (rst)
            csr_o <= 0;
        else if (!stall) begin
            case (CSR_RI)
                2'd2: csr_o <= Zimm;
                2'd1: csr_o <= RS1_data;
                default: csr_o <= 0;
            endcase
        end
    end

    // always @(posedge clk or posedge rst) begin
    // $display("At time %0d: clk=%b, rst=%b, CSR_RI=%b, RS1_data=%h, Zimm=%h, csr_o=%h", 
    //          $time, clk, rst, CSR_RI, RS1_data, Zimm, csr_o);
    // end
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

// Register of D-Type Flip-flops
module REGISTER(q, d, clk);
   parameter N = 1;
   output reg [N-1:0] q;
   input [N-1:0]      d;
   input 	     clk;
   always @(posedge clk)
    q <= d;
endmodule // REGISTER

// Register with clock enable
module REGISTER_CE(q, d, ce, clk);
   parameter N = 1;
   output reg [N-1:0] q;
   input [N-1:0]      d;
   input 	      ce, clk;
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
   always @(posedge clk)
     if (rst) q <= INIT;
     else if (ce) q <= d;
     else q <= q;
endmodule // REGISTER_R_CE


/* 
 Memory Blocks.  These will simulate correctly and synthesize
 correctly to memory resources in the FPGA flow.  Eventually, will
 need to make an ASIC version.
*/

// Single-ported RAM with asynchronous read
module RAM(q, d, addr, we, clk);
   parameter DWIDTH = 8;               // Data width
   parameter AWIDTH = 8;               // Address width
   parameter DEPTH = 256;              // Memory depth
   input [DWIDTH-1:0] d;               // Data input
   input [AWIDTH-1:0] addr;            // Address input
   input 	      we, clk;
   reg [DWIDTH-1:0]   mem [DEPTH-1:0];
   output [DWIDTH-1:0] q;
   always @(posedge clk)
      if (we) mem[addr] <= d;
   assign q = mem[addr];
endmodule // RAM

/*
 To add: multiple ports, synchronous read, ASIC synthesis support.
 */
`include "util.vh"
`include "const.vh"

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

module ImmGen #(
    parameter size = 32
) (
    ImmSel,
    instr_i,
    instr_o
);

    input [2:0] ImmSel;
    input [31:7] instr_i;
    output [size-1:0] instr_o;

    assign instr_o = (ImmSel == `Imm_J) ? {{11{instr_i[31]}}, instr_i[31], instr_i[19:12], instr_i[20], instr_i[30:21], 1'b0} :
                     (ImmSel == `Imm_JR) ? {{20{instr_i[31]}}, instr_i[31:20]} :
                     (ImmSel == `Imm_I) ? {{20{instr_i[31]}}, instr_i[31:20]} :
                     (ImmSel == `Imm_U) ? {instr_i[31:12], {12'd0}} :
                     (ImmSel == `Imm_B) ? {{19{instr_i[31]}}, instr_i[31], instr_i[7], instr_i[30:25], instr_i[11:8], 1'b0} :
                     (ImmSel == `Imm_S) ? {{20{instr_i[31]}}, instr_i[31:25], instr_i[11:7]} : 
                     (ImmSel == `Imm_csr) ? {27'd0, instr_i[19:15]} : 0;  

    
endmodule
module LoadMask #(
    parameter size = 32
) (
    instr_func3_M,
    instr_op_M,
    DMEM_out,
    LoadMask_out,
    imm,
    addr
);
    //I/O
    input [2:0] instr_func3_M;
    input [6:0] instr_op_M;
    input [size-1:0] DMEM_out;
    input [size-1:0] imm;
    input [1:0] addr;
    output [size-1:0] LoadMask_out;


    //Main Function
    assign LoadMask_out = ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 0) ? {{24{DMEM_out[7]}},DMEM_out[7:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 1) ? {{24{DMEM_out[15]}},DMEM_out[15:8]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 2) ? {{24{DMEM_out[23]}},DMEM_out[23:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LB) && addr == 3) ? {{24{DMEM_out[31]}},DMEM_out[31:24]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 0) ? {24'd0, DMEM_out[7:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 1) ? {24'd0, DMEM_out[15:8]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 2) ? {24'd0, DMEM_out[23:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LBU) && addr == 3) ? {24'd0, DMEM_out[31:24]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 0) ? {{16{DMEM_out[15]}},DMEM_out[15:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 1) ? {{16{DMEM_out[15]}},DMEM_out[15:0]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 2) ? {{16{DMEM_out[31]}},DMEM_out[31:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LH) && addr == 3) ? {{16{DMEM_out[31]}},DMEM_out[31:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 0) ? {16'd0, DMEM_out[15:0]} : 
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 1) ? {16'd0, DMEM_out[15:0]} : 
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 2) ? {16'd0, DMEM_out[31:16]} :
                          ((instr_op_M == `OPC_LOAD) && (instr_func3_M == `FNC_LHU) && addr == 3) ? {16'd0, DMEM_out[31:16]} : DMEM_out;

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

module Mux2to1 #(parameter size = 32) (
    data0_i,
    data1_i,
    select_i,
    data_o
);

  //I/O ports
  input [size-1:0] data0_i;
  input [size-1:0] data1_i;
  input select_i;

  output [size-1:0] data_o;

  //Internal Signals
  wire [size-1:0] data_o;

  //Main function
  assign data_o = select_i ? data1_i : data0_i;

endmodule

module Mux3to1 #(parameter size = 32) (
    data0_i,
    data1_i,
    data2_i,
    select_i,
    data_o
);

  //I/O ports
  input [size-1:0] data0_i;
  input [size-1:0] data1_i;
  input [size-1:0] data2_i;
  input [2-1:0] select_i;

  output [size-1:0] data_o;

  //Internal Signals
  wire [size-1:0] data_o;

  //Main function
  /*your code here*/
  assign data_o = (select_i == 2'b00) ? data0_i :
    (select_i == 2'b01) ? data1_i :
    (select_i == 2'b10) ? data2_i : data0_i;

endmodule

`include "util.vh"
`include "const.vh"

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


module Pipe_Reg #(parameter size = 32)(
  clk,
  rst,
  data_i,
  data_o,
  stall
);

  //I/O ports
  input clk;
  input rst;
  input [size-1:0] data_i;

  output [size-1:0] data_o;
  input stall;

  //Internal Signals
  reg [size-1:0] data_o;

  //Main function
  /*your code here*/
  always @(posedge clk ) begin
    if (rst) data_o <= 0;
    else if (!stall) data_o <= data_i;
  end
  
  

endmodule


module Program_Counter #(parameter size = 32)(
    clk,
    rst,
    PC_in,
    PC_out,
    stall
);

  //I/O ports
  input clk;
  input rst;
  input [size-1:0] PC_in;
  output reg [size-1:0] PC_out;
  input stall;

  //Internal Signals
  reg [size-1:0] current_PC;
  //assertion:
  // assert property(@(posedge clk) rst && (PC_out ==`PC_RESET)) begin
  // end
  //Main function

  // always @(posedge clk or posedge rst) begin
  //   if (rst) current_PC <= `PC_RESET;
  //   else current_PC <= PC_in;
  // end

  always @(posedge clk or posedge rst) begin
    if (rst) PC_out <= `PC_RESET;
    else if (!stall) PC_out <= PC_in;
  end

endmodule

module RegFile #(parameter size = 32)(
    //input
    clk,
    rst,
    RS1_addr_i,
    RS2_addr_i,
    RDaddr_i,
    RDdata_i,
    RegWrite_en,
    stall,

    //output
    RS1_data_o,
    RS2_data_o
);
  parameter N = 32; // number of reg


  //I/O ports
  input clk;
  input rst;
  input RegWrite_en;
  input [$clog2(N)-1:0] RS1_addr_i;
  input [$clog2(N)-1:0] RS2_addr_i;
  input [$clog2(N)-1:0] RDaddr_i;
  input [size-1:0] RDdata_i;
  input stall;

  output [size-1:0] RS1_data_o;
  output [size-1:0] RS2_data_o;

  //Internal signals/registers
  reg signed [size-1:0] Reg_File [0:N-1];  //32 word registers

  //assertion
  // assert (Reg_File[0] == 0) 
  //   else $error("Error: x0 != 0");

  //Read the data
  assign RS1_data_o = Reg_File[RS1_addr_i];
  assign RS2_data_o = Reg_File[RS2_addr_i];

  //clk are just for writing
  //Writing data when postive edge clk_i and RegWrite_i was set.
  always @(posedge rst or posedge clk) begin
    if (rst) begin
      Reg_File[0]  <= 0;
      Reg_File[1]  <= 0;
      Reg_File[2]  <= 0;
      Reg_File[3]  <= 0;
      Reg_File[4]  <= 0;
      Reg_File[5]  <= 0;
      Reg_File[6]  <= 0;
      Reg_File[7]  <= 0;
      Reg_File[8]  <= 0;
      Reg_File[9]  <= 0;
      Reg_File[10] <= 0;
      Reg_File[11] <= 0;
      Reg_File[12] <= 0;
      Reg_File[13] <= 0;
      Reg_File[14] <= 0;
      Reg_File[15] <= 0;
      Reg_File[16] <= 0;
      Reg_File[17] <= 0;
      Reg_File[18] <= 0;
      Reg_File[19] <= 0;
      Reg_File[20] <= 0;
      Reg_File[21] <= 0;
      Reg_File[22] <= 0;
      Reg_File[23] <= 0;
      Reg_File[24] <= 0;
      Reg_File[25] <= 0;
      Reg_File[26] <= 0;
      Reg_File[27] <= 0;
      Reg_File[28] <= 0;
      Reg_File[29] <= 0;
      Reg_File[30] <= 0;
      Reg_File[31] <= 0;
    end else if (!stall) begin
      if (RegWrite_en && RDaddr_i != 0) Reg_File[RDaddr_i] <= RDdata_i;
      else Reg_File[RDaddr_i] <= Reg_File[RDaddr_i];
    end
  end

    // assign Reg_File[RDaddr_i] = (rst) ? 0 :
    //                             (RegWrite_en && RDaddr_i != 0) ? RDdata_i : 0;
    

endmodule







`include "const.vh"

module Riscv151(
    input clk,
    input reset,

    // Memory system ports
    output [31:0] dcache_addr,
    output [31:0] icache_addr,
    output [3:0] dcache_we, //store
    output dcache_re, // load
    output icache_re, // always be high
    output [31:0] dcache_din,
    input [31:0] dcache_dout,
    input [31:0] icache_dout,
    input stall, // reg remain the value when stall is 1, do nothing if stall is 0
    output [31:0] csr // parallel to dmem

);

  // Implement your core here, then delete this comment


  //internal signals
  wire [`CPU_ADDR_BITS-1:0] PC_in, PC_out, PC_add_4_X, PC_add_4_M, PC_I;
  wire [`CPU_ADDR_BITS-1:0] ALU_out_X, ALU_out_M;
  wire [`CPU_INST_BITS-1:0] instr_X, instr_M;
  wire [`CPU_INST_BITS-1:0] instr_in;
  wire [`CPU_INST_BITS-1:0] instr_Imm, instr_Imm_M;
  wire [`CPU_DATA_BITS-1:0] RegWrite_data;
  wire [`CPU_DATA_BITS-1:0] LoadMask_out;
  wire [`CPU_DATA_BITS-1:0] RS1_data, RS2_data;  
  wire [`CPU_DATA_BITS-1:0] RS1_data_fw, RS2_data_fw;
  wire [3:0] ALUop;
  wire [`CPU_DATA_BITS-1:0] A_out, B_out;
  wire [`CPU_DATA_BITS-1:0] MemWrite_data;
  wire [31:0] tohost;

  //control signal
  wire [1:0] PC_sel_X, PC_sel_M;
  wire [1:0] PC_sel;
  wire RegWEn, RegWEn_X;
  wire [1:0] WBSel, WBSel_X;
  wire [2:0] ImmSel;
  wire fw_RS1, fw_RS2;
  wire BrEq, BrLT, BrUn;
  wire A_sel, B_sel;
  wire MemRW; //MemRW_X;
  wire NOP;
  wire NOP_X;
  wire [1:0] CSR_RI;
  wire [3:0] dcache_mask;

  //assign wires
  assign dcache_re = (instr_X[6:0] == `OPC_LOAD) ? 1 : 0;
  assign dcache_we = (MemRW) ? dcache_mask : 0;
  assign icache_re = 1;
  assign csr = tohost;
  // assign PC_sel = (reset) ? 2 : PC_sel_M;

  //blocks

  //I
  Mux3to1 #(.size(`CPU_ADDR_BITS)) Mux_PC(
    .data0_i(PC_add_4_X),
    .data1_i(ALU_out_M),
    .data2_i(`PC_RESET),
    .select_i(PC_sel),
    .data_o(PC_in)
  );

  //reset PC
  // Rst_Reg #(.size(`CPU_DATA_BITS)) PC_reset(
  //     .clk(clk),
  //     .rst(reset),
  //     .data_i(PC_in),
  //     .data_o(PC_I)
  // );

  //Pipe_reg between I/X
  
  Program_Counter #(.size(`CPU_ADDR_BITS)) PC(
      .clk(clk),
      .rst(reset),
      .PC_in(PC_in),
      // .PC_in(PC_I),
      .PC_out(PC_out),
      .stall(stall)
  );

  assign icache_addr = PC_in; // imem syn read takes 1 cycle
  // assign icache_addr = PC_I; // imem syn read takes 1 cycle

  assign PC_add_4_X = PC_out + 4;

  //X
  CSdec #(.size(`CPU_INST_BITS)) U_CSdec(
    //input
    .instr_X(instr_X),
    .instr_M(instr_M),
    .BrEq(BrEq),
    .BrLT(BrLT),
    .rst(reset),

    //output
    //Xstage
    .fw_RS1(fw_RS1), 
    .fw_RS2(fw_RS2),
    .BrUn(BrUn),
    .ImmSel(ImmSel),
    .A_sel(A_sel),
    .B_sel(B_sel),
    .MemRW(MemRW),
    .CSR_RI(CSR_RI),

    //Mstage
    .RegWEn(RegWEn_X),
    .WBSel(WBSel_X),
    .PC_sel(PC_sel_X),
    .NOP(NOP_X)
  );

  assign instr_in = (PC_sel != 2) ? icache_dout : 0;

  Mux2to1 #(.size(`CPU_INST_BITS)) Mux_NOP(
    .data0_i(instr_in), 
    .data1_i(`INSTR_NOP),
    .select_i(NOP),
    .data_o(instr_X)
  );

  RegFile #(.size(`CPU_INST_BITS)) Reg_File(
      //input
      .clk(clk),
      .rst(reset),
      .RS1_addr_i(instr_X[19:15]),
      .RS2_addr_i(instr_X[24:20]),
      .RDaddr_i(instr_M[11:7]), // from instr of M stage (but need a cycle)
      .RDdata_i(RegWrite_data),
      .RegWrite_en(RegWEn),
      .stall(stall),

      //output
      .RS1_data_o(RS1_data),
      .RS2_data_o(RS2_data)
  );
  
  ImmGen #(.size(`CPU_INST_BITS)) U_ImmGen(
    .ImmSel(ImmSel),
    .instr_i(instr_X[31:7]),
    .instr_o(instr_Imm)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_fw_RS1(
    .data0_i(RS1_data), 
    // .data1_i(ALU_out_M),
    .data1_i(RegWrite_data),
    .select_i(fw_RS1),
    .data_o(RS1_data_fw)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_fw_RS2(
    .data0_i(RS2_data),
    .data1_i(RegWrite_data),
    .select_i(fw_RS2),
    .data_o(RS2_data_fw)
  );

  BranchComp #(.size(`CPU_DATA_BITS)) U_BranchComp (
    //input
    .RS1_data(RS1_data_fw),
    .RS2_data(RS2_data_fw),
    .BrUn(BrUn),

    //output
    .BrEq(BrEq),
    .BrLT(BrLT)
  );

  ALUdec U_ALUdec(
    .opcode(instr_X[6:0]),
    .funct(instr_X[14:12]),
    .add_rshift_type(instr_X[30]),
    .ALUop(ALUop)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_ALU_A(
    .data0_i(RS1_data_fw),
    .data1_i(PC_out),
    .select_i(A_sel),
    .data_o(A_out)
  );

  Mux2to1 #(.size(`CPU_DATA_BITS)) Mux_ALU_B(
    .data0_i(RS2_data_fw),
    .data1_i(instr_Imm),
    .select_i(B_sel),
    .data_o(B_out)
  ); 

  ALU U_custom_ALU(
    .A(A_out),
    .B(B_out),
    .ALUop(ALUop),
    .Out(ALU_out_X)
  );

  StoreMask #(.size(`CPU_DATA_BITS)) U_StoreMask(
    .instr_func3_X(instr_X[14:12]),
    .instr_op_X(instr_X[6:0]),
    .RS2_data_fw(RS2_data_fw),
    .MemWrite_data(MemWrite_data),
    .mask(dcache_mask),
    .addr(ALU_out_X[1:0])
  );


  //Pipe_reg between X/M

  assign dcache_addr = ALU_out_X; // dmem syn read takes 1 cycle
  assign dcache_din = MemWrite_data;

  CSR #(.size(`CPU_DATA_BITS)) X_M_CSR(
      .clk(clk),
      .rst(reset),
      .CSR_RI(CSR_RI),
      .RS1_data(RS1_data_fw),
      .Zimm(instr_Imm),
      .csr_o(tohost),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_DATA_BITS)) X_M_custom_ALU(
      .clk(clk),
      .rst(reset),
      .data_i(ALU_out_X),
      .data_o(ALU_out_M),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_DATA_BITS)) X_M_PCplus4(
      .clk(clk),
      .rst(reset),
      .data_i(PC_add_4_X),
      .data_o(PC_add_4_M),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_INST_BITS)) X_M_instr(
      .clk(clk),
      .rst(reset),
      .data_i(instr_X),
      .data_o(instr_M),
      .stall(stall)
  );

  Pipe_Reg #(.size(6)) X_M_CS(
      .clk(clk),  
      .rst(1'b0),
      .data_i({RegWEn_X, WBSel_X, PC_sel_X, NOP_X}),
      .data_o({RegWEn, WBSel, PC_sel, NOP}),
      .stall(stall)
  );

  Pipe_Reg #(.size(`CPU_INST_BITS)) X_M_imm(
      .clk(clk),
      .rst(reset),
      .data_i(instr_Imm),
      .data_o(instr_Imm_M),
      .stall(stall)
  );
  //M

  LoadMask #(.size(`CPU_DATA_BITS)) U_LoadMask(
    .instr_func3_M(instr_M[14:12]),
    .instr_op_M(instr_M[6:0]),
    .DMEM_out(dcache_dout),
    .LoadMask_out(LoadMask_out),
    .imm(instr_Imm_M),
    .addr(ALU_out_M[1:0])
  );

  Mux3to1 #(.size(`CPU_DATA_BITS)) Mux_WB(
    .data0_i(LoadMask_out),
    .data1_i(ALU_out_M),
    .data2_i(PC_add_4_M),
    .select_i(WBSel),
    .data_o(RegWrite_data)
  );

  // cache Cache(
  //   .clk(clk), 
  //   .reset(rst),

  //   .cpu_req_valid(cpu_req_valid), //input
  //   .cpu_req_ready(cpu_req_ready), //output // connects to stall signal
  //   .cpu_req_addr(cpu_req_addr), //input
  //   .cpu_req_data(cpu_req_data), //input
  //   .cpu_req_write(cpu_req_write), //input

  //   .cpu_resp_valid(cpu_resp_valid), //output
  //   .cpu_resp_data(cpu_resp_data), //output

  //   .mem_req_valid(mem_req_valid), //output
  //   .mem_req_ready(mem_req_ready), //input
  //   .mem_req_addr(mem_req_addr), //output
  //   .mem_req_rw(mem_req_rw), //output
  //   .mem_req_data_valid(mem_req_data_valid), //output
  //   .mem_req_data_ready(mem_req_data_ready), //input
  //   .mem_req_data_bits(mem_req_data_bits), //output

  //   .mem_req_data_mask(mem_req_data_mask), //output // byte level masking

  //   .mem_resp_valid(mem_resp_valid), //input
  //   .mem_resp_data(mem_resp_data) //input
  // );

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

`include "const.vh"

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



module Rst_Reg #(parameter size = 32)(
    clk,
    rst,
    data_i,
    data_o
);

  //I/O ports
  input clk;
  input rst;
  input [size-1:0] data_i;
  output reg [size-1:0] data_o;

  //Internal Signals
  //Main function

  always @(posedge clk or posedge rst) begin
    if (rst) data_o <= `PC_RESET;
    else data_o <= data_i;
  end

endmodule

module StoreMask #(
    parameter size = 32
) (
    instr_func3_X,
    instr_op_X,
    RS2_data_fw,
    MemWrite_data,
    addr,
    mask
);
    //I/O
    input [2:0] instr_func3_X;
    input [6:0] instr_op_X;
    input [size-1:0] RS2_data_fw;
    input [1:0] addr;
    output [size-1:0] MemWrite_data;
    output [3:0] mask;


    //Main Function
    assign mask = ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 0) ? 4'b0001 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 1) ? 4'b0010 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 2) ? 4'b0100 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 3) ? 4'b1000 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 0) ? 4'b0011 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 1) ? 4'b0011 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 2) ? 4'b1100 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 3) ? 4'b1100 :
                  ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SW)) ? 4'b1111 : 0;

    assign MemWrite_data = ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB)) ? {4{RS2_data_fw[7:0]}} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 1) ? {24'd0,RS2_data_fw[15:8]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 2) ? {24'd0,RS2_data_fw[23:16]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SB) && addr == 3) ? {24'd0,RS2_data_fw[31:24]} :
                           ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH)) ? {2{RS2_data_fw[15:0]}} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 1) ? {16'd0,RS2_data_fw[15:0]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 2) ? {16'd0,RS2_data_fw[31:16]} :
                        //    ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SH) && addr == 3) ? {16'd0,RS2_data_fw[31:16]} :
                           ((instr_op_X == `OPC_STORE) && (instr_func3_X == `FNC_SW) ) ? RS2_data_fw : 0;
endmodule
