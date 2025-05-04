`include "util.vh"
`include "const.vh"

module cache #
(
  parameter LINES = 64,
  parameter CPU_WIDTH = `CPU_INST_BITS,
  parameter WORD_ADDR_BITS = `CPU_ADDR_BITS-`ceilLog2(`CPU_INST_BITS/8)
)
(
  input clk,
  input reset,

  input                       cpu_req_valid,
  output                      cpu_req_ready,
  input [WORD_ADDR_BITS-1:0]  cpu_req_addr,
  input [CPU_WIDTH-1:0]       cpu_req_data,
  input [3:0]                 cpu_req_write,

  output                      cpu_resp_valid,
  output [CPU_WIDTH-1:0]      cpu_resp_data,

  output                      mem_req_valid,
  input                       mem_req_ready,
  output [WORD_ADDR_BITS-1:`ceilLog2(`MEM_DATA_BITS/CPU_WIDTH)] mem_req_addr,
  output                           mem_req_rw,
  output                           mem_req_data_valid,
  input                            mem_req_data_ready,
  output [`MEM_DATA_BITS-1:0]      mem_req_data_bits,
  // byte level masking
  output [(`MEM_DATA_BITS/8)-1:0]  mem_req_data_mask,

  input                       mem_resp_valid,
  input [`MEM_DATA_BITS-1:0]  mem_resp_data
);

localparam DATA_SRAM_ADDR_BITS = $clog2(256); // 8
localparam METADATA_SRAM_ADDR_BITS = $clog2(64); // 6

wire data0_we;
wire [3:0] data0_wmask;
wire [DATA_SRAM_ADDR_BITS-1:0] data0_addr;
wire [31:0] data0_din;
wire [31:0] data0_dout;

wire data1_we;
wire [3:0] data1_wmask;
wire [DATA_SRAM_ADDR_BITS-1:0] data1_addr;
wire [31:0] data1_din;
wire [31:0] data1_dout;

wire data2_we;
wire [3:0] data2_wmask;
wire [DATA_SRAM_ADDR_BITS-1:0] data2_addr;
wire [31:0] data2_din;
wire [31:0] data2_dout;

wire data3_we;
wire [3:0] data3_wmask;
wire [DATA_SRAM_ADDR_BITS-1:0] data3_addr;
wire [31:0] data3_din;
wire [31:0] data3_dout;

wire metadata_we;
wire [3:0] metadata_wmask;
wire [METADATA_SRAM_ADDR_BITS-1:0] metadata_addr;
wire [31:0] metadata_din;
wire [31:0] metadata_dout;

sram22_256x32m4w8 data_sram0 (
	.clk(clk),
	.we(data0_we),
	.wmask(data0_wmask),
	.addr(data0_addr),
	.din(data0_din),
	.dout(data0_dout)
);
sram22_256x32m4w8 data_sram1 (
	.clk(clk),
	.we(data1_we),
	.wmask(data1_wmask),
	.addr(data1_addr),
	.din(data1_din),
	.dout(data1_dout)
);
sram22_256x32m4w8 data_sram2 (
	.clk(clk),
	.we(data2_we),
	.wmask(data2_wmask),
	.addr(data2_addr),
	.din(data2_din),
	.dout(data2_dout)
);
sram22_256x32m4w8 data_sram3 (
	.clk(clk),
	.we(data3_we),
	.wmask(data3_wmask),
	.addr(data3_addr),
	.din(data3_din),
	.dout(data3_dout)
);
wire [`MEM_DATA_BITS-1:0] subline_dout; // 128 bits
assign subline_dout = {data0_dout, data1_dout, data2_dout, data3_dout};
wire [CPU_WIDTH-1:0] subsubline_douts [3:0]; // 128 bits
assign subsubline_douts[0] = data0_dout;
assign subsubline_douts[1] = data1_dout;
assign subsubline_douts[2] = data2_dout;
assign subsubline_douts[3] = data3_dout;

sram22_64x32m4w8 metadata_sram (
	.clk(clk),
	.we(metadata_we),
	.wmask(metadata_wmask),
	.addr(metadata_addr),
	.din(metadata_din),
	.dout(metadata_dout)
);

reg [64-1:0] valids;

localparam NUM_STATES = 5;
localparam STATE_BITS = $clog2(NUM_STATES);

localparam [STATE_BITS-1:0] STATE_IDLE = 0;
localparam [STATE_BITS-1:0] STATE_READ = 1;
localparam [STATE_BITS-1:0] STATE_FILL = 2;
localparam [STATE_BITS-1:0] STATE_SEND_ADDR = 3;
localparam [STATE_BITS-1:0] STATE_SEND_DATA = 4;

reg [STATE_BITS-1:0] state;
reg [STATE_BITS-1:0] next_state;
reg [1:0] ctr;
reg prev_state_is_fill;

always @(posedge clk) begin
	if (reset)
		state <= STATE_IDLE;
	else
		state <= next_state;
end
always @(posedge clk) begin
	if (state != STATE_FILL && next_state==STATE_FILL) ctr <= 0;
	else ctr <= ctr+mem_resp_valid;
end
always @(posedge clk) begin
	if (reset) prev_state_is_fill <= 0;
	else prev_state_is_fill <= state==STATE_FILL;
end


wire cpu_req_fire;
assign cpu_req_fire = cpu_req_ready & cpu_req_valid;
wire mem_req_fire, mem_req_data_fire;
assign mem_req_fire = mem_req_ready & mem_req_valid;
assign mem_req_data_fire = mem_req_data_ready & mem_req_data_valid;

localparam OFFSET_SIZE = 4;
localparam INDEX_SIZE = 6;
localparam TAG_SIZE = WORD_ADDR_BITS - INDEX_SIZE - OFFSET_SIZE;

// TIO for requested operation

wire [TAG_SIZE-1:0] tag;
wire [INDEX_SIZE-1:0] index;
wire [OFFSET_SIZE-1:0] offset;
reg [TAG_SIZE-1:0] reg_tag;
reg [INDEX_SIZE-1:0] reg_index;
reg [OFFSET_SIZE-1:0] reg_offset;
reg [CPU_WIDTH-1:0] reg_write_data;
reg [3:0] reg_write_mask;

assign tag = cpu_req_addr[WORD_ADDR_BITS-1 : INDEX_SIZE+OFFSET_SIZE];
assign index = cpu_req_addr[INDEX_SIZE+OFFSET_SIZE-1 : OFFSET_SIZE];
assign offset = cpu_req_addr[OFFSET_SIZE-1 : 0];

// TIO for cache line

wire [TAG_SIZE-1:0] line_tag;
wire line_valid;

assign line_tag = metadata_dout[TAG_SIZE-1:0];
assign line_valid = valids[reg_index];

//

wire hit;
wire write; // 0 => read, 1 => write
assign hit = (line_tag == reg_tag) && (line_valid);
assign write = (cpu_req_write != 4'b0); // ~write <=> read


// SRAM control
// Data
assign data0_we = mem_resp_valid;
assign data1_we = mem_resp_valid;
assign data2_we = mem_resp_valid;
assign data3_we = mem_resp_valid;
assign data0_wmask = 4'b1111;
assign data1_wmask = 4'b1111;
assign data2_wmask = 4'b1111;
assign data3_wmask = 4'b1111;
// 128 bits received from different SRAM should be consecutive
// so bottom two bits of offset select which SRAM a word is from
assign data0_addr = cpu_req_fire ? {index, offset[OFFSET_SIZE-1:OFFSET_SIZE-2]} : {reg_index, ctr};
assign data1_addr = cpu_req_fire ? {index, offset[OFFSET_SIZE-1:OFFSET_SIZE-2]} : {reg_index, ctr};
assign data2_addr = cpu_req_fire ? {index, offset[OFFSET_SIZE-1:OFFSET_SIZE-2]} : {reg_index, ctr};
assign data3_addr = cpu_req_fire ? {index, offset[OFFSET_SIZE-1:OFFSET_SIZE-2]} : {reg_index, ctr};
// Don't worry about updating inputs because we don't fill on write
assign data0_din = mem_resp_data[CPU_WIDTH-1:0];
assign data1_din = mem_resp_data[CPU_WIDTH*2-1:CPU_WIDTH];
assign data2_din = mem_resp_data[CPU_WIDTH*3-1:CPU_WIDTH*2];
assign data3_din = mem_resp_data[CPU_WIDTH*4-1:CPU_WIDTH*3];

// Meta
assign metadata_we = next_state==STATE_FILL;
assign metadata_wmask = 4'b1111;
assign metadata_addr = next_state==STATE_FILL ? reg_index : index;
assign metadata_din = reg_tag;
// cpu_req_ready => cpu_resp_valid
assign cpu_req_ready = (state == STATE_IDLE) | ((state == STATE_READ) & hit);
assign cpu_resp_valid = ((state == STATE_READ) & hit) || (state==STATE_IDLE && prev_state_is_fill);
// Output from cache in read; output from stored data in IDLE in case CPU busy
assign cpu_resp_data = state==STATE_READ ? subsubline_douts[reg_offset[OFFSET_SIZE-3:0]] : reg_write_data;

// Remember input we're currently processing
always @(posedge clk) begin
	if (cpu_req_fire) begin
		reg_tag <= tag;
		reg_index <= index;
		reg_offset <= offset;

		reg_write_data <= cpu_req_data;
		reg_write_mask <= cpu_req_write;
	end
end

// Mem interface
assign mem_req_valid = (state==STATE_READ && !hit) || (cpu_req_fire && cpu_req_write!=4'b0) || state==STATE_SEND_ADDR; //(cpu_req_fire && cpu_req_write!=4'b0)
assign mem_req_addr = (cpu_req_fire && cpu_req_write!=4'b0 ? cpu_req_addr : {reg_tag, reg_index, reg_offset}) >> 2;
assign mem_req_rw = state!=STATE_READ;
assign mem_req_data_valid = state==STATE_SEND_DATA;
assign mem_req_data_bits = {96'b0, reg_write_data} << (32*(reg_offset[OFFSET_SIZE-3:0]));
assign mem_req_data_mask = {12'b0, reg_write_mask} << (4*(reg_offset[OFFSET_SIZE-3:0]));

// Grab read while filling cache
always @(posedge clk) begin
	if (state==STATE_FILL && ctr==reg_offset[OFFSET_SIZE-1:OFFSET_SIZE-2])
		case (reg_offset[OFFSET_SIZE-3:0])
			0: reg_write_data <= mem_resp_data[CPU_WIDTH*1-1:CPU_WIDTH*0];
			1: reg_write_data <= mem_resp_data[CPU_WIDTH*2-1:CPU_WIDTH*1];
			2: reg_write_data <= mem_resp_data[CPU_WIDTH*3-1:CPU_WIDTH*2];
			3: reg_write_data <= mem_resp_data[CPU_WIDTH*4-1:CPU_WIDTH*3];
		endcase
	else if (state==STATE_READ && hit) reg_write_data <= cpu_resp_data; // keep output while in idle TODO set valid in idle
end

always @(posedge clk) begin
	if (reset) valids <= 0;
	else begin
		valids <= valids;
		if (state==STATE_SEND_DATA) valids[reg_index] <= 1'b0; //TODO: Only invalidate when tag matches, or rewrite with correct data
		else if (next_state==STATE_FILL) valids[reg_index] <= 1'b1;
	end
end

reg [STATE_BITS-1:0] idle_next_state;
always @(*) begin
	if (cpu_req_fire) begin
		idle_next_state = cpu_req_write==4'b0 ? STATE_READ : (mem_req_fire ? STATE_SEND_DATA : STATE_SEND_ADDR); // Currently goes into either write state. If changed, change req_valid
	end else
		idle_next_state = STATE_IDLE;
end

always @(*) begin
	next_state = {STATE_BITS{1'bx}};
	case (state)
		STATE_IDLE: begin
			next_state = idle_next_state;
		end
		STATE_READ: begin
			if (hit) begin // hit
				next_state = idle_next_state;
			end else begin // miss
				next_state = mem_req_fire ? STATE_FILL : STATE_READ;
			end
		end
		STATE_FILL: begin
			next_state = mem_resp_valid && ctr==3 ? STATE_IDLE : STATE_FILL;
		end
		STATE_SEND_ADDR: begin
			next_state = mem_req_fire ? STATE_SEND_DATA : STATE_SEND_ADDR;
		end
		STATE_SEND_DATA: begin
			next_state = mem_req_data_fire ? STATE_IDLE : STATE_SEND_DATA;
		end
	endcase
end

endmodule