// Cheat Code handling by Kitrinx
// Apr 21, 2019

// Code layout:
// {clock bit, code flags,     32'b address, 32'b compare, 32'b replace}
//  128        127:96          95:64         63:32         31:0
// Integer values are in BIG endian byte order, so it up to the loader
// or generator of the code to re-arrange them correctly.

module CODES(
	input  clk,        // Best to not make it too high speed for timing reasons
	input  reset,      // This should only be triggered when a new rom is loaded or before new codes load, not warm reset
	input  enable,
	output available,
	input  [128:0] code,
	input  [ADDR_WIDTH - 1:0] addr_in,
	input  [DATA_WIDTH - 1:0] data_in,
	output [DATA_WIDTH - 1:0] data_out
);

parameter ADDR_WIDTH   = 16; // Not more than 32
parameter DATA_WIDTH   = 8;  // Not more than 32
parameter MAX_CODES    = 32;
parameter BIG_ENDIAN   = 0;

localparam INDEX_SIZE  = $clog2(MAX_CODES-1); // Number of bits for index, must accomodate MAX_CODES

localparam DATA_S      = DATA_WIDTH - 1;
localparam COMP_S      = DATA_S + DATA_WIDTH;
localparam ADDR_S      = COMP_S + ADDR_WIDTH;
localparam COMP_F_S    = ADDR_S + 1;
localparam CODE_WIDTH  = COMP_F_S + 1;
localparam ENA_F_S     = CODE_WIDTH + 1;

localparam NO_ADDR_LSB = (DATA_WIDTH == 16) ? 1 : 0;

reg [ENA_F_S:0] codes[MAX_CODES];

wire [ADDR_WIDTH-1: 0] code_addr    = code[64+:ADDR_WIDTH] ^ BIG_ENDIAN[0];
wire [DATA_WIDTH-1: 0] code_compare = code[32+:DATA_WIDTH];
wire [DATA_WIDTH-1: 0] code_data    = code[0+:DATA_WIDTH];
wire code_comp_f = code[96];
wire code_width  = code[97] && (DATA_WIDTH == 16);

// If MAX_INDEX is changes, these need to be made larger
wire [INDEX_SIZE-1:0] index, dup_index;
reg [INDEX_SIZE:0] next_index;
wire found_dup;

assign index = found_dup ? dup_index : next_index[INDEX_SIZE-1:0];

// See if the code exists already, so it can be disabled if loaded again
always_comb begin
	int x;
	dup_index = 0;
	found_dup = 0;

	for (x = 0; x < MAX_CODES; x = x + 1) begin
		if (codes[x][ADDR_S-:ADDR_WIDTH] == code_addr) begin
			dup_index = x[INDEX_SIZE-1:0];
			found_dup = 1;
		end
	end
end

assign available = |next_index;

reg code_change;
always_ff @(posedge clk) begin
	int x;
	if (reset) begin
		next_index <= 0;
		code_change <= 0;
		for (x = 0; x < MAX_CODES; x = x + 1) codes[x] <= '0;
	end else begin
		code_change <= code[128];
		if (code[128] && ~code_change && (found_dup || next_index < MAX_CODES)) begin // detect posedge
			// replace it if the same address, otherwise, add a new code
			codes[index] <= {1'b1, code_width, code_comp_f, code_addr, code_compare, code_data};
			if (~found_dup) next_index <= next_index + 1'b1;
		end
	end
end

always_comb begin
	int x;
	data_out = data_in;

	if (enable) begin
		for (x = 0; x < MAX_CODES; x = x + 1) begin
			if (codes[x][ENA_F_S] && codes[x][ADDR_S-:(ADDR_WIDTH-NO_ADDR_LSB)] == addr_in[ADDR_WIDTH-1:NO_ADDR_LSB]) begin
				if (!codes[x][COMP_F_S] || (
					(DATA_WIDTH == 8 || !codes[x][CODE_WIDTH]) ? (data_in       == codes[x][COMP_S-:DATA_WIDTH]) : 
					(codes[x][ADDR_S-ADDR_WIDTH+1])            ? (data_in[15:8] == codes[x][(COMP_S-DATA_WIDTH+1) +:8]) : 
					                                             (data_in[7:0]  == codes[x][(COMP_S-DATA_WIDTH+1) +:8]) ))
				begin
					if(DATA_WIDTH == 8 || !codes[x][CODE_WIDTH])  data_out       = codes[x][DATA_S-:DATA_WIDTH];
					else if (codes[x][ADDR_S-ADDR_WIDTH+1])       data_out[15:8] = codes[x][(DATA_S-DATA_WIDTH+1) +:8];
					else                                          data_out[7:0]  = codes[x][(DATA_S-DATA_WIDTH+1) +:8];
				end
			end
		end
	end
end

endmodule
