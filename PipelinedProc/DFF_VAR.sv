`timescale 1ns/10ps

module DFF_VAR #(parameter WIDTH = 64) (q, d, reset, clk);
	output logic [WIDTH-1:0] q;
	input logic [WIDTH-1:0] d;
	input logic reset, clk;
	
	initial assert(WIDTH>0);
	
	genvar i;
	
	generate
		for(i=0; i<WIDTH; i++) begin : eachDff
			D_FF d (.q(q[i]), .d(d[i]), .reset(reset), .clk);
		end
	endgenerate

endmodule