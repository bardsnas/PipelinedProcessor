`timescale 1ns/10ps

module mux4_1_alu(i, sel, out);
	
	output logic out;
	input logic [3:0]i;
	input logic [1:0] sel;
	logic v0, v1;
	
	mux2_1_alu m0(.i0(i[0]), .i1(i[1]), .sel(sel[0]), .out(v0));
	mux2_1_alu m1(.i0(i[2]), .i1(i[3]), .sel(sel[0]), .out(v1));
	
	mux2_1_alu m (.i0(v0), .i1(v1), .sel(sel[1]), .out(out));
endmodule
	
module mux4_1_alu_testbench();
	
	logic [3:0]i;
	logic out;
	logic [1:0] sel;
	
	mux4_1_alu dut (.i, .sel, .out);

	initial begin
		i[0]=64'd64357; i[1]=64'd26000; i[2] = 64'd256; i[3] = 64'd128; #10;
		sel=2'b00; #10;
		sel=2'b01; #10;
		sel=2'b10; #10;
		sel=2'b11; #10;
	end
endmodule