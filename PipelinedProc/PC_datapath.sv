`timescale 1ns/10ps

module PC_datapath(instruction, UncondBr, BrTaken, PC_in, PC_out);
	input logic [31:0] instruction;
	input logic UncondBr, BrTaken;
	input logic [63:0] PC_in;
	output logic [63:0] PC_out;
	
	logic [63:0] min0;
	logic [63:0] min1;
	logic [63:0] mout1;
	logic [63:0] shiftOut;
	logic [63:0] min11;
	logic [63:0] min00;
	
	sign_extend #(19) se1(.in(instruction[23:5]), .out(min0));
	sign_extend #(26) se2(.in(instruction[25:0]), .out(min1));
	
	mux2_1 m1(.i0(min0), .i1(min1), .sel(UncondBr), .out(mout1));
	
	shifter s1(.value(mout1), .direction(0), .distance(2), .result(shiftOut));
	
	assign min11 = shiftOut - 4 + PC_in;
	
	assign min00 = PC_in + 4;
	
	mux2_1 m2(.i0(min00), .i1(min11), .sel(BrTaken), .out(PC_out));
	
	
endmodule 
	