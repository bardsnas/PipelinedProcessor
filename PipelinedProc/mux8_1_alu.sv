`timescale 1ns/10ps

module mux8_1_alu(i, sel, out);
	output logic out;
	input logic [7:0]i;
	input logic [2:0] sel;
	logic v0, v1;
	
	mux4_1_alu m0(.i(i[3:0]), .sel(sel[1:0]), .out(v0));
	mux4_1_alu m1(.i(i[7:4]), .sel(sel[1:0]), .out(v1));
	
	mux2_1_alu m2(.i0(v0), .i1(v1), .sel(sel[2]), .out(out));
	
endmodule

module mux8_1_alu_testbench();
	
	logic [7:0] i;
	logic out;
	logic [2:0] sel;
	
	mux8_1_alu dut (.i, .sel, .out);

	initial begin
		i[0]=64'd64357; i[1]=64'd26000; i[2]=64'd24556; i[3]=64'd12328; i[4]=64'd63; i[5]=64'd31; i[6]=64'd132346; i[7]=64'd7; #10;
		sel=3'b000; #10;
		sel=3'b001; #10;
		sel=3'b010; #10;
		sel=3'b011; #10;
		sel=3'b100; #10;
		sel=3'b101; #10;
		sel=3'b110; #10;
		sel=3'b111; #10;
	end
endmodule