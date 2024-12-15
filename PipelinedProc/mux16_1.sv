`timescale 1ns/10ps

module mux16_1(i, sel, out);
	output logic [63:0] out;
	input logic [15:0][63:0] i;
	input logic [3:0] sel;
	logic [63:0] v0, v1;
	
	mux8_1 m0(.i(i[7:0]), .sel(sel[2:0]), .out(v0));
	mux8_1 m1(.i(i[15:8]), .sel(sel[2:0]), .out(v1));
	
	mux2_1 m2(.i0(v0), .i1(v1), .sel(sel[3]), .out(out));
	
endmodule

module mux16_1_testbench();
	
	logic [15:0][63:0] i;
	logic [63:0] out;
	logic [3:0] sel;
	
	mux16_1 dut (.i, .sel, .out);

	initial begin
		i[0]=64'd64357; i[1]=64'd26000; i[2]=64'd24556; i[3]=64'd12328; i[4]=64'd63; i[5]=64'd31; i[6]=64'd132346; i[7]=64'd7;
		i[8]=64'd157; i[9]=64'd2803; i[10]=64'd308; i[11]=64'd64; i[12]=64'd27; i[13]=64'd879; i[14]=64'd538129; i[15]=64'd1327; #10;
		sel=4'b0000; #10;
		sel=4'b0001; #10;
		sel=4'b0010; #10;
		sel=4'b0011; #10;
		sel=4'b0100; #10;
		sel=4'b0101; #10;
		sel=4'b0110; #10;
		sel=4'b0111; #10;
		sel=4'b1000; #10;
		sel=4'b1001; #10;
		sel=4'b1010; #10;
		sel=4'b1011; #10;
		sel=4'b1100; #10;
		sel=4'b1101; #10;
		sel=4'b1110; #10;
		sel=4'b1111; #10;
	end
endmodule