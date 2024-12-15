`timescale 1ns/10ps

module register #(parameter WIDTH) (wr, in, out, clk);
	output logic [31:0][WIDTH-1:0] out;
	input logic [WIDTH-1:0] in;
	input logic [31:0] wr;
	input logic clk;
	
	genvar i;
	generate
		for(i=0; i<31; i++) begin : reg32
			logic [WIDTH-1:0]regIn;
			logic [WIDTH-1:0]regOut;		
			mux2_1 mux1 (.i0(regOut), .i1(in), .sel(wr[i]), .out(regIn));
			DFF_VAR #(WIDTH) reg1 (.q(out[i]), .d(regIn), .clk(clk));
			assign regOut = out[i];
		end
		
	endgenerate
	
	assign out[31] = 64'b0;
endmodule

module register_testbench();

	parameter ClockDelay = 2500; 

	logic [31:0][63:0] out;
	logic [63:0] in;
	logic [31:0] wr; 
	logic clk;
	register #(64) dut (.wr, .in, .out, .clk);
	
	initial begin
		clk <= 0;
		forever #(ClockDelay/2) clk <= ~clk;
	end
	
	initial begin // begin test
		wr = 32'd1; in = 64'd1; @(posedge clk);
		wr = 32'd2; in = 64'd2; @(posedge clk);
		wr = 32'd4; in = 64'd3; @(posedge clk);
		wr = 32'd8; in = 64'd4; @(posedge clk);
		wr = 32'd16; in = 64'd5; @(posedge clk);
		wr = 32'd32; in = 64'd6; @(posedge clk);
		wr = 32'd64; in = 64'd7; @(posedge clk);
		wr = 32'd128; in = 64'd8; @(posedge clk);
		$stop;
	end
	
endmodule
