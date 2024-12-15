module flag_regs #(parameter WIDTH = 4) (wr, in, out, clk);
	output logic [WIDTH-1:0] out;
	input logic [WIDTH-1:0] in;
	input logic wr;
	input logic clk;
	
	genvar i;
	generate
		for(i=0; i<WIDTH; i++) begin : reg4
			logic regIn;
			logic regOut;		
			
			mux2_1 mux1 (.i0(regOut), .i1(in[i]), .sel(wr), .out(regIn));
			
			D_FF flagreg (.q(out[i]), .d(regIn), .clk(clk));
			assign regOut = out[i];
		end
		
	endgenerate
endmodule