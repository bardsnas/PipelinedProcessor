`timescale 1ns/10ps

module mux2_1 (
	input logic [63:0] i0,    // 64-bit input 0
   input logic [63:0] i1,    // 64-bit input 1
   input logic sel,         // select line
   output logic [63:0] out    // 64-bit output
);
	logic [63:0] i0_sel;  // Result of ANDing 'a' with ~sel
   logic [63:0] i1_sel;  // Result of ANDing 'b' with sel
   logic sel_not;       // Inverted select line

   // NOT of the select signal
   not #(50ps) not1 (sel_not, sel);

   // AND i0 and i1 with their corresponding select lines
   genvar i;
	generate
		for (i = 0; i < 64; i = i + 1) begin : andloop
			and #(50ps) and1 (i0_sel[i], i0[i], sel_not);  // a AND ~sel
			and #(50ps) and2 (i1_sel[i], i1[i], sel);      // b AND sel
			or #(50ps) or1 (out[i], i0_sel[i], i1_sel[i]);  // Final OR operation
		end
	endgenerate
		
endmodule

module mux2_1_testbench ();
	logic [63:0] i0, i1, out;
	logic sel;
	
	mux2_1 dut (.i0, .i1, .sel, .out);
	
	initial begin
		i0=64'd64357; i1=64'd26000; #10;
		sel=2'b00; #10;
		sel=2'b01; #10;
	end
endmodule