`timescale 1ns/10ps

module mux2_1_5bit (
	input logic [4:0] i0,    // 64-bit input 0
   input logic [4:0] i1,    // 64-bit input 1
   input logic sel,         // select line
   output logic [4:0] out    // 64-bit output
);
	logic [4:0] i0_sel;  // Result of ANDing 'a' with ~sel
   logic [4:0] i1_sel;  // Result of ANDing 'b' with sel
   logic sel_not;       // Inverted select line

   // NOT of the select signal
   not #(50ps) not1 (sel_not, sel);

   // AND i0 and i1 with their corresponding select lines
   genvar i;
	generate
		for (i = 0; i < 5; i = i + 1) begin : andloop
			and #(50ps) and1 (i0_sel[i], i0[i], sel_not);  // a AND ~sel
			and #(50ps) and2 (i1_sel[i], i1[i], sel);      // b AND sel
			or #(50ps) or1 (out[i], i0_sel[i], i1_sel[i]);  // Final OR operation
		end
	endgenerate
		
endmodule

module mux2_1_5bit_testbench ();
	logic [4:0] i0, i1, out;
	logic sel;
	
	mux2_1_5bit dut (.i0, .i1, .sel, .out);
	
	initial begin
		i0=5'd16; i1=5'd31; #10;
		sel=1'b0; #10;
		sel=1'b1; #10;
	end
endmodule