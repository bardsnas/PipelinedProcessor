`timescale 1ns/10ps

module zero_equal (value, is_zero);
	input logic [63:0] value;
	output logic is_zero;
	
	always_comb begin
		if (value == 0)
			is_zero = 1;
		else 
			is_zero = 0;
	end
endmodule
