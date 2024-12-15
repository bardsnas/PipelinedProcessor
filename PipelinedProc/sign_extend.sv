`timescale 1ns/10ps

module sign_extend #(parameter IN_WIDTH) (
    input logic signed [IN_WIDTH-1:0] in,
    output logic signed [63:0] out
);
    logic signed [63:0] out1;
	 logic signed [63:0] out2;
	 // Sign extend the input to 64 bits
	 assign out1 = {{(64-IN_WIDTH){in[IN_WIDTH-1]}}, in};
    assign out2 = {{(64-IN_WIDTH){in[IN_WIDTH-2]}}, in};
	 
	 assign out = (in[IN_WIDTH-2])? out1 : out2;
endmodule