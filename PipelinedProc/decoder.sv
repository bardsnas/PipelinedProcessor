`timescale 1ns/10ps

module decoder (register, writeRegister,regWrite);
	
	output logic [31:0] register;
	input logic [4:0] writeRegister;
	input logic regWrite;
	
	genvar i;
	generate
		for (i = 0; i < 32; i = i + 1) begin : decoder_loop
			// Instantiate logic for each bit of the register
			wire [4:0] writeRegister_xor;
			wire select;
			wire lsb;

			// XOR each bit of writeRegister with the loop index
			xor #(50ps) (writeRegister_xor[0], writeRegister[0], i[0]);
			xor #(50ps) (writeRegister_xor[1], writeRegister[1], i[1]);
			xor #(50ps) (writeRegister_xor[2], writeRegister[2], i[2]);
			xor #(50ps) (writeRegister_xor[3], writeRegister[3], i[3]);
			xor #(50ps) (writeRegister_xor[4], writeRegister[4], i[4]);

			// AND all bits of the XOR result to check if writeRegister equals i
			or #(50ps) (lsb,  writeRegister_xor[0], writeRegister_xor[1], 
					writeRegister_xor[2]);
			nor #(50ps) (select, lsb, writeRegister_xor[3], writeRegister_xor[4]);
			// not more than 4 inputs per gate!!

			// AND with regWrite to determine the final output
			and #(50ps) (register[i], select, regWrite);
		end
	endgenerate
endmodule

module DECODER_testbench();

	logic [31:0] register;
	logic [4:0] writeRegister;
	logic regWrite;
	
	decoder dut (.register, .writeRegister, .regWrite);
	
		
	initial begin // begin test
		writeRegister = 5'd3; regWrite = 1'b0; #10;
		writeRegister = 5'd4; regWrite = 1'b0; #10;
		writeRegister = 5'd3; regWrite = 1'b1; #10;
		writeRegister = 5'd4; regWrite = 1'b1; #10;
		writeRegister = 5'd31; regWrite = 1'b0; #10;
		writeRegister = 5'd30; regWrite = 1'b0; #10;
		writeRegister = 5'd31; regWrite = 1'b1; #10;
		writeRegister = 5'd30; regWrite = 1'b1; #10;
	end
		
endmodule