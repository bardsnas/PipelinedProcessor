`timescale 1ns/10ps

module regfile(ReadData1, ReadData2, WriteData, 
					 ReadRegister1, ReadRegister2, WriteRegister,
					 RegWrite, clk
					);
	input logic [4:0] ReadRegister1, ReadRegister2, WriteRegister;
	input logic [63:0] WriteData;
	input logic RegWrite, clk;
	output logic [63:0] ReadData1, ReadData2;
	
	logic [31:0] regIn;
	logic [31:0][63:0] muxIn;
	
	decoder d1(.register(regIn), .writeRegister(WriteRegister), .regWrite(RegWrite));
	register #(64) r1(.wr(regIn), .in(WriteData), .out(muxIn), .clk(clk));
	mux32_1 m1(.i(muxIn), .sel(ReadRegister1), .out(ReadData1));
	mux32_1 m2(.i(muxIn), .sel(ReadRegister2), .out(ReadData2));
	
endmodule
