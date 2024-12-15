`timescale 1ns/10ps

module cntrl_main(instruction, ALUSrc, MemtoReg, RegWrite, 
						MemWrite, ALUOp, Imm, dir, mush, wr);

	output logic 	ALUSrc, MemtoReg, RegWrite, 
						MemWrite, Imm, dir, wr;
	output logic [1:0] mush;
	logic [1:0] mush1, mush2;
	
	output logic [2:0] ALUOp;
	
	input logic [31:0] instruction;

	logic 	ALUSrc1, MemtoReg1, RegWrite1, 
				MemWrite1, Imm1, dir1;
	logic [2:0] ALUOp1;
	logic 	ALUSrc2, MemtoReg2, RegWrite2, 
				MemWrite2, Imm2, dir2;
	logic [2:0] ALUOp2;
	
	
	/*assign ADDI 	= 11'b100100 01000; // unique
	assign ADDS 	= 11'b101010 11000; // unique
	assign B 		= 11'b000101 xxxxx; // unique
	assign B_cond 	= 11'b010101 00xxx; // unique
	assign CBZ 		= 11'b101101 00xxx; // unique
	assign LDUR 	= 11'b111110 00010; // same as STUR
	assign LSL		= 11'b110100 11011; // same as LSR
	assign LSR		= 11'b110100 11010; // same as LSL
	assign MUL		= 11'b100110 11000; // unique
	assign STUR 	= 11'b111110 00000; // same as LDUR
	assign SUBS 	= 11'b111010 11000; // unique
	*/
	always_comb begin
		case (instruction[31:26])
		
			6'b100100: begin // ADDI
					ALUSrc = 1;
					MemtoReg = 0; 		RegWrite = 1;
					MemWrite = 0; 		ALUOp = 3'b010;
					Imm = 1;				dir = 1'bx;
					mush = 2'b00;		wr = 0;
			end
			
			6'b101010: begin // ADDS
					ALUSrc = 0;
					MemtoReg = 0;		RegWrite = 1;
					MemWrite = 0; 		ALUOp = 3'b010;
					Imm = 1'bx;			dir = 1'bx;
					mush = 2'b00;		wr = 1;
			end
			
			6'b000101: begin // B
					ALUSrc = 1'bx; 
					MemtoReg = 1'bx; 	RegWrite = 0; 
					MemWrite = 0; 		ALUOp = 1'bx;
					Imm = 1'bx;			dir = 1'bx;
					mush = 2'b00;		wr = 0;
			end
			
			6'b010101: begin	// B.cond
					ALUSrc = 1'bx;
					MemtoReg = 1'bx; 	RegWrite = 0;
					MemWrite = 0;	 	ALUOp = 1'bx;
					Imm = 1'bx;			dir = 1'bx;
					mush = 2'b00;		wr = 0;
			end
			
			6'b101101: begin // CBZ
					ALUSrc = 0; 	
					MemtoReg = 1'bx; 	RegWrite = 0;
					MemWrite = 0; 		ALUOp = 3'b000;
					Imm = 1'bx;			dir = 1'bx;
					mush = 2'b00;		wr = 1;
			end
			6'b111110: begin	// LDUR/STUR
					ALUSrc = ALUSrc1; 	
					MemtoReg = MemtoReg1; 		RegWrite = RegWrite1;
					MemWrite = MemWrite1; 		ALUOp = ALUOp1;
					Imm = Imm1;				dir = dir1;
					mush = mush1;			wr = 0;
			end	
			6'b111010: begin	// SUBS
					ALUSrc = 0; 	
					MemtoReg = 0; 		RegWrite = 1;
					MemWrite = 0; 		ALUOp = 3'b011;
					Imm = 1'bx;			dir = 1'bx;
					mush = 2'b00;		wr = 1;
			end
			6'b110100: begin // LSL/LSR
					ALUSrc = ALUSrc2; 	
					MemtoReg = MemtoReg2; 		RegWrite = RegWrite2;
					MemWrite = MemWrite2; 		ALUOp = ALUOp2;
					Imm = Imm2;					dir = dir2;
					mush = mush2;				wr = 0;
			end
			6'b100110: begin	// MUL
					ALUSrc = 1'b0; 	
					MemtoReg = 1'bx; 		RegWrite = 1;
					MemWrite = 0; 		ALUOp = 3'bx;
					Imm = 1'bx;			dir = 1'bx;
					mush = 2'b10;		wr = 0;
			end
			default: begin
					ALUSrc = 1'bx; 	
					MemtoReg = 1'bx; 		RegWrite = 1'bx;
					MemWrite = 1'bx; 		ALUOp = 3'bx;
					Imm = 1'bx;				dir = 1'bx;
					mush = 2'bx;			wr = 1'bx;
			end	
		endcase
	end
	
	always_comb begin
		case (instruction[25:21])
		
			5'b00010: begin
					ALUSrc1 = 1; 	
					MemtoReg1 = 1; 		RegWrite1 = 1;
					MemWrite1 = 0; 		ALUOp1 = 3'b010;
					Imm1 = 0;	dir1 = 1'bx;
					mush1 = 2'b00;
			end
			
			5'b00000: begin
					ALUSrc1 = 1;
					MemtoReg1 = 1'bx;		RegWrite1 = 0;
					MemWrite1 = 1; 		ALUOp1 = 3'b010;
					Imm1 = 0;	 dir1 = 1'bx;
					mush1 = 2'b00;	
			end
			
			default: begin
					ALUSrc1 = 1'bx; 	
					MemtoReg1 = 1'bx; 		RegWrite1 = 1'bx;
					MemWrite1 = 1'bx; 		ALUOp1 = 3'bx;
					Imm1 = 1'bx;	dir1 = 1'bx;
					mush1 = 2'bx;	
			end
		endcase
	end
	
	always_comb begin
		case (instruction[25:21])
		
			5'b11011: begin // LSL
					ALUSrc2 = 1'bx; 	
					MemtoReg2 = 1'bx; 		RegWrite2 = 1;
					MemWrite2 = 1'bx; 		ALUOp2 = 3'b010;
					Imm2 = 1'bx;	dir2 = 0;
					mush2 = 2'b01;	
			end
			
			5'b11010: begin // LSR
					ALUSrc2 = 1'bx;
					MemtoReg2 = 1'bx;		RegWrite2 = 1;
					MemWrite2 = 1'bx; 		ALUOp2 = 3'bx;
					Imm2 = 1'bx;	dir2 = 1;
					mush2 = 2'b01;
			end
			
			default: begin
					ALUSrc2 = 1'bx; 	
					MemtoReg2 = 1'bx; 		RegWrite2 = 1'bx;
					MemWrite2 = 1'bx; 		ALUOp2 = 3'bx;
					Imm2 = 1'bx;		dir2 = 1'bx;
					mush2 = 2'bx;
			end
		endcase
	end
endmodule

//module controller_testbench();
//
//	logic [63:0] address;
//	logic Reg2Loc, ALUSrc, MemtoReg, RegWrite, MemWrite, BrTaken, UncondBr, ALUOp;
//	
//	controller dut (.address, .Reg2Loc, .ALUSrc, .MemtoReg, .RegWrite, .MemWrite, .BrTaken, .UncondBr, .ALUOp);
//	
//	initial begin // begin test
//		address = 64b'