`timescale 1ns/10ps

module pipelineCPU(clk, reset);
	
	logic [63:0] q;
	logic [63:0] d;
	logic [31:0] instruction;
	logic negative, zero, overflow, Reg2Loc, ALUSrc, MemtoReg, RegWrite, 
			MemWrite, Imm, BrTaken, UncondBr, carry_out, dir, wr, negOut, 
			negflag;
	logic [1:0] mush;
 	logic [2:0] ALUOp;
	input logic clk, reset;
	
	logic [3:0] frOut;
	logic [3:0] flags_in;
	logic [31:0] ifid;
	logic [4:0] Ab;
	logic [63:0] Da;
	logic [63:0] Db1;
	logic [63:0] DAddr9;
	logic [63:0] Imm12;
	logic [63:0] val;
	logic [63:0] Db2, Db3;
	logic [63:0] result;
	logic [63:0] msr_result;
	logic [63:0] data_in;
	logic [63:0] Dout;
	logic [63:0] Dw, Dw1, Dw2, Dw12, Dw3;
	logic [224:0] idexi;
	logic [224:0] idexo;
	logic [329:0] exmemi;
	logic [329:0] exmemo;
	logic [199:0] memwbi;
	logic [199:0] memwbo;    
	logic [1:0] forwardA, forwardB, forwardD;
	logic zf, zero1;
	
	
// Instruction Fetch (IF)
	DFF_VAR PC (.q(q), .d(d), .reset(reset), .clk(clk));

	instructmem instr (.address(q), .instruction(instruction), .clk(clk));
	
	PC_control pcCntrl  (.instruction(ifid), 
								.negative(negflag), .zero(zf), .overflow(frOut[3]),
								.Brtaken(BrTaken), .UncondBr(UncondBr), .Reg2Loc(Reg2Loc));
	
	PC_datapath PCData  (.instruction(ifid), 
								.UncondBr(UncondBr), .BrTaken(BrTaken), 
								.PC_in(q), .PC_out(d));
	
// IF/ID Register
	DFF_VAR #(32) IFID (.q(ifid), .d(instruction), .reset(reset), .clk(clk));
	
// Main Control
	
	flag_regs fr1 (.wr(idexo[143]), .in(flags_in), .out(frOut), .clk(clk));

	cntrl_main ctrl (.instruction(ifid), .ALUSrc(ALUSrc), .MemtoReg(MemtoReg), .RegWrite(RegWrite), 
							.MemWrite(MemWrite), .ALUOp(ALUOp), .Imm(Imm), .dir(dir), .mush(mush), .wr(wr));

	
//	Instruction Decoding (ID)
	mux2_1_5bit m1(.i0(ifid[4:0]), .i1(ifid[20:16]), .sel(Reg2Loc), .out(Ab));
	
	// idexi = {ALUSrc, MemtoReg, RegWrite, MemWrite, ALUOp, Imm, dir, mush, wr, 
	//				ReadData2, ReadData1, Rm, Rn, Rd}
	regfile RF (.ReadData1(Da), .ReadData2(Db2), .WriteData(Dw), 
					 .ReadRegister1(ifid[9:5]), .ReadRegister2(Ab), .WriteRegister(memwbo[4:0]),
					 .RegWrite(memwbo[199]), .clk(~clk));
	sign_extend #(9) se1(.in(ifid[20:12]), .out(DAddr9));
	
	sign_extend #(12) se2(.in(ifid[21:10]), .out(Imm12));
	
	mux2_1 m2(.i0(DAddr9), .i1(Imm12), .sel(Imm), .out(val));
	
	mux2_1 m3(.i0(Db2), .i1(val), .sel(ALUSrc), .out(Db3));
					 
	// forwarding
	
	forwardunit forunit (.rd_idex(idexo[4:0]), .rd_exmem(exmemo[4:0]), .rd_memwb(memwbo[4:0]),
								.rn(ifid[9:5]), .rm(ifid[20:16]), 
								.rw_idex(idexo[152]), .rw_exmem(exmemo[264]), .rw_memwb(memwbo[199]),
								.ALUSrc(ALUSrc),
								.forwardA(forwardA), .forwardB(forwardB));
	// forwarding A unit
	logic [3:0][63:0] ia;
	assign ia = {Dw, Dw1, msr_result, Da}; //exmemo[132:69]
	mux4_1 fma (.i(ia), .sel(forwardA), .out(idexi[78:15]));
	
	// forwarding B unit	
	logic [3:0][63:0] ib;
	assign ib = {Dw, exmemo[132:69], msr_result, Db3};
	mux4_1 fmb (.i(ib), .sel(forwardB), .out(idexi[142:79]));
	
	zero_equal ze (.value(idexi[142:79]), .is_zero(zf));
	
	
// ID/EX Register
	assign idexi[14:10] = ifid[20:16];  //Rm
	assign idexi[9:5] = ifid[9:5];  //Rn
	assign idexi[4:0] = ifid[4:0]; //Rd
								//  154       153       152       151  [150:148] 147 146[145:144] 143
	assign idexi[154:143] = {ALUSrc, MemtoReg, RegWrite, MemWrite, ALUOp, Imm, dir, mush, wr};
	assign idexi[218:155] = Db2;
	assign idexi[224:219] = ifid[15:10];
	DFF_VAR #(225) IDEX (.q(idexo), .d(idexi), .reset(reset), .clk(clk));
	
// Execute (EX)


											  
//	mux2_1 fma (.i0(idexo[218:155]), .i1(idexo[78:15]), .sel(forwardB), .out(Da));
	
	alu alu1(.A(idexo[78:15]), .B(idexo[142:79]), .cntrl(idexo[150:148]), .result(result), .negative(negative), 
				.zero(zero), .overflow(overflow), .carry_out(carry_out));
				
	flag_regs #(1) negreg (.wr(idexo[143]), .in(negative), .out(negOut), .clk(clk));
	
	mux2_1_alu negmux (.i0(negOut), .i1(negative), .sel(idexo[143]), .out(negflag)); 
	
	
	assign flags_in[3] = overflow;
	assign flags_in[2] = negflag;
	assign flags_in[1] = zero;
	assign flags_in[0] = carry_out;
	
	shifter shft (.value(idexo[78:15]), .direction(idexo[146]), .distance(idexo[224:219]), .result(Dw2));
	
	logic [63:0] multh;
	mult mul (.A(idexo[78:15]), .B(idexo[142:79]), .doSigned(1'b1), .mult_low(Dw3), .mult_high(multh));
	
	logic [3:0][63:0] i_msr;
//	assign i_msr[0] = result; // result from ALU
//	assign i_msr[1] = Dw2; // result from Shift
//	assign i_msr[2] = Dw3; // result from Mult
//	assign i_msr[3] = 0;
	assign i_msr = {64'd0, Dw3, Dw2, result};
	mux4_1 msr (.i(i_msr), .sel(idexo[145:144]), .out(msr_result));
	

// EX/MEM Register
					//  						265		 		264			263 	[262:261] [260:197][196:133][132:69]		[68:5]			[4:0] 
	// exmemi = 	{						MemtoReg, 		RegWrite, 	MemWrite, 		mush,   Dw3, 	Dw2, 		result, 		ReadData2, 		 Rd}
	assign exmemi = {idexo[218:155], idexo[153], idexo[152], idexo[151], idexo[145:144], Dw3, Dw2, msr_result, idexo[142:79], idexo[4:0]};
	DFF_VAR #(330) EXMEM (.q(exmemo), .d(exmemi), .reset(reset), .clk(clk));
	
	dataForward df1(.rd_exmemo(exmemo[4:0]), .rd_memwbo(memwbo[4:0]), .forwardD(forwardD));
	mux2_1 fmd (.i0(exmemo[329:266]), .i1(Dw), .sel(forwardD), .out(data_in));
// Memory (MEM)
	datamem dm1(.address(exmemo[132:69]), .write_enable(exmemo[263]), .read_enable(~exmemo[263]), 
					.write_data(data_in), .clk(clk), .xfer_size(4'h8), .read_data(Dout));
	
	mux2_1 m4(.i0(exmemo[132:69]), .i1(Dout), .sel(exmemo[265]), .out(Dw1));
// MEM/WB Register
	
	//                     199          [198:197] [196:133]  [132:69]      [68:5]      		[4:0]
	// memwbi = 		 RegWrite         mush       Dw1        Dw3        		Dw2             Rd
	assign memwbi = {exmemo[264], exmemo[262:261], Dw1, exmemo[260:197], exmemo[196:133], exmemo[4:0]};
	DFF_VAR #(200) MEMWB (.q(memwbo), .d(memwbi), .reset(reset), .clk(clk));
	
// Writeback (WB)
	logic [3:0][63:0] in;
	assign in[0] = memwbo[196:133]; // Dw1
	assign in[1] = memwbo[68:5]; // Dw2
	assign in[2] = memwbo[132:69]; // Dw3
	
	mux4_1 m5 (.i(in), .sel(memwbo[198:197]), .out(Dw));
	

endmodule

//module pipelineCPU (clk, reset);
//
//	input logic clk, reset;
//	logic 	Reg2Loc, ALUSrc, MemtoReg, RegWrite, 
//				MemWrite, Brtaken, UncondBr, Imm, dir, wr;
//	logic [1:0] mush;
//	logic [2:0] ALUOp;
//
//	logic [199:0] memwbo;
//	// IF
//	
//	logic [64:0] q, d, Dw;
//	logic [31:0] instruction;
//	
//	DFF_VAR PC (.q(q), .d(d), .reset(reset), .clk(clk));
//	
//	instructmem instr (.address(q), .instruction(instruction), .clk(clk));
//	
//	// IF/ID
//	
//	logic [31:0] ifid;
//	
//	PC_datapath PCData  (.instruction(ifid), .UncondBr(UncondBr), 
//								.BrTaken(Brtaken), .PC_in(q), .PC_out(d));
//	
//	DFF_VAR #(32) IFID (.q(ifid), .d(instruction), .reset(reset), .clk(clk));
//	
//	// ID
//	
//	logic overflow, negative, zero, carry_out;
//	logic [3:0] flags_in;
//	logic [3:0] frOut;
//	
//	assign flags_in = {overflow, negative, zero, carry_out};
//	
//	flag_regs fr1 (.wr(wr), .in(flags_in), .out(frOut), .clk(clk));
//	
//	
//	
//	cntrl_main ctrl	(.instruction(ifid), .negative(frOut[2]), .zero(zero), .overflow(frOut[3]), 
//							.Reg2Loc(Reg2Loc), .ALUSrc(ALUSrc), .MemtoReg(MemtoReg), .RegWrite(RegWrite), 
//							.MemWrite(MemWrite), .ALUOp(ALUOp), .Imm(Imm), .Brtaken(Brtaken), .UncondBr(UncondBr), 
//							.dir(dir), .mush(mush), .wr(wr));
//							
//	
//	logic [4:0] Ab;
//	
//	mux2_1_5bit m1(.i0(ifid[4:0]), .i1(ifid[20:16]), .sel(Reg2Loc), .out(Ab));
//	
//	logic [154:0] idexi;
//	// idexi = {ALUSrc, MemtoReg, RegWrite, MemWrite, ALUOp, Imm, dir, mush, wr, ReadData2, ReadData1, ifid[20:16], ifid[9:5], ifid[4:0]}
//	
//	assign idexi[4:0] = ifid[4:0];
//	assign idexi[9:5] = ifid[9:5];
//	assign idexi[14:10] = ifid[20:16];
//	assign idexi[154:143] = {ALUSrc, MemtoReg, RegWrite, MemWrite, ALUOp, Imm, dir, mush, wr};
//	// 154, 		153, 		152, 		151, 		150-148, 147, 146, 145-144 143, 	142-79		78-15			14-10			9-5			4-0
//	// ALUSrc	MemtoReg	RegWrite MemWrite ALUOp, 	Imm, dir, mush, 	wr,	ReadData2, 	ReadData1	ifid[20:16] ifid[9:5] 	ifid[4:0]
//	
//	regfile rf (.ReadData1(idexi[78:15]), .ReadData2(idexi[142:79]), .WriteData(Dw), 
//				.ReadRegister1(ifid[9:5]), .ReadRegister2(Ab), .WriteRegister(memwbo[4:0]),
//				.RegWrite(RegWrite), .clk(clk));
//	// ID/EX
//	
//	logic [154:0] idexo;
//	DFF_VAR #(155) IDEX (.q(idexo), .d(idexi), .reset(reset), .clk(clk));
//	
//	// EX
//	logic [63:0] DAddr9, Imm12, val, Db2, result, Dw2, Dw3, multh;
//	
//	sign_extend #(9) se1(.in(ifid[20:12]), .out(DAddr9));
//	
//	sign_extend #(12) se2(.in(ifid[21:10]), .out(Imm12));
//	
//	mux2_1 m2(.i0(DAddr9), .i1(Imm12), .sel(idexo[147]), .out(val));
//	
//	mux2_1 m3(.i0(idexo[142:79]), .i1(val), .sel(idexo[154]), .out(Db2));
//	
//	alu alu1(.A(idexo[78:15]), .B(Db2), .cntrl(idexo[150:148]), .result(result), .negative(negative), 
//				.zero(zero), .overflow(overflow), .carry_out(carry_out));
//
//	shifter shft (.value(idexo[78:15]), .direction(idexo[146]), .distance(ifid[15:10]), .result(Dw2));
//	
//	mult mul (.A(idexo[78:15]), .B(idexo[142:79]), .doSigned(1'b1), .mult_low(Dw3), .mult_high(multh));
//	
//	module forwarding_unit (.rs1, rs2,            // Source registers from ID/EX stage
//    input logic [4:0] rd_MEM, rd_WB,      // Destination registers from MEM and WB stages
//    input logic RegWrite_MEM, RegWrite_WB, // RegWrite signals for MEM and WB stages
//    output logic [1:0] forwardA, forwardB  // Forwarding control signals
//);
//				
//	// EX/MEM
//	logic [265:0] exmemo;
//	logic [265:0] exmemi;
//	assign exmemi = {idexo[150], idexo[149], idexo[148], idexo[144], Dw3, Dw2, result, idexo[142:79], idexo[4:0]};
//	// 265, 		264, 		263, 		262-261 	260-197	196-133	132-69	68-5				4-0
//	// MemtoReg	RegWrite MemWrite mush, 	Dw3, 		Dw2		ressult 	idexo[142:79] 	idexo[4:0]
//	//                                                       from alu
//	
//	DFF_VAR #(266) EXMEM (.q(exmemo), .d(exmemi), .reset(reset), .clk(clk));
//	
//	// MEM
//	logic [63:0] Dout, Dw1;
//	datamem dm1 (.address(exmemo[132:69]), .write_enable(exmemo[263]), .read_enable(~exmemo[263]), 
//					.write_data(exmemo[68:5]), .clk(clk), .xfer_size(4'h8), .read_data(Dout));
//					
//	mux2_1 m4(.i0(exmemo[132:69]), .i1(Dout), .sel(exmemo[265]), .out(Dw1));
//	
//	// MEM/WB
//	logic [199:0] memwbi;
//	//               199				198-197				196-133	132-69				68:5					4:0
//	assign memwbi = {exmemo[264], exmemo[262:261], 	Dw1, 		exmemo[260:197], 	exmemo[196:133], exmemo[4:0]};
//	// 				  RegWrite,    mush,        	  Output		Result from 	 	result from    	rd
//	//															  from		mult					 shifter
//	//															  mux 4
//	
//	// WB
//	DFF_VAR #(200) MEMWB (.q(memwbo), .d(memwbi), .reset(reset), .clk(clk));
//	
//	// WB/ID
//	
//	logic [3:0][63:0] in;
//	assign in[0] = memwbo[196:133];
//	assign in[1] = memwbo[68:5];
//	assign in[2] = memwbo[132:69];
//	
//	mux4_1 m5 (.i(in), .sel(memwbo[198:197]), .out(Dw));
//	
//endmodule

module pipelineCPU_testbench();

	logic clock;
	logic reset;
	
	pipelineCPU dut(.clk(clock), .reset(reset));
	
	initial begin
			clock <= 0;
			forever #(5000) clock <= ~clock; // every 50 time stamps we flip the clk (1/0)
					
		end //initial
		
		integer i;
		initial begin
			reset = 1; @(posedge clock);
			reset = 0; @(posedge clock);
			for (i = 0; i < 1000; i++) begin
				@(posedge clock);
			end
		$stop;
		end
endmodule 
