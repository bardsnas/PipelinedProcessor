`timescale 1ns/10ps

module PC_control(instruction, negative, zero, overflow, Brtaken, UncondBr, Reg2Loc);

	input logic [31:0] instruction;
	input logic negative, zero, overflow;
	output logic Brtaken, UncondBr, Reg2Loc;
	
	logic Br, C, Reg2Loc1, Brtaken1, Reg2Loc2, Brtaken2, UncondBr1, UncondBr2;  
	
	always_comb begin
		case (instruction[31:26])
			
			6'b100100: begin // ADDI
					Reg2Loc = 1'bx; 		
					Brtaken = 0;		UncondBr = 1'bx;
			end
			
			6'b101010: begin // ADDS
					Reg2Loc = 1;
					Brtaken = 0;		UncondBr = 1'bx;
			end
			
			6'b000101: begin // B
					Reg2Loc = 1'bx;
					Brtaken = 1;		UncondBr = 1;
			end
			
			6'b010101: begin	// B.cond
					Reg2Loc = 1'bx;
					Brtaken = Br;		UncondBr = 0;
			end
			
			6'b101101: begin // CBZ
					Reg2Loc = 0;
					Brtaken = zero;	UncondBr = 0;
			end
			6'b111110: begin	// LDUR/STUR
					Reg2Loc = Reg2Loc1;
					Brtaken = Brtaken1;		UncondBr = UncondBr1;
			end	
			6'b111010: begin	// SUBS
					Reg2Loc = 1;
					Brtaken = 0;		UncondBr = 1'bx;
			end
			6'b110100: begin // LSL/LSR
					Reg2Loc = Reg2Loc2;
					Brtaken = Brtaken2;		UncondBr = UncondBr2;
			end
			6'b100110: begin	// MUL
					Reg2Loc = 1'b1;
					Brtaken = 0;		UncondBr = 1'bx;
			end
			default: begin
					Reg2Loc = 1'bx;
					Brtaken = 1'b0;		UncondBr = 1'bx;
			end	
		endcase
	end
	
	always_comb begin
		case (instruction[25:21])
		
			5'b00010: begin //LDUR
					Reg2Loc1 = 1'bx;
					Brtaken1 = 0;		UncondBr1 = 1'bx;
			end
			
			5'b00000: begin //STUR
					Reg2Loc1 = 0;
					Brtaken1 = 0;		UncondBr1 = 1'bx;
			end
			
			default: begin
					Reg2Loc1 = 1'bx;
					Brtaken1 = 1'b0;		UncondBr1 = 1'bx;
			end
		endcase
	end
	
	always_comb begin
		case (instruction[25:21])
		
			5'b11011: begin // LSL
					Reg2Loc2 = 0;
					Brtaken2 = 0;		UncondBr2 = 1'bx;
			end
			
			5'b11010: begin // LSR
					Reg2Loc2 = 0;
					Brtaken2 = 0;		UncondBr2 = 1'bx;
			end
			
			default: begin
					Reg2Loc2 = 1'bx;
					Brtaken2 = 1'b0;		UncondBr2 = 1'bx;
			end
		endcase
	end
	
	always_comb begin // Brtaken
		case (instruction[4:0])
		
			5'd0: begin // EQ
					Br = (zero==1);
			end
			
			5'd1: begin // NE
					Br = (zero==0);
			end
			
			5'd2: begin // HS
					Br = (C==1);
			end
			
			5'd3: begin // LO
					Br = (C==0);
			end
			
			5'd4: begin // MI
					Br = (negative==1);
			end
			
			5'd5: begin // PL
					Br = (negative==0);
			end
			
			5'd6: begin // VS
					Br = (overflow==1);
			end
			
			5'd7: begin // VC
					Br = (overflow==0);
			end
			
			5'd8: begin // HI
					Br = (C==1 && zero==0);
			end
			
			5'd9: begin // LS
					Br = (C==0 || zero==1);
			end
			
			5'd10: begin // GE
					Br = (negative==overflow);
			end
			
			5'd11: begin // LT
					Br = (negative==1 && overflow==0);
			end
			
			5'd12: begin // GT
					Br = (zero==0 && negative==overflow);
			end
			
			5'd13: begin // LE
					Br = !(zero==0 && negative==overflow);
			end
			
			default: begin
					Br = 1'b0;
			end
		endcase
	end
endmodule