module forwardunit (rd_idex, rd_exmem, rd_memwb, rn, rm, rw_idex, 
							rw_exmem, rw_memwb, ALUSrc, forwardA, forwardB);

	input logic [4:0] rd_idex, rd_exmem, rd_memwb, rn, rm;
	input logic rw_idex, rw_exmem, rw_memwb, ALUSrc;
	output logic [1:0] forwardA, forwardB;
	
	always_comb begin
		if ((rn != 31) && (rd_idex == rn) && rw_idex)
			forwardA = 1;
		else if ((rn != 31) && (rd_exmem == rn) && rw_exmem)
			forwardA = 2;
		else if ((rn != 31) && (rd_memwb == rn) && rw_memwb)
			forwardA = 3;
		else
			forwardA = 0;
	end
	
	always_comb begin
		if ((rd_idex == rm) && (ALUSrc == 0) && rw_idex) //  && rw_idex == 1
			forwardB = 1;
		else if ((rd_exmem == rm) && (ALUSrc == 0) && rw_exmem) //  && rw_exmem == 1
			forwardB = 2;
		else if ((rd_memwb == rm) && (ALUSrc == 0) && rw_memwb)
			forwardB = 3;
		else
			forwardB = 0;
	end
	
endmodule


//module forwarding_unit (
//    input logic [4:0] rs1, rs2,            // Source registers from ID/EX stage
//    input logic [4:0] rd_MEM, rd_WB,      // Destination registers from MEM and WB stages
//    input logic RegWrite_MEM, RegWrite_WB, // RegWrite signals for MEM and WB stages
//    output logic [1:0] forwardA, forwardB  // Forwarding control signals
//);
//
//    always_comb begin
//        // Forwarding for Source Register 1 (rs1)
//        if (RegWrite_MEM && (rd_MEM != 0) && (rd_MEM == rs1))
//            forwardA = 2'b01; // Forward from MEM stage
//        else if (RegWrite_WB && (rd_WB != 0) && (rd_WB == rs1))
//            forwardA = 2'b10; // Forward from WB stage
//        else
//            forwardA = 2'b00; // No forwarding
//
//        // Forwarding for Source Register 2 (rs2)
//        if (RegWrite_MEM && (rd_MEM != 0) && (rd_MEM == rs2))
//            forwardB = 2'b01; // Forward from MEM stage
//        else if (RegWrite_WB && (rd_WB != 0) && (rd_WB == rs2))
//            forwardB = 2'b10; // Forward from WB stage
//        else
//            forwardB = 2'b00; // No forwarding
//    end
//
//endmodule

//module forwarding_unit_testbench();
//	
//	logic [4:0] rs1, rs2,            // Source registers from ID/EX stage
//   logic [4:0] rd_MEM, rd_WB,      // Destination registers from MEM and WB stages
//   logic RegWrite_MEM, RegWrite_WB, // RegWrite signals for MEM and WB stages
//   logic [1:0] forwardA, forwardB  // Forwarding control signals
//	
//	forwarding_unit dut (.rs1, .rs2,            // Source registers from ID/EX stage
//								.rd_MEM, .rd_WB,      // Destination registers from MEM and WB stages
//								.RegWrite_MEM, .RegWrite_WB, // RegWrite signals for MEM and WB stages
//								.forwardA, .forwardB);
//	initial begin
//		
//endmodule
