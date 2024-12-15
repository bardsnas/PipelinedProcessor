module dataForward(rd_exmemo, rd_memwbo, forwardD);
	input logic [4:0] rd_exmemo, rd_memwbo;
	output logic forwardD;
	
	always_comb begin
		if (rd_exmemo == rd_memwbo)
			forwardD = 1;
		else
			forwardD = 0;
	end
endmodule