`timescale 1ns/10ps

module bitSlice(Ai, Bi, s, cin, cout, out);
    input logic Ai, Bi, cin;
    input logic [2:0] s;
    output logic cout, out;
    
    logic [7:0] i;
    logic sum;


    assign i[0] = Bi;
    

    and #(50ps) a1(i[4], Ai, Bi);
    or #(50ps) o1(i[5], Ai, Bi);
    xor #(50ps) x1(i[6], Ai, Bi);

    fullAdder f1(.A(Ai), .B(Bi), .cin(cin), .s0(s[0]), .sum(sum), .cout(cout));


    assign i[2] = sum;  
    assign i[3] = sum;  


    mux8_1_alu mux2 (.i(i), .sel(s), .out(out));

endmodule

module bitSlice_testbench();
	logic Ai, Bi, cin;
	logic [2:0] s;
	logic cout, out;
	
	bitSlice dut(.Ai, .Bi, .s, .cin, .cout, .out);
	integer i;
	initial begin
		for (i=0; i<2**4; i++) begin
			{Ai, Bi, cin, s} = i; #10;
		end
	end
endmodule