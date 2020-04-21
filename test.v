`timescale 1ns / 1ps

////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:
//
// Create Date:   12:30:53 03/10/2020
// Design Name:   fetch
// Module Name:   /home/jayantduneja10/Desktop/ProcessorFinal/test.v
// Project Name:  ProcessorFinal
// Target Device:  
// Tool versions:  
// Description: 
//
// Verilog Test Fixture created by ISE for module: fetch
//
// Dependencies:
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
////////////////////////////////////////////////////////////////////////////////

module test;

	// Inputs
	reg clk;

	// Outputs
	wire [31:0] reg_output;
	wire [31:0] mem_output;
	wire [31:0] instruction;
	
	// Instantiate the Unit Under Test (UUT)
	fetch uut (
		.clk(clk), 
		.reg_output(reg_output), 
		.mem_output(mem_output), 
		.instruction(instruction), 
	
	);
	always #5 clk=~clk;

	initial begin
		// Initialize Inputs
		

		// Wait 100 ns for global reset to finish
		#100;
      clk=1;
		// Add stimulus here

	end
      
endmodule

