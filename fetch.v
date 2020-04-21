`timescale 1ns / 1ps

//ALU OPERATION
/////////////////////////////////////////
module alu(A,B,shift,ALU_op,C,zero,ovfl);
output reg zero;
output reg ovfl; 
input [31:0] A; 
input [4:0] shift;
output reg [31:0] C;
reg signed [31:0] A1,B1;
input [31:0] B;
input [3:0] ALU_op;
always@(*)
begin
 ovfl = 1'b0;
 A1=A;
 B1=B;
 zero = 1'b0; 
case(ALU_op)
4'b0000 : {ovfl,C} = A1+ B1;//ADD,ADDI
4'b0100 : C = A^B;//XOR
4'b1010 : C = A>B;//BGTZ
4'b1110 : C = A >> B;//SRLV
4'b1111 : C = A < B;//SLTIU,SLTU 
4'b1000 : C = A<<shift;//SLL 

4'b0001 :
				begin
				C=A1-B1;
				case(C)
					32'd0 : zero = 1'b1;
					default : zero = 1'b0;					//SUB
				endcase
				end

4'b1011 : C = A<=B;//BLEZ
4'b0010 : C = A&B;//AND
4'b0011 : C = A|B;//OR 
4'b1100 : C = (A!==B);//BNE  
4'b0111 : C = A1 >> B1;//SRLV,SRA,SRAV
4'b0110 : C = ~(A|B);//NOR
4'b0101 : C = A1 << B1;//SLLV 
4'b1001 : C = A>>shift;//SRL
4'b1101 : C = A1 < B1;//SLT,SLTI 
			
default : C= 32'bz;
endcase
end
endmodule


//DECODER MODULE
///////////////////////////////////////

module dec(instruction,branch,jump,rmem_wren,dmem_wren,mem_to_reg,ALU_op,rs,rt,rd,immediate,
							alu_src,regdst,shamt,beq_off,address,shift_ctrl,sign_ctrl,save_PC);
	input [31:0] instruction;
	output reg [31:0] immediate;
	output reg branch;
	output reg [25:0] address;
   output reg jump; 
	output reg [4:0] rs;
	output reg dmem_wren;
   output reg rmem_wren;
	output reg shift_ctrl;
	output reg sign_ctrl;
	output reg save_PC;
	output reg [3:0] ALU_op;
   output reg [4:0] shamt;
   output reg mem_to_reg;
	output reg alu_src;
	output reg regdst; 
	output reg [4:0] rt;
	output reg [4:0] rd;
	output reg [15:0] beq_off;
	
	always @(instruction)
		begin
			dmem_wren=0;
			rmem_wren=0;
			mem_to_reg=1'bz;
			alu_src=1'bz;
			immediate=32'd0;
			branch=0;
			ALU_op=4'd15;
			regdst=1'bz;
			rs=5'd0;
			rt=5'd0;
			rd=5'd0;
			shift_ctrl = 1'b0;
			sign_ctrl =1'b0;
			save_PC = 1'b0;
			beq_off=16'd0;
			address=26'd0;
			jump=1'b0;
			shamt=5'dz;
			
			
		if(instruction[31:26] == 6'b000000)//R-TYPE
			begin
				alu_src=0;
				dmem_wren=0;
				mem_to_reg=0;
				rmem_wren=1;
				regdst=1; 
				rt=instruction[20:16]; 
				if(instruction[5:0] !== 6'b000011 || instruction[5:0] !== 6'b000011 || instruction[5:0] !== 6'b000010)
				begin
					rs=instruction[25:21];
				end
				case(instruction[5:0]) 
				
					6'b100000:			//ADD
					begin
						ALU_op=4'b0000;
					end
						
					6'b000011:			//SRA:SHIFT RIGHT ARITHEMETIC
					begin
						ALU_op=4'b0111;
						shamt=instruction[10:6];
						shift_ctrl=1'b1;
						rs=5'd0;
					end
					
					6'b100010:			//SUB
					begin
						ALU_op=4'b0001;
					end
					
					
				
				
				
					6'b100101:			//OR
					begin
						ALU_op=4'b0011;
					end 
					
					
					6'b000010:			//SRL:SHIFT RIGHT LOGICAL
					begin
						ALU_op=4'b1001;
						shamt=instruction[10:6]; 
						rs=5'd0;
						shift_ctrl=1'b1;
					end



					6'b100100:		//AND
					begin
						ALU_op=4'b0010;
					end 
					
					
					
					6'b100111:		//NOR
					begin
						ALU_op=0110;
					end
				
						
					6'b101010:			//SLT:STRICLTY LESS THAN
					begin
						ALU_op=4'b1101;
						sign_ctrl=1'b1;
					end	
					
					6'b100111:		//XOR
					begin
						ALU_op=4'b0100;
						
					end
					
					
					
					6'b000100:		//SLLV:SHIFT LEFT LOGICAL VARIABLE
					begin
						ALU_op=4'b0101;
					end
					
					6'b000000:		//SLL:SHIFT LEFT LOGICAL VARIABLE
					begin
						ALU_op=4'b1000;
						rs=5'd0;
						shift_ctrl=1'b1;
					end
					
					6'b000111:		//SRAV:SHIFT RIGHT LOGICAL VARIABLE
					begin
						ALU_op=4'b0111;
					end
					
					
					6'b000110:		//SRLV:SHIFT RIGHT LOGICAL VARIABLE
					begin
						ALU_op=4'b1000;			
					end
					
					6'b101011:		//SLTU:SLTRICTLY LESSS THAN UNSIGNED
					begin
						ALU_op=4'b0111;      
					end
					
					endcase
					rd=instruction[15:11];
				end
				
			case(instruction[31:26])
			   6'b100011:								//LW
					begin
						regdst=0;
						rt=instruction[20:16];
						alu_src=1;
						ALU_op=4'b0000;
						rs=instruction[25:21];
						immediate={{16{instruction[15]}},instruction[15:0]};
						rmem_wren=1;
						mem_to_reg=1;
						dmem_wren=0;
						
					end
				6'b101011:								//SW
					begin
						regdst=1'bz;
						immediate={{16{instruction[15]}},instruction[15:0]};
						rt=instruction[20:16];
						alu_src=1;
						ALU_op=4'b0000;
						rs=instruction[25:21];
						dmem_wren=1;
						rmem_wren=0;
						mem_to_reg=1'bz;
					end
				6'b001000:								//ADDI
					begin
						regdst=0;
						ALU_op=4'b0000;
						rs=instruction[25:21];
						rt=instruction[20:16];
						alu_src=1;
						dmem_wren=0;
						rmem_wren=1;
						mem_to_reg=0;
						immediate={{16{instruction[15]}},instruction[15:0]};
						
					end
				
				6'b001101:								//ORI
					begin
						regdst=0;
						ALU_op=4'b0011;
						rs=instruction[25:21];
						immediate={{16{0}},instruction[15:0]};
						rt=instruction[20:16];
						alu_src=1;
						dmem_wren=0;
						rmem_wren=1;
						mem_to_reg=0;
					end
				
				6'b001100:								//ANDI
					begin
						ALU_op=4'b0010;
						alu_src=1;
						mem_to_reg=0;
						rs=instruction[25:21];
						regdst=0;
						dmem_wren=0;
						immediate={{16{0}},instruction[15:0]};
						rmem_wren=1;
						rt=instruction[20:16];
					end
					
				6'b000111:								//BGTZ:BRANCH IF GREATER THAN ZERO
					begin
						
						branch=1;
						beq_off[15:0]=instruction[15:0];
						rt=instruction[20:16];
						ALU_op=4'b1010; 
						rs=instruction[25:21]; 
						alu_src=1'b1;
					end
					
				
			
				6'b001010:								//SLTI
					begin
						regdst=0;
						ALU_op=4'b1101;
						rs=instruction[25:21];
						immediate={{16{instruction[15]}},instruction[15:0]};
						rt=instruction[20:16];
						alu_src=1;
						dmem_wren=0;
						rmem_wren=1;
						mem_to_reg=0; 
						sign_ctrl=1'b1;
					end 
				6'b001101:								//XORI
					begin
						regdst=1'b0;
						ALU_op=4'b0100;
						rs=instruction[25:21];
						immediate={{16{1'b0}},instruction[15:0]};
						rt=instruction[20:16];
						alu_src=1'b1;
						dmem_wren=1'b0;
						rmem_wren=1'b1;
						mem_to_reg=1'b0;
					end
					
				
				6'b000101:								//BNE:BRANCH NOT EQUAL
					begin
						rs=instruction[25:21];
						rt=instruction[20:16];
						branch=1'b1;
						beq_off[15:0]=instruction[15:0];
						ALU_op=4'b1100; 
						alu_src=1'b0;
					end
				
				6'b000011:        				  //JAL:JUMP & LOOP
					begin
						save_PC=1'b1;
						jump=1'b1;
						address[25:0]=instruction[25:0];
					end
					
				
				6'b001010:							//SLTIU
					begin
						regdst=1'b0;
						ALU_op=4'b1111;
						rs=instruction[25:21];
						immediate={{16{1'b0}},instruction[15:0]};
						rt=instruction[20:16];
						alu_src=1'b1;
						dmem_wren=1'b0;
						rmem_wren=1'b1;
						mem_to_reg=1'b0;
					end 
					
				
				6'b000110:				//BLEZ:BRANCH IF LESS THAN OR EQUAL TO ZERO
					begin
						rs=instruction[25:21];
						beq_off[15:0]=instruction[15:0];
						ALU_op=4'b1011;
						rt=instruction[20:16];
						branch=1'b1; 
						alu_src=1'b0;
						
					end
						
				
			6'b000010:					//JUMP
					begin
						jump=1'b1;
						address[25:0] = instruction[25:0];
					end	
				
				
			6'b000100:					//BEQ:BRANCH IF EQUAL
					begin
						rs=instruction[25:21];
						beq_off[15:0]=instruction[15:0]; 
						branch=1; 
						rt=instruction[20:16]; 
						ALU_op=4'b0001; 
						alu_src=1'b0;
					end 
			endcase
				end
endmodule 

////////////////////////////////////////////////////////////////////
//FETCH/EXECUTE MODULE 
////////////////////////

module fetch(clk,reg_output,mem_output,instruction);
input clk;
output reg [31:0] reg_output; 
output reg [31:0] mem_output;  

wire alu_src;
wire regdst;
wire [4:0]shift;
wire overflow;
reg [31:0] pipeline;
reg [31:0] pc;
output [31:0] instruction;  
reg [7:0] instruction_memory[0:127]; 
reg [31:0] data_memory[0:31];
reg [29:0] pc_update_1;
reg [31:0] reg_memory[0:31]; 
wire jump;
wire [29:0] pc_new; 
reg [31:0] RM;
wire [31:0] RC;
reg [29:0] pc_update_2;
wire [31:0] Temp_Ra;
wire [31:0] Temp_Rb;
wire [31:0]immediate;
wire mem_regwe;
wire data_memwe;
wire data_to_reg;
wire [3:0]ALU_op;
wire [4:0] RS;
wire [4:0] RT;
wire [4:0] RD;
wire [31:0] RX;
reg [31:0] RA,RB;
reg [4:0] RP; 
wire shift_ctrl;
wire sign_ctrl;
wire save_PC;

 

initial begin 														//INSTR1:CONTAINS PROGRAM TO ADD FIRST 10 NATURAL NUMBERS
$readmemb ("instr1.mem" ,  instruction_memory) ;      //ADD R2 R2 R0 
																		//ADD R2 R2 R1
data_memory[4] = 40;												//ADDI R1 R1 -1
data_memory[3] = 30;												//BEQ R1 R0 LABEL
data_memory[1] = 10;												//NO-OP
data_memory[2] = 20;												//JUMP TO INSTRUCTION 1
data_memory[0] = 0;												// NO-OP
reg_memory[5] = 5;												// SW R2 
																		// INSTR1 CONTAINS THE FOLLOWING INSTRUCTIONS
reg_memory[4] = 4;												//INSTR CONTAINS BASIC ARITHIMETIC INSTRUCTIONS AND SW INSTRUCTION
reg_memory[3] = 3;
reg_memory[2] = 0;
reg_memory[1] = 10;
reg_memory[0] = 0;


pipeline = 32'bx;
pc = 32'd0;

end

//FETCH MODULE

wire [25:0] target_addr;
wire branch;
wire zero; 
wire [15:0] beq_offset;
wire [29:0] pc_jump;
assign pc_jump = {pc[31:28],target_addr[25:0]};
assign pc_new = pc[31:2] + 1; 


always@(pc_jump,pc_update_1,jump)    
begin 
if(jump == 1'b1)                 // pc_update_2 will give us the final value of pc 
  
  pc_update_2 = pc_jump;			//after passing through the mux which has jump control

else										// signal as the select line
	
	pc_update_2 = pc_update_1;

end 


always@(pc_new,branch,zero) 
begin 												//pc_update_1 contains the value of pc after 
if(branch == 1'b1 & zero == 1'b1) 			//passing through the branch control logic

	pc_update_1 = pc_new + {{14{beq_offset[15]}},beq_offset[15:0]} - 30'd1; 
 
else 

	pc_update_1 = pc_new;

end 





always@(negedge clk) 
begin 
pipeline = {instruction_memory[pc+3],instruction_memory[pc+2],instruction_memory[pc+1],instruction_memory[pc]};
pc[31:0] = {pc_update_2,{2{1'b0}}};
end	 

assign instruction = pipeline;  


//EXECUTE UNIT 



dec D(instruction,branch,jump,mem_regwe,data_memwe,data_to_reg,ALU_op,RS,RT,RD,immediate,alu_src,regdst,shift,beq_offset,target_addr,shift_ctrl,sign_ctrl,save_PC); 
assign Temp_Rb= reg_memory[RT]; 
assign Temp_Ra= reg_memory[RS];


always@(Temp_Ra or shift or shift_ctrl)
begin
	if(shift_ctrl==1'b1)
		begin
			RA[4:0] = shift;
			RA[31:5] = 27'd0;
			
		end
	else
		
			RA = Temp_Ra;
		
end 

always@( Temp_Rb or immediate or alu_src) 
begin 
	RB=32'bz;
	if(alu_src == 1'b1) 
		 
			RB=immediate;
		 
	else if(alu_src == 1'b0) 
		
			RB=Temp_Rb;
		 
end  



alu A1(RA,RB,shift,ALU_op,RC,zero,overflow); 
assign RX= data_memory[RC];

	
always@(regdst or RT or RD) 
begin 
	RP=32'bz;
	if( regdst == 1'b1) 
		 RP=RD;
	else if( regdst == 1'b0) 
		 RP=RT;
		 
end  

always@( data_to_reg or RX or RC) 
begin 
	RM=32'bz;
	if(data_to_reg == 1'b1) 
		 
			RM=RX;
		
	else if(data_to_reg == 1'b0) 
		 
			RM=RC;
		
end
 


always@(negedge clk) 
begin 
if(data_memwe == 1'b1) 
	begin 
		data_memory[RC]=Temp_Rb;  
		mem_output=Temp_Rb;
	end  
end

always@(negedge clk) 
begin 
	   if((jump == 1'b1) & (save_PC == 1'b1)) 
		begin
			reg_memory[31] = pc+8;
			reg_output = pc+8;
		end 
		else if((mem_regwe == 1'b1) &((overflow == 1'b0) | alu_src == 1'b1)) 
		begin 
			reg_memory[RP]=RM;
			reg_output =RM;
		end 
end
endmodule 
	


