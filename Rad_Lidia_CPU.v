module processor( input         clk, reset,
                  output [31:0] PC,
                  input  [31:0] instruction,
                  output        WE,
                  output [31:0] address_to_mem,
                  output [31:0] data_to_mem,
                  input  [31:0] data_from_mem
                );	
	
	wire [31:0] ALUout,rd2,PChere;
	assign PC = PChere;
	assign address_to_mem = ALUout;
	assign data_to_mem = rd2;
	
	wire PCSrcJal,PCSrcJr,PCSrcJ,RegWrite,MemToReg,ALUSrc,RegDst,Branch;
	wire [1:0] ALUOp;
	wire [31:0] rd1;
	wire [31:0] signImm;
	wire [3:0] ALUControl;
	wire [4:0] A3,WriteReg;
	wire [31:0] SrcB;
	wire zero;
	wire [31:0] PCPlus4;
	reg [31:0] PCnext;
	wire [31:0] WD3,results;
	wire [31:0] signImmx4;
	wire [31:0] PCBranch,PCJal,PCJr,PCj;
	wire PCSrcBeq;
	
	main_decoder control1(instruction[31:26],ALUOp,PCSrcJal,PCSrcJr,PCSrcJ,RegWrite,MemToReg,WE,ALUSrc,RegDst,Branch);
	ALUOpDecoder control2(ALUOp,instruction[5:0],instruction[10:6],ALUControl);	
	
	mux_2_1b mux1(instruction[20:16],instruction[15:11],RegDst,WriteReg);
	
	register_file rf(clk,RegWrite,A3,WD3,instruction[25:21],rd1,instruction[20:16],rd2);
	
	sign_ext sign(instruction[15:0],signImm);
	mux_2_1 mux2(rd2,signImm,ALUSrc,SrcB);
	
	alu alucomp(rd1,SrcB,ALUControl,ALUout,zero);
	
	mux_2_1 mux3(ALUout,data_from_mem,MemToReg,results);
	assign PCPlus4 = PChere + 4;
	mux_2_1b mux4(WriteReg,5'd31,PCSrcJal,A3);
	mux_2_1 mux5(results,PCPlus4,PCSrcJal,WD3);

	shift_left_2 sll(signImm,signImmx4);
	assign PCBranch = PCPlus4 + signImmx4;
	assign PCSrcBeq = Branch & zero;
	
	assign PCJal = {PCPlus4[31:28],instruction[25:0],2'b00};
	assign PCJr = rd1;
	assign PCj = {PCPlus4[31:28],instruction[25:0],2'b00};
	always@(*)
	begin
	if(PCSrcBeq)
		PCnext = PCBranch;
		else if(PCSrcJal)
			PCnext = PCJal;
			else if(PCSrcJr)
				PCnext = PCJr;
				else if(PCSrcJ)
					PCnext = PCj;
					else
						PCnext = PCPlus4;
	end
	register regcom(clk,reset,PCnext,PChere); 
	
	
	
    //... write your code here ...
endmodule

//... add new Verilog modules here ...
module main_decoder( 
		input[5:0] opcode, 
		output reg[1:0] ALUOp,
        output reg PCSrcJal,
		output reg PCSrcJr,
		output reg PCSrcJ,
		output reg RegWrite,
		output reg MemToReg,
		output reg MemWrite,
		output reg ALUSrc,
		output reg RegDst,
		output reg Branch                      
   );  
	always @(*)  
	begin  
		case(opcode)   
		6'b000000: begin //R type  
            RegWrite = 1'b1;  
            RegDst = 1'b1;  
            ALUSrc = 1'b0;  
            ALUOp = 2'b10; 
			Branch = 1'b0;  
			MemWrite = 1'b0;  
			MemToReg = 1'b0; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b0;
			PCSrcJ = 1'b0;
			end  
		6'b100011: begin //lw  
            RegWrite = 1'b1;  
            RegDst = 1'b0;  
            ALUSrc = 1'b1;  
            ALUOp = 2'b00; 
			Branch = 1'b0;  
			MemWrite = 1'b0;  
			MemToReg = 1'b1; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b0;
			PCSrcJ = 1'b0;
            end  
        6'b101011: begin //sw  
            RegWrite = 1'b0;  
            RegDst = 1'b?;  
            ALUSrc = 1'b1;  
            ALUOp = 2'b00; 
			Branch = 1'b0;  
			MemWrite = 1'b1;  
			MemToReg = 1'b?; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b0;
			PCSrcJ = 1'b0;			
			end
        6'b000100: begin //beq 
            RegWrite = 1'b0;  
            RegDst = 1'b?;  
            ALUSrc = 1'b0;  
            ALUOp = 2'b01; 
			Branch = 1'b1;  
			MemWrite = 1'b0;  
			MemToReg = 1'b?; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b0;
			PCSrcJ = 1'b0;
			end 
        6'b001000: begin //addi
            RegWrite = 1'b1;  
            RegDst = 1'b0;  
            ALUSrc = 1'b1;  
            ALUOp = 2'b00; 
			Branch = 1'b0;  
			MemWrite = 1'b0;  
			MemToReg = 1'b0; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b0;
			PCSrcJ = 1'b0;
			end 
        6'b000011: begin //jal  
            RegWrite = 1'b1;  
            RegDst = 1'b?;  
            ALUSrc = 1'b?;  
            ALUOp = 2'b??; 
			Branch = 1'b?;  
			MemWrite = 1'b0;  
			MemToReg = 1'b?; 
			PCSrcJal = 1'b1;  
			PCSrcJr = 1'b0;
			PCSrcJ = 1'b0;
			end  
        6'b000111: begin //jr  
            RegWrite = 1'b0;  
            RegDst = 1'b?;  
            ALUSrc = 1'b?;  
            ALUOp = 2'b??; 
			Branch = 1'b?;  
			MemWrite = 1'b0;  
			MemToReg = 1'b?; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b1;
			PCSrcJ = 1'b0;			
			end    
		6'b000010: begin //j 
            RegWrite = 1'b0;  
            RegDst = 1'b?;  
            ALUSrc = 1'b?;  
            ALUOp = 2'b??; 
			Branch = 1'b?;  
			MemWrite = 1'b0;  
			MemToReg = 1'b?; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b0;
			PCSrcJ = 1'b1;
			end    
        6'b011111: begin //addu  
            RegWrite = 1'b1;  
            RegDst = 1'b1;  
            ALUSrc = 1'b0;  
            ALUOp = 2'b11; 
			Branch = 1'b0;  
			MemWrite = 1'b0;  
			MemToReg = 1'b0; 
			PCSrcJal = 1'b0;  
			PCSrcJr = 1'b0; 
			PCSrcJ = 1'b0;
			end   
		endcase  
    end  
 endmodule   

 module ALUOpDecoder( 
		input[1:0] ALUOp,
		input[5:0] funct,
        input[4:0] shamt,
		output reg [3:0] ALUControl  
		);
	
	wire [12:0] ALUControlIn;
	assign ALUControlIn = {ALUOp,funct,shamt};  
	always @(ALUControlIn)  
	casex (ALUControlIn)  
		13'b00xxxxxxxxxxx: ALUControl = 4'b0010; //add
		13'b01xxxxxxxxxxx: ALUControl = 4'b0110; //sub
		13'b10100000xxxxx: ALUControl = 4'b0010; //add
		13'b10100010xxxxx: ALUControl = 4'b0110; //sub
		13'b10100100xxxxx: ALUControl = 4'b0000; //and
		13'b10100101xxxxx: ALUControl = 4'b0001; //or 
		13'b10101010xxxxx: ALUControl = 4'b0111; //slt
		13'b1101000000000: ALUControl = 4'b1000; //addu
		13'b1101000000100: ALUControl = 4'b1001; //addu_s
		13'b10000100xxxxx: ALUControl = 4'b0011; //sllv
		13'b10000110xxxxx: ALUControl = 4'b0100; //srlv
		13'b10000111xxxxx: ALUControl = 4'b0101; //srav
		default: ALUControl = 4'b0000;  
	endcase  
 endmodule  

module sign_ext(input [15:0] unextend,
                output [31:0] extended);				
assign extended = {{16{unextend[15]}}, unextend};
endmodule

module alu(input signed [31:0] SrcA,SrcB,  // ALU 8-bit Inputs                 
           input [3:0] ALUControl,// ALU Selection
           output signed [31:0] ALUResult, // ALU 8-bit Output
           output Zero // Carry Out Flag
    );
    reg signed [31:0] ALU_out;
    reg z;
    assign ALUResult = ALU_out; // ALU out
    assign Zero = z; // Zero flag
	
	wire [7:0] res1, res2, res3, res4;
	satsum sat1(SrcA[31:24], SrcB[31:24], res1);
	satsum sat2(SrcA[23:16], SrcB[23:16], res2);
	satsum sat3(SrcA[15:8], SrcB[15:8], res3);
	satsum sat4(SrcA[7:0], SrcB[7:0], res4);
    always @(*)
    begin
        case(ALUControl)
			4'b0010: // Addition
				ALU_out = SrcA + SrcB ; 
			4'b0110: // Subtraction
				ALU_out = SrcA - SrcB ;
			4'b0000: //  Logical and 
				ALU_out = SrcA & SrcB;
			4'b0001: //  Logical or
				ALU_out = SrcA | SrcB;
			4'b0011: //  sllv 
				ALU_out = SrcA << SrcB;
			4'b0100: //  srlv 
				ALU_out = SrcA >> SrcB;
			4'b0101: //  srav 
				ALU_out = SrcA >>> SrcB;
			4'b0111: // stl(Set Less Then)
				ALU_out = SrcA < SrcB?1:0;
			4'b1000: // addu.qb - four unsigned sums 
				begin
					ALU_out[31:24] = SrcA[31:24] + SrcB[31:24];
					ALU_out[23:16] = SrcA[23:16] + SrcB[23:16];
					ALU_out[15:8] = SrcA[15:8] + SrcB[15:8];
					ALU_out[7:0] = SrcA[7:0] + SrcB[7:0];
				end
			4'b1001: // addu_s.qb - saturated four unsigned sums 
				begin 
					ALU_out[31:24] = res1;
					ALU_out[23:16] = res2;
					ALU_out[15:8] = res3;
					ALU_out[7:0] = res4;
				end
			default:
                ALU_out = 0;// beq
        endcase
		
		case (ALU_out)
            32'b0:
				z = 1;
            default:
                z = 0;
        endcase;
    end
	
endmodule

//if a+b>255, then a+b is 255
module satsum(input [7:0] a, b,
			  output reg [7:0] result);
	wire [8:0] tmp;
	assign tmp = {1'b0,a} + {1'b0,b};
	always @(*)
	begin
	if (tmp > 255)
		result = 255;
	else
		result = tmp;
	end
endmodule
	
module register_file(
	input clk,  
    // write port  
    input WE3,         //reg_write_en
    input [4:0] A3,    //reg_write_dest,
    input [31:0] WD3,  //reg_write_data,  
    //read port 1 
    input [4:0] A1,    //reg_read_addr_1  
    output [31:0] RD1, //reg_read_data_1,  
    //read port 2  
    input [4:0] A2,    //reg_read_addr_2,  
    output [31:0] RD2  //reg_read_data_2  
 );  
    reg [31:0] reg_array [63:0];  
    // write port  
    always @(posedge clk) 
		begin  	 
			if(WE3) begin  
				reg_array[A3] <= WD3;  
			end    
		end 
		
    assign RD1 = ( A1 == 0)? 31'b0 : reg_array[A1];  
    assign RD2 = ( A2 == 0)? 31'b0 : reg_array[A2];
	
endmodule  
 
module register(input clk, reset,
			    input [31:0] data_in,
			    output reg [31:0] data_out);
	
	always @(posedge clk) begin
		if(reset == 1'b0)
			data_out <= data_in;
		else
			data_out <= 00000000000000000000000000000000;
	end
endmodule

module mux_2_1(input [31:0] d0, d1,
	       input select,
	       output reg [31:0] y);
   always @(*)
	if(select) 
	   y = d1;
	else 
	   y = d0;
endmodule

module mux_2_1b(input [4:0] d0, d1,
	       input select,
	       output reg [4:0] y);
   always @(*)
	if(select) 
	   y = d1;
	else 
	   y = d0;
endmodule

module mux_4_1(input [31:0] d0, d1, d2, d3,
	       input [1:0] select,
	       output [31:0] y);
 wire [31:0] w1, w2;
 
 mux_2_1 mux1(d0,d1,select[0],w1); 
 mux_2_1 mux2(d2,d3,select[0],w2); 
 mux_2_1 mux3(w1,w2,select[1],y);

endmodule

module shift_left_2(input [31:0] x,
			   output reg [31:0] y);
   always @(*)
	
	y = x << 2;
endmodule

module top(input         clk, reset,
		   output [31:0] data_to_mem, address_to_mem,
		   output        write_enable);

	wire [31:0] pc, instruction, data_from_mem;

	inst_mem  imem(pc[7:2], instruction);
	data_mem  dmem(clk, write_enable, address_to_mem, data_to_mem, data_from_mem);
	processor CPU(clk, reset, pc, instruction, write_enable, address_to_mem, data_to_mem, data_from_mem);
endmodule

//-------------------------------------------------------------------
module data_mem (input clk, we,
		 input  [31:0] address, wd,
		 output [31:0] rd);

	reg [31:0] RAM[63:0];

	initial begin
		$readmemh ("memfile_data.hex",RAM,0,63);
	end

	assign rd=RAM[address[31:2]]; // word aligned

	always @ (posedge clk)
		if (we)
			RAM[address[31:2]]<=wd;
endmodule

//-------------------------------------------------------------------
module inst_mem (input  [5:0]  address,
		 output [31:0] rd);

	reg [31:0] RAM[63:0];
	initial begin
		$readmemh ("memfile_inst.hex",RAM,0,63);
	end
	assign rd=RAM[address]; // word aligned
endmodule
