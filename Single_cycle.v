`timescale 1ns / 1ps


module D_ff(input clk, input reset, input regWrite,input d, output reg q);
	always@(negedge clk)
		begin
			if(reset)
				q=0;
			else
				if(regWrite == 1) begin q=d; end
		end
endmodule

module D_ff_I(input clk, input reset, input regWrite,input d, output reg q);
	always@(posedge clk)
		begin
			if(reset)
				q=0;
			else
				if(regWrite == 1) begin q=d; end
		end
endmodule

//Instruction Memory Code Starts
module D_ff_IM(input clk, input reset, input d, output reg q);
	always@(reset or posedge clk)
	if(reset)
		q=d;
endmodule

module Flag_reg(input clk,input reset,input flagWr, input pos, input neg, input zero,input carry, output [3:0] out);

D_ff df0(clk,reset,flagWr,neg,out[0]);
D_ff df1(clk,reset,flagWr,carry,out[1]);
D_ff df2(clk,reset,flagWr,zero,out[2]);
D_ff df3(clk,reset,flagWr,pos,out[3]);

endmodule

module register_IM(input clk, input reset, input [15:0] d_in, output [15:0] q_out);
	D_ff_IM dIM0 (clk, reset, d_in[0], q_out[0]);
	D_ff_IM dIM1 (clk, reset, d_in[1], q_out[1]);
	D_ff_IM dIM2 (clk, reset, d_in[2], q_out[2]);
	D_ff_IM dIM3 (clk, reset, d_in[3], q_out[3]);
	D_ff_IM dIM4 (clk, reset, d_in[4], q_out[4]);
	D_ff_IM dIM5 (clk, reset, d_in[5], q_out[5]);
	D_ff_IM dIM6 (clk, reset, d_in[6], q_out[6]);
	D_ff_IM dIM7 (clk, reset, d_in[7], q_out[7]);
	D_ff_IM dIM8 (clk, reset, d_in[8], q_out[8]);
	D_ff_IM dIM9 (clk, reset, d_in[9], q_out[9]);
	D_ff_IM dIM10 (clk, reset, d_in[10], q_out[10]);
	D_ff_IM dIM11 (clk, reset, d_in[11], q_out[11]);
	D_ff_IM dIM12 (clk, reset, d_in[12], q_out[12]);
	D_ff_IM dIM13 (clk, reset, d_in[13], q_out[13]);
	D_ff_IM dIM14 (clk, reset, d_in[14], q_out[14]);
	D_ff_IM dIM15 (clk, reset, d_in[15], q_out[15]);
endmodule

module mux16to1( input [15:0] outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,outR8,outR9,outR10,outR11,outR12,outR13,outR14,outR15, input [3:0] Sel, output reg [15:0] outBus );
	always@(outR0 or outR1 or outR2 or outR3 or outR4 or outR5 or outR6 or outR7 or outR8 or outR9 or outR10 or outR11 or outR12 or outR13 or outR14 or outR15 or Sel)
	case (Sel)
				4'b0000: outBus=outR0;
				4'b0001: outBus=outR1;
				4'b0010: outBus=outR2;
				4'b0011: outBus=outR3;
				4'b0100: outBus=outR4;
				4'b0101: outBus=outR5;
				4'b0110: outBus=outR6;
				4'b0111: outBus=outR7;
				4'b1000: outBus=outR8;
				4'b1001: outBus=outR9;
				4'b1010: outBus=outR10;
				4'b1011: outBus=outR11;
				4'b1100: outBus=outR12;
				4'b1101: outBus=outR13;
				4'b1110: outBus=outR14;
				4'b1111: outBus=outR15;
	endcase
endmodule

module IM(  input clk, input reset, input [4:0] pc_5bits, output [15:0] IR );
	wire [15:0] Qout0, Qout1, Qout2, Qout3, Qout4, Qout5, Qout6, Qout7,
					Qout8, Qout9, Qout10, Qout11, Qout12, Qout13, Qout14, Qout15;
	register_IM rIM0 (clk, reset, 16'b0001_001_010_000_000,Qout0); 		//sub $r0, $r1, $r2
	register_IM rIM1 (clk, reset, 16'b0011_000_110_000_110, Qout1); 	//addic $r6, $r0, 6
	register_IM rIM2 (clk, reset, 16'b0000_111_110_101_000, Qout2); 	//add $r5, $r7, $r6
	register_IM rIM3 (clk, reset, 16'b0010_000_110_111_000, Qout3); 	//addc $r7, $r6, $r0
	register_IM rIM4 (clk, reset, 16'b0001_110_111_001_000, Qout4); 	//sub $r1, $r6, $r7
	register_IM rIM5 (clk, reset, 16'b0100_001_010_000_001, Qout5); 	//shift $r2, $r1, 1
	register_IM rIM6 (clk, reset, 16'b0000_001_010_011_000, Qout6); 	//add $r3 ,$r1, $r2
	register_IM rIM7 (clk, reset, 16'b0000_111_011_100_000, Qout7);		//add $r4, $r7, $r3
	register_IM rIM8 (clk, reset, 16'b0101_110_111_000_000, Qout8); 	//mul $r6,$r7
	register_IM rIM9 (clk, reset, 16'b0000_000_000_000_000, Qout9); 		
	register_IM rIM10 (clk, reset, 16'b0000_000_000_000_000, Qout10); 	
	register_IM rIM11 (clk, reset, 16'b0000_000_000_000_000, Qout11); 	
	register_IM rIM12 (clk, reset, 16'b0000_000_000_000_000, Qout12); 
	register_IM rIM13 (clk, reset, 16'b0000_000_000_000_000, Qout13); 
	register_IM rIM14 (clk, reset, 16'b0000_000_000_000_000, Qout14); 	
	register_IM rIM15 (clk, reset, 16'b0000_000_000_000_000, Qout15); 	
	mux16to1 mIM (Qout0,Qout1,Qout2,Qout3,Qout4,Qout5,Qout6,Qout7,Qout8,Qout9,Qout10,Qout11,Qout12,Qout13,Qout14,Qout15,pc_5bits[4:1],IR);
endmodule

//Instruction Memory Code Ends

//%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%//

//Register Memory Code Starts
module D_ff_reg (input clk, input reset, input regWrite, input decOut1b, input d, output reg q);
	always @ (negedge clk)
		begin
			if(reset==1)
				q=1;
			else
				if(regWrite == 1 && decOut1b==1)
					begin
						q=d;
					end
		end
endmodule


module register16bit( input clk, input reset, input regWrite, input decOut1b, input [15:0] writeData, output  [15:0] outR );
	D_ff_reg d0(clk, reset, regWrite, decOut1b, writeData[0], outR[0]);
	D_ff_reg d1(clk, reset, regWrite, decOut1b, writeData[1], outR[1]);
	D_ff_reg d2(clk, reset, regWrite, decOut1b, writeData[2], outR[2]);
	D_ff_reg d3(clk, reset, regWrite, decOut1b, writeData[3], outR[3]);
	D_ff_reg d4(clk, reset, regWrite, decOut1b, writeData[4], outR[4]);
	D_ff_reg d5(clk, reset, regWrite, decOut1b, writeData[5], outR[5]);
	D_ff_reg d6(clk, reset, regWrite, decOut1b, writeData[6], outR[6]);
	D_ff_reg d7(clk, reset, regWrite, decOut1b, writeData[7], outR[7]);
	D_ff_reg d8(clk, reset, regWrite, decOut1b, writeData[8], outR[8]);
	D_ff_reg d9(clk, reset, regWrite, decOut1b, writeData[9], outR[9]);
	D_ff_reg d10(clk, reset, regWrite, decOut1b, writeData[10], outR[10]);
	D_ff_reg d11(clk, reset, regWrite, decOut1b, writeData[11], outR[11]);
	D_ff_reg d12(clk, reset, regWrite, decOut1b, writeData[12], outR[12]);
	D_ff_reg d13(clk, reset, regWrite, decOut1b, writeData[13], outR[13]);
	D_ff_reg d14(clk, reset, regWrite, decOut1b, writeData[14], outR[14]);
	D_ff_reg d15(clk, reset, regWrite, decOut1b, writeData[15], outR[15]);
endmodule


module register16bit_1( input clk, input reset, input regWrite, input [15:0] writeData, output  [15:0] outR );
	D_ff_I d0(clk, reset, regWrite, writeData[0], outR[0]);
	D_ff_I d1(clk, reset, regWrite, writeData[1], outR[1]);
	D_ff_I d2(clk, reset, regWrite, writeData[2], outR[2]);
	D_ff_I d3(clk, reset, regWrite, writeData[3], outR[3]);
	D_ff_I d4(clk, reset, regWrite, writeData[4], outR[4]);
	D_ff_I d5(clk, reset, regWrite, writeData[5], outR[5]);
	D_ff_I d6(clk, reset, regWrite, writeData[6], outR[6]);
	D_ff_I d7(clk, reset, regWrite, writeData[7], outR[7]);
	D_ff_I d8(clk, reset, regWrite, writeData[8], outR[8]);
	D_ff_I d9(clk, reset, regWrite, writeData[9], outR[9]);
	D_ff_I d10(clk, reset, regWrite, writeData[10], outR[10]);
	D_ff_I d11(clk, reset, regWrite, writeData[11], outR[11]);
	D_ff_I d12(clk, reset, regWrite, writeData[12], outR[12]);
	D_ff_I d13(clk, reset, regWrite, writeData[13], outR[13]);
	D_ff_I d14(clk, reset, regWrite, writeData[14], outR[14]);
	D_ff_I d15(clk, reset, regWrite, writeData[15], outR[15]);
endmodule

module registerSet( input clk, input reset, input regWrite, input [7:0] decOut, input [15:0] writeData,
output [15:0] outR0, output [15:0] outR1, output [15:0] outR2, output [15:0] outR3,
	output [15:0] outR4,output [15:0] outR5, output [15:0] outR6, output [15:0] outR7);

	register16bit rs0( clk, reset, regWrite, decOut[0], writeData, outR0 );
	register16bit rs1( clk, reset, regWrite, decOut[1], writeData, outR1 );
	register16bit rs2( clk, reset, regWrite, decOut[2], writeData, outR2 );
	register16bit rs3( clk, reset, regWrite, decOut[3], writeData, outR3 );
	
	register16bit rs4( clk, reset, regWrite, decOut[4], writeData, outR4 );
	register16bit rs5( clk, reset, regWrite, decOut[5], writeData, outR5 );
	register16bit rs6( clk, reset, regWrite, decOut[6], writeData, outR6 );
	register16bit rs7( clk, reset, regWrite, decOut[7], writeData, outR7 );
	
endmodule

module decoder( input [2:0] destReg, output reg [7:0] decOut);
always @(destReg)
	case(destReg)
	3'd0: decOut=8'b0000_0001;
	3'd1: decOut=8'b0000_0010;
	3'd2: decOut=8'b0000_0100;
	3'd3: decOut=8'b0000_1000;
	3'd4: decOut=8'b0001_0000;
	3'd5: decOut=8'b0010_0000;
	3'd6: decOut=8'b0100_0000;
	3'd7: decOut=8'b1000_0000;
	endcase
endmodule

module mux8to1( input [15:0] outR0, input [15:0] outR1, input [15:0] outR2, input [15:0] outR3, input [15:0] outR4, input [15:0] outR5, 
input [15:0] outR6, input [15:0] outR7, input [2:0] Sel, output reg [15:0] outBus );

always@(outR0,outR1,outR2,outR3,outR4,outR5,outR6,outR7,Sel)
	case(Sel)
		3'd0:outBus=outR0;
		3'd1:outBus=outR1;
		3'd2:outBus=outR2;
		3'd3:outBus=outR3;
		3'd4:outBus=outR4;
		3'd5:outBus=outR5;
		3'd6:outBus=outR6;
		3'd7:outBus=outR7;
	endcase
endmodule

module registerFile(input clk, input reset, input regWrite, input [2:0] rs, input [2:0] rt,input [2:0] rd, input [15:0] writeData, output [15:0] outR0, output [15:0] outR1);
	
	wire [7:0] decOut;
	wire [15:0] outR00,outR11, outR2, outR3, outR4, outR5, outR6, outR7;
	decoder dec(rd,decOut);
	registerSet rSet( clk, reset, regWrite,decOut, writeData,outR00,outR11, outR2, outR3, outR4, outR5, outR6, outR7);
	mux8to1 mux8_1_1( outR00, outR11,outR2,outR3,outR4, outR5, outR6,outR7, rs, outR0 );
	mux8to1 mux8_1_2( outR00, outR11,outR2,outR3,outR4, outR5, outR6,outR7, rt, outR1 );
endmodule
//Register Memory Code Ends

module register16bit_PC( input clk, input reset, input regWrite, input [15:0] writeData, output  [15:0] outR );
	
	D_ff d0(clk, reset,regWrite,writeData[0], outR[0]);
	D_ff d1(clk, reset,regWrite,writeData[1], outR[1]);
	D_ff d2(clk, reset,regWrite,writeData[2], outR[2]);
	D_ff d3(clk, reset,regWrite,writeData[3], outR[3]);
	D_ff d4(clk, reset,regWrite,writeData[4], outR[4]);
	D_ff d5(clk, reset,regWrite,writeData[5], outR[5]);
	D_ff d6(clk, reset,regWrite,writeData[6], outR[6]);
	D_ff d7(clk, reset,regWrite,writeData[7], outR[7]);
	
	D_ff d8(clk, reset,regWrite,writeData[8], outR[8]);
	D_ff d9(clk, reset,regWrite,writeData[9], outR[9]);
	D_ff d10(clk, reset,regWrite,writeData[10], outR[10]);
	D_ff d11(clk, reset,regWrite,writeData[11], outR[11]);
	D_ff d12(clk, reset,regWrite,writeData[12], outR[12]);
	D_ff d13(clk, reset,regWrite,writeData[13], outR[13]);
	D_ff d14(clk, reset,regWrite,writeData[14], outR[14]);
	D_ff d15(clk, reset,regWrite,writeData[15], outR[15]);
	
endmodule

module ctrlCkt	(input [3:0] opcode, output reg regWrite, output reg regDst, output reg [1:0] aluSrcB, output reg aluSrcC,
		output reg [1:0] aluOp, output reg hiWrite, output reg loWrite,output reg flagWr);

	always@(opcode)
		begin
			case(opcode)
				4'b0000:	begin
								regWrite = 1'b1;
								regDst = 1'b1;
								aluSrcB = 2'b00;
								aluSrcC = 1'b0;
								aluOp = 2'b00;
								hiWrite = 1'b0;
								loWrite = 1'b0;
								flagWr = 1'b1;
							end
				4'b0001:	begin
								regWrite = 1'b1;
								regDst = 1'b1;
								aluSrcB = 2'b00;
								aluSrcC = 1'b0;
								aluOp = 2'b01;
								hiWrite = 1'b0;
								loWrite = 1'b0;
								flagWr = 1'b1;
							end
				4'b0010:	begin
								regWrite = 1'b1;
								regDst = 1'b1;
								aluSrcB = 2'b00;
								aluSrcC = 1'b1;
								aluOp = 2'b00;
								hiWrite = 1'b0;
								loWrite = 1'b0;
								flagWr = 1'b1;
							end
				4'b0011:	begin
								regWrite = 1'b1;
								regDst = 1'b0;
								aluSrcB = 2'b01;
								aluSrcC = 1'b1;
								aluOp = 2'b00;
								hiWrite = 1'b0;
								loWrite = 1'b0;
								flagWr = 1'b1;
							end
				4'b0100:	begin
								regWrite = 1'b1;
								regDst = 1'b0;
								aluSrcB = 2'b10;
								aluSrcC = 1'b0;
								aluOp = 2'b10;
								hiWrite = 1'b0;
								loWrite = 1'b0;
								flagWr = 1'b1;
							end
				4'b0101:	begin
								regWrite = 1'b0;
								regDst = 1'b0;
								aluSrcB = 2'b00;
								aluSrcC = 1'b0;
								aluOp = 2'b11;
								hiWrite = 1'b1;
								loWrite = 1'b1;
								flagWr = 1'b0;
							end
			endcase
		end
endmodule

module mux2to1_3bits(input [2:0] in1, input [2:0] in2, input sel, output reg [2:0] muxOut);

	always@(in1 or in2 or sel)
	case(sel)
		1'd0:muxOut=in1;
		1'd1:muxOut=in2;
	endcase
endmodule

module mux2to1_16bits(input [15:0] in1, input [15:0] in2, input sel, output reg [15:0] muxOut);

	always@(in1 or in2 or sel)
	case(sel)
		1'd0:muxOut=in1;
		1'd1:muxOut=in2;
	endcase
endmodule

module mux4to1_16bits(input [15:0] in1, input [15:0] in2, input [15:0] in3, input [1:0] sel, output reg [15:0] muxOut);

	 always@(in1 or in2 or in3 or sel)
	case(sel)
		2'b00:muxOut = in1;
		2'b01:muxOut = in2;
		2'b10:muxOut = in3;
		2'b11:muxOut = 16'bz;
	endcase
endmodule

module adder(input [15:0] in1, input [15:0] in2, output reg [15:0] adder_out);

	 always@(in1 or in2)
		begin
			adder_out = in1 + in2;
		end
	
endmodule

module signExt6to16( input [5:0] offset, output reg [15:0] signExtOffset);

	always@(offset)
		begin
			if(offset[5] == 1)
				begin
				signExtOffset[15:6] = 10'b1111111111;
				signExtOffset[5:0] = offset;
				end
			else
				begin
				signExtOffset[15:6] = 10'b0000000000;
				signExtOffset[5:0] = offset;
				end
		end
		
endmodule

module zeroExt4to16( input [3:0] offset, output reg [15:0] zeroExtOffset);

	always@(offset)
		begin
				zeroExtOffset = {10'b0000000000,offset};
		end
endmodule	


module alu(input [15:0] aluIn1, input [15:0] aluIn2,input [15:0] aluIn3, input [1:0] aluOp,output reg zero,output reg neg,output reg pos,
		output reg carry, output reg [15:0] aluOut1,output reg [15:0] aluOut2);
		reg [16:0] out1;

	always@(aluIn1 or aluIn2 or aluIn3 or aluOp)
		begin
			case(aluOp)
				2'b00:	begin
								out1[16:0] = aluIn1 + aluIn2 + aluIn3;
									begin
									if (out1 == 0)
										zero = 1'b1;
									else 
										zero = 1'b0;
									end
									begin
									if (out1[15] == 0)
										pos = 1'b1;
									else 
										pos = 1'b0;
									end
									begin
									if (out1[15] == 1)
										neg = 1'b1;
									else 
										neg = 1'b0;
									end
								carry = out1[16];
								aluOut1 = out1[15:0];
								aluOut2 = 16'd0;	
							end
				2'b01:	begin
								out1[16:0] = aluIn1 - aluIn2;
									begin
									if (out1 == 0)
										zero = 1'b1;
									else 
										zero = 1'b0;
									end
									begin
									if (out1[15] == 0)
										pos = 1'b1;
									else 
										pos = 1'b0;
									end
									begin
									if (out1[15] == 1)
										neg = 1'b1;
									else 
										neg = 1'b0;
									end
								carry = out1[16];
								aluOut1 = out1[15:0];
								aluOut2 = 16'h0000;
							end
				2'b10:	begin
								out1[16:0] = aluIn1 << aluIn2;
									begin
									if (out1 == 0)
										zero = 1'b1;
									else 
										zero = 1'b0;
									end
									begin
									if (out1[15] == 0)
										pos = 1'b1;
									else 
										pos = 1'b0;
									end
									begin
									if (out1[15] == 1)
										neg = 1'b1;
									else 
										neg = 1'b0;
									end
								carry = out1[16];
								aluOut1 = out1[15:0];
								aluOut2 = 16'h0000;
							end
				2'b11:{aluOut2,aluOut1} = aluIn1 * aluIn2 ;
							
					
			endcase
		end
				
endmodule


module singleCycle(input clk, input reset, output wire [15:0] Result );

wire [15:0]adder_out,hiout,aluOut2;
wire [15:0]outR;
wire	[15:0]IR;
wire	regDst;
wire	[15:0]muxOut;
wire [15:0]muxOut1;
wire [15:0]muxOut2;
wire [15:0]outR0;
wire [15:0]outR1;
wire	regWrite;
wire [1:0]aluSrcB;
wire 	aluSrcC;
wire 	[1:0]aluOp;
wire hiWrite; 
wire 	loWrite;
wire 	flagWr;
wire 	[15:0]signExtOffset;
wire 	[15:0]zeroExtOffset;
wire	carry;
wire pos,neg,zero;
wire [3:0] out;

alu alu1(outR0, muxOut, muxOut1,aluOp, zero, neg, pos, carry, Result, aluOut2);
register16bit_PC PC0(clk, reset,1'b1,adder_out,outR );
adder add(16'd2,outR,adder_out);
IM im( clk,reset,outR[4:0],IR );
mux2to1_3bits mux2to1_1(IR[8:6], IR[5:3],regDst, muxOut2);
registerFile regfile(clk,reset,regWrite,IR[11:9],IR[8:6], muxOut2, Result, outR0,outR1);
ctrlCkt ctrlckt(IR[15:12],regWrite,regDst, aluSrcB, aluSrcC, aluOp, hiWrite, loWrite,flagWr);
signExt6to16 signext(IR[5:0],signExtOffset);
zeroExt4to16 zeroext(IR[3:0],zeroExtOffset);
mux4to1_16bits mux4to1_1(outR1,signExtOffset,zeroExtOffset,aluSrcB ,muxOut);
mux2to1_16bits mux2to1(16'b0000000000000000,{15'b000000000000000,out[1]},aluSrcC , muxOut1);
Flag_reg flagreg(clk,reset,flagWr,pos, neg,zero,carry, out);
register16bit_1 reghi(clk,reset,hiWrite,aluOut2,hiout );
	
endmodule

module singleCycleTestBench;
	reg clk;
	reg reset;
	wire [15:0] Result;
	singleCycle uut (.clk(clk), .reset(reset), .Result(Result));

	always
	#5 clk=~clk;
	
	initial
	begin
		clk=0; reset=1;
		#7  reset=0;	
		#85 $finish; 
	end
endmodule




