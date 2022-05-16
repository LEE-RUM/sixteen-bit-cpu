// 16-bit MIPS ALU in Verilog
module ALU (op,x,y,result,zero);
   input [15:0] x;
   input [15:0] y;
   input [2:0] op;
   output [15:0] result;
   output zero;
   wire z1,z2,z3,z4,z5,z6,z7,z8,z9,z10,z11,z12,z13,z14,z15,z16;
	
   ALU0to2   alu0  (x[0], y[0], op[2], op[1:0],set,op[2],z1,result[0]);
   ALU0to2   alu1  (x[1], y[1], op[2], op[1:0],1'b0,  z1,   z2,result[1]);
   ALU0to2   alu2  (x[2], y[2], op[2], op[1:0],1'b0,  z2,   z3,result[2]);
   ALU0to2   alu3  (x[3], y[3], op[2], op[1:0],1'b0,  z3,   z4,result[3]);
   ALU0to2   alu4  (x[4], y[4], op[2], op[1:0],1'b0,  z4,   z5,result[4]);
   ALU0to2   alu5  (x[5], y[5], op[2], op[1:0],1'b0,  z5,   z6,result[5]);
   ALU0to2   alu6  (x[6], y[6], op[2], op[1:0],1'b0,  z6,   z7,result[6]);
   ALU0to2   alu7  (x[7], y[7], op[2], op[1:0],1'b0,  z7,   z8,result[7]);
   ALU0to2   alu8  (x[8], y[8], op[2], op[1:0],1'b0,  z8,   z9,result[8]);
   ALU0to2   alu9  (x[9], y[9], op[2], op[1:0],1'b0,  z9,   z10,result[9]);
   ALU0to2   alu10 (x[10],y[10],op[2], op[1:0],1'b0,  z10,  z11,result[10]);
   ALU0to2   alu11 (x[11],y[11],op[2], op[1:0],1'b0,  z11,  z12,result[11]);
   ALU0to2   alu12 (x[12],y[12],op[2], op[1:0],1'b0,  z12,  z13,result[12]);
   ALU0to2   alu13 (x[13],y[13],op[2], op[1:0],1'b0,  z13,  z14,result[13]);
   ALU0to2   alu14 (x[14],y[14],op[2], op[1:0],1'b0,  z14,  z15,result[14]);
   ALUmsb alu15 (x[15],y[15],op[2], op[1:0],1'b0,  z15,  z16,result[15],set);

 or or1(or0to15, result[0],result[1],result[2],result[3],result[4],result[5],
                result[6],result[7],result[8],result[9],result[10],result[11],
                result[12],result[13],result[14],result[15]);
  not n1(zero, or0to15);
 
endmodule


// 1-bit ALU for bits 0-2

module ALU0to2 (x,y,yinvert,op,less,carryin,carryout,result);
   input x,y,less,carryin,yinvert;
   input [1:0] op;
   output carryout,result;
   wire sum, x_and_y, x_or_y, y_inv;
	
   not not1(y_inv, y);
   mux2x1 mux1(y,y_inv,yinvert,y1);
   and and1(x_and_y, x, y);
   or or1(x_or_y, x, y);
   fulladder adder1(sum,carryout,x,y1,carryin);
   mux4x1 mux2(x_and_y,x_or_y,sum,less,op[1:0],result); 

endmodule


// 1-bit ALU for the most significant bit

module ALUmsb (x,y,yinvert,op,less,carryin,carryout,result,sum);
   input x,y,less,carryin,yinvert;
   input [1:0] op;
   output carryout,result,sum;
   wire sum, x_and_y, x_or_y, y_inv;
	
   not not1(y_inv, y);
   mux2x1 mux1(y,y_inv,yinvert,y1);
   and and1(x_and_y, x, y);
   or or1(x_or_y, x, y);
   fulladder adder1(sum,carryout,x,y1,carryin);
   mux4x1 mux2(x_and_y,x_or_y,sum,less,op[1:0],result); 

endmodule

module halfadder (S,C,x,y); 
   input x,y; 
   output S,C; 

   xor (S,x,y); 
   and (C,x,y); 
endmodule 


module fulladder (S,C,x,y,z); 
   input x,y,z; 
   output S,C; 
   wire S1,D1,D2;

   halfadder HA1 (S1,D1,x,y), 
             HA2 (S,D2,S1,z); 
   or g1(C,D2,D1); 
endmodule 


module mux2x1(X,Y,select,OUT); // 2x1 multiplexer
   input X,Y,select; 
   output OUT; 
   
   not not1(i0, select);
   and and1(i1, X, i0);
   and and2(i2, Y, select);
   or or1(OUT, i1, i2);
   
endmodule 

module mux4x1(i0,i1,i2,i3,select,y); // 4x1 multiplexer
   input i0,i1,i2,i3; 
   input [1:0] select; 
   output y; 
   
   mux2x1 mux1(i0, i1, select[0], mu1);
   mux2x1 mux2(i2, i3, select[0], mu2);
   mux2x1 mux3(mu1, mu2, select[1], y);
 
endmodule 

module mux2_bit_2x1(X,Y,select,OUT);
	input [1:0] X,Y;
    input select;
	output [1:0] OUT;
    mux2x1 mux1(X[0], Y[0], select, OUT[0]),
           mux2(X[1], Y[1], select, OUT[1]);
endmodule

module mux16_bit_2x1(X, Y, select, OUT);
	input [15:0] X,Y;
    input select;
	output [15:0] OUT;
    mux2x1 mux1(X[0],   Y[0],  select, OUT[0]),
           mux2(X[1],   Y[1],  select, OUT[1]),
           mux3(X[2],   Y[2],  select, OUT[2]),
           mux4(X[3],   Y[3],  select, OUT[3]),
           mux5(X[4],   Y[4],  select, OUT[4]),
           mux6(X[5],   Y[5],  select, OUT[5]),
           mux7(X[6],   Y[6],  select, OUT[6]),
           mux8(X[7],   Y[7],  select, OUT[7]),
           mux9(X[8],   Y[8],  select, OUT[8]),
           mux10(X[9],  Y[9],  select, OUT[9]),
           mux11(X[10], Y[10], select, OUT[10]),
           mux12(X[11], Y[11], select, OUT[11]),
           mux13(X[12], Y[12], select, OUT[12]),
           mux14(X[13], Y[13], select, OUT[13]),
           mux15(X[14], Y[14], select, OUT[14]),
           mux16(X[15], Y[15], select, OUT[15]);
endmodule


module mainControl (Op,Control); 

  input [3:0] Op;
  output reg [9:0] Control;
// IDEX_RegDst,IDEX_ALUSrc,IDEX_MemtoReg,IDEX_RegWrite,IDEX_MemWrite,IDEX_Beq,IDEX_Bne,IDEX_ALUctl
  always @(Op) case (Op)
  
        4'b0000: Control <= 10'b1001000010; // ADD  
        4'b1001: Control <= 10'b0111000010; // LW
        4'b0100: Control <= 10'b1001000111; // SLT
        4'b1011: Control <= 10'b0000010110; // BEQ
        4'b0001: Control <= 10'b1001000110; // SUB  
        4'b0010: Control <= 10'b1001000000; // AND  
        4'b0011: Control <= 10'b1001000001; // OR  
        4'b0101: Control <= 10'b0101000010; // ADDI 
        4'b0110: Control <= 10'b0101000111; // SLTI 
        4'b0111: Control <= 10'b0101000000; // ANDI 
        4'b1000: Control <= 10'b0101000001; // ORI 
        4'b1010: Control <= 10'b0100100010; // SW  
        4'b1100: Control <= 10'b0000001110; // BNE  
      
  endcase

endmodule

module branchControl (beq,bne,zero,branchOut); // branch control

  input beq,bne; 
  input zero;
  output branchOut;
  wire not_Zero,i0,i1;

  not n1(not_Zero,zero);
  and a1(i0,beq,zero);
  and a2(i1,bne,not_Zero);
  or o1(branchOut,i0,i1);
  
endmodule

module CPU (clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);

  input clock;
  output [15:0] PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD;

  initial begin 
// Program: swap memory cells (if needed) and compute absolute value |5-7|=2

   IMemory[0] = 16'b1001_00_01_00000000;   // lw $1, 0($0)
   IMemory[1] = 16'b1001_00_10_00000010;   // lw $2, 2($0)
   IMemory[2] = 16'b0000000000000000;      // nop
   IMemory[3] = 16'b0000000000000000;      // nop
   IMemory[4] = 16'b0000000000000000;      // nop
   IMemory[5] = 16'b0100_01_10_11_000000;  // slt $3, $1, $2
   IMemory[6] = 16'b0000000000000000;      // nop
   IMemory[7] = 16'b0000000000000000;      // nop
   IMemory[8] = 16'b0000000000000000;      // nop
   IMemory[9] = 16'b1011_11_00_00000101;   // beq $3, $0, IMemory[15]
  //IMemory[9] = 16'b1100_11_00_00000101;   // bne $3, $0, IMemory[15]
   IMemory[10] = 16'b0000000000000000;     // nop
   IMemory[11] = 16'b0000000000000000;     // nop
   IMemory[12] = 16'b0000000000000000;     // nop
   IMemory[13] = 16'b1010_00_01_00000010;  // sw $1, 2($0)
   IMemory[14] = 16'b1010_00_10_00000000;  // sw $2, 0($0)
   IMemory[15] = 16'b0000000000000000;     // nop
   IMemory[16] = 16'b0000000000000000;     // nop
   IMemory[17] = 16'b0000000000000000;     // nop
   IMemory[18] = 16'b1001_00_01_00000000;  // lw $1, 0($0)
   IMemory[19] = 16'b1001_00_10_00000010;  // lw $2, 2($0)
   IMemory[20] = 16'b0000000000000000;     // nop
   IMemory[21] = 16'b0000000000000000;     // nop
   IMemory[22] = 16'b0000000000000000;     // nop
   IMemory[23] = 16'b0001_01_10_11_000000; // sub $3, $1, $2
 
  
  //without nops
  /*
   IMemory[0] = 16'b1001_00_01_00000000;      // lw $1, 0($0)
   IMemory[1] = 16'b1001_00_10_00000010;      // lw $2, 2($0)
   IMemory[2] = 16'b0100_01_10_11_000000;     // slt $3, $1, $2
   IMemory[3] = 16'b1011_11_00_00000101;      // beq $3, $0, IMemory[15]
   IMemory[4] = 16'b1010_00_01_00000010;     // sw $1, 2($0)
   IMemory[5] = 16'b1010_00_10_00000000;     // sw $2, 0($0)
   IMemory[6] = 16'b1001_00_01_00000000;     // lw $1, 0($0)
   IMemory[7] = 16'b1001_00_10_00000010;     // lw $2, 2($0)
   IMemory[8] = 16'b0001_01_10_11_000000;    // sub $3, $1, $2
   
  */
  
// Data
   DMemory [0] = 16'h5; // switch the cells and see how the simulation output changes
   DMemory [1] = 16'h7; // (beq is taken if [0]=32'h7; [1]=32'h5, not taken otherwise)
  end

// Pipeline 

// IF 
   wire [15:0] PCplus4, NextPC;
   reg[15:0] PC, IMemory[0:1023], IFID_IR, IFID_PCplus4;
   ALU fetch (3'b010,PC,16'b0000000000000010,PCplus4,Unused1);
   //assign NextPC = (EXMEM_Beq && EXMEM_Zero || EXMEM_Bne && !EXMEM_Zero) ? EXMEM_Target: PCplus4;
   
    

// ID
   wire [9:0] Control;
   reg IDEX_RegWrite,IDEX_MemtoReg,
       IDEX_Beq, IDEX_Bne, IDEX_MemWrite,
       IDEX_ALUSrc,  IDEX_RegDst;
   reg [2:0]  IDEX_ALUctl;
   wire [15:0] RD1,RD2,SignExtend, WD;
   reg [15:0] IDEX_PCplus4,IDEX_RD1,IDEX_RD2,IDEX_SignExt,IDEXE_IR;
   reg [15:0] IDEX_IR; // For monitoring the pipeline
   reg [1:0]  IDEX_rt,IDEX_rd;
   reg MEMWB_RegWrite; // part of MEM stage, but declared here before use (to avoid error)
   reg [1:0] MEMWB_rd; // part of MEM stage, but declared here before use (to avoid error)
   reg_file rf (IFID_IR[11:10],IFID_IR[9:8],MEMWB_rd,WD,MEMWB_RegWrite,RD1,RD2,clock);
   mainControl MainCtr (IFID_IR[15:12],Control); 
   assign SignExtend = {{8{IFID_IR[7]}},IFID_IR[7:0]}; 
  
// EXE
   reg EXMEM_RegWrite,EXMEM_MemtoReg,
       EXMEM_Beq, EXMEM_Bne, EXMEM_MemWrite;
   wire [15:0] Target;
   reg EXMEM_Zero;
   reg [15:0] EXMEM_Target,EXMEM_ALUOut,EXMEM_RD2;
   reg [15:0] EXMEM_IR; // For monitoring the pipeline
   reg [1:0] EXMEM_rd;
   wire [15:0] B,ALUOut;
  // wire [2:0] ALUctl;
   wire [4:0] WR;
   ALU branch (3'b010,IDEX_SignExt<<1,IDEX_PCplus4,Target,Unused2);
   ALU ex (IDEX_ALUctl, IDEX_RD1, B, ALUOut, Zero);
   //ALUControl ALUCtrl(IDEX_ALUOp, IDEX_SignExt[5:0], ALUctl); // ALU control unit
   //assign B  = (IDEX_ALUSrc) ? IDEX_SignExt: IDEX_RD2;         
   mux16_bit_2x1 m1 (IDEX_RD2,IDEX_SignExt,IDEX_ALUSrc,B);      // ALUSrcMux
   //assign WR = (IDEX_RegDst) ? IDEX_rd: IDEX_rt;               
   mux2_bit_2x1 m2 (IDEX_rt,IDEX_rd,IDEX_RegDst,WR);            // RegDst Mux
   branchControl BCtr (EXMEM_Beq,EXMEM_Bne,EXMEM_Zero,branchOut);   // branch
   mux16_bit_2x1 m4(PCplus4,EXMEM_Target,branchOut,NextPC);           //Mux for branch

// MEM
   reg MEMWB_MemtoReg;
   reg [15:0] DMemory[0:1023],MEMWB_MemOut,MEMWB_ALUOut;
   reg [15:0] MEMWB_IR; // For monitoring the pipeline
   wire [15:0] MemOut;
   assign MemOut = DMemory[EXMEM_ALUOut>>1];
   always @(negedge clock) if (EXMEM_MemWrite) DMemory[EXMEM_ALUOut>>1] <= EXMEM_RD2;
  
// WB
   //assign WD = (MEMWB_MemtoReg) ? MEMWB_MemOut: MEMWB_ALUOut; // MemtoReg Mux
    mux16_bit_2x1 m3 (MEMWB_ALUOut, MEMWB_MemOut, MEMWB_MemtoReg, WD);

   initial begin
    PC = 0;
// Initialize pipeline registers
    IDEX_RegWrite=0;IDEX_MemtoReg=0;IDEX_Beq=0;IDEX_Bne=0;IDEX_MemWrite=0;IDEX_ALUSrc=0;IDEX_RegDst=0;IDEX_ALUctl=0;
    IFID_IR=0;
    EXMEM_RegWrite=0;EXMEM_MemtoReg=0;EXMEM_Beq=0;EXMEM_Bne=0;EXMEM_MemWrite=0;
    EXMEM_Target=0;
    MEMWB_RegWrite=0;MEMWB_MemtoReg=0;
   end

// Running the pipeline

   always @(negedge clock) begin 

// IF
    PC <= NextPC;
    IFID_PCplus4 <= PCplus4;
    IFID_IR <= IMemory[PC>>1];

// ID
    IDEX_IR <= IFID_IR; // For monitoring the pipeline
    {IDEX_RegDst,IDEX_ALUSrc,IDEX_MemtoReg,IDEX_RegWrite,IDEX_MemWrite,IDEX_Beq,IDEX_Bne,IDEX_ALUctl} <= Control;   
    IDEX_PCplus4 <= IFID_PCplus4;
    IDEX_RD1 <= RD1; 
    IDEX_RD2 <= RD2;
    IDEX_SignExt <= SignExtend;
    IDEX_rt <= IFID_IR[9:8];
    IDEX_rd <= IFID_IR[7:6];

// EXE
    EXMEM_IR <= IDEX_IR; // For monitoring the pipeline
    EXMEM_RegWrite <= IDEX_RegWrite;
    EXMEM_MemtoReg <= IDEX_MemtoReg;
    EXMEM_Beq   <= IDEX_Beq;
    EXMEM_Bne   <= IDEX_Bne;
    EXMEM_MemWrite <= IDEX_MemWrite;
    EXMEM_Target <= Target;
    EXMEM_Zero <= Zero;
    EXMEM_ALUOut <= ALUOut;
    EXMEM_RD2 <= IDEX_RD2;
    EXMEM_rd <= WR;

// MEM
    MEMWB_IR <= EXMEM_IR; // For monitoring the pipeline
    MEMWB_RegWrite <= EXMEM_RegWrite;
    MEMWB_MemtoReg <= EXMEM_MemtoReg;
    MEMWB_MemOut <= MemOut;
    MEMWB_ALUOut <= EXMEM_ALUOut;
    MEMWB_rd <= EXMEM_rd;

// WB
// Register write happens on neg edge of the clock (if MEMWB_RegWrite is asserted)

  end

endmodule

// Behavioral model of MIPS - pipelined implementation

module reg_file (RR1,RR2,WR,WD,RegWrite,RD1,RD2,clock);

  input [1:0] RR1,RR2,WR;
  input [15:0] WD;
  input RegWrite,clock;
  output [15:0] RD1,RD2;

  reg [15:0] Regs[0:3];

  assign RD1 = Regs[RR1];
  assign RD2 = Regs[RR2];

  initial Regs[0] = 0;

  always @(negedge clock)
    if (RegWrite==1 & WR!=0) 
	Regs[WR] <= WD;

endmodule

// Test module

module test ();

  reg clock;
  wire [15:0] PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD;

  CPU test_cpu(clock,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);

  always #1 clock = ~clock;
  
  initial begin
    $display ("time PC  IFID_IR  IDEX_IR  EXMEM_IR MEMWB_IR WD");
    $monitor ("%2d  %3d    %h   %h    %h      %h    %2d", $time,PC,IFID_IR,IDEX_IR,EXMEM_IR,MEMWB_IR,WD);
    clock = 1;
    #56 $finish;
  end

endmodule
