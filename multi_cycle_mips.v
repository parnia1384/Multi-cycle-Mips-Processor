
`timescale 1ns/100ps

   `define ADD  3'b000
   `define SUB  3'b001
   `define SLT  3'b010
   `define SLTU 3'b011
   `define AND  3'b100
   `define XOR  3'b101
   `define OR   3'b110
   `define NOR  3'b111

`default_nettype none

module multi_cycle_mips(

   input wire clk,
   input wire reset,

   // Memory Ports
   output wire  [31:0] mem_addr,
   input  wire  [31:0] mem_read_data,
   output wire  [31:0] mem_write_data,
   output wire         mem_read,
   output wire         mem_write
);
//Parnia Rezaei 402101793
   // Data Path Registers
   reg MRE, MWE;
   reg [31:0] A, B, PC, IR, MDR, MAR, lo, hi, jal;

   // Data Path Control Lines, don't forget, regs are not always register !!
   reg setMRE, clrMRE, setMWE, clrMWE, start;//setMemoryReadEnable, clearMemoryEnable, setMemoryWriteEbanble, setMemoryReadEnable
   reg Awrt, Bwrt, RFwrt, IRwrt, MDRwrt, MARwrt;//MDWrt: Memory Data write, MARwt: Memory Address Register
   reg[1:0] PCwrt;

   // Memory Ports Bindings
   assign mem_addr = MAR;
   assign mem_read = MRE;
   assign mem_write = MWE;
   assign mem_write_data = B;

   // Mux & ALU Control Line
   reg [2:0] aluOp;
   reg [1:0] MemtoReg;
   reg [2:0] aluSelB;
   reg SgnExt, aluSelA, RegDst, IorD;

   // Wiring
   wire aluZero, ready;

   wire [31:0] aluResult, RD1, RD2, LO, HI;

   wire [ 4:0] RR1 = IR[25:21];
   wire [ 4:0] RR2 = IR[20:16];

   wire [31:0] WD = (MemtoReg == 2'b01) ? MDR : 
                    (MemtoReg == 2'b00) ? aluResult :
                    (MemtoReg == 2'b10) ? ((IR[5:0] == 6'b010_000) ? HI : LO) : 
                    (MemtoReg == 2'b11) ? {IR[15:0], 16'b0} : 31'bx;
   wire [ 4:0] WR  = (jal == 1'b1) ? 5'b11111 : (RegDst ? IR[15:11] : IR[20:16]);

   // Clocked Registers
   always @( posedge clk ) begin
      if( reset )
         PC <= #0.1 32'h00000000;
      else if(PCwrt == 2'b01)
         PC <= #0.1 aluResult;
      else if(PCwrt == 2'b10)
         PC <= #0.1 A;
      else if(PCwrt == 2'b11)//jump & jal instruction
         PC <= #0.1 {PC[31:28], IR[25:0], 2'b00};

      if( Awrt ) A <= #0.1 RD1;
      if( Bwrt ) B <= #0.1 RD2;

      if( MARwrt ) MAR <= #0.1 IorD ? aluResult : PC;

      if( IRwrt ) IR <= #0.1 mem_read_data;
      if( MDRwrt ) MDR <= #0.1 mem_read_data;

      if( reset | clrMRE ) MRE <= #0.1 1'b0;
          else if( setMRE) MRE <= #0.1 1'b1;

      if( reset | clrMWE ) MWE <= #0.1 1'b0;
          else if( setMWE) MWE <= #0.1 1'b1;
   end

   // Register File
   reg_file rf(
      .write( RFwrt ),
      .clk(clk),
      .WR(WR),
      .WD(WD),
      .RR1(RR1),
      .RR2(RR2),
      .RD1(RD1),
      .RD2(RD2)
   );

   // Sign/Zero Extension
   wire [31:0] SZout = SgnExt ? {{16{IR[15]}}, IR[15:0]} : {16'h0000, IR[15:0]};

   // ALU-A Mux
   wire [31:0] aluA = aluSelA ? A : PC;

   // ALU-B Mux
   reg [31:0] aluB;
   always @(*)
   case (aluSelB)
      3'b000: aluB = B;
      3'b001: aluB = 32'h4;
      3'b010: aluB = SZout;
      3'b011: aluB = SZout << 2;
      3'b100: aluB = 0;
      default : aluB = 32'hxxxxxxxx;
   endcase

   my_alu alu(
      .A( aluA ),
      .B( aluB ),
      .Op( aluOp ),

      .X( aluResult ),
      .Z( aluZero )
   );
   multiplier my_multiplier(
      .clk(clk),
      .start(start),
      .A(A),
      .B(B),
      .lo(LO),
      .hi(HI),
      .ready(ready)
   );
   always @ (*)begin
      if(ready) begin
         lo = LO;
         hi = HI;
      end
   end
   // Controller Starts Here

   // Controller State Registers
   reg [4:0] state, nxt_state;

   // State Names & Numbers
   localparam
      RESET = 0, FETCH1 = 1, FETCH2 = 2, FETCH3 = 3, DECODE = 4, 
      EX_ALU_R = 7, EX_ALU_I = 8, EX_MULT_1 = 9, EX_MULT_2 = 10,
      EX_LW_1 = 11, EX_LW_2 = 12, EX_LW_3 = 13, EX_LW_4 = 14, EX_LW_5 = 15,
      EX_JALR = 16, EX_JR = 17,
      EX_SW_1 = 21, EX_SW_2 = 22, EX_SW_3 = 23,
      EX_BRA_1 = 25, EX_BRA_2 = 26;

   // State Clocked Register 
   always @(posedge clk)
      if(reset)
         state <= #0.1 RESET;
      else
         state <= #0.1 nxt_state;

   task PrepareFetch;
      begin
         IorD = 0;
         setMRE = 1;
         MARwrt = 1;
         nxt_state = FETCH1;
      end
   endtask

   // State Machine Body Starts Here
   always @( * ) begin

      nxt_state = 'bx;

      SgnExt = 'bx; IorD = 'bx;
      MemtoReg = 'bx; RegDst = 'bx;
      aluSelA = 'bx; aluSelB = 'bx; aluOp = 'bx;
      start = 0;
      PCwrt = 0;
      Awrt = 0; Bwrt = 0;
      RFwrt = 0; IRwrt = 0;
      MDRwrt = 0; MARwrt = 0;
      setMRE = 0; clrMRE = 0;
      setMWE = 0; clrMWE = 0;
      jal = 0;
      case(state)
         RESET:
            PrepareFetch;

         FETCH1:
            nxt_state = FETCH2;

         FETCH2:
            nxt_state = FETCH3;

         FETCH3: begin
           IRwrt = 1;
            PCwrt = 1;
            clrMRE = 1;
            aluSelA = 0;
            aluSelB = 3'b001;
            aluOp = `ADD;
            nxt_state = DECODE;
         end

         DECODE: begin
            Awrt = 1;
            Bwrt = 1;
            case( IR[31:26] )
               6'b000_000: begin// R-format
                  case( IR[5:3] )
                     3'b000: ;
                     3'b001: begin                        
                        case(IR[2:0])
                           3'b000: nxt_state = EX_JR;//jr
                           3'b001: nxt_state = EX_JALR;//jalr
                           default : nxt_state = RESET;
                        endcase
                     end
                     3'b010: begin
                        case(IR[2:0])
                           3'b000,
                           3'b010: begin
                              RFwrt = 1'b1;
                              MemtoReg = 2'b10;
                              RegDst = 1'b1;
                              nxt_state = RESET;
                           end
                        endcase
                     end
                     3'b011: begin
                        case(IR[2:0])
                           3'b001: begin
                              nxt_state = EX_MULT_1;//multu
                           end
                           default : nxt_state = RESET;
                        endcase
                     end
                     3'b100: nxt_state = EX_ALU_R;//add, addu, sub, subu, and, or, xor, nor
                     3'b101: nxt_state = EX_ALU_R;//slt, sltu
                     3'b110: ;
                     3'b111: ;
                  endcase
               end
               6'b001_000,             // addi
               6'b001_001,             // addiu
               6'b001_010,             // slti
               6'b001_011,             // sltiu
               6'b001_100,             // andi
               6'b001_101,             // ori
               6'b001_110:             // xori
                  nxt_state = EX_ALU_I;

               6'b100_011://lw
                  nxt_state = EX_LW_1;

               6'b101_011://sw
                  nxt_state = EX_SW_1;

               6'b000_100,//beq
               6'b000_101://bne
                  nxt_state = EX_BRA_1;
               6'b000_010: begin
                  nxt_state = RESET;
                  PCwrt = 2'b11;
               end//j
               6'b000_011: begin
                  jal = 1;
                  aluOp = `ADD;
                  MemtoReg = 2'b00;
                  aluSelA = 1'b0;
                  aluSelB = 3'b100;
                  RFwrt = 1'b1;
                  PCwrt = 2'b11;
                  nxt_state = RESET;
               end//jal
               6'b001111:begin
                  MemtoReg = 2'b11;
                  RFwrt = 1'b1;
                  RegDst = 1'b0;
                  nxt_state = RESET;
               end//lui
            endcase
         end
         EX_JR: begin
            nxt_state = RESET;
            PCwrt = 2'b10;
         end
         EX_JALR: begin
            PCwrt = 2'b10;
            aluSelA = 0;
            aluSelB = 3'b100;
            MemtoReg = 3'b000;
            RFwrt = 1;
            aluOp = `ADD;
            RegDst = 1;
            nxt_state = RESET;
         end
         EX_ALU_R: begin
            RFwrt = 1;
            RegDst = 1'b1;
            MemtoReg = 2'b00;
            aluSelB = 3'b000;
            aluSelA = 1'b1;
            case(IR[2:0])
               3'b000:begin
                  SgnExt = 1'b1;
                  aluOp = `ADD;
               end//add
               3'b001:begin
                  SgnExt = 0;
                  aluOp = `ADD;
               end//addu
               3'b010:begin
                  SgnExt = 1'b1;
                  case(IR[5:3])
                     3'b100: begin//sub
                        aluOp = `SUB;
                     end
                     3'b101: begin//slt
                        aluOp = `SLT;
                     end
                  endcase
               end//sub,slt
               3'b011:begin
                  SgnExt = 1'b0;
                  case(IR[5:3])
                     3'b100: begin//subu
                        aluOp = `SUB;
                     end
                     3'b101: begin//sltu
                        aluOp = `SLTU;
                     end
                  endcase
               end//subu, sltu
               3'b100:begin
                  SgnExt = 1'b0;
                  aluOp = `AND;
               end//and
               3'b101:begin
                  SgnExt = 1'b0;
                  aluOp = `OR;
               end//or
               3'b110:begin
                  SgnExt = 1'b0;
                  aluOp = `XOR;
               end//xor
               3'b111:begin
                  SgnExt = 1'b0;
                  aluOp = `NOR;
               end//nor
            endcase
            nxt_state = RESET;
         end
         EX_ALU_I: begin
            aluSelB = 3'b010;
            aluSelA = 1'b1;
            MemtoReg = 2'b00;
            RegDst = 1'b0;
            RFwrt = 1'b1;
            nxt_state = RESET;
            case(IR[28:26])
               3'b000:begin
                  SgnExt = 1'b1;
                  aluOp = `ADD;
               end//addi
               3'b001:begin
                  SgnExt = 1'b0;
                  aluOp = `ADD;
               end//addiu
               3'b010:begin
                  SgnExt = 1'b1;
                  aluOp = `SLT;
               end//slti
               3'b011:begin
                  SgnExt = 1'b0;
                  aluOp = `SLTU;
               end//sltiu
               3'b100:begin
                  SgnExt = 1'b0;
                  aluOp = `AND;
               end//andi
               3'b101:begin
                  SgnExt = 1'b0;
                  aluOp = `OR;
               end//ori
               3'b110:begin
                  SgnExt = 1'b0;
                  aluOp = `XOR;
               end//xori
               3'b111:begin
                  SgnExt = 1'b0;
                  aluOp = `ADD;
               end//lui
            endcase
         end
         EX_LW_1: begin
            SgnExt = 1'b1;
            aluOp = `ADD;
            aluSelB = 3'b010;
            aluSelA = 1'b1;
            IorD = 1'b1;
            MARwrt = 1'b1;
            setMRE = 1'b1;
            nxt_state = EX_LW_2;
         end
         EX_LW_2: begin
            nxt_state = EX_LW_3;
         end
         EX_LW_3: begin
            nxt_state = EX_LW_4;
         end
         EX_LW_4: begin
            clrMRE = 1'b1;
            MDRwrt = 1'b1;
            nxt_state = EX_LW_5;
         end
         EX_LW_5: begin
            MemtoReg = 2'b01;
            RegDst = 1'b0;
            RFwrt = 1'b1;
            PrepareFetch;
         end
         EX_SW_1: begin
            SgnExt = 1'b1;
            IorD = 1'b1;
            aluOp = `ADD;
            MARwrt = 1'b1;
            aluSelB = 3'b010;
            aluSelA = 1'b1;
            setMWE = 1'b1;
            clrMWE = 1'b0;
            nxt_state = EX_SW_2;
         end
         EX_SW_2: begin
            IorD = 1'b0;
            MARwrt = 1'b1;
            IRwrt = 1'b1;
            clrMWE = 1'b1;
            nxt_state = EX_SW_3;
         end
         EX_SW_3: begin
            PrepareFetch;
         end
         EX_MULT_1: begin
            start = 1;
            nxt_state = EX_MULT_2;
         end
         EX_MULT_2: begin
            if(ready) nxt_state = RESET;
            else nxt_state = EX_MULT_2;
         end
         EX_BRA_1: begin
            SgnExt = 1'b1;
            aluOp = `SUB;
            aluSelB = 3'b000;
            aluSelA = 1'b1;
            if((aluZero && !IR[26]) || (!aluZero && IR[26])) nxt_state = EX_BRA_2;
            else PrepareFetch;
         end
         EX_BRA_2: begin
            SgnExt = 1'b1;
            aluSelA = 1'b0;
            aluSelB = 3'b011;
            aluOp = `ADD;
            IorD = 1'b1;
            IRwrt = 1'b1;
            PCwrt = 1'b1;
            MARwrt = 1'b1;
            setMRE = 1'b1;
            nxt_state = FETCH1;
         end
         default: nxt_state = RESET;
      endcase

   end

endmodule

//==============================================================================

module my_alu(
   input wire  [2:0] Op,
   input wire  [31:0] A,
   input wire  [31:0] B,

   output wire [31:0] X,
   output wire        Z
);

   wire sub = Op != `ADD;

   wire [31:0] bb = sub ? ~B : B;

   wire [32:0] sum = A + bb + sub;

   wire sltu = ! sum[32];

   wire v = sub ? 
        ( A[31] != B[31] && A[31] != sum[31] )
      : ( A[31] == B[31] && A[31] != sum[31] );

   wire slt = v ^ sum[31];

   reg [31:0] x;

   always @( * )
      case( Op )
         `ADD : x = sum;
         `SUB : x = sum;
         `SLT : x = slt;
         `SLTU: x = sltu;
         `AND : x =   A & B;
         `OR  : x =   A | B;
         `NOR : x = ~(A | B);
         `XOR : x =   A ^ B;
         default : x = 32'hxxxxxxxx;
      endcase

   assign #2 X = x;
   assign #2 Z = x == 32'h00000000;

endmodule

//==============================================================================

module reg_file(
   input  wire clk,
   input  wire write,
   input  wire [4:0] WR,
   input  wire [31:0] WD,
   input  wire [4:0] RR1,
   input  wire [4:0] RR2,
   output wire [31:0] RD1,
   output wire [31:0] RD2
);

   reg [31:0] rf_data [0:31];

   assign #2 RD1 = rf_data[ RR1 ];
   assign #2 RD2 = rf_data[ RR2 ];   

   always @( posedge clk ) begin
      if ( write )
         rf_data[ WR ] <= WD;

      rf_data[0] <= 32'h00000000;
   end

endmodule

//==============================================================================
