module part1 #(parameter n = 4) (input clk, input [1:0] FunSel, input [n-1:0] data_in, input enable, output reg [n-1:0] data_out);

    wire [n-1:0] zero = 0;
    always @(posedge clk)
    begin
        if (enable == 0)
            data_out = data_out;
        else
            case (FunSel)
                2'b00: data_out = zero;
                2'b01: data_out = data_in;
                2'b10: data_out = data_out - 1;
                2'b11: data_out = data_out + 1;
            endcase
    end
endmodule

module part2a_IRreg(input clk, input[7:0] I, input [1:0] FunSel, input LH, input enable, output reg [15:0] data_out);

    always @(posedge clk)
    begin
        if (enable == 0)
            data_out = data_out;
        else
            if(FunSel == 2'b00) data_out = 16'b000000000000000;
            else if(FunSel == 2'b01) begin
                if(LH == 0) data_out[7:0] = I;
                else if(LH == 1) data_out[15:8] = I;
            end
            else if(FunSel == 2'b10) data_out = data_out - 1;
            else if(FunSel == 2'b11) data_out = data_out + 1;
    end

endmodule

module part2b_RF (
    input clk,
    input [7:0] I,
    input [2:0] O1Sel,
    input [2:0] O2Sel,
    input [1:0] FunSel,
    input [3:0] RSel,
    input [3:0] TSel,
    output reg [7:0] O1,
    O2
);

    reg [7:0] R1;
    reg [7:0] R2;
    reg [7:0] R3;
    reg [7:0] R4;
    
    reg [7:0] T1;
    reg [7:0] T2;
    reg [7:0] T3;
    reg [7:0] T4;
 
    always @ (posedge clk) begin

        if(FunSel == 2'b00) begin
            if(RSel[0] == 1) 
                 R4 = 8'b00000000;
            if(RSel[1] == 1) 
                 R3 = 8'b00000000;
            if(RSel[2] == 1) 
                 R2 = 8'b00000000;
            if(RSel[3] == 1) 
                 R1 = 8'b00000000;
            if(TSel[0] == 1) 
                 T4 = 8'b00000000;
            if(TSel[1] == 1) 
                 T3 = 8'b00000000;
            if(TSel[2] == 1) 
                 T2 = 8'b00000000;
            if(TSel[3] == 1) 
                 T1 = 8'b00000000;
        end
        else if(FunSel == 2'b01) begin
            if(RSel[0])  R4 = I;
            if(RSel[1])  R3 = I;
            if(RSel[2])  R2 = I;
            if(RSel[3])  R1 = I;
            if(TSel[0])  T4 = I;
            if(TSel[1])  T3 = I;
            if(TSel[2])  T2 = I;
            if(TSel[3])  T1 = I;
        end
        else if(FunSel == 2'b10) begin
            if(RSel[0])  R4 = R4 - 1;
            if(RSel[1])  R3 = R3 - 1;
            if(RSel[2])  R2 = R2 - 1;
            if(RSel[3])  R1 = R1 - 1;
            if(TSel[0])  T4 = T4 - 1;
            if(TSel[1])  T3 = T3 - 1;
            if(TSel[2])  T2 = T2 - 1;
            if(TSel[3])  T1 = T1 - 1;
        end
        else if(FunSel == 2'b11) begin
            if(RSel[0])  R4 = R4 + 1;
            if(RSel[1])  R3 = R3 + 1;
            if(RSel[2])  R2 = R2 + 1;
            if(RSel[3])  R1 = R1 + 1;
            if(TSel[0])  T4 = T4 + 1;
            if(TSel[1])  T3 = T3 + 1;
            if(TSel[2])  T2 = T2 + 1;
            if(TSel[3])  T1 = T1 + 1;
        end
                
        if( O1Sel == 3'b000)  O1 = T1;
        else if( O1Sel == 3'b001)  O1 = T2;
        else if( O1Sel == 3'b010)  O1 = T3;
        else if( O1Sel == 3'b011)  O1 = T4;
        else if( O1Sel == 3'b100)  O1 = R1;
        else if( O1Sel == 3'b101)  O1 = R2;
        else if( O1Sel == 3'b110)  O1 = R3;
        else if( O1Sel == 3'b111)  O1 = R4;


        if( O2Sel == 3'b000)  O2 = T1;
        else if( O2Sel == 3'b001)  O2 = T2;
        else if( O2Sel == 3'b010)  O2 = T3;
        else if( O2Sel == 3'b011)  O2 = T4;
        else if( O2Sel == 3'b100)  O2 = R1;
        else if( O2Sel == 3'b101)  O2 = R2;
        else if( O2Sel == 3'b110)  O2 = R3;
        else if( O2Sel == 3'b111)  O2 = R4;

    end 
endmodule


//Address Register File
module part2c_ARF(
    input clk,
    input [7:0] I,
    input [1:0] OutASel,
    input [1:0] OutBSel,
    input [1:0] FunSel,
    input [3:0] RSel,
    output reg [7:0] OutA,
    output reg [7:0] OutB
);

reg [7:0] PC;
reg [7:0] AR;
reg [7:0] SP;
reg [7:0] PCPast;


always @ (posedge clk) begin

    if(FunSel == 2'b00)
    begin
        if(RSel[3] == 1) 
            PC = 8'b00000000;
        if(RSel[2] == 1) 
            AR = 8'b00000000;
        if(RSel[1] == 1) 
            SP = 8'b00000000;
        if(RSel[0] == 1) 
            PCPast = 8'b00000000;
    end
    else if(FunSel == 2'b01)
    begin
        if(RSel[3] == 1) 
            PC = I;
        if(RSel[2] == 1) 
            AR = I;
        if(RSel[1] == 1) 
            SP = I;
        if(RSel[0] == 1)
            PCPast = I;
    end
    else if(FunSel == 2'b10)
    begin
        if(RSel[3] == 1) 
            PC = PC + 1;
        if(RSel[2] == 1) 
            AR = AR + 1;
        if(RSel[1] == 1) 
            SP = SP + 1;
        if(RSel[0] == 1)
            PCPast = PCPast + 1;
    end
    else if(FunSel == 2'b11)
    begin
        if(RSel[3] == 1) 
            PC = PC - 1;
        if(RSel[2] == 1) 
            AR = AR - 1;
        if(RSel[1] == 1) 
            SP = SP - 1;
        if(RSel[0] == 1)
            PCPast = PCPast - 1;
    end


    if(OutASel == 2'b00) OutA = AR;
    else if(OutASel == 2'b01) OutA = SP;
    else if(OutASel == 2'b10) OutA = PCPast;
    else if(OutASel == 2'b11) OutA = PC;

    if(OutBSel == 2'b00) OutB = AR;
    else if(OutBSel == 2'b01) OutB = SP;
    else if(OutBSel == 2'b10) OutB = PCPast;
    else if(OutBSel == 2'b11) OutB = PC;

end

endmodule


module part3_ALU (input clk, input [7:0] A, input [7:0] B, input [3:0] FunSel, output reg [7:0] OutALU, output reg [3:0] Flags);

    reg [7:0] B_neg;
    reg cout;

    always @(posedge clk) begin
        B_neg = (~B) + 8'b00000001; // 2's complement of B
        cout = Flags[2];

        if(FunSel == 4'b0000)
            OutALU = A;
        else if(FunSel == 4'b0001)
            OutALU = B;
        else if(FunSel == 4'b0010)
            OutALU = ~A;
        else if(FunSel == 4'b0011)
            OutALU = ~B;
        else if(FunSel == 4'b0100) begin // A+B
            {cout, OutALU} = {1'b0, A} + {1'b0, B};
            if(cout == 1) Flags[0] = 1;
            else Flags[0] = 0;
        end
        else if(FunSel == 4'b0101)begin
            {cout, OutALU} = {1'b0, A} + {1'b0, B_neg};
            if(cout !== OutALU[7]) Flags[0] = 1; //Overflow
            else Flags[0] = 0;
        end
        else if(FunSel == 4'b0110)
            begin
                if(A > B) OutALU = A;
                else OutALU = 0;
            end
        else if(FunSel == 4'b0111)
            OutALU = A & B;
        else if(FunSel == 4'b1000)
            OutALU = A | B;
        else if(FunSel == 4'b1001)
            OutALU = ~(A & B);
        else if(FunSel == 4'b1010)
            OutALU = (~A & B) | (A & ~B);
        else if(FunSel == 4'b1011) begin // LSL
            cout = A[7];
            OutALU = A << 1;
        end
        else if (FunSel == 4'b1100) begin //LSR
            cout = A[0];
            OutALU = A >> 1;
        end
        else if (FunSel == 4'b1101) //ASL
            OutALU = A << 1;
        else if (FunSel == 4'b1110)
            OutALU = {A[7], A[7:1]}; 
        else if (FunSel == 4'b1111) begin //CSR
            cout = A[0];
            OutALU = {Flags[2], A[7:1]};
        end

        // Set flags

        if (OutALU == 8'b00000000) Flags[3] = 1; // Z Flag
        else Flags[3] = 0;

        Flags[2] = cout; // C Flag

        if (OutALU[7] == 1) Flags[1] = 1; // N Flag
        else Flags[1] = 0;

    end
endmodule

module Memory(
    input wire clock,
    input wire[7:0] address,
    input wire[7:0] data,
    input wire wr, //Read = 0, Write = 1
    input wire cs, //Chip is enable when cs = 0
    output reg[7:0] o // Output
);
    //Declaration of the RAM Area
    reg[7:0] RAM_DATA[0:255];
    //Read Ram data from the file
    initial $readmemh("RAM.mem", RAM_DATA);
    //Read the selected data from RAM
    always @(*) begin
        o = ~wr && ~cs ? RAM_DATA[address] : 8'hZ;
    end
    
    //Write the data to RAM
    always @(posedge clock) begin
        if (wr && ~cs) begin
            RAM_DATA[address] = data; 
        end
    end
endmodule

module mux_2to1(
    input clk,
    input sel,
    input [7:0] in0,
    input [7:0] in1,
    output reg [7:0] out
);

    always @(*) begin
        case(sel)
            1'b0: out = in0;
            1'b1: out = in1;
        endcase
    end

endmodule

module mux_4to1(
    input clk,
    input [1:0] sel,
    input [7:0] in0,
    input [7:0] in1,
    input [7:0] in2,
    input [7:0] in3,
    output reg [7:0] out
);

    always @(*) begin
        case(sel)
            2'b00: out = in0;
            2'b01: out = in1;
            2'b10: out = in2;
            2'b11: out = in3;
        endcase
    end

endmodule

module ALUSystem(
input[2:0] RF_O1Sel, 
input[2:0] RF_O2Sel, 
input[1:0] RF_FunSel,
input[3:0] RF_RSel,
input[3:0] RF_TSel,
input[3:0] ALU_FunSel,
input[1:0] ARF_OutASel, 
input[1:0] ARF_OutBSel,
input[1:0] ARF_FunSel,
input[3:0] ARF_RSel,
input      IR_LH,
input      IR_Enable,
input[1:0] IR_FunSel,
input      Mem_WR,
input      Mem_CS,
input[1:0] MuxASel,
input[1:0] MuxBSel,
input      MuxCSel,
input      Clock
);

    wire [7:0] MemOut;
    wire [7:0] RF_O1,RF_O2;
    wire [7:0] MuxAOut, MuxBOut, MuxCOut;
    wire [3:0] ALU_FlagOut;
    wire [7:0] ALU_Out;
    wire [7:0] ARF_OutA, ARF_OutB;
    wire [15:0] IR_Out;

    wire [7:0] IR_Out_L;

    part2b_RF RF(Clock, MuxAOut, RF_O1Sel, RF_O2Sel, RF_FunSel, RF_RSel, RF_TSel, RF_O1, RF_O2);
    
    part2c_ARF ARF(Clock, MuxBOut, ARF_OutASel, ARF_OutBSel, ARF_FunSel, ARF_RSel, ARF_OutA, ARF_OutB);

    Memory Mem(Clock, ARF_OutB, ALU_Out, Mem_WR, Mem_CS, MemOut);

    part2a_IRreg IR(Clock, MemOut, IR_FunSel, IR_LH, IR_Enable, IR_Out);

    assign IR_Out_L = IR_Out[7:0];

    mux_4to1 MuxA(Clock, MuxASel, ALU_Out, MemOut, IR_Out_L, ARF_OutA, MuxAOut);

    mux_4to1 MuxB(Clock, MuxBSel, ALU_Out, MemOut, IR_Out_L, ARF_OutA, MuxBOut);

    mux_2to1 MuxC(Clock, MuxCSel, RF_O1, ARF_OutA, MuxCOut);

    part3_ALU ALU(Clock, MuxCOut, RF_O2, ALU_FunSel, ALU_Out, ALU_FlagOut);


endmodule


module ControlUnit(input Clock);

//Input Registers of ALUSystem
    reg[1:0] RF_O1Sel; 
    reg[1:0] RF_O2Sel; 
    reg[1:0] RF_FunSel;
    reg[3:0] RF_RSel;
    reg[3:0] RF_TSel;
    reg[3:0] ALU_FunSel;
    reg[1:0] ARF_OutASel; 
    reg[1:0] ARF_OutBSel; 
    reg[1:0] ARF_FunSel;
    reg[2:0] ARF_RSel;
    reg      IR_LH;
    reg      IR_Enable;
    reg[1:0] IR_Funsel;
    reg      Mem_WR;
    reg      Mem_CS;
    reg[1:0] MuxASel;
    reg[1:0] MuxBSel;
    reg      MuxCSel;
    reg RST;
         
    ALUSystem ALU1(RF_O1Sel, 
         RF_O2Sel,
         RF_FunSel,
         RF_RSel,
         RF_TSel,
         ALU_FunSel,
         ARF_OutASel, 
         ARF_OutBSel, 
         ARF_FunSel,
         ARF_RSel,
         IR_LH,
         IR_Enable,
         IR_Funsel,
         Mem_WR,
         Mem_CS,
         MuxASel,
         MuxBSel,
         MuxCSel,
         Clock);
          
reg [1:0] REGSEL;
reg [3:0] OPCODE;
reg [3:0] CHECK;
reg ADDRESSINGMODE;
reg [3:0] SRCREG1;
reg [3:0] SRCREG2;
reg [3:0] DESTREG;
reg OPCODELOADED;


reg [3:0] T;

initial 
begin
OPCODELOADED = 0;
CHECK = 4'b1111;
RST = 1;
T= 4'b0000;
end

/*
T = 0 ==> LOAD LSB
T = 1 ==> LOAD MSB
T= 2,3,4... ==> OPERATIONS
*/

always @(posedge Clock)
begin
$display ("----");
if (CHECK == 4'b0000 || CHECK == 4'b1111) T = 4'b0000;
else T = T+1;
end

always @(negedge Clock)
begin

if (CHECK == 4'b1111) //
begin
$display("SIFIRLA");
ARF_RSel= 3'b000;
ARF_FunSel= 2'b11;
ARF_OutBSel= 2'b00;

RF_RSel= 4'b0000;
RF_FunSel= 2'b11;
MuxASel= 2'b00;
MuxBSel= 2'b00;
MuxCSel= 1'b0;
Mem_CS = 1;
IR_Enable = 1;
IR_Funsel = 2'b11;
CHECK = 4'b0000;
RST = 0;
end

else if (CHECK == 4'b0000 && RST) CHECK = 4'b1111;

else if (CHECK == 4'b0000 && RST == 0) // LSB LOAD TO IR
begin
RST = 0;
$display("LSB YÜKLE");
Mem_CS = 0;
Mem_WR = 0;
ARF_FunSel = 2'b01;
ARF_RSel = 3'b011;
ARF_OutBSel = 2'b00;
RF_RSel= 4'b1111;
IR_Enable = 1;
IR_LH = 1;
IR_Funsel = 2'b10;
CHECK = 4'b0001;
end

else if (CHECK == 4'b0001) // MSB LOAD TO IR
    begin //1
    $display("MSB YÜKLE");
    $display("IR OUT: %h",ALU1.IROut);
    RF_RSel= 4'b1111;
    ARF_FunSel = 2'b01;
    ARF_RSel = 3'b011;
    ARF_OutBSel = 2'b00;
    IR_Enable = 1;
    IR_LH = 0;
    IR_Funsel = 2'b10;
    CHECK = 4'b1110;
    end//1

else if(CHECK == 4'b1110)
begin//13
$display("OPCODE AL");
IR_Enable =1'b0;
OPCODE = ALU1.IROut [15:12];
OPCODELOADED = 1;
ADDRESSINGMODE = ALU1.IROut [10];
$display("IR OUT: %h",ALU1.IROut);
if ((OPCODE == 4'b1001|| OPCODE == 4'b1010 || OPCODE == 4'b1100 || OPCODE== 4'b1101) && OPCODELOADED == 1'b1) 
begin //2
$display("N/A OLMAYAN");
    if (ALU1.IROut [10] == 0 && OPCODE != 4'b1101 ) // If addressing mode is direct and the operation is not ST.
        begin
        CHECK = 4'b0010;
        end
    
    else// If addressing mode is immediate.
        begin
        CHECK = 4'b0011;
        end
end

end//2   

if ((OPCODE == 4'b1001|| OPCODE == 4'b1010 || OPCODE == 4'b1100 || OPCODE== 4'b1101) && OPCODELOADED == 1)
    begin //3
    
    $display("immediate");
    IR_Enable =1'b0;// TO PREVENT IR CHANGE, IR IS DISABLED.
    case (OPCODE)
        4'b1001:
        begin
        
        $display("OPCODE 0");
        MuxBSel = 2'b01;
        ARF_FunSel = 2'b10;
        ARF_RSel = 3'b011;
        RF_RSel = 4'b1111;
        Mem_WR = 1'b0;
        CHECK = 4'b0000;
        OPCODELOADED = 0;
        end
        
        4'b1010: // OPCODE 1 LOAD
        begin
        
        if (CHECK == 4'b0010) //direct to immediate
            begin
            $display("direct to immediate: LSB TO AR");
            ARF_RSel = 3'b111;
            ARF_OutBSel = 2'b10;
            IR_LH  =1'b1;
            IR_Enable =1'b1;
            IR_Funsel = 2'b10;
            RF_RSel = 4'b1111;
            CHECK = 4'b0011;
            end
            
        else if (CHECK == 4'b0011)
            begin
                $display("OPCODE 1");
                ARF_RSel =  3'b111; 
        
                MuxASel = 2'b00;
                RF_FunSel = 2'b10;
                RF_O1Sel = ALU1.IROut [9:8];
                
                case(ALU1.IROut [9:8])
                  2'b00: RF_RSel = 4'b0111;
                  2'b01: RF_RSel = 4'b1011;
                  2'b10: RF_RSel = 4'b1101;
                  2'b11: RF_RSel = 4'b1110;
                endcase 
                
                OPCODELOADED = 0;    
                CHECK = 4'b0000; 
            end
        
        end
        
        4'b1100:
        begin
        
        $display("OPCODE 2");
        if (CHECK == 4'b0011)
        
            begin 
            $display ("0011'e girdi.");
            Mem_WR = 1'b0;
            ARF_RSel = 3'b101;
            ARF_FunSel = 2'b10;
            MuxBSel = 2'b01;
            RF_RSel = 4'b1111;
            Mem_CS = 0;
            RF_O2Sel = ALU1.IROut [9:8];
            ALU_FunSel = 4'b0001;
            CHECK = 4'b0100;
            end
        
        else if (CHECK == 4'b0100)
            begin
            RF_RSel = 4'b1111;
            Mem_WR = 1'b1;
            ARF_RSel = 3'b111;
            ARF_OutBSel = 2'b10;
            RF_RSel = 3'b111;
            RF_O2Sel = ALU1.IROut [9:8];
            ALU_FunSel = 4'b0001;
            OPCODELOADED = 0;
            CHECK = 4'b0000; 
            end 
        end
        
        4'b1101:
        begin
        
        
        RF_RSel = 4'b1111;//
        Mem_WR = 1'b0;//       
        $display ("Z FLAG: %d",ALU1.ALUOutFlag[3]);
        if (ALU1.ALUOutFlag[3]== 1'b0)
        begin
            MuxBSel = 2'b01;
            ARF_FunSel = 2'b10;
            ARF_RSel = 3'b011;
        end
        else ARF_RSel = 3'b111;
        OPCODELOADED = 0;
        CHECK = 4'b0000; 
        end 
        endcase
    end//3

if((OPCODE != 4'b1001 && OPCODE != 4'b1010 && OPCODE != 4'b1100 && OPCODE != 4'b1101) && OPCODELOADED == 1)
    begin //4 n/a bloku
    
    $display("N/A BLOKU");
    $display("IR OUT: %h",ALU1.IROut);
    IR_Enable =1'b0;// TO PREVENT IR CHANGE, IR IS DISABLED.
    SRCREG2 = ALU1.IROut[3:0];
    SRCREG1 = ALU1.IROut[7:4];
    DESTREG = ALU1.IROut[11:8];
    
        case (OPCODE)
        4'b1011: // OPCODE 12 C
        begin
        $display("OPCODE 3");
          Mem_WR = 1'b0;
          
          if(SRCREG1[2] == 1 && DESTREG[2] == 1) begin // RF TO RF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b0001;
              MuxASel = 2'b11;
              RF_FunSel = 2'b10;
              ARF_RSel = 3'b111;
              
              case(DESTREG[1:0])
              2'b00: RF_RSel = 4'b0111;
              2'b01: RF_RSel = 4'b1011;
              2'b10: RF_RSel = 4'b1101;
              2'b11: RF_RSel = 4'b1110;
              endcase 
          end
          
          else if(SRCREG1[2] == 1 && DESTREG[2] == 0) begin // RF TO ARF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b0001;
              MuxBSel = 2'b11;
              ARF_FunSel = 2'b10;
              RF_RSel = 4'b1111;
              
              case(DESTREG[1:0])
              2'b00: ARF_RSel = 3'b011;
              2'b01: ARF_RSel = 3'b011;
              2'b10: ARF_RSel = 3'b101;
              2'b11: ARF_RSel = 3'b110;
              endcase
          end
          
          else if(SRCREG1[2] == 0 && DESTREG[2] == 1) begin // ARF TO RF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b0000;
               MuxASel = 2'b11;
               RF_FunSel = 2'b10;
               ARF_RSel = 3'b111;
               
               case(DESTREG[1:0])
               2'b00: RF_RSel = 4'b0111;
               2'b01: RF_RSel = 4'b1011;
               2'b10: RF_RSel = 4'b1101;
               2'b11: RF_RSel = 4'b1110;
               endcase  
          end
            
          else if(SRCREG1[2] == 0 && DESTREG[2] == 0) begin // ARF TO ARF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b0000;
               MuxBSel = 2'b11;
               RF_RSel = 4'b1111;
               ARF_FunSel = 2'b10;
               
               case(DESTREG[1:0])
               2'b00: ARF_RSel = 3'b011;
               2'b01: ARF_RSel = 3'b011;
               2'b10: ARF_RSel = 3'b101;
               2'b11: ARF_RSel = 3'b110;
               endcase   
          end  
          OPCODELOADED = 0;
          CHECK = 4'b0000;
        end
        
        4'b0000: // OPCODE 0
        begin
        
        Mem_WR = 1'b0;
        
            if(SRCREG1[2] == 1 && SRCREG2[2] == 1 && DESTREG[2] == 1) begin // RF AND RF -> RF
                RF_O1Sel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b1;
                ALU_FunSel = 4'b0111;
                MuxASel = 2'b11;
                RF_FunSel = 2'b10;
                ARF_RSel = 3'b111;
                
                case(DESTREG[1:0])
                2'b00: RF_RSel = 4'b0111;
                2'b01: RF_RSel = 4'b1011;
                2'b10: RF_RSel = 4'b1101;
                2'b11: RF_RSel = 4'b1110;
                endcase
            end
            
            else if(SRCREG1[2] == 1 && SRCREG2[2] == 1 && DESTREG[2] == 0) begin // RF AND RF -> ARF
                RF_O1Sel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b1;
                ALU_FunSel = 4'b0111;
                MuxBSel = 2'b11;
                RF_RSel = 4'b1111;
                ARF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: ARF_RSel = 3'b011;
                2'b01: ARF_RSel = 3'b011;
                2'b10: ARF_RSel = 3'b101;
                2'b11: ARF_RSel = 3'b110;
                endcase  
            end
            
            else if(SRCREG1[2] == 0 && SRCREG2[2] == 1 && DESTREG[2] == 1) begin // ARF AND RF ->  RF
                ARF_OutASel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b0;
                ALU_FunSel = 4'b0111;
                MuxASel = 2'b11;
                ARF_RSel = 3'b111;
                RF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: RF_RSel = 4'b0111;
                2'b01: RF_RSel = 4'b1011;
                2'b10: RF_RSel = 4'b1101;
                2'b11: RF_RSel = 4'b1110;
                endcase    
            end
            
            else if(SRCREG1[2] == 0 && SRCREG2[2] == 1 && DESTREG[2] == 0) begin // ARF AND RF -> ARF
                ARF_OutASel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b0;
                ALU_FunSel = 4'b0111;
                MuxBSel = 2'b11;
                RF_RSel = 4'b1111;
                ARF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: ARF_RSel = 3'b011;
                2'b01: ARF_RSel = 3'b011;
                2'b10: ARF_RSel = 3'b101;
                2'b11: ARF_RSel = 3'b110;
                endcase   
            end
            OPCODELOADED = 0;
            CHECK = 4'b0000;
        end
        
        4'b0001: // OPCODE 1
        begin
        Mem_WR = 1'b0;
       
    
            if(SRCREG1[2] == 1 && SRCREG2[2] == 1 && DESTREG[2] == 1) begin // RF AND RF -> RF
                RF_O1Sel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b1;
                ALU_FunSel = 4'b1000;
                MuxASel = 2'b11;
                RF_FunSel = 2'b10;
                ARF_RSel = 3'b111;
                
                case(DESTREG[1:0])
                2'b00: RF_RSel = 4'b0111;
                2'b01: RF_RSel = 4'b1011;
                2'b10: RF_RSel = 4'b1101;
                2'b11: RF_RSel = 4'b1110;
                endcase
            end
            
            else if(SRCREG1[2] == 1 && SRCREG2[2] == 1 && DESTREG[2] == 0) begin // RF AND RF -> ARF
                RF_O1Sel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b1;
                ALU_FunSel = 4'b1000;
                MuxBSel = 2'b11;
                RF_RSel = 4'b1111;
                ARF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: ARF_RSel = 3'b011;
                2'b01: ARF_RSel = 3'b011;
                2'b10: ARF_RSel = 3'b101;
                2'b11: ARF_RSel = 3'b110;
                endcase  
            end
            
            else if(SRCREG1[2] == 0 && SRCREG2[2] == 1 && DESTREG[2] == 1) begin // ARF AND RF ->  RF
                ARF_OutASel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b0;
                ALU_FunSel = 4'b1000;
                MuxASel = 2'b11;
                ARF_RSel = 3'b111;
                RF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: RF_RSel = 4'b0111;
                2'b01: RF_RSel = 4'b1011;
                2'b10: RF_RSel = 4'b1101;
                2'b11: RF_RSel = 4'b1110;
                endcase    
            end
            
            else if(SRCREG1[2] == 0 && SRCREG2[2] == 1 && DESTREG[2] == 0) begin // ARF AND RF -> ARF
                ARF_OutASel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b0;
                ALU_FunSel = 4'b1000;
                MuxBSel = 2'b11;
                RF_RSel = 4'b1111;
                ARF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: ARF_RSel = 3'b011;
                2'b01: ARF_RSel = 3'b011;
                2'b10: ARF_RSel = 3'b101;
                2'b11: ARF_RSel = 3'b110;
                endcase   
            end
            OPCODELOADED = 0;
            CHECK = 4'b0000;
        end // END OF OPCODE 5
        
        4'b0010: // OPCODE 2
        begin
          Mem_WR = 1'b0;
           
          if(SRCREG1[2] == 1 && DESTREG[2] == 1) begin // RF TO RF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b0011;
              MuxASel = 2'b11;
              RF_FunSel = 2'b10;
              ARF_RSel = 3'b111;
              
              case(DESTREG[1:0])
              2'b00: RF_RSel = 4'b0111;
              2'b01: RF_RSel = 4'b1011;
              2'b10: RF_RSel = 4'b1101;
              2'b11: RF_RSel = 4'b1110;
              endcase 
          end
          
          else if(SRCREG1[2] == 1 && DESTREG[2] == 0) begin // RF TO ARF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b0011;
              MuxBSel = 2'b11;
              ARF_FunSel = 2'b10;
              RF_RSel = 4'b1111;
              
              case(DESTREG[1:0])
              2'b00: ARF_RSel = 3'b011;
              2'b01: ARF_RSel = 3'b011;
              2'b10: ARF_RSel = 3'b101;
              2'b11: ARF_RSel = 3'b110;
              endcase
          end
          
          else if(SRCREG1[2] == 0 && DESTREG[2] == 1) begin // ARF TO RF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b0010;
               MuxASel = 2'b11;
               RF_FunSel = 2'b10;
               ARF_RSel = 3'b111;
               
               case(DESTREG[1:0])
               2'b00: RF_RSel = 4'b0111;
               2'b01: RF_RSel = 4'b1011;
               2'b10: RF_RSel = 4'b1101;
               2'b11: RF_RSel = 4'b1110;
               endcase  
          end
            
          else if(SRCREG1[2] == 0 && DESTREG[2] == 0) begin // ARF TO ARF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b0010;
               MuxBSel = 2'b11;
               RF_RSel = 4'b1111;
               ARF_FunSel = 2'b10;
               
               case(DESTREG[1:0])
               2'b00: ARF_RSel = 3'b011;
               2'b01: ARF_RSel = 3'b011;
               2'b10: ARF_RSel = 3'b101;
               2'b11: ARF_RSel = 3'b110;
               endcase   
          end  
          OPCODELOADED = 0;
          CHECK = 4'b0000;
        end // END OF OPCODE 6
        
        4'b0101: // OPCODE 5
        begin
        
          Mem_WR = 1'b0;
           
          if(SRCREG1[2] == 1 && DESTREG[2] == 1) begin // RF TO RF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b1011;
              MuxASel = 2'b11;
              RF_FunSel = 2'b10;
              ARF_RSel = 3'b111;
              
              case(DESTREG[1:0])
              2'b00: RF_RSel = 4'b0111;
              2'b01: RF_RSel = 4'b1011;
              2'b10: RF_RSel = 4'b1101;
              2'b11: RF_RSel = 4'b1110;
              endcase 
          end
          
          else if(SRCREG1[2] == 1 && DESTREG[2] == 0) begin // RF TO ARF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b1011;
              MuxBSel = 2'b11;
              ARF_FunSel = 2'b10;
              RF_RSel = 4'b1111;
              
              case(DESTREG[1:0])
              2'b00: ARF_RSel = 3'b011;
              2'b01: ARF_RSel = 3'b011;
              2'b10: ARF_RSel = 3'b101;
              2'b11: ARF_RSel = 3'b110;
              endcase
          end
          
          else if(SRCREG1[2] == 0 && DESTREG[2] == 1) begin // ARF TO RF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b1011;
               MuxASel = 2'b11;
               RF_FunSel = 2'b10;
               ARF_RSel = 3'b111;
               
               case(DESTREG[1:0])
               2'b00: RF_RSel = 4'b0111;
               2'b01: RF_RSel = 4'b1011;
               2'b10: RF_RSel = 4'b1101;
               2'b11: RF_RSel = 4'b1110;
               endcase  
          end
            
          else if(SRCREG1[2] == 0 && DESTREG[2] == 0) begin // ARF TO ARF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b1011;
               MuxBSel = 2'b11;
               RF_RSel = 4'b1111;
               ARF_FunSel = 2'b10;
               
               case(DESTREG[1:0])
               2'b00: ARF_RSel = 3'b011;
               2'b01: ARF_RSel = 3'b011;
               2'b10: ARF_RSel = 3'b101;
               2'b11: ARF_RSel = 3'b110;
               endcase   
          end  
          OPCODELOADED = 0;
          CHECK = 4'b0000;
        end // END OF OPCODE 9
    
        4'b0110: // OPCODE 6
        begin
        
          Mem_WR = 1'b0;
           
          if(SRCREG1[2] == 1 && DESTREG[2] == 1) begin // RF TO RF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b1010;
              MuxASel = 2'b11;
              RF_FunSel = 2'b10;
              ARF_RSel = 3'b111;
              
              case(DESTREG[1:0])
              2'b00: RF_RSel = 4'b0111;
              2'b01: RF_RSel = 4'b1011;
              2'b10: RF_RSel = 4'b1101;
              2'b11: RF_RSel = 4'b1110;
              endcase 
          end
          
          else if(SRCREG1[2] == 1 && DESTREG[2] == 0) begin // RF TO ARF
              RF_O2Sel = SRCREG1[1:0];
              ALU_FunSel = 4'b1010;
              MuxBSel = 2'b11;
              ARF_FunSel = 2'b10;
              RF_RSel = 4'b1111;
              
              case(DESTREG[1:0])
              2'b00: ARF_RSel = 3'b011;
              2'b01: ARF_RSel = 3'b011;
              2'b10: ARF_RSel = 3'b101;
              2'b11: ARF_RSel = 3'b110;
              endcase
          end
          
          else if(SRCREG1[2] == 0 && DESTREG[2] == 1) begin // ARF TO RF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b1010;
               MuxASel = 2'b11;
               RF_FunSel = 2'b10;
               ARF_RSel = 3'b111;
               
               case(DESTREG[1:0])
               2'b00: RF_RSel = 4'b0111;
               2'b01: RF_RSel = 4'b1011;
               2'b10: RF_RSel = 4'b1101;
               2'b11: RF_RSel = 4'b1110;
               endcase  
          end
            
          else if(SRCREG1[2] == 0 && DESTREG[2] == 0) begin // ARF TO ARF
               ARF_OutASel = SRCREG1[1:0]; 
               MuxCSel = 1'b0;
               ALU_FunSel = 4'b1010;
               MuxBSel = 2'b11;
               RF_RSel = 4'b1111;
               ARF_FunSel = 2'b10;
               
               case(DESTREG[1:0])
               2'b00: ARF_RSel = 3'b011;
               2'b01: ARF_RSel = 3'b011;
               2'b10: ARF_RSel = 3'b101;
               2'b11: ARF_RSel = 3'b110;
               endcase   
          end  
          CHECK = 4'b0000;
          OPCODELOADED = 0;
        end // END OF OPCODE 10
    
        4'b0011: // OPCODE 3
        begin
        
        Mem_WR = 1'b0;
    
            if(SRCREG1[2] == 1 && SRCREG2[2] == 1 && DESTREG[2] == 1) begin // RF + RF -> RF
                RF_O1Sel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b1;
                ALU_FunSel = 4'b0100;
                MuxASel = 2'b11;
                RF_FunSel = 2'b10;
                ARF_RSel = 3'b111;
                $display ("ALU OUT AFTER SUMMATION : %d",ALU1.ALUOut);
                case(DESTREG[1:0])
                2'b00: RF_RSel = 4'b0111;
                2'b01: RF_RSel = 4'b1011;
                2'b10: RF_RSel = 4'b1101;
                2'b11: RF_RSel = 4'b1110;
                endcase
            end
            
            else if(SRCREG1[2] == 1 && SRCREG2[2] == 1 && DESTREG[2] == 0) begin // RF + RF -> ARF
                RF_O1Sel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b1;
                ALU_FunSel = 4'b0100;
                MuxBSel = 2'b11;
                RF_RSel = 4'b1111;
                ARF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: ARF_RSel = 3'b011;
                2'b01: ARF_RSel = 3'b011;
                2'b10: ARF_RSel = 3'b101;
                2'b11: ARF_RSel = 3'b110;
                endcase  
            end
            
            else if(SRCREG1[2] == 0 && SRCREG2[2] == 1 && DESTREG[2] == 1) begin // ARF + RF ->  RF
                ARF_OutASel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b0;
                ALU_FunSel = 4'b0100;
                MuxASel = 2'b11;
                ARF_RSel = 3'b111;
                RF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: RF_RSel = 4'b0111;
                2'b01: RF_RSel = 4'b1011;
                2'b10: RF_RSel = 4'b1101;
                2'b11: RF_RSel = 4'b1110;
                endcase    
            end
            
            else if(SRCREG1[2] == 0 && SRCREG2[2] == 1 && DESTREG[2] == 0) begin // ARF + RF -> ARF
                ARF_OutASel = SRCREG1[1:0];
                RF_O2Sel = SRCREG2[1:0];
                MuxCSel = 1'b0;
                ALU_FunSel = 4'b0100;
                MuxBSel = 2'b11;
                RF_RSel = 4'b1111;
                ARF_FunSel = 2'b10;
                
                case(DESTREG[1:0])
                2'b00: ARF_RSel = 3'b011;
                2'b01: ARF_RSel = 3'b011;
                2'b10: ARF_RSel = 3'b101;
                2'b11: ARF_RSel = 3'b110;
                endcase   
            end
            OPCODELOADED = 0;
            CHECK = 4'b0000;
        end // END OF OPCODE 7
        
        4'b0100: begin// OPCODE 4
            Mem_WR = 1'b0;
            
            if (CHECK == 4'b1110)
            begin
                  Mem_WR = 1'b0;
                  
                  if(SRCREG1[2] == 1 && DESTREG[2] == 1) begin // RF TO RF
                      RF_O2Sel = SRCREG1[1:0];
                      ALU_FunSel = 4'b0011;
                      MuxASel = 2'b11;
                      RF_FunSel = 2'b10;
                      ARF_RSel = 3'b111;
                      
                      case(DESTREG[1:0])
                      2'b00: RF_RSel = 4'b0111;
                      2'b01: RF_RSel = 4'b1011;
                      2'b10: RF_RSel = 4'b1101;
                      2'b11: RF_RSel = 4'b1110;
                      endcase 
                  end
                  
                  else if(SRCREG1[2] == 1 && DESTREG[2] == 0) begin // RF TO ARF
                      RF_O2Sel = SRCREG1[1:0];
                      ALU_FunSel = 4'b0011;
                      MuxBSel = 2'b11;
                      ARF_FunSel = 2'b10;
                      RF_RSel = 4'b1111;
                      
                      case(DESTREG[1:0])
                      2'b00: ARF_RSel = 3'b011;
                      2'b01: ARF_RSel = 3'b011;
                      2'b10: ARF_RSel = 3'b101;
                      2'b11: ARF_RSel = 3'b110;
                      endcase
                  end
                  
                  else if(SRCREG1[2] == 0 && DESTREG[2] == 1) begin // ARF TO RF
                       ARF_OutASel = SRCREG1[1:0]; 
                       MuxCSel = 1'b0;
                       ALU_FunSel = 4'b0010;
                       MuxASel = 2'b11;
                       RF_FunSel = 2'b10;
                       ARF_RSel = 3'b111;
                       
                       case(DESTREG[1:0])
                       2'b00: RF_RSel = 4'b0111;
                       2'b01: RF_RSel = 4'b1011;
                       2'b10: RF_RSel = 4'b1101;
                       2'b11: RF_RSel = 4'b1110;
                       endcase  
                  end
                  
                  else if(SRCREG1[2] == 0 && DESTREG[2] == 0) begin // ARF TO ARF
                       ARF_OutASel = SRCREG1[1:0]; 
                       MuxCSel = 1'b0;
                       ALU_FunSel = 4'b0010;
                       MuxBSel = 2'b11;
                       RF_RSel = 4'b1111;
                       ARF_FunSel = 2'b10;
                       
                       case(DESTREG[1:0])
                       2'b00: ARF_RSel = 3'b011;
                       2'b01: ARF_RSel = 3'b011;
                       2'b10: ARF_RSel = 3'b101;
                       2'b11: ARF_RSel = 3'b110;
                       endcase   
                  end
                  CHECK = 4'b0100;
                end
                
                else if (CHECK == 4'b0100)
                begin 
                    if(DESTREG[2] == 0)begin// ARF
                        ARF_FunSel = 2'b01;
                        
                        case(DESTREG[1:0])
                        2'b00: ARF_RSel = 3'b011;
                        2'b01: ARF_RSel = 3'b011;
                        2'b10: ARF_RSel = 3'b101;
                        2'b11: ARF_RSel = 3'b110;
                        endcase
                        
                        RF_RSel = 4'b1111;
                    end
                    
                    if(DESTREG[2] == 1)begin// RF
                        RF_FunSel = 2'b01;
                        
                        case(DESTREG[1:0])
                        2'b00: RF_RSel = 4'b0111;
                        2'b01: RF_RSel = 4'b1011;
                        2'b10: RF_RSel = 4'b1101;
                        2'b11: RF_RSel = 4'b1110;
                        endcase 
                        
                        ARF_RSel = 3'b111;
                    end
                    CHECK = 4'b0101;
                    
                end
                
                else if (CHECK == 4'b0101)
                   begin
                        RF_O2Sel = SRCREG2[1:0];
                        
                        if(DESTREG[2] == 0)begin// ARF
                            ARF_OutASel = DESTREG[1:0];
                            ARF_FunSel = 2'b10;
                            MuxCSel = 1'b0;
                            ALU_FunSel = 4'b0100;
                            MuxBSel = 2'b11;
                            
                            case(DESTREG[1:0])
                            2'b00: ARF_RSel = 3'b011;
                            2'b01: ARF_RSel = 3'b011;
                            2'b10: ARF_RSel = 3'b101;
                            2'b11: ARF_RSel = 3'b110;
                            endcase 
                           
                           RF_RSel = 4'b1111;
                        end
                        
                        if(DESTREG[2] == 1)begin // RF
                            MuxASel = 2'b11;
                            ALU_FunSel = 4'b0100;
                            RF_O1Sel = DESTREG[1:0];
                            ARF_RSel = 3'b111;
                            RF_FunSel = 2'b10;
                            MuxCSel = 1'b1;
                            case(DESTREG[1:0])
                            2'b00: RF_RSel = 4'b0111;
                            2'b01: RF_RSel = 4'b1011;
                            2'b10: RF_RSel = 4'b1101;
                            2'b11: RF_RSel = 4'b1110;
                            endcase 
                        end
                   OPCODELOADED = 0;  
                   CHECK = 4'b0000;
                   end
                 
            end // END OF OPCODE 8
            
            
        4'b0111: begin// OPCODE 7 
            Mem_WR = 1'b0;
            
            if (CHECK == 4'b1110)
            begin               
                  if(SRCREG1[2] == 1 && DESTREG[2] == 1) begin // RF TO RF
                      RF_O2Sel = SRCREG1[1:0];
                      ALU_FunSel = 4'b0001;
                      MuxASel = 2'b11;
                      RF_FunSel = 2'b10;
                      ARF_RSel = 3'b111;
                      
                      case(DESTREG[1:0])
                      2'b00: RF_RSel = 4'b0111;
                      2'b01: RF_RSel = 4'b1011;
                      2'b10: RF_RSel = 4'b1101;
                      2'b11: RF_RSel = 4'b1110;
                      endcase 
                  end
                  
                  else if(SRCREG1[2] == 1 && DESTREG[2] == 0) begin // RF TO ARF
                      RF_O2Sel = SRCREG1[1:0];
                      ALU_FunSel = 4'b0001;
                      MuxBSel = 2'b11;
                      ARF_FunSel = 2'b10;
                      RF_RSel = 4'b1111;
                      
                      case(DESTREG[1:0])
                      2'b00: ARF_RSel = 3'b011;
                      2'b01: ARF_RSel = 3'b011;
                      2'b10: ARF_RSel = 3'b101;
                      2'b11: ARF_RSel = 3'b110;
                      endcase
                  end
                  
                  else if(SRCREG1[2] == 0 && DESTREG[2] == 1) begin // ARF TO RF
                       ARF_OutASel = SRCREG1[1:0]; 
                       MuxCSel = 1'b0;
                       ALU_FunSel = 4'b0000;
                       MuxASel = 2'b11;
                       RF_FunSel = 2'b10;
                       ARF_RSel = 3'b111;
                       
                       case(DESTREG[1:0])
                       2'b00: RF_RSel = 4'b0111;
                       2'b01: RF_RSel = 4'b1011;
                       2'b10: RF_RSel = 4'b1101;
                       2'b11: RF_RSel = 4'b1110;
                       endcase  
                  end
                    
                  else if(SRCREG1[2] == 0 && DESTREG[2] == 0) begin // ARF TO ARF
                       ARF_OutASel = SRCREG1[1:0]; 
                       MuxCSel = 1'b0;
                       ALU_FunSel = 4'b0000;
                       MuxBSel = 2'b11;
                       RF_RSel = 4'b1111;
                       ARF_FunSel = 2'b10;
                       
                       case(DESTREG[1:0])
                       2'b00: ARF_RSel = 3'b011;
                       2'b01: ARF_RSel = 3'b011;
                       2'b10: ARF_RSel = 3'b101;
                       2'b11: ARF_RSel = 3'b110;
                       endcase   
                  end  
                CHECK = 4'b0100;
            end    
                
                
             else if(CHECK == 4'b0100) begin
             
                    if(DESTREG[2] == 0)begin// ARF
                       ARF_FunSel = 2'b01;         
                       case(DESTREG[1:0])
                       2'b00: ARF_RSel = 3'b011;
                       2'b01: ARF_RSel = 3'b011;
                       2'b10: ARF_RSel = 3'b101;
                       2'b11: ARF_RSel = 3'b110;
                       endcase         
                        RF_RSel = 4'b1111;
                       end
                             
                       if(DESTREG[2] == 1)begin// RF
                            RF_FunSel = 2'b01;    
                           case(DESTREG[1:0])
                           2'b00: RF_RSel = 4'b0111;
                           2'b01: RF_RSel = 4'b1011;
                           2'b10: RF_RSel = 4'b1101;
                           2'b11: RF_RSel = 4'b1110;
                           endcase      
                            ARF_RSel = 3'b111;
                        end
                        
                        CHECK = 4'b0101;
             end   
             
             else if(CHECK == 4'b0101)begin
                   ARF_RSel = 3'b111;
                   RF_RSel = 4'b1111;
                   
                  if(DESTREG[2] == 0)begin// ARF
                    ARF_OutASel = DESTREG[1:0];
                    MuxCSel = 1'b0;
                    ALU_FunSel= 4'b0000;
                  end
                  
                  else if(DESTREG[2] == 1)begin// RF
                      RF_O2Sel = DESTREG[1:0];
                      ALU_FunSel= 4'b0001;
                               
                  end
                OPCODELOADED = 0;
                CHECK = 4'b0000;
             end
             
           end
    
        4'b1000: begin// OPCODE 8
            ;
            Mem_WR = 1'b0;
            
            if (CHECK == 4'b1110)
            begin               
                          if(SRCREG1[2] == 1 && DESTREG[2] == 1) begin // RF TO RF
                              RF_O2Sel = SRCREG1[1:0];
                              ALU_FunSel = 4'b0001;
                              MuxASel = 2'b11;
                              RF_FunSel = 2'b10;
                              ARF_RSel = 3'b111;
                              
                              case(DESTREG[1:0])
                              2'b00: RF_RSel = 4'b0111;
                              2'b01: RF_RSel = 4'b1011;
                              2'b10: RF_RSel = 4'b1101;
                              2'b11: RF_RSel = 4'b1110;
                              endcase 
                          end
                          
                          else if(SRCREG1[2] == 1 && DESTREG[2] == 0) begin // RF TO ARF
                              RF_O2Sel = SRCREG1[1:0];
                              ALU_FunSel = 4'b0001;
                              MuxBSel = 2'b11;
                              ARF_FunSel = 2'b10;
                              RF_RSel = 4'b1111;
                              
                              case(DESTREG[1:0])
                              2'b00: ARF_RSel = 3'b011;
                              2'b01: ARF_RSel = 3'b011;
                              2'b10: ARF_RSel = 3'b101;
                              2'b11: ARF_RSel = 3'b110;
                              endcase
                          end
                          
                          else if(SRCREG1[2] == 0 && DESTREG[2] == 1) begin // ARF TO RF
                               ARF_OutASel = SRCREG1[1:0]; 
                               MuxCSel = 1'b0;
                               ALU_FunSel = 4'b0000;
                               MuxASel = 2'b11;
                               RF_FunSel = 2'b10;
                               ARF_RSel = 3'b111;
                               
                               case(DESTREG[1:0])
                               2'b00: RF_RSel = 4'b0111;
                               2'b01: RF_RSel = 4'b1011;
                               2'b10: RF_RSel = 4'b1101;
                               2'b11: RF_RSel = 4'b1110;
                               endcase  
                          end
                            
                          else if(SRCREG1[2] == 0 && DESTREG[2] == 0) begin // ARF TO ARF
                               ARF_OutASel = SRCREG1[1:0]; 
                               MuxCSel = 1'b0;
                               ALU_FunSel = 4'b0000;
                               MuxBSel = 2'b11;
                               RF_RSel = 4'b1111;
                               ARF_FunSel = 2'b10;
                               
                               case(DESTREG[1:0])
                               2'b00: ARF_RSel = 3'b011;
                               2'b01: ARF_RSel = 3'b011;
                               2'b10: ARF_RSel = 3'b101;
                               2'b11: ARF_RSel = 3'b110;
                               endcase   
                          end  
                        CHECK = 4'b0100;
                    end
                
                
                
             else if(CHECK == 4'b0100) begin        
                       if(DESTREG[2] == 0)begin// ARF
                                ARF_FunSel = 2'b00;          
                                case(DESTREG[1:0])
                                2'b00: ARF_RSel = 3'b011;
                                2'b01: ARF_RSel = 3'b011;
                                2'b10: ARF_RSel = 3'b101;
                                2'b11: ARF_RSel = 3'b110;
                                endcase         
                                 RF_RSel = 4'b1111;
                                end
                                      
                                if(DESTREG[2] == 1)begin// RF
                                     RF_FunSel = 2'b00;     
                                    case(DESTREG[1:0])
                                    2'b00: RF_RSel = 4'b0111;
                                    2'b01: RF_RSel = 4'b1011;
                                    2'b10: RF_RSel = 4'b1101;
                                    2'b11: RF_RSel = 4'b1110;
                                    endcase      
                                     ARF_RSel = 3'b111;
                                 end
                                 CHECK = 4'b0101;
             end   
             
             else if(CHECK == 4'b0101)begin
                   ARF_RSel = 3'b111;
                   RF_RSel = 4'b1111;
                   
                  if(DESTREG[2] == 0)begin// ARF
                    ARF_OutASel = DESTREG[1:0];
                    MuxCSel = 1'b0;
                    ALU_FunSel= 4'b0000;
                  end
                  
                  else if(DESTREG[2] == 1)begin// RF
                       
                      RF_O2Sel = DESTREG[1:0];
                      ALU_FunSel= 4'b0001;
                      $display ("BOUT: %d",ALU1.BOut);
                  end
                OPCODELOADED = 0;
                CHECK = 4'b0000;
             end
             
             
           end
            
        4'b1110: begin// OPCODE 15 E
            Mem_WR = 0;
           
            if (CHECK == 4'b1110)
            begin
                RF_RSel = 4'b1111;
                ARF_FunSel = 2'b01;
                ARF_RSel = 3'b110;
                CHECK = 4'b0100;
            end
            
            else if (CHECK == 4'b0100)
            begin
               ARF_OutBSel = 2'b11;
               
               if(DESTREG[2]==1'b0) begin
                   MuxBSel = 2'b10;
                   ARF_FunSel = 2'b10; 
                   
                   case(DESTREG[1:0])
                   2'b00: ARF_RSel = 3'b011;
                   2'b01: ARF_RSel = 3'b011;
                   2'b10: ARF_RSel = 3'b101;
                   2'b11: ARF_RSel = 3'b110;
                   endcase 
                   RF_RSel = 4'b1111;              
               end
               
               else if(DESTREG[2]==1'b1) begin
                    MuxASel = 2'b01;
                    RF_FunSel = 2'b10;
                    
                    case(DESTREG[1:0])
                    2'b00: RF_RSel = 4'b0111;
                    2'b01: RF_RSel = 4'b1011;
                    2'b10: RF_RSel = 4'b1101;
                    2'b11: RF_RSel = 4'b1110;
                    endcase
                    ARF_RSel = 3'b111;                     
               end
               OPCODELOADED = 0;
               CHECK = 4'b0000;
               
            end
            
            
            end
            
        4'b1111: begin// OPCODE 16 F
            
            if (CHECK == 4'b1110)
             begin
                CHECK = 4'b0100;
                ARF_RSel = 3'b111; 
                RF_RSel = 4'b1111;
                Mem_WR = 1;       
                ARF_OutBSel = 2'b11;     
                if(SRCREG1[2]==1'b0)begin
                    ARF_OutASel = SRCREG1[1:0]; 
                    MuxCSel = 1'b0;
                    ALU_FunSel = 4'b0000; 
                end
                
                else if(SRCREG1[2]==1'b1)begin
                    RF_O2Sel = SRCREG1[1:0];
                    ALU_FunSel = 4'b0001; 
                end            
                
            
             end
            else if (CHECK == 4'b0100)
              begin
              Mem_WR = 0;
              RF_RSel = 4'b1111;
              ARF_RSel = 3'b110; 
              ARF_FunSel = 2'b00;
              OPCODELOADED = 0;
              
              CHECK = 4'b0000;
              end         
             
            end
    
        endcase
    
    end //4 */
    


end//13

endmodule