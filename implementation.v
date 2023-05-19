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
            {cout, OutALU} = {1'b0, A} + {1'b0, B_neg}; // TODO: check if this is correct
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
    
    part2c_ARF ARF(Clock, MuxBOut, ARF_OutASel, ARF_OutBSel, ARF_FunSel, ARF_RSel, ARF_OutA, ARF_OutB);

    mux_4to1 MuxA(Clock, MuxASel, ALU_Out, MemOut, IR_Out[7:0], ARF_OutA, MuxAOut);

    mux_4to1 MuxB(Clock, MuxBSel, ALU_Out, MemOut, IR_Out[7:0], ARF_OutA, MuxBOut);

    part2b_RF RF(Clock, MuxAOut, RF_O1Sel, RF_O2Sel, RF_FunSel, RF_RSel, RF_TSel, RF_O1, RF_O2);

    mux_2to1 MuxC(Clock, MuxCSel, RF_O1, ARF_OutA, MuxCOut);

    part3_ALU ALU(Clock, MuxCOut, RF_O2, ALU_FunSel, ALU_Out, ALU_FlagOut);

    Memory Mem(Clock, ARF_OutB, ALU_Out, Mem_WR, Mem_CS, MemOut);

    part2a_IRreg IR(Clock, MemOut, IR_FunSel, IR_LH, IR_Enable, IR_Out);


endmodule

module SeqCounter(clk,Reset,T);
    input clk;
    input Reset;
    output [2:0] T;
    
    reg [2:0] counter = 3'd7; //Start counter from T7. Because at the beginning we will reset all registers
    
    always@(posedge clk)begin
            counter = counter + 3'd1;
            if(Reset == 1'b1)begin //Reset counter to T0
                counter = 3'd0;
            end
    end
    assign T = counter;
endmodule

module Dec_8_1(
    input [2:0] s,
    output S0,
    output S1,
    output S2,
    output S3,
    output S4,
    output S5,
    output S6,
    output S7
    );
    assign S0=(~s[0]&~s[1]&~s[2]),
    S1=(~s[2]&~s[1]&s[0]),
    S2=(~s[2]&s[1]&~s[0]),
    S3=(~s[2]&s[1]&s[0]),
    S4=(s[2]&~s[1]&~s[0]),
    S5=(s[2]&~s[1]&s[0]),
    S6=(s[2]&s[1]&~s[0]),
    S7=(s[2]&s[1]&s[0]);
endmodule

module Dec_16_1(
    input [3:0] s,
    output S0,
    output S1,
    output S2,
    output S3,
    output S4,
    output S5,
    output S6,
    output S7,
    output S8,
    output S9,
    output S10,
    output S11,
    output S12,
    output S13,
    output S14,
    output S15
    );
    assign S0=(~s[3]&~s[2]&~s[1]&~s[0]),
    S1=(~s[3]&~s[2]&~s[1]&s[0]),
    S2=(~s[3]&~s[2]&s[1]&~s[0]),
    S3=(~s[3]&~s[2]&s[1]&s[0]),
    S4=(~s[3]&s[2]&~s[1]&~s[0]),
    S5=(~s[3]&s[2]&~s[1]&s[0]),
    S6=(~s[3]&s[2]&s[1]&~s[0]),
    S7=(~s[3]&s[2]&s[1]&s[0]),
    S8=(s[3]&~s[2]&~s[1]&~s[0]),
    S9=(s[3]&~s[2]&~s[1]&s[0]),
    S10=(s[3]&~s[2]&s[1]&~s[0]),
    S11=(s[3]&~s[2]&s[1]&s[0]),
    S12=(s[3]&s[2]&~s[1]&~s[0]),
    S13=(s[3]&s[2]&~s[1]&s[0]),
    S14=(s[3]&s[2]&s[1]&~s[0]),
    S15=(s[3]&s[2]&s[1]&s[0]);
endmodule



module ControlUnit(
    input [7:0] T,
    input AND,
    input OR,
    input NOT,
    input ADD,
    input SUB,
    input LSR,
    input LSL,
    input INC,
    input DEC,
    input BRA,
    input BNE,
    input MOV,
    input LD,
    input ST,
    input PUL,
    input PSH,

    input [1:0] RSel,
    input [3:0] DSTREG,
    input [3:0] SREG1,
    input [3:0] SREG2,
    input AdrsMode,

    input Z, C, N, O,


    output wire SeqCounter_Reset,

    output wire [2:0] RF_O1Sel, 
    output wire [2:0] RF_O2Sel, 
    output wire [1:0] RF_FunSel,
    output wire [3:0] RF_RSel,
    output wire [3:0] RF_TSel,
    output wire [3:0] ALU_FunSel,
    output wire [1:0] ARF_OutASel, 
    output wire [1:0] ARF_OutBSel,
    output wire [1:0] ARF_FunSel,
    output wire [3:0] ARF_RSel,
    output wire       IR_LH,
    output wire       IR_Enable,
    output wire [1:0] IR_FunSel,
    output wire       Mem_WR,
    output wire       Mem_CS,
    output wire [1:0] MuxASel,
    output wire [1:0] MuxBSel,
    output wire       MuxCSel
);



    reg [2:0] temp_RF_O1Sel;
    reg [2:0] temp_RF_O2Sel;
    reg [1:0] temp_RF_FunSel;
    reg [3:0] temp_RF_RSel;
    reg [3:0] temp_RF_TSel;
    reg [3:0] temp_ALU_FunSel;
    reg [1:0] temp_ARF_OutASel;
    reg [1:0] temp_ARF_OutBSel;
    reg [1:0] temp_ARF_FunSel;
    reg [3:0] temp_ARF_RSel;
    reg       temp_IR_LH;
    reg       temp_IR_Enable;
    reg [1:0] temp_IR_FunSel;
    reg       temp_Mem_WR;
    reg       temp_Mem_CS;
    reg [1:0] temp_MuxASel;
    reg [1:0] temp_MuxBSel;
    reg       temp_MuxCSel;

    reg temP_SeqCounter_Reset = 1'b0;





endmodule