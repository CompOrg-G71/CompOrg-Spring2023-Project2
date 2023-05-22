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
    input T0, T1, T2, T3, T4, T5, T6, T7,
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

    input [1:0] RSEL,
    input [3:0] DSTREG,
    input [3:0] SREG1,
    input [3:0] SREG2,
    input AddressMode,

    input Z_Flag, C_Flag, N_Flag, O_Flag,

    output wire SC_reset,

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


    // Register File
    reg [2:0] temp_RF_O1Sel;
    reg [2:0] temp_RF_O2Sel;
    reg [1:0] temp_RF_FunSel;
    reg [3:0] temp_RF_RSel;
    reg [3:0] temp_RF_TSel;

    //ALU
    reg [3:0] temp_ALU_FunSel;

    //Address Register File
    reg [1:0] temp_ARF_OutASel;
    reg [1:0] temp_ARF_OutBSel;
    reg [1:0] temp_ARF_FunSel;
    reg [3:0] temp_ARF_RSel;

    //Instruction Register
    reg       temp_IR_LH;
    reg       temp_IR_Enable;
    reg [1:0] temp_IR_FunSel;

    //Memory
    reg       temp_Mem_WR;
    reg       temp_Mem_CS;

    //Multiplexers
    reg [1:0] temp_MuxASel;
    reg [1:0] temp_MuxBSel;
    reg       temp_MuxCSel;

    reg temp_SC_reset = 1'b0;


    always@(*) begin 

        if(T7) begin 

            temp_IR_Enable = 1'b1;
            temp_IR_FunSel = 2'b00;

            temp_RF_RSel = 4'b1111;
            temp_RF_TSel = 4'b1111;
            temp_RF_FunSel = 2'b00;

            temp_ARF_RSel = 4'b1111;
            temp_ARF_FunSel = 2'b00;

            temp_Mem_CS = 1'b1;
        end


        //Load LSB of M[PC] to IR and increment PC by one
        else if(T0) begin
            temp_SeqCounter_Reset = 1'b0;

            temp_IR_LH = 1'b0;
            temp_IR_Enable = 1'b1;
            temp_IR_FunSel = 2'b01;

            temp_RF_RSel = 4'b0000;
            temp_RF_TSel = 4'b0000;

            temp_ARF_RSel = 4'b1000; // Selecting PC
            temp_ARF_FunSel = 2'b11; // Incrementing PC

            temp_Mem_CS = 1'b0;
            temp_Mem_WR = 1'b0;

        end

        else if(T1) begin 
            temp_SeqCounter_Reset = 1'b0;

            temp_IR_LH = 1'b1;
            temp_IR_Enable = 1'b1;
            temp_IR_FunSel = 2'b01;

            temp_RF_RSel = 4'b0000;
            temp_RF_TSel = 4'b0000;

            temp_ARF_RSel = 4'b1000; // Selecting PC
            temp_ARF_FunSel = 2'b11; // Incrementing PC

            temp_Mem_CS = 1'b0;
            temp_Mem_WR = 1'b0;
        end

        if(T2) begin // Conditions for second clock cycle

            //Constraints before moving on, this ensures that conditions for some operations are met before executing, if not, the sequence counter is reset.
            if((BRA & AddressMode) | (BNE & AddressMode) | (ST & ~AddressMode) | (BNE & Z_Flag))begin
                temp_SC_reset <= 1'b1;
                temp_RF_RSel <= 4'b0000;
                temp_ARF_RSel = 4'b0000;
                temp_IR_Enable = 1'b0;
                rMem_CS = 1'b1;
            end

            else if(~AddressMode & ((BRA) | (BNE & ~Z_Flag)))begin // Load LSB of IR to PC

                temp_MuxBSel = 2'b10;
                temp_ARF_RSel = 4'b1000;
                temp_ARF_FunSel = 2'b01;

                // Disable IR, Memory, and Register File
                temp_IR_Enable = 1'b0;
                temp_RF_RSel = 4'b0000;
                temp_Mem_CS = 1'b1;

                temp_SC_reset = 1'b1; //Counter reset is 1 because BRA instruction is finished in 1 clock cycle

            end

            else if(LD & AddressMode) begin //Direct Version of LD Operation

                temp_IR_Enable = 1'b0;

                temp_ARF_RSel = 4'b0000; // Disabling all register in ARF
                temp_ARF_OutBSel = 2'b00; // Selecting the AR Register as output to memory address

                temp_Mem_WR = 1'b0; // Reading from memory
                temp_Mem_CS = 1'b0; // Enabling memory

                temp_MuxASel = 2'b01; // Selecting Data from memory to input into RF

                temp_RF_FunSel = 2'b01; // Writing to RF

                temp_SC_reset = 1'b1; //Counter reset is 1 because LD instruction is finished in 1 clock cycle

                //Selecting the RF Register to load to based on REGSEL
                if(RSEL == 2'b00)      temp_RF_RSel <= 4'b1000;
                else if(RSEL == 2'b01) temp_RF_RSel <= 4'b0100;
                else if(RSEL == 2'b10) temp_RF_RSel <= 4'b0010;
                else if(RSEL == 2'b11) temp_RF_RSel <= 4'b0001;

            end

            else if(LD & ~AddressMode) begin // "Immediate" Version of LD Operation

                temp_Mem_CS = 1'b1; // Disabling memory
                temp_ARF_RSel = 4'b0000; // Disabling all register in ARF

                temp_MuxASel = 2'b10; // Selecting IR(7-0) as input to RF
                temp_IR_Enable = 1'b0; // no writing to IR

                temp_RF_FunSel = 2'b01; // Writing to RF

                temp_SC_reset = 1'b1; //Counter reset is 1 because LD instruction is finished in 1 clock cycle

                //Selecting the RF Register to load to based on REGSEL
                if(RSEL == 2'b00)      temp_RF_RSel <= 4'b1000;
                else if(RSEL == 2'b01) temp_RF_RSel <= 4'b0100;
                else if(RSEL == 2'b10) temp_RF_RSel <= 4'b0010;
                else if(RSEL == 2'b11) temp_RF_RSel <= 4'b0001;

            end

            else if(ST & AddressMode) begin //Write data in Register determined by REGSEL to M[AR] (Direct)

                temp_IR_Enable = 1'b0; // no writing to IR

                temp_RF_O2Sel = {1'b1, RSEL}; // Selecting the RF Register to write to memory based on RSEL
                temp_RF_RSel = 4'b0000; // Disabling RF

                temp_ALU_FunSel = 4'b0001; // OutALU is O2Sel into memory data input
                temp_ARF_OutBSel = 2'b00; // Selecting the AR Register as output to memory address input

                temp_ARF_RSel = 4'b0000; // Disabling all register in ARF

                temp_Mem_WR = 1'b1; // Writing to memory
                temp_Mem_CS = 1'b0; // Enabling memory

                temp_SC_reset = 1'b1; //Counter reset is 1 because ST instruction is finished in 1 clock cycle

            end

            else if(MOV|AND|OR|NOT|ADD|SUB|LSR|LSL|INC|DEC|PSH|PUL) begin // all of these operations have a common execution sequence

                temp_Mem_CS = 1'b1; // Disabling memory
                temp_IR_Enable = 1'b0; // no writing to IR

                temp_RF_RSel = 4'b0000; // Disabling all Registers in RF
                temp_ARF_RSel = 4'b0000; // Disabling all Registers in ARF


                // Determining the destination Register, since we disabled all register prior to this,
                // it will result in only one register being enabled and written to
                if(~DSTREG[2]) begin
                    case(DSTREG[1:0])
                        2'b00: temp_RF_RSel <= 4'b1000; // Enabling RF Register 1
                        2'b01: temp_RF_RSel <= 4'b0100; // Enabling RF Register 2
                        2'b10: temp_RF_RSel <= 4'b0010; // Enabling RF Register 3
                        2'b11: temp_RF_RSel <= 4'b0001; // Enabling RF Register 4
                    endcase
                end
                else if (DSTREG[2]) begin 
                    case(DSTREG[1:0])
                        2'b00: temp_ARF_RSel <= 4'b0010; // Enabling SR Register
                        2'b01: temp_ARF_RSel <= 4'b0100; // Enabling AR Register
                        2'b10: temp_ARF_RSel <= 4'b1000; // Enabling PC Register
                        2'b11: temp_ARF_RSel <= 4'b1000; // Enabling PC Register
                    endcase
                end

                //Determining the Source Register, 
                if(SREG1[2]) begin 
                    case(SREG1[1:0]) 
                        2'b00: temp_ARF_OutASel <= 2'b01;
                        2'b01: temp_ARF_OutASel <= 2'b00;
                        2'b10: temp_ARF_OutASel <= 2'b11;
                        2'b11: temp_ARF_OutASel <= 2'b11;
                    endcase
                end
                else if (~SREG1[2]) begin 
                    temp_RF_O1Sel = {1'b1, SREG1[1:0]};
                end

                // if(SREG2[2]) begin 
                //     case(SREG2[1:0]) 
                //         2'b00: temp_ARF_OutBSel <= 2'b01;
                //         2'b01: temp_ARF_OutBSel <= 2'b00;
                //         2'b10: temp_ARF_OutBSel <= 2'b11;
                //         2'b11: temp_ARF_OutBSel <= 2'b11;
                //     endcase
                // end
                // else 
                if (~SREG2[2]) begin 
                    temp_RF_O2Sel = {1'b1, SREG2[1:0]};
                end
                

                if( LSL | LSR | MOV | AND | OR | NOT | ADD | SUB ) begin // all of these operations finish in T2 and are assigned SREG1 and DSTREG in the same way.

                    if(LSL) temp_ALU_FunSel = 4'b1011; // LSL operation
                    else if(LSR) temp_ALU_FunSel = 4'b1100; // LSR operation
                    else if(MOV) temp_ALU_FunSel = 4'b0000; // Passing Output of MuxC Directly to OutALU
                    else if (AND) temp_ALU_FunSel = 4'b0111; // AND operation
                    else if (OR) temp_ALU_FunSel = 4'b1000; // OR operation
                    else if (NOT) temp_ALU_FunSel = 4'b0010; // NOT operation
                    else if (ADD) temp_ALU_FunSel = 4'b0100; // ADD operation 
                    else if (SUB) temp_ALU_FunSel = 4'b0101; // SUB operation


                    if(~SREG1[2]) begin 
                        temp_MuxCSel = 1'b0; // Selecting RF_O1 as input to ALU
                    end
                    else if (SREG1[2]) begin 
                        temp_MuxCSel = 1'b1; // Selecting ARF_OutA as input to ALU
                    end

                    if(~DSTREG[2]) begin 
                        temp_RF_FunSel = 2'b01; // Writing ALU output to RF
                        temp_MuxASel = 2'b00; // Selecting ALU output as input to RF
                    end
                    else if (DSTREG[2]) begin
                        temp_ARF_FunSel = 2'b01; // Writing ALU output to ARF
                        temp_MuxBSel = 2'b00; // Selecting ALU output as input to ARF
                    end

                    temp_SC_reset = 1'b1; //Counter reset is 1 because all of these instructions are finished in 1 clock cycle

                end


                else if (INC | DEC) begin 
                    
                    if(SREG1 == DSTREG) begin // Can be done in 1 clock cycle
                        if(~SREG1[2]) begin 
                            
                            if(INC) temp_RF_FunSel = 2'b11; // Increment Operation on RF
                            else if(DEC) temp_RF_FunSel = 2'b10; // Decrement Operation on RF

                        end
                        else if(SREG1[2]) begin 

                            if(INC) temp_ARF_FunSel = 2'b11; // Increment Operation on RF
                            else if(DEC) temp_ARF_FunSel = 2'b10; // Decrement Operation on RF

                        end

                        temp_SC_reset = 1'b1;
                    end

                    else begin  // if destination and source are different, we need to first load the data into the destination, then increment from there.

                        temp_ALU_FunSel = 4'b0000; // ADD operation
                        
                        if(~SREG1[2]) begin 
                            temp_MuxCSel = 1'b0; // Selecting RF_O1 as input to ALU
                        end
                        else if (SREG1[2]) begin 
                            temp_MuxCSel = 1'b1; // Selecting ARF_OutA as input to ALU
                        end
                        
                        if(~DSTREG[2]) begin 
                            temp_RF_FunSel = 2'b01; // Writing ALU output to RF
                            temp_MuxASel = 2'b00; // Selecting ALU output as input to RF
                        end
                        else if (DSTREG[2]) begin
                            temp_ARF_FunSel = 2'b01; // Writing ALU output to ARF
                            temp_MuxBSel = 2'b00; // Selecting ALU output as input to ARF
                        end

                        temp_SC_reset = 1'b0; // Counter reset is 0 because these instructions take 2 clock cycles
                    end

                end

                if (PSH & ~SREG1[2]) begin // PSH at T2, Write to memory first

                    temp_ALU_FunSel = 4'b0000; // PASS A to OutALU

                    temp_MuxCSel = 1'b0; // Selecting RF_O1 as input to ALU

                    temp_IR_Enable = 1'b0 // Disable IR

                    temp_ARF_RSel = 4'b0000; // Disabling all ARF registers

                    temp_ARF_OutBSel = 2'b01; // Selecting SP as input to Memory Address

                    temp_Mem_WR = 1'b1; // Write to memory
                    temp_Mem_CS = 1'b0; // Enable memory

                    temp_SC_reset = 1'b0; // Counter is not reset because this instruction takes 2 clock cycles
                end

                if(PUL & ~DSTREG[2]) begin // INCREMENT SP

                    temp_ARF_RSel = 4'b0010; // Enabling SP register

                    temp_ARF_FunSel = 2'b11; // Increment Operation on ARF

                    temp_IR_Enable = 1'b0 // Disable IR
                    
                    temp_SC_reset = 1'b0; // Counter is not reset because this instruction takes 2 clock cycles
                end
                    
            end

        end

        else if (T3) begin 

            // Determining the destination Register, since we disabled all register prior to this,
            // it will result in only one register being enabled and written to
            if(~DSTREG[2]) begin
                case(DSTREG[1:0])
                    2'b00: temp_RF_RSel <= 4'b1000; // Enabling RF Register 1
                    2'b01: temp_RF_RSel <= 4'b0100; // Enabling RF Register 2
                    2'b10: temp_RF_RSel <= 4'b0010; // Enabling RF Register 3
                    2'b11: temp_RF_RSel <= 4'b0001; // Enabling RF Register 4
                endcase
            end
            else if (DSTREG[2]) begin 
                case(DSTREG[1:0])
                    2'b00: temp_ARF_RSel <= 4'b0010; // Enabling SR Register
                    2'b01: temp_ARF_RSel <= 4'b0100; // Enabling AR Register
                    2'b10: temp_ARF_RSel <= 4'b1000; // Enabling PC Register
                    2'b11: temp_ARF_RSel <= 4'b1000; // Enabling PC Register
                endcase
            end

            //Determining the Source Register, 
            if(SREG1[2]) begin 
                case(SREG1[1:0]) 
                    2'b00: temp_ARF_OutASel <= 2'b01;
                    2'b01: temp_ARF_OutASel <= 2'b00;
                    2'b10: temp_ARF_OutASel <= 2'b11;
                    2'b11: temp_ARF_OutASel <= 2'b11;
                endcase
            end
            else if (~SREG1[2]) begin 
                temp_RF_O1Sel = {1'b1, SREG1[1:0]};
            end

            // if(SREG2[2]) begin 
            //     case(SREG2[1:0]) 
            //         2'b00: temp_ARF_OutBSel <= 2'b01;
            //         2'b01: temp_ARF_OutBSel <= 2'b00;
            //         2'b10: temp_ARF_OutBSel <= 2'b11;
            //         2'b11: temp_ARF_OutBSel <= 2'b11;
            //     endcase
            // end
            // else 
            if (~SREG2[2]) begin 
                temp_RF_O2Sel = {1'b1, SREG2[1:0]};
            end
            
            if(INC | DEC) begin 
                if(~DSTREG[2]) begin 
                    if(INC) temp_RF_FunSel = 2'b11; // Increment Operation on RF
                    else if(DEC) temp_RF_FunSel = 2'b10; // Decrement Operation on RF
                end
                else if (DSTREG[2]) begin 
                    if(INC) temp_ARF_FunSel = 2'b11; // Increment Operation on ARF
                    else if(DEC) temp_ARF_FunSel = 2'b10; // Decrement Operation on ARF
                end

                temp_SC_reset = 1'b1; // Counter reset
            end

            if(PSH & ~SREG1[2]) begin // Decrement SP and Reset Counter

                temp_ARF_RSel = 4'b0010; // Enabling SP Register in ARF

                temp_ARF_FunSel = 2'b10; // Decrement Operation on ARF

                temp_SC_reset = 1'b1; // Counter reset
            end

            if(PUL & ~DSTREG[2]) begin // Load M[SP] to Rx

                temp_ARF_OutBSel = 4'b0010; // Selecting SP as input to Memory Address

                temp_Mem_CS = 1'b0; // Enable memory
                temp_Mem_WR = 1'b0; // Read from memory

                temp_MuxASel = 1'b1; // Selecting Memory Output as input to ALU

                temp_RF_FunSel = 2'b01; // Load Operation on RF

                temp_SC_reset = 1'b1; // Counter reset
            end

        end


    end


    assign RF_O1Sel = temp_RF_O1Sel;
    assign RF_O2Sel = temp_RF_O2Sel;
    assign RF_FunSel = temp_RF_FunSel;
    assign RF_RSel = temp_RF_RSel;
    assign RF_TSel = temp_RF_TSel;
    assign ALU_FunSel = temp_ALU_FunSel;
    assign ARF_OutASel = temp_ARF_OutASel;
    assign ARF_OutBSel = temp_ARF_OutBSel;
    assign ARF_FunSel = temp_ARF_FunSel;
    assign ARF_RSel = temp_ARF_RSel;
    assign IR_LH = temp_IR_LH;
    assign IR_Enable = temp_IR_Enable;
    assign IR_FunSel = temp_IR_FunSel;
    assign Mem_WR = temp_Mem_WR;
    assign Mem_CS = temp_Mem_CS;
    assign MuxASel = temp_MuxASel;
    assign MuxBSel = temp_MuxBSel;
    assign MuxCSel = temp_MuxCSel;

    assign SC_reset = temp_SC_reset;

endmodule

module CPUSystem(
    input Clock,
    input Reset,
    input [7:0] T_dec,
);


    wire [7:0] RF_O1Sel;
    wire [7:0] RF_O2Sel;
    wire [1:0] RF_FunSel;
    wire [3:0] RF_RSel;
    wire [3:0] RF_TSel;
    wire [3:0] ALU_FunSel;
    wire [7:0] ARF_OutASel;
    wire [7:0] ARF_OutBSel;
    wire [1:0] ARF_FunSel;
    wire [3:0] ARF_RSel;
    wire IR_LH;
    wire IR_Enable;
    wire [1:0] IR_FunSel;
    wire Mem_WR;
    wire Mem_CS;
    wire [1:0] MuxASel;
    wire [1:0] MuxBSel;
    wire MuxCSel;

    wire AND;
    wire OR;
    wire NOT;
    wire ADD;
    wire SUB;
    wire LSR;
    wire LSL;
    wire INC;
    wire DEC;
    wire BRA;
    wire BNE;
    wire MOV;
    wire LD;
    wire ST;
    wire PUL;
    wire PSH;

    wire [3:0] T_binary;
    

    ALUSystem ALU_Sys(
        .RF_O1Sel(RF_O1Sel), 
        .RF_O2Sel(RF_O2Sel), 
        .RF_FunSel(RF_FunSel),
        .RF_RSel(RF_RSel),
        .RF_TSel(RF_TSel),
        .ALU_FunSel(ALU_FunSel),
        .ARF_OutASel(ARF_OutASel), 
        .ARF_OutBSel(ARF_OutBSel),
        .ARF_FunSel(ARF_FunSel),
        .ARF_RSel(ARF_RSel),
        .IR_LH(IR_LH),
        .IR_Enable(IR_Enable),
        .IR_FunSel(IR_FunSel),
        .Mem_WR(Mem_WR),
        .Mem_CS(Mem_CS),
        .MuxASel(MuxASel),
        .MuxBSel(MuxBSel),
        .MuxCSel(MuxCSel),
        .Clock(Clock)
    );

    ControlUnit Ctrl_Unit(
        T_dec[0], T_dec[1], T_dec[2], T_dec[3], T_dec[4], T_dec[5], T_dec[6], T_dec[7],
        AND,
        OR,
        NOT,
        ADD,
        SUB,
        LSR,
        LSL,
        INC,
        DEC,
        BRA,
        BNE,
        MOV,
        LD,
        ST,
        PUL,
        PSH,

        .RSEL(ALU_Sys.IR_Out[9:8]),
        .DSTREG(ALU_Sys.IR_Out[11:8]),
        .SREG1(ALU_Sys.IR_Out[7:4]),
        .SREG2(ALU_Sys.IR_Out[3:0]),
        .AddressMode(ALU_Sys.IR_Out[10]),

        ALU_Sys.ALU_FlagOut[3], ALU_Sys.ALU_FlagOut[2], ALU_Sys.ALU_FlagOut[1], ALU_Sys.ALU_FlagOut[0],

        Reset,

        RF_O1Sel, 
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
        IR_FunSel,
        Mem_WR,
        Mem_CS,
        MuxASel,
        MuxBSel,
        MuxCSel
    );

    Dec_16_1 IR_Decoder(ALU_Sys.IR_Out[15:12],
        AND,
        OR,
        NOT,
        ADD,
        SUB,
        LSR,
        LSL,
        INC,
        DEC,
        BRA,
        BNE,
        MOV,
        LD,
        ST,
        PUL,
        PSH
    );

    SeqCounter SC(
        Clock,
        Reset,
        T_binary
    );

    Dec_8_1 SC_Decoder(T_binary, T_dec[7], T_dec[6], T_dec[5], T_dec[4], T_dec[3], T_dec[2], T_dec[1], T_dec[0]);

endmodule