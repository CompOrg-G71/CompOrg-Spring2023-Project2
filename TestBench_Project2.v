`timescale 1ns / 1ps
`include "implementation.v"
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 04/05/2023 01:16:48 AM
// Design Name: 
// Module Name: TestBench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: 
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////
// module Project1Test();
//     //Input Registers of ALUSystem
//     reg[2:0] RF_O1Sel; 
//     reg[2:0] RF_O2Sel; 
//     reg[1:0] RF_FunSel;
//     reg[3:0] RF_RSel;
//     reg[3:0] RF_TSel;
//     reg[3:0] ALU_FunSel;
//     reg[1:0] ARF_OutASel; 
//     reg[1:0] ARF_OutBSel; 
//     reg[1:0] ARF_FunSel;
//     reg[3:0] ARF_RSel;
//     reg      IR_LH;
//     reg      IR_Enable;
//     reg[1:0]      IR_Funsel;
//     reg      Mem_WR;
//     reg      Mem_CS;
//     reg[1:0] MuxASel;
//     reg[1:0] MuxBSel;
//     reg MuxCSel;
//     reg      Clock;
    
//     //Test Bench Connection of ALU System
//     ALUSystem _ALUSystem(
//         .RF_O1Sel(RF_O1Sel), 
//         .RF_O2Sel(RF_O2Sel), 
//         .RF_FunSel(RF_FunSel),
//         .RF_RSel(RF_RSel),
//         .RF_TSel(RF_TSel),
//         .ALU_FunSel(ALU_FunSel),
//         .ARF_OutASel(ARF_OutASel), 
//         .ARF_OutBSel(ARF_OutBSel), 
//         .ARF_FunSel(ARF_FunSel),
//         .ARF_RegSel(ARF_RSel),
//         .IR_LH(IR_LH),
//         .IR_Enable(IR_Enable),
//         .IR_Funsel(IR_Funsel),
//         .Mem_WR(Mem_WR),
//         .Mem_CS(Mem_CS),
//         .MuxASel(MuxASel),
//         .MuxBSel(MuxBSel),
//         .MuxCSel(MuxCSel),
//         .Clock(Clock)
//         );
    
//     //Test Vector Variables
//     reg [41:0] VectorNum, Errors, TotalLine; 
//     reg [41:0] TestVectors[3:0];
//     reg Reset, Operation;
//     initial begin
//         Reset = 0;
//     end
//     //Clock Signal Generation
//     always 
//     begin
//         Clock = 1; #5; Clock = 0; #5; // 10ns period
//     end
    
//     //Read Test Bench Values
//     initial begin
//         $readmemb("TestBench.mem", TestVectors); // Read vectors
//         VectorNum = 0; Errors = 0; TotalLine=0; Reset=0;// Initialize
//     end
    
//     // Apply test vectors on rising edge of clock
//     always @(posedge Clock)
//     begin
//         #1; 
//         {Operation, RF_O1Sel, RF_O2Sel, RF_FunSel, 
//         RF_RSel, RF_TSel, ALU_FunSel, ARF_OutASel, ARF_OutBSel, 
//         ARF_FunSel, ARF_RSel, IR_LH, IR_Enable, IR_Funsel, 
//         Mem_WR, Mem_CS, MuxASel, MuxBSel, MuxCSel} = TestVectors[VectorNum];
//     end
    
//     // Check results on falling edge of clk
//     always @(negedge Clock)
//         if (~Reset) // skip during reset
//         begin
//             $display("Input Values:");
//             $display("Operation: %d", Operation);
//             $display("Register File: O1Sel: %d, O2Sel: %d, FunSel: %d, RSel: %d, TSel: %d", RF_O1Sel, RF_O2Sel, RF_FunSel, RF_RSel, RF_TSel);            
//             $display("ALU FunSel: %d", ALU_FunSel);
//             $display("Addres Register File: OutASel: %d, OutBSel: %d, FunSel: %d, Regsel: %d", ARF_OutASel, ARF_OutBSel, ARF_FunSel, ARF_RSel);            
//             $display("Instruction Register: LH: %d, Enable: %d, FunSel: %d", IR_LH, IR_Enable, IR_Funsel);            
//             $display("Memory: WR: %d, CS: %d", Mem_WR, Mem_CS);
//             $display("MuxASel: %d, MuxBSel: %d, MuxCSel: %d", MuxASel, MuxBSel, MuxCSel);
            
//             $display("");
//             $display("Output Values:");
//             $display("Register File: AOut: %d, BOut: %d", _ALUSystem.AOut, _ALUSystem.BOut);            
//             $display("ALUOut: %d, ALUOutFlag: %d, ALUOutFlags: Z:%d, C:%d, N:%d, O:%d,", _ALUSystem.ALUOut, _ALUSystem.ALUOutFlag, _ALUSystem.ALUOutFlag[3],_ALUSystem.ALUOutFlag[2],_ALUSystem.ALUOutFlag[1],_ALUSystem.ALUOutFlag[0]);
//             $display("Address Register File: AOut: %d, BOut (Address): %d", _ALUSystem.AOut, _ALUSystem.Address);            
//             $display("Memory Out: %d", _ALUSystem.MemoryOut);            
//             $display("Instruction Register: IROut: %d", _ALUSystem.IROut);            
//             $display("MuxAOut: %d, MuxBOut: %d, MuxCOut: %d", _ALUSystem.MuxAOut, _ALUSystem.MuxBOut, _ALUSystem.MuxCOut);
            
//             // increment array index and read next testvector
//             VectorNum = VectorNum + 1;
//             if (TestVectors[VectorNum] === 42'bx)
//             begin
//                 $display("%d tests completed.",
//                 VectorNum);
//                 $finish; // End simulation
//             end
//         end
// endmodule

module Project2Test();
    reg Clock;
    wire Reset;
    wire BRA_out;
    wire [15:0] IR_out;
    wire [7:0] RF_R1,  RF_R2,  RF_R3,  RF_R4,  ARF_PC,  ARF_SP,  ARF_AR;
    wire [2:0] T;

    CPUSystem CPU_Sys( 
        .Clock(Clock),
        .Reset(Reset),
        .T_out(T),
        .BRA_out(BRA_out),
        .IR_out(IR_out),
        .RF_R1(RF_R1),
        .RF_R2(RF_R2),
        .RF_R3(RF_R3),
        .RF_R4(RF_R4),
        .ARF_PC(ARF_PC),
        .ARF_SP(ARF_SP),
        .ARF_AR(ARF_AR)

    );
    always begin
        Clock = 0; #5; Clock = 1; #5; // 10ns period
    end

    initial begin 
        $dumpfile("Project2Test.vcd");
        $dumpvars(0, Project2Test);
        
        #1300; 

        $display("Project 2 testbench");
        $finish;
        end

endmodule
