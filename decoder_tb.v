`timescale 1ps/1ps
`include "implementation.v"

module decoder_tb();

    reg [3:0] Ctrl;

    wire S0, S1, S2, S3, S4, S5, S6, S7, S8, S9, S10, S11, S12, S13, S14, S15;

    Dec_16_1 uut(
        .s(Ctrl),
        .S0(S0),
        .S1(S1),
        .S2(S2),
        .S3(S3),
        .S4(S4),
        .S5(S5),
        .S6(S6),
        .S7(S7),
        .S8(S8),
        .S9(S9),
        .S10(S10),
        .S11(S11),
        .S12(S12),
        .S13(S13),
        .S14(S14),
        .S15(S15)
    );

    initial begin 
        $dumpfile("decoder_tb.vcd");
        $dumpvars(0, decoder_tb);

        Ctrl = 0;
        #10 Ctrl = 1;
        #10 Ctrl = 2;
        #10 Ctrl = 3;
        #10 Ctrl = 4;
        #10 Ctrl = 5;
        #10 Ctrl = 6;
        #10 Ctrl = 7;
        #10 Ctrl = 8;
        #10 Ctrl = 9;
        #10 Ctrl = 10;
        #10 Ctrl = 11;
        #10 Ctrl = 12;
        #10 Ctrl = 13;
        #10 Ctrl = 14;
        #10 Ctrl = 15;
        #10 
        $display("Decoder testbench");
        $finish;
    end

endmodule