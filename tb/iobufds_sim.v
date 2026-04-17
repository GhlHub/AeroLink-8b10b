`timescale 1ns / 1ps
// Behavioral simulation model of Xilinx IOBUFDS primitive
module IOBUFDS (
    output wire O,
    inout  wire IO,
    inout  wire IOB,
    input  wire I,
    input  wire T
);
    assign IO  = T ? 1'bz : I;
    assign IOB = T ? 1'bz : ~I;
    assign O   = IO;
endmodule
