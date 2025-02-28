`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Engineer:  S. Weston & Phillipe Bakhirev
// Module Name: OTTER_TESTBENCH
// Create Date: 01/23/2025
// Revision: 0.1
//
//////////////////////////////////////////////////////////////////////////////////
module Otter_TB();
    
    //OTTER SIMULATION TEST BENCH
    
    logic clk, intr, rst, wr;
    logic [31:0] in, iobus_out, iobus_addr;
    OTTER_MCU CPU( .CLK(clk), .IOBUS_IN(in), .RESET(rst),
              .IOBUS_OUT(iobus_out), .IOBUS_ADDR(iobus_addr), .IOBUS_WR(wr) );
    initial begin
    clk = 0;
    rst = 1'b1;
    end
    always begin
    #10 clk = ~clk; end
    always begin
    #60 rst = 0;
     in = 32'h0;
    end
endmodule
