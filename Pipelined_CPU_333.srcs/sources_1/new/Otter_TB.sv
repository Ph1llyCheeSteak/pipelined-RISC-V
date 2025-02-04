`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 01/23/2025 07:11:47 PM
// Design Name: 
// Module Name: Otter_TB
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


module Otter_TB();
//     logic RST;
//     logic [31:0] IOBUS_IN;
//     logic CLK;
//     logic IOBUS_WR;
//     logic [31:0] IOBUS_OUT;
//     logic [31:0] IOBUS_ADDR;
    
//    OTTER my_otter(
//    .CLK (CLK),
//    .RST (RST),
//    .IOBUS_IN (IOBUS_IN),
//    .IOBUS_WR (IOBUS_WR),
//    .IOBUS_ADDR (IOBUS_ADDR)
//    );
    
//    //- Generate periodic clock signal    
//   initial    
//      begin       
//         CLK = 0;   //- init signal        
//         forever  #10 CLK = ~CLK;
//      end 

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
