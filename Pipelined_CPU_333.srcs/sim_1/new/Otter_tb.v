`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Ratner Surf Designs
// Engineer:  James Ratner
// 
// Create Date: 01/07/2020 12:59:51 PM
// Design Name: 
// Module Name: Ex6_6_testbench
// Project Name: 
// Target Devices: 
// Tool Versions: 
// Description: Testbench file for Exp 6
// 
// Dependencies: 
// 
// Revision:
// Revision 0.01 - File Created
// Additional Comments:
// 
//////////////////////////////////////////////////////////////////////////////////

module testbench(); 

   reg [15:0] switches; 
   reg [4:0] buttons;
   reg clk; 
   wire [15:0] leds; 
   wire [7:0] segs; 
   wire [3:0] an; 


OTTER_Wrapper my_wrapper(
   .CLK    (clk),
   .BTNC  (buttons),
   .SWITCHES  (switches),
   .LEDS  (leds),
   .CATHODES  (segs),
   .ANODES  (an)  );

   
  //- Generate periodic clock signal    
   initial    
      begin       
         clk = 0;   //- init signal        
         forever  #10 clk = ~clk;    
      end                        
endmodule
