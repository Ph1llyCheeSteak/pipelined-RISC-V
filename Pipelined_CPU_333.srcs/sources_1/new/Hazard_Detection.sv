`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
// Engineer:  S. Weston & Phillipe Bakhirev
// Module Name: PIPELINED_OTTER_CPU
// Revision: 0.1
//
//////////////////////////////////////////////////////////////////////////////////

module Hazard_Detection(
    input [4:0] rs1_e, rs2_e,rs1_d, rs2_d,
    input de_ex_rs1_used, 
    input de_ex_rs2_used, 
    input mem_rd_used,
    input wb_rd_used, 
    input de_rs1_used,
    input de_rs2_used, 
    input de_ex_rd_used, 
    input [4:0] ex_mem_rd, 
    input [4:0] mem_wb_rd, 
    input [4:0] id_ex_rd,
    input ex_mem_regWrite,
    input mem_wb_regWrite,
    input memRead2, 
    input stalled,
    input stalled2,
    input [2:0] pcSource,
    input [6:0] opcode,
    input [2:0] de_ex_rf_wr_sel,
    output reg [1:0] ForwardA, 
    output reg [1:0] ForwardB,
    output reg stall, 
    output reg flush
    );
     
    always_comb begin
        ForwardA = 2'b00; 
        ForwardB = 2'b00;
        stall = 1'b0; 
        flush = 1'b0; 
        
        // conditions for forwarding A
        // RAW 1 inst above
         if((rs1_e == ex_mem_rd) && ex_mem_regWrite && de_ex_rs1_used && mem_rd_used && !stalled2) begin // need more?
            ForwardA = 2'b10;
        end 
        // RAW 2 inst above
        else if(((rs1_e == mem_wb_rd) && mem_wb_regWrite) && de_ex_rs1_used && wb_rd_used) begin
            ForwardA = 2'b01;
        end        
        // conditions for forwarding B
        // RAW 1 inst above
         if(((rs2_e == ex_mem_rd) && ex_mem_regWrite) && de_ex_rs2_used && mem_rd_used && !stalled2) begin // need more?
            ForwardB = 2'b10;
        end
        // RAW 2 inst above
        else if(((rs2_e == mem_wb_rd) && mem_wb_regWrite) && de_ex_rs2_used && wb_rd_used)begin
            ForwardB = 2'b01;
        end
                       
        // conditions for stall
        //if((memRead2 && !stalled && de_ex_rd_used && ((de_rs1_used && rs1_d == id_ex_rd) || (de_rs2_used && rs2_d == id_ex_rd)))) begin
        //    stall = 1'b1;
        //end
        if(de_ex_rf_wr_sel == 2'b10 && !stalled && ((de_rs1_used && rs1_d == id_ex_rd) || (de_rs2_used && rs2_d == id_ex_rd))) begin
            stall = 1'b1;
        end
        
        // control hazards
        if(pcSource != 0) begin
            flush = 1'b1;
        end
        else begin
            flush = 0;
        end
    end
endmodule
