`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 02/18/2025 02:44:26 PM
// Design Name: 
// Module Name: Hazard_Detection
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


module Hazard_Detection(
    input logic [6:0] opcode,
    input logic [4:0] de_adr1,
    input logic [4:0] de_adr2,
    input logic [4:0] ex_adr1,
    input logic [4:0] ex_adr2,
    input logic [4:0] ex_rd,
    input logic [4:0] mem_rd,
    input logic [4:0] wb_rd,
    input logic [1:0] pc_source,
    input logic mem_regWrite,
    input logic de_rs1_used,
    input logic de_rs2_used,
    input logic ex_rs1_used,
    input logic ex_rs2_used,
    output logic [1:0] fsel1,
    output logic [1:0] fsel2,
    output logic load_use_haz,
    output logic control_haz,
    output logic flush
    );
    
    always_comb begin
        fsel1 = 2'b00;
        fsel2 = 2'b00;
        load_use_haz = 1'b0;
        control_haz = 1'b0;
        flush = 1'b0;
        // Selects for forwarding muxes
        if (mem_regWrite && mem_rd == ex_adr1 && ex_rs1_used) begin
            fsel1 = 2'b01;
            // add logic
            end
        // Load use data hazard
        if (load_use_haz) //need more logic in if condition 
        begin
        end
        // Control hazards - jal, jalr, branch
        if (pc_source != 2'b00) begin
            control_haz = 1'b1;
            flush = 1'b1;
        end
        else begin 
            control_haz = 1'b0;
            flush = 1'b0;
        end
    end
    
    
endmodule
