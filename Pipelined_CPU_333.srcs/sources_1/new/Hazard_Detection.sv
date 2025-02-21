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
    input logic [1:0] ex_pc_source,
    input logic mem_regWrite,
//    input logic de_rs1_used,
//    input logic de_rs2_used,
    input logic ex_rs1_used,
    input logic ex_rs2_used,
    output logic [1:0] fsel1,
    output logic [1:0] fsel2,
    output logic load_use_haz,
    output logic control_haz,
    output logic flush
    );
    
    assign load_haz_val = ((de_adr1 == ex_rd) || (de_adr2 == ex_rd)) && (opcode == 7'b0000011);
    
    always_comb begin
        fsel1 = 2'b00; //RS1 mux
        fsel2 = 2'b00; //RS2 mux
        load_use_haz = 1'b0;
        control_haz = 1'b0;
        flush = 1'b0;
        // Selects for forwarding muxes
        // ALU Source A
        if (mem_regWrite && mem_rd == ex_adr1 && ex_rs1_used) // RAW 1 inst above
            fsel1 = 2'b10; 
        else if (mem_regWrite && wb_rd == ex_adr1 && ex_rs1_used) // RAW 2 inst above being
            fsel1 = 2'b01;
        else 
            fsel1 = 2'b00;
        // ALU Source B
        if (mem_regWrite && mem_rd == ex_adr2 && ex_rs2_used) 
            fsel2 = 2'b10;
        else if (mem_regWrite && wb_rd == ex_adr2 && ex_rs2_used)
            fsel2 = 2'b01;
        else 
            fsel2 = 2'b00;
        
        // Load use data hazard
        if (load_haz_val) 
            load_use_haz = 1'b1;
        // Control hazards - jal, jalr, branch
        if (ex_pc_source != 2'b00) begin
            control_haz = 1'b1;
            flush = 1'b1;
        end
        else begin 
            control_haz = 1'b0;
            flush = 1'b0;
        end
    end
    
    
endmodule
