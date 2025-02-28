`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: California Polytechnic University, San Luis Obispo
// Engineer: Diego Renato Curiel
// Create Date: 02/23/2023 09:39:49 AM
// Module Name: CU_DCDR
//////////////////////////////////////////////////////////////////////////////////


module CU_DCDR(
    input logic IR_30,
    input logic [6:0] IR_OPCODE,
    input logic [2:0] IR_FUNCT,
    input logic BR_EQ,
    input logic BR_LT,
    input logic BR_LTU,
    output logic [3:0] ALU_FUN,
    output logic ALU_SRCA,
    output logic [1:0] ALU_SRCB,
    output logic [1:0] RF_WR_SEL,
    output logic REG_WRITE,
    output logic MEM_WRITE,
    output logic MEM_READ_2
    );
    
    always_comb begin
        ALU_FUN = 4'b0000;
        ALU_SRCA = 1'b0;
        ALU_SRCB = 2'b00;
        
        RF_WR_SEL = 2'b00;
        REG_WRITE = 1'b0;
        
        MEM_WRITE = 1'b0;
        MEM_READ_2 = 1'b0;

        case (IR_OPCODE)
            7'b0010111: begin   // AUIPC
                ALU_SRCA    = 1'b1;
                ALU_SRCB    = 2'b11;
                RF_WR_SEL   = 2'b11;
                REG_WRITE   = 1'b1;
            end
            
            7'b1101111:         // JAL
                REG_WRITE   = 1'b1;
                
            7'b1100111:         // JALR
                REG_WRITE   = 1'b1;
                
            7'b0100011: begin   // STORE
                ALU_SRCB    = 2'b10;
                MEM_WRITE   = 1'b1;
            end
            
            7'b0000011: begin   // LOAD
                ALU_SRCB    = 2'b01;
                RF_WR_SEL   = 2'b10;
                REG_WRITE   = 1'b1;
                MEM_READ_2  = 1'b1;
            end
            
            7'b0110111: begin   // LUI
                ALU_FUN     = 4'b1001;
                ALU_SRCA    = 1'b1;
                RF_WR_SEL   = 2'b11;
                REG_WRITE   = 1'b1;
            end
            
            7'b0010011: begin   // I-Type
                ALU_SRCB    = 2'b01;
                RF_WR_SEL   = 2'b11;
                REG_WRITE   = 1'b1;
                case (IR_FUNCT)
                    3'b000: ALU_FUN = 4'b0000; 
                    3'b001: ALU_FUN = 4'b0001; 
                    3'b010: ALU_FUN = 4'b0010; 
                    3'b011: ALU_FUN = 4'b0011; 
                    3'b100: ALU_FUN = 4'b0100; 
                    3'b101: begin
                        case(IR_30)
                            1'b0: ALU_FUN = 4'b0101;
                            1'b1: ALU_FUN = 4'b1101;
                            default: begin end
                        endcase
                    end
                    3'b110: begin ALU_FUN = 4'b0110; end
                    3'b111: begin ALU_FUN = 4'b0111; end
                endcase
            end
            
            7'b0110011: begin   // R-Type
                ALU_FUN = {IR_30, IR_FUNCT};
                RF_WR_SEL = 2'b11;
                REG_WRITE = 1'b1;
            end
            
            default: begin end
            
        endcase
    end
    
endmodule
