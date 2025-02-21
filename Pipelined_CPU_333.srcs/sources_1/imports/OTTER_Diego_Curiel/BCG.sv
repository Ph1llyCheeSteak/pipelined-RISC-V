`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: Cal Poly San Luis Obispo
// Engineer: Diego Curiel
// Create Date: 02/09/2023 11:30:51 AM
// Module Name: BCG
//////////////////////////////////////////////////////////////////////////////////

module BCG(
    input logic [31:0] RS1,
    input logic [31:0] RS2,
    input logic BRANCH,
    input logic JAL,
    input logic JALR,
    input logic [2:0] FUNCT,
    output logic [2:0] PC_SOURCE
    );
    logic BR_EQ;
    logic BR_LT;
    logic BR_LTU;
    //Assign outputs using conditional logic operators.
    assign BR_LT = $signed(RS1) < $signed(RS2);
    assign BR_LTU = RS1 < RS2;
    assign BR_EQ = RS1 == RS2;
    
    always_comb begin
        case(FUNCT)
            3'b000: begin
                if (BR_EQ == 1'b1)
                    PC_SOURCE = 3'b010;
                else
                    PC_SOURCE = 3'b000; 
            end
            3'b001: begin 
                if (BR_EQ == 1'b0)
                    PC_SOURCE = 3'b010;
                else
                    PC_SOURCE = 3'b000; 
            end
            3'b100: begin 
                if (BR_LT == 1'b1)
                    PC_SOURCE = 3'b010;
                else
                    PC_SOURCE = 3'b000;
            end
            3'b101: begin 
                if (BR_LT == 1'b0)
                    PC_SOURCE = 3'b010;
                else
                    PC_SOURCE = 3'b000;
            end
            3'b110: begin 
                if (BR_LTU == 1'b1)
                    PC_SOURCE = 3'b010;
                else
                    PC_SOURCE = 3'b000;
            end
            3'b111: begin 
                if (BR_LTU == 1'b0)
                    PC_SOURCE = 3'b010;
                else
                    PC_SOURCE = 3'b000;
            end
        endcase
    end
endmodule
