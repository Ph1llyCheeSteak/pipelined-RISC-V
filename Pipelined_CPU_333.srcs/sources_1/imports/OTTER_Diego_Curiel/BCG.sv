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
    input logic [2:0]func3,
    input logic [6:0]opcode,
    output logic [2:0]PC_SOURCE,
    output logic branch
    );
     
    // Assign Internal Signals
    wire BR_LT, BR_LTU, BR_EQ;
    assign BR_LT = $signed(RS1) < $signed(RS2);
    assign BR_LTU = RS1 < RS2;
    assign BR_EQ = RS1 == RS2;
    
    always_comb begin
    branch = 1'b0; 
    PC_SOURCE = 3'b000;
        case (opcode)
            7'b1101111: begin                       // JAL
                PC_SOURCE = 3'b011;
            end
            7'b1100111: begin                       // JALR
                PC_SOURCE = 3'b001;
            end
            
            7'b1100011: begin
                case(func3) 
                    3'b000: begin                   // BEQ
                        if (BR_EQ == 1'b1) begin
                            PC_SOURCE = 3'b010;
                            branch = 1'b1;
                        end
                        else
                            PC_SOURCE = 3'b000; 
                    end
                    
                    3'b001: begin                   // BNE
                        if (BR_EQ == 1'b0) begin
                            PC_SOURCE = 3'b010;
                            branch = 1'b1;
                        end
                        else
                            PC_SOURCE = 3'b000; 
                    end
                    
                    3'b100: begin                   // BLT
                        if (BR_LT == 1'b1) begin
                            PC_SOURCE = 3'b010;
                            branch = 1'b1;
                        end
                        else
                            PC_SOURCE = 3'b000;
                    end
                    
                    3'b101: begin                   // BGE
                        if (BR_LT == 1'b0) begin
                            PC_SOURCE = 3'b010;
                            branch = 1'b1;
                        end
                        else
                            PC_SOURCE = 3'b000;
                    end
                    
                    3'b110: begin                   // BLTU
                        if (BR_LTU == 1'b1) begin
                            PC_SOURCE = 3'b010;
                            branch = 1'b1;
                        end
                        else
                            PC_SOURCE = 3'b000;
                    end
                    
                    3'b111: begin                   // BGEU
                        if (BR_LTU == 1'b0) begin
                            PC_SOURCE = 3'b010;
                            branch = 1'b1;
                        end
                        else
                            PC_SOURCE = 3'b000;
                    end
                    
               endcase
           end
        endcase
    end
endmodule
