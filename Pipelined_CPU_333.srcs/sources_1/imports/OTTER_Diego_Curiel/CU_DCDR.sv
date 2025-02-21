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
    output logic [3:0] ALU_FUN,
    output logic ALU_SRCA,
    output logic [1:0] ALU_SRCB,
    output logic [1:0] RF_WR_SEL,
    output logic PC_WRITE, //NEW FROM HERE DOWN
    output logic REG_WRITE, 
    output logic MEM_WE2, 
    output logic MEM_RDEN1, 
    output logic MEM_RDEN2,
    output logic PC_RST,
    output logic OJAL,
    output logic OJALR,
    output logic OBRANCH
    );
    
typedef enum logic [6:0] {
    AUIPC  = 7'b0010111,
    JAL    = 7'b1101111,
    JALR   = 7'b1100111,
    STORE  = 7'b0100011,
    LOAD   = 7'b0000011,
    LUI    = 7'b0110111,
    I_TYPE = 7'b0010011,
    R_TYPE = 7'b0110011,
    B_TYPE = 7'b1100011
} opcode_t;

typedef enum logic [2:0] {
    ALU_ADD  = 3'b000,  // ADDI
    ALU_SLL  = 3'b001,  // SLLI
    ALU_SLT  = 3'b010,  // SLTI
    ALU_SLTU = 3'b011,  // SLTIU
    ALU_XOR  = 3'b100,  // XORI
    ALU_SRL_SRA  = 3'b101,  // SRLI or SRAI
    ALU_OR   = 3'b110,  // ORI
    ALU_AND  = 3'b111   // ANDI
} alu_t;



    //Create always comb clock for decoder logic
    always_comb begin
        //Instantiate all outputs to 0 so as to avoid
        //unwanted leftovers from previous operations
        //and maintain direct control of outputs through
        //case statement below
        ALU_FUN = 4'b0000;
        ALU_SRCA = 1'b0;
        ALU_SRCB = 2'b00;
        RF_WR_SEL = 2'b00;
        PC_WRITE = 1'b0; 
        REG_WRITE = 1'b0;
        MEM_WE2 = 1'b0;
        MEM_RDEN1 = 1'b1; // ALWAYS HIGH
        MEM_RDEN2 = 1'b0;
        PC_RST = 1'b0;
        OBRANCH = 1'b0;
        OJAL = 1'b0;
        OJALR = 1'b0;
        
        if (IR_OPCODE === 7'bx) begin
            PC_RST = 1'b1; // Set default value if uninitialized
        end
        
        //Case statement depending on the opcode for the 
        //instruction, or the last seven bits of each instruction
        case (opcode_t'(IR_OPCODE))
            AUIPC: begin 
                ALU_SRCA = 1'b1;
                ALU_SRCB = 2'b11;
                RF_WR_SEL = 2'b11;
                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
            end
            JAL: begin 
                OJAL = 1'b1;
                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
            end
            JALR: begin 
                OJALR = 1'b1;
                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
            end
            STORE: begin 
                ALU_SRCB = 2'b10;
                MEM_WE2 = 1'b1;
                PC_WRITE = 1'b1;
            end
            LOAD: begin 
                ALU_SRCB = 2'b01;
                RF_WR_SEL = 2'b10;
                REG_WRITE = 1'b1; //Previously in WB
                PC_WRITE = 1'b1; // Previously in WB
                MEM_RDEN2 = 1'b1;
            end
            LUI: begin 
                ALU_FUN = 4'b1001;
                ALU_SRCA = 1'b1;
                RF_WR_SEL = 2'b11;
                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
            end
            I_TYPE: begin 
                //set constants for all I-type instructions
                ALU_SRCB = 2'b01;
                RF_WR_SEL = 2'b11;
                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
                
                //Nested case statement
                //dependent on the function 3 bits
                case (alu_t'(IR_FUNCT))
                    ALU_ADD: begin ALU_FUN = 4'b0000; end // ADDI
                    ALU_SLL: begin ALU_FUN = 4'b0001; end // SLLI
                    ALU_SLT: begin ALU_FUN = 4'b0010; end // SLTI
                    ALU_SLTU: begin ALU_FUN = 4'b0011; end // SLTIU
                    ALU_XOR: begin ALU_FUN = 4'b0100; end // XORI
                    ALU_SRL_SRA: begin //SRLI or SRAI
                        //nested case statement
                        //dependent on the 30th bit for 
                        //instructions that have the same opcode and 
                        //fucntion 3 bits
                        case(IR_30)
                            1'b0: begin ALU_FUN = 4'b0101; end //SRLI
                            1'b1: begin ALU_FUN = 4'b1101; end //SRAI
                            default: begin end
                        endcase
                    end
                    ALU_OR: begin ALU_FUN = 4'b0110; end //ORI
                    ALU_AND: begin ALU_FUN = 4'b0111; end //ANDI
                endcase
            end
            R_TYPE: begin 
                //set constants for all R-types;
                //ALU_FUN is just the concatenation of
                //the 30th bit and the function 3 bits
                RF_WR_SEL = 2'b11;
                ALU_FUN = {IR_30, IR_FUNCT};
                PC_WRITE = 1'b1;
                REG_WRITE = 1'b1;
            end
            B_TYPE: begin 
                //nested case statement dependent on the
                //function three bits.
                //Because there are six real branch instructions, there
                //are six pairs of if-else statements in each of six cases
                //for the branch instructions.
                PC_WRITE = 1'b1;
                OBRANCH = 1'b1;
            end
            default: 
            begin 
            end
        endcase
    end
    
endmodule
