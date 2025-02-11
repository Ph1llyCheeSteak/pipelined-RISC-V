`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer:  J. Callenes
//
// Create Date: 01/04/2019 04:32:12 PM
// Design Name: 
// Module Name: PIPELINED_OTTER_CPU
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

  typedef enum logic [6:0] {
           LUI      = 7'b0110111,
           AUIPC    = 7'b0010111,
           JAL      = 7'b1101111,
           JALR     = 7'b1100111,
           BRANCH   = 7'b1100011,
           LOAD     = 7'b0000011,
           STORE    = 7'b0100011,
           OP_IMM   = 7'b0010011,
           OP       = 7'b0110011,
           SYSTEM   = 7'b1110011
 } opcode_t;
        
module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);   
        
    wire [31:0] I_immed, S_immed, U_immed, pc, pc_value, pc_next, pc_jalr, pc_branch, pc_jal;
    wire [31:0] ir;
    wire [2:0] pc_sel;
    wire br_lt, br_eq, br_ltu,memERR;

    //DEC_INST
    logic [4:0] de_inst_rs1_addr;
    logic [4:0] de_inst_rs2_addr;
    logic [4:0] rd_addr;
    logic [6:0] de_inst_opcode;
    logic [31:0] aluRes;
    logic memWrite;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [31:0] reg_wd;
    logic sign;
    logic [1:0] size;
    logic [31:0] if_de_pc;
    
    //DEC_EX
    logic [31:0] de_ex_pc, de_ex_mem_rden, de_ex_opA, de_ex_opB, de_ex_rs1, de_ex_rs2;
    logic [3:0] de_ex_alu_fun;
    logic de_ex_regWrite, de_ex_memWrite;
    logic [31:0] de_ex_rs1_addr;
    logic [31:0] de_ex_rs2_addr;
    logic [31:0] de_ex_rd_addr;
    logic [31:0] de_ex_opcode;
    logic [31:0] de_ex_aluRes;
    
    //EX_MEM
    logic [31:0] ex_mem_rs1_addr;
    logic [31:0] ex_mem_rs2_addr;
    logic [31:0] ex_mem_rd_addr;
    logic [31:0] ex_mem_opcode;
    logic ex_mem_mem_rden, ex_mem_memWrite, ex_mem_regWrite;
    logic [31:0] ex_mem_pc, ex_mem_opA, ex_mem_opB, ex_mem_rs1, ex_mem_rs2, ex_mem_alu_fun;
    logic [31:0] ex_mem_aluRes;
    logic [31:0] opA_forwarded;
    logic [31:0] opB_forwarded;
    logic [1:0] ex_mem_rf_wr_sel;
    logic ex_mem_sign;
    logic [1:0] ex_mem_size;
    
    //MEM_WB
     logic [31:0] mem_data;
     logic [1:0] mem_wb_rf_wr_sel;
     logic mem_wb_regWrite;
     logic [1:0] alu_src_b;
     logic [31:0] mem_wb_aluRes;
     logic [31:0] mem_wb_rs2;
     logic mem_wb_sign;
     logic [1:0] mem_wb_size;
     logic [4:0] mem_wb_rd_addr;

     //OUTPUTS
     assign IOBUS_ADDR = mem_wb_aluRes;
     assign IOBUS_OUT = ex_mem_rs2;

     
//==== Instruction Fetch ===========================================
     
     always_ff @(posedge CLK) begin //pipeline register
        if_de_pc <= pc;
     end
     
     assign pcWrite = 1'b1; 	// Hardwired high, assuming no hazards
     assign mem_rden1 = 1'b1; 	// Fetch new instruction every cycle
    
    PC OTTER_PC(.CLK(CLK), .RST(pc_int), .PC_WRITE(pcWrite), .PC_SEL(pc_sel),
        .JALR(pc_jalr), .JAL(pc_jal), .BRANCH(pc_branch), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc), .PC_OUT_INC(pc_next));

    logic [31:0] addr1;
//    assign addr1 = pc[15:2];
    assign addr1 = pc;
          
    OTTER_mem_byte OTTER_MEM(.MEM_CLK(CLK), .MEM_READ1(mem_rden1), .MEM_READ2(ex_mem_mem_rden), 
        .MEM_WRITE2(ex_mem_memWrite), .MEM_ADDR1(addr1), .MEM_ADDR2(ex_mem_aluRes), .MEM_DIN2(mem_wb_rs2), .MEM_SIZE(ex_mem_size),
         .MEM_SIGN(ex_mem_sign), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR), .MEM_DOUT1(ir), .MEM_DOUT2(mem_data), .ERR(memERR));
    
//==== Instruction Decode ===========================================

    // Logic for Decoder
	logic ir30;
    assign ir30 = ir[30];
    logic [6:0] opcode;
    assign opcode = ir[6:0];
    logic [2:0] funct;
    assign funct = ir[14:12];
    
    // Logic for Immediate Generator 
    logic [24:0] imgen_ir;
    assign imgen_ir = ir[31:7]; 
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(opcode);
    
    assign de_inst_rs1_addr = ir[19:15];
    assign de_inst_rs2_addr = ir[24:20];
    assign rd_addr = ir[11:7];
    assign de_inst_opcode = OPCODE;
    
     always_ff@(posedge CLK) begin
        de_ex_pc <= if_de_pc;
        de_ex_rs1_addr <= de_inst_rs1_addr;
        de_ex_rs2_addr <= de_inst_rs2_addr;
        de_ex_opcode <= de_inst_opcode;
        de_ex_memWrite <= memWrite;
        de_ex_regWrite <= regWrite;
        sign <= ir[14];
        size <= ir[13:12]; 
	end
	
	//Instantiate RegFile
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(mem_wb_regWrite), .ADR1(de_inst_rs1_addr), .ADR2(de_inst_rs2_addr), .WA(mem_wb_rd_addr), 
        .WD(reg_wd), .RS1(de_ex_rs1), .RS2(de_ex_rs2));

	//Instantiate Decoder
    CU_DCDR OTTER_DCDR(.IR_30(ir30), .IR_OPCODE(opcode), .IR_FUNCT(funct), .BR_EQ(br_eq), 
        .BR_LT(br_lt), .BR_LTU(br_ltu), .ALU_FUN(de_ex_alu_fun), .ALU_SRCA(alu_src_a), 
        .ALU_SRCB(alu_src_b), .PC_SOURCE(pc_sel), .RF_WR_SEL(rf_wr_sel), .PC_WRITE(pc_write), 
        .REG_WRITE(regWrite), .MEM_WE2(memWrite), .MEM_RDEN1(de_ex_mem_rden), .MEM_RDEN2(de_ex_mem_rden), .PC_RST(pc_int));

    //Create logic for Immediate Generator outputs and BAG and ALU MUX inputs    
    logic [31:0] Utype, Itype, Stype, Btype, Jtype;
    
    //Instantiate Immediate Generator
    ImmediateGenerator OTTER_IMGEN(.IR(imgen_ir), .U_TYPE(Utype), .I_TYPE(Itype), .S_TYPE(Stype),
        .B_TYPE(Btype), .J_TYPE(Jtype));
        
    //Instantiate ALU MUXes
    TwoMux OTTER_ALU_MUXA(.SEL(alu_src_a), .RS1(de_ex_rs1), .U_TYPE(Utype), .OUT(de_ex_opA));
    FourMux OTTER_ALU_MUXB(.SEL(alu_src_b), .ZERO(de_ex_rs2), .ONE(Itype), .TWO(Stype), .THREE(de_inst_pc), .OUT(de_ex_opB));

//==== Execute ======================================================

    always_ff@(posedge CLK) begin
        ex_mem_pc <= de_ex_pc;
        ex_mem_rs1_addr <= de_ex_rs1_addr;
        ex_mem_rs2_addr <= de_ex_rs2_addr;
        ex_mem_rd_addr <= rd_addr;
        ex_mem_opcode <= de_ex_opcode;
        ex_mem_memWrite <= de_ex_memWrite;
        ex_mem_regWrite <= de_ex_regWrite;
        ex_mem_opA <= de_ex_opA;
        ex_mem_opB <= de_ex_opB;
        ex_mem_rs1 <= de_ex_rs1;
        ex_mem_rs2 <= de_ex_rs2;
        ex_mem_alu_fun <= de_ex_alu_fun;
        ex_mem_rf_wr_sel <= rf_wr_sel;
        ex_mem_aluRes <= aluRes;
        ex_mem_sign <= sign;
        ex_mem_size <= size;
	end

    // Instantiate ALU
    ALU OTTER_ALU(.SRC_A(de_ex_opA), .SRC_B(de_ex_opB), .ALU_FUN(de_ex_alu_fun), .RESULT(aluRes));
    
	//Instantiate Branch Condition Generator
    BCG OTTER_BCG(.RS1(rs1), .RS2(IOBUS_OUT), .BR_EQ(br_eq), .BR_LT(br_lt), .BR_LTU(br_ltu));
	
    // Instantiate Branch Address Generator
    BAG OTTER_BAG(.RS1(rs1), .I_TYPE(Itype), .J_TYPE(Jtype), .B_TYPE(Btype), .FROM_PC(pc_out),
                  .JAL(jal), .JALR(jalr), .BRANCH(branch));

//==== Memory ======================================================

    always_ff@(posedge CLK) begin
        mem_wb_regWrite <= ex_mem_regWrite;
        mem_wb_rf_wr_sel <= ex_mem_rf_wr_sel;
        mem_wb_rd_addr <= ex_mem_rd_addr;
        mem_wb_aluRes <= ex_mem_aluRes;
        mem_wb_rs2 <= ex_mem_rs2;
	end
     
//==== Write Back ==================================================
     
    FourMux OTTER_REG_MUX(.SEL(mem_wb_rf_wr_sel), .ZERO(pc_next), .ONE(32'b0), .TWO(mem_data), .THREE(mem_wb_aluRes),
        .OUT(reg_wd));
        
endmodule
