`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Engineer:  S. Weston & Phillipe Bakhirev
// Module Name: PIPELINED_OTTER_CPU
// Revision: 0.1
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
        
    wire [31:0] I_immed, S_immed, U_immed , pc_value, pc_next, pc_jalr, pc_branch, pc_jal;
    wire [2:0] pc_sel;
    wire jal, jalr, branch, memERR;
    logic [31:0] pc, ir, de_ir;
    
    //DEC_INST
    logic [31:0] aluRes, if_de_pc, reg_wd;
    logic [6:0] de_inst_opcode;
    logic [4:0] de_inst_rs1_addr, de_inst_rs2_addr, rd_addr;
    logic [1:0] rf_wr_sel, size;
    logic regWrite, memWrite, sign;

    //DEC_EX
    logic [31:0] de_ex_pc, de_ex_opA, de_ex_opB, de_ex_rs1, de_ex_rs2;
    logic [4:0] de_ex_rs1_addr, de_ex_rs2_addr, de_ex_rd_addr;
    logic [6:0] de_ex_opcode;
    logic [3:0] de_ex_alu_fun;
    logic [1:0] de_ex_size;
    logic de_ex_regWrite, de_ex_memWrite, de_ex_mem_rden1, de_ex_mem_rden2, de_ex_sign;
    
    //EX_MEM
    logic [31:0] ex_mem_pc, ex_mem_rs2, ex_mem_rd_addr;
    logic [1:0] ex_mem_size;
    logic ex_mem_mem_rden1, ex_mem_memWrite, ex_mem_regWrite, ex_mem_mem_rden2, ex_mem_sign;

    logic [31:0] ex_mem_aluRes;
    logic [1:0] ex_mem_rf_wr_sel;
    logic [2:0] ex_mem_pc_sel;

    
    //MEM_WB
    logic mem_wb_regWrite;

    logic [31:0] mem_data, mem_wb_aluRes;
    logic [4:0] mem_wb_rd_addr;
    logic [1:0] mem_wb_rf_wr_sel, alu_src_a, alu_src_b;

    //OUTPUTS
    assign IOBUS_ADDR = mem_wb_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    // Logic for Immediate Generator outputs and BAG and ALU MUX inputs    
    logic [31:0] Utype, Itype, Stype, Btype, Jtype;
    
//==== HAZARD DETECTION, DATA FORWARDING, FLUSHING ===========================================
 
    // not sure if we need these
    logic de_rs1_used;
    logic de_rs2_used;
    logic ex_rs1_used;
    logic ex_rs2_used;
   
    logic [1:0] fsel1;
    logic [1:0] fsel2;
    logic load_use_haz;
    logic control_haz;
    logic flush;
    
    logic [31:0] pre_mux_A;
    logic [31:0] pre_mux_B;

    // DO WE NEED THE _USED SIGNALS
    // IF WE GET ERRORS, CHECK THE OPCODE DATAPATH, PC & IR FLOW, 
    Hazard_Detection OTTER_Hazard_Detection(.opcode(OPCODE), .de_adr1(de_inst_rs1_addr), .de_adr2(de_inst_rs2_addr), 
        .ex_adr1(de_ex_rs1_addr), .ex_adr2(de_ex_rs2_addr), .ex_rd(rd_addr), .mem_rd(ex_mem_rd_addr), 
        .wb_rd(mem_wb_rd_addr), .ex_pc_source(ex_mem_pc_sel), .mem_regWrite(ex_mem_regWrite),
        .ex_rs1_used(), .ex_rs2_used(), .fsel1(fsel1), .fsel2(fsel2), .load_use_haz(load_use_haz), 
        .control_haz(control_haz), .flush(flush));
    
    //PRE-MUXES
    FourMux OTTER_ALU_PREMUXA(.SEL(fsel1), .ZERO(de_ex_rs1), .ONE(reg_wd), .TWO(ex_mem_aluRes), .THREE(32'b0), .OUT(pre_mux_A));   
    FourMux OTTER_ALU_PREMUXB(.SEL(fsel2), .ZERO(de_ex_rs2), .ONE(reg_wd), .TWO(ex_mem_aluRes), .THREE(32'b0), .OUT(pre_mux_B));
    
    // ALU MUXES
    TwoMux OTTER_ALU_MUXA(.SEL(alu_src_a), .RS1(pre_mux_A), .U_TYPE(Utype), .OUT(de_ex_opA));    
    FourMux OTTER_ALU_MUXB(.SEL(alu_src_b), .ZERO(pre_mux_B), .ONE(Itype), .TWO(Stype), .THREE(de_inst_pc), .OUT(de_ex_opB));

     
//==== Instruction Fetch ===========================================
     
    always_ff @(posedge CLK) begin //pipeline register
        if(!load_use_haz) begin // stallF equivalent
            if_de_pc <= pc;
            de_ir = ir;
        end
    end
    
    assign mem_rden1 = 1'b1; 	// Fetch new instruction every cycle
    
    // Disable PC when load_use_haz enabled
    PC OTTER_PC(.CLK(CLK), .RST(pc_int), .PC_WRITE(!load_use_haz), .PC_SEL(pc_sel),
        .JALR(pc_jalr), .JAL(pc_jal), .BRANCH(pc_branch), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc), .PC_OUT_INC(pc_next));

    logic [31:0] addr1;
    assign addr1 = pc;
          
    //Instantiate Mem
    OTTER_mem_byte OTTER_MEM(.MEM_CLK(CLK), .MEM_READ1(ex_mem_mem_rden1), .MEM_READ2(ex_mem_mem_rden2), 
        .MEM_WRITE2(ex_mem_memWrite), .MEM_ADDR1(addr1), .MEM_ADDR2(ex_mem_aluRes), .MEM_DIN2(ex_mem_rs2), .MEM_SIZE(ex_mem_size),
         .MEM_SIGN(ex_mem_sign), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR), .MEM_DOUT1(ir), .MEM_DOUT2(mem_data), .ERR(memERR));
    
//==== Instruction Decode ===========================================

    // Logic for Decoder
	logic ir30;
    assign ir30 = de_ir[30];
    logic [6:0] opcode;
    assign opcode = de_ir[6:0];
    logic [2:0] funct;
    assign funct = de_ir[14:12];
    
    // Logic for Immediate Generator 
    logic [24:0] imgen_ir;
    assign imgen_ir = de_ir[31:7]; 
    
    // Logic for OPCODE parsing
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(opcode);
    
    assign de_inst_rs1_addr = de_ir[19:15];
    assign de_inst_rs2_addr = de_ir[24:20];
    assign rd_addr = de_ir[11:7];
    assign de_inst_opcode = OPCODE;
    assign sign = de_ir[14];
    assign size = de_ir[13:12];
    
    logic de_ex_branch;
    logic de_ex_jal;
    logic de_ex_jalr;
    logic [2:0] de_ex_funct;

    always_ff@(posedge CLK) begin
        if(flush && control_haz) begin //branch; need to flush D and E
            de_ex_pc <= 31'b0;
            de_ex_rs1_addr <= 5'b0;
            de_ex_rs2_addr <= 5'b0;
            de_ex_opcode <= 7'b0;
            de_ex_regWrite <= 1'b0;
            de_ex_sign <= 1'b0;
            de_ex_size <= 2'b0;
            de_ex_jal <= 1'b0;
            de_ex_branch <= 1'b0;
            de_ex_jalr <= 1'b0;
            de_ex_funct <= 3'b0;
        end
        else if (load_use_haz) begin // stall D equivalent
            de_ex_pc <= de_ex_pc;
            de_ex_rs1_addr <= de_ex_rs1_addr;
            de_ex_rs2_addr <= de_ex_rs2_addr;
            de_ex_opcode <= de_ex_opcode;
            de_ex_regWrite <= de_ex_regWrite;
            de_ex_sign <= de_ex_sign;
            de_ex_size <= de_ex_size;
            de_ex_jal <= de_ex_jal;
            de_ex_branch <= de_ex_branch;
            de_ex_jalr <= de_ex_jalr;
            de_ex_funct <= de_ex_funct;
        end
        else begin
            de_ex_pc <= if_de_pc;
            de_ex_rs1_addr <= de_inst_rs1_addr;
            de_ex_rs2_addr <= de_inst_rs2_addr;
            de_ex_opcode <= OPCODE;
            de_ex_regWrite <= regWrite;
            de_ex_sign <= sign;
            de_ex_size <= size;
            de_ex_jal <= jal;
            de_ex_branch <= branch;
            de_ex_jalr <= jalr;
            de_ex_funct <= funct;
        end
        
	end
	
	// Instantiate RegFile
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(mem_wb_regWrite), .ADR1(de_inst_rs1_addr), .ADR2(de_inst_rs2_addr), .WA(mem_wb_rd_addr), 
        .WD(reg_wd), .RS1(de_ex_rs1), .RS2(de_ex_rs2));

	// Instantiate Decoder
    CU_DCDR OTTER_DCDR(.IR_30(ir30), .IR_OPCODE(OPCODE), .IR_FUNCT(funct), .OBRANCH(branch), 
        .ALU_FUN(de_ex_alu_fun), .ALU_SRCA(alu_src_a), .OJALR(jalr), .OJAL(jal),
        .ALU_SRCB(alu_src_b), .RF_WR_SEL(rf_wr_sel), .PC_WRITE(pc_write), 
        .REG_WRITE(regWrite), .MEM_WE2(de_ex_memWrite), .MEM_RDEN1(de_ex_mem_rden1), .MEM_RDEN2(de_ex_mem_rden2), .PC_RST(pc_int));
    
    // Instantiate Immediate Generator
    ImmediateGenerator OTTER_IMGEN(.IR(imgen_ir), .U_TYPE(Utype), .I_TYPE(Itype), .S_TYPE(Stype),
        .B_TYPE(Btype), .J_TYPE(Jtype));
       

//==== Execute ======================================================
    logic [31:0] ex_mem_rs1;
    always_ff@(posedge CLK) begin
        if (load_use_haz) begin // flushE equivalent
            ex_mem_pc <= 0;
            ex_mem_rd_addr <= 0;
            ex_mem_memWrite <= 0;
            ex_mem_regWrite <= 0;
            ex_mem_rs1 <= 0;
            ex_mem_rs2 <= 0;
            ex_mem_rf_wr_sel <= 0;
            ex_mem_aluRes <= 0;
            ex_mem_sign <= 0;
            ex_mem_size <= 0;
            ex_mem_mem_rden1 <= 0;
            ex_mem_mem_rden2 <= 0;
            ex_mem_pc_sel <= 0;
        end
        else begin
            ex_mem_pc <= de_ex_pc;
            ex_mem_rd_addr <= rd_addr;
            ex_mem_memWrite <= de_ex_memWrite;
            ex_mem_regWrite <= de_ex_regWrite;
            ex_mem_rs1 <= de_ex_rs1; /// chcek for issues
            ex_mem_rs2 <= de_ex_rs2;
            ex_mem_rf_wr_sel <= rf_wr_sel;
            ex_mem_aluRes <= aluRes;
            ex_mem_sign <= sign;
            ex_mem_size <= size;
            ex_mem_mem_rden1 <= de_ex_mem_rden1;
            ex_mem_mem_rden2 <= de_ex_mem_rden2;
//            ex_mem_pc_sel <= pc_sel;
        end
	end

    // Instantiate ALU
    ALU OTTER_ALU(.SRC_A(de_ex_opA), .SRC_B(de_ex_opB), .ALU_FUN(de_ex_alu_fun), .RESULT(aluRes));
    
	//Instantiate Branch Condition Generator
    BCG OTTER_BCG(.RS1(de_ex_rs1), .RS2(IOBUS_OUT), .BRANCH(de_ex_branch), .JAL(de_ex_jal), 
                  .JALR(de_ex_jalr), .FUNCT(de_ex_funct), .PC_SOURCE(pc_sel));
	
    // Instantiate Branch Address Generator
    //ex_mem_rs1, ex_mem_px
    BAG OTTER_BAG(.RS1(ex_mem_rs1), .I_TYPE(Itype), .J_TYPE(Jtype), .B_TYPE(Btype), .FROM_PC(ex_mem_pc),
                  .JAL(pc_jal), .JALR(pc_jalr), .BRANCH(pc_branch));

//==== Memory ======================================================

    always_ff@(posedge CLK) begin
        mem_wb_regWrite <= ex_mem_regWrite;
        mem_wb_rf_wr_sel <= ex_mem_rf_wr_sel;
        mem_wb_rd_addr <= ex_mem_rd_addr;
        mem_wb_aluRes <= ex_mem_aluRes;
	end
     
//==== Write Back ==================================================
     
    FourMux OTTER_REG_MUX(.SEL(mem_wb_rf_wr_sel), .ZERO(pc_next), .ONE(32'b0), .TWO(mem_data), .THREE(mem_wb_aluRes),
        .OUT(reg_wd));
        
endmodule
