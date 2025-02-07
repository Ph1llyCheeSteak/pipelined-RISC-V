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
        
typedef struct packed{
    opcode_t opcode;
    logic [4:0] rs1_addr;
    logic [4:0] rs2_addr;
    logic [4:0] rd_addr;
    logic rs1_used;
    logic rs2_used;
    logic rd_used;
    logic [3:0] alu_fun;
    logic memWrite;
    logic mem_rden;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
    logic [31:0] opA;
    logic [31:0] opB;
    logic [31:0] rs2;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           
    wire [6:0] opcode;
    wire [31:0] pc, pc_value, pc_next, pc_jalr, pc_branch, pc_jal,
        I_immed, S_immed, U_immed, aluBin, aluAin, aluResult, rfIn, csr_reg, mem_data, rs1, rs2;
    
    wire [31:0] ir;
    
    wire pcWrite, regWrite, memWrite, op1_sel, mem_op, pcWriteCond, mem_rden1;
    wire [1:0] opB_sel, rf_sel, wb_sel, mSize;
    wire [3:0] alu_fun;
    wire [2:0] pc_sel;
    wire opA_sel, pc_int, mem_rden;
    
    wire br_lt, br_eq, br_ltu;
   
    
//==== Instruction Fetch ===========================================

     logic [31:0] if_de_pc;
     
     always_ff @(posedge CLK) begin //pipeline register
        if_de_pc <= pc;
     end
     
     assign pcWrite = 1'b1; 	//Hardwired high, assuming now hazards
     assign mem_rden1 = 1'b1; 	//Fetch new instruction every cycle
    
    // Instantiate the PC and connect relevant I/O
    PC OTTER_PC(.CLK(CLK), .RST(pc_int), .PC_WRITE(pcWrite), .PC_SEL(pc_sel),
        .JALR(pc_jalr), .JAL(pc_jal), .BRANCH(pc_branch), .MTVEC(32'b0), .MEPC(32'b0),
        .PC_OUT(pc), .PC_OUT_INC(pc_next));

    logic [13:0] addr1;
    assign addr1 = pc[15:2];
    
//******CONFUSED ABOUT THESE BELOW*******

    logic sign;
    assign sign = ir[14]; //before init?
    logic [1:0] size;
    assign size = ir[13:12]; // before init ??
    
    // Instantiate the Memory module and connect relevant I/O    
    Memory OTTER_MEMORY(.MEM_CLK(CLK), .MEM_RDEN1(mem_rden1), .MEM_RDEN2(ex_mem_inst.mem_rden), 
        .MEM_WE2(ex_mem_inst.memWrite), .MEM_ADDR1(addr1), .MEM_ADDR2(IOBUS_ADDR), .MEM_DIN2(ex_mem_rs2), .MEM_SIZE(size),
         .MEM_SIGN(sign), .IO_IN(IOBUS_IN), .IO_WR(IOBUS_WR), .MEM_DOUT1(ir), .MEM_DOUT2(mem_data));
    
     
//==== Instruction Decode ===========================================

    logic [31:0] de_ex_opA;
    logic [31:0] de_ex_opB;
    logic [31:0] de_ex_rs2;

    instr_t de_ex_inst, de_inst;
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(opcode);
    
    assign de_inst.rs1_addr = ir[19:15];
    assign de_inst.rs2_addr = ir[24:20];
    assign de_inst.rd_addr = ir[11:7];
    assign de_inst.opcode = OPCODE;
   
    assign de_inst.rs1_used =   de_inst.rs1_addr != 0
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
//  assign de_inst.rs2_used;
//  assign de_inst.rd_used;
//  assign de_inst.mem_type;  //sign, size
    assign de_inst.pc = if_de_pc;
    
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
    
     always_ff@(posedge CLK) begin
        de_ex_inst <= de_inst;
//        assign de_ex_inst.memWrite = memWrite;
//        assign de_ex_inst.alu_fun = alu_fun;
//        assign de_ex_inst.mem_rden = mem_rden;
//        assign de_ex_inst.regWrite = regWrite;
	end
	
	//Instantiate RegFile
    REG_FILE OTTER_REG_FILE(.CLK(CLK), .EN(mem_wb_inst.regWrite), .ADR1(de_inst.rs1_addr), .ADR2(de_inst.rs2_addr), .WA(mem_wb_inst.rd_addr), 
        .WD(reg_wd), .RS1(rs1), .RS2(de_ex_inst.rs2));

	//Instantiate Decoder
	//[TODO] Move BR_LT, BR_LTU, BR_EQ and their conditions to just 
    CU_DCDR OTTER_DCDR(.IR_30(ir30), .IR_OPCODE(opcode), .IR_FUNCT(funct), .BR_EQ(br_eq), 
        .BR_LT(br_lt), .BR_LTU(br_ltu), .ALU_FUN(de_ex_inst.alu_fun), .ALU_SRCA(alu_src_a), 
        .ALU_SRCB(alu_src_b), .PC_SOURCE(pc_sel), .RF_WR_SEL(rf_wr_sel), .PC_WRITE(pc_write), 
        .REG_WRITE(de_ex_inst.regWrite), .MEM_WE2(de_ex_inst.memWrite), .MEM_RDEN1(de_ex_inst.mem_rden), .MEM_RDEN2(mem_rden), .PC_RST(pc_int));

    //Create logic for Immediate Generator outputs and BAG and ALU MUX inputs    
    logic [31:0] Utype, Itype, Stype, Btype, Jtype;
    
    //Instantiate Immediate Generator, connect all relevant I/O
    ImmediateGenerator OTTER_IMGEN(.IR(imgen_ir), .U_TYPE(Utype), .I_TYPE(Itype), .S_TYPE(Stype),
        .B_TYPE(Btype), .J_TYPE(Jtype));
        
    //Instantiate ALU two-to-one Mux, ALU four-to-one MUX,
    //and ALU; connect all relevant I/O     
    TwoMux OTTER_ALU_MUXA(.SEL(alu_src_a), .RS1(rs1), .U_TYPE(Utype), .OUT(de_ex_inst.opA));
    FourMux OTTER_ALU_MUXB(.SEL(alu_src_b), .ZERO(de_ex_inst.rs2), .ONE(Itype), .TWO(Stype), .THREE(de_inst.pc), .OUT(de_ex_inst.opB));
//==== Execute ======================================================

    logic [31:0] ex_mem_rs2;
    logic [31:0] ex_mem_aluRes;
    logic [31:0] opA_forwarded;
    logic [31:0] opB_forwarded;
    
    instr_t ex_mem_inst;
    
    always_ff@(posedge CLK) begin
        ex_mem_inst <= de_ex_inst;
//        assign opA_forwarded = opA;
//        assign opB_forwarded = opB;  
//        assign ex_mem_rs2 = rs2;     
	end

    // Instantiate ALU
    ALU OTTER_ALU(.SRC_A(de_ex_inst.opA), .SRC_B(de_ex_inst.opB), .ALU_FUN(de_ex_inst.alu_fun), .RESULT(ex_mem_aluRes));
    
	//Instantiate Branch Condition Generator
    BCG OTTER_BCG(.RS1(rs1), .RS2(IOBUS_OUT), .BR_EQ(br_eq), .BR_LT(br_lt), .BR_LTU(br_ltu));
	
    // Instantiate Branch Address Generator
    BAG OTTER_BAG(.RS1(rs1), .I_TYPE(Itype), .J_TYPE(Jtype), .B_TYPE(Btype), .FROM_PC(pc_out),
                  .JAL(jal), .JALR(jalr), .BRANCH(branch));

//==== Memory ======================================================
     
    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;
    
    instr_t mem_wb_inst;
 
    always_ff@(posedge CLK) begin
        mem_wb_inst <= ex_mem_inst;
	end
     
//==== Write Back ==================================================
     

    //Instantiate RegFile Mux, connect all relevant I/O
//    logic [31:0] reg_wd, rs1; // in wb state
//    FourMux OTTER_REG_MUX(.SEL(rf_wr_sel), .ZERO(pc_next), .ONE(32'b0), .TWO(mem_data), .THREE(IOBUS_ADDR),
//        .OUT(reg_wd));
       
            
endmodule
