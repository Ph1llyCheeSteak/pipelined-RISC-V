`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
//
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
    logic memRead2;
    logic regWrite;
    logic [1:0] rf_wr_sel;
    logic [2:0] mem_type;  //sign, size
    logic [31:0] pc;
    logic [31:0] jalr;
    logic [31:0] branch;
    logic [31:0] jump;
    logic [31:0]U_immed, I_immed, S_immed, J_type, B_type;
    logic [2:0] pc_sel;
    logic [31:0] rs1;
} instr_t;

module OTTER_MCU(input CLK,
                input INTR,
                input RESET,
                input [31:0] IOBUS_IN,
                output [31:0] IOBUS_OUT,
                output [31:0] IOBUS_ADDR,
                output logic IOBUS_WR 
);           

    wire  [31:0] next_pc, aluBin, aluAin, aluResult, rfIn, mem_data;
    wire [31:0] IR, HazardAout, HazardBout;
    wire [1:0] opB_sel;
    wire opA_sel;
    
    logic [31:0] pc, B_type, J_type, rs2;
    logic [2:0]  BCG_PC_SOURCE;
    logic [1:0] ForwardA, ForwardB;
    logic br_lt,br_eq,br_ltu;
    logic stall, stalled, stalled2, flush, flushed;
    logic pcWrite, memRead1;
    

    // DE_EX
    logic [31:0] de_ex_next_pc;
    logic de_ex_opA_sel;
    logic [1:0] de_ex_opB_sel;
    logic [31:0] de_ex_rs2;
    logic [31:0] de_ex_pc;

    // EX_MEM
    instr_t ex_mem_inst;
    logic [31:0] ex_mem_rs2;
    logic [31:0] ex_mem_next_pc;
    logic [31:0] ex_mem_HazardBout;
    logic [31:0] ex_mem_aluRes;
    
    // MEM_WB
    instr_t mem_wb_inst;
    logic [31:0] mem_wb_aluRes;
    logic [31:0] mem_wb_next_pc;

              
//==== Instruction Fetch ===========================================
    logic [2:0] PCSOURCEIN;
    assign PCSOURCEIN = de_ex_inst.pc_sel | BCG_PC_SOURCE;

    PC PC  (
       .CLK        (CLK),
       .RST        (RESET),
       .PC_WRITE   (pcWrite),
       .PC_SEL     (PCSOURCEIN),
       .JALR       (de_ex_inst.jalr),
       .BRANCH     (de_ex_inst.branch),
       .JAL        (de_ex_inst.jump),
       .MTVEC      (),
       .MEPC       (),
       .PC_OUT     (pc),
       .PC_OUT_INC (next_pc)
    );

    logic [31:0] if_de_pc;
    logic [31:0] if_de_next_pc;
    instr_t de_ex_inst, de_inst;

    always_comb begin
      if (stall == 1'b0) begin
            pcWrite <= 1'b1;
            memRead1 <= 1'b1;
        end
        else begin
            pcWrite <= 1'b0;
            memRead1 <= 1'b0;
        end
    end  
    
    always_ff @(posedge CLK) begin
        if(stall == 0) begin
            if_de_pc <= pc;
            if_de_next_pc <= next_pc; 
            stalled <= 1'b0;
        end
        else if(stall == 1) begin
            stalled <=1'b1;
        end   
    end
    
    always_ff @(posedge CLK) begin
        if (stalled) begin
            stalled2 <= 1'b1;
        end
        else begin
            stalled2 <=1'b0;
        end
end    

//==== Decode ===========================================
    
    opcode_t OPCODE;
    assign OPCODE = opcode_t'(IR[6:0]); //assign OPCODE_t = opcode_t'(opcode);
    
    assign de_inst.rs1_addr=IR[19:15];
    assign de_inst.rs2_addr=IR[24:20];
    assign de_inst.rd_addr=IR[11:7];
    assign de_inst.opcode=OPCODE;
    
    assign de_inst.mem_type=IR[14:12];

//==== Hazard Detection ===========================================
   wire  BCGMUXSelA, BCGMUXSelB;
   
    assign de_inst.rs1_used=    de_inst.rs1_addr != 0    //erm... shouldnt this be address?
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
    
    assign de_inst.rs2_used=    de_inst.rs2_addr != 0   //same logic here but w/ rs2 readwrites
                                && de_inst.opcode != OP_IMM
                                && de_inst.opcode != LUI
                                && de_inst.opcode != AUIPC
                                && de_inst.opcode != JAL;
                                
    assign de_inst.rd_used=     de_inst.rd_addr != 0    //heh. rd means read dest i think so these
                                && de_inst.opcode != BRANCH
                                && de_inst.opcode != STORE;
                                
    // Instantiate Hazard Unit
    Hazard_Detection Hazard_Detection_Unit(
        // RS1 AND RS2                      
        .rs1_d          (de_inst.rs1_addr),
        .rs2_d          (de_inst.rs2_addr),
        .de_rs1_used    (de_inst.rs1_used), 
        .de_rs2_used    (de_inst.rs2_used),
        .rs1_e          (de_ex_inst.rs1_addr),
        .rs2_e          (de_ex_inst.rs2_addr),
        .de_ex_rs1_used (de_ex_inst.rs1_used),
        .de_ex_rs2_used (de_ex_inst.rs2_used),
        // RD 
        .id_ex_rd       (de_ex_inst.rd_addr),
        .de_ex_rd_used  (de_ex_inst.rd_used),
        .mem_rd_used    (ex_mem_inst.rd_used),          
        .wb_rd_used     (mem_wb_inst.rd_used),
        .ex_mem_rd      (ex_mem_inst.rd_addr),
        .mem_wb_rd      (mem_wb_inst.rd_addr),
        // OTHER
        .ex_mem_regWrite    (ex_mem_inst.regWrite),
        .mem_wb_regWrite    (mem_wb_inst.regWrite),
        .memRead2       (de_ex_inst.memRead2),
        .stalled        (stalled),
        .stalled2       (stalled2),
        .pcSource       (PCSOURCEIN),
        .ForwardA       (ForwardA),
        .ForwardB       (ForwardB),
        .stall          (stall),
        .flush          (flush)
    );

//==== End of Hazard Detection ===========================================
 
    // Instantiate Decoder
    CU_DCDR CU_DCDR (
        .IR_30      (IR[30]),
        .IR_OPCODE  (OPCODE),
        .IR_FUNCT   (IR[14:12]),
        .BR_EQ      (br_eq),       
        .BR_LT      (br_lt),
        .BR_LTU     (br_ltu),
        .ALU_FUN    (de_inst.alu_fun),
        .ALU_SRCA   (opA_sel),
        .ALU_SRCB   (opB_sel), 
        .PC_SOURCE  (de_inst.pc_sel),
        .RF_WR_SEL  (de_inst.rf_wr_sel),
        .REG_WRITE  (de_inst.regWrite),
        .MEM_WRITE  (de_inst.memWrite),
        .MEM_READ_2 (de_inst.memRead2)
    );
    
    // Instantiate Immediate Generator
    ImmediateGenerator immgen(
        .IR     (IR[31:7]),
        .U_TYPE (de_inst.U_immed),
        .I_TYPE (de_inst.I_immed),
        .S_TYPE (de_inst.S_immed),
        .B_TYPE (de_inst.B_type),
        .J_TYPE (de_inst.J_type)
    );

    // Instantiate Register
    REG_FILE regfile(
        .CLK    (CLK),
        .EN     (mem_wb_inst.regWrite),
        .ADR1   (de_inst.rs1_addr),
        .ADR2   (de_inst.rs2_addr),
        .WA     (mem_wb_inst.rd_addr),
        .WD     (rfIn),
        .RS1    (de_inst.rs1),
        .RS2    (rs2)
    );

	always_ff @ (posedge CLK) begin
	    if(flush) begin
	   	    de_ex_inst <= 0;

            de_ex_opA_sel <= 0;
            de_ex_opB_sel <= 0;
            de_ex_rs2 <= 0;
               
            de_ex_next_pc <= 0;
            de_ex_pc <= 0;
            flushed <= 1;
	    end
	    else if(flushed) begin
	        de_ex_inst <= 0;
               
            de_ex_opA_sel <= 0;
            de_ex_opB_sel <= 0;
            de_ex_rs2 <= 0;
              
            de_ex_next_pc <= 0;
            de_ex_pc <= 0; 
            flushed <= 0;
        end     
        else if(stall) begin
	        de_ex_inst <= de_ex_inst;      
            de_ex_rs2 <= de_ex_rs2;
            de_ex_pc <= de_ex_pc;	       
               
            de_ex_opA_sel <= de_ex_opA_sel;
            de_ex_opB_sel <= de_ex_opB_sel;
            de_ex_next_pc <= de_ex_next_pc;
	    end
	    else begin
            de_ex_inst <= de_inst;
            de_ex_rs2 <= rs2;
             
            de_ex_opA_sel <= opA_sel;
            de_ex_opB_sel <= opB_sel;	       
            de_ex_next_pc <= if_de_next_pc;
            de_ex_pc <= if_de_pc;
	    end
	end

//==== Execute ======================================================
    
    // Instantiate ALU
    ALU ALU (
        .ALU_FUN    (de_ex_inst.alu_fun),  
        .SRC_A      (aluAin),
        .SRC_B      (aluBin), 
        .RESULT     (aluResult)
     );
    
    // Instantiate Branch Condition Generator
    BCG BCG(
        .RS1        (HazardAout),
        .RS2        (HazardBout),
        .func3      (de_ex_inst.mem_type),
        .opcode     (de_ex_inst.opcode),
        .PC_SOURCE  (BCG_PC_SOURCE)
    );
        
    // Instantiate Branch Address Generator
    BAG BAG(
        .RS1        (HazardAout),
        .I_TYPE     (de_ex_inst.I_immed),
        .J_TYPE     (de_ex_inst.J_type),
        .B_TYPE     (de_ex_inst.B_type),
        .FROM_PC    (de_ex_pc), //?
        .JAL        (de_ex_inst.jump),
        .BRANCH     (de_ex_inst.branch),
        .JALR       (de_ex_inst.jalr)  
    );
    
    // Instantiate Hazard MUX A
    FourMux HazardMUXA (
       .ZERO        (de_ex_inst.rs1),     
       .ONE         (rfIn),
       .TWO         (ex_mem_aluRes),
       .THREE       (31'b0),
       .SEL         (ForwardA),
       .OUT         (HazardAout)
    );
    
    // Instantiate Hazard MUX B
    FourMux HazardMUXB (  
       .ZERO        (de_ex_rs2),
       .ONE         (rfIn),
       .TWO         (ex_mem_aluRes),
       .THREE       (31'b0),
       .SEL         (ForwardB),
       .OUT         (HazardBout)
    );

    // Instantiate ALU MUX A
    TwoMux muxAluA  (
        .SEL        (de_ex_opA_sel),   
        .RS1        (HazardAout),
        .U_TYPE     (de_ex_inst.U_immed),
        .OUT        (aluAin)  
    );

    // Instantiate ALU MUX B   
    FourMux muxAluB (
        .SEL    (de_ex_opB_sel),
        .ZERO   (HazardBout),
        .ONE    (de_ex_inst.I_immed),
        .TWO    (de_ex_inst.S_immed),
        .THREE  (de_ex_pc),
        .OUT    (aluBin)  
    );

    always_ff @ (posedge CLK) begin  
        begin
            ex_mem_inst <= de_ex_inst;
            ex_mem_rs2 <= de_ex_rs2;
            ex_mem_aluRes <= aluResult;
            ex_mem_next_pc <= de_ex_next_pc;
            ex_mem_HazardBout <= HazardBout;
        end
    end

//==== Memory ======================================================

    OTTER_mem_byte Memdual (
        .MEM_CLK    (CLK),
        .MEM_READ1  (memRead1),  //IF
        .MEM_READ2  (ex_mem_inst.memRead2),
        .MEM_WRITE2 (ex_mem_inst.memWrite),        
        .MEM_ADDR1  (pc),  //IF.
        .MEM_ADDR2  (ex_mem_aluRes),          
        .MEM_DIN2   (ex_mem_HazardBout),
        .MEM_SIZE   (ex_mem_inst.mem_type[1:0]),     
        .MEM_SIGN   (ex_mem_inst.mem_type[2]),          
        .IO_IN      (IOBUS_IN),
        .IO_WR      (IOBUS_WR),
        .MEM_DOUT1  (IR),
        .MEM_DOUT2  (mem_data),
        .ERR        (memERR)
    );     

    assign IOBUS_ADDR = ex_mem_aluRes;
    assign IOBUS_OUT = ex_mem_rs2;

    always_ff @ (posedge CLK) begin
        mem_wb_inst <= ex_mem_inst;
        mem_wb_next_pc <= ex_mem_next_pc;
        mem_wb_aluRes <= ex_mem_aluRes;
    end
//==== Write Back ==================================================
     
    FourMux regMux (
        .SEL   (mem_wb_inst.rf_wr_sel),
        .ZERO  (next_pc), //mem_wb_next_pc?
        .ONE   (0),
        .TWO   (mem_data),
        .THREE (mem_wb_aluRes),
        .OUT   (rfIn)  
    );
endmodule
