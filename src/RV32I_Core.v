`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/21/2024 02:28:30 AM
// Design Name: 
// Module Name: RV32I_Core
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
`include "RV32I_Headers.vh"

module RV32I_Core #(parameter PC_RESET=32'h0000_0000,USE_CACHE = 0) (
    input i_clk,i_rst_n,

    //Instruction Mem Interface
    input   [31:0] i_inst,
    output  [31:0] o_iaddr
    //`ifdef USE_CACHE
      ,input   i_cache_stall
      ,input   i_cache_valid
    //`else
      ,output  o_imem_req
      ,input   i_ack_inst,
    //`endif
    //Data Memory Interface
    output  o_wb_cyc_data,          //Bus cycle Active
    output  o_wb_stb_data,  //Request for read/Write
    output  o_wb_we_data,   //Write-enable (1=Write,0=Read)
    output  [31:0] o_wb_addr_data, //Addr to Memory
    output  [31:0] o_wb_data_data, //data to be stored to memory
    output  [3:0] o_wb_sel_data, //byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
    input   i_wb_ack_data, //ack by data memory (high when read data is ready or when write data is already written)
    input   i_wb_stall_data, //stall by data memory 
    input   [31:0] i_wb_data_data, //data retrieve from memory
    
    //Interrupts
    input   i_external_interrupt,
    input   i_software_interrupt,
    input   i_timer_interrupt,
    output end_of_Instr_WB

    );

    //wires for basereg
    wire[31:0] rs1_orig,rs2_orig;   
    wire[31:0] rs1,rs2;  
    wire ce_read;

    //wires for rv32i_fetch
     wire[31:0] IF_pc;
     wire[31:0] IF_inst;
     wire end_of_Instr;

    //wires for rv32i_decoder
    wire[`ALU_WIDTH-1:0] ID_AluOp;
    wire[`OPCODE_WIDTH-1:0] ID_opcode;
    wire[31:0] ID_pc;
    wire[4:0] decoder_rs1_addr, decoder_rs2_addr;
    wire[4:0] decoder_rs1_addr_q, decoder_rs2_addr_q;
    wire[4:0] decoder_rd_addr; 
    wire[31:0] decoder_imm; 
    wire[2:0] decoder_funct3;
    wire[`EXCEPTION_WIDTH-1:0] decoder_exception;
    wire decoder_ce;
    wire decoder_flush;
    wire end_of_Instr_ID;

    //wires for rv32i_alu
    wire[`OPCODE_WIDTH-1:0] alu_opcode;
    wire[4:0] alu_rs1_addr;
    wire[4:0] alu_rs2_addr;
    wire[31:0] alu_rs1;
    wire[31:0] alu_rs2;
    wire[11:0] alu_imm;
    wire[2:0] alu_funct3;
    wire[31:0] alu_y;
    wire[31:0] alu_pc;
    wire[31:0] alu_next_pc;
    wire alu_change_pc;
    wire alu_wr_rd;
    wire[4:0] alu_rd_addr;
    wire[31:0] alu_rd;
    wire alu_rd_valid;
    wire[`EXCEPTION_WIDTH-1:0] alu_exception;
    wire alu_ce;
    wire alu_flush;
    wire alu_force_stall;
    wire end_of_Instr_IE;

    //wires for rv32i_memoryaccess
    wire [31:0] o_mem_rs2_store_data;
    wire[`OPCODE_WIDTH-1:0] memoryaccess_opcode;
    wire[2:0] memoryaccess_funct3;
    wire[31:0] memoryaccess_pc;
    wire memoryaccess_wr_rd;
    wire[4:0] memoryaccess_rd_addr;
    wire[31:0] memoryaccess_rd;
    wire[31:0] memoryaccess_data_load;
    wire memoryaccess_wr_mem;
    wire memoryaccess_ce;
    wire memoryaccess_flush;
    wire o_stall_from_alu;
    wire end_of_Instr_IMEM;
    wire o_store_data_mux_sel;

    //wires for rv32i_writeback
    wire writeback_wr_rd; 
    wire[4:0] writeback_rd_addr; 
    wire[31:0] writeback_rd;
    wire[31:0] writeback_next_pc;
    wire writeback_change_pc;
    wire writeback_ce;
    wire writeback_flush;
    //wire end_of_Instr_WB;

    //wires for rv32i_csr
    wire[31:0] csr_out; //CSR value to be stored to basereg
    wire[31:0] csr_return_address; //mepc CSR
    wire[31:0] csr_trap_address; //mtvec CSR
    wire csr_go_to_trap; //high before going to trap (if exception/interrupt detected)
    wire csr_return_from_trap; //high before returning from trap (via mret)
    
    wire stall_decoder,
         stall_alu,
         stall_memoryaccess,
         stall_writeback; //control stall of each pipeline stages
    assign ce_read = decoder_ce && !stall_decoder; //reads basereg only decoder is not stalled 

    //module instantiations
    FORWARDING_UNIT operand_forwarding ( //logic for operand forwarding
        .i_rs1_orig(rs1_orig), //current rs1 value saved in basereg
        .i_rs2_orig(rs2_orig), //current rs2 value saved in basereg
        .i_decoder_rs1_addr_q(decoder_rs1_addr_q), //address of operand rs1 used in ALU stage
        .i_decoder_rs2_addr_q(decoder_rs2_addr_q), //address of operand rs2 used in ALU stage
        .o_alu_force_stall(alu_force_stall), //high to force ALU stage to stall
        .o_rs1(rs1), //rs1 value with Operand Forwarding
        .o_rs2(rs2), //rs2 value with Operand Forwarding
        .IE_opcode(ID_opcode),
        .MEM_opcode(alu_opcode),
        .i_execute_rs2_addr_q(alu_rs2_addr),
        .i_alu_rs2(alu_rs2),
        .o_mem_rs2_store_data(o_mem_rs2_store_data),
        // Stage 4 [MEMORYACCESS]
        .i_alu_rd_addr(alu_rd_addr), //destination register address
        .i_alu_wr_rd(alu_wr_rd), //high if rd_addr will be written
        .i_alu_rd_valid(alu_rd_valid), //high if rd is already valid at this stage (not LOAD nor CSR instruction)
        .i_alu_rd(alu_rd), //rd value in stage 4
        .i_memoryaccess_ce(memoryaccess_ce), //high if stage 4 is enabled
        // Stage 5 [WRITEBACK]
        .i_memoryaccess_rd_addr(memoryaccess_rd_addr), //destination register address
        .i_memoryaccess_wr_rd(memoryaccess_wr_rd), //high if rd_addr will be written
        .i_writeback_rd(writeback_rd), //rd value in stage 5
        .i_writeback_ce(writeback_ce) //high if stage 4 is enabled
    );

    BASEREG RF_Controller( //regfile controller for the 32 integer base registers
        .i_clk(i_clk),
        .i_rstn(i_rst_n),
        .i_ce_read(ce_read), //clock enable for reading from basereg [STAGE 2]
        .i_rs1_addr(decoder_rs1_addr), //source register 1 address
        .i_rs2_addr(decoder_rs2_addr), //source register 2 address
        .i_rd_addr(writeback_rd_addr), //destination register address
        .i_rd(writeback_rd), //data to be written to destination register
        .i_wr(writeback_wr_rd), //write enable
        .o_rs1(rs1_orig), //source register 1 value
        .o_rs2(rs2_orig) //source register 2 value
    );
   
   //-----------------IF Stage----------------------------------------- 
    FETCH #(.PC_RESET(PC_RESET),.USE_CACHE(USE_CACHE)) IF_STAGE( // logic for fetching instruction [FETCH STAGE , STAGE 1]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .o_iaddr(o_iaddr), //Instruction address
        .o_pc(IF_pc), //PC value of o_inst
        .i_inst(i_inst), // retrieved instruction from Memory
        .o_inst(IF_inst), // instruction
        //`ifdef USE_CACHE
          .i_cache_valid(i_cache_valid),
          .i_cache_stall(i_cache_stall),
        //`else
          .o_stb_inst(o_imem_req), // request for instruction
          .i_ack_inst(i_ack_inst), //ack (high if new instruction is ready)
        //`endif          
          //.i_cache_stall(i_cache_stall),
        // PC Control
        //.i_writeback_change_pc(1'b0), //high when PC needs to change when going to trap or returning from trap
        //.i_writeback_next_pc(1'b0), //next PC due to trap
        .i_alu_change_pc(alu_change_pc), //high when PC needs to change for taken branches and jumps
        .i_alu_next_pc(alu_next_pc), //next PC due to branch or jump
        /// Pipeline Control ///
        .o_ce(decoder_ce), // output clk enable for pipeline stalling of next stage
        .i_stall((stall_decoder || stall_alu || stall_memoryaccess || stall_writeback || i_cache_stall)), //informs this stage to stall
        .i_flush(decoder_flush), //flush this stage
        .end_of_Instr(end_of_Instr)
    ); 
  
   //-----------------ID Stage----------------------------------------- 
    DECODE ID_STAGE( //logic for the decoding of the 32 bit instruction [DECODE STAGE , STAGE 2]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_inst(IF_inst), //32 bit instruction
        .i_pc(IF_pc), //PC value from fetch stage
        .o_pc(ID_pc), //PC value
        .o_rs1_addr(decoder_rs1_addr),// address for register source 1
        .o_rs1_addr_q(decoder_rs1_addr_q), // registered address for register source 1
        .o_rs2_addr(decoder_rs2_addr), // address for register source 2
        .o_rs2_addr_q(decoder_rs2_addr_q), // registered address for register source 2
        .o_rd_addr(decoder_rd_addr), // address for destination register
        .o_imm(decoder_imm), // extended value for immediate
        .o_funct3(decoder_funct3), // function type
        .o_alu(ID_AluOp), //alu operation type
        .o_opcode(ID_opcode), //opcode type
        .o_exception(decoder_exception), //exceptions: illegal inst, ecall, ebreak, mret
         /// Pipeline Control ///
        .i_ce(decoder_ce), // input clk enable for pipeline stalling of this stage
        .o_ce(alu_ce), // output clk enable for pipeline stalling of next stage
        .i_stall((stall_alu || stall_memoryaccess || stall_writeback)), //informs this stage to stall
        .o_stall(stall_decoder), //informs pipeline to stall
        .i_flush(alu_flush), //flush this stage
        .o_flush(decoder_flush), //flushes previous stages
        .end_of_Instr(end_of_Instr),
        .end_of_Instr_ID(end_of_Instr_ID)
    );

   //-----------------IE Stage----------------------------------------- 
    EXECUTE IE_STAGE( //ALU combinational logic [EXECUTE STAGE , STAGE 3]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_alu(ID_AluOp), //alu operation type
        .i_rs1_addr(decoder_rs1_addr_q), //address for register source 1
        .o_rs1_addr(alu_rs1_addr), //address for register source 1
        .i_rs2_addr(decoder_rs2_addr_q), //address for register source 1
        .o_rs2_addr(alu_rs2_addr), //address for register source 1
        .i_rs1(rs1), //Source register 1 value
        .o_rs1(alu_rs1), //Source register 1 value
        .i_rs2(rs2), //Source Register 2 value
        .o_rs2(alu_rs2), //Source Register 2 value
        .i_imm(decoder_imm), //Immediate value from previous stage
        .o_imm(alu_imm), //Immediate value
        .i_funct3(decoder_funct3), //function type from decoder stage
        .o_funct3(alu_funct3), //function type
        .i_opcode(ID_opcode), //opcode type from previous stage
        .o_opcode(alu_opcode), //opcode type
        .i_exception(decoder_exception), //exception from decoder stage
        .o_exception(alu_exception), //exception: illegal inst,ecall,ebreak,mret
        .o_y(alu_y), //result of arithmetic operation
        // PC Control
        .i_pc(ID_pc), //pc from decoder stage
        .o_pc(alu_pc), // current pc 
        .o_next_pc(alu_next_pc), //next pc 
        .o_change_pc(alu_change_pc), //change pc if high
        // Basereg Control
        .o_wr_rd(alu_wr_rd), //write rd to basereg if enabled
        .i_rd_addr(decoder_rd_addr), //address for destination register (from previous stage)
        .o_rd_addr(alu_rd_addr), //address for destination register
        .o_rd(alu_rd), //value to be written back to destination register
        .o_rd_valid(alu_rd_valid), //high if o_rd is valid (not load nor csr instruction)
         /// Pipeline Control ///
        .o_stall_from_alu(o_stall_from_alu), //prepare to stall next stage(memory-access stage) for load/store instruction
        .i_ce(alu_ce), // input clk enable for pipeline stalling of this stage
        .o_ce(memoryaccess_ce), // output clk enable for pipeline stalling of next stage
        .i_stall((stall_memoryaccess || stall_writeback)), //informs this stage to stall
        .i_force_stall(alu_force_stall), //force this stage to stall
        .o_stall(stall_alu), //informs pipeline to stall
        .i_flush(memoryaccess_flush), //flush this stage
        .o_flush(alu_flush), //flushes previous stages
        .end_of_Instr_ID(end_of_Instr_ID),
        .end_of_Instr_IE(end_of_Instr_IE)
    );
    
   //-----------------MEM Stage----------------------------------------- 
    MEMORY_STAGE MEM_STAGE( //logic controller for data memory access (load/store) [MEMORY STAGE , STAGE 4]
        .i_clk(i_clk),
        .i_rst_n(i_rst_n),
        .i_rs2(o_mem_rs2_store_data), //data to be stored to memory is always rs2
        .i_y(alu_y), //y value from ALU (address of data to memory be stored or loaded)
        .i_funct3(alu_funct3), //funct3 from previous stage
        .o_funct3(memoryaccess_funct3), //funct3 (byte,halfword,word)
        .i_opcode(alu_opcode), //opcode type from previous stage
        .o_opcode(memoryaccess_opcode), //opcode type
        .i_pc(alu_pc), //PC from previous stage
        .o_pc(memoryaccess_pc), //PC value
        .o_store_data_mux_sel(o_store_data_mux_sel),
        .i_writeback_rd(writeback_rd),
        // Basereg Control
        .i_wr_rd(alu_wr_rd), //write rd to base reg is enabled (from memoryaccess stage)
        .o_wr_rd(memoryaccess_wr_rd), //write rd to the base reg if enabled
        .i_rd_addr(alu_rd_addr), //address for destination register (from previous stage)
        .o_rd_addr(memoryaccess_rd_addr), //address for destination register
        .i_rd(alu_rd), //value to be written back to destination reg
        .o_rd(memoryaccess_rd), //value to be written back to destination register
        // Data Memory Control
        .o_wb_cyc_data(o_wb_cyc_data), //bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
        .o_wb_stb_data(o_wb_stb_data), //request for read/write access to data memory
        .o_wb_we_data(o_wb_we_data),  //write-enable (1 = write, 0 = read)
        .o_wb_addr_data(o_wb_addr_data), //data memory address
        .o_wb_data_data(o_wb_data_data), //data to be stored to memory (mask-aligned)
        .o_wb_sel_data(o_wb_sel_data), //byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
        .i_wb_ack_data(i_wb_ack_data), //ack by data memory (high when read data is ready or when write data is already written
        .i_wb_stall_data(i_wb_stall_data), //stall by data memory (1 = data memory is busy)
        .i_wb_data_data(i_wb_data_data), //data retrieve from data memory 
        .o_data_load(memoryaccess_data_load), //data to be loaded to base reg (z-or-s extended) 
         /// Pipeline Control ///   
        .i_stall_from_alu(o_stall_from_alu), //stalls this stage when incoming instruction is a load/store
        .i_ce(memoryaccess_ce), // input clk enable for pipeline stalling of this stage
        .o_ce(writeback_ce), // output clk enable for pipeline stalling of next stage
        .i_stall(stall_writeback), //informs this stage to stall
        .o_stall(stall_memoryaccess), //informs pipeline to stall
        .i_flush(writeback_flush), //flush this stage
        .o_flush(memoryaccess_flush), //flushes previous stages
        .end_of_Instr_IE(end_of_Instr_IE),
        .end_of_Instr_IMEM(end_of_Instr_IMEM)
    );
    
    
   //-----------------WB Stage----------------------------------------- 
    WRITEBACK WB_STAGE( //logic controller for the next PC and rd value [WRITEBACK STAGE , STAGE 5]
        .i_funct3(memoryaccess_funct3), //function type
        .i_data_load(memoryaccess_data_load), //data to be loaded to base reg (from previous stage)
        .i_csr_out(csr_out), //CSR value to be loaded to basereg
        .i_opcode_load(memoryaccess_opcode[`LOAD]),
        .i_opcode_system(memoryaccess_opcode[`SYSTEM]), 
        // Basereg Control
        .i_wr_rd(memoryaccess_wr_rd), //write rd to base reg is enabled (from memoryaccess stage)
        .o_wr_rd(writeback_wr_rd), //write rd to the base reg if enabled
        .i_rd_addr(memoryaccess_rd_addr), //address for destination register (from previous stage)
        .o_rd_addr(writeback_rd_addr), //address for destination register
        .i_rd(memoryaccess_rd), //value to be written back to destination reg
        .o_rd(writeback_rd), //value to be written back to destination register
        // PC Control
        .i_pc(memoryaccess_pc), //pc value
        //.o_next_pc(writeback_next_pc), //new PC value
        //.o_change_pc(writeback_change_pc), //high if PC needs to jump
        // Trap-Handler
        //.i_go_to_trap(csr_go_to_trap), //high before going to trap (if exception/interrupt detected)
        //.i_return_from_trap(csr_return_from_trap), //high before returning from trap (via mret)
        //.i_return_address(csr_return_address), //mepc CSR
        //.i_trap_address(csr_trap_address), //mtvec CSR
        /// Pipeline Control ///
        .i_ce(writeback_ce), // input clk enable for pipeline stalling of this stage
        .o_stall(stall_writeback), //informs pipeline to stall
        .o_flush(writeback_flush), //flushes previous stages 
        .end_of_Instr_IMEM(end_of_Instr_IMEM),
        .end_of_Instr_WB(end_of_Instr_WB)
    );

endmodule
