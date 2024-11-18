`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/22/2024 06:56:38 PM
// Design Name: 
// Module Name: WRITEBACK
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
module WRITEBACK(
    input wire[2:0] i_funct3, //function type 
    input wire[31:0] i_data_load, //data to be loaded to base reg
    input wire[31:0] i_csr_out, //CSR value to be loaded to basereg
    input wire i_opcode_load,
    input wire i_opcode_system, 
    // Basereg Control
    input wire i_wr_rd, //write rd to basereg if enabled (from previous stage)
    output reg o_wr_rd, //write rd to the base reg if enabled
    input wire[4:0] i_rd_addr, //address for destination register (from previous stage)
    output reg[4:0] o_rd_addr, //address for destination register
    input wire[31:0] i_rd, //value to be written back to destination register (from previous stage)
    output reg[31:0] o_rd, //value to be written back to destination register
    // PC Control
    input wire[31:0] i_pc, // pc value (from previous stage)
    //output reg[31:0] o_next_pc, //new pc value
    //output reg o_change_pc, //high if PC needs to jump
    /// Pipeline Control ///
    input wire i_ce, // input clk enable for pipeline stalling of this stage
    output reg o_stall, //informs pipeline to stall
    output reg o_flush, //flush previous stages
    input end_of_Instr_IMEM,
    output end_of_Instr_WB
    );


    assign end_of_Instr_WB = end_of_Instr_IMEM;

    //determine next value of pc and o_rd
    always @* begin
        o_stall = 0; //stall when this stage needs wait time
        o_flush = 0; //flush this stage along with previous stages when changing PC
        o_wr_rd = i_wr_rd && i_ce && !o_stall;
        o_rd_addr = i_rd_addr;
        o_rd = 0;
        //o_next_pc = 0;
        //o_change_pc = 0;

        if(i_opcode_load) o_rd = i_data_load; //load data from memory to basereg
        else if(i_opcode_system && i_funct3!=0) begin //CSR write
            o_rd = i_csr_out; 
        end
        else o_rd = i_rd; //rd value is already computed at ALU stage
        
    end
endmodule
