`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/04/2024 03:04:28 AM
// Design Name: 
// Module Name: Instruction_mem
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


module Instruction_mem #(parameter MEMORY_DEPTH = 8192)(
    input wire i_clk,
    // Instruction Memory
    input wire[$clog2(MEMORY_DEPTH)-1:0] i_addr,
    output reg[31:0] instr,
    input wire i_stb, // request for instruction
    output reg o_ack //ack (high if new instruction is now on the bus)
);
    reg [31:0] INST_MEMORY [0:MEMORY_DEPTH-1];
    integer i;
    
    initial begin
      $readmemh("Instruction_Greatest.mem",INST_MEMORY);
    end
    
    initial begin //initialize memory to zero
        o_ack <= 0;
        instr <= 0;
    end
    
    //reading must be registered to be inferred as block ram
    always @(posedge i_clk) begin 
        o_ack <= i_stb; //go high next cycle after receiving request (data o_inst_out is also sent at next cycle)
        instr <= INST_MEMORY[{i_addr}]; //read instruction   
    end


endmodule
