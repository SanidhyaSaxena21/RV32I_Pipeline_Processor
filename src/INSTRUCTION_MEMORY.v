`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/23/2024 05:55:50 PM
// Design Name: 
// Module Name: INSTRUCTION_MEMORY
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

`ifndef ICAHCE
`define ICACHE 0
`endif

(* DONT_TOUCH = "yes" *)
module INSTRUCTION_MEMORY #(parameter DEPTH = 8192, DELAY_CYCLE = 10, BURST_LENGTH = 4)(
    input i_clk,
    input i_rst_n,
    input [$clog2(DEPTH)-1:0] i_addr,
    input i_stb,
    input burst,
    output reg o_ack,
    output reg [31:0] instr
    );

    reg [31:0] INST_MEMORY [0:DEPTH-1];
    reg [$clog2(DELAY_CYCLE)-1:0] delay_counter;
  
    initial begin
      //$readmemh("Instruction_Load_Store.mem",INST_MEMORY);
      $readmemh("Instruction_Greatest.mem",INST_MEMORY);
    end
    initial begin
      o_ack = 0;
      instr = 32'd0;
    end

    reg [(32*BURST_LENGTH)-1:0] instr_shift_reg;

    always @(posedge i_clk or negedge i_rst_n) begin
      if(!i_rst_n) instr_shift_reg <= 0;
      else begin
        if(burst) instr_shift_reg <= {instr,instr_shift_reg[(32*BURST_LENGTH)-1:32]};
      end
    end

    integer j;
    always @(posedge i_clk) begin
      if(delay_counter == DELAY_CYCLE) begin
        if(burst) begin
          for(j=0;j<BURST_LENGTH;j=j+1) begin
            instr <= instr_shift_reg[32*j+:32];
            o_ack <= 1'b1;
          end
        end
        else begin
          o_ack <= i_stb;
          instr <= INST_MEMORY[i_addr];
        end
      end
      else o_ack <= 1'b0;
    end



    always @(posedge i_clk or negedge i_rst_n) begin
      if(!i_rst_n) delay_counter <= 0;
      else if(i_stb && !o_ack) begin
        if(delay_counter == DELAY_CYCLE) begin
          delay_counter <= 0;
        end
        else delay_counter <= delay_counter + 1;
      end
    end
endmodule
