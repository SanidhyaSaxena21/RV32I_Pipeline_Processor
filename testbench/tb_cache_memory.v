`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/27/2024 03:45:56 PM
// Design Name: 
// Module Name: tb_cache_memory
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


module tb_cache_memory(

    );
    
    parameter MEM_SIZE = 32768;
        //Global Signals
    reg i_clk;
    reg i_rst_n;
    
    // Processor Interface
    reg [$clog2(MEM_SIZE)-1:0] i_cpu_addr;
    wire o_valid;
    wire [31:0] o_instr;
    wire stall_q;

    // Memory Interface
    wire [$clog2(MEM_SIZE)-1:0] o_mem_addr;
    wire  o_stb;
    reg i_ack;
    wire [31:0] i_instr_mem;
    
    ICACHE cache (.i_clk(i_clk),
                  .i_rst_n(i_rst_n),
                  .i_cpu_addr(i_cpu_addr),
                  .o_valid(o_valid),
                  .o_instr(o_instr),
                  .stall_q(stall_q),
                  .o_mem_addr(o_mem_addr),
                  .o_stb(o_stb),
                  .i_ack(o_ack),
                  .i_instr_mem(i_instr_mem));
                  
     INSTRUCTION_MEMORY #(.DEPTH(8192),.DELAY_CYCLE(0)) IMEM (
      .i_clk(i_clk),
      .i_rst_n(i_rst_n),
      .i_addr(o_mem_addr[$clog2(MEM_SIZE)-1:2]),
      .i_stb(o_stb),
      .o_ack(o_ack),
      .instr(i_instr_mem)
     );
     
     always #5 i_clk = ~i_clk;
     
     initial begin
        i_clk = 0;
     end

     initial begin
        i_rst_n = 0;
        
        #100 i_rst_n = 1;
        
        i_cpu_addr = 15'h4;
        #100 i_cpu_addr = 15'hc;
        #20 i_cpu_addr = 15'h10;
     end
endmodule
