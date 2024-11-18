`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/27/2024 03:32:34 PM
// Design Name: 
// Module Name: cache_sram
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


module cache_sram #(parameter WIDTH = 32, DEPTH = 16) (
    input clk,
    input [$clog2(DEPTH)-1:0] addr,
    input [WIDTH-1:0] wdata,
    output [WIDTH-1:0] rdata,
    input wea

    );

    reg [WIDTH-1:0] cache_mem [DEPTH-1:0];

    integer i;
    initial begin
      for(i=0;i<DEPTH;i=i+1) begin
        cache_mem[i] = 32'd0;
      end
    end

    always @(posedge clk) begin
      if(wea) cache_mem[addr] <= wdata;
    end
    
    /*always @(posedge clk) begin
        rdata <= cache_mem[addr];
    end*/


  assign rdata = cache_mem[addr];

endmodule
