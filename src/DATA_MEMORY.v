`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/23/2024 05:56:04 PM
// Design Name: 
// Module Name: DATA_MEMORY
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

(* DONT_TOUCH = "yes" *)
module DATA_MEMORY #(parameter DEPTH=8192) (
    input i_clk,
    input [$clog2(DEPTH)-1:0] i_wb_addr, // Input Addr
    input [31:0] i_wb_data, //Write Data
    output reg [31:0] o_wb_data,   //Read Data
    input wire i_wb_cyc,   
    input wire i_wb_stb,  // R/W Request
    input wire i_wb_we,   //Write Enable
    output reg o_wb_ack,  // R/W Ack
    output o_wb_stall,    //Stall
    input [3:0] i_wb_sel  //Write byte select
    );

    assign o_wb_stall = 1'b0; //Never Stall
    reg [31:0] DATA_MEM [DEPTH-1:0];
    
    initial begin
      $readmemh("Data.mem",DATA_MEM);
    end

    always @(posedge i_clk) begin
      o_wb_ack  <= i_wb_stb && i_wb_cyc;                          //Ack
      o_wb_data <= DATA_MEM[i_wb_addr[$clog2(DEPTH)-1:2]];  //Read Data
    end
    always @(posedge i_clk) begin
      if(i_wb_we && i_wb_stb && i_wb_cyc) begin
        if(i_wb_sel[0]) DATA_MEM[i_wb_addr[$clog2(DEPTH)-1:2]][7:0]   <= i_wb_data[7:0];    //1st byte
        if(i_wb_sel[1]) DATA_MEM[i_wb_addr[$clog2(DEPTH)-1:2]][15:8]  <= i_wb_data[15:8];   //2nd byte
        if(i_wb_sel[2]) DATA_MEM[i_wb_addr[$clog2(DEPTH)-1:2]][23:16] <= i_wb_data[23:16];  //3rd byte
        if(i_wb_sel[3]) DATA_MEM[i_wb_addr[$clog2(DEPTH)-1:2]][31:24] <= i_wb_data[31:24];  //4th byte
      end 
    end


endmodule
