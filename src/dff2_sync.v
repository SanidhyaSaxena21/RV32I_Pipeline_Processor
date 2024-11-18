`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2024 02:11:56 AM
// Design Name: 
// Module Name: dff2_sync
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


module dff2_sync #(
    parameter DATA_WIDTH=1,
    parameter IS_PULUP=0
    )(
    input [DATA_WIDTH-1:0] async,
    input clk,
    input rst_n,
    output [DATA_WIDTH-1:0] sync
    );
    
    reg [DATA_WIDTH-1:0] async1;
    reg [DATA_WIDTH-1:0] async2;
    
    // 2 Flop Synchroniser for meeting metastability
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            async1 <= {DATA_WIDTH{IS_PULUP}};
            async2 <= {DATA_WIDTH{IS_PULUP}};
        end
        else begin
            async1 <= async; // Meta flop
            async2 <= async1; // Sync Flop
        end
    end
    
    assign sync = async2;
    
    
endmodule
