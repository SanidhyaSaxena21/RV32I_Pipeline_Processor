`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 11/03/2024 01:57:51 AM
// Design Name: 
// Module Name: switch_debouncer
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


module switch_debouncer #(
    parameter BOUNCE_TIME = 3,
    parameter IS_PULLUP =0
)
(
    input clk,
    input rst_n,
    input i_switch,
    output o_debounced
    );
    
    wire i_switch_sync;
    reg sig_switch;
    reg sig_debounced;
    reg [BOUNCE_TIME:0]counter_reg;
    
    dff2_sync #(.DATA_WIDTH(1)) SYNC
    (
    .clk(clk),
    .rst_n(rst_n),
    .async(i_switch),
    .sync(i_switch_sync));
    
    always @(posedge clk or negedge rst_n) begin
        if(!rst_n) begin
            sig_switch= IS_PULLUP;
            sig_debounced = IS_PULLUP;
            counter_reg <= 0;
        end
        else begin
            sig_switch = i_switch_sync;
            
            counter_reg <= (sig_switch != sig_debounced)?counter_reg +1:
                           (counter_reg>0)?counter_reg-1:counter_reg;
            
            if(counter_reg[BOUNCE_TIME]) begin
                sig_debounced <= sig_switch;
            end
        end
    end
    
    assign o_debounced = sig_debounced;
endmodule

