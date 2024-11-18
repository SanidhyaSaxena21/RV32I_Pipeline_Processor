`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/24/2024 01:17:29 AM
// Design Name: 
// Module Name: tb_RV32I_Top
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


module tb_RV32I_Top(
    );
    
    reg clk;
    reg reset;
    
    wire end_of_Instr_WB;
    wire [7:0] final_data;
    wire dp;
	wire [0:6] seg;
	wire [3:0] digit;
	reg [7:0] Wdata_top;
	reg data_result,start;
	reg Wen_top;
	reg pulse_next_sw,pulse_prev_sw;
	wire done;
	  
    RV32I_Top RV32I(
        .clk_100mhz(clk),
        .i_rst(reset),
        .pulse_next_sw(pulse_next_sw),
        .pulse_prev_sw(pulse_prev_sw),
        .data_result(data_result),
        .start(start),
        .Wdata_top(Wdata_top),
        .Wen_top(Wen_top),
        .end_of_Instr_WB(end_of_Instr_WB),
        .final_data(final_data),
        .dp(dp),
        .seg(seg),
        .digit(digit));
        
    always #10 clk = ~clk;
    
    initial begin
        clk=0;
        reset=1;
        
        #200
        reset=0;
    end
    
    initial begin
        start =0;
        data_result=0;
        Wen_top=0;
        Wdata_top = 8'd0;
        pulse_next_sw=0;
        pulse_prev_sw=0;
        #100
        //start = 1;
        
        /*#150;
        
        @(posedge clk);
        Wdata_top = 8'd56;
        repeat(4) @(posedge clk);
        Wen_top = 1'b1;
        repeat(4) @(posedge clk);
        Wen_top = 1'b0;
        
        #40;
        @(posedge clk);
        pulse_next_sw = 1'b1;
        repeat (20) @(posedge clk);
        pulse_next_sw = 1'b0;
        @(posedge clk);
        Wdata_top = 8'd10;

        repeat(4) @(posedge clk);
        Wen_top = 1'b1;
        repeat(4) @(posedge clk);
        Wen_top = 1'b0;
        
        #70;
        @(posedge clk);
        pulse_next_sw = 1'b1;
        repeat (20) @(posedge clk);
        pulse_next_sw = 1'b0;
        @(posedge clk);
        Wdata_top = 8'd20;
        
        repeat(4) @(posedge clk);
        Wen_top = 1'b1;
        repeat(4) @(posedge clk);
        Wen_top = 1'b0;
        
        #70;
        @(posedge clk);
        pulse_next_sw = 1'b1;
        repeat (20) @(posedge clk);
        pulse_next_sw = 1'b0;
        Wdata_top = 8'd35;
        
        repeat(4) @(posedge clk);
        Wen_top = 1'b1;
        repeat(4) @(posedge clk);
        Wen_top = 1'b0;
        
        #70;
        @(posedge clk);
        pulse_next_sw = 1'b1;
        repeat (20) @(posedge clk);
        pulse_next_sw = 1'b0;
        Wdata_top = 8'd16;
        
        repeat(4) @(posedge clk);
        Wen_top = 1'b1;
        repeat(4) @(posedge clk);
        Wen_top = 1'b0;
        
      
         #70;
        @(posedge clk);
        pulse_next_sw = 1'b1;
        repeat (20) @(posedge clk);
        pulse_next_sw = 1'b0;
        Wdata_top = 8'd78;
        
        repeat(4) @(posedge clk);
        Wen_top = 1'b1;
        repeat(4) @(posedge clk);
        Wen_top = 1'b0;          
        
        #70;
        @(posedge clk);
        pulse_next_sw = 1'b1;
        repeat (20) @(posedge clk);
        pulse_next_sw = 1'b0;
        Wdata_top = 8'd89;
        
        repeat(4) @(posedge clk);
        Wen_top = 1'b1;
        repeat(4) @(posedge clk);
        Wen_top = 1'b0;  */
        
         
        #50;
        start = 1'b1;
                     
        
    end
    
 
    
    assign done = end_of_Instr_WB;
    
        
endmodule
