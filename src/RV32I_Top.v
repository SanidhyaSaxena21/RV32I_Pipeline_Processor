`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/23/2024 06:35:35 PM
// Design Name: 
// Module Name: RV32I_Top
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
module RV32I_Top(
    input clk_100mhz,
    input i_rst,
    input pulse_next_sw,
    input pulse_prev_sw,
    input data_result,
    input start,
    input [7:0] Wdata_top,
    input Wen_top,
    //output [31:0] i_wb_data_data,
    //output [31:0] inst,
    output end_of_Instr_WB,
    output [7:0] final_data,
    output dp,
	  output [0:6] seg,
	  output [3:0] digit
    
    );
    /*clk_wiz_0 clock_divider
    (
     // Clock out ports  
        .clk_out1(i_clk),
    // Status and control signals               
  .     reset(i_rst), 
        // Clock in ports
  .     clk_in1(clk_100mhz)
    );*/
    
    reg i_clk;
    
    always @(posedge clk_100mhz or posedge i_rst) begin
        if(i_rst) i_clk <= 1'b0;
        else i_clk <= ~i_clk;
    end
    parameter PC_RESET = 32'h0000_0000;
    parameter I_DEPTH = 8192;
    parameter D_DEPTH = 8192;
    parameter MEM_SIZE = 32768;
    parameter USE_CACHE = 1;

    wire   [31:0] inst;
    wire  [31:0] iaddr;
    //wire  i_stb_inst;
    //wire   i_ack_inst;

    //Data Memory Interface
    wire  wb_cyc_data;          
    wire  wb_stb_data;  
    wire  wb_we_data;   
    wire  [31:0] wb_addr_data; 
    wire  [31:0] o_wb_data_data;
    wire  [3:0] wb_sel_data; 
    wire   wb_ack_data; 
    wire   wb_stall_data;  
    wire   [31:0] i_wb_data_data; 
    
   // `ifdef USE_CACHE
      wire cache_valid;
      wire cache_stall;
      wire [$clog2(MEM_SIZE)-1:0] o_mem_addr;
      wire [31:0] instr_mem_cache;
      wire o_cache_mem_stb;
      wire i_cache_mem_ack;
    //`else
      wire i_stb_inst;
      wire o_ack_inst;
      
      wire cache_stall_i,cache_valid_i,imem_ack_inst_i;
    //`endif
     //wire o_cache_mem_stb;
     //wire i_cache_mem_ack;

     //wire [$clog2(MEM_SIZE)-1:0] o_mem_addr;
     //wire [31:0] instr_mem_cache;
     
     
       wire [31:0] ADDR;
       wire [31:0] WDATA;
       wire WEN;
       wire STB;
       wire [3:0] byte_sel;
       wire STB_TOP,CYC_TOP;
    
        wire resetn_core;
        
        
        assign resetn_core = (!i_rst) && (start);
        
     RV32I_Core #(.PC_RESET(PC_RESET),.USE_CACHE(USE_CACHE)) CORE( //main RV32I core
       .i_clk(i_clk),
       .i_rst_n(resetn_core),

       //Instruction Memory Interface
       .i_inst(inst), //32-bit instruction
       .o_iaddr(iaddr), //address of instruction
       //.i_cache_valid(cache_valid),
       //.i_cache_stall(cache_stall),
      //`ifdef USE_CACHE
        .i_cache_valid(cache_valid_i),
        .i_cache_stall(cache_stall_i),
       //`else
        .o_imem_req(i_stb_inst), //request for read access to instruction memory
        .i_ack_inst(imem_ack_inst_i),  //ack (high if new instruction is ready)
      //`endif

       //Data Memory Interface
       .o_wb_cyc_data(wb_cyc_data), //bus cycle active (1 = normal operation, 0 = all ongoing transaction are to be cancelled)
       .o_wb_stb_data(wb_stb_data), //request for read/write access to data memory
       .o_wb_we_data(wb_we_data), //write-enable (1 = write, 0 = read)
       .o_wb_addr_data(wb_addr_data), //address of data memory for store/load
       .o_wb_data_data(o_wb_data_data), //data to be stored to memory
       .o_wb_sel_data(wb_sel_data), //byte strobe for write (1 = write the byte) {byte3,byte2,byte1,byte0}
       .i_wb_ack_data(wb_ack_data), //ack by data memory (high when read data is ready or when write data is already written)
       .i_wb_stall_data(wb_stall_data), //stall by data memory
       .i_wb_data_data(i_wb_data_data), //data retrieved from memory
       
       //Interrupts
       .i_external_interrupt(1'b0), //interrupt from external source
       .i_software_interrupt(1'b0), //interrupt from software (inter-processor interrupt)
       .i_timer_interrupt(1'b0), //interrupt from timer
       .end_of_Instr_WB(end_of_Instr_WB)
     );
  

    generate 
        if(USE_CACHE) begin
            assign cache_stall_i = cache_stall;
            assign cache_valid_i = cache_valid;
            assign imem_ack_inst_i = o_ack_inst;
        end
        else begin
            assign cache_stall_i = 1'b0;
            assign cache_valid_i = 1'b1;
            assign imem_ack_inst_i = o_ack_inst;
        end
    endgenerate

    generate 
        if(USE_CACHE) begin
        ICACHE #(.MEM_SIZE(32768),.CACHE_SIZE(1024),.BLK_SIZE(16)) ICACHE(
         .i_clk(i_clk),
         .i_rst_n(!i_rst),
         .i_cpu_addr(iaddr[14:0]),
         .o_valid(cache_valid),
         .i_stb_cache(i_stb_inst),
         .o_ack_cache(o_ack_inst),
         .o_instr(inst),
         .stall_q(cache_stall),
         .o_mem_addr(o_mem_addr),
         .o_stb(o_cache_mem_stb),
         .i_ack(i_cache_mem_ack),
         .i_instr_mem(instr_mem_cache)
        );
       end
     endgenerate

    generate
     if(USE_CACHE) begin
       INSTRUCTION_MEMORY #(.DEPTH(I_DEPTH),.DELAY_CYCLE(0)) IMEM (
        .i_clk(i_clk),
        .i_rst_n(!i_rst),
        .i_addr(o_mem_addr[$clog2(MEM_SIZE)-1:2]),
        .i_stb(o_cache_mem_stb),
        .o_ack(i_cache_mem_ack),
        .instr(instr_mem_cache)
       );
     end 
     else begin
       INSTRUCTION_MEMORY #(.DEPTH(I_DEPTH),.DELAY_CYCLE(1)) IMEM (
        .i_clk(i_clk),
        .i_rst_n(!i_rst),
        .i_addr(iaddr[$clog2(MEM_SIZE)-1:2]),
        .burst(1'b0),
        .i_stb(i_stb_inst),
        .o_ack(o_ack_inst),
        .instr(inst)
       );
       /*Instruction_mem #(.DEPTH(I_DEPTH)) IMEM (
        .i_clk(i_clk),
        .i_addr(iaddr[$clog2(MEM_SIZE)-1:2]),
        .i_stb(i_stb_inst),
        .o_ack(o_ack_inst),
        .instr(inst)
       );*/  
       end
    endgenerate
    
     /*DATA_MEMORY #(.DEPTH(D_DEPTH)) DMEM (
       .i_clk(i_clk),
       .i_wb_addr(Addr_top[12:0]),   // Input Addr
       .i_wb_data(o_wb_data_data),   //Write Data
       .o_wb_data(i_wb_data_data),   //Read Data
       .i_wb_cyc(wb_cyc_data),   
       .i_wb_stb(wb_stb_data),    // R/W Request
       .i_wb_we(wb_we_data),     //Write Enable
       .o_wb_ack(wb_ack_data),    // R/W Ack
       .o_wb_stall(wb_stall_data),  //Stall
       .i_wb_sel(wb_sel_data)    //Write byte select
     );*/
     
     DATA_MEMORY #(.DEPTH(D_DEPTH)) DMEM (
       .i_clk(i_clk),
       .i_wb_addr(ADDR[12:0]),   // Input Addr
       .i_wb_data(WDATA),   //Write Data
       .o_wb_data(i_wb_data_data),   //Read Data
       .i_wb_cyc(CYC),   
       .i_wb_stb(STB),    // R/W Request
       .i_wb_we(WEN),     //Write Enable
       .o_wb_ack(wb_ack_data),    // R/W Ack
       .o_wb_stall(wb_stall_data),  //Stall
       .i_wb_sel(byte_sel)    //Write byte select
     );


     wire rst_n;
     assign rst_n = !i_rst;

    wire pulse_next_sw_debounced;
    wire pulse_prev_sw_debounced;
    reg counter_en_1;
    reg counter_en_2;
    wire  pulse_sw;
    wire pulse_sw_prev;
    reg [15:0] nxt_ADDR;
    reg [3:0] thousand_bit;
    wire [31:0] Addr_top;

    parameter NUM_MEM_LOCATION = 11;
    parameter BOUNCE_TIME=4;
    parameter IS_PULLUP=0;
    

    assign STB_TOP = Wen_top;
    assign CYC_TOP = Wen_top;
    
    assign ADDR = (start) ? wb_addr_data : Addr_top;
    assign WDATA = (start) ? o_wb_data_data : {24'd0,Wdata_top};
    assign WEN = (start) ? wb_we_data : Wen_top;
    assign STB = (start) ? wb_stb_data : STB_TOP;
    assign CYC = (start) ? wb_cyc_data : CYC_TOP;
    assign byte_sel = (start) ? wb_sel_data : 4'b1111;
    
    
    //assign WEN = (start) ? wb_we_data : WEN;
    assign Addr_top = data_result ? 32'd40 : nxt_ADDR;
    //assign Addr_top = (end_of_Instr_WB) ? 32'd40 : wb_addr_data;
    assign final_data = (i_wb_data_data[7:0]);

    //assign Addr_top = 32'd40;
    
    switch_debouncer #(
    .BOUNCE_TIME(BOUNCE_TIME),
    .IS_PULLUP(IS_PULLUP)
    ) debouncer_next
    (
    .clk(i_clk),
    .rst_n(rst_n),
    .i_switch(pulse_next_sw),
    .o_debounced(pulse_next_sw_debounced));
    
    switch_debouncer #(
    .BOUNCE_TIME(BOUNCE_TIME),
    .IS_PULLUP(IS_PULLUP)
    ) debouncer_prev
    (
    .clk(i_clk),
    .rst_n(rst_n),
    .i_switch(pulse_prev_sw),
    .o_debounced(pulse_prev_sw_debounced));    
      
    always @(posedge i_clk or negedge rst_n) begin
        if(!rst_n)
            counter_en_1 <= IS_PULLUP;
        else begin
            counter_en_1 <= pulse_next_sw_debounced;
        end
    end

    always @(posedge i_clk or negedge rst_n) begin
        if(!rst_n)
            counter_en_2 <= IS_PULLUP;
        else begin
            counter_en_2 <= pulse_prev_sw_debounced;
        end
    end
    
    assign pulse_sw = (~counter_en_1) & pulse_next_sw_debounced;
    assign pulse_sw_prev = (~counter_en_2) & pulse_prev_sw_debounced;
    
    always @(posedge i_clk or negedge rst_n) begin
        if(!rst_n) begin 
            nxt_ADDR <= 16'd0;
            thousand_bit <= 4'd0;
        end
        else if(pulse_sw) begin
            nxt_ADDR <= nxt_ADDR + 3'd4;
            thousand_bit <= thousand_bit + 1'd1;
        end
        else if(pulse_sw_prev) begin 
            nxt_ADDR <= nxt_ADDR - 3'd4;
            thousand_bit <= thousand_bit - 1'd1;
        end
        else begin 
            nxt_ADDR <= nxt_ADDR;
            thousand_bit <= thousand_bit;
        end
    end
    
    wire [3:0] thousands,hundreds, tens, ones;
    assign thousands = (!end_of_Instr_WB) ? thousand_bit : ((end_of_Instr_WB) ? 4'b1111 : 4'b0000) ;
    assign hundreds = ((final_data % 1000)/100);
    assign tens = (((final_data%1000)%100)/10);
    assign ones = (((final_data%1000)%100)%10);
    
    seg7_control SEVEN_SEGMENT (
	    .clk_100MHz(i_clk),
	    .reset(!rst_n),
	    .ones(ones),
	    .tens(tens),
	    .hundreds(hundreds),
	    .thousands(thousands),
	    .dp(dp),
	    .seg(seg),
	    .digit(digit)
    );
    


endmodule
