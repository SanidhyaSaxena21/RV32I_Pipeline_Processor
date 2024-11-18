`timescale 1ns / 1ps
//////////////////////////////////////////////////////////////////////////////////
// Company: 
// Engineer: 
// 
// Create Date: 10/26/2024 07:34:00 PM
// Design Name: 
// Module Name: ICACHE
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

//-------Calculations ----------------
// ADDR_WIDTH   = $clog2(MEM_SIZE)    // 13 
// BLOCK_WIDTH  = $clog2(BLK_SIZE)   // 4
// INDEX_WIDTH  = $clog2(CACHE_SZIE/BLK_SIZE) //6
// TAG_WIDTH    = ADDR_WDTH - INDEX_WIDTH - BLOCK_WIDTH

module ICACHE #(parameter MEM_SIZE = 32768, CACHE_SIZE=1024, BLK_SIZE = 16) (

    //Global Signals
    input i_clk,
    input i_rst_n,
    
    // Processor Interface
    input [$clog2(MEM_SIZE)-1:0] i_cpu_addr,
    output reg o_valid,
    input i_stb_cache,
    output reg o_ack_cache,
    output reg [31:0] o_instr,
    output reg stall_q,

    // Memory Interface
    output reg [$clog2(MEM_SIZE)-1:0] o_mem_addr,
    output reg o_stb,
    input i_ack, 
    input [31:0] i_instr_mem
    );

    //assign stall_q = 1'b0;
    localparam ADDR_WIDTH   = $clog2(MEM_SIZE);                          //    15 
    localparam BLOCK_WIDTH  = $clog2(BLK_SIZE);                          //    4
    localparam INDEX_WIDTH  = $clog2(CACHE_SIZE/BLK_SIZE);               //    6
    localparam TAG_WIDTH    = (ADDR_WIDTH - INDEX_WIDTH - BLOCK_WIDTH);    //    5
    localparam DEPTH_ARRAY  = 1 << INDEX_WIDTH;
    localparam BURST_LENGTH = ((8*BLK_SIZE)/32);
    
    wire [BLOCK_WIDTH-1:0] block_offset                 = i_cpu_addr[BLOCK_WIDTH-1:0];                        //3:0
    wire [BLOCK_WIDTH+INDEX_WIDTH-1:BLOCK_WIDTH] index  = i_cpu_addr[BLOCK_WIDTH+INDEX_WIDTH-1:BLOCK_WIDTH];   //9:4
    wire [ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH ]tags       = i_cpu_addr[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH];      //12:10
    
    wire hit;
    wire miss;

    wire [TAG_WIDTH:0] rdata_tag_array;
    wire [TAG_WIDTH:0] wdata_tag_array;
    wire [8*BLK_SIZE-1:0] rdata_data_array;
    wire [8*BLK_SIZE-1:0] wdata_data_array;
  
    wire valid = rdata_tag_array[TAG_WIDTH];
    reg cache_write_enable;

    assign hit = (valid == 1'b1) && (tags == rdata_tag_array[TAG_WIDTH-1:0]);
    assign miss = ~hit;


    cache_sram #(.WIDTH(1+TAG_WIDTH),.DEPTH(DEPTH_ARRAY)) tag_array (.clk(i_clk), .addr(index), .wdata(wdata_tag_array), .rdata(rdata_tag_array),  .wea(cache_write_enable));
    cache_sram #(.WIDTH(8*BLK_SIZE),.DEPTH(DEPTH_ARRAY)) data_array (.clk(i_clk), .addr(index), .wdata(wdata_data_array), .rdata(rdata_data_array), .wea(cache_write_enable));


    reg [31:0] data_line [BLK_SIZE/4-1:0];
    integer i;

    always@(*) begin
      for(i=0;i<(BLK_SIZE/4);i=i+1) begin
        data_line[i] = rdata_data_array[32*i+:32];
      end
    end

    // If it is a hit, return the instruction from the data array
    always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            o_instr <= 32'd0;
            o_ack_cache <= 1'b0;
            //o_valid <= 1'b0;
        end
        else begin
            o_instr <= (hit)?data_line[block_offset[BLOCK_WIDTH-1:2]]:32'd0;
            //o_ack_cache <= (hit) ? i_stb_cache : 1'b0;
            o_ack_cache <= i_stb_cache;
            //o_valid <= hit;
        end
    end
    
    always @(*) begin
        o_valid = hit;
    end
    //assign o_instr = (hit)?data_line[block_offset[BLOCK_WIDTH-1:2]]:32'd0;
    //assign o_valid = hit;

    // If it is a miss, we need to stall the IF stage
    //assign o_mem_addr = (!hit) ? i_cpu_addr : {ADDR_WIDTH{1'b0}};


    //reg stall_q;
    //assign cache_stall = o_stb && !i_ack;
    
    reg [$clog2(BURST_LENGTH):0] burst; 

    reg [ADDR_WIDTH-1:0] nxt_burst_addr;
    reg pending;

    always @(*) begin
      nxt_burst_addr = {i_cpu_addr[ADDR_WIDTH-1:4],4'b0000};
      if(!hit && o_stb &&  !(burst == BURST_LENGTH)) begin
        nxt_burst_addr = o_mem_addr + 3'd4;
      end
    end
    integer j;
    
    /*always @(posedge i_clk or negedge i_rst_n) begin
        if(!i_rst_n) begin
            stall_q <= 1'b1;
        end
        else begin
            stall_q <= !hit;
        end
    end*/
    
    always @(*) begin
        stall_q = !hit;
        //stall_q = 1'b1;
    end
    
    //assign stall_q = !hit;
    always @(posedge i_clk or negedge i_rst_n) begin
      if(!i_rst_n) begin
        o_mem_addr <= {ADDR_WIDTH{1'b0}};
        o_stb <= 1'b0;
        //stall_q <= 1'b0;
        burst <= 0;
        pending <= 1'b0;
      end
      else begin 
        if(!hit && !(burst == BURST_LENGTH) & !pending) begin
          o_mem_addr <= nxt_burst_addr;
          burst <= burst + 1'd1;
          o_stb <= 1'b1;
          //stall_q <= 1'b1;
        end
        else if(!hit && !pending) begin
          o_stb <= 1'b0;
          pending <= 1'b1;
          /*if(!o_stb && i_ack) begin
            stall_q <= 1'b0;
            burst <= 0;
            pending <= 1'b0;
          end*/
        end
        else if(hit) begin
          //stall_q <= 1'b0;
          burst <= 0;
          pending <= 1'b0;
        end
       end
       end

    reg [8*BLK_SIZE-1:0] instr_shift_reg;


    always @(posedge i_clk or negedge i_rst_n) begin
      if(!i_rst_n) begin
        instr_shift_reg <= 0;
      end
      else if(!hit && i_ack) begin
        instr_shift_reg <= {i_instr_mem ,instr_shift_reg[(8*BLK_SIZE)-1:32]};
      end
    end

    always @(posedge i_clk or negedge i_rst_n) begin
      if(!i_rst_n) cache_write_enable <= 1'b0;
      else if(!hit && i_ack && !o_stb && !cache_write_enable) cache_write_enable <= 1'b1;
      else cache_write_enable <= 1'b0;
    end
    assign wdata_tag_array[TAG_WIDTH] = (!hit) ? 1'b1 : 1'b0;
    assign wdata_tag_array[TAG_WIDTH-1:0] =(!hit) ? o_mem_addr[ADDR_WIDTH-1:ADDR_WIDTH-TAG_WIDTH] : {TAG_WIDTH{1'b0}};
    assign wdata_data_array = (!hit) ? instr_shift_reg : 0;




endmodule
