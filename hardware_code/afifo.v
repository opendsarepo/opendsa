   module afifo   #( parameter ADDR_WIDTH = 12,
                     parameter DATA_WIDTH = 64
                    )
   (
   input  wire                      wrclk        ,                                              // system clock for write
   input  wire                      wrrstn       ,                                              // system asynchronous reset for write
   input  wire                      wrsrst       ,                                              // system synchronous reset for write

   input  wire                      rdclk        ,                                              // system clock for read
   input  wire                      rdrstn       ,                                              // system asynchronous reset for read
   input  wire                      rdsrst       ,                                              // system synchronous reset for read

   input  wire                      wrreq        ,                                              // request a write
   input  wire [DATA_WIDTH-1   :0]  wrdata       ,                                              // write data
   output wire                      wrfull       ,                                              // buffer is full
   output wire                      wr_almostfull,
   output wire [ADDR_WIDTH     :0]  wr_avail     ,
   input  wire                      rdreq        ,                                              // request a read
   output reg  [DATA_WIDTH-1   :0]  rddata       ,                                              // read data
   output wire                      rdempty                                                     // buffer is empty
   );

   localparam FIFO_RAM_DEPTH                = (1 << ADDR_WIDTH);
   localparam PLACE_FREE                    = FIFO_RAM_DEPTH - 10;
   localparam ADDR_BIT_LOCAL                = ADDR_WIDTH +1;

   wire [DATA_WIDTH-1   :0]    rd_data_i       ;                                                // INTERNAL SIGNAL read data

   reg  [ADDR_WIDTH     :0]    wrgray_meta     ;
   reg  [ADDR_WIDTH     :0]    wrgray_rd       ;
   reg  [ADDR_WIDTH     :0]    rdgray_meta     ;
   reg  [ADDR_WIDTH     :0]    rdgray_wr       ;

   reg  [ADDR_WIDTH     :0]    rd_addr         ;
   reg  [ADDR_WIDTH     :0]    rd_gray         ;
   wire [ADDR_WIDTH     :0]    rd_addr_next    ;                                                // INTERNAL SIGNAL next read address
   wire [ADDR_WIDTH     :0]    rd_gray_next    ;

   reg  [ADDR_WIDTH     :0]    wr_addr         ;                                                // INTERNAL SIGNAL write address
   reg  [ADDR_WIDTH     :0]    wr_gray         ;
   reg  [ADDR_WIDTH     :0]    wr_gray_r       ;
   wire [ADDR_WIDTH     :0]    wr_addr_next    ;
   wire [ADDR_WIDTH     :0]    wr_gray_next    ;
   wire [ADDR_WIDTH-1   :0]    sel_rd_addr     ;

   wire [ADDR_WIDTH     :0]    rd_binary       ;


   wire                        read_allow      ;                                                // INTERNAL SIGNAL read is allowed
   wire                        write_allow     ;                                                // INTERNAL SIGNAL write is allowed
   reg                         wrfull_r        ;                                                // REGISTERED wr_full signal
   reg                         rdempty_r       ;                                                // REGISTERED rd_empty signal
   reg  [ADDR_WIDTH     :0]    status_cnt      ;

   
   assign                      wr_avail  =  FIFO_RAM_DEPTH[ADDR_WIDTH     :0] -status_cnt;

   ///////////////////////////////////////////////////////////////////////////////
   // RAM INSTANCE
   ///////////////////////////////////////////////////////////////////////////////
   dcram_fifo #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
      )
   DRAM_INST(
      .wrclock       (wrclk                         )        ,
      .wren          (write_allow                   )        ,
      .wraddress     (wr_addr[ADDR_WIDTH-1:0]       )        ,
      .data          (wrdata                        )        ,
      .rdclock       (rdclk                         )        ,
      .rdaddress     (sel_rd_addr                   )        ,
      .q             (rd_data_i                     )
   );

   ///////////////////////////////////////////////////////////////////////////////
   // WRITE POINTER MANAGEMENT
   ///////////////////////////////////////////////////////////////////////////////
   always @(posedge wrclk or negedge wrrstn) begin : WRPOINTER_PROC
      if (!wrrstn) begin
         wr_addr      <= {ADDR_BIT_LOCAL{1'b0}};
         wr_gray      <= {ADDR_BIT_LOCAL{1'b0}};
         wr_gray_r    <= {ADDR_BIT_LOCAL{1'b0}};
      end else  begin
         if (wrsrst) begin
            wr_addr      <= {ADDR_BIT_LOCAL{1'b0}};
            wr_gray      <= {ADDR_BIT_LOCAL{1'b0}};
            wr_gray_r    <= {ADDR_BIT_LOCAL{1'b0}};
         end else begin
            // Write pointer management
            if(write_allow) begin
               wr_addr     <= wr_addr_next;                 // if write is allowed, increment by 1 the wr_addr pointer
               wr_gray     <= wr_gray_next;
            end
            wr_gray_r   <= wr_gray;
         end
      end
   end

   assign write_allow = wrreq & (~wrfull_r);           // write is allow if wrreq is asserted and the buffer is not full
   assign wr_addr_next = wr_addr + 1'b1;
   assign wr_gray_next = wr_addr_next ^ (wr_addr_next>>1);

   always @(posedge rdclk or negedge rdrstn) begin : RDPOINTER_PROC
      if (!rdrstn) begin
         rd_addr      <= {ADDR_BIT_LOCAL{1'b0}};
         rd_gray      <= {ADDR_BIT_LOCAL{1'b0}};
         rddata       <= {DATA_WIDTH{1'b0}};
      end else  begin
         if (rdsrst==1'b1) begin
            rd_addr      <= {ADDR_BIT_LOCAL{1'b0}};
            rd_gray      <= {ADDR_BIT_LOCAL{1'b0}};
            rddata       <= {DATA_WIDTH{1'b0}};
         end else begin
            if(read_allow == 1'b1)  rd_addr <= rd_addr_next;
            if(read_allow == 1'b1)  rd_gray <= rd_gray_next;
            if(read_allow == 1'b1)  rddata  <= rd_data_i;
         end
      end
   end

   assign read_allow  = rdreq & (~rdempty_r);          // read is allow if rdreq is asserted and the buffer is not full
   assign rd_addr_next = rd_addr + 1'b1;
   assign rd_gray_next = rd_addr_next ^ (rd_addr_next>>1);
   assign sel_rd_addr = (read_allow)? rd_addr_next[ADDR_WIDTH-1:0] : rd_addr[ADDR_WIDTH-1:0];

   ///////////////////////////////////////////////////////////////////////////////
   // CDC management
   ///////////////////////////////////////////////////////////////////////////////
   always @(posedge rdclk or negedge rdrstn)begin : WRITE_TO_READ_PROC
      if(!rdrstn)begin
         wrgray_meta  <= {ADDR_BIT_LOCAL{1'b0}};
         wrgray_rd    <= {ADDR_BIT_LOCAL{1'b0}};
      end else begin
         if (rdsrst) begin
            wrgray_meta  <= {ADDR_BIT_LOCAL{1'b0}};
            wrgray_rd    <= {ADDR_BIT_LOCAL{1'b0}};
         end else begin
            wrgray_meta <= wr_gray_r;
            wrgray_rd   <= wrgray_meta;
         end
      end
   end

   always @(posedge wrclk or negedge wrrstn)begin : READ_TO_WRITE_PROC
      if(!wrrstn)begin
         rdgray_meta <= {ADDR_BIT_LOCAL{1'b0}};
         rdgray_wr   <= {ADDR_BIT_LOCAL{1'b0}};
      end else begin
         if (wrsrst) begin
            rdgray_meta <= {ADDR_BIT_LOCAL{1'b0}};
            rdgray_wr   <= {ADDR_BIT_LOCAL{1'b0}};
         end else begin
            rdgray_meta  <= rd_gray;
            rdgray_wr    <= rdgray_meta;
         end
      end
   end
   ///////////////////////////////////////////////////////////////////////////////
   // wrfull and rdempty management
   ///////////////////////////////////////////////////////////////////////////////
   always @(posedge wrclk or negedge wrrstn) begin : WR_FULL_PROC
      if (!wrrstn)  begin
         wrfull_r         <= 1'b0;
         status_cnt       <= {ADDR_BIT_LOCAL{1'b0}};
      end else begin
         if (wrsrst)begin
            wrfull_r         <= 1'b0;
            status_cnt       <= {ADDR_BIT_LOCAL{1'b0}};
         end else begin
            wrfull_r <= (wr_gray== {~rdgray_wr[ADDR_WIDTH:ADDR_WIDTH-1],rdgray_wr[ADDR_WIDTH-2:0]}) || ((wr_gray_next=={~rdgray_wr[ADDR_WIDTH:ADDR_WIDTH-1],rdgray_wr[ADDR_WIDTH-2:0]}));
            
            //(wr_gray_next=={~rdgray_wr[ADDR_WIDTH:ADDR_WIDTH-1],rdgray_wr[ADDR_WIDTH-2:0]});

            status_cnt       <= wr_addr - rd_binary;
         end
      end
   end

   always @(posedge rdclk or negedge rdrstn) begin : RD_EMPTY_MGT_PROC
      if (!rdrstn)  begin
         rdempty_r      <= 1'b1;
      end else begin
         if (rdsrst)begin
            rdempty_r   <= 1'b1;
         end else begin
            rdempty_r <= (rd_gray==wrgray_rd || (read_allow & (rd_gray_next==wrgray_rd)));
         end
      end
   end

   assign wrfull = wrfull_r;
   assign rdempty = rdempty_r;
   assign wr_almostfull = (status_cnt >= PLACE_FREE)          ? 1'b1 : 1'b0;
   assign rd_binary = rdgray_wr ^ {1'b0, rd_binary[ADDR_WIDTH:1]};
endmodule