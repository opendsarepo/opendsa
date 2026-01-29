   module fifo   #( parameter ADDR_WIDTH = 12,
                    parameter DATA_WIDTH = 64
                    )
   (
   input  wire                      clk          ,                                              // system clock
   input  wire                      rstn         ,                                              // system asynchronous reset
   input  wire                      srst         ,                                              // system synchronous reset

   input  wire                      wrreq        ,                                              // request a write
   input  wire [DATA_WIDTH-1   :0]  wrdata       ,                                              // write data
   output wire [ADDR_WIDTH     :0]  wravail      ,                                              // available written place
   output wire                      wrfull       ,                                              // buffer is full

   input  wire                      rdreq        ,                                              // request a read
   output reg  [DATA_WIDTH-1   :0]  rddata       ,                                              // read data
   output wire [ADDR_WIDTH     :0]  rdused       ,                                              // used read place
   output wire                      rdempty                                                     // buffer is empty
   );





   wire [DATA_WIDTH-1   :0]    rd_data_i       ;                                                // INTERNAL SIGNAL read data
   reg  [ADDR_WIDTH-1   :0]    rd_addr         ;                                                // INTERNAL SIGNAL current read address
   reg  [ADDR_WIDTH-1   :0]    wr_addr         ;                                                // INTERNAL SIGNAL write address
   reg  [ADDR_WIDTH-1   :0]    rdaddr_plus1    ;                                                // INTERNAL SIGNAL incremented read address
   wire [ADDR_WIDTH-1   :0]    rdaddr_next     ;                                                // INTERNAL SIGNAL next read address

   wire                        read_allow      ;                                                // INTERNAL SIGNAL read is allowed
   wire                        write_allow     ;                                                // INTERNAL SIGNAL write is allowed
   reg                         write_allow_r   ;                                                // INTERNAL SIGNAL REGISTERED write is allowed
   reg                         write_allow_rr  ;                                                // INTERNAL SIGNAL DOUBLE REGISTERED write is allowed
   reg [ADDR_WIDTH   :0]       wrcount         ;                                                // INTERNAL SIGNAL write counter
   reg [ADDR_WIDTH   :0]       rdcount         ;                                                // INTERNAL SIGNAL read counter
   reg                         wrfull_r        ;                                                // REGISTERED wr_full signal
   reg                         rdempty_r       ;                                                // REGISTERED rd_empty signal


///////////////////////////////////////////////////////////////////////////////
// RAM INSTANCE
///////////////////////////////////////////////////////////////////////////////

   dcram_fifo #(
      .ADDR_WIDTH(ADDR_WIDTH),
      .DATA_WIDTH(DATA_WIDTH)
      )
   DRAM_INST(
      .wrclock       (clk           )        ,
      .wren          (write_allow   )        ,
      .wraddress     (wr_addr       )        ,
      .data          (wrdata        )        ,
      .rdclock       (clk           )        ,
      .rdaddress     (rdaddr_next   )        ,
      .q             (rd_data_i     )
   );

///////////////////////////////////////////////////////////////////////////////
// POINTER MANAGEMENT
///////////////////////////////////////////////////////////////////////////////
   always @(posedge clk or negedge rstn) begin : POINTER_PROC
      if (!rstn) begin
         rddata       <= {DATA_WIDTH{1'b0}};
         wr_addr      <= {ADDR_WIDTH{1'b0}};
         rd_addr      <= {ADDR_WIDTH{1'b0}};
         rdaddr_plus1 <= {{(ADDR_WIDTH-1){1'b0}},1'b1};
      end else  begin
         if (srst) begin
            rddata          <= {DATA_WIDTH{1'b0}};
            wr_addr         <= {ADDR_WIDTH{1'b0}};
            rd_addr         <= {ADDR_WIDTH{1'b0}};
            rdaddr_plus1    <= {{(ADDR_WIDTH-1){1'b0}},1'b1};
         end else begin
            // Write pointer management
            if(write_allow) wr_addr <= wr_addr + {{(ADDR_WIDTH-1){1'b0}},1'b1};                 // if write is allowed, increment by 1 the wr_addr pointer

            // read pointer management
            if (read_allow) rddata <= rd_data_i;                                                // if read allow, load the next readen data
            rd_addr      <= rdaddr_next;                                                        // load next read address
            rdaddr_plus1 <= rdaddr_next + 1'b1;                                                 // increment by 1 the rd_addr pointer

         end
      end
   end

   assign rdaddr_next = (read_allow == 1'b1) ? rdaddr_plus1 :                                   // if read allow, load the next read address
                        rd_addr;                                                                // else the current read address

///////////////////////////////////////////////////////////////////////////////
// wrfull and rdempty management
///////////////////////////////////////////////////////////////////////////////

   assign write_allow = wrreq & (~wrfull_r);                                                    // write is allow if wrreq is asserted and the buffer is not full
   assign read_allow  = rdreq & (~rdempty_r);                                                   // read is allow if wrreq is asserted and the buffer is not full

   // Data counters

   always @(posedge clk or negedge rstn) begin : BUFFER_MGT_PROC
      if (!rstn)  begin
         write_allow_r  <= 1'b0;
         write_allow_rr <= 1'b0;
         wrcount        <= {1'b1,{ADDR_WIDTH{1'b0}}};
         rdcount        <= {ADDR_WIDTH+1{1'b0}};
         wrfull_r       <= 1'b0;
         rdempty_r      <= 1'b1;
      end else begin
         if (srst)begin
            write_allow_r  <= 1'b0;
            write_allow_rr <= 1'b0;
            wrcount        <= {1'b1,{ADDR_WIDTH{1'b0}}};
            rdcount        <= {ADDR_WIDTH+1{1'b0}};
            wrfull_r       <= 1'b0;
            rdempty_r      <= 1'b1;
         end else begin
            write_allow_r  <= write_allow;                                                      // register write_allow
            write_allow_rr <= write_allow_r;                                                    // double registration of write_allow

            if (read_allow && !write_allow)                                                     // read without write -> one place more available
               wrcount <= wrcount + 1'b1;
            else if (write_allow && !read_allow)                                                // write without read -> one place less available
               wrcount <= wrcount - 1'b1;

            if (read_allow && !write_allow_rr)                                                  // read without write -> one place more available
               rdcount <= rdcount - 1'b1;
            else if (write_allow_rr && !read_allow)                                             // write without read -> one place less available
               rdcount <= rdcount + 1'b1;

            if (wrcount[ADDR_WIDTH:1] == 0 && rdreq == 1'b0 && (wrcount[0] == 1'b0 || wrreq == 1'b1))
               wrfull_r <= 1'b1;
            else
               wrfull_r <= 1'b0;

            if (rdcount[ADDR_WIDTH:1] == 0 & write_allow_rr == 1'b0 & (rdcount[0] == 1'b0 | rdreq == 1'b1))
               rdempty_r <= 1'b1;
            else
               rdempty_r <= 1'b0;
         end
      end
   end

   assign wravail = wrcount;
   assign rdused = rdcount;
   assign wrfull = wrfull_r;
   assign rdempty = rdempty_r;

endmodule