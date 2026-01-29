//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe Recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------

module ram_completion #(
   parameter                                       PCIE_BUS_WIDTH             =          256
)(
   input    wire                                         clk                      ,
   input    wire                                         rstn                     ,
   input    wire                                         srst                     ,

   // COMPLETION interface from completer_request
   input    wire                                         cpl_request              ,
   input    wire  [1                      :0]            cpl_address_type         ,
   input    wire  [7                      :0]            cpl_tag                  ,
   input    wire  [15                     :0]            cpl_requester_id         ,
   input    wire  [10                     :0]            cpl_length               ,
   input    wire  [6                      :0]            cpl_lower_address        ,
   input    wire  [2                      :0]            cpl_status               ,
   output   reg                                          cpl_ack                  ,
   output   wire                                         start_completion         ,
   output   wire                                         fifo_cc_almostfull       ,

   // AXI PCIE Interconnection
   output   reg   [PCIE_BUS_WIDTH-1       :0]            m_axis_cc_tdata          ,
   output   reg   [3                      :0]            m_axis_cc_lbe            ,
   output   reg                                          m_axis_cc_tlast          ,
   output   reg   [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_cc_tkeep          ,
   output   reg                                          m_axis_cc_tvalid         ,
   input    wire                                         m_axis_cc_tready         ,

   //  RAM backend for Configuration interface
   input    wire                                         subbuf_rdvalid           ,
   input    wire  [255                    :0]            subbuf_rddata            ,

   // OPAL RAM interface
`ifdef MODE_OPAL
   input   wire                                          opalbuf_rdvalid          ,
   input   wire   [63                     :0]            opalbuf_rddata           ,
`endif
   // RAM backend for Index interface
   input    wire                                         dataprplistbuf_rdvalid   ,
   input    wire  [63                     :0]            dataprplistbuf_rddata
);
   //-------------------------------------------------------------
   // FSM to detect type tlp or data
   //-------------------------------------------------------------
   localparam [1:0]      ST_CPL_IDLE              = 0,
                         ST_CPL_HEADER            = 1,
                         ST_CPL_DATA              = 2;
   reg    [1   :0]       tlp_cpl ;

   localparam            ZERO_DATA_UR       = (256/32) - 4'b011;
   localparam            WHOLE_PCIE_WIDTH   = PCIE_BUS_WIDTH+PCIE_BUS_WIDTH/32+1+4;
   localparam            FIFO_ADDR_WIDTH    = 7;

   /////////////////////////////////////////////////////////////////////////////////////////////
   // INTERNAL SIGNALS
   /////////////////////////////////////////////////////////////////////////////////////////////
   reg   [256-1                        :0]         tlp_start_r                        ;
   wire  [256-1                        :0]         tlp_completion_generic             ;
   reg   [256-1                        :0]         dataprplistbuf_rddata_r            ;
   reg                                             dataprplistbuf_rdvalid_r           ;
   reg   [256-1                        :0]         subbuf_rddata_r                    ;
   reg                                             subbuf_rdvalid_r                   ;
`ifdef MODE_OPAL
   reg                                             opalbuf_rdvalid_r                  ;
   reg   [63                           :0]         opalbuf_rddata_r                   ;
`endif
   reg                                             ram_rdata_req_r                    ;
   reg   [2                            :0]         cpl_status_r                       ;
   reg   [2                            :0]         cpl_sof_r                          ;
   reg                                             cpl_eof_r                          ;

   reg   [9                            :0]         cpt_data_r                         ;

   wire  [PCIE_BUS_WIDTH-1             :0]         m_axis_cc_tdata_i                  ;
   reg   [3                            :0]         m_axis_cc_lbe_i                    ;
   wire                                            m_axis_cc_tlast_i                  ;
   wire  [(PCIE_BUS_WIDTH/8)-1         :0]         m_axis_cc_tkeep_i                  ;
   wire  [(PCIE_BUS_WIDTH/32)-1        :0]         m_axis_cc_tkeep_ii                 ;
   wire                                            m_axis_cc_tvalid_i                 ;
   wire                                            m_axis_cc_tready_i                 ;
   wire  [PCIE_BUS_WIDTH-1             :0]         m_axis_cc_tdata_fifo               ;
   wire  [3                            :0]         m_axis_cc_lbe_fifo                 ;
   wire                                            m_axis_cc_tlast_fifo               ;
   wire  [(PCIE_BUS_WIDTH/32)-1        :0]         m_axis_cc_tkeep_fifo               ;
   wire                                            fiforam_wrfull                     ;
   wire                                            fiforam_req                        ;
   wire                                            fiforam_rdempty                    ;
   wire  [WHOLE_PCIE_WIDTH -1          :0]         fiforam_rddata                     ;
   reg   [PCIE_BUS_WIDTH-1             :0]         m_axis_cc_tdata_r                  ;
   reg   [3                            :0]         m_axis_cc_lbe_r                    ;
   reg                                             m_axis_cc_tlast_r                  ;
   reg   [(PCIE_BUS_WIDTH/32)-1        :0]         m_axis_cc_tkeep_r                  ;
   wire  [PCIE_BUS_WIDTH-1             :0]         sel_m_axis_cc_tdata                ;
   wire  [32                           :0]         sel_m_axis_cc_lbe                  ;
   wire                                            sel_m_axis_cc_tlast                ;
   wire  [(PCIE_BUS_WIDTH/32)-1        :0]         sel_m_axis_cc_tkeep                ;
   wire                                            axi_ack                            ;
   reg                                             m_axis_cc_tready_r                 ;
   reg   [1                            :0]         cnt_busy_ramout_r                  ;
   wire                                            incr_ack                           ;
   wire                                            decr_ack                           ;
   reg   [7                            :0]         tlp_complete_r                     ;
   reg                                             incr_ack_r                         ;
   reg                                             incr_ack_rr                        ;
   reg                                             incr_ack_rrr                       ;
   reg                                             incr_ack_rrrr                      ;
   reg                                             fiforam_req_r                      ;
   reg   [3                            :0]         in_byte_r                          ;
   wire  [3                            :0]         in_byte                            ;
   wire  [(256/8)-1                    :0]         in_be                              ;
   wire  [(256/8)-1                    :0]         in_be_i                            ;

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Routing
   /////////////////////////////////////////////////////////////////////////////////////////////
   assign tlp_completion_generic  = {7'b0,1'b0,16'h0,cpl_tag,cpl_requester_id,2'b0,cpl_status,cpl_length[10:0],3'b0,{cpl_length[10:0],2'b0},6'b0,cpl_address_type,1'b0,cpl_lower_address,160'b0};

   //------------------------------------------------------------
   // FSM to send a completion TLP
   //------------------------------------------------------------
   always @(posedge clk or negedge rstn)
   if (rstn == 1'b0) begin
      tlp_cpl <= ST_CPL_IDLE;
   end else begin
      if(srst == 1'b1) begin
         tlp_cpl <= ST_CPL_IDLE;
      end else begin
         case (tlp_cpl)
            ST_CPL_IDLE : begin
               if(cpl_request == 1'b1 && cpl_ack == 1'b1) begin
                  tlp_cpl <= ST_CPL_HEADER;
               end
            end

            ST_CPL_HEADER :
               if(cpl_status_r[1] == 1'b1)
                  tlp_cpl <= ST_CPL_IDLE;
               else
                  tlp_cpl <= ST_CPL_DATA;

            ST_CPL_DATA :
               if(cpl_eof_r == 1'b1 && ram_rdata_req_r == 1'b1) begin
                  tlp_cpl <= ST_CPL_IDLE;
               end

            default : tlp_cpl <= ST_CPL_IDLE;
         endcase
      end
   end

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Management
   /////////////////////////////////////////////////////////////////////////////////////////////
   always @(negedge rstn or posedge clk)
   begin : REGISTER_SHIFTER
      integer j_v;
      if (rstn == 1'b0) begin
         tlp_start_r                <= 256'h0;
         m_axis_cc_lbe_i            <= 4'h0;
         cpl_ack                    <= 1'b1;
         dataprplistbuf_rddata_r    <= 64'h0;
         dataprplistbuf_rdvalid_r   <= 1'b0;
         subbuf_rddata_r            <= 256'h0;
         subbuf_rdvalid_r           <= 1'b0;
`ifdef MODE_OPAL
          opalbuf_rdvalid_r          <= 1'b0;
          opalbuf_rddata_r           <= 64'h0;
`endif
         in_byte_r                  <= 4'b0;

         cpt_data_r                 <= {10{1'b0}};
         ram_rdata_req_r            <= 1'b0;
         cpl_status_r               <= 3'b0;
         cpl_sof_r                  <= 2'b0;
         cpl_eof_r                  <= 1'b0;

         incr_ack_r                 <= 1'b0;
         incr_ack_rr                <= 1'b0;
         incr_ack_rrr               <= 1'b0;
         incr_ack_rrrr              <= 1'b0;
         fiforam_req_r              <= 1'b0;
         tlp_complete_r             <= 8'b0;
      end else begin
         if(srst == 1'b1) begin
            tlp_start_r                <= 256'h0;
            m_axis_cc_lbe_i            <= 4'h0;
            cpl_ack                    <= 1'b1;
            dataprplistbuf_rddata_r    <= 64'h0;
            dataprplistbuf_rdvalid_r   <= 1'b0;
            subbuf_rddata_r            <= 256'h0;
            subbuf_rdvalid_r           <= 1'b0;
`ifdef MODE_OPAL
            opalbuf_rdvalid_r          <= 1'b0;
            opalbuf_rddata_r           <= 64'h0;
`endif
            in_byte_r                  <= 4'b0;

            cpt_data_r                 <= {10{1'b0}};
            ram_rdata_req_r            <= 1'b0;
            cpl_status_r               <= 3'b0;
            cpl_sof_r                  <= 2'b0;
            cpl_eof_r                  <= 1'b0;

            incr_ack_r                 <= 1'b0;
            incr_ack_rr                <= 1'b0;
            incr_ack_rrr               <= 1'b0;
            incr_ack_rrrr              <= 1'b0;
            fiforam_req_r              <= 1'b0;
            tlp_complete_r             <= 8'b0;
         end else begin
            if(cpl_request == 1'b1 && cpl_ack == 1'b1) begin
               cpl_status_r         <= cpl_status;
            end

            if(cpl_request == 1'b1 && cpl_ack == 1'b1)
               cpl_ack              <= 1'b0;
            else if(cpl_eof_r & ram_rdata_req_r)
               cpl_ack              <= 1'b1;

            if((cpl_request == 1'b1 && cpl_ack == 1'b1) || dataprplistbuf_rdvalid_r || subbuf_rdvalid_r
`ifdef MODE_OPAL
                     || opalbuf_rdvalid_r
`endif
            )
               ram_rdata_req_r <= 1'b1;
            else
               ram_rdata_req_r <= 1'b0;

            if(cpl_request == 1'b1 && cpl_ack == 1'b1) begin
               if(cpl_length == 11'h1)
                  m_axis_cc_lbe_i            <= 4'h0;
               else
                  m_axis_cc_lbe_i            <= 4'hf;
            end

            if(cpl_request == 1'b1 && cpl_ack == 1'b1)
               tlp_start_r <= tlp_completion_generic;
            else if(subbuf_rdvalid_r == 1'b1)
               tlp_start_r <= subbuf_rddata_r;
            else if(dataprplistbuf_rdvalid_r == 1'b1)
               tlp_start_r  <= {192'h0,dataprplistbuf_rddata_r};
`ifdef MODE_OPAL
         else if(opalbuf_rdvalid_r)
            tlp_start_r  <= {192'h0,opalbuf_rddata_r};
`endif

            dataprplistbuf_rddata_r   <= dataprplistbuf_rddata;
            dataprplistbuf_rdvalid_r  <= dataprplistbuf_rdvalid;

            subbuf_rddata_r           <= subbuf_rddata;
            subbuf_rdvalid_r          <= subbuf_rdvalid;
`ifdef MODE_OPAL
            opalbuf_rdvalid_r         <= opalbuf_rdvalid  ;
            opalbuf_rddata_r          <= opalbuf_rddata   ;
`endif
            if(cpl_request == 1'b1 && cpl_ack == 1'b1)
               cpl_sof_r   <= 3'b011;
            else if(tlp_cpl == ST_CPL_HEADER)
               cpl_sof_r   <= 3'b0;

            if(cpl_request == 1'b1 && cpl_ack == 1'b1) begin
               in_byte_r                  <= 4'h3;
            end else if(subbuf_rdvalid_r == 1'b1) begin
               if(cpt_data_r < (PCIE_BUS_WIDTH/32))
                  in_byte_r               <= cpt_data_r;
               else
                  in_byte_r               <= 4'h8;
            end else if(dataprplistbuf_rdvalid_r == 1'b1) begin
               if(cpt_data_r <= (64/32))
                  in_byte_r               <= cpt_data_r;
               else
                  in_byte_r               <= 4'h2;
`ifdef MODE_OPAL
            end else if(opalbuf_rdvalid_r == 1'b1) begin
//                if(cpt_data_r <= (32/32))
//                   in_byte_r               <= cpt_data_r;
//                else
               if(cpt_data_r <= (64/32))
                  in_byte_r               <= cpt_data_r;
               else
                  in_byte_r               <= 4'h2;

`endif
            end

            if(cpl_request == 1'b1 && cpl_ack == 1'b1)
               cpt_data_r  <=  cpl_length;
            else if(subbuf_rdvalid_r == 1'b1) begin
               if(cpt_data_r <= (256/32))
                 cpt_data_r  <= {10{1'b0}};
               else
                 cpt_data_r  <= cpt_data_r - (256/32);
            end else if(dataprplistbuf_rdvalid_r == 1'b1) begin
               if(cpt_data_r <= (64/32))
                  cpt_data_r  <= {10{1'b0}};
               else
                  cpt_data_r  <= cpt_data_r - (64/32);
`ifdef MODE_OPAL
            end else if(opalbuf_rdvalid_r == 1'b1) begin
               if(cpt_data_r <= (64/32))
                  cpt_data_r  <= {10{1'b0}};
               else
                  cpt_data_r  <= cpt_data_r - (64/32);
`endif
            end

            if(cpl_request == 1'b1 && cpl_ack == 1'b1 && cpl_status_r[0] == 1'b1)
               cpl_eof_r   <= 1'b1;
            else if(dataprplistbuf_rdvalid_r == 1'b1 && cpt_data_r <= (64/32))
               cpl_eof_r   <= 1'b1;
            else if(subbuf_rdvalid_r == 1'b1 && cpt_data_r <= (256/32))
               cpl_eof_r   <= 1'b1;
`ifdef MODE_OPAL
            else if(opalbuf_rdvalid_r == 1'b1 && cpt_data_r <= (64/32))
               cpl_eof_r   <= 1'b1;
`endif
            else
               cpl_eof_r   <= 1'b0;

            incr_ack_r    <= incr_ack;
            incr_ack_rr   <= incr_ack_r;
            incr_ack_rrr  <= incr_ack_rr;
            incr_ack_rrrr <= incr_ack_rrr;

            if(incr_ack_rrrr == 1'b1 && decr_ack == 1'b0)
              tlp_complete_r             <= tlp_complete_r+1'b1;
            else if(incr_ack_rrrr == 1'b0 && decr_ack == 1'b1)
              tlp_complete_r             <= tlp_complete_r-1'b1;

            fiforam_req_r                <= fiforam_req;
        end
      end
   end

   assign start_completion = cpl_eof_r & ram_rdata_req_r;
   assign in_byte          = in_byte_r;
   assign in_be            = (tlp_cpl == ST_CPL_HEADER) ? 32'hfff00000 : in_be_i;

   generate
   genvar a_v;
   for (a_v=0; a_v<(256/32); a_v=a_v+1) begin: loop_read_gen
      assign in_be_i[a_v*4 +:4]  = (in_byte_r>a_v) ? 4'hf : 4'h0;
   end
   endgenerate

   Data_shift_generic_rp #(
   .NVME_DATA_WIDTH   ( PCIE_BUS_WIDTH     ),
   .PCIE_DATA_WIDTH   ( 256                ),
   .RAM_LENGTH_WIDTH  ( 4                  )
   )Data_shift_generic_inst (
      .clk         (clk                 ),
      .rstn        (rstn                ),
      .srst        (srst                ),
      .in_valid    (ram_rdata_req_r     ),
      .in_be       (in_be               ),
      .in_byte     (in_byte             ),
      .in_data     (tlp_start_r         ),
      .in_sof      (cpl_sof_r           ),
      .in_eof      (cpl_eof_r           ),
      .out_ready   (m_axis_cc_tready_i  ),
      .out_valid   (m_axis_cc_tvalid_i  ),
      .out_be      (m_axis_cc_tkeep_i   ),
      .out_data    (m_axis_cc_tdata_i   ),
      .out_eof     (m_axis_cc_tlast_i   )
   );

   assign m_axis_cc_tready_i = ~fiforam_wrfull;

   afifo #(  .ADDR_WIDTH(FIFO_ADDR_WIDTH),
             .DATA_WIDTH(WHOLE_PCIE_WIDTH)
   ) afifo_ram_inst (
   .wrclk         (clk                                                                        ),
   .wrrstn        (rstn                                                                       ),
   .wrsrst        (srst                                                                       ),
   .wr_avail      (/*open       */                                                            ),
   .wrreq         (m_axis_cc_tvalid_i                                                         ),
   .wrdata        ({m_axis_cc_tkeep_ii,m_axis_cc_tlast_i,m_axis_cc_lbe_i,m_axis_cc_tdata_i}   ),
   .wrfull        (fiforam_wrfull                                                             ),
   .wr_almostfull (fifo_cc_almostfull                                                         ),
   .rdclk         (clk                                                                        ),
   .rdrstn        (rstn                                                                       ),
   .rdsrst        (srst                                                                       ),
   .rdreq         (fiforam_req                                                                ),
   .rdempty       (fiforam_rdempty                                                            ),
   .rddata        (fiforam_rddata                                                             )
   );

   generate
      genvar j_v;
      for (j_v=0; j_v<(PCIE_BUS_WIDTH/32); j_v=j_v+1) begin: loop_axi_cc
         assign m_axis_cc_tkeep_ii[j_v] = m_axis_cc_tkeep_i[(j_v*4)];
      end
   endgenerate

   assign incr_ack = m_axis_cc_tvalid_i & m_axis_cc_tlast_i &~(fiforam_wrfull);
   assign decr_ack = fiforam_req_r & m_axis_cc_tlast_fifo;

   assign fiforam_req          =  (decr_ack == 1'b1 && tlp_complete_r ==8'h1 )             ?    1'b0 :
                                  (cnt_busy_ramout_r < 2'b11  && tlp_complete_r !=8'h0  )  ?    ~fiforam_rdempty :
                                                                                                1'b0;

   assign m_axis_cc_tdata_fifo = fiforam_rddata[0                     +: PCIE_BUS_WIDTH];
   assign m_axis_cc_lbe_fifo   = fiforam_rddata[PCIE_BUS_WIDTH        +:  4];
   assign m_axis_cc_tlast_fifo = fiforam_rddata[PCIE_BUS_WIDTH+4      +:  1];
   assign m_axis_cc_tkeep_fifo = fiforam_rddata[PCIE_BUS_WIDTH+4+1    +: PCIE_BUS_WIDTH/32];

   always@(negedge rstn or posedge clk)
   begin : WRITE_RQ_PROC
      if(rstn == 1'b0) begin
         m_axis_cc_tdata    <= {PCIE_BUS_WIDTH{1'b0}};
         m_axis_cc_lbe      <= {(4){1'b0}};
         m_axis_cc_tlast    <= 1'b0;
         m_axis_cc_tkeep    <= {(PCIE_BUS_WIDTH/32){1'b0}};
         m_axis_cc_tvalid   <= 1'b0;

         m_axis_cc_tdata_r  <= {PCIE_BUS_WIDTH{1'b0}};
         m_axis_cc_lbe_r    <= {(33){1'b0}};
         m_axis_cc_tlast_r  <= 1'b0;
         m_axis_cc_tkeep_r  <= {(PCIE_BUS_WIDTH/32){1'b0}};
         m_axis_cc_tready_r <= 1'b0;
         cnt_busy_ramout_r  <= 2'b0;
      end else begin
         if(srst == 1'b1) begin
            m_axis_cc_tdata    <= {PCIE_BUS_WIDTH{1'b0}};
            m_axis_cc_lbe      <= {(33){1'b0}};
            m_axis_cc_tlast    <= 1'b0;
            m_axis_cc_tkeep    <= {(PCIE_BUS_WIDTH/32){1'b0}};
            m_axis_cc_tvalid   <= 1'b0;

            m_axis_cc_tdata_r  <= {PCIE_BUS_WIDTH{1'b0}};
            m_axis_cc_lbe_r    <= {(33){1'b0}};
            m_axis_cc_tlast_r  <= 1'b0;
            m_axis_cc_tkeep_r  <= {(PCIE_BUS_WIDTH/32){1'b0}};
            m_axis_cc_tready_r <= 1'b0;
            cnt_busy_ramout_r  <= 2'b0;
         end else begin
            if(fiforam_req && !axi_ack)
               cnt_busy_ramout_r <= cnt_busy_ramout_r + 1'b1;
            else if(!fiforam_req && axi_ack)
               cnt_busy_ramout_r <= cnt_busy_ramout_r - 1'b1;

            if(cnt_busy_ramout_r ==1 && !m_axis_cc_tvalid)
               m_axis_cc_tvalid   <= 1'b1;
            else if(cnt_busy_ramout_r == 1 && m_axis_cc_tready)
               m_axis_cc_tvalid   <= 1'b0;

            if(cnt_busy_ramout_r > 2'b00 && !m_axis_cc_tvalid) begin
               m_axis_cc_tdata  <= m_axis_cc_tdata_fifo ;
               m_axis_cc_lbe    <= m_axis_cc_lbe_fifo   ;
               m_axis_cc_tlast  <= m_axis_cc_tlast_fifo ;
               m_axis_cc_tkeep  <= m_axis_cc_tkeep_fifo ;
            end else if(m_axis_cc_tready) begin
               m_axis_cc_tdata  <= sel_m_axis_cc_tdata ;
               m_axis_cc_lbe    <= sel_m_axis_cc_lbe   ;
               m_axis_cc_tlast  <= sel_m_axis_cc_tlast ;
               m_axis_cc_tkeep  <= sel_m_axis_cc_tkeep ;
            end

            if(cnt_busy_ramout_r == 2'b10 || m_axis_cc_tready_r == 1'b1) begin
               m_axis_cc_tdata_r  <= m_axis_cc_tdata_fifo    ;
               m_axis_cc_lbe_r    <= m_axis_cc_lbe_fifo      ;
               m_axis_cc_tlast_r  <= m_axis_cc_tlast_fifo    ;
               m_axis_cc_tkeep_r  <= m_axis_cc_tkeep_fifo    ;
            end

            m_axis_cc_tready_r <= m_axis_cc_tready;
         end
      end
   end

   assign sel_m_axis_cc_tdata = (!m_axis_cc_tready_r && cnt_busy_ramout_r == 2'b11) ? m_axis_cc_tdata_r  : m_axis_cc_tdata_fifo ;
   assign sel_m_axis_cc_lbe   = (!m_axis_cc_tready_r && cnt_busy_ramout_r == 2'b11) ? m_axis_cc_lbe_r    : m_axis_cc_lbe_fifo   ;
   assign sel_m_axis_cc_tlast = (!m_axis_cc_tready_r && cnt_busy_ramout_r == 2'b11) ? m_axis_cc_tlast_r  : m_axis_cc_tlast_fifo ;
   assign sel_m_axis_cc_tkeep = (!m_axis_cc_tready_r && cnt_busy_ramout_r == 2'b11) ? m_axis_cc_tkeep_r  : m_axis_cc_tkeep_fifo ;

   assign axi_ack             = m_axis_cc_tvalid & m_axis_cc_tready   ;
endmodule