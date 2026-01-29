
//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------

`include "AXI_flb_h.v"

module completer_completion_rp #(
   parameter                                       PCIE_BUS_WIDTH              =          256
)(
   input    wire                                         clk                      ,
   input    wire                                         rstn                     ,
   input    wire                                         srst                     ,

   input    wire  [2                      :0]            device_mps               ,

   input    wire                                         cpl_request              ,
   input    wire  [1                      :0]            cpl_address_type         ,
   input    wire  [7                      :0]            cpl_tag                  ,
   input    wire  [15                     :0]            cpl_requester_id         ,
   input    wire  [10                     :0]            cpl_length               ,
   input    wire  [6                      :0]            cpl_lower_address        ,
   input    wire  [2                      :0]            cpl_status               ,
   output   wire                                         cpl_ack                  ,
   output   wire                                         start_completion         ,
   output   wire                                         fifo_cc_almostfull       ,
   // AXI PCIE Interconnection
   output   wire  [PCIE_BUS_WIDTH-1       :0]            m_axis_cc_tdata          ,
   output   wire  [32                     :0]            m_axis_cc_tuser          ,
   output   wire                                         m_axis_cc_tlast          ,
   output   wire  [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_cc_tkeep          ,
   output   wire                                         m_axis_cc_tvalid         ,
   input    wire  [3                      :0]            m_axis_cc_tready         ,


   input    wire                                         data_ddr_cpl_FLB_Sop     ,
   input    wire                                         data_ddr_cpl_FLB_Eop     ,
   input    wire  [`FLB_RP_SIZE -1        :0]            data_ddr_cpl_FLB_Type    ,
   input    wire  [PCIE_BUS_WIDTH-1       :0]            data_ddr_cpl_FLB_Data    ,
   input    wire  [PCIE_BUS_WIDTH/8-1     :0]            data_ddr_cpl_FLB_Data_en ,
   input    wire                                         data_ddr_cpl_FLB_valid   ,
   input    wire  [64-1                   :0]            data_ddr_cpl_FLB_addr    ,
   output   wire                                         data_ddr_cpl_FLB_rdy     ,

`ifdef MODE_OPAL
   input   wire                                          opalbuf_rdvalid          ,
   input   wire   [63                     :0]            opalbuf_rddata           ,
//    input   wire                                          switch_to_opal           ,
`endif

   input    wire                                         subbuf_rdvalid           ,
   input    wire  [255                    :0]            subbuf_rddata            ,
   input    wire                                         dataprplistbuf_rdvalid   ,
   input    wire  [63                     :0]            dataprplistbuf_rddata
);

localparam WHOLE_AAWL_WIDTH   = 1+PCIE_BUS_WIDTH/32+PCIE_BUS_WIDTH;

wire                                          write_request            ;
wire   [PCIE_BUS_WIDTH/32-1    :0]            write_byteen             ;
wire                                          write_sof                ;
wire                                          write_eof                ;
wire   [PCIE_BUS_WIDTH-1       :0]            write_data               ;
wire                                          write_rdy                ;

wire                                          incr_ack                 ;
wire                                          decr_ack                 ;
wire                                          decr_ack2                ;
reg                                           incr_ack_r               ;
reg                                           incr_ack_rr              ;
reg                                           incr_ack_rrr             ;
reg                                           incr_ack_rrrr            ;
reg    [7                      :0]            tlp_complete_r           ;
reg                                           fifowr_wrreq             ;
reg    [WHOLE_AAWL_WIDTH-1     :0]            fifo_wrrl_wrdata         ;
wire                                          fifo_wrrl_wrfull         ;
wire                                          fifo_wrrl_rdreq          ;
wire   [WHOLE_AAWL_WIDTH-1     :0]            fifo_wrrl_rddata         ;
wire                                          fifo_wrrl_rdempty        ;
reg                                           fifo_wrrl_rdreq_r        ;
wire   [PCIE_BUS_WIDTH-1       :0]            m_axis_ram_tdata_i       ;
wire                                          m_axis_ram_tlast_i       ;
wire   [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_ram_tkeep_i       ;
wire                                          m_axis_ram_tvalid_i      ;
wire                                          m_axis_ram_tready_i      ;


reg    [PCIE_BUS_WIDTH-1       :0]            m_axis_cc_tdata_r        ;
reg                                           m_axis_cc_tlast_r        ;
reg    [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_cc_tkeep_r        ;
reg                                           m_axis_cc_tready_r       ;
wire   [PCIE_BUS_WIDTH-1       :0]            m_axis_cc_tdata_i        ;
wire                                          m_axis_cc_tlast_i        ;
wire   [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_cc_tkeep_i        ;
wire                                          m_axis_cc_tvalid_i       ;
wire                                          m_axis_cc_tready_i       ;
wire   [PCIE_BUS_WIDTH-1       :0]            sel_m_axis_cc_tdata      ;
wire                                          sel_m_axis_cc_tlast      ;
wire   [(PCIE_BUS_WIDTH/32)-1  :0]            sel_m_axis_cc_tkeep      ;
wire   [PCIE_BUS_WIDTH-1       :0]            m_axis_cc_tdata_fifo     ;
wire   [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_cc_tkeep_fifo     ;
wire                                          m_axis_cc_tlast_fifo     ;
reg    [1                      :0]            cnt_busy_ramout_r        ;

reg                                           transfer_progress_r      ;
reg                                           select_request_r         ;
wire   [PCIE_BUS_WIDTH-1       :0]            select_request_data      ;
wire                                          select_request_eof       ;
wire   [(PCIE_BUS_WIDTH/32)-1  :0]            select_request_byteen    ;
wire                                          select_ready             ;
wire                                          select_request           ;
reg    [11-1                   :0]            device_mps_in_dword      ;

tlpcpl_format #(
      .PCIE_BUS_WIDTH             (PCIE_BUS_WIDTH            )
) tlpcpl_format_inst (
   .clk                      (clk                     ),                 // clk
   .rstn                     (rstn                    ),                 // ASynchronous reset
   .srst                     (srst                    ),                 // Synchronous reset
   .max_payload              (device_mps_in_dword     ),                 //11'd128                 ),
   .select_FLB_request       (data_ddr_cpl_FLB_valid  ),
   .select_FLB_Type          (data_ddr_cpl_FLB_Type   ),
   .select_FLB_Data          (data_ddr_cpl_FLB_Data   ),
   .select_FLB_Data_en       (data_ddr_cpl_FLB_Data_en),
   .select_FLB_addr          (data_ddr_cpl_FLB_addr   ),
   .select_FLB_eof           (data_ddr_cpl_FLB_Eop    ),
   .select_FLB_sof           (data_ddr_cpl_FLB_Sop    ),
   .select_FLB_ready         (data_ddr_cpl_FLB_rdy    ),
   .write_request            (write_request           ),
   .write_byteen             (write_byteen            ),
   .write_sof                (write_sof               ),
   .write_eof                (write_eof               ),
   .write_data               (write_data              ),
   .write_lbe                (/*open*/                ),
   .write_rdy                (write_rdy               )
);

ram_completion #(
      .PCIE_BUS_WIDTH             (PCIE_BUS_WIDTH            )
) ram_completion_inst (
   .clk                      (clk                     ),                 // clk
   .rstn                     (rstn                    ),                 // ASynchronous reset
   .srst                     (srst                    ),                 // Synchronous reset

   .cpl_request              (cpl_request             ),
   .cpl_address_type         (cpl_address_type        ),
   .cpl_tag                  (cpl_tag                 ),
   .cpl_requester_id         (cpl_requester_id        ),
   .cpl_length               (cpl_length              ),
   .cpl_lower_address        (cpl_lower_address       ),
   .cpl_status               (cpl_status              ),
   .cpl_ack                  (cpl_ack                 ),
   .start_completion         (start_completion        ),
   .fifo_cc_almostfull       (fifo_cc_almostfull      ),
   .m_axis_cc_tdata          (m_axis_ram_tdata_i      ),
   .m_axis_cc_lbe            (/*open*/                ),
   .m_axis_cc_tlast          (m_axis_ram_tlast_i      ),
   .m_axis_cc_tkeep          (m_axis_ram_tkeep_i      ),
   .m_axis_cc_tvalid         (m_axis_ram_tvalid_i     ),
   .m_axis_cc_tready         (m_axis_ram_tready_i     ),
`ifdef MODE_OPAL
   .opalbuf_rdvalid          (opalbuf_rdvalid         ),
   .opalbuf_rddata           (opalbuf_rddata          ),
//    .switch_to_opal           (switch_to_opal          ),
`endif
   .subbuf_rdvalid           (subbuf_rdvalid          ),
   .subbuf_rddata            (subbuf_rddata           ),
   .dataprplistbuf_rdvalid   (dataprplistbuf_rdvalid  ),
   .dataprplistbuf_rddata    (dataprplistbuf_rddata   )
);

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Arbitration of Request
   /////////////////////////////////////////////////////////////////////////////////////////////
   always @(posedge clk or negedge rstn)
   begin : REGISTER_ROUNDROBIN
      if (rstn == 1'b0) begin
         transfer_progress_r          <= 1'b0;
         select_request_r             <= 1'b0;
         device_mps_in_dword          <= 11'd0;
      end else begin
         if(srst == 1'b1) begin
            transfer_progress_r          <= 1'b0;
            select_request_r             <= 1'b0;
            device_mps_in_dword          <= 11'd0;
         end else begin
            case(device_mps)
            3'd0: device_mps_in_dword <= 11'd32;
            3'd1: device_mps_in_dword <= 11'd64;
            3'd2: device_mps_in_dword <= 11'd128;
            3'd3: device_mps_in_dword <= 11'd256;
            3'd4: device_mps_in_dword <= 11'd512;
            3'd5: device_mps_in_dword <= 11'd1024;
            default : device_mps_in_dword <= 11'd32;
            endcase

            if(transfer_progress_r == 1'b1 && select_request_eof == 1'b1 && select_ready == 1'b1) begin
               if( m_axis_ram_tvalid_i == 1'b1 && select_request_r == 1'b0)
                  transfer_progress_r <= 1'b1;
               else
                  transfer_progress_r <= 1'b0;
            end else if(transfer_progress_r == 1'b0) begin
               transfer_progress_r <= m_axis_ram_tvalid_i | write_request;
            end

            if(transfer_progress_r == 1'b0) begin
               if(m_axis_ram_tvalid_i == 1'b1)
                  select_request_r <= 1'b1;
               else
                  select_request_r <= 1'b0;
            end else if(transfer_progress_r == 1'b1 && select_request_eof == 1'b1 && select_ready == 1'b1) begin
               if(m_axis_ram_tvalid_i == 1'b1 && select_request_r == 1'b0)
                  select_request_r <= 1'b1;
               else
                  select_request_r <= 1'b0;
            end
         end
      end
   end

   //Select_request mux
   assign select_request         = (select_request_r == 1'b1)     ?  m_axis_ram_tvalid_i & transfer_progress_r  :
                                   (transfer_progress_r == 1'b0)  ?  write_request & ~m_axis_ram_tvalid_i       :
                                   write_request;

   assign select_request_data    = (select_request_r == 1'b1)  ? m_axis_ram_tdata_i   : write_data   ;
   assign select_request_eof     = (select_request_r == 1'b1)  ? m_axis_ram_tlast_i   : write_eof    ;

   generate
      genvar a_v;
      for (a_v=0; a_v<(PCIE_BUS_WIDTH/32); a_v=a_v+1) begin: loop_read_gen
         assign select_request_byteen[a_v] = (select_request_r == 1'b1)  ? m_axis_ram_tkeep_i[a_v]    : write_byteen[a_v] ;
      end
   endgenerate

   assign m_axis_ram_tready_i    = (select_request_r == 1'b1 && transfer_progress_r == 1'b1) ? select_ready : 1'b0;
   assign write_rdy              = (transfer_progress_r == 1'b0) ?  select_ready & ~m_axis_ram_tvalid_i : (select_request_r == 1'b0) ? select_ready : 1'b0;




   //////////////////////////////////////////////////////////////////////////////////////////////
   // Synchronize Ready signal
   //////////////////////////////////////////////////////////////////////////////////////////////
   bus_valid_ready_synchronizer #(
      .DATA_WIDTH (PCIE_BUS_WIDTH + (PCIE_BUS_WIDTH/32) + 1                                    )
   ) AXIS_RQ_BUS_SYNC_INST (
      .clk        (clk                                                                           ),
      .rstn       (rstn                                                                          ),
      .srst       (srst                                                                          ),
      .validin    (select_request                                                                ),
      .datain     ({select_request_byteen,select_request_eof,select_request_data}                ),
      .readyout   (select_ready                                                                  ),
      .validout   (m_axis_cc_tvalid                                                              ),
      .dataout    ({m_axis_cc_tkeep,m_axis_cc_tlast,m_axis_cc_tdata}                             ),
      .readyin    (m_axis_cc_tready                                                              )
   );

   //////////////////////////////////////////////////////////////////////////////////////////////
   // Synchronize Ready signal
   //////////////////////////////////////////////////////////////////////////////////////////////
   /*always @(negedge rstn or posedge clk)
   begin : REGISTER_READ_LATENCY_FOR_TRANSMIT
      if(rstn == 1'b0) begin
         m_axis_cc_tready_r  <= 1'b0;
         fifo_wrrl_rdreq_r   <= 1'b0;
         fifowr_wrreq        <= 1'b0;
         fifo_wrrl_wrdata    <= {WHOLE_AAWL_WIDTH{1'b0}};
      end else begin
         if(srst == 1'b1) begin
            m_axis_cc_tready_r<= 1'b0;
            fifo_wrrl_rdreq_r <= 1'b0;
            fifowr_wrreq      <= 1'b0;
            fifo_wrrl_wrdata  <= {WHOLE_AAWL_WIDTH{1'b0}};
         end else begin
            m_axis_cc_tready_r     <= m_axis_cc_tready;
            fifo_wrrl_rdreq_r      <= fifo_wrrl_rdreq;

            if(~fifo_wrrl_wrfull)
               fifowr_wrreq    <= m_axis_cc_tvalid_i;

            if(m_axis_cc_tvalid_i & ~fifo_wrrl_wrfull)
               fifo_wrrl_wrdata   <= {m_axis_cc_tkeep_i,m_axis_cc_tlast_i,m_axis_cc_tdata_i};
         end
      end
   end

   assign m_axis_cc_tready_i = ~fifo_wrrl_wrfull;

   assign fifo_wrrl_rdreq    = (fifo_wrrl_rdreq_r == 1'b1 && m_axis_cc_tlast_fifo == 1'b1  && tlp_complete_r ==8'h1 )   ?    1'b0 :
                               (cnt_busy_ramout_r < 2'b11  && tlp_complete_r !=8'h0   )                                 ?    ~fifo_wrrl_rdempty :
                                                                                                                             1'b0 ;

   assign incr_ack                = m_axis_cc_tvalid_i & m_axis_cc_tlast_i & ~fifo_wrrl_wrfull;
   assign decr_ack                = fifo_wrrl_rdreq_r & m_axis_cc_tlast_fifo;
   assign decr_ack2               = m_axis_cc_tvalid & m_axis_cc_tready;

   assign m_axis_cc_tdata_fifo    = fifo_wrrl_rddata[PCIE_BUS_WIDTH-1     :0];
   assign m_axis_cc_tkeep_fifo    = fifo_wrrl_rddata[PCIE_BUS_WIDTH+1    +:(PCIE_BUS_WIDTH/32)];
   assign m_axis_cc_tlast_fifo    = fifo_wrrl_rddata[PCIE_BUS_WIDTH];


   always@(negedge rstn or posedge clk)
   begin : WRITE_RQ_PROC
      if(rstn == 1'b0) begin
         m_axis_cc_tdata    <= {PCIE_BUS_WIDTH{1'b0}};
         m_axis_cc_tlast    <= 1'b0;
         m_axis_cc_tkeep    <= {(PCIE_BUS_WIDTH/32){1'b0}};
         m_axis_cc_tvalid   <= 1'b0;

         m_axis_cc_tdata_r  <= {PCIE_BUS_WIDTH{1'b0}};
         m_axis_cc_tlast_r  <= 1'b0;
         m_axis_cc_tkeep_r  <= {(PCIE_BUS_WIDTH/32){1'b0}};

         m_axis_cc_tready_r <= 1'b0;
         cnt_busy_ramout_r  <= 2'b0;
      end else begin
         if(srst == 1'b1) begin
            m_axis_cc_tdata    <= {PCIE_BUS_WIDTH{1'b0}};
            m_axis_cc_tlast    <= 1'b0;
            m_axis_cc_tkeep    <= {(PCIE_BUS_WIDTH/32){1'b0}};
            m_axis_cc_tvalid   <= 1'b0;

            m_axis_cc_tdata_r  <= {PCIE_BUS_WIDTH{1'b0}};
            m_axis_cc_tlast_r  <= 1'b0;
            m_axis_cc_tkeep_r  <= {(PCIE_BUS_WIDTH/32){1'b0}};

            m_axis_cc_tready_r <= 1'b0;
            cnt_busy_ramout_r  <= 2'b0;
         end else begin
            if(fifo_wrrl_rdreq && !decr_ack2)
               cnt_busy_ramout_r <= cnt_busy_ramout_r + 1'b1;
            else if(!fifo_wrrl_rdreq && decr_ack2)
               cnt_busy_ramout_r <= cnt_busy_ramout_r - 1'b1;

            if(cnt_busy_ramout_r ==1 && !m_axis_cc_tvalid)
               m_axis_cc_tvalid   <= 1'b1;
            else if(cnt_busy_ramout_r == 1 && m_axis_cc_tready)
               m_axis_cc_tvalid   <= 1'b0;

            if(cnt_busy_ramout_r > 2'b00 && !m_axis_cc_tvalid) begin
               m_axis_cc_tdata  <= m_axis_cc_tdata_fifo ;
               m_axis_cc_tlast  <= m_axis_cc_tlast_fifo ;
               m_axis_cc_tkeep  <= m_axis_cc_tkeep_fifo ;
            end else if(m_axis_cc_tready) begin
               m_axis_cc_tdata  <= sel_m_axis_cc_tdata ;
               m_axis_cc_tlast  <= sel_m_axis_cc_tlast ;
               m_axis_cc_tkeep  <= sel_m_axis_cc_tkeep ;
            end

            if(cnt_busy_ramout_r == 2'b10 || m_axis_cc_tready_r == 1'b1) begin
               m_axis_cc_tdata_r  <= m_axis_cc_tdata_fifo    ;
               m_axis_cc_tlast_r  <= m_axis_cc_tlast_fifo    ;
               m_axis_cc_tkeep_r  <= m_axis_cc_tkeep_fifo    ;
            end

            m_axis_cc_tready_r <= m_axis_cc_tready;
         end
      end
   end

   assign sel_m_axis_cc_tdata = (!m_axis_cc_tready_r && cnt_busy_ramout_r == 2'b11) ? m_axis_cc_tdata_r  : m_axis_cc_tdata_fifo ;
   assign sel_m_axis_cc_tlast = (!m_axis_cc_tready_r && cnt_busy_ramout_r == 2'b11) ? m_axis_cc_tlast_r  : m_axis_cc_tlast_fifo ;
   assign sel_m_axis_cc_tkeep = (!m_axis_cc_tready_r && cnt_busy_ramout_r == 2'b11) ? m_axis_cc_tkeep_r  : m_axis_cc_tkeep_fifo ;

   always @(posedge clk or negedge rstn)
   if (rstn == 1'b0) begin
      tlp_complete_r             <= 8'b0;
      incr_ack_r                 <= 1'b0;
      incr_ack_rr                <= 1'b0;
      incr_ack_rrr               <= 1'b0;
      incr_ack_rrrr              <= 1'b0;
   end else begin
      if(srst == 1'b1) begin
         tlp_complete_r             <= 8'b0;
         incr_ack_r                 <= 1'b0;
         incr_ack_rr                <= 1'b0;
         incr_ack_rrr               <= 1'b0;
         incr_ack_rrrr              <= 1'b0;
      end else begin
         incr_ack_r    <= incr_ack;
         incr_ack_rr   <= incr_ack_r;
         incr_ack_rrr  <= incr_ack_rr;
         incr_ack_rrrr <= incr_ack_rrr;

         if(incr_ack_rrrr == 1'b1 && decr_ack == 1'b0)
           tlp_complete_r   <= tlp_complete_r+1'b1;
         else if(incr_ack_rrrr == 1'b0 && decr_ack == 1'b1)
           tlp_complete_r   <= tlp_complete_r-1'b1;
      end
   end

   afifo   #( .ADDR_WIDTH   (  8                          ),
              .DATA_WIDTH   (  WHOLE_AAWL_WIDTH           )
   ) afifo_inst (
      .wrclk         ( clk               ),
      .wrrstn        ( rstn              ),
      .wrsrst        ( srst              ),
      .rdclk         ( clk               ),
      .rdrstn        ( rstn              ),
      .rdsrst        ( srst              ),
      .wrreq         ( fifowr_wrreq      ),
      .wrdata        ( fifo_wrrl_wrdata  ),
      .wrfull        ( fifo_wrrl_wrfull  ),
      .wr_almostfull (                   ),
      .wr_avail      (                   ),
      .rdreq         ( fifo_wrrl_rdreq   ),
      .rddata        ( fifo_wrrl_rddata  ),
      .rdempty       ( fifo_wrrl_rdempty )
   );*/
   assign m_axis_cc_tuser = 32'b0;
endmodule