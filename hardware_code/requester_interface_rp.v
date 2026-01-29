
//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------

module requester_interface_rp #(
   parameter                                       PCIE_BUS_WIDTH             =          256
)(
   input    wire                                         clk                             ,
   input    wire                                         rstn                            ,
   input    wire                                         srst                            ,

   // Signals coming from
   input    wire  [3                      :0]            pcie_tfc_nph_av                 ,
   // AXI PCIE Interconnection
   output   wire                                         m_axis_rq_tlast                 ,
   output   wire  [PCIE_BUS_WIDTH-1       :0]            m_axis_rq_tdata                 ,
   output   wire  [59                     :0]            m_axis_rq_tuser                 ,
   output   wire  [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_rq_tkeep                 ,
   input    wire  [3                      :0]            m_axis_rq_tready                ,
   output   wire                                         m_axis_rq_tvalid                ,

   input    wire  [PCIE_BUS_WIDTH-1       :0]            s_axis_rc_tdata                 ,
   input    wire  [74                     :0]            s_axis_rc_tuser                 ,
   input    wire                                         s_axis_rc_tlast                 ,
   input    wire  [(PCIE_BUS_WIDTH/32)-1  :0]            s_axis_rc_tkeep                 ,
   input    wire                                         s_axis_rc_tvalid                ,
   output   wire                                         s_axis_rc_tready                ,

   output   wire  [18                     :0]            cfg_mgmt_addr                   ,
   output   wire                                         cfg_mgmt_write                  ,
   output   wire  [31                     :0]            cfg_mgmt_write_data             ,
   output   wire  [3                      :0]            cfg_mgmt_byte_enable            ,
   output   wire                                         cfg_mgmt_read                   ,
   input    wire  [31                     :0]            cfg_mgmt_read_data              ,
   input    wire                                         cfg_mgmt_read_write_done        ,
   output   wire                                         cfg_mgmt_type1_cfg_reg_access   ,

   //  Flow Bus backend for Submission interface
   // Master interface
   input    wire                                         init_ram_rden                   ,
   output   wire  [32-1                   :0]            init_ram_rdata                  ,
   output   wire  [32/8-1                 :0]            init_ram_rbe                    ,
   input    wire                                         init_ram_wren                   ,
   input    wire  [32-1                   :0]            init_ram_wdata                  ,
   input    wire  [32/8-1                 :0]            init_ram_wbe                    ,
   input    wire                                         init_ram_req                    ,
//    input    wire  [4-1                    :0]            init_ram_type                   ,
   input    wire  [32-1                   :0]            init_ram_addr                   ,
   output   wire                                         init_ram_rdy                    ,

   // Interface with nvme submission manager
   input    wire                                         sub_incr_ptr                    ,
   input    wire  [15                     :0]            sub_incr_data                   ,
   input    wire  [9                      :0]            sub_incr_q                      ,
   output   wire                                         sub_incr_ack
);
   //////////////////////////////
   // Internal signal in flb2axis
   //////////////////////////////
   wire                                         wr_valid                   ;
   wire  [7                      :0]            wr_tag                     ;
   wire                                         wr_ack                     ;
   wire                                         wr_type                    ;
   wire  [32-1       :0]                        wr_data                    ;
   wire  [32-1       :0]                        wr_baseaddr                ;
   wire  [7                      :0]            select_tag                 ;
   wire                                         select_tag_valid           ;
   wire  [32-1                   :0]            init_ram_rdata_i           ;
   wire                                         cfg_mgmt_read_sel          ;
   wire  [32/8-1                 :0]            init_ram_rbe_i             ;
   wire                                         replay_ram_rden            ;
   wire                                         replay_ram_wren            ;
   wire  [32-1                   :0]            replay_ram_wdata           ;
   wire  [32/8-1                 :0]            replay_ram_wbe             ;
   wire                                         replay_ram_req             ;
   wire  [32-1                   :0]            replay_ram_addr            ;
   wire                                         replay_ram_rdy             ;
   wire                                         select_replay_cfg          ;
   wire                                         init_ram_rden_i            ;
   wire                                         init_ram_wren_i            ;
   wire  [32-1                   :0]            init_ram_wdata_i           ;
   wire  [32/8-1                 :0]            init_ram_wbe_i             ;
   wire                                         init_ram_req_i             ;
   wire  [32-1                   :0]            init_ram_addr_i            ;
   wire                                         init_ram_rdy_i             ;
   wire                                         init_ram_rden_sel          ;
   wire                                         init_ram_wren_sel          ;
   wire  [32-1                   :0]            init_ram_wdata_sel         ;
   wire  [32/8-1                 :0]            init_ram_wbe_sel           ;
   wire                                         init_ram_req_sel           ;
   wire  [32-1                   :0]            init_ram_addr_sel          ;
   wire                                         init_ram_rdy_sel           ;

   assign init_ram_rdata = (cfg_mgmt_read_sel == 1'b1) ? cfg_mgmt_read_data :  init_ram_rdata_i;
   assign init_ram_rbe   = (cfg_mgmt_read_sel == 1'b1 & cfg_mgmt_read_write_done == 1'b1) ? 4'hf : init_ram_rbe_i;

   assign replay_ram_rdy  = (replay_ram_req == 1'b1) ? init_ram_rdy_sel : 1'b0;
   assign init_ram_rdy    = (replay_ram_req == 1'b0) ? init_ram_rdy_sel : 1'b0;

   assign init_ram_wren_sel     = (replay_ram_req == 1'b0) ?   init_ram_wren       : replay_ram_wren  ;
   assign init_ram_wdata_sel    = (replay_ram_req == 1'b0) ?   init_ram_wdata      : replay_ram_wdata ;
   assign init_ram_wbe_sel      = (replay_ram_req == 1'b0) ?   init_ram_wbe        : replay_ram_wbe   ;
   assign init_ram_req_sel      = (replay_ram_req == 1'b0) ?   init_ram_req        : replay_ram_req   ;
   assign init_ram_rden_sel     = (replay_ram_req == 1'b0) ?   init_ram_rden       : replay_ram_rden  ;
   assign init_ram_addr_sel     = (replay_ram_req == 1'b0) ?   init_ram_addr       : replay_ram_addr  ;

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Bus_valid_ready_synchronizer instanciation
   /////////////////////////////////////////////////////////////////////////////////////////////
   bus_valid_ready_synchronizer #(
      .DATA_WIDTH(32+32+2+32/8)
   ) bus_valid_ready_synchronizer_init_ram_inst (
      .clk              ( clk                                                                                                 ),
      .rstn             ( rstn                                                                                                ),
      .srst             ( srst                                                                                                ),
      .validin          ( init_ram_req_sel                                                                                    ),
      .datain           ( { init_ram_wbe_sel,init_ram_rden_sel,init_ram_wren_sel,init_ram_wdata_sel,init_ram_addr_sel}         ),
      .readyout         ( init_ram_rdy_sel                                                                                    ),
      .validout         ( init_ram_req_i                                                                                      ),
      .dataout          ( {init_ram_wbe_i,init_ram_rden_i,init_ram_wren_i,init_ram_wdata_i,init_ram_addr_i}    ),
      .readyin          ( init_ram_rdy_i                                                                                      )
   );

   requester_completion_rp #(
      .PCIE_BUS_WIDTH             (PCIE_BUS_WIDTH            )
   ) requester_completion_rp_inst (
      .clk                        (  clk                        ),
      .rstn                       (  rstn                       ),
      .srst                       (  srst                       ),
      .select_tag                 (  select_tag                 ),
      .select_tag_valid           (  select_tag_valid           ),
      .select_replay_cfg          (  select_replay_cfg          ),
      .init_ram_rdata             (  init_ram_rdata_i           ),
      .init_ram_rbe               (  init_ram_rbe_i             ),
      .s_axis_rc_tdata            (  s_axis_rc_tdata            ),
      .s_axis_rc_tuser            (  s_axis_rc_tuser            ),
      .s_axis_rc_tlast            (  s_axis_rc_tlast            ),
      .s_axis_rc_tkeep            (  s_axis_rc_tkeep            ),
      .s_axis_rc_tvalid           (  s_axis_rc_tvalid           ),
      .s_axis_rc_tready           (  s_axis_rc_tready           )
   );

   requester_request_rp #(
      .PCIE_BUS_WIDTH             (PCIE_BUS_WIDTH            )
   ) requester_request_rp_inst(
      .clk                             (  clk                              ),
      .rstn                            (  rstn                             ),
      .srst                            (  srst                             ),
      .wr_valid                        (  wr_valid                         ),
      .wr_type                         (  wr_type                          ),
      .wr_data                         (  wr_data                          ),
      .wr_baseaddr                     (  wr_baseaddr                      ),
      .wr_tag                          (  wr_tag                           ),
      .wr_ack                          (  wr_ack                           ),
      .pcie_tfc_nph_av                 (  pcie_tfc_nph_av                  ),
      .m_axis_rq_tlast                 (  m_axis_rq_tlast                  ),
      .m_axis_rq_tdata                 (  m_axis_rq_tdata                  ),
      .m_axis_rq_tuser                 (  m_axis_rq_tuser                  ),
      .m_axis_rq_tkeep                 (  m_axis_rq_tkeep                  ),
      .m_axis_rq_tready                (  m_axis_rq_tready                 ),
      .m_axis_rq_tvalid                (  m_axis_rq_tvalid                 ),
      .cfg_mgmt_addr                   (  cfg_mgmt_addr                    ),
      .cfg_mgmt_write                  (  cfg_mgmt_write                   ),
      .cfg_mgmt_write_data             (  cfg_mgmt_write_data              ),
      .cfg_mgmt_byte_enable            (  cfg_mgmt_byte_enable             ),
      .cfg_mgmt_read                   (  cfg_mgmt_read                    ),
      .cfg_mgmt_read_sel               (  cfg_mgmt_read_sel                ),
      .cfg_mgmt_read_write_done        (  cfg_mgmt_read_write_done         ),
      .cfg_mgmt_type1_cfg_reg_access   (  cfg_mgmt_type1_cfg_reg_access    ),
      .init_ram_rden                   (  init_ram_rden_i                  ),
      .init_ram_wren                   (  init_ram_wren_i                  ),
      .init_ram_wdata                  (  init_ram_wdata_i                 ),
      .init_ram_wbe                    (  init_ram_wbe_i                   ),
      .init_ram_req                    (  init_ram_req_i                   ),
      .init_ram_addr                   (  init_ram_addr_i                  ),
      .init_ram_rdy                    (  init_ram_rdy_i                   ),
      .sub_incr_ptr                    (  sub_incr_ptr                     ),
      .sub_incr_data                   (  sub_incr_data                    ),
      .sub_incr_q                      (  sub_incr_q                       ),
      .sub_incr_ack                    (  sub_incr_ack                     )
   );

   tag_available tag_available_inst (
      .clk                        (  clk                        ),
      .rstn                       (  rstn                       ),
      .srst                       (  srst                       ),
      .wr_valid                   (  wr_valid                   ),
      .wr_type                    (  wr_type                    ),
      .wr_data                    (  wr_data                    ),
      .wr_baseaddr                (  wr_baseaddr                ),
      .wr_tag                     (  wr_tag                     ),
      .wr_ack                     (  wr_ack                     ),
      .replay_ram_rden            (  replay_ram_rden            ),
      .replay_ram_wren            (  replay_ram_wren            ),
      .replay_ram_wdata           (  replay_ram_wdata           ),
      .replay_ram_wbe             (  replay_ram_wbe             ),
      .replay_ram_req             (  replay_ram_req             ),
      .replay_ram_addr            (  replay_ram_addr            ),
      .replay_ram_rdy             (  replay_ram_rdy             ),
      .select_tag                 (  select_tag                 ),
      .select_replay_cfg          (  select_replay_cfg          ),
      .select_free_valid          (  select_tag_valid           )
   );
endmodule
