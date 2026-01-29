

//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------

module requester_request_rp #(
   parameter                                       PCIE_BUS_WIDTH             =          256
)(
   input    wire                                         clk                             ,
   input    wire                                         rstn                            ,
   input    wire                                         srst                            ,

   // Signals coming from
   input    wire  [3                      :0]            pcie_tfc_nph_av                 ,

   input    wire                                         wr_valid                        ,
   output   wire                                         wr_type                         ,
   output   wire  [32-1                   :0]            wr_data                         ,
   output   wire  [32-1                   :0]            wr_baseaddr                     ,

   input    wire  [7                      :0]            wr_tag                          ,
   output   wire                                         wr_ack                          ,

   // AXI PCIE Interconnection
   output   wire                                         m_axis_rq_tlast                 ,
   output   wire  [PCIE_BUS_WIDTH-1       :0]            m_axis_rq_tdata                 ,
   output   wire  [59                     :0]            m_axis_rq_tuser                 ,
   output   wire  [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_rq_tkeep                 ,
   input    wire  [3                      :0]            m_axis_rq_tready                ,
   output   wire                                         m_axis_rq_tvalid                ,

   output   reg   [18                     :0]            cfg_mgmt_addr                   ,
   output   reg                                          cfg_mgmt_write                  ,
   output   reg   [31                     :0]            cfg_mgmt_write_data             ,
   output   reg   [3                      :0]            cfg_mgmt_byte_enable            ,
   output   reg                                          cfg_mgmt_read                   ,
   output   reg                                          cfg_mgmt_read_sel               ,
   input    wire                                         cfg_mgmt_read_write_done        ,
   output   reg                                          cfg_mgmt_type1_cfg_reg_access   ,

   //  Flow Bus backend for Submission interface
   // Master interface
   input    wire                                         init_ram_rden                   ,
   input    wire                                         init_ram_wren                   ,
   input    wire  [32-1                   :0]            init_ram_wdata                  ,
   input    wire  [32/8-1                 :0]            init_ram_wbe                    ,
   input    wire                                         init_ram_req                    ,
   input    wire  [32-1                   :0]            init_ram_addr                   ,
   output   wire                                         init_ram_rdy                    ,

   // Interface with nvme submission manager
   input    wire                                         sub_incr_ptr                    ,
   input    wire  [15                     :0]            sub_incr_data                   ,
   input    wire  [9                      :0]            sub_incr_q                      ,
   output   wire                                         sub_incr_ack
);

   wire                                         sub_incr_ptr_i             ;
   wire  [15                     :0]            sub_incr_data_i            ;
   wire  [9                      :0]            sub_incr_q_i               ;
   wire                                         sub_incr_ack_i             ;
   wire                                         init_ram_rden_i            ;
   wire                                         init_ram_wren_i            ;
   wire  [32-1                   :0]            init_ram_wdata_i           ;
   wire  [32/8-1                 :0]            init_ram_wbe_i             ;
   wire                                         init_ram_req_i             ;
   wire  [4-1                    :0]            init_ram_type_i            ;
   wire  [32-1                   :0]            init_ram_addr_i            ;
   wire                                         init_ram_rdy_i             ;
   wire                                         init_ram_req_sel           ;
   wire                                         init_ram_rdy_sel           ;
   wire                                         init_ram_req_cfg           ;
   wire                                         init_ram_rdy_cfg           ;
   reg   [31                     :0]            ep_base_addr_bar1          ;
   reg   [31                     :0]            ep_base_addr_bar0          ;
   wire  [3                      :0]            init_ram_type              ;
   wire                                         select_FLB_request         ;
   wire  [3                      :0]            select_FLB_Type            ;
   wire  [32-1                   :0]            select_FLB_Data            ;
   wire  [32/8-1                 :0]            select_FLB_Data_en         ;
   wire  [64-1                   :0]            select_FLB_addr            ;
   wire                                         select_FLB_wren            ;
   wire                                         select_FLB_ready           ;
   reg                                          flb_wr_progress_r          ;
   reg                                          select_flb_wr_request_r    ;

   assign init_ram_type = (init_ram_req == 1'b1 && init_ram_wren == 1'b1 && init_ram_addr > 32'h1FFC && init_ram_addr < 32'h4000) ? 4'b0001 :
                          (init_ram_req == 1'b1 && init_ram_rden == 1'b1 && init_ram_addr > 32'h0FFC && init_ram_addr < 32'h2000) ? 4'b1000 :
                          //(init_ram_req == 1'b1 && init_ram_rden == 1'b1 && init_ram_addr < 32'h1000)                             ? 4'b1001 :
                          (init_ram_req == 1'b1 && init_ram_wren == 1'b1 && init_ram_addr > 32'h0FFC && init_ram_addr < 32'h2000) ? 4'b1010 :
                          //(init_ram_req == 1'b1 && init_ram_wren == 1'b1 && init_ram_addr < 32'h1000)                             ? 4'b1011 :
                                                                                                                                    4'b0000 ;

   assign init_ram_req_sel  = (init_ram_addr < 32'h1000)? 1'b0 : init_ram_req;
   assign init_ram_req_cfg  = (init_ram_addr < 32'h1000)? 1'b1 : 1'b0;
   assign init_ram_rdy      = (init_ram_addr < 32'h1000)? init_ram_rdy_cfg  : init_ram_rdy_sel;
   assign init_ram_rdy_cfg  = ~cfg_mgmt_write & ~cfg_mgmt_read;

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Bus_valid_ready_synchronizer instanciation
   /////////////////////////////////////////////////////////////////////////////////////////////
   bus_valid_ready_synchronizer #(
      .DATA_WIDTH(32+32/8+32+2+4)
   ) bus_valid_ready_synchronizer_init_ram_inst (
      .clk              ( clk                                                                                                  ),
      .rstn             ( rstn                                                                                                 ),
      .srst             ( srst                                                                                                 ),
      .validin          ( init_ram_req_sel                                                                                     ),
      .datain           ( {init_ram_type,init_ram_rden,init_ram_wren,init_ram_wdata,init_ram_wbe,init_ram_addr}                ),
      .readyout         ( init_ram_rdy_sel                                                                                     ),
      .validout         ( init_ram_req_i                                                                                       ),
      .dataout          ( {init_ram_type_i,init_ram_rden_i,init_ram_wren_i,init_ram_wdata_i,init_ram_wbe_i,init_ram_addr_i}    ),
      .readyin          ( init_ram_rdy_i                                                                                       )
   );

   bus_valid_ready_synchronizer #(
      .DATA_WIDTH(16+10)
   ) bus_valid_ready_synchronizer_sub_incr_inst (
      .clk              ( clk                                                                  ),
      .rstn             ( rstn                                                                 ),
      .srst             ( srst                                                                 ),
      .validin          ( sub_incr_ptr                                                         ),
      .datain           ( {sub_incr_data,sub_incr_q                             }              ),
      .readyout         ( sub_incr_ack                                                         ),
      .validout         ( sub_incr_ptr_i                                                       ),
      .dataout          ( {sub_incr_data_i,sub_incr_q_i                         }              ),
      .readyin          ( sub_incr_ack_i                                                       )
   );

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Merged TLP/Data merged management
   /////////////////////////////////////////////////////////////////////////////////////////////
   tlprq_format #(
      .PCIE_BUS_WIDTH             (PCIE_BUS_WIDTH            )
   ) tlprq_format_inst (
      .clk                       (  clk                         ),
      .rstn                      (  rstn                        ),
      .srst                      (  srst                        ),
      .pcie_tfc_nph_av           (  pcie_tfc_nph_av             ),
      .wr_valid                  (  wr_valid                    ),
      .wr_type                   (  wr_type                     ),
      .wr_data                   (  wr_data                     ),
      .wr_baseaddr               (  wr_baseaddr                 ),
      .wr_tag                    (  wr_tag                      ),
      .wr_ack                    (  wr_ack                      ),
      .select_FLB_Data           (  select_FLB_Data             ),
      .select_FLB_Type           (  select_FLB_Type             ),
      .select_FLB_wren           (  select_FLB_wren             ),
      .select_FLB_Data_en        (  select_FLB_Data_en          ),
      .select_FLB_request        (  select_FLB_request          ),
      .select_FLB_addr           (  select_FLB_addr             ),
      .select_FLB_ready          (  select_FLB_ready            ),
      .m_axis_rq_tlast           (  m_axis_rq_tlast             ),
      .m_axis_rq_tdata           (  m_axis_rq_tdata             ),
      .m_axis_rq_tuser           (  m_axis_rq_tuser             ),
      .m_axis_rq_tkeep           (  m_axis_rq_tkeep             ),
      .m_axis_rq_tready          (  m_axis_rq_tready            ),
      .m_axis_rq_tvalid          (  m_axis_rq_tvalid            )
   );


   /////////////////////////////////////////////////////////////////////////////////////////////
   // Arbitration of Request
   /////////////////////////////////////////////////////////////////////////////////////////////
   always @(posedge clk or negedge rstn)
   begin : REGISTER_ROUNDROBIN
      if (rstn == 1'b0) begin
         flb_wr_progress_r               <= 1'b0;
         select_flb_wr_request_r         <= 1'b0;
         ep_base_addr_bar1               <= 32'b0;
         ep_base_addr_bar0               <= 32'b0;

         cfg_mgmt_addr                   <= 19'b0;
         cfg_mgmt_write                  <= 1'b0;
         cfg_mgmt_write_data             <= 32'b0;
         cfg_mgmt_byte_enable            <= 4'b0;
         cfg_mgmt_read                   <= 1'b0;
         cfg_mgmt_type1_cfg_reg_access   <= 1'b0;
         cfg_mgmt_read_sel               <= 1'b0;
      end else begin
         if(srst == 1'b1) begin
            flb_wr_progress_r               <= 1'b0;
            select_flb_wr_request_r         <= 1'b0;
            ep_base_addr_bar1               <= 32'b0;
            ep_base_addr_bar0               <= 32'b0;

            cfg_mgmt_addr                   <= 19'b0;
            cfg_mgmt_write                  <= 1'b0;
            cfg_mgmt_write_data             <= 32'b0;
            cfg_mgmt_byte_enable            <= 4'b0;
            cfg_mgmt_read                   <= 1'b0;
            cfg_mgmt_type1_cfg_reg_access   <= 1'b0;
            cfg_mgmt_read_sel               <= 1'b0;
         end else begin
             if( cfg_mgmt_read == 1'b1)
                cfg_mgmt_read_sel           <= 1'b1;
             else cfg_mgmt_read_sel           <= 1'b0;

             if(cfg_mgmt_read_write_done == 1'b1 && (cfg_mgmt_write == 1'b1 || cfg_mgmt_read == 1'b1) ) begin
               cfg_mgmt_write               <= 1'b0;
               cfg_mgmt_read                <= 1'b0;
             end else if(init_ram_req_cfg == 1'b1 && init_ram_rdy_cfg == 1'b1) begin
               cfg_mgmt_write               <= init_ram_wren;
               cfg_mgmt_read                <= init_ram_rden;
            end

            if(init_ram_req_cfg == 1'b1 && init_ram_rdy_cfg == 1'b1) begin
               cfg_mgmt_addr                   <= init_ram_addr[20:2];
               cfg_mgmt_write_data             <= init_ram_wdata;
               cfg_mgmt_byte_enable            <= init_ram_wbe;
            end

            cfg_mgmt_type1_cfg_reg_access   <= 1'b1;

            if(flb_wr_progress_r == 1'b1 && select_FLB_ready == 1'b1) begin
               if(select_flb_wr_request_r== 1'b0)
                  flb_wr_progress_r <= sub_incr_ptr_i;
               else
                  flb_wr_progress_r <= init_ram_req_i;
            end else if(flb_wr_progress_r == 1'b0)
               flb_wr_progress_r <= sub_incr_ptr_i | init_ram_req_i;

            if(flb_wr_progress_r == 1'b0) begin
               select_flb_wr_request_r <= sub_incr_ptr_i;
            end else if(flb_wr_progress_r == 1'b1 && select_FLB_ready == 1'b1) begin
               if(select_flb_wr_request_r== 1'b0)
                  select_flb_wr_request_r <= sub_incr_ptr_i;
               else if(init_ram_req_i == 1'b1)
                  select_flb_wr_request_r <= 1'b0;
            end

            if(init_ram_req_i == 1'b1 && init_ram_wren_i == 1'b1 && init_ram_rdy_i == 1'b1 && init_ram_addr_i == 32'h1014)
               ep_base_addr_bar1         <= init_ram_wdata_i;

            if(init_ram_req_i == 1'b1 && init_ram_wren_i == 1'b1 && init_ram_rdy_i == 1'b1 && init_ram_addr_i == 32'h1010)
               ep_base_addr_bar0         <= init_ram_wdata_i;
         end
      end
   end

   assign select_FLB_request     = ((sub_incr_ptr_i & select_flb_wr_request_r) | init_ram_req_i) & flb_wr_progress_r    ;
   assign select_FLB_Type        = (select_flb_wr_request_r == 1'b1)   ? 4'b1                   : init_ram_type_i       ;
   assign select_FLB_Data        = (select_flb_wr_request_r == 1'b1)   ? {16'b0,sub_incr_data_i}: init_ram_wdata_i      ;
   assign select_FLB_Data_en     = (select_flb_wr_request_r == 1'b1)   ? 4'hF                   : init_ram_wbe_i        ;
   assign select_FLB_addr        = (select_flb_wr_request_r == 1'b1)   ? {ep_base_addr_bar1,ep_base_addr_bar0[31:13],1'b1,sub_incr_q_i,2'b0}   : (init_ram_type_i[3:1] == 3'b0) ? {ep_base_addr_bar1,ep_base_addr_bar0 | (init_ram_addr_i-16'h2000)} : {36'h0,init_ram_addr_i[11:0]}       ;



   assign select_FLB_wren        = (select_flb_wr_request_r == 1'b1)   ? 1'b1                   : init_ram_wren_i       ;

   assign sub_incr_ack_i         = (select_flb_wr_request_r == 1'b1 && flb_wr_progress_r == 1'b1) ? select_FLB_ready : 1'b0;
   assign init_ram_rdy_i         = (select_flb_wr_request_r == 1'b0 && flb_wr_progress_r == 1'b1) ? select_FLB_ready : 1'b0;
endmodule
