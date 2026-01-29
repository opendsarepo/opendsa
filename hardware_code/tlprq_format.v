//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP-Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------

module tlprq_format #(
   parameter                                       PCIE_BUS_WIDTH             =          256
)(
   input    wire                                         clk                      ,                 // clk
   input    wire                                         rstn                     ,                 // ASynchronous reset
   input    wire                                         srst                     ,                 // Synchronous reset

   input    wire  [3                      :0]            pcie_tfc_nph_av          ,
   input    wire                                         wr_valid                 ,
   output   wire                                         wr_type                  ,
   output   wire  [32-1                   :0]            wr_data                  ,
   output   wire  [32-1                   :0]            wr_baseaddr              ,
   input    wire  [7                      :0]            wr_tag                   ,
   output   wire                                         wr_ack                   ,

   // Flow Bus backend for Submission interface
   input    wire                                         select_FLB_request       ,
   input    wire  [3                      :0]            select_FLB_Type          ,
   input    wire  [32-1                   :0]            select_FLB_Data          ,
   input    wire  [32/8-1                 :0]            select_FLB_Data_en       ,
   input    wire  [64-1                   :0]            select_FLB_addr          ,
   input    wire                                         select_FLB_wren          ,
   output   wire                                         select_FLB_ready         ,

   output   reg                                          m_axis_rq_tlast          ,
   output   wire  [PCIE_BUS_WIDTH-1       :0]            m_axis_rq_tdata          ,
   output   wire  [59                     :0]            m_axis_rq_tuser          ,
   output   wire  [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_rq_tkeep          ,
   input    wire  [3                      :0]            m_axis_rq_tready         ,
   output   reg                                          m_axis_rq_tvalid
);
   /////////////////////////////////////////////////////////////////////////////////////////////
   // INTERNAL SIGNALS
   /////////////////////////////////////////////////////////////////////////////////////////////
   wire                                         select_FLB_request_i    ;
   wire  [3                      :0]            select_FLB_Type_i       ;
   wire  [32-1                   :0]            select_FLB_Data_i       ;
   wire  [32/8-1                 :0]            select_FLB_Data_en_i    ;
   wire  [64-1                   :0]            select_FLB_addr_i       ;
   wire                                         select_FLB_wren_i       ;
   wire                                         select_FLB_ready_i      ;
   reg                                          select_FLB_ready_r      ;
   wire  [16-1                   :0]            completer_id            ;
   reg   [127                    :0]            tlp_r                   ;
   reg   [127                    :0]            tlp_data_r              ;
   reg   [3                      :0]            tlp_keep_r              ;
   reg   [3                      :0]            tlp_data_keep_r         ;
   reg                                          tlp_is_write            ;
   reg                                          tlp_is_write_r          ;

   assign m_axis_rq_tuser    = {52'h0,4'h0,4'hf};
   //------------------------------------------------------------
   // Management
   //------------------------------------------------------------
   always @(posedge clk or negedge rstn)
   begin : REGISTER_RD_FLB
      if (rstn == 1'b0) begin
         m_axis_rq_tvalid           <= 1'b0;
         m_axis_rq_tlast            <= 1'b0;

         tlp_data_r                 <= 128'b0;
         tlp_r                      <= 128'b0;
         tlp_data_keep_r            <= 4'h0;
         tlp_keep_r                 <= 4'h0;
         tlp_is_write               <= 1'b0;
         select_FLB_ready_r         <= 1'b1;
         tlp_is_write_r             <= 1'b0;
      end else begin
         if(srst == 1'b1) begin
            m_axis_rq_tvalid           <= 1'b0;
            m_axis_rq_tlast            <= 1'b0;
            tlp_data_r                 <= 128'b0;
            tlp_r                      <= 128'b0;
            tlp_data_keep_r            <= 4'h0;
            tlp_keep_r                 <= 4'h0;
            tlp_is_write               <= 1'b0;
            select_FLB_ready_r         <= 1'b1;
            tlp_is_write_r             <= 1'b0;
         end else begin
            if(select_FLB_request_i & select_FLB_ready_i) begin
               if(select_FLB_wren_i == 1'b1) begin
                  tlp_data_keep_r            <= 4'h1;
                  tlp_keep_r                 <= 4'hf;
                  tlp_data_r                 <=  {96'h0,select_FLB_Data_i};
                  tlp_r[95:0] <= { 16'h0,1'b0,select_FLB_Type_i,11'd1,select_FLB_addr_i};

                  if(select_FLB_Type_i[3] == 1'b0)
                     tlp_r[127:96] <=  {7'h0,1'b1,completer_id,8'h0};
                  else
                     tlp_r[127:96] <=  {7'h0,1'b1,completer_id,wr_tag};
               end else begin
                  tlp_data_keep_r         <= 4'h0;
                  tlp_keep_r              <= 4'hf;
                  tlp_data_r              <= 128'h0;
                  tlp_r                   <= {7'h0,1'b1,completer_id,wr_tag,16'h0,1'b0,select_FLB_Type_i,11'd1,select_FLB_addr_i};
               end
            end else if(m_axis_rq_tvalid && m_axis_rq_tlast && m_axis_rq_tready[0]) begin
               tlp_data_keep_r            <= 4'h0;
               tlp_keep_r                 <= 4'h0;
               tlp_data_r                 <= 128'h0;
               tlp_r                      <= 128'h0;
            end

            if(select_FLB_request_i & select_FLB_ready_i) begin
               m_axis_rq_tvalid               <= 1'b1;

               if( PCIE_BUS_WIDTH == 256 || select_FLB_wren_i == 1'b0) begin
                  m_axis_rq_tlast             <= 1'b1;
                  tlp_is_write                <= 1'b0;
               end else begin
                  tlp_is_write                <= 1'b1;
               end
               select_FLB_ready_r             <= 1'b0;
            end else if(m_axis_rq_tvalid && m_axis_rq_tready[0]) begin
               if(tlp_is_write == 1'b1 && PCIE_BUS_WIDTH == 128) begin
                  m_axis_rq_tvalid               <= 1'b1;
                  m_axis_rq_tlast                <= 1'b1;
                  tlp_is_write                   <= 1'b0;
                  tlp_is_write_r                 <= 1'b1;
                  select_FLB_ready_r             <= 1'b0;
               end else begin
                  m_axis_rq_tvalid               <= 1'b0;
                  m_axis_rq_tlast                <= 1'b0;
                  tlp_is_write_r                 <= 1'b0;
                  select_FLB_ready_r             <= 1'b1;
               end

            end
         end
      end
   end
   assign completer_id        = /*(select_FLB_Type_i[3:2] == 2'b10) ?*/ {8'h01,5'b00000,3'b0} /* : {8'h0,5'b0,3'b0}*/;
   assign select_FLB_ready_i  = select_FLB_ready_r & wr_valid & (pcie_tfc_nph_av !=4'b0);
   assign wr_ack              = select_FLB_request_i & (select_FLB_Type_i != 3'b1) & select_FLB_ready_i;
   assign wr_type             = select_FLB_wren_i;
   assign wr_data             = select_FLB_Data_i;
   assign wr_baseaddr         = (select_FLB_Type_i[3:1]==3'b0)? {20'h2, select_FLB_addr_i[11:0]} : {20'h1, select_FLB_addr_i[11:0]};

   generate
   if(PCIE_BUS_WIDTH == 128)
   begin // PCIE_DATA_WIDTH_128_ADDRESS
      assign m_axis_rq_tdata = (tlp_is_write_r) ? tlp_data_r : tlp_r;
      assign m_axis_rq_tkeep = (tlp_is_write_r) ? tlp_data_keep_r:tlp_keep_r;
   end else begin// PCIE_DATA_WIDTH_256_ADDRESS
      assign m_axis_rq_tdata = {tlp_data_r,tlp_r};
      assign m_axis_rq_tkeep = {tlp_data_keep_r,tlp_keep_r};
   end
   endgenerate

   //-------------------------------------------------------------
   // Internally bus synchronization
   //-------------------------------------------------------------
   bus_valid_ready_synchronizer #(
      .DATA_WIDTH ( 4 + 32 + 32/8 + 64 + 1           )
   ) DATA_SYNC_INST (
      .clk        (clk                                                                                                   ),
      .rstn       (rstn                                                                                                  ),
      .srst       (srst                                                                                                  ),
      .validin    (select_FLB_request                                                                                    ),
      .datain     ({ select_FLB_Type,select_FLB_Data,select_FLB_Data_en,select_FLB_addr,select_FLB_wren}                  ),
      .readyout   (select_FLB_ready                                                                                      ),
      .validout   (select_FLB_request_i                                                                                  ),
      .dataout    ({select_FLB_Type_i,select_FLB_Data_i,select_FLB_Data_en_i,select_FLB_addr_i,select_FLB_wren_i}        ),
      .readyin    (select_FLB_ready_i                                                                                    )
   );
endmodule
