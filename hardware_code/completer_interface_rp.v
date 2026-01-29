
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

module completer_interface_rp #(
   parameter                                       PCIE_BUS_WIDTH             =          256,
   parameter   [63:0]                              DDR_BASE_OFFSET            = 64'hFFFFFFFF,
   parameter   [31:0]                              SUBMISSION_BUFFER_OFFSET   = 32'hFFFFFFFF,
   parameter   [31:0]                              COMPLETION_BUFFER_OFFSET   = 32'hFFFFFFFF,
   parameter   [31:0]                              DATAPRP_BUFFER_OFFSET      = 32'hFFFFFFFF,
   parameter   [31:0]                              SUBMISSION_BUFFER_SIZE     = 32'hFFFFFFFF,
   parameter   [31:0]                              COMPLETION_BUFFER_SIZE     = 32'hFFFFFFFF,
   parameter   [31:0]                              DATAPRP_BUFFER_SIZE        = 32'hFFFFFFFF
)(
   input    wire                                         clk                      ,
   input    wire                                         rstn                     ,
   input    wire                                         srst                     ,

   input    wire  [2                      :0]            device_mps               ,
   // AXI PCIE Interconnection
   input    wire  [PCIE_BUS_WIDTH-1       :0]            s_axis_cq_tdata          ,
   input    wire  [84                     :0]            s_axis_cq_tuser          ,
   input    wire                                         s_axis_cq_tlast          ,
   input    wire  [(PCIE_BUS_WIDTH/32)-1  :0]            s_axis_cq_tkeep          ,
   input    wire                                         s_axis_cq_tvalid         ,
   output   wire                                         s_axis_cq_tready         ,

   output   wire  [PCIE_BUS_WIDTH-1       :0]            m_axis_cc_tdata          ,
   output   wire  [32                     :0]            m_axis_cc_tuser          ,
   output   wire                                         m_axis_cc_tlast          ,
   output   wire  [(PCIE_BUS_WIDTH/32)-1  :0]            m_axis_cc_tkeep          ,
   output   wire                                         m_axis_cc_tvalid         ,
   input    wire  [3                      :0]            m_axis_cc_tready         ,

   output   wire                                         rq_ddr_rd_FLB_Sop        ,
   output   wire                                         rq_ddr_rd_FLB_Eop        ,
   output   wire  [`FLB_RP_SIZE -1        :0]            rq_ddr_rd_FLB_Type       ,
   output   wire  [PCIE_BUS_WIDTH-1       :0]            rq_ddr_rd_FLB_Data       ,
   output   wire  [PCIE_BUS_WIDTH/8-1     :0]            rq_ddr_rd_FLB_Data_en    ,
   output   wire                                         rq_ddr_rd_FLB_valid      ,
   output   wire  [64-1                   :0]            rq_ddr_rd_FLB_addr       ,
   input    wire                                         rq_ddr_rd_FLB_rdy        ,

   output   wire                                         rq_ddr_wr_FLB_Sop        ,
   output   wire                                         rq_ddr_wr_FLB_Eop        ,
   output   wire  [6 -1                   :0]            rq_ddr_wr_FLB_byte       ,
   output   wire  [`FLB_RP_SIZE -1        :0]            rq_ddr_wr_FLB_Type       ,
   output   wire  [PCIE_BUS_WIDTH-1       :0]            rq_ddr_wr_FLB_Data       ,
   output   wire  [PCIE_BUS_WIDTH/8-1     :0]            rq_ddr_wr_FLB_Data_en    ,
   output   wire                                         rq_ddr_wr_FLB_valid      ,
   output   wire  [64-1                   :0]            rq_ddr_wr_FLB_addr       ,
   input    wire                                         rq_ddr_wr_FLB_rdy        ,

//    `ifndef FIFO_INTERFACE
   input    wire                                         data_in_unaligned        ,
//    `endif

   input    wire                                         data_ddr_cpl_FLB_Sop     ,
   input    wire                                         data_ddr_cpl_FLB_Eop     ,
   input    wire  [`FLB_RP_SIZE -1        :0]            data_ddr_cpl_FLB_Type    ,
   input    wire  [PCIE_BUS_WIDTH-1       :0]            data_ddr_cpl_FLB_Data    ,
   input    wire  [PCIE_BUS_WIDTH/8-1     :0]            data_ddr_cpl_FLB_Data_en ,
   input    wire                                         data_ddr_cpl_FLB_valid   ,
   input    wire  [64-1                   :0]            data_ddr_cpl_FLB_addr    ,
   output   wire                                         data_ddr_cpl_FLB_rdy     ,

   // Interface with completion buffer
   output   wire                                         pcie_cpl_wren            ,
   input    wire                                         pcie_cpl_ack             ,
   output   wire  [127                    :0]            pcie_cpl_wrdata          ,

   // Interface with nvme submission buffer
   output   wire                                         subbuf_rden              ,
   output   wire  [63                     :0]            subbuf_addr              ,
   input    wire  [255                    :0]            subbuf_rddata            ,

`ifdef MODE_OPAL
   output   wire                                           opal_mem_ce                     ,
   output   wire  [9              :0]                      opal_mem_addr                   ,
   input    wire  [64-1           :0]                      opal_mem_data                   ,
    input    wire                                           switch_to_opal                  ,
`endif

   output   wire                                         dataprplistbuf_rden      ,
   output   wire  [8                      :0]            dataprplistbuf_addr      ,
   input    wire  [63                     :0]            dataprplistbuf_rddata
);


   //////////////////////////////
   // Internal signal in flb2axis
   //////////////////////////////
   wire                                         cpl_request                ;
   wire  [1                      :0]            cpl_address_type           ;
   wire  [2                      :0]            cpl_status                 ;
   wire                                         cpl_ack                    ;
   wire                                         start_completion           ;
   wire                                         fifo_cc_almostfull         ;

   wire  [7                      :0]            cpl_tag                    ;
   wire  [15                     :0]            cpl_requester_id           ;
   wire  [10                     :0]            cpl_length                 ;
   wire  [6                      :0]            cpl_lower_address          ;
   wire                                         subbuf_rdvalid             ;
   reg                                          subbuf_rdvalid_r           ;
   reg                                          subbuf_rdvalid_rr          ;
   wire                                         dataprplistbuf_rd_valid    ;
   reg                                          dataprplistbuf_rd_valid_r  ;
   reg                                          dataprplistbuf_rd_valid_rr ;

`ifdef MODE_OPAL
   wire                                         opalbuf_rd_valid           ;
   reg                                          opalbuf_rd_valid_r         ;
   reg                                          opalbuf_rd_valid_rr        ;

   reg   [63                     :0]            opalbuf_rddata_r           ;

`endif
//    `ifdef FIFO_INTERFACE
//    reg   [7                      :0]            dataprplistbuf_dma_r,dataprplistbuf_dma_rr;
//    wire  [7                      :0]            dataprplistbuf_dma         ;
//    wire  [8                      :0]            dataprplistbuf_block       ;
//    reg   [8                      :0]            dataprplistbuf_block_r,dataprplistbuf_block_rr;
//    `endif
   reg   [63                     :0]            dataprplistbuf_rddata_r    ;
   reg   [255                    :0]            subbuf_rddata_r            ;


   completer_request_rp  #(
      .PCIE_BUS_WIDTH             (PCIE_BUS_WIDTH            ),
      .DDR_BASE_OFFSET            (DDR_BASE_OFFSET           ),
      .SUBMISSION_BUFFER_OFFSET   (SUBMISSION_BUFFER_OFFSET  ),
      .COMPLETION_BUFFER_OFFSET   (COMPLETION_BUFFER_OFFSET  ),
      .DATAPRP_BUFFER_OFFSET      (DATAPRP_BUFFER_OFFSET     ),
      .SUBMISSION_BUFFER_SIZE     (SUBMISSION_BUFFER_SIZE    ),
      .COMPLETION_BUFFER_SIZE     (COMPLETION_BUFFER_SIZE    ),
      .DATAPRP_BUFFER_SIZE        (DATAPRP_BUFFER_SIZE       )
   ) completer_request_rp_inst (
      .clk                    (  clk                     ),
      .rstn                   (  rstn                    ),
      .srst                   (  srst                    ),

      .s_axis_cq_tdata        (  s_axis_cq_tdata         ),
      .s_axis_cq_tuser        (  s_axis_cq_tuser         ),
      .s_axis_cq_tlast        (  s_axis_cq_tlast         ),
      .s_axis_cq_tkeep        (  s_axis_cq_tkeep         ),
      .s_axis_cq_tvalid       (  s_axis_cq_tvalid        ),
      .s_axis_cq_tready       (  s_axis_cq_tready        ),

      .rq_ddr_rd_FLB_Type     (  rq_ddr_rd_FLB_Type      ),    //  cpl_tag  cpl_length cpl_requester_id
      .rq_ddr_rd_FLB_valid    (  rq_ddr_rd_FLB_valid     ),
      .rq_ddr_rd_FLB_addr     (  rq_ddr_rd_FLB_addr      ),    //  lower_address
      .rq_ddr_rd_FLB_rdy      (  rq_ddr_rd_FLB_rdy       ),

      .rq_ddr_wr_FLB_Sop      (  rq_ddr_wr_FLB_Sop       ),
      .rq_ddr_wr_FLB_Eop      (  rq_ddr_wr_FLB_Eop       ),
      .rq_ddr_wr_FLB_Type     (  rq_ddr_wr_FLB_Type      ),
      .rq_ddr_wr_FLB_Data     (  rq_ddr_wr_FLB_Data      ),
      .rq_ddr_wr_FLB_byte     (  rq_ddr_wr_FLB_byte      ),
      .rq_ddr_wr_FLB_Data_en  (  rq_ddr_wr_FLB_Data_en   ),
      .rq_ddr_wr_FLB_valid    (  rq_ddr_wr_FLB_valid     ),
      .rq_ddr_wr_FLB_addr     (  rq_ddr_wr_FLB_addr      ),
      .rq_ddr_wr_FLB_rdy      (  rq_ddr_wr_FLB_rdy       ),

//       `ifndef FIFO_INTERFACE
       .data_in_unaligned     (  data_in_unaligned       ),
//        `endif

      .pcie_cpl_wren          (  pcie_cpl_wren           ),
      .pcie_cpl_ack           (  pcie_cpl_ack            ),
      .pcie_cpl_wrdata        (  pcie_cpl_wrdata         ),

      .subbuf_rden            (  subbuf_rden             ),
      .subbuf_addr            (  subbuf_addr             ),

`ifdef MODE_OPAL
      .opalbuf_rden           (opal_mem_ce              ),
      .opalbuf_addr           (opal_mem_addr              ),
      .opalbuf_rd_valid       (opalbuf_rd_valid          ),
      .switch_to_opal         (switch_to_opal             ),
`endif


      .dataprplistbuf_rd_valid(  dataprplistbuf_rd_valid ),
//       `ifdef FIFO_INTERFACE
//       .dataprplistbuf_dma     (  dataprplistbuf_dma      ),
//       .dataprplistbuf_block   (  dataprplistbuf_block    ),
//       `endif
      .dataprplistbuf_rden    (  dataprplistbuf_rden     ),
      .dataprplistbuf_addr    (  dataprplistbuf_addr     ),

      .cpl_request            (  cpl_request             ),
      .cpl_address_type       (  cpl_address_type        ),
      .cpl_status             (  cpl_status              ),
      .cpl_ack                (  cpl_ack                 ),
      .start_completion       (  start_completion        ),
      .fifo_cc_almostfull     (  fifo_cc_almostfull      ),
      .cpl_tag                (  cpl_tag                 ),
      .cpl_requester_id       (  cpl_requester_id        ),
      .cpl_length             (  cpl_length              ),
      .cpl_lower_address      (  cpl_lower_address       )
   );

   always @(posedge clk) begin
      subbuf_rdvalid_r           <= subbuf_rden;
      dataprplistbuf_rd_valid_r  <= dataprplistbuf_rd_valid;
      subbuf_rdvalid_rr          <= subbuf_rdvalid_r;
      dataprplistbuf_rd_valid_rr <= dataprplistbuf_rd_valid_r;

      subbuf_rddata_r            <= subbuf_rddata;

//       `ifdef FIFO_INTERFACE
//       dataprplistbuf_dma_r <= dataprplistbuf_dma ;
//       dataprplistbuf_dma_rr<= dataprplistbuf_dma_r;
//       dataprplistbuf_rddata_r    <= {dataprplistbuf_rddata[63:29],dataprplistbuf_dma_r,9'b0,dataprplistbuf_rddata[11:0]};
//
//       dataprplistbuf_block_r   <= dataprplistbuf_block;
//       dataprplistbuf_block_rr  <= dataprplistbuf_block_r;
//       `else
      dataprplistbuf_rddata_r    <= dataprplistbuf_rddata;
//       `endif
`ifdef MODE_OPAL
     opalbuf_rd_valid_r       <= opalbuf_rd_valid;
     opalbuf_rd_valid_rr      <= opalbuf_rd_valid_r;
      opalbuf_rddata_r         <= opal_mem_data;
`endif
   end

   completer_completion_rp #(
      .PCIE_BUS_WIDTH             (PCIE_BUS_WIDTH            )
   ) completer_completion_rp_inst (
      .clk                        (  clk                         ),
      .rstn                       (  rstn                        ),
      .srst                       (  srst                        ),

      .device_mps                 (  device_mps                  ),

      .subbuf_rdvalid             (  subbuf_rdvalid_rr           ),
      .subbuf_rddata              (  subbuf_rddata_r             ),

`ifdef MODE_OPAL
      .opalbuf_rdvalid            (opalbuf_rd_valid_rr           ),
      .opalbuf_rddata             (opalbuf_rddata_r              ),
//       .switch_to_opal             (switch_to_opal                ),
`endif


      .dataprplistbuf_rdvalid     (  dataprplistbuf_rd_valid_rr  ),

//       `ifdef FIFO_INTERFACE
//       .dataprplistbuf_rddata      (  {dataprplistbuf_rddata_r[63:21],dataprplistbuf_block_rr,dataprplistbuf_rddata_r[11:0]}),
//       `else
      .dataprplistbuf_rddata      (  dataprplistbuf_rddata_r     ),
//       `endif
      .start_completion           (  start_completion            ),
      .fifo_cc_almostfull         (  fifo_cc_almostfull          ),
      .cpl_request                (  cpl_request                 ),
      .cpl_address_type           (  cpl_address_type            ),
      .cpl_status                 (  cpl_status                  ),
      .cpl_ack                    (  cpl_ack                     ),
      .cpl_tag                    (  cpl_tag                     ),
      .cpl_requester_id           (  cpl_requester_id            ),
      .cpl_length                 (  cpl_length                  ),
      .cpl_lower_address          (  cpl_lower_address           ),

      .data_ddr_cpl_FLB_Sop       (  data_ddr_cpl_FLB_Sop        ),
      .data_ddr_cpl_FLB_Eop       (  data_ddr_cpl_FLB_Eop        ),
      .data_ddr_cpl_FLB_Type      (  data_ddr_cpl_FLB_Type       ),
      .data_ddr_cpl_FLB_Data      (  data_ddr_cpl_FLB_Data       ),
      .data_ddr_cpl_FLB_Data_en   (  data_ddr_cpl_FLB_Data_en    ),
      .data_ddr_cpl_FLB_valid     (  data_ddr_cpl_FLB_valid      ),
      .data_ddr_cpl_FLB_addr      (  data_ddr_cpl_FLB_addr       ),
      .data_ddr_cpl_FLB_rdy       (  data_ddr_cpl_FLB_rdy        ),
      .m_axis_cc_tdata            (  m_axis_cc_tdata             ),
      .m_axis_cc_tuser            (  m_axis_cc_tuser             ),
      .m_axis_cc_tlast            (  m_axis_cc_tlast             ),
      .m_axis_cc_tkeep            (  m_axis_cc_tkeep             ),
      .m_axis_cc_tvalid           (  m_axis_cc_tvalid            ),
      .m_axis_cc_tready           (  m_axis_cc_tready            )
   );

   assign rq_ddr_rd_FLB_Sop      = 1'b1;
   assign rq_ddr_rd_FLB_Eop      = 1'b1;
   assign rq_ddr_rd_FLB_Data     = {PCIE_BUS_WIDTH{1'b0}};
   assign rq_ddr_rd_FLB_Data_en  = 32'b0;
endmodule