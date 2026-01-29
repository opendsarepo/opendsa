//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe Recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------
`include "AXI_flb_h.v"

module completer_request_rp #(
   parameter                                       PCIE_BUS_WIDTH             =          256,
   parameter   [63:0]                              DDR_BASE_OFFSET            = 64'hFFFFFFFF,
   parameter   [31:0]                              SUBMISSION_BUFFER_OFFSET   = 32'hFFFFFFFF,
   parameter   [31:0]                              COMPLETION_BUFFER_OFFSET   = 32'hFFFFFFFF,
   parameter   [31:0]                              DATAPRP_BUFFER_OFFSET      = 32'hFFFFFFFF,
   parameter   [31:0]                              SUBMISSION_BUFFER_SIZE     = 32'hFFFFFFFF,
   parameter   [31:0]                              COMPLETION_BUFFER_SIZE     = 32'hFFFFFFFF,
   parameter   [31:0]                              DATAPRP_BUFFER_SIZE        = 32'hFFFFFFFF
) (
   input    wire                                         clk                      ,
   input    wire                                         rstn                     ,
   input    wire                                         srst                     ,

   // COMPLETION interface to completer_completion
   output   wire                                         cpl_request              ,
   output   wire  [1                      :0]            cpl_address_type         ,
   output   wire  [7                      :0]            cpl_tag                  ,
   output   wire  [15                     :0]            cpl_requester_id         ,
   output   wire  [2                      :0]            cpl_status               ,
   output   wire  [10                     :0]            cpl_length               ,
   output   wire  [6                      :0]            cpl_lower_address        ,
   input    wire                                         cpl_ack                  ,
   input    wire                                         fifo_cc_almostfull       ,
   input    wire                                         start_completion         ,

   // AXI PCIE Interconnection
   input    wire  [PCIE_BUS_WIDTH-1       :0]            s_axis_cq_tdata          ,
   input    wire  [84                     :0]            s_axis_cq_tuser          ,
   input    wire                                         s_axis_cq_tlast          ,
   input    wire  [(PCIE_BUS_WIDTH/32)-1  :0]            s_axis_cq_tkeep          ,
   input    wire                                         s_axis_cq_tvalid         ,
   output   wire                                         s_axis_cq_tready         ,

   output   wire  [`FLB_RP_SIZE -1        :0]            rq_ddr_rd_FLB_Type       ,
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

   // Interface with completion buffer
   output   wire                                         pcie_cpl_wren            ,
   input    wire                                         pcie_cpl_ack             ,
   output   wire  [127                    :0]            pcie_cpl_wrdata          ,

   // Interface with nvme submission buffer
   output   wire                                         subbuf_rden              ,
   output   wire  [63                     :0]            subbuf_addr              ,

`ifdef MODE_OPAL
   output   wire                                         opalbuf_rd_valid         ,
   output   wire                                         opalbuf_rden             ,
   output   wire  [9                      :0]            opalbuf_addr             ,
   input    wire                                         switch_to_opal           ,
`endif


   output   wire                                         dataprplistbuf_rd_valid  ,
   output   wire                                         dataprplistbuf_rden      ,
//    `ifdef FIFO_INTERFACE
//    output   wire  [7                      :0]            dataprplistbuf_dma       ,
//    output   wire  [8                      :0]            dataprplistbuf_block     ,
//    `endif
   output   wire  [8                      :0]            dataprplistbuf_addr
);
   localparam FIRST_BIT_DWORD_COUNT     = 64;
   localparam FIRST_BIT_REQUEST_TYPE    = 75;
   localparam FIRST_BIT_REQUESTERID     = 80;
   localparam FIRST_BIT_TAG             = 96;
   localparam FIRST_BIT_TARGET_FUNCTION = 104;
   localparam FIRST_BIT_BAR_ID          = 112;
   localparam FIRST_BIT_BAR_APERTURE    = 115;
   localparam DWORD_MAX                 =   4;

   //////////////////////////////
   // Internal signal in flb2axis
   //////////////////////////////
   wire   [PCIE_BUS_WIDTH-1       :0]                   s_axis_cq_tdata_i        ;
   wire                                                 s_axis_cq_tuser_i        ;
   wire                                                 s_axis_cq_tlast_i        ;
   wire   [(PCIE_BUS_WIDTH/32)-1  :0]                   s_axis_cq_tkeep_i        ;
   wire                                                 s_axis_cq_tvalid_i       ;
   wire                                                 s_axis_cq_tready_single  ;
   wire                                                 s_axis_cq_tready_read    ;

   wire   [15                     :0]                   requesterid              ;
   wire   [7                      :0]                   tag                      ;
   wire   [63                     :0]                   read_address             ;
   wire   [10                     :0]                   dword_read               ;
   wire   [5                      :0]                   bar_aperture_rd          ;

   wire   [15                     :0]                   requesterid_i            ;
   wire   [7                      :0]                   tag_i                    ;
   wire   [63                     :0]                   read_address_i           ;
   wire   [10                     :0]                   dword_read_i             ;
   wire   [5                      :0]                   bar_aperture_rd_i        ;

   wire   [3                      :0]                   request_type             ;

//    wire   [2                      :0]                   bar_id                   ;
   wire   [5                      :0]                   bar_aperture             ;
//    wire   [7                      :0]                   target_function          ;

   wire   [PCIE_BUS_WIDTH-1       :0]                   window_write_wdata       ;
   wire   [PCIE_BUS_WIDTH/8 -1    :0]                   window_write_wbe         ;
   wire   [10                     :0]                   window_write_length      ;
   wire                                                 window_write_req         ;
   wire   [64-1                   :0]                   window_write_addr        ;
   wire                                                 window_write_rdy         ;
   wire                                                 window_write_eof         ;
   wire                                                 window_write_sof         ;

   wire   [3                      :0]                   byte_valid               ;
   reg                                                  shift_valid              ;
   reg                                                  shift_filter             ;
   reg    [63                     :0]                   shift_mask_address       ;
   reg                                                  shift_sof                ;
   reg                                                  shift_eof                ;
   reg    [3                      :0]                   shift_byte               ;
   reg    [PCIE_BUS_WIDTH/32-1    :0]                   shift_be                 ;
   wire   [PCIE_BUS_WIDTH/8-1     :0]                   shift_be_i               ;
   reg    [PCIE_BUS_WIDTH-1       :0]                   shift_data               ;
   wire                                                 shift_ready              ;

   wire                                                 window_read_req_i        ;
   reg                                                  window_read_req          ;
   reg    [10                     :0]                   window_read_length       ;
   wire                                                 window_read_rdy          ;
   wire   [25                     :0]                   window_read_type         ;
   reg    [64-1                   :0]                   window_read_addr         ;
   wire                                                 isreadtlp                ;
   reg    [64-1                   :0]                   read_mask_address        ;
   reg    [15                     :0]                   requesterid_r            ;
   reg    [7                      :0]                   tag_r                    ;
   wire                                                 dataprplistbuf_rd_valid_i  ;
   wire                                                 dataprplistbuf_rden_i      ;
   wire  [8                      :0]                    dataprplistbuf_addr_i      ;

   assign isreadtlp          = s_axis_cq_tvalid & s_axis_cq_tuser[40] & (s_axis_cq_tdata[78:75] == 4'b0);
   assign s_axis_cq_tready   = (isreadtlp) ? s_axis_cq_tready_read: s_axis_cq_tready_single;

   //--------------------------------------------------------------
   // AXIS bus synchronization
   //-------------------------------------------------------------~
   bus_valid_ready_synchronizer #(
      .DATA_WIDTH (16+8+64+6+11                                                               )
   ) AXIS_READ_BUS_SYNC_INST (
      .clk        (clk                                                                        ),
      .rstn       (rstn                                                                       ),
      .srst       (srst                                                                       ),
      .validin    (s_axis_cq_tvalid & isreadtlp                                               ),
      .datain     ({bar_aperture_rd,read_address,dword_read,tag,requesterid}                  ),
      .readyout   (s_axis_cq_tready_read                                                      ),
      .validout   (window_read_req_i                                                          ),
      .dataout    ({bar_aperture_rd_i,read_address_i,dword_read_i,tag_i,requesterid_i}        ),
      .readyin    (window_read_rdy                                                            )
   );

   bus_valid_ready_synchronizer #(
      .DATA_WIDTH (PCIE_BUS_WIDTH + (PCIE_BUS_WIDTH/32) + 1 + 1                               )
   ) AXIS_WRITE_BUS_SYNC_INST (
      .clk        (clk                                                                        ),
      .rstn       (rstn                                                                       ),
      .srst       (srst                                                                       ),
      .validin    (s_axis_cq_tvalid & ~isreadtlp                                              ),
      .datain     ({s_axis_cq_tlast,s_axis_cq_tuser[40],s_axis_cq_tdata,s_axis_cq_tkeep}      ),
      .readyout   (s_axis_cq_tready_single                                                    ),
      .validout   (s_axis_cq_tvalid_i                                                         ),
      .dataout    ({s_axis_cq_tlast_i,s_axis_cq_tuser_i,s_axis_cq_tdata_i,s_axis_cq_tkeep_i}  ),
      .readyin    (shift_ready                                                                )
   );

   //-------------------------------------------------------------
   // TLP descriptor
   //-------------------------------------------------------------
   assign requesterid      =  s_axis_cq_tdata[FIRST_BIT_REQUESTERID     +:16];
   assign tag              =  s_axis_cq_tdata[FIRST_BIT_TAG             +:8];
   assign read_address     =  s_axis_cq_tdata[63:0];
   assign dword_read       =  s_axis_cq_tdata[FIRST_BIT_DWORD_COUNT     +:10];
   assign bar_aperture_rd  =  s_axis_cq_tdata[FIRST_BIT_BAR_APERTURE    +:6];

   assign request_type     =  s_axis_cq_tdata_i[FIRST_BIT_REQUEST_TYPE    +:4];
   assign bar_aperture     =  s_axis_cq_tdata_i[FIRST_BIT_BAR_APERTURE    +:6];
   assign window_read_type = {window_read_addr[1:0],tag_r,requesterid_r};

   // Keep for the furture use
//    assign target_function =  s_axis_cq_tdata_i[FIRST_BIT_TARGET_FUNCTION +:8];
//    assign bar_id          =  s_axis_cq_tdata_i[FIRST_BIT_BAR_ID          +:3];

   generate
   if(PCIE_BUS_WIDTH == 128)
   begin // PCIE_DATA_WIDTH_128_ADDRESS

   InVectorBitsCounter_04 InVectorBitsCounter_4_inst(
      .InBit         ({s_axis_cq_tkeep_i[3],s_axis_cq_tkeep_i[2],s_axis_cq_tkeep_i[1],s_axis_cq_tkeep_i[0]}),
      .NumberOfOnes  (byte_valid[2:0])
   );
   assign byte_valid[3] = 1'b0;
   end else begin  // PCIE_DATA_WIDTH_256_ADDRESS
   InVectorBitsCounter_08 InVectorBitsCounter_8_hi_inst(
      .InBit         ({s_axis_cq_tkeep_i[7],s_axis_cq_tkeep_i[6],s_axis_cq_tkeep_i[5],s_axis_cq_tkeep_i[4],s_axis_cq_tkeep_i[3],s_axis_cq_tkeep_i[2],s_axis_cq_tkeep_i[1],s_axis_cq_tkeep_i[0]}),
      .NumberOfOnes  (byte_valid)
   );
   end
   endgenerate

   always @ (bar_aperture_rd_i) begin
      case(bar_aperture_rd_i)
          6'b000111: read_mask_address <=64'h7F;
          6'b001000: read_mask_address <=64'h00FF;
          6'b001001: read_mask_address <=64'h01FF;
          6'b001010: read_mask_address <=64'h03FF;
          6'b001011: read_mask_address <=64'h07FF;
          6'b001100: read_mask_address <=64'h0FFF;
          6'b001101: read_mask_address <=64'h1FFF;
          6'b001110: read_mask_address <=64'h3FFF;
          6'b001111: read_mask_address <=64'h7FFF;
          6'b010000: read_mask_address <=64'hFFFF;
          6'b010001: read_mask_address <=64'h1FFFF;
          6'b010010: read_mask_address <=64'h3FFFF;
          6'b010011: read_mask_address <=64'h7FFFF;
          6'b010100: read_mask_address <=64'hFFFFF;
          6'b010101: read_mask_address <=64'h1FFFFF;
          6'b010110: read_mask_address <=64'h3FFFFF;
          6'b010111: read_mask_address <=64'h7FFFFF;
          6'b011000: read_mask_address <=64'hFFFFFF;
          6'b011001: read_mask_address <=64'h1FFFFFF;
          6'b011010: read_mask_address <=64'h3FFFFFF;
          6'b011011: read_mask_address <=64'h7FFFFFF;
          6'b011100: read_mask_address <=64'hFFFFFFF;
          6'b011101: read_mask_address <=64'h1FFFFFFF;
          6'b011110: read_mask_address <=64'h3FFFFFFF;
          6'b011111: read_mask_address <=64'h7FFFFFFF;
          6'b100000: read_mask_address <=64'hFFFFFFFF;
          6'b100001: read_mask_address <=64'h1FFFFFFFF;
          6'b100010: read_mask_address <=64'h3FFFFFFFF;
          6'b100011: read_mask_address <=64'h7FFFFFFFF;
          6'b100100: read_mask_address <=64'hFFFFFFFFF;
          6'b100101: read_mask_address <=64'h1FFFFFFFFF;
          6'b100110: read_mask_address <=64'h3FFFFFFFFF;
          default  : read_mask_address <=64'h0;
       endcase
   end

   //-------------------------------------------------------------
   // Information Request & completion registers
   //-------------------------------------------------------------
   always @(posedge clk or negedge rstn)
   begin : REGISTER_SHIFTER
      integer j_v, i_v;
      if (rstn == 1'b0) begin
         window_read_addr        <= 64'b0;
         window_read_req         <= 1'b0;
         window_read_length      <= 10'b0;
         requesterid_r           <= 16'b0;
         tag_r                   <= 8'b0;
      end else begin
         if(srst == 1'b1)
         begin
            window_read_addr        <= 64'b0;
            window_read_req         <= 1'b0;
            window_read_length      <= 10'b0;
            requesterid_r           <= 16'b0;
            tag_r                   <= 8'b0;
         end else begin
            if(window_read_rdy) begin
               window_read_length <= dword_read_i;
               tag_r <= tag_i;
               requesterid_r <= requesterid_i;
               window_read_req <= window_read_req_i;

               for (i_v=0; i_v<64; i_v=i_v+1)
                  window_read_addr[i_v]  <= read_mask_address[i_v] & read_address_i[i_v];
            end
         end
      end
   end

   always @(negedge rstn or posedge clk)
   begin : REGISTER_FIRST
      integer j_v, i_v;
      if (rstn == 1'b0) begin
         shift_valid             <= 1'b0;
         shift_filter            <= 1'b0;
         shift_sof               <= 1'b0;
         shift_eof               <= 1'b0;
         shift_byte              <= 4'b0;
         shift_be                <= {PCIE_BUS_WIDTH/32{1'b0}};
         shift_data              <= {PCIE_BUS_WIDTH{1'b0}};
         shift_mask_address      <= 64'b0;
      end else begin
         if(srst == 1'b1) begin
            shift_valid             <= 1'b0;
            shift_filter            <= 1'b0;
            shift_sof               <= 1'b0;
            shift_eof               <= 1'b0;
            shift_byte              <= 4'b0;
            shift_be                <= {PCIE_BUS_WIDTH/32{1'b0}};
            shift_data              <= {PCIE_BUS_WIDTH{1'b0}};
            shift_mask_address      <= 64'b0;
         end else begin
            if(shift_ready == 1'b1) begin
               shift_valid   <= s_axis_cq_tvalid_i & ( ( ~shift_filter & ~s_axis_cq_tuser_i)  | (request_type == 4'b1 & s_axis_cq_tuser_i)) ;

               if(s_axis_cq_tuser_i == 1'b1 & s_axis_cq_tlast_i == 1'b0 && request_type != 4'b1)
                  shift_filter            <= 1'b1;
               else if(s_axis_cq_tlast_i)
                  shift_filter            <= 1'b0;

               if(s_axis_cq_tuser_i == 1'b1)
                  shift_byte <= byte_valid - 4'b0100;
               else
                  shift_byte <= byte_valid;

               shift_be   <= s_axis_cq_tkeep_i;
               shift_eof  <= s_axis_cq_tlast_i;
               shift_sof  <= s_axis_cq_tuser_i;
               shift_data <= s_axis_cq_tdata_i;

               case(bar_aperture)
                  6'b000111: shift_mask_address <=64'h7F;
                  6'b001000: shift_mask_address <=64'h00FF;
                  6'b001001: shift_mask_address <=64'h01FF;
                  6'b001010: shift_mask_address <=64'h03FF;
                  6'b001011: shift_mask_address <=64'h07FF;
                  6'b001100: shift_mask_address <=64'h0FFF;
                  6'b001101: shift_mask_address <=64'h1FFF;
                  6'b001110: shift_mask_address <=64'h3FFF;
                  6'b001111: shift_mask_address <=64'h7FFF;
                  6'b010000: shift_mask_address <=64'hFFFF;
                  6'b010001: shift_mask_address <=64'h1FFFF;
                  6'b010010: shift_mask_address <=64'h3FFFF;
                  6'b010011: shift_mask_address <=64'h7FFFF;
                  6'b010100: shift_mask_address <=64'hFFFFF;
                  6'b010101: shift_mask_address <=64'h1FFFFF;
                  6'b010110: shift_mask_address <=64'h3FFFFF;
                  6'b010111: shift_mask_address <=64'h7FFFFF;
                  6'b011000: shift_mask_address <=64'hFFFFFF;
                  6'b011001: shift_mask_address <=64'h1FFFFFF;
                  6'b011010: shift_mask_address <=64'h3FFFFFF;
                  6'b011011: shift_mask_address <=64'h7FFFFFF;
                  6'b011100: shift_mask_address <=64'hFFFFFFF;
                  6'b011101: shift_mask_address <=64'h1FFFFFFF;
                  6'b011110: shift_mask_address <=64'h3FFFFFFF;
                  6'b011111: shift_mask_address <=64'h7FFFFFFF;
                  6'b100000: shift_mask_address <=64'hFFFFFFFF;
                  6'b100001: shift_mask_address <=64'h1FFFFFFFF;
                  6'b100010: shift_mask_address <=64'h3FFFFFFFF;
                  6'b100011: shift_mask_address <=64'h7FFFFFFFF;
                  6'b100100: shift_mask_address <=64'hFFFFFFFFF;
                  6'b100101: shift_mask_address <=64'h1FFFFFFFFF;
                  6'b100110: shift_mask_address <=64'h3FFFFFFFFF;
                  default  : shift_mask_address <=64'h0;
               endcase
            end
         end
      end
   end

   //-------------------------------------------------------------
   // Data management
   //-------------------------------------------------------------
   completer_data_extractor #(
   .NVME_DATA_WIDTH   ( PCIE_BUS_WIDTH     ),
   .PCIE_DATA_WIDTH   ( PCIE_BUS_WIDTH     ),
   .RAM_LENGTH_WIDTH  ( 4                  )
   ) completer_data_extractor_inst (
      .clk               (clk                 ),
      .rstn              (rstn                ),
      .srst              (srst                ),
      .in_ready          (shift_ready         ),
      .in_valid          (shift_valid         ),
      .in_mask_address   (shift_mask_address  ),
      .in_be             (shift_be_i          ),
      .in_byte           (shift_byte          ),
      .in_data           (shift_data          ),
      .in_sof            (shift_sof           ),
      .in_eof            (shift_eof           ),
      .out_ready         (window_write_rdy    ),
      .out_valid         (window_write_req    ),
      .out_be            (window_write_wbe    ),
      .out_address       (window_write_addr   ),
      .out_length        (window_write_length ),
      .out_data          (window_write_wdata  ),
      .out_sof           (window_write_sof    ),
      .out_eof           (window_write_eof    )
   );

   generate
   genvar a_v;
   for (a_v=0; a_v<(PCIE_BUS_WIDTH/32); a_v=a_v+1) begin: loop_read_gen
      assign shift_be_i[a_v*4 +:4]  = {4{shift_be[a_v]}};
   end
   endgenerate


`ifdef MODE_OPAL
      assign opalbuf_rden              = switch_to_opal  ? dataprplistbuf_rden_i      : 1'b0;
      assign opalbuf_rd_valid          = switch_to_opal  ? dataprplistbuf_rd_valid_i  : 1'b0;
      assign opalbuf_addr              = switch_to_opal  ? dataprplistbuf_addr_i      : 10'h0;
      assign dataprplistbuf_rd_valid   = switch_to_opal  ? 1'b0  : dataprplistbuf_rd_valid_i ;
      assign dataprplistbuf_rden       = switch_to_opal  ? 1'b0  : dataprplistbuf_rden_i     ;
      assign dataprplistbuf_addr       = switch_to_opal  ? 10'h0 : dataprplistbuf_addr_i     ;


`else
      assign dataprplistbuf_rd_valid   =  dataprplistbuf_rd_valid_i ;
      assign dataprplistbuf_rden       =  dataprplistbuf_rden_i     ;
      assign dataprplistbuf_addr       =  dataprplistbuf_addr_i     ;

`endif




   dispatch_window_request #(
      .PCIE_BUS_WIDTH             ( PCIE_BUS_WIDTH           ),
      .DDR_BASE_OFFSET            ( DDR_BASE_OFFSET          ),
      .SUBMISSION_BUFFER_OFFSET   ( SUBMISSION_BUFFER_OFFSET ),
      .COMPLETION_BUFFER_OFFSET   ( COMPLETION_BUFFER_OFFSET ),
      .DATAPRP_BUFFER_OFFSET      ( DATAPRP_BUFFER_OFFSET    ),
      .SUBMISSION_BUFFER_SIZE     ( SUBMISSION_BUFFER_SIZE   ),
      .COMPLETION_BUFFER_SIZE     ( COMPLETION_BUFFER_SIZE   ),
      .DATAPRP_BUFFER_SIZE        ( DATAPRP_BUFFER_SIZE      )
   ) dispatch_window_request_inst (
      .clk                      (  clk                     ),
      .rstn                     (  rstn                    ),
      .srst                     (  srst                    ),
      .fifo_cc_almostfull       (  fifo_cc_almostfull      ),
`ifdef MODE_OPAL
      .switch_to_opal           (switch_to_opal            ),
`endif

      .window_read_req          (  window_read_req         ),
      .window_read_length       (  window_read_length      ),
      .window_read_type         (  window_read_type        ),
      .window_read_addr         (  window_read_addr        ),
      .window_read_rdy          (  window_read_rdy         ),

      .window_write_wdata       (  window_write_wdata      ),
      .window_write_length      (  window_write_length     ),
      .window_write_wbe         (  window_write_wbe        ),
      .window_write_sof         (  window_write_sof        ),
      .window_write_eof         (  window_write_eof        ),
      .window_write_req         (  window_write_req        ),
      .window_write_addr        (  window_write_addr       ),
      .window_write_rdy         (  window_write_rdy        ),

      .rq_ddr_rd_FLB_Type       (  rq_ddr_rd_FLB_Type      ),
      .rq_ddr_rd_FLB_valid      (  rq_ddr_rd_FLB_valid     ),
      .rq_ddr_rd_FLB_addr       (  rq_ddr_rd_FLB_addr      ),
      .rq_ddr_rd_FLB_rdy        (  rq_ddr_rd_FLB_rdy       ),

      .cpl_request              (  cpl_request             ),
      .cpl_address_type         (  cpl_address_type        ),
      .cpl_tag                  (  cpl_tag                 ),
      .cpl_requester_id         (  cpl_requester_id        ),
      .cpl_status               (  cpl_status              ),
      .cpl_length               (  cpl_length              ),
      .cpl_lower_address        (  cpl_lower_address       ),
      .cpl_ack                  (  cpl_ack                 ),

      .rq_ddr_wr_FLB_Sop        (  rq_ddr_wr_FLB_Sop       ),
      .rq_ddr_wr_FLB_Eop        (  rq_ddr_wr_FLB_Eop       ),
      .rq_ddr_wr_FLB_byte       (  rq_ddr_wr_FLB_byte      ),
      .rq_ddr_wr_FLB_Type       (  rq_ddr_wr_FLB_Type      ),
      .rq_ddr_wr_FLB_Data       (  rq_ddr_wr_FLB_Data      ),
      .rq_ddr_wr_FLB_Data_en    (  rq_ddr_wr_FLB_Data_en   ),
      .rq_ddr_wr_FLB_valid      (  rq_ddr_wr_FLB_valid     ),
      .rq_ddr_wr_FLB_addr       (  rq_ddr_wr_FLB_addr      ),
      .rq_ddr_wr_FLB_rdy        (  rq_ddr_wr_FLB_rdy       ),

//       `ifndef FIFO_INTERFACE
      .data_in_unaligned        (  data_in_unaligned       ),
//       `endif

      .pcie_cpl_wren            (  pcie_cpl_wren           ),
      .pcie_cpl_ack             (  pcie_cpl_ack            ),
      .pcie_cpl_wrdata          (  pcie_cpl_wrdata         ),

      .subbuf_rden              (  subbuf_rden             ),
      .subbuf_addr              (  subbuf_addr             ),

      .dataprplistbuf_rd_valid  (  dataprplistbuf_rd_valid_i ),
      .dataprplistbuf_rden      (  dataprplistbuf_rden_i     ),
//       `ifdef FIFO_INTERFACE
//       .dataprplistbuf_dma       (  dataprplistbuf_dma      ),
//       .dataprplistbuf_block     (  dataprplistbuf_block    ),
//       `endif
      .dataprplistbuf_addr      (  dataprplistbuf_addr_i     )
   );
endmodule