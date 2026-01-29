
//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------

module requester_completion_rp #(
   parameter                                       PCIE_BUS_WIDTH             =          256
)(
   input    wire                                         clk                      ,
   input    wire                                         rstn                     ,
   input    wire                                         srst                     ,

   // Signal coming from tag queue
   output   reg   [7                      :0]            select_tag               ,
   output   reg                                          select_tag_valid         ,
   output   reg                                          select_replay_cfg        ,

   // AXI PCIE Interconnection
   input    wire  [PCIE_BUS_WIDTH-1       :0]            s_axis_rc_tdata          ,
   input    wire  [74                     :0]            s_axis_rc_tuser          ,
   input    wire                                         s_axis_rc_tlast          ,
   input    wire  [(PCIE_BUS_WIDTH/32)-1  :0]            s_axis_rc_tkeep          ,
   input    wire                                         s_axis_rc_tvalid         ,
   output   wire                                         s_axis_rc_tready         ,

   //  Flow Bus backend for Submission interface
   // Master interface
   output   wire  [32-1                   :0]            init_ram_rdata           ,
   output   wire  [32/8-1                 :0]            init_ram_rbe
);

   //-------------------------------------------------------------
   // FSM to detect type tlp or data
   //-------------------------------------------------------------
   localparam [1:0]      ST_TLP_HEADER1            = 0,
                         ST_DASH_REQUEST           = 1,
                         ST_END_REQUEST            = 2;
   reg    [1   :0]       tlp_sm                       ;

   wire   [10                     :0]         cpl_length              ;
   wire   [7                      :0]         cpl_tag                 ;
   wire   [3                      :0]         cpl_error_code          ;

   wire   [2                      :0]         byte_valid_low          ;
   wire   [2                      :0]         byte_valid_high         ;
   wire   [4                      :0]         byte_valid_i            ;

   wire   [PCIE_BUS_WIDTH-1       :0]         s_axis_rc_tdata_i       ;
   wire   [74                     :0]         s_axis_rc_tuser_i       ;
   wire                                       s_axis_rc_tlast_i       ;
   wire   [(PCIE_BUS_WIDTH/32)-1  :0]         s_axis_rc_tkeep_i       ;
   wire                                       s_axis_rc_tvalid_i      ;
   wire                                       s_axis_rc_tready_i      ;
   reg    [4                      :0]         counter_i               ;                            // INTERNAL SIGNAL
   reg                                        init_ram_valid          ;                            // INTERNAL SIGNAL
   reg    [PCIE_BUS_WIDTH - 1     :0]         init_ram_data           ;

   //-------------------------------------------------------------
   // completion descriptor
   //-------------------------------------------------------------
   assign cpl_error_code           =    s_axis_rc_tdata_i[15:12];
   assign cpl_request_completed    =    s_axis_rc_tdata_i[30];
   assign cpl_length               =    s_axis_rc_tdata_i[42:32];

   assign cpl_tag                  =    s_axis_rc_tdata_i[71:64];

   generate
   if(PCIE_BUS_WIDTH == 128)
   begin // PCIE_DATA_WIDTH_128_ADDRESS

      InVectorBitsCounter_04 InVectorBitsCounter_04_low_inst(
         .InBit         ({s_axis_rc_tkeep_i[3],s_axis_rc_tkeep_i[2],s_axis_rc_tkeep_i[1],s_axis_rc_tkeep_i[0]}),
         .NumberOfOnes  (byte_valid_low)
      );
      assign byte_valid_high       = 4'b0;
      assign byte_valid_i          = (tlp_sm == ST_TLP_HEADER1) ? s_axis_rc_tkeep_i[3] :
                                                                  byte_valid_low;
   end else begin  // PCIE_DATA_WIDTH_256_ADDRESS

      InVectorBitsCounter_04 InVectorBitsCounter_04_low_inst(
         .InBit         ({s_axis_rc_tkeep_i[3],s_axis_rc_tkeep_i[2],s_axis_rc_tkeep_i[1],s_axis_rc_tkeep_i[0]}),
         .NumberOfOnes  (byte_valid_low)
      );

      InVectorBitsCounter_04 InVectorBitsCounter_04_high_inst(
         .InBit         ({s_axis_rc_tkeep_i[7],s_axis_rc_tkeep_i[6],s_axis_rc_tkeep_i[5],s_axis_rc_tkeep_i[4]}),
         .NumberOfOnes  (byte_valid_high)
      );

      assign byte_valid_i          = (tlp_sm == ST_TLP_HEADER1) ? byte_valid_high+s_axis_rc_tkeep_i[3] :
                                                                  byte_valid_high+byte_valid_low;
   end
   endgenerate

   //------------------------------------------------------------
   // FSM to check received TLP
   //------------------------------------------------------------
   always @(posedge clk or negedge rstn)
   if (rstn == 1'b0) begin
      tlp_sm <= ST_TLP_HEADER1;
   end else begin
      if(srst == 1'b1) begin
         tlp_sm <= ST_TLP_HEADER1;
      end else begin
         case (tlp_sm)
            ST_TLP_HEADER1 : begin
               if(s_axis_rc_tvalid_i == 1'b1 && s_axis_rc_tlast_i != 1'b1 && s_axis_rc_tready_i == 1'b1) begin
                  tlp_sm <= ST_END_REQUEST;
               end
            end

            ST_END_REQUEST :
               if(s_axis_rc_tlast_i == 1'b1 && s_axis_rc_tready_i==1'b1)
                  tlp_sm <= ST_TLP_HEADER1;

            default: tlp_sm <= ST_TLP_HEADER1;
         endcase
      end
   end

   //-------------------------------------------------------------
   // Information Request register
   //-------------------------------------------------------------
   always @(posedge clk or negedge rstn)
   if (rstn == 1'b0) begin
      select_tag                 <= {8{1'b0}};
      select_tag_valid           <= 1'b0;
      select_replay_cfg          <= 1'b0;
      counter_i                  <= 4'b0;
      init_ram_valid             <= 1'b0;
      init_ram_data              <= {PCIE_BUS_WIDTH{1'b0}};
   end else begin
      if(srst == 1'b1)
      begin
         select_tag                 <= {8{1'b0}};
         select_tag_valid           <= 1'b0;
         select_replay_cfg          <= 1'b0;
         counter_i                  <= 4'b0;
         init_ram_valid             <= 1'b0;
         init_ram_data              <= {PCIE_BUS_WIDTH{1'b0}};
      end else begin
         if((tlp_sm == ST_TLP_HEADER1) && s_axis_rc_tvalid_i == 1'b1 && s_axis_rc_tready_i == 1'b1)
           select_tag_valid     <= 1'b1;
         else
           select_tag_valid     <= 1'b0;
           
         if((tlp_sm == ST_TLP_HEADER1) && s_axis_rc_tvalid_i == 1'b1 && s_axis_rc_tready_i == 1'b1 && cpl_error_code!=4'b0)
           select_replay_cfg     <= 1'b1;
         else
           select_replay_cfg     <= 1'b0;  

         if(tlp_sm == ST_TLP_HEADER1) begin
           select_tag            <=  cpl_tag;
         end

        if(s_axis_rc_tvalid_i == 1'b1 && s_axis_rc_tready_i == 1'b1) begin
            if(tlp_sm == ST_TLP_HEADER1)
               init_ram_data              <= {96'b0,s_axis_rc_tdata_i[PCIE_BUS_WIDTH-1:96]};
            else
               init_ram_data              <= s_axis_rc_tdata_i;
        end else if(init_ram_valid == 1'b1)
            init_ram_data <= {32'b0,init_ram_data[PCIE_BUS_WIDTH-1:32]};

         if(s_axis_rc_tvalid_i == 1'b1 && s_axis_rc_tready_i == 1'b1 && byte_valid_i != 4'b0)
            init_ram_valid             <= 1'b1;
         else if(counter_i == 4'h1)
            init_ram_valid             <= 1'b0;

         if(s_axis_rc_tvalid_i == 1'b1 && s_axis_rc_tready_i == 1'b1)
            counter_i                  <= byte_valid_i;
         else if(init_ram_valid == 1'b1)
            counter_i                  <= counter_i - 1'b1;
      end
   end


   assign s_axis_rc_tready_i = ~init_ram_valid | (counter_i == 4'h1);

   assign init_ram_rbe = {32/8{init_ram_valid}};
   assign init_ram_rdata = init_ram_data[31:0];

   //-------------------------------------------------------------
   // AXIS bus synchronization
   //-------------------------------------------------------------
   bus_valid_ready_synchronizer #(
      .DATA_WIDTH (PCIE_BUS_WIDTH + (PCIE_BUS_WIDTH/32) + 75 + 1                                 )
   ) AXIS_TX_BUS_SYNC_INST (
      .clk        (clk                                                                           ),
      .rstn       (rstn                                                                          ),
      .srst       (srst                                                                          ),
      .validin    (s_axis_rc_tvalid                                                              ),
      .datain     ({s_axis_rc_tlast,s_axis_rc_tuser,s_axis_rc_tdata,s_axis_rc_tkeep}             ),
      .readyout   (s_axis_rc_tready                                                              ),
      .validout   (s_axis_rc_tvalid_i                                                            ),
      .dataout    ({s_axis_rc_tlast_i,s_axis_rc_tuser_i,s_axis_rc_tdata_i,s_axis_rc_tkeep_i}     ),
      .readyin    (s_axis_rc_tready_i                                                            )
   );

endmodule
