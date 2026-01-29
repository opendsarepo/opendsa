//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP-Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------
`include "AXI_flb_h.v"

module tlpcpl_format #(
   parameter                                       PCIE_BUS_WIDTH             =          256
)(
   input    wire                                         clk                      ,                 // clk
   input    wire                                         rstn                     ,                 // ASynchronous reset
   input    wire                                         srst                     ,                 // Synchronous reset

   input    wire  [10                     :0]            max_payload              ,

   // Flow Bus backend for Submission interface
   input    wire                                         select_FLB_request       ,
   input    wire  [`FLB_RP_SIZE -1        :0]            select_FLB_Type          ,
   input    wire  [PCIE_BUS_WIDTH-1       :0]            select_FLB_Data          ,
   input    wire  [PCIE_BUS_WIDTH/8-1     :0]            select_FLB_Data_en       ,
   input    wire  [64-1                   :0]            select_FLB_addr          ,
   input    wire                                         select_FLB_eof           ,
   input    wire                                         select_FLB_sof           ,
   output   wire                                         select_FLB_ready         ,

   // out data Interface
   output   wire                                         write_request            ,
   output   wire  [PCIE_BUS_WIDTH/32-1    :0]            write_byteen             ,
   output   wire                                         write_sof                ,
   output   wire                                         write_eof                ,
   output   wire  [PCIE_BUS_WIDTH-1       :0]            write_data               ,
   output   wire  [3                      :0]            write_lbe                ,
   input    wire                                         write_rdy
);


   localparam            DWORD_PCIE_ALIGNED        = 3;
   localparam            DWORD_PCIE                = PCIE_BUS_WIDTH/32;
   localparam            REMAINDER_WIDTH           = (PCIE_BUS_WIDTH==256) ? 160 : 32;

   localparam [1:0]      ST_TLP_IDLE               = 0,
                         ST_TLP_HEADER1            = 1,
                         ST_TLP_END_REQUEST        = 2;


   /////////////////////////////////////////////////////////////////////////////////////////////
   // INTERNAL SIGNALS
   /////////////////////////////////////////////////////////////////////////////////////////////
   reg   [3                        :0]         tlp_tx_sm                ;
   wire                                        in_valid                 ;
   reg                                         in_sof                   ;
   reg   [1                        :0]         in_eof                   ;
   wire  [3                        :0]         in_byte                  ;
   reg   [PCIE_BUS_WIDTH-1         :0]         in_data                  ;
   wire                                        in_ready                 ;

   reg   [96-1                     :0]         tlp_start_r              ;
   wire  [10                       :0]         tlp_length               ;
   wire  [10                       :0]         tlp_length_cut           ;
   wire  [10                       :0]         flb_length               ;
   reg   [11                       :0]         cpt_request_r            ;
   reg   [11                       :0]         length_request_r         ;
   reg   [6                        :0]         lower_address_r          ;
   reg   [PCIE_BUS_WIDTH-1         :0]         select_FLB_Data_r        ;
   reg   [96-1                     :0]         select_FLB_Data_rr       ;
   reg   [10                       :0]         size_tlp_r               ;

   wire                                        multi_request            ;
   wire                                        multi_request2           ;
   reg                                         multi_request_r          ;

   reg                                         select_FLB_request_r     ;
   reg   [PCIE_BUS_WIDTH/32-1      :0]         in_byteen                ;

   /////////////////////////////////////////////////////////////////////////////////////////////
   // Data reordering
   /////////////////////////////////////////////////////////////////////////////////////////////
   assign flb_length        = `MACRO_RP_FLB_SIZE(select_FLB_Type);
   assign tlp_length        = (flb_length > max_payload) ? max_payload : flb_length;
   assign tlp_length_cut    = (multi_request2 == 1'b1)   ? max_payload : size_tlp_r;

   assign multi_request     = (flb_length > max_payload) ? 1'b1 : 1'b0;
   assign multi_request2    = (size_tlp_r > max_payload) ? 1'b1 : 1'b0;


   assign in_valid          = (tlp_tx_sm == ST_TLP_HEADER1)         ? 1'b1                            :
                              (tlp_tx_sm == ST_TLP_END_REQUEST)     ? select_FLB_request_r & in_ready :
                                                                      1'b0                            ;

   assign select_FLB_ready  = (in_eof != 2'b0)? in_eof[0]&in_ready : in_ready;

   //------------------------------------------------------------
   // FSM to transmit TLP
   //------------------------------------------------------------
   always @(posedge clk or negedge rstn)
   if (rstn == 1'b0) begin
      tlp_tx_sm    <= ST_TLP_IDLE;
   end else begin
      if(srst == 1'b1) begin
         tlp_tx_sm    <= ST_TLP_IDLE;
      end else begin
         case (tlp_tx_sm)
            ST_TLP_IDLE :
               if(select_FLB_sof & select_FLB_request & select_FLB_ready)
                  tlp_tx_sm <= ST_TLP_HEADER1;

            ST_TLP_HEADER1:
               if(in_valid == 1'b1 && in_ready == 1'b1) begin
                  if( in_eof[0] == 1'b1 &&  select_FLB_request == 1'b0)
                    tlp_tx_sm <= ST_TLP_IDLE;
                  else if(in_eof[0] == 1'b0 && select_FLB_sof == 1'b0)
                  tlp_tx_sm <= ST_TLP_END_REQUEST;
               end

            ST_TLP_END_REQUEST :
               if(in_valid == 1'b1 && in_eof[0] == 1'b1 && in_ready == 1'b1) begin
                  if((select_FLB_request == 1'b1 && select_FLB_sof == 1'b1 && select_FLB_ready==1'b1) || multi_request_r == 1'b1)
                     tlp_tx_sm <= ST_TLP_HEADER1;
                  else
                     tlp_tx_sm <= ST_TLP_IDLE;
               end

             default : tlp_tx_sm    <= ST_TLP_IDLE;
         endcase
      end
   end

   //------------------------------------------------------------
   // Management
   //------------------------------------------------------------
   always @(posedge clk or negedge rstn)
   if (rstn == 1'b0) begin
      tlp_start_r                <= {128{1'b0}};
      in_sof                     <= 1'b0;
      in_eof                     <= 2'b0;
      lower_address_r            <= 6'h0;
      select_FLB_Data_r          <= {(PCIE_BUS_WIDTH){1'b0}};
      select_FLB_Data_rr         <= 96'b0;
      size_tlp_r                 <= {11{1'b0}};

      cpt_request_r              <= {12{1'b0}};
      length_request_r           <= {12{1'b0}};
      multi_request_r            <= 1'b0;

      select_FLB_request_r       <= 1'b0;
   end else begin
      if(srst == 1'b1) begin
         tlp_start_r                <= {128{1'b0}};
         in_sof                     <= 1'b0;
         in_eof                     <= 2'b0;
         lower_address_r            <= 6'h0;
         select_FLB_Data_r          <= {PCIE_BUS_WIDTH{1'b0}};
         select_FLB_Data_rr         <= 96'b0;
         size_tlp_r                 <= {11{1'b0}};

         cpt_request_r              <= {12{1'b0}};
         length_request_r           <= {12{1'b0}};
         multi_request_r            <= 1'b0;
         select_FLB_request_r       <= 1'b0;
      end else begin
         if(select_FLB_sof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1 && multi_request == 1'b1)
            multi_request_r         <= 1'b1;
         else if(in_valid == 1'b1 && in_eof[1] == 1'b1 && in_ready == 1'b1 && multi_request2 == 1'b0)
            multi_request_r         <= 1'b0;

         if(select_FLB_ready)
            select_FLB_request_r <= in_eof[1] | select_FLB_request;

         if(select_FLB_request == 1'b1 && select_FLB_ready == 1'b1)
            select_FLB_Data_r  <= select_FLB_Data;
         if(in_ready == 1'b1 && in_valid == 1'b1)
            select_FLB_Data_rr <= select_FLB_Data_r[PCIE_BUS_WIDTH-1 -:96] ;

         if(select_FLB_sof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1) begin
            tlp_start_r         <= {7'b0,1'b0,16'h0,`MACRO_RP_FLB_TAG(select_FLB_Type),`MACRO_RP_FLB_REQUESTER(select_FLB_Type),2'b0,3'b0,tlp_length,3'b0,{flb_length[10:0],2'b0},6'b0,`MACRO_RP_FLB_AT(select_FLB_Type),1'b0,select_FLB_addr[6:0]};

         end else if(in_valid == 1'b1 && in_eof[0] == 1'b1 && in_ready == 1'b1 && multi_request_r == 1'b1) begin
            tlp_start_r         <= {7'b0,1'b0,16'h0,`MACRO_RP_FLB_TAG(select_FLB_Type),`MACRO_RP_FLB_REQUESTER(select_FLB_Type),2'b0,3'b0,tlp_length_cut[10:0],3'b0,{size_tlp_r[10:0],2'b0},6'b0,`MACRO_RP_FLB_AT(select_FLB_Type),1'b0,lower_address_r};


         end

         if(select_FLB_sof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1) begin
            lower_address_r   <= select_FLB_addr[6:0];
         end else if(in_valid == 1'b1 && in_eof[0] == 1'b1 && in_ready == 1'b1)
            lower_address_r   <= lower_address_r + max_payload*4;

         if(select_FLB_sof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1) begin
            size_tlp_r     <= flb_length;
         end else if(in_ready == 1'b1 && in_eof[1] == 1'b1 && in_valid == 1'b1) begin
            if(size_tlp_r >= max_payload)
               size_tlp_r  <= size_tlp_r - max_payload;
            else
               size_tlp_r  <= {11{1'b0}};
         end

         if(select_FLB_sof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1) begin

            cpt_request_r      <= tlp_length+ DWORD_PCIE_ALIGNED;
            length_request_r   <= tlp_length+ DWORD_PCIE_ALIGNED;
         end else if(in_valid == 1'b1 && in_ready == 1'b1) begin
              if(in_eof[0] == 1'b1 && multi_request_r == 1'b1) begin
                cpt_request_r      <= tlp_length_cut+ DWORD_PCIE_ALIGNED;
                length_request_r   <= tlp_length_cut+ DWORD_PCIE_ALIGNED;
              end else
               cpt_request_r   <= cpt_request_r - in_byte;
         end

         // SOF management
         if(in_ready == 1'b1 && select_FLB_sof == 1'b0 && in_sof == 1'b1 && in_valid == 1'b1)
            in_sof         <= 1'b0;
         else if(in_valid == 1'b1 && in_eof[0] == 1'b1 && multi_request_r == 1'b1 && in_ready == 1'b1) begin
            in_sof         <= 1'b1;
         end else if(select_FLB_sof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1) begin
            in_sof         <= 1'b1;
         end

         if(multi_request2 == 1'b1 && cpt_request_r < (3*PCIE_BUS_WIDTH/32) && cpt_request_r>= (PCIE_BUS_WIDTH/16) && in_ready == 1'b1 && in_valid == 1'b1)
            in_eof[1] <= 1'b1;
         else if(select_FLB_sof == 1'b1 && select_FLB_eof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1 && tlp_length > DWORD_PCIE_ALIGNED)
            in_eof[1] <= 1'b1;
         else if(select_FLB_eof == 1'b1 &&  select_FLB_sof == 1'b0 && length_request_r[11:3] != 9'b0 && length_request_r[2:0] != 3'b0 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1)
            in_eof[1] <= 1'b1;
         else if(in_valid == 1'b1 && in_ready == 1'b1)
            in_eof[1] <= 1'b0;
         // EOF management
         if(in_ready == 1'b1 && in_eof[0] == 1'b1 && select_FLB_eof == 1'b0 && in_valid == 1'b1 && (multi_request_r==1'b0 || (multi_request_r==1'b1 && tlp_length_cut>= DWORD_PCIE_ALIGNED)) )
           in_eof[0] <= 1'b0;
         else if(select_FLB_sof == 1'b1 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1 && tlp_length <= DWORD_PCIE_ALIGNED)
           in_eof[0] <= 1'b1;
         else if(select_FLB_eof == 1'b1 && select_FLB_sof == 1'b0 && length_request_r[2:0] == 3'b0 && select_FLB_request == 1'b1 && select_FLB_ready == 1'b1)
           in_eof[0] <= 1'b1;


         else if(in_valid == 1'b1 && in_ready == 1'b1)




           in_eof[0] <= in_eof[1];
   end
      end

   assign in_byte = ( cpt_request_r >= (PCIE_BUS_WIDTH/32)                     ) ? (PCIE_BUS_WIDTH/32)              :
                                                               cpt_request_r[3:0];


   //-------------------------------------------------------------

   // Internally bus synchronization
   //-------------------------------------------------------------
   bus_valid_ready_synchronizer #(
      .DATA_WIDTH (PCIE_BUS_WIDTH + PCIE_BUS_WIDTH/32 +1 +1                )
   ) DATA_SYNC_INST (
      .clk        (clk                                                       ),
      .rstn       (rstn                                                      ),
      .srst       (srst                                                      ),
      .validin    (in_valid                                                  ),
      .datain     ({in_byteen,in_data,in_sof,in_eof[0]          }          ),
      .readyout   (in_ready                                                  ),
      .validout   (write_request                                           ),
      .dataout    ({write_byteen,write_data,write_sof,write_eof}           ),
      .readyin    (write_rdy                                               )
   );

   always @*
   begin: BYTEEN_ALTERA_PROC
      case(in_byte)
      4'd1    : in_byteen <= 8'd1;
      4'd2    : in_byteen <= 8'd3;
      4'd3    : in_byteen <= 8'd7;
      4'd4    : in_byteen <= 8'hf;
      4'd5    : in_byteen <= 8'h1f;
      4'd6    : in_byteen <= 8'h3f;
      4'd7    : in_byteen <= 8'h7f;
      default : in_byteen <= 8'hff;
      endcase

      case(in_sof)
      1'b1:    in_data <= {select_FLB_Data_r[159:0],tlp_start_r[95:0]};
      default: in_data <= {select_FLB_Data_r[159:0],select_FLB_Data_rr};     
      endcase
   end
endmodule
