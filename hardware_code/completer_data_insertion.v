//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------
module completer_data_insertion #(
   parameter                                       NVME_DATA_WIDTH  = 256,
   parameter                                       PCIE_DATA_WIDTH  = 256,
   parameter                                       RAM_LENGTH_WIDTH = 7
) (
   input    wire                                         clk         ,                 // clk
   input    wire                                         rstn        ,                 // ASynchronous reset
   input    wire                                         srst        ,                 // Synchronous reset

   input    wire                                         in_valid    ,
   input    wire                                         in_sof      ,
   input    wire                                         in_eof      ,
   input    wire   [3                        :0]         in_byte     ,
   input    wire   [3                        :0]         in_lbe      ,
   input    wire   [(PCIE_DATA_WIDTH/8)-1    :0]         in_be       ,
   input    wire   [PCIE_DATA_WIDTH-1        :0]         in_data     ,
   output   wire                                         in_ready    ,

   output   reg                                          out_valid   ,
   output   wire                                         out_sof     ,
   output   wire                                         out_eof     ,
   output   reg    [3                        :0]         out_lbe     ,
   output   wire   [(NVME_DATA_WIDTH/8)-1    :0]         out_be      ,
   output   wire   [NVME_DATA_WIDTH-1        :0]         out_data    ,
   input    wire                                         out_ready
);
`include "log2ceil_pkg.v"

   localparam            INTERNAL_BUS_WIDTH = PCIE_DATA_WIDTH+96;
   localparam            COUNTER_WIDTH      = log2ceil(INTERNAL_BUS_WIDTH/32);
   localparam            EOF_POSITION       = (PCIE_DATA_WIDTH == 128) ? 'd9 : 'd17;

   wire   [INTERNAL_BUS_WIDTH-1     :0]                 shift_data_iii           ;
   reg    [INTERNAL_BUS_WIDTH-1     :0]                 shift_data_r             ;

   reg    [COUNTER_WIDTH            :0]                 cpt_data_r               ;
   wire   [COUNTER_WIDTH            :0]                 add_cpt_data             ;
   wire                                                 check_read               ;
   wire                                                 check_write              ;
   reg    [1                        :0]                 out_sof_r                ;
   reg    [1                        :0]                 out_eof_r                ;
   wire   [3                        :0]                 out_byte                 ;

   generate
   if(PCIE_DATA_WIDTH == 64)
   begin // PCIE_DATA_WIDTH_64_ADDRESS
      assign shift_data_iii    = in_data;
   end else if(PCIE_DATA_WIDTH == 128) begin // PCIE_DATA_WIDTH_128_ADDRESS
      assign shift_data_iii    = {{(INTERNAL_BUS_WIDTH-96){1'b0}},in_data[127:32]};

   end else begin// PCIE_DATA_WIDTH_256_ADDRESS
      assign shift_data_iii    = {{(INTERNAL_BUS_WIDTH-160){1'b0}},in_data[255:160]};

   end
   endgenerate

   always @(negedge rstn or posedge clk)
   begin : REGISTER_SHIFTER
      if (rstn == 1'b0) begin
         shift_data_r       <= {INTERNAL_BUS_WIDTH{1'b0}};
         cpt_data_r         <= {COUNTER_WIDTH+1{1'b0}};
         out_valid          <= 1'b0;
         out_sof_r          <= 2'b0;
         out_eof_r          <= 2'b0;
         out_lbe            <= 4'b0;
      end else begin
         if(srst == 1'b1) begin
            shift_data_r       <= {INTERNAL_BUS_WIDTH{1'b0}};
            cpt_data_r         <= {COUNTER_WIDTH+1{1'b0}};
            out_valid          <= 1'b0;
            out_sof_r          <= 2'b0;
            out_eof_r          <= 2'b0;
            out_lbe            <= 4'b0;
         end else begin
            if(in_eof == 1'b1 && check_write == 1'b1)
               out_valid     <= 1'b1;
            else if(out_ready == 1'b1 && out_eof == 1'b1)
               out_valid     <= 1'b0;
            else if(check_write == 1'b1 && check_read == 1'b0 && add_cpt_data >= (NVME_DATA_WIDTH/32))
               out_valid     <= 1'b1;
/*            else if(cpt_data_r >= (NVME_DATA_WIDTH/32) && out_valid == 1'b0)
               out_valid     <= 1'b1;*/
            else if(out_ready == 1'b1 && cpt_data_r < (2*NVME_DATA_WIDTH/32) && out_eof_r[1] == 1'b0 && check_write == 1'b0)
               out_valid     <= 1'b0;

            if(check_write == 1'b1 && check_read == 1'b0)
               cpt_data_r <= add_cpt_data;
            else if(check_write == 1'b1 && check_read == 1'b1)
               cpt_data_r <= add_cpt_data - out_byte;
            else if(check_read == 1'b1) begin
               if(cpt_data_r <= (NVME_DATA_WIDTH/32))
                cpt_data_r <= {COUNTER_WIDTH+1{1'b0}};
               else
                cpt_data_r <= cpt_data_r-(NVME_DATA_WIDTH/32);
            end

            if(check_write == 1'b1 && in_sof == 1'b1)
               out_lbe <= in_lbe;

            if(check_write == 1'b1 && in_sof == 1'b1)
               out_sof_r   <= 1'b1;
            else if(check_read == 1'b1)
               out_sof_r   <= 1'b0;

            if(check_write == 1'b1 && check_read == 1'b0) begin
               if(add_cpt_data <= (NVME_DATA_WIDTH/32)) begin
                  out_eof_r[0]   <= in_eof;
               end else begin
                  out_eof_r[1]   <= in_eof;
               end
            end else if(check_write == 1'b1 && check_read == 1'b1) begin
               //if(add_cpt_data - out_byte <= (NVME_DATA_WIDTH/32)) begin
               if(add_cpt_data >= EOF_POSITION) begin
                  out_eof_r[1]   <= in_eof;
                  out_eof_r[0]   <= out_eof_r[1];
               end else begin
                  out_eof_r[0]   <= in_eof;
               end
            end else if(check_read == 1'b1) begin
               out_eof_r   <= {1'b0,out_eof_r[1]};
            end

            if(in_sof == 1'b1 && check_write == 1'b1 ) begin
               shift_data_r <= shift_data_iii;
            end else begin
               if(PCIE_DATA_WIDTH == 256) begin
                  if(check_write == 1'b1 && check_read == 1'b0)
                     shift_data_r <= {in_data,shift_data_r[95:0]};
                  else if(check_read == 1'b1)
                     shift_data_r <= {in_data,shift_data_r[PCIE_DATA_WIDTH +:96]};
               end else if(PCIE_DATA_WIDTH == 128) begin
                  if(check_write == 1'b1 && check_read == 1'b0)
                     shift_data_r <= {32'b0,in_data,shift_data_r[95:0]};
                  else if(check_read == 1'b1)
                     shift_data_r <= {32'b0,in_data,shift_data_r[PCIE_DATA_WIDTH +:96]};
               end else if(PCIE_DATA_WIDTH == 64) begin
                  if(check_write == 1'b1 && check_read == 1'b0)
                     shift_data_r <= {in_data,shift_data_r[63:0]};
                  else if(check_read == 1'b1)
                     shift_data_r <= {64'h0,in_data};
               end
            end
         end
      end
   end

   assign in_ready        = (check_read == 1'b1 && cpt_data_r <='d11) ? ~out_eof_r[1] : (cpt_data_r <= (PCIE_DATA_WIDTH/32)) ? ~out_eof_r[0] & ~out_eof_r[1]: 1'b0;
   assign check_write     = in_valid & in_ready;
   assign check_read      = out_valid & out_ready;
   assign add_cpt_data    = cpt_data_r+in_byte;

   assign out_data        = shift_data_r[NVME_DATA_WIDTH-1      :0];
   assign out_sof         = out_sof_r[0]   ;
   assign out_eof         = out_eof_r[0]   ;
   assign out_byte        = (out_eof_r[0]) ? cpt_data_r[COUNTER_WIDTH:0] : (NVME_DATA_WIDTH/32);

   generate
      genvar a_v;
      for (a_v=0; a_v<(NVME_DATA_WIDTH/32); a_v=a_v+1)
         assign out_be[a_v*4 +:4] =  (a_v >=cpt_data_r )                        ? 4'h0 :
                                                                                  4'hF;
   endgenerate
endmodule
