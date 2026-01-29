//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from IP Maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//-----------------------------------------------------------------------------
// Project : NVMe recorder
// Description :
// Author : J.DENIS
//-----------------------------------------------------------------------------
module completer_data_extractor #(
   parameter                                       NVME_DATA_WIDTH  = 256,
   parameter                                       PCIE_DATA_WIDTH  = 256,
   parameter                                       RAM_LENGTH_WIDTH = 7
) (
   input    wire                                         clk               ,                 // clk
   input    wire                                         rstn              ,                 // ASynchronous reset
   input    wire                                         srst              ,                 // Synchronous reset

   input    wire                                         in_valid          ,
   input    wire                                         in_sof            ,
   input    wire                                         in_eof            ,
   input    wire   [63                       :0]         in_mask_address   ,
   input    wire   [3                        :0]         in_byte           ,
   input    wire   [(PCIE_DATA_WIDTH/8)-1    :0]         in_be             ,
   input    wire   [PCIE_DATA_WIDTH-1        :0]         in_data           ,
   output   wire                                         in_ready          ,

   output   reg                                          out_valid         ,
   output   reg    [63                       :0]         out_address       ,
   output   wire                                         out_sof           ,
   output   wire                                         out_eof           ,
   output   reg    [10                       :0]         out_length        ,
   output   wire   [(NVME_DATA_WIDTH/8)-1    :0]         out_be            ,
   output   wire   [NVME_DATA_WIDTH-1        :0]         out_data          ,
   input    wire                                         out_ready
);
`include "log2ceil_pkg.v"

   localparam                                  INTERNAL_BUS_WIDTH      = (PCIE_DATA_WIDTH == 256) ? 2*PCIE_DATA_WIDTH-128 : 2*PCIE_DATA_WIDTH;
   localparam                                  COUNTER_WIDTH           = log2ceil(INTERNAL_BUS_WIDTH/32);
   localparam   [COUNTER_WIDTH            :0]  COUNTER_EOF             = 'd17;
   localparam                                  FIRST_BIT_DWORD_COUNT   = 64;

   wire   [INTERNAL_BUS_WIDTH-1     :0]                 shift_data_iii           ;
   reg    [INTERNAL_BUS_WIDTH-1     :0]                 shift_data_r             ;

   reg    [COUNTER_WIDTH            :0]                 cpt_data_r               ;
   wire   [COUNTER_WIDTH            :0]                 add_cpt_data             ;
   wire                                                 check_read               ;
   wire                                                 check_write              ;
   wire   [10                       :0]                 dword_count              ;
   reg    [1                        :0]                 out_sof_r                ;
   reg    [1                        :0]                 out_eof_r                ;
   wire   [3                        :0]                 out_byte                 ;

   generate
   if(PCIE_DATA_WIDTH == 64)
   begin // PCIE_DATA_WIDTH_64_ADDRESS
      assign shift_data_iii = in_data ;
   end else if(PCIE_DATA_WIDTH == 128) begin // PCIE_DATA_WIDTH_128_ADDRESS
      wire [3:0] case_shift;
      reg  [127:0] shift_data_iii_case;

      assign shift_data_iii[INTERNAL_BUS_WIDTH-1:128] =  (cpt_data_r[3:2] != 2'b00) ?  in_data  : {COUNTER_WIDTH+1{1'b0}};
      assign shift_data_iii[127:0] = shift_data_iii_case;
      assign case_shift = {check_write & ~in_sof, check_read,cpt_data_r[3:2]};

      always @* // (case_shift or shift_data_iii or shift_data_r or in_data) begin
      begin
      case ({case_shift}) // synthesis parallel_case full_case

      4'd8,4'hd :  shift_data_iii_case = in_data;
      4'd9      :  shift_data_iii_case = shift_data_r[127:0];
      4'he,4'h6 :  shift_data_iii_case = shift_data_r[255:128];
      default : shift_data_iii_case    = 128'b0;
     endcase
     end
   end else begin// PCIE_DATA_WIDTH_256_ADDRESS
      //assign shift_data_iii = (in_sof == 1'b1) ? {{(INTERNAL_BUS_WIDTH-128){1'b0}},in_data[255:128]} : {in_data,shift_data_r[127:0]} : {in_data,shift_data_r[PCIE_DATA_WIDTH +:128]};

      assign shift_data_iii[INTERNAL_BUS_WIDTH-1:128] =  (in_sof == 1'b1) ? {{(INTERNAL_BUS_WIDTH-128){1'b0}}} : in_data;
      assign shift_data_iii[127:0] =  (in_sof == 1'b1) ? in_data[255:128] : (check_write == 1'b1 && check_read == 1'b0 ) ? shift_data_r[127:0] : shift_data_r[PCIE_DATA_WIDTH +:128];
   end
   endgenerate

   assign dword_count     =  in_data[FIRST_BIT_DWORD_COUNT     +:10];


   always @(negedge rstn or posedge clk)
   begin : REGISTER_SHIFTER
      integer i_v;
      if (rstn == 1'b0) begin
         shift_data_r       <= {INTERNAL_BUS_WIDTH{1'b0}};
         cpt_data_r         <= {COUNTER_WIDTH+1{1'b0}};
         out_valid          <= 1'b0;
         out_address        <= 64'h0;
         out_sof_r          <= 2'b0;
         out_eof_r          <= 2'b0;
         out_length         <= 11'b0;
      end else begin
         if(srst == 1'b1) begin
            shift_data_r       <= {INTERNAL_BUS_WIDTH{1'b0}};
            cpt_data_r         <= {COUNTER_WIDTH+1{1'b0}};
            out_valid          <= 1'b0;
            out_address        <= 64'h0;
            out_sof_r          <= 2'b0;
            out_eof_r          <= 2'b0;
            out_length         <= 11'b0;
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
               out_length  <= dword_count;

            if(check_write == 1'b1 && in_sof == 1'b1) begin
               for (i_v=0; i_v<64; i_v=i_v+1)
               out_address[i_v]  <= in_mask_address[i_v] & in_data[i_v];
            end else if(check_read == 1'b1)
               out_address <= out_address + 4'd4;

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
               if(add_cpt_data >= 'd17) begin
                  out_eof_r[1]   <= in_eof;
                  out_eof_r[0]   <= out_eof_r[1];
               end else begin
                  out_eof_r[0]   <= in_eof;
               end
            end else if(check_read == 1'b1) begin
               out_eof_r   <= {1'b0,out_eof_r[1]};
            end

            if(PCIE_DATA_WIDTH == 256 || PCIE_DATA_WIDTH == 128) begin
               if(check_write == 1'b1 || check_read == 1'b1)
                  shift_data_r <= shift_data_iii;
            //   if(check_write == 1'b1 && check_read == 1'b0)
            //      shift_data_r <= shift_data_iii;
            //   else if(check_read == 1'b1)
            //      shift_data_r <= {in_data,shift_data_r[PCIE_DATA_WIDTH +:128]};
            //end else if(PCIE_DATA_WIDTH == 128) begin
            //   if(check_write == 1'b1 && check_read == 1'b0)
            //      shift_data_r <= {in_data,shift_data_r[127:0]};
            //   else if(check_read == 1'b1)
            //      shift_data_r <= {128'h0,in_data};
            end else if(PCIE_DATA_WIDTH == 64) begin
               if(check_write == 1'b1 && check_read == 1'b0)
                  shift_data_r <= {in_data,shift_data_r[63:0]};
               else if(check_read == 1'b1)
                  shift_data_r <= {64'h0,in_data};
            end
         end
      end
   end

   assign in_ready        = (check_read == 1'b1 && cpt_data_r <= 'd12) ? ~out_eof_r[1] : (cpt_data_r <= (PCIE_DATA_WIDTH/32)) ? ~out_eof_r[0] & ~out_eof_r[1]: 1'b0;
   assign check_write     = in_valid & in_ready;
   assign check_read      = out_valid & out_ready;
   assign add_cpt_data    = cpt_data_r+in_byte;

   assign out_data        = shift_data_r[NVME_DATA_WIDTH-1      :0];
   assign out_sof         = out_sof_r[0]   ;
   assign out_eof         = out_eof_r[0]   ;
   assign out_byte        = (out_eof_r[0] == 1'b1) ?  cpt_data_r[3:0] : (NVME_DATA_WIDTH/32) ;

   generate
      genvar a_v;
      for (a_v=0; a_v<(NVME_DATA_WIDTH/32); a_v=a_v+1)
         assign out_be[a_v*4 +:4] =  (a_v >=cpt_data_r )                        ? 4'h0 :
                                                                                  4'hF;
   endgenerate
endmodule
