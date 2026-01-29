//-----------------------------------------------------------------------------
// This confidential and proprietary software may be used only as authorized by
// a licensing agreement from ip-maker. In the event of publication, a
// copyright notice must be reproduced on all authorized copies.
//
//-----------------------------------------------------------------------------
//-----------------------------------------------------------------------------
// Description:
//
// Dependency :
//-----------------------------------------------------------------------------
// Revision:
// $Log:  $
// Revision 1.1  16/09/2010
// "creation"
//
//-----------------------------------------------------------------------------

module dcram_fifo(
   data,
   wren,
   wraddress,
   rdaddress,
   wrclock,
   rdclock,
   q
);
   parameter                      ADDR_WIDTH = 8;
   parameter                      DATA_WIDTH = 16;
   input wire [(DATA_WIDTH-1):0]  data;
   input wire                     wren;
   input wire [(ADDR_WIDTH-1):0]  wraddress;
   input wire [(ADDR_WIDTH-1):0]  rdaddress;
   input wire                     wrclock;
   input wire                     rdclock;
   output wire [(DATA_WIDTH-1):0] q;

   reg [DATA_WIDTH-1:0]      ram_block[0:2**ADDR_WIDTH-1];
   reg [ADDR_WIDTH-1:0]      rdaddr_r;
   reg [ADDR_WIDTH-1:0]      wraddr_r;
   reg                       wren_r;
   reg [DATA_WIDTH-1:0]      wrdata_r;

   // Write process

   always @(posedge wrclock)
   begin: WRITE_PROC

      begin
         wren_r <= wren;
         wraddr_r <= wraddress;
         wrdata_r <= data;
      end
   end

   // This process aims to reproduce a standard RAM behaviour where the
   // memory segment corresponding to the address is effectively written
   // with the input data on the second half of the clock cycle. The rising
   // edge of the clock is used in order to include also the RAM delay.

   always @(posedge wrclock)
   begin: RAM_PROC

      begin
         if (wren_r == 1'b1)
            ram_block[wraddr_r] <= wrdata_r;
      end
   end

   // Read process

   always @(posedge rdclock)
   begin: READ_PROC

         rdaddr_r <= rdaddress;
   end

   assign q = ram_block[rdaddr_r];

endmodule
