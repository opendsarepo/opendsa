/******************************************************************************
*
* Copyright (C) 2009 - 2014 Xilinx, Inc.  All rights reserved.
*
* Permission is hereby granted, free of charge, to any person obtaining a copy
* of this software and associated documentation files (the "Software"), to deal
* in the Software without restriction, including without limitation the rights
* to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
* copies of the Software, and to permit persons to whom the Software is
* furnished to do so, subject to the following conditions:
*
* The above copyright notice and this permission notice shall be included in
* all copies or substantial portions of the Software.
*
* Use of the Software is limited solely to applications:
* (a) running on a Xilinx device, or
* (b) that interact with a Xilinx device through a bus or interconnect.
*
* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
* IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
* FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL
* XILINX  BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY,
* WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF
* OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
* SOFTWARE.
*
* Except as contained in this notice, the name of the Xilinx shall not be used
* in advertising or otherwise to promote the sale, use or other dealings in
* this Software without prior written authorization from Xilinx.
*
******************************************************************************/

/*
 * helloworld.c: simple test application
 *
 * This application configures UART 16550 to baud rate 9600.
 * PS7 UART (Zynq) is not initialized by this application, since
 * bootrom/bsp configures it to baud rate 115200
 *
 * ------------------------------------------------
 * | UART TYPE   BAUD RATE                        |
 * ------------------------------------------------
 *   uartns550   9600
 *   uartlite    Configurable only in HW design
 *   ps7_uart    115200 (configured by bootrom/bsp)
 */

#include <stdio.h>
#include "platform.h"
#include "xil_printf.h"
#include "xparameters.h"
#include "xaxipmon.h"
#include "xil_cache.h"
#include "xuartps_hw.h"
#include <string.h>

#define AXIPMON_DEVICE_ID     XPAR_PSU_APM_1_DEVICE_ID

static XAxiPmon  AxiPmonInst;


//prototypes
void IOoutbyte(char c);
//static XIOModule IOModule; /* Instance of the IO Module */
void print_status (int type);
void set_tag(u8 nb_queue [], int nb_oc);
void set_dword(u8 nb_queue [], int nb_oc);
void set_type(u8 nb_queue [], int nb_oc);
void set_lba_addr(u8 nb_queue []);
void set_local_addr(u8 nb_queue []);

void run_test(void);
u8 XIOModule_Rxchar(u32 BaseAddress);
u8 qd;
u16 size;
u8 type;
u8 tag;
u32 local_addr;
u32 lba_addr;
u64 SSD_sizemax;

int main()
{
    init_platform();
    XAxiPmon_Config *ConfigPtr = NULL;
    u32 Status;
    u8 test_choice;
    u32 readdata;

    u8 cmd[4] = {0,0,0,0};
    int i_v =0;
    ConfigPtr = XAxiPmon_LookupConfig(AXIPMON_DEVICE_ID);
    if (ConfigPtr == NULL) {
       return XST_FAILURE;
    }

    Status = XAxiPmon_CfgInitialize(&AxiPmonInst, ConfigPtr,
                            ConfigPtr->BaseAddress);
    if (Status != XST_SUCCESS) {
       return XST_FAILURE;
    }
    while(1){

      cmd[3] = 0;
      cmd[2] = 0;
      cmd[1] = 0;
      cmd[0] = 0;
      i_v = 0;
      readdata = 0;
      print("Check if NVMe is ready\r\n");
      do {
        readdata = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR);
      } while(readdata != 0x000000001);
      print("NVMe ready\r\n");

      //readdata = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+ 0x00000004);
      //print("dma_max_size=%x and dma_page_size=%x\n",readdata&&0x0000000FF,(readdata>>16));

      readdata = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+ 0x0000000C);
      SSD_sizemax = readdata << 32;

      readdata = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+ 0x00000008);
      SSD_sizemax = SSD_sizemax + readdata;

      print("enter test to start\r\n");
     /* print("1/ define Queue depth (from 1 to 128)\r\n");
      do{
           test_choice = inbyte();
           if(test_choice != 0xD){
           cmd[i_v] =  test_choice;
           i_v = i_v +1;
           }
      }while (test_choice!=0xD);

      set_tag(cmd, i_v);*/

      print("1/ define packet size in X * 1KDWord packet\r\n");
      i_v =0;
      do{
           test_choice = inbyte();
           if(test_choice != 0xD){
           cmd[i_v] =  test_choice;
           i_v = i_v +1;
           }
      }while (test_choice!=0xD);
      set_dword(cmd, i_v);

      print("2/ define direction (1/ for write, 2/ for read)\r\n");
      i_v =0;
      do{
           test_choice = inbyte();
           if(test_choice != 0xD){
           cmd[i_v] =  test_choice;
           i_v = i_v +1;
           }
      }while (test_choice!=0xD);
      set_type(cmd, i_v);

      print("3/ set local address\r\n");
      i_v =0;
      do{
           test_choice = inbyte();
           if(test_choice != 0xD){
           cmd[i_v] =  test_choice;
           i_v = i_v +1;
           }
      }while (test_choice!=0xD);
      set_local_addr(cmd);

      print("4/ set LBA address\r\n");
      i_v =0;
      do{
           test_choice = inbyte();
           if(test_choice != 0xD){
           cmd[i_v] =  test_choice;
           i_v = i_v +1;
           }
      }while (test_choice!=0xD);
      set_lba_addr(cmd);

      print("START : Run \r\n");

      run_test();
    }

    cleanup_platform();
    return 0;
}

void set_tag(u8 nb_queue [], int nb_oc){
   if(nb_oc == 1){
      qd = nb_queue[0] - '0';
   } else if(nb_oc == 2){
      qd = ((nb_queue[1] - '0')) | ((nb_queue[0] - '0') << 4);
   }else if(nb_oc == 3){
      qd = ((nb_queue[2] - '0')) | ((nb_queue[1] - '0') << 4) | ((nb_queue[0] - '0') << 8);
   } else {
      qd = ((nb_queue[3] - '0')) | ((nb_queue[2] - '0') << 4) | ((nb_queue[1] - '0') << 8) | ((nb_queue[0] - '0') <<12);
   }
   xil_printf("qd = %x \r\n", qd);
}
void set_dword(u8 nb_queue [], int nb_oc){
   if(nb_oc == 1){
      size = nb_queue[0] - '0';
   } else if(nb_oc == 2){
      size = ((nb_queue[1] - '0')) | ((nb_queue[0] - '0') << 4);
   }else if(nb_oc == 3){
      size = ((nb_queue[2] - '0')) | ((nb_queue[1] - '0') << 4) | ((nb_queue[0] - '0') << 8);
   } else {
      size = ((nb_queue[3] - '0')) | ((nb_queue[2] - '0') << 4) | ((nb_queue[1] - '0') << 8) | ((nb_queue[0] - '0') <<12);
   }
   xil_printf("size = %x \r\n", size);
}
void set_type(u8 nb_queue [], int nb_oc){
   if(nb_oc == 1){
      type = nb_queue[0] - '0';
   } else if(nb_oc == 2){
      type = ((nb_queue[1] - '0')) | ((nb_queue[0] - '0') << 4);
   }else if(nb_oc == 3){
      type = ((nb_queue[2] - '0')) | ((nb_queue[1] - '0') << 4) | ((nb_queue[0] - '0') << 8);
   } else {
      type = ((nb_queue[3] - '0')) | ((nb_queue[2] - '0') << 4) | ((nb_queue[1] - '0') << 8) | ((nb_queue[0] - '0') <<12);
   }
   xil_printf("type = %x \r\n", type);
}

void set_local_addr(u8 nb_queue []){

      local_addr = ((nb_queue[3] - '0')) | ((nb_queue[2] - '0') << 4) | ((nb_queue[1] - '0') << 8) | ((nb_queue[0] - '0') <<12);

   xil_printf("local_addr = %x \r\n", local_addr);
}

void set_lba_addr(u8 nb_queue []){

      lba_addr = ((nb_queue[3] - '0')) | ((nb_queue[2] - '0') << 4) | ((nb_queue[1] - '0') << 8) | ((nb_queue[0] - '0') <<12);
   xil_printf("lba_addr = %x \r\n", lba_addr);
}

void run_test(void){
   u8 test_choice;
   u8 cmd[128];
   int i_v = 0;
   u32 local_addr;
   u32 lba_addr;
   u32 read_data,read_status;
   u32 cpl_status;
   u32 cnt_check;
   u8 rcv_tag;
   u8 cnt_wait_cpl,wait_cpl;
   u32 reg08;

   local_addr = 0x00000000;
   lba_addr = 0x00000000;
   cnt_check = 0x0;


   for (i_v = 0 ; i_v < 128 ; i_v = i_v + 1) {
      cmd[i_v] = 0x0;
   }
   xil_printf("init table\r\n");

   do{
      for(i_v=31; i_v>= 0; i_v = i_v -1){
         if(cmd[i_v] == 0x0) {
             tag = i_v;
             local_addr = (size*i_v)<<12;
             //xil_printf("Program DMA %d\r\n", tag);
             Xil_Out32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR + 0x00000000, local_addr);
             Xil_Out32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR + 0x00000004, lba_addr);

             reg08 = tag <<24;
             //xil_printf("Program DMA %x\r\n", reg08);
             reg08 = reg08+ (type<<16);
             //xil_printf("Program DMA %x\r\n", reg08);
             reg08 = reg08+size;
             //xil_printf("Program DMA %x\r\n", reg08);
             Xil_Out32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR + 0x00000008, reg08);
             if(lba_addr == SSD_sizemax - size)
                lba_addr = 0x00000000;
             else
                lba_addr = lba_addr + size;
             cmd[i_v] = 0x1;
         }
      }

      read_data = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+0x000000010);
      if(read_data == 0x00000001)
      {
         read_data = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+0x000000020);

         rcv_tag = read_data>>24;
         //xil_printf("Completion DMA %d %x\r\n", rcv_tag,read_data);
         cmd[rcv_tag] = 0x0;
         cpl_status = read_data>>12;
         if(cpl_status&0x00000fff != 0x00000000)
          xil_printf("Error NVME command\r\n");
      }
      cnt_check = cnt_check + 1;

      if(cnt_check== 0x0100000) {
         if((!XUartPs_IsReceiveData(STDIN_BASEADDRESS))){
             print_status(type);
         } else {
    	     test_choice = XUartPs_ReadReg(STDIN_BASEADDRESS, XUARTPS_FIFO_OFFSET);
         }
         cnt_check = 0x0;
      }
   }while (test_choice!=0x0D);

   wait_cpl = 0x0;
   cnt_wait_cpl = 0x0;
   for (i_v = 0 ; i_v < 35 ; i_v = i_v + 1) {
   	   if(cmd[i_v] == 0x1) wait_cpl=wait_cpl+1;
   }

   do{
	   read_status = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+0x000000010);
	   //xil_printf("wait_cpl %d cnt_wait_cpl=%d\r\n", wait_cpl,cnt_wait_cpl);
	   if(read_status == 0x00000001)
	   {
	      read_data = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+0x000000020);
	      cnt_wait_cpl = cnt_wait_cpl+1;
	       rcv_tag = read_data>>24;
	       cmd[rcv_tag] = 0x0;
	       //xil_printf("Completion DMA %d %x\r\n", rcv_tag,cmd[rcv_tag]);
	       cpl_status = read_data>>12;
	       if(cpl_status&0x00000fff != 0x00000000)
	          xil_printf("Error NVME command\r\n");
	    }
   } while (cnt_wait_cpl!=wait_cpl);

   /*for (i_v = 0 ; i_v < 35 ; i_v = i_v + 1) {
	   xil_printf("i_v=%d cmd =%x\r\n", i_v, cmd[i_v]);
      }*/
   xil_printf("Test ended\r\n");
}


void print_status (int type){
   unsigned int performance_data;
     if(type == 0x2) {
        performance_data = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+ 0x00000050);
      } else {
        performance_data = Xil_In32(XPAR_NVME_RECORDER_APPLICATION_0_BASEADDR+ 0x00000040);
        //xil_printf("type = %x \r\n", performance_data);
      }
      if(performance_data!=0){
        performance_data = performance_data * 64;
        performance_data = performance_data/1000;
        performance_data = performance_data/1000;
        xil_printf("    perf =  %d MB/s\r\n", performance_data );
      }
}

u8 XIOModule_Rxchar(u32 BaseAddress)
{
//   while (XIOModule_IsReceiveEmpty(BaseAddress));

   return Xil_In32(BaseAddress);
}

