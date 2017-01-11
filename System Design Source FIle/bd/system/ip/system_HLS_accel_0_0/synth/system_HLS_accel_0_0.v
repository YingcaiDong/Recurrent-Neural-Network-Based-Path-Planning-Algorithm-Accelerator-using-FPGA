// (c) Copyright 1995-2016 Xilinx, Inc. All rights reserved.
// 
// This file contains confidential and proprietary information
// of Xilinx, Inc. and is protected under U.S. and
// international copyright and other intellectual property
// laws.
// 
// DISCLAIMER
// This disclaimer is not a license and does not grant any
// rights to the materials distributed herewith. Except as
// otherwise provided in a valid license issued to you by
// Xilinx, and to the maximum extent permitted by applicable
// law: (1) THESE MATERIALS ARE MADE AVAILABLE "AS IS" AND
// WITH ALL FAULTS, AND XILINX HEREBY DISCLAIMS ALL WARRANTIES
// AND CONDITIONS, EXPRESS, IMPLIED, OR STATUTORY, INCLUDING
// BUT NOT LIMITED TO WARRANTIES OF MERCHANTABILITY, NON-
// INFRINGEMENT, OR FITNESS FOR ANY PARTICULAR PURPOSE; and
// (2) Xilinx shall not be liable (whether in contract or tort,
// including negligence, or under any other theory of
// liability) for any loss or damage of any kind or nature
// related to, arising under or in connection with these
// materials, including for any direct, or any indirect,
// special, incidental, or consequential loss or damage
// (including loss of data, profits, goodwill, or any type of
// loss or damage suffered as a result of any action brought
// by a third party) even if such damage or loss was
// reasonably foreseeable or Xilinx had been advised of the
// possibility of the same.
// 
// CRITICAL APPLICATIONS
// Xilinx products are not designed or intended to be fail-
// safe, or for use in any application requiring fail-safe
// performance, such as life-support or safety devices or
// systems, Class III medical devices, nuclear facilities,
// applications related to the deployment of airbags, or any
// other applications that could lead to death, personal
// injury, or severe property or environmental damage
// (individually and collectively, "Critical
// Applications"). Customer assumes the sole risk and
// liability of any use of Xilinx products in Critical
// Applications, subject only to applicable laws and
// regulations governing limitations on product liability.
// 
// THIS COPYRIGHT NOTICE AND DISCLAIMER MUST BE RETAINED AS
// PART OF THIS FILE AT ALL TIMES.
// 
// DO NOT MODIFY THIS FILE.


// IP VLNV: xilinx.com:hls:HLS_accel:1.0
// IP Revision: 1605192151

(* X_CORE_INFO = "HLS_accel,Vivado 2014.4" *)
(* CHECK_LICENSE_TYPE = "system_HLS_accel_0_0,HLS_accel,{}" *)
(* CORE_GENERATION_INFO = "system_HLS_accel_0_0,HLS_accel,{x_ipProduct=Vivado 2014.4,x_ipVendor=xilinx.com,x_ipLibrary=hls,x_ipName=HLS_accel,x_ipVersion=1.0,x_ipCoreRevision=1605192151,x_ipLanguage=VERILOG,x_ipSimLanguage=MIXED,C_S_AXI_CONTROL_BUS_ADDR_WIDTH=5,C_S_AXI_CONTROL_BUS_DATA_WIDTH=32}" *)
(* DowngradeIPIdentifiedWarnings = "yes" *)
module system_HLS_accel_0_0 (
  s_axi_CONTROL_BUS_AWADDR,
  s_axi_CONTROL_BUS_AWVALID,
  s_axi_CONTROL_BUS_AWREADY,
  s_axi_CONTROL_BUS_WDATA,
  s_axi_CONTROL_BUS_WSTRB,
  s_axi_CONTROL_BUS_WVALID,
  s_axi_CONTROL_BUS_WREADY,
  s_axi_CONTROL_BUS_BRESP,
  s_axi_CONTROL_BUS_BVALID,
  s_axi_CONTROL_BUS_BREADY,
  s_axi_CONTROL_BUS_ARADDR,
  s_axi_CONTROL_BUS_ARVALID,
  s_axi_CONTROL_BUS_ARREADY,
  s_axi_CONTROL_BUS_RDATA,
  s_axi_CONTROL_BUS_RRESP,
  s_axi_CONTROL_BUS_RVALID,
  s_axi_CONTROL_BUS_RREADY,
  ap_clk,
  ap_rst_n,
  interrupt,
  INPUT_STREAM_TVALID,
  INPUT_STREAM_TREADY,
  INPUT_STREAM_TDATA,
  INPUT_STREAM_TDEST,
  INPUT_STREAM_TKEEP,
  INPUT_STREAM_TSTRB,
  INPUT_STREAM_TUSER,
  INPUT_STREAM_TLAST,
  INPUT_STREAM_TID,
  OUTPUT_STREAM_TVALID,
  OUTPUT_STREAM_TREADY,
  OUTPUT_STREAM_TDATA,
  OUTPUT_STREAM_TDEST,
  OUTPUT_STREAM_TKEEP,
  OUTPUT_STREAM_TSTRB,
  OUTPUT_STREAM_TUSER,
  OUTPUT_STREAM_TLAST,
  OUTPUT_STREAM_TID
);

(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS AWADDR" *)
input wire [4 : 0] s_axi_CONTROL_BUS_AWADDR;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS AWVALID" *)
input wire s_axi_CONTROL_BUS_AWVALID;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS AWREADY" *)
output wire s_axi_CONTROL_BUS_AWREADY;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS WDATA" *)
input wire [31 : 0] s_axi_CONTROL_BUS_WDATA;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS WSTRB" *)
input wire [3 : 0] s_axi_CONTROL_BUS_WSTRB;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS WVALID" *)
input wire s_axi_CONTROL_BUS_WVALID;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS WREADY" *)
output wire s_axi_CONTROL_BUS_WREADY;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS BRESP" *)
output wire [1 : 0] s_axi_CONTROL_BUS_BRESP;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS BVALID" *)
output wire s_axi_CONTROL_BUS_BVALID;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS BREADY" *)
input wire s_axi_CONTROL_BUS_BREADY;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS ARADDR" *)
input wire [4 : 0] s_axi_CONTROL_BUS_ARADDR;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS ARVALID" *)
input wire s_axi_CONTROL_BUS_ARVALID;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS ARREADY" *)
output wire s_axi_CONTROL_BUS_ARREADY;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS RDATA" *)
output wire [31 : 0] s_axi_CONTROL_BUS_RDATA;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS RRESP" *)
output wire [1 : 0] s_axi_CONTROL_BUS_RRESP;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS RVALID" *)
output wire s_axi_CONTROL_BUS_RVALID;
(* X_INTERFACE_INFO = "xilinx.com:interface:aximm:1.0 s_axi_CONTROL_BUS RREADY" *)
input wire s_axi_CONTROL_BUS_RREADY;
(* X_INTERFACE_INFO = "xilinx.com:signal:clock:1.0 ap_clk CLK" *)
input wire ap_clk;
(* X_INTERFACE_INFO = "xilinx.com:signal:reset:1.0 ap_rst_n RST" *)
input wire ap_rst_n;
(* X_INTERFACE_INFO = "xilinx.com:signal:interrupt:1.0 interrupt INTERRUPT" *)
output wire interrupt;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TVALID" *)
input wire INPUT_STREAM_TVALID;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TREADY" *)
output wire INPUT_STREAM_TREADY;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TDATA" *)
input wire [31 : 0] INPUT_STREAM_TDATA;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TDEST" *)
input wire [4 : 0] INPUT_STREAM_TDEST;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TKEEP" *)
input wire [3 : 0] INPUT_STREAM_TKEEP;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TSTRB" *)
input wire [3 : 0] INPUT_STREAM_TSTRB;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TUSER" *)
input wire [3 : 0] INPUT_STREAM_TUSER;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TLAST" *)
input wire [0 : 0] INPUT_STREAM_TLAST;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 INPUT_STREAM TID" *)
input wire [4 : 0] INPUT_STREAM_TID;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TVALID" *)
output wire OUTPUT_STREAM_TVALID;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TREADY" *)
input wire OUTPUT_STREAM_TREADY;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TDATA" *)
output wire [31 : 0] OUTPUT_STREAM_TDATA;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TDEST" *)
output wire [4 : 0] OUTPUT_STREAM_TDEST;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TKEEP" *)
output wire [3 : 0] OUTPUT_STREAM_TKEEP;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TSTRB" *)
output wire [3 : 0] OUTPUT_STREAM_TSTRB;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TUSER" *)
output wire [3 : 0] OUTPUT_STREAM_TUSER;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TLAST" *)
output wire [0 : 0] OUTPUT_STREAM_TLAST;
(* X_INTERFACE_INFO = "xilinx.com:interface:axis:1.0 OUTPUT_STREAM TID" *)
output wire [4 : 0] OUTPUT_STREAM_TID;

  HLS_accel #(
    .C_S_AXI_CONTROL_BUS_ADDR_WIDTH(5),
    .C_S_AXI_CONTROL_BUS_DATA_WIDTH(32)
  ) inst (
    .s_axi_CONTROL_BUS_AWADDR(s_axi_CONTROL_BUS_AWADDR),
    .s_axi_CONTROL_BUS_AWVALID(s_axi_CONTROL_BUS_AWVALID),
    .s_axi_CONTROL_BUS_AWREADY(s_axi_CONTROL_BUS_AWREADY),
    .s_axi_CONTROL_BUS_WDATA(s_axi_CONTROL_BUS_WDATA),
    .s_axi_CONTROL_BUS_WSTRB(s_axi_CONTROL_BUS_WSTRB),
    .s_axi_CONTROL_BUS_WVALID(s_axi_CONTROL_BUS_WVALID),
    .s_axi_CONTROL_BUS_WREADY(s_axi_CONTROL_BUS_WREADY),
    .s_axi_CONTROL_BUS_BRESP(s_axi_CONTROL_BUS_BRESP),
    .s_axi_CONTROL_BUS_BVALID(s_axi_CONTROL_BUS_BVALID),
    .s_axi_CONTROL_BUS_BREADY(s_axi_CONTROL_BUS_BREADY),
    .s_axi_CONTROL_BUS_ARADDR(s_axi_CONTROL_BUS_ARADDR),
    .s_axi_CONTROL_BUS_ARVALID(s_axi_CONTROL_BUS_ARVALID),
    .s_axi_CONTROL_BUS_ARREADY(s_axi_CONTROL_BUS_ARREADY),
    .s_axi_CONTROL_BUS_RDATA(s_axi_CONTROL_BUS_RDATA),
    .s_axi_CONTROL_BUS_RRESP(s_axi_CONTROL_BUS_RRESP),
    .s_axi_CONTROL_BUS_RVALID(s_axi_CONTROL_BUS_RVALID),
    .s_axi_CONTROL_BUS_RREADY(s_axi_CONTROL_BUS_RREADY),
    .ap_clk(ap_clk),
    .ap_rst_n(ap_rst_n),
    .interrupt(interrupt),
    .INPUT_STREAM_TVALID(INPUT_STREAM_TVALID),
    .INPUT_STREAM_TREADY(INPUT_STREAM_TREADY),
    .INPUT_STREAM_TDATA(INPUT_STREAM_TDATA),
    .INPUT_STREAM_TDEST(INPUT_STREAM_TDEST),
    .INPUT_STREAM_TKEEP(INPUT_STREAM_TKEEP),
    .INPUT_STREAM_TSTRB(INPUT_STREAM_TSTRB),
    .INPUT_STREAM_TUSER(INPUT_STREAM_TUSER),
    .INPUT_STREAM_TLAST(INPUT_STREAM_TLAST),
    .INPUT_STREAM_TID(INPUT_STREAM_TID),
    .OUTPUT_STREAM_TVALID(OUTPUT_STREAM_TVALID),
    .OUTPUT_STREAM_TREADY(OUTPUT_STREAM_TREADY),
    .OUTPUT_STREAM_TDATA(OUTPUT_STREAM_TDATA),
    .OUTPUT_STREAM_TDEST(OUTPUT_STREAM_TDEST),
    .OUTPUT_STREAM_TKEEP(OUTPUT_STREAM_TKEEP),
    .OUTPUT_STREAM_TSTRB(OUTPUT_STREAM_TSTRB),
    .OUTPUT_STREAM_TUSER(OUTPUT_STREAM_TUSER),
    .OUTPUT_STREAM_TLAST(OUTPUT_STREAM_TLAST),
    .OUTPUT_STREAM_TID(OUTPUT_STREAM_TID)
  );
endmodule
