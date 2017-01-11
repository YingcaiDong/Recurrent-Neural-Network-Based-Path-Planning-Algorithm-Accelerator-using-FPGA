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

// The following must be inserted into your Verilog file for this
// core to be instantiated. Change the instance name and port connections
// (in parentheses) to your own signal names.

//----------- Begin Cut here for INSTANTIATION Template ---// INST_TAG
system_HLS_accel_0_0 your_instance_name (
  .s_axi_CONTROL_BUS_AWADDR(s_axi_CONTROL_BUS_AWADDR),    // input wire [4 : 0] s_axi_CONTROL_BUS_AWADDR
  .s_axi_CONTROL_BUS_AWVALID(s_axi_CONTROL_BUS_AWVALID),  // input wire s_axi_CONTROL_BUS_AWVALID
  .s_axi_CONTROL_BUS_AWREADY(s_axi_CONTROL_BUS_AWREADY),  // output wire s_axi_CONTROL_BUS_AWREADY
  .s_axi_CONTROL_BUS_WDATA(s_axi_CONTROL_BUS_WDATA),      // input wire [31 : 0] s_axi_CONTROL_BUS_WDATA
  .s_axi_CONTROL_BUS_WSTRB(s_axi_CONTROL_BUS_WSTRB),      // input wire [3 : 0] s_axi_CONTROL_BUS_WSTRB
  .s_axi_CONTROL_BUS_WVALID(s_axi_CONTROL_BUS_WVALID),    // input wire s_axi_CONTROL_BUS_WVALID
  .s_axi_CONTROL_BUS_WREADY(s_axi_CONTROL_BUS_WREADY),    // output wire s_axi_CONTROL_BUS_WREADY
  .s_axi_CONTROL_BUS_BRESP(s_axi_CONTROL_BUS_BRESP),      // output wire [1 : 0] s_axi_CONTROL_BUS_BRESP
  .s_axi_CONTROL_BUS_BVALID(s_axi_CONTROL_BUS_BVALID),    // output wire s_axi_CONTROL_BUS_BVALID
  .s_axi_CONTROL_BUS_BREADY(s_axi_CONTROL_BUS_BREADY),    // input wire s_axi_CONTROL_BUS_BREADY
  .s_axi_CONTROL_BUS_ARADDR(s_axi_CONTROL_BUS_ARADDR),    // input wire [4 : 0] s_axi_CONTROL_BUS_ARADDR
  .s_axi_CONTROL_BUS_ARVALID(s_axi_CONTROL_BUS_ARVALID),  // input wire s_axi_CONTROL_BUS_ARVALID
  .s_axi_CONTROL_BUS_ARREADY(s_axi_CONTROL_BUS_ARREADY),  // output wire s_axi_CONTROL_BUS_ARREADY
  .s_axi_CONTROL_BUS_RDATA(s_axi_CONTROL_BUS_RDATA),      // output wire [31 : 0] s_axi_CONTROL_BUS_RDATA
  .s_axi_CONTROL_BUS_RRESP(s_axi_CONTROL_BUS_RRESP),      // output wire [1 : 0] s_axi_CONTROL_BUS_RRESP
  .s_axi_CONTROL_BUS_RVALID(s_axi_CONTROL_BUS_RVALID),    // output wire s_axi_CONTROL_BUS_RVALID
  .s_axi_CONTROL_BUS_RREADY(s_axi_CONTROL_BUS_RREADY),    // input wire s_axi_CONTROL_BUS_RREADY
  .ap_clk(ap_clk),                                        // input wire ap_clk
  .ap_rst_n(ap_rst_n),                                    // input wire ap_rst_n
  .interrupt(interrupt),                                  // output wire interrupt
  .INPUT_STREAM_TVALID(INPUT_STREAM_TVALID),              // input wire INPUT_STREAM_TVALID
  .INPUT_STREAM_TREADY(INPUT_STREAM_TREADY),              // output wire INPUT_STREAM_TREADY
  .INPUT_STREAM_TDATA(INPUT_STREAM_TDATA),                // input wire [31 : 0] INPUT_STREAM_TDATA
  .INPUT_STREAM_TDEST(INPUT_STREAM_TDEST),                // input wire [4 : 0] INPUT_STREAM_TDEST
  .INPUT_STREAM_TKEEP(INPUT_STREAM_TKEEP),                // input wire [3 : 0] INPUT_STREAM_TKEEP
  .INPUT_STREAM_TSTRB(INPUT_STREAM_TSTRB),                // input wire [3 : 0] INPUT_STREAM_TSTRB
  .INPUT_STREAM_TUSER(INPUT_STREAM_TUSER),                // input wire [3 : 0] INPUT_STREAM_TUSER
  .INPUT_STREAM_TLAST(INPUT_STREAM_TLAST),                // input wire [0 : 0] INPUT_STREAM_TLAST
  .INPUT_STREAM_TID(INPUT_STREAM_TID),                    // input wire [4 : 0] INPUT_STREAM_TID
  .OUTPUT_STREAM_TVALID(OUTPUT_STREAM_TVALID),            // output wire OUTPUT_STREAM_TVALID
  .OUTPUT_STREAM_TREADY(OUTPUT_STREAM_TREADY),            // input wire OUTPUT_STREAM_TREADY
  .OUTPUT_STREAM_TDATA(OUTPUT_STREAM_TDATA),              // output wire [31 : 0] OUTPUT_STREAM_TDATA
  .OUTPUT_STREAM_TDEST(OUTPUT_STREAM_TDEST),              // output wire [4 : 0] OUTPUT_STREAM_TDEST
  .OUTPUT_STREAM_TKEEP(OUTPUT_STREAM_TKEEP),              // output wire [3 : 0] OUTPUT_STREAM_TKEEP
  .OUTPUT_STREAM_TSTRB(OUTPUT_STREAM_TSTRB),              // output wire [3 : 0] OUTPUT_STREAM_TSTRB
  .OUTPUT_STREAM_TUSER(OUTPUT_STREAM_TUSER),              // output wire [3 : 0] OUTPUT_STREAM_TUSER
  .OUTPUT_STREAM_TLAST(OUTPUT_STREAM_TLAST),              // output wire [0 : 0] OUTPUT_STREAM_TLAST
  .OUTPUT_STREAM_TID(OUTPUT_STREAM_TID)                  // output wire [4 : 0] OUTPUT_STREAM_TID
);
// INST_TAG_END ------ End INSTANTIATION Template ---------

// You must compile the wrapper file system_HLS_accel_0_0.v when simulating
// the core, system_HLS_accel_0_0. When compiling the wrapper file, be sure to
// reference the Verilog simulation library.

