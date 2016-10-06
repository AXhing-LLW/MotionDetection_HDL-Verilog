module DE2_70 (
  //////////////////////// Clock Input ////////////////////////
  input          iCLK_28,           // 28.63636 MHz
  input          iCLK_50,           // 50 MHz
  input          iCLK_50_2,         // 50 MHz
  input          iCLK_50_3,         // 50 MHz
  input          iCLK_50_4,         // 50 MHz
  input          iEXT_CLOCK,        // External Clock
  //////////////////////// Push Button ////////////////////////
  input  [3:0]   iKEY,              // Pushbutton[3:0]
  //////////////////////// DPDT Switch ////////////////////////
  input  [17:0]  iSW,               // Toggle Switch[17:0]
  //////////////////////// 7-SEG Dispaly ////////////////////////
  output [6:0]   oHEX0_D,           // Seven Segment Digit 0
  output         oHEX0_DP,          // Seven Segment Digit 0 decimal point
  output [6:0]   oHEX1_D,           // Seven Segment Digit 1
  output         oHEX1_DP,          // Seven Segment Digit 1 decimal point
  output [6:0]   oHEX2_D,           // Seven Segment Digit 2
  output         oHEX2_DP,          // Seven Segment Digit 2 decimal point
  output [6:0]   oHEX3_D,           // Seven Segment Digit 3
  output         oHEX3_DP,          // Seven Segment Digit 3 decimal point
  output [6:0]   oHEX4_D,           // Seven Segment Digit 4
  output         oHEX4_DP,          // Seven Segment Digit 4 decimal point
  output [6:0]   oHEX5_D,           // Seven Segment Digit 5
  output         oHEX5_DP,          // Seven Segment Digit 5 decimal point
  output [6:0]   oHEX6_D,           // Seven Segment Digit 6
  output         oHEX6_DP,          // Seven Segment Digit 6 decimal point
  output [6:0]   oHEX7_D,           // Seven Segment Digit 7
  output         oHEX7_DP,          // Seven Segment Digit 7 decimal point
  //////////////////////////// LED ////////////////////////////
  output [8:0]   oLEDG,             // LED Green[8:0]
  output [17:0]  oLEDR,             // LED Red[17:0]
  //////////////////////////// UART ////////////////////////////
  output         oUART_TXD,         // UART Transmitter
  input          iUART_RXD,         // UART Receiver
  output         oUART_CTS,         // UART Clear To Send
  input          iUART_RTS,         // UART Requst To Send
  //////////////////////////// IRDA ////////////////////////////
  output         oIRDA_TXD,         // IRDA Transmitter
  input          iIRDA_RXD,         // IRDA Receiver
  /////////////////////// SDRAM Interface ////////////////////////
  inout   [31:0] DRAM_DQ,           // SDRAM Data bus 32 Bits
  output  [12:0] oDRAM0_A,          // SDRAM0 Address bus 13 Bits
  output  [12:0] oDRAM1_A,          // SDRAM1 Address bus 13 Bits
  output         oDRAM0_LDQM0,      // SDRAM0 Low-byte Data Mask 
  output         oDRAM1_LDQM0,      // SDRAM1 Low-byte Data Mask 
  output         oDRAM0_UDQM1,      // SDRAM0 High-byte Data Mask
  output         oDRAM1_UDQM1,      // SDRAM1 High-byte Data Mask
  output         oDRAM0_WE_N,       // SDRAM0 Write Enable
  output         oDRAM1_WE_N,       // SDRAM1 Write Enable
  output         oDRAM0_CAS_N,      // SDRAM0 Column Address Strobe
  output         oDRAM1_CAS_N,      // SDRAM1 Column Address Strobe
  output         oDRAM0_RAS_N,      // SDRAM0 Row Address Strobe
  output         oDRAM1_RAS_N,      // SDRAM1 Row Address Strobe
  output         oDRAM0_CS_N,       // SDRAM0 Chip Select
  output         oDRAM1_CS_N,       // SDRAM1 Chip Select
  output  [1:0]  oDRAM0_BA,         // SDRAM0 Bank Address
  output  [1:0]  oDRAM1_BA,         // SDRAM1 Bank Address
  output         oDRAM0_CLK,        // SDRAM0 Clock
  output         oDRAM1_CLK,        // SDRAM1 Clock
  output         oDRAM0_CKE,        // SDRAM0 Clock Enable
  output         oDRAM1_CKE,        // SDRAM1 Clock Enable
  //////////////////////// Flash Interface ////////////////////////
  inout   [14:0] FLASH_DQ,          // FLASH Data bus 15 Bits (0 to 14)
  inout          FLASH_DQ15_AM1,    // FLASH Data bus Bit 15 or Address A-1
  output  [21:0] oFLASH_A,          // FLASH Address bus 26 Bits
  output         oFLASH_WE_N,       // FLASH Write Enable
  output         oFLASH_RST_N,      // FLASH Reset
  output         oFLASH_WP_N,       // FLASH Write Protect /Programming Acceleration 
  input          iFLASH_RY_N,       // FLASH Ready/Busy output 
  output         oFLASH_BYTE_N,     // FLASH Byte/Word Mode Configuration
  output         oFLASH_OE_N,       // FLASH Output Enable
  output         oFLASH_CE_N,       // FLASH Chip Enable
  //////////////////////// SRAM Interface ////////////////////////
  inout   [31:0] SRAM_DQ,           // SRAM Data Bus 32 Bits
  inout   [3:0]  SRAM_DPA,          // SRAM Parity Data Bus
  output  [18:0] oSRAM_A,           // SRAM Address bus 21 Bits
  output         oSRAM_ADSC_N,      // SRAM Controller Address Status 	
  output         oSRAM_ADSP_N,      // SRAM Processor Address Status
  output         oSRAM_ADV_N,       // SRAM Burst Address Advance
  output  [3:0]  oSRAM_BE_N,        // SRAM Byte Write Enable
  output         oSRAM_CE1_N,       // SRAM Chip Enable
  output         oSRAM_CE2,         // SRAM Chip Enable
  output         oSRAM_CE3_N,       // SRAM Chip Enable
  output         oSRAM_CLK,         // SRAM Clock
  output         oSRAM_GW_N,        // SRAM Global Write Enable
  output         oSRAM_OE_N,        // SRAM Output Enable
  output         oSRAM_WE_N,        // SRAM Write Enable
  //////////////////// ISP1362 Interface ////////////////////////
  inout   [15:0] OTG_D,             // ISP1362 Data bus 16 Bits
  output  [1:0]  oOTG_A,            // ISP1362 Address 2 Bits
  output         oOTG_CS_N,         // ISP1362 Chip Select
  output         oOTG_OE_N,         // ISP1362 Read
  output         oOTG_WE_N,         // ISP1362 Write
  output         oOTG_RESET_N,      // ISP1362 Reset
  inout          OTG_FSPEED,        // USB Full Speed,	0 = Enable, Z = Disable
  inout          OTG_LSPEED,        // USB Low Speed, 	0 = Enable, Z = Disable
  input          iOTG_INT0,         // ISP1362 Interrupt 0
  input          iOTG_INT1,         // ISP1362 Interrupt 1
  input          iOTG_DREQ0,        // ISP1362 DMA Request 0
  input          iOTG_DREQ1,        // ISP1362 DMA Request 1
  output         oOTG_DACK0_N,      // ISP1362 DMA Acknowledge 0
  output         oOTG_DACK1_N,      // ISP1362 DMA Acknowledge 1
  //////////////////// LCD Module 16X2 ////////////////////////////
  inout   [7:0]  LCD_D,             // LCD Data bus 8 bits
  output         oLCD_ON,           // LCD Power ON/OFF
  output         oLCD_BLON,         // LCD Back Light ON/OFF
  output         oLCD_RW,           // LCD Read/Write Select, 0 = Write, 1 = Read
  output         oLCD_EN,           // LCD Enable
  output         oLCD_RS,           // LCD Command/Data Select, 0 = Command, 1 = Data
  //////////////////// SD Card Interface ////////////////////////
  inout          SD_DAT,            // SD Card Data
  inout          SD_DAT3,           // SD Card Data 3
  inout          SD_CMD,            // SD Card Command Signal
  output         oSD_CLK,           // SD Card Clock
  //////////////////////// I2C ////////////////////////////////
  inout          I2C_SDAT,          // I2C Data
  output         oI2C_SCLK,         // I2C Clock
  //////////////////////// PS2 ////////////////////////////////
  inout          PS2_KBDAT,         // PS2 Keyboard Data
  inout          PS2_KBCLK,         // PS2 Keyboard Clock
  inout          PS2_MSDAT,         // PS2 Mouse Data
  inout          PS2_MSCLK,         // PS2 Mouse Clock
  //////////////////////// VGA ////////////////////////////
  output         oVGA_CLOCK,        // VGA Clock
  output         oVGA_HS,           // VGA H_SYNC
  output         oVGA_VS,           // VGA V_SYNC
  output         oVGA_BLANK_N,      // VGA BLANK
  output         oVGA_SYNC_N,       // VGA SYNC
  output  [9:0]  oVGA_R,            // VGA Red[9:0]
  output  [9:0]  oVGA_G,            // VGA Green[9:0]
  output  [9:0]  oVGA_B,            // VGA Blue[9:0]
  //////////////// Ethernet Interface ////////////////////////////
  inout   [15:0] ENET_D,            // DM9000A DATA bus 16Bits
  output         oENET_CMD,         // DM9000A Command/Data Select, 0 = Command, 1 = Data
  output         oENET_CS_N,        // DM9000A Chip Select
  output         oENET_IOW_N,       // DM9000A Write
  output         oENET_IOR_N,       // DM9000A Read
  output         oENET_RESET_N,     // DM9000A Reset
  input          iENET_INT,         // DM9000A Interrupt
  output         oENET_CLK,         // DM9000A Clock 25 MHz
  //////////////////// Audio CODEC  ////////////////////////////
  inout          AUD_ADCLRCK,       // Audio CODEC ADC LR Clock
  input          iAUD_ADCDAT,       // Audio CODEC ADC Data
  inout          AUD_DACLRCK,       // Audio CODEC DAC LR Clock
  output         oAUD_DACDAT,       // Audio CODEC DAC Data
  inout          AUD_BCLK,          // Audio CODEC Bit-Stream Clock
  output         oAUD_XCK,          // Audio CODEC Chip Clock
  //////////////////// TV Devoder   ////////////////////////////
  input          iTD1_CLK27,        // TV Decoder1 Line_Lock Output Clock 
  input   [7:0]  iTD1_D,            // TV Decoder1 Data bus 8 bits
  input          iTD1_HS,           // TV Decoder1 H_SYNC
  input          iTD1_VS,           // TV Decoder1 V_SYNC
  output         oTD1_RESET_N,      // TV Decoder1 Reset
  input          iTD2_CLK27,        // TV Decoder2 Line_Lock Output Clock 		
  input   [7:0]  iTD2_D,            // TV Decoder2 Data bus 8 bits
  input          iTD2_HS,           // TV Decoder2 H_SYNC
  input          iTD2_VS,           // TV Decoder2 V_SYNC
  output         oTD2_RESET_N,      // TV Decoder2 Reset
  //////////////////////// GPIO ////////////////////////////////
  inout   [31:0] GPIO_0,            // GPIO Connection 0 I/O
  input          GPIO_CLKIN_N0,     // GPIO Connection 0 Clock Input 0
  input          GPIO_CLKIN_P0,     // GPIO Connection 0 Clock Input 1
  inout          GPIO_CLKOUT_N0,    // GPIO Connection 0 Clock Output 0
  inout          GPIO_CLKOUT_P0,    // GPIO Connection 0 Clock Output 1
  inout   [31:0] GPIO_1,            // GPIO Connection 1 I/O
  input          GPIO_CLKIN_N1,     // GPIO Connection 1 Clock Input 0
  input          GPIO_CLKIN_P1,     // GPIO Connection 1 Clock Input 1
  inout          GPIO_CLKOUT_N1,    // GPIO Connection 1 Clock Output 0
  inout          GPIO_CLKOUT_P1     // GPIO Connection 1 Clock Output 1
);
 
//  CCD
wire  [11:0]  CCD_DATA;
wire          CCD_SDAT;
wire          CCD_SCLK;
wire          CCD_FLASH;
wire          CCD_FVAL;
wire          CCD_LVAL;
wire          CCD_PIXCLK;
wire          CCD_MCLK; //  CCD Master Clock

wire  [15:0]  Read_DATA1;
wire  [15:0]  Read_DATA2;
wire  [15:0]  Read_DATA3; // add by oomusou for RGB16
wire  [15:0]  Read_DATA4; // add by oomusou for RGB16
wire          VGA_CTRL_CLK;
wire  [11:0]  mCCD_DATA;
wire          mCCD_DVAL;
wire          mCCD_DVAL_d;
wire  [15:0]  X_Cont;
wire  [15:0]  Y_Cont;
wire  [9:0]   X_ADDR;
wire  [31:0]  Frame_Cont;
wire          DLY_RST_0;
wire          DLY_RST_1;
wire          DLY_RST_2;
wire          Read;
reg   [11:0]  rCCD_DATA;
reg           rCCD_LVAL;
reg           rCCD_FVAL;
wire  [11:0]  sCCD_R;
wire  [11:0]  sCCD_G;
wire  [11:0]  sCCD_B;
wire          sCCD_DVAL;
reg   [1:0]   rClk;
wire          sdram_ctrl_clk;

// Touch panel signal
wire  [7:0] ltm_r;    //  LTM Red Data 8 Bits
wire  [7:0] ltm_g;    //  LTM Green Data 8 Bits
wire  [7:0] ltm_b;    //  LTM Blue Data 8 Bits
wire        ltm_nclk; //  LTM Clcok
wire        ltm_hd;
wire        ltm_vd;
wire        ltm_den;
wire        adc_dclk;
wire        adc_cs;
wire        adc_penirq_n;
wire        adc_busy;
wire        adc_din;
wire        adc_dout;
wire        adc_ltm_sclk;
wire        ltm_grst;

// LTM Config
wire      ltm_sclk;
wire      ltm_sda;
wire      ltm_scen;
wire      ltm_3wirebusy_n;

assign  CCD_DATA[0]     = GPIO_1[11];
assign  CCD_DATA[1]     = GPIO_1[10];
assign  CCD_DATA[2]     = GPIO_1[9];
assign  CCD_DATA[3]     = GPIO_1[8];
assign  CCD_DATA[4]     = GPIO_1[7];
assign  CCD_DATA[5]     = GPIO_1[6];
assign  CCD_DATA[6]     = GPIO_1[5];
assign  CCD_DATA[7]     = GPIO_1[4];
assign  CCD_DATA[8]     = GPIO_1[3];
assign  CCD_DATA[9]     = GPIO_1[2];
assign  CCD_DATA[10]    = GPIO_1[1];
assign  CCD_DATA[11]    = GPIO_1[0];
assign  GPIO_CLKOUT_N1  = CCD_MCLK;
assign  CCD_FVAL        = GPIO_1[18];
assign  CCD_LVAL        = GPIO_1[17];
assign  CCD_PIXCLK      = GPIO_CLKIN_N1;
//assign  GPIO_1[15]      = 1'b1;  // tRIGGER
//assign  GPIO_1[14]      = DLY_RST_1;

assign  oLEDR           = iSW;
assign  oLEDG           = Y_Cont;

assign  oTD1_RESET_N    = 1'b1;
assign  oVGA_CLOCK      = ~VGA_CTRL_CLK;

assign  CCD_MCLK        = rClk[0];
assign  oUART_TXD       = iUART_RXD;

assign  adc_penirq_n    = GPIO_CLKIN_N0;
assign  adc_dout        = GPIO_0[0];
assign  adc_busy        = GPIO_CLKIN_P0;
assign  GPIO_0[1]       = adc_din;
assign  GPIO_0[2]       = adc_ltm_sclk;
assign  GPIO_0[3]       = ltm_b[3];
assign  GPIO_0[4]       = ltm_b[2];
assign  GPIO_0[5]       = ltm_b[1];
assign  GPIO_0[6]       = ltm_b[0];
assign  GPIO_0[7]       =~ltm_nclk;
assign  GPIO_0[8]       = ltm_den;
assign  GPIO_0[9]       = ltm_hd;
assign  GPIO_0[10]      = ltm_vd;
assign  GPIO_0[11]      = ltm_b[4];
assign  GPIO_0[12]      = ltm_b[5];
assign  GPIO_0[13]      = ltm_b[6];
assign  GPIO_CLKOUT_N0  = ltm_b[7];
assign  GPIO_0[14]      = ltm_g[0];
assign  GPIO_CLKOUT_P0  = ltm_g[1];
assign  GPIO_0[15]      = ltm_g[2];
assign  GPIO_0[16]      = ltm_g[3];
assign  GPIO_0[17]      = ltm_g[4];
assign  GPIO_0[18]      = ltm_g[5];
assign  GPIO_0[19]      = ltm_g[6];
assign  GPIO_0[20]      = ltm_g[7];
assign  GPIO_0[21]      = ltm_r[0];
assign  GPIO_0[22]      = ltm_r[1];
assign  GPIO_0[23]      = ltm_r[2];
assign  GPIO_0[24]      = ltm_r[3];
assign  GPIO_0[25]      = ltm_r[4];
assign  GPIO_0[26]      = ltm_r[5];
assign  GPIO_0[27]      = ltm_r[6];
assign  GPIO_0[28]      = ltm_r[7];
assign  GPIO_0[29]      = ltm_grst;
assign  GPIO_0[30]      = ltm_scen;
assign  GPIO_0[31]      = ltm_sda;

assign ltm_grst         = iKEY[0];
assign adc_ltm_sclk     = ltm_sclk;

always@(posedge iCLK_50)
  rClk	<=  rClk  + 1;

always@(posedge CCD_PIXCLK) begin
  rCCD_DATA <=  CCD_DATA;
  rCCD_LVAL <=  CCD_LVAL;
  rCCD_FVAL <=  CCD_FVAL;
end
 
 VGA_Controller        u1    (    //    Host Side
                             .oRequest(Read),
                             .iRed(mVGA_R),
                             .iGreen(mVGA_G),
                             .iBlue(mVGA_B),
                             .oCoord_X(mVGA_X),
                             .oCoord_Y(mVGA_Y),
                             //    VGA Side
                             .oVGA_R(VGA_R),
                             .oVGA_G(VGA_G),
                             .oVGA_B(VGA_B),
                             .oVGA_H_SYNC(VGA_HS),
                             .oVGA_V_SYNC(VGA_VS),
                             .oVGA_SYNC(VGA_SYNC),
                             .oVGA_BLANK(VGA_BLANK),
                             //    Control Signal
                             .iCLK(VGA_CTRL_CLK),
                             .iRST_N(DLY_RST_2)    );
 
Reset_Delay reset0  (
  .iCLK(iCLK_50),
  .iRST(iKEY[0]),
  .oRST_0(DLY_RST_0),
  .oRST_1(DLY_RST_1),
  .oRST_2(DLY_RST_2)
);
 
 CCD_Capture capture0 (
  .oDATA(mCCD_DATA),
  .oDVAL(mCCD_DVAL),
  .oX_Cont(X_Cont),
  .oY_Cont(Y_Cont),
  .oFrame_Cont(Frame_Cont),
  .iDATA(rCCD_DATA),
  .iFVAL(rCCD_FVAL),
  .iLVAL(rCCD_LVAL),
  .iSTART(!iKEY[3]),
  .iEND(!iKEY[2]),
  .iCLK(CCD_PIXCLK),
  .iRST(DLY_RST_2)
);
 
 RAW2RGB                u4    (    .oRed(mCCD_R),
                             .oGreen(mCCD_G),
                             .oBlue(mCCD_B),
                             .oDVAL(mCCD_DVAL_d),
                             .iX_Cont(X_Cont),
                             .iY_Cont(Y_Cont),
                             .iDATA(mCCD_DATA),
                             .iDVAL(mCCD_DVAL),
                             .iCLK(CCD_PIXCLK),
                             .iRST(DLY_RST_1)    );
 
 SEG7_LUT_8  seg0 (
  .oSEG0(oHEX0_D),
  .oSEG1(oHEX1_D),
  .oSEG2(oHEX2_D),
  .oSEG3(oHEX3_D),
  .oSEG4(oHEX4_D),
  .oSEG5(oHEX5_D),
  .oSEG6(oHEX6_D),
  .oSEG7(oHEX7_D),
  .iDIG(Frame_Cont[31:0])
);
 
Sdram_Control_4Port sdram0 (
  //  HOST Side
  .REF_CLK(iCLK_50),
  .RESET_N(1'b1),
  .CLK(sdram_ctrl_clk),
  //  FIFO Write Side 1
  .WR1_DATA({sCCD_R[11:7], sCCD_G[11:6],  sCCD_B[11:7]}),
  .WR1(sCCD_DVAL),
  .WR1_ADDR(0),
  .WR1_MAX_ADDR(800*480),
  .WR1_LENGTH(9'h100),
  .WR1_LOAD(!DLY_RST_0),
  .WR1_CLK(CCD_PIXCLK),
  //  FIFO Write Side 1
  .WR2_DATA({sCCD_R[11:7], sCCD_G[11:6],  sCCD_B[11:7]}),
  .WR2(sCCD_DVAL),
  .WR2_ADDR(22'h100000),
  .WR2_MAX_ADDR(22'h100000 + 800*480),
  .WR2_LENGTH(9'h100),
  .WR2_LOAD(!DLY_RST_0),
  .WR2_CLK(CCD_PIXCLK),
  //  FIFO Read Side 1 to LTM
  .RD1_DATA(Read_DATA3),
  .RD1(Read),
  .RD1_ADDR(0),
  .RD1_MAX_ADDR(800*480),
  .RD1_LENGTH(9'h100),
  .RD1_LOAD(!DLY_RST_0),
  .RD1_CLK(~ltm_nclk),
  //  FIFO Read Side 1
  .RD2_DATA(Read_DATA4),
  .RD2(Read),
  .RD2_ADDR(22'h100000 + 800*480 + 200),
  .RD2_MAX_ADDR(22'h100000 + 800*480 + 300),
  .RD2_LENGTH(9'h100),
  .RD2_LOAD(!DLY_RST_0),
  .RD2_CLK(~ltm_nclk),
  //  SDRAM Side
  .SA(oDRAM0_A[11:0]),
  .BA(oDRAM0_BA),
  .CS_N(oDRAM0_CS_N),
  .CKE(oDRAM0_CKE),
  .RAS_N(oDRAM0_RAS_N),
  .CAS_N(oDRAM0_CAS_N),
  .WE_N(oDRAM0_WE_N),
  .DQ(DRAM_DQ[15:0]),
  .DQM({oDRAM0_UDQM1,oDRAM0_LDQM0})
);

// add by oomusou for RGB16
assign Read_DATA1 = {Read_DATA3[10:6], Read_DATA3[4:0], 5'h00};
assign Read_DATA2 = {Read_DATA3[5:5], 4'h0, Read_DATA3[15:11], 5'h00};
 
I2C_CCD_Config ccd_config0 (
  //  Host Side
  .iCLK(iCLK_50),
  .iRST_N(DLY_RST_1),
  .iEXPOSURE_ADJ(iKEY[1]),
  .iEXPOSURE_DEC_p(iSW[0]),
  .iMIRROR_SW(iSW[17]),
  //  I2C Side
  .I2C_SCLK(GPIO_1[20]),
  .I2C_SDAT(GPIO_1[19])
);
                             
 I2C_AV_Config         u8    (    //    Host Side
                             .iCLK(iCLK_50),
                             .iRST_N(iKEY[0]),
                             //    I2C Side
                             .I2C_SCLK(I2C_SCLK),
                             .I2C_SDAT(I2C_SDAT)    );
 
 AUDIO_DAC             u9    (    //    Audio Side
                             .oAUD_BCK(AUD_BCLK),
                             .oAUD_DATA(AUD_DACDAT),
                             .oAUD_LRCK(AUD_DACLRCK),
                             //    Control Signals
                             .iSrc_Select(~(SP_cont[21]&SP)),
                             .iCLK_18_4(AUD_CTRL_CLK),
                             .iRST_N(DLY_RST_1)    );
 
 Audio_PLL             u10    (    .inclk0(iCLK_28),.c0(AUD_CTRL_CLK)    );
  
 //======================    motion detect    ======================//
 wire    [10:0]    mTap_0;
 reg        [10:0]    mTap_1,mTap_2,mTap_3,
                 mTap_4,mTap_5,mTap_6,
                 mTap_7,mTap_8,mTap_9,mTap_10;
 wire    [10:0]    rTap_0;
 reg        [10:0]    rTap_1,rTap_2,rTap_3,
                 rTap_4,rTap_5,rTap_6,
                 rTap_7,rTap_8,rTap_9,rTap_10;
 wire    [10:0]    sTap_0;
 reg        [10:0]    sTap_1,sTap_2,sTap_3,
                 sTap_4,sTap_5,sTap_6,
                 sTap_7,sTap_8,sTap_9,sTap_10;
 reg                X,Y,Z;
 reg                F1,F2;
 reg        [5:0]    Read_d;
 
 always@(posedge VGA_CTRL_CLK)
 begin
     //---------------    binary    -------------------//    
     F1    <=    (    Read_DATA1[14:10] + Read_DATA1[9:5] + Read_DATA1[4:0] )    >48;
     F2    <=    (    Read_DATA2[14:10] + Read_DATA2[9:5] + Read_DATA2[4:0] )    >48;    
     //---------------------------------------------//
     mTap_1    <=    mTap_0;
     mTap_2    <=    mTap_1;
     mTap_3    <=    mTap_2;
     mTap_4    <=    mTap_3;
     mTap_5    <=    mTap_4;
     mTap_6    <=    mTap_5;
     mTap_7    <=    mTap_6;
     mTap_8    <=    mTap_7;
     mTap_9    <=    mTap_8;
     mTap_10    <=    mTap_9;
     //---------------    erode    -------------------//
     X        <=    (&mTap_0) & (&mTap_1) & (&mTap_2) &
                 (&mTap_3) & (&mTap_4) & (&mTap_5) &
                 (&mTap_6) & (&mTap_7) & (&mTap_8) &
                 (&mTap_9) & (&mTap_10);
     //---------------------------------------------//
     rTap_1    <=    rTap_0;
     rTap_2    <=    rTap_1;
     rTap_3    <=    rTap_2;
     rTap_4    <=    rTap_3;
     rTap_5    <=    rTap_4;
     rTap_6    <=    rTap_5;
     rTap_7    <=    rTap_6;
     rTap_8    <=    rTap_7;
     rTap_9    <=    rTap_8;
     rTap_10    <=    rTap_9;
     //---------------    dilate    -------------------//
     Y        <=    (|rTap_0) | (|rTap_1) | (|rTap_2) |
                 (|rTap_3) | (|rTap_4) | (|rTap_5) |
                 (|rTap_6) | (|rTap_7) | (|rTap_8) |
                 (|rTap_9) | (|rTap_10);
     //---------------------------------------------//
     sTap_1    <=    sTap_0;
     sTap_2    <=    sTap_1;
     sTap_3    <=    sTap_2;
     sTap_4    <=    sTap_3;
     sTap_5    <=    sTap_4;
     sTap_6    <=    sTap_5;
     sTap_7    <=    sTap_6;
     sTap_8    <=    sTap_7;
     sTap_9    <=    sTap_8;
     sTap_10    <=    sTap_9;
     //---------------    erode    -------------------//
     Z        <=    (&sTap_0) & (&sTap_1) & (&sTap_2) &
                 (&sTap_3) & (&sTap_4) & (&sTap_5) &
                 (&sTap_6) & (&sTap_7) & (&sTap_8) &
                 (&sTap_9) & (&sTap_10);
     //---------------------------------------------//
     Read_d    <=    {Read_d[4:0],Read};
 end
 //---------------    detect method 1    -------------------//
 Tap_1     u99    (    .clken(Read),
                 .clock(VGA_CTRL_CLK),
                 .shiftin(    (Read_DATA1[14:10] ^ Read_DATA2[14:10]) | 
                             (Read_DATA1[9:5] ^ Read_DATA2[9:5]) |
                             (Read_DATA1[4:0] ^ Read_DATA2[4:0]) ),
                 .taps(mTap_0));
         
 Tap_1     u98    (    .clken(Read_d[5]),
                 .clock(VGA_CTRL_CLK),
                 .shiftin(X),
                 .taps(rTap_0));
 //---------------    detect method 2    -------------------//
 Tap_1     u97    (    .clken(Read_d[0]),
                 .clock(VGA_CTRL_CLK),
                 .shiftin(F1^F2),
                 .taps(sTap_0));
 //==================================================================//
 wire    [9:0]    mVGA_R    =    (    (mVGA_X>=20 && mVGA_X<620) && (mVGA_Y>=20 && mVGA_Y<460)    )?
                             (    Y|Z    ?    1023    :    {Read_DATA1[14:10],5'h00}    ):
                                                     {Read_DATA1[14:10],5'h00}    ;
 wire    [9:0]    mVGA_G    =    {Read_DATA1[9:5],5'h00};
 wire    [9:0]    mVGA_B    =    {Read_DATA1[4:0],5'h00};
 wire    [9:0]    mVGA_X;
 wire    [9:0]    mVGA_Y;
  
 //======================    Speaker Control        ====================//
 reg            SP;
 reg    [21:0]    SP_cont;
 reg    [23:0]    DLY_cont;
 
 always@(posedge iCLK_50)
 begin
     SP_cont    <=    SP_cont+1'b1;
     if(Y|Z)                //    if datected => turn on speaker
     DLY_cont    <=    0;
     else
     begin
         if(DLY_cont<24'hffffff)        //    20 * 2^24 ns
         begin
             DLY_cont    <=    DLY_cont+1;
             SP            <=    1;
         end
         else
         SP            <=    0;        
     end
 end
 //==================================================================//
 
 /*wire    [9:0]    sCCD_R;
 wire    [9:0]    sCCD_G;
 wire    [9:0]    sCCD_B;
 wire            sCCD_DVAL;*/
 
 Mirror_Col u11    (    //    Input Side
   .iCCD_R(mCCD_R),
   .iCCD_G(mCCD_G),
   .iCCD_B(mCCD_B),
   .iCCD_DVAL(mCCD_DVAL_d),
   .iCCD_PIXCLK(CCD_PIXCLK),
   .iRST_N(DLY_RST_1),
   //    Output Side
   .oCCD_R(sCCD_R),
   .oCCD_G(sCCD_G),
   .oCCD_B(sCCD_B),
   .oCCD_DVAL(sCCD_DVAL)
 );
 endmodule         