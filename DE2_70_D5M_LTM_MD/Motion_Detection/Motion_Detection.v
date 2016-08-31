module Motion_Detection (
	input [15:0] Read_DATA1,
	input [15:0] Read_DATA2,
	input LCD_CTRL_CLK,
	input iCLK_50,
	input Read,
	input iRST_N,
	
	output reg oDVAL,
	output reg[15:0] oDATA_1,
	output reg[15:0] oDATA_2
);

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
 
 always@(posedge LCD_CTRL_CLK)
 begin
     //---------------    binary    -------------------//    
     F1    <=    (    Read_DATA1[15:11] + Read_DATA1[10:6] + Read_DATA1[5:1] )    >48;//Replace Read_DATA1[14:10] + Read_DATA2[9:5] + Read_DATA2[4:0]
     F2    <=    (    Read_DATA2[15:11] + Read_DATA2[10:6] + Read_DATA2[5:1] )    >48;//Replace Read_DATA2[14:10] + Read_DATA2[9:5] + Read_DATA2[4:0]    
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
                 .clock(LCD_CTRL_CLK),
					  //Replace (Read_DATA1[14:10] ^ Read_DATA2[14:10]) | Read_DATA1[9:5] ^ Read_DATA2[9:5]) | (Read_DATA1[4:0] ^ Read_DATA2[4:0])
                 .shiftin(    (Read_DATA1[15:11] ^ Read_DATA2[15:11]) | 
                             (Read_DATA1[10:6] ^ Read_DATA2[10:6]) |
                             (Read_DATA1[5:1] ^ Read_DATA2[5:1]) ),
                 .taps(mTap_0));
         
 Tap_1     u98    (    .clken(Read_d[5]),
                 .clock(LCD_CTRL_CLK),
                 .shiftin(X),
                 .taps(rTap_0));
 //---------------    detect method 2    -------------------//
 Tap_1     u97    (    .clken(Read_d[0]),
                 .clock(LCD_CTRL_CLK),
                 .shiftin(F1^F2),
                 .taps(sTap_0));
 //==================================================================//
 
 //Replace ( (mVGA_X>=20 && mVGA_X<620) && (mVGA_Y>=20 && mVGA_Y<460) ) ? ( Y|Z ? 1023 : {Read_DATA1[14:10],5'h00} ) : {Read_DATA1[14:10],5'h00};
 wire    [9:0]    mVGA_R    =    (    (mVGA_X>=20 && mVGA_X<620) && (mVGA_Y>=20 && mVGA_Y<460)    )?
                             (    Y|Z    ?    1023    :    {Read_DATA1[15:11],5'h00}    ):
                                                     {Read_DATA1[15:11],5'h00}    ;
 
 //Replace {Read_DATA1[9:5],5'h00};
 wire    [9:0]    mVGA_G    =    {Read_DATA1[10:6],5'h00};
 
 //Replace {Read_DATA1[4:0],5'h00};
 wire    [9:0]    mVGA_B    =    {Read_DATA1[5:1],5'h00};
 wire    [9:0]    mVGA_X;
 wire    [9:0]    mVGA_Y;
  
/*
  //======================    Speaker Control        ====================//
 reg            SP;
 reg    [21:0]    SP_cont;
 reg    [23:0]    DLY_cont;
 
 always@(posedge CLOCK_50)
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
 */
 
 always@(posedge LCD_CTRL_CLK)begin
	oDATA_1[15:0]<=rTap_0;
	oDATA_2[15:0]<=sTap_0;
 end
 
always@(posedge LCD_CTRL_CLK, negedge iRST_N) begin
  if (!iRST_N) begin
    oDVAL <= 1'b0;
    //oDATA <= 10'h0;
  end
  else begin
    oDVAL <= Read;
    
    //if (iDVAL)
      //oDATA <= (iDATA > iTHRESHOLD) ? 1023 : 0;
    //else
      //oDATA <= 10'b0;
  end
end
 
endmodule
