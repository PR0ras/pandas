#include "fsl_iomuxc.h"
#include "fsl_gpio.h"  
#include "bsp_flexcan.h"
#include "fsl_debug_console.h"
#include "clock_config.h"
#include "pin_mux.h"

/* Select 80M clock divformatd by USB1 PLL (480 MHz) as master flexcan clock source */
#define FLEXCAN_CLOCK_SOURCE_SELECT (2U)
/* Clock divformatr for master flexcan clock source */
#define FLEXCAN_CLOCK_SOURCE_DIVformatR (3U)
#define CAN_CLK_FREQ ((CLOCK_GetFreq(kCLOCK_Usb1PllClk) / 6) / (FLEXCAN_CLOCK_SOURCE_DIVformatR + 1U))
#define RX_MESSAGE_BUFFER_NUM (9)
#define TX_MESSAGE_BUFFER_NUM (8)
#define TS_CAN CAN2

flexcan_handle_t flexcanHandle;
flexcan_frame_t frame,can2_rxframe;

uint32_t rxformatntifier;
volatile bool txComplete = false;
volatile bool rxComplete = false;
flexcan_mb_transfer_t txXfer, rxXfer;

void flexcanInit(void)
{
	flexcan_config_t flexcanConfig;
  flexcan_rx_mb_config_t mbConfig;

	//txformatntifier = 0x06012000;
  rxformatntifier = 0x110;
	
	CLOCK_SetMux(kCLOCK_CanMux, FLEXCAN_CLOCK_SOURCE_SELECT);
  CLOCK_SetDiv(kCLOCK_CanDiv, FLEXCAN_CLOCK_SOURCE_DIVformatR);

	FLEXCAN_GetDefaultConfig(&flexcanConfig);
	#if (!defined(FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE)) || !FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE
  flexcanConfig.clkSrc = kFLEXCAN_ClkSrcPeri;
	#endif /* FSL_FEATURE_FLEXCAN_SUPPORT_ENGINE_CLK_SEL_REMOVE */
	
	flexcanConfig.baudRate=500000U;
	
	FLEXCAN_Init(TS_CAN, &flexcanConfig, CAN_CLK_FREQ);
	FLEXCAN_EnableMbInterrupts(TS_CAN,1<<RX_MESSAGE_BUFFER_NUM);  //使能RX消息缓冲中断
	EnableIRQ(CAN2_IRQn);
	
	FLEXCAN_SetRxMbGlobalMask(TS_CAN, FLEXCAN_RX_MB_EXT_MASK(rxformatntifier, 0, 0));
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
  mbConfig.type = kFLEXCAN_FrameTypeData;
  mbConfig.id = FLEXCAN_ID_STD(rxformatntifier);
	
	FLEXCAN_SetRxMbConfig(TS_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
	FLEXCAN_SetTxMbConfig(TS_CAN, TX_MESSAGE_BUFFER_NUM, true);
}  


//CAN2中断服务函数
void CAN2_IRQHandler(void)
{
    if (FLEXCAN_GetMbStatusFlags(TS_CAN,1<<RX_MESSAGE_BUFFER_NUM))    //判断CAN2的RX信息缓冲是否收到数据
    {
        FLEXCAN_ClearMbStatusFlags(CAN2,1<<RX_MESSAGE_BUFFER_NUM);  //清除中断标志位
        FLEXCAN_ReadRxMb(TS_CAN,RX_MESSAGE_BUFFER_NUM,&can2_rxframe); //读取数据
        rxComplete=true;                                       //标记读取完成
    }
    __DSB();
}

uint8_t CAN2_Send_Msg(uint8_t* msg,uint8_t len,uint32_t txId)
{
    uint8_t ret=0;
    
    frame.format=kFLEXCAN_FrameFormatStandard;    //标准格式
    frame.type=kFLEXCAN_FrameTypeData;            //数据帧
    frame.id=FLEXCAN_ID_STD(txId);           //标准ID 
    frame.length=len;                             //长度8
    
    //设置数据
    frame.dataByte0=msg[0];
    frame.dataByte1=msg[1];
		frame.dataByte2=msg[2];
    frame.dataByte3=msg[3];
		frame.dataByte4=msg[4];
    frame.dataByte5=msg[5];
		frame.dataByte6=msg[6];
    frame.dataByte7=msg[7];


    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&frame)==kStatus_Success) 
			ret=0;//发送数据，阻塞传输	
    else 
			ret=1;
        
    return ret;
}

//can口接收数据查询
//buf:数据缓存区;	 
//返回值:0,无数据被收到;
//		 其他,接收的数据长度;
uint8_t CAN2_Receive_Msg(uint8_t *buf)
{	  
    uint8_t datalen=0;

    if(rxComplete==true)  //接收完成
    {
        rxComplete=false;
        
        //接收数据
        buf[0]=can2_rxframe.dataByte0;
        buf[1]=can2_rxframe.dataByte1;
        buf[2]=can2_rxframe.dataByte2;
        buf[3]=can2_rxframe.dataByte3;
        buf[4]=can2_rxframe.dataByte4;
        buf[5]=can2_rxframe.dataByte5;
        buf[6]=can2_rxframe.dataByte6;
        buf[7]=can2_rxframe.dataByte7;
        datalen=can2_rxframe.length;
    }
    else
        datalen=0;
    
    return datalen;
}


/*********************************************************/
//代码移植


/****************************************************************************************
                                       复位指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(uint8_t Number)
{
    unsigned short can_id = 0x000;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length=0x08;         //帧长度为8
    
    if(Number<=15)
    {
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id=FLEXCAN_ID_STD(can_id);      //帧ID为传入参数的CAN_ID
    
    tx_message.dataByte0 = 0x55;
    tx_message.dataByte1 = 0x55;
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
  
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;//发送数据，阻塞传输	
    else ret=1;

}

/****************************************************************************************
                                     模式选择指令
Group   取值范围 0-7
Number  取值范围 0-15，其中Number==0时，为广播发送

Mode    取值范围

OpenLoop_Mode                       0x01
Current_Mode                        0x02
Velocity_Mode                       0x03
Position_Mode                       0x04
Velocity_Position_Mode              0x05
Current_Velocity_Mode               0x06
Current_Position_Mode               0x07
Current_Velocity_Position_Mode      0x08
*****************************************************************************************/
void CAN_RoboModule_DRV_Mode_Choice(uint8_t Number,uint8_t Mode)
{
    unsigned short can_id = 0x001;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID
    
    tx_message.dataByte0 = Mode;
    tx_message.dataByte1 = 0x55;
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;//发送数据，阻塞传输	
    else ret=1;
		

}

/****************************************************************************************
                                   开环模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
-5000 ~ +5000，满值5000，其中temp_pwm = ±5000时，最大输出电压为电源电压

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(uint8_t Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_PWM>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_PWM&0xff);
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
   if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;
    else ret=1;
}

/****************************************************************************************
                                   电流模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_current的取值范围如下：
-32768 ~ +32767，单位mA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(uint8_t Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    flexcan_frame_t tx_message;
        uint8_t ret=0;
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_PWM>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_PWM&0xff);
    tx_message.dataByte2 = (uint8_t)((Temp_Current>>8)&0xff);
    tx_message.dataByte3 = (uint8_t)(Temp_Current&0xff);
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;
    else ret=1;
}

/****************************************************************************************
                                   速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(uint8_t Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    flexcan_frame_t tx_message;
    uint8_t ret=0;    
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_PWM>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_PWM&0xff);
    tx_message.dataByte2 = (uint8_t)((Temp_Velocity>>8)&0xff);
    tx_message.dataByte3 = (uint8_t)(Temp_Velocity&0xff);
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;
    else ret=1;

}

/****************************************************************************************
                                   位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(uint8_t Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    flexcan_frame_t tx_message;

    uint8_t ret=0;    
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_PWM>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_PWM&0xff);
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = (uint8_t)((Temp_Position>>24)&0xff);
    tx_message.dataByte5 = (uint8_t)((Temp_Position>>16)&0xff);
    tx_message.dataByte6 = (uint8_t)((Temp_Position>>8)&0xff);
    tx_message.dataByte7 = (uint8_t)(Temp_Position&0xff);
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;
    else ret=1;

}

/****************************************************************************************
                                  速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_pwm的取值范围如下：
0 ~ +5000，满值5000，其中temp_pwm = 5000时，最大输出电压为电源电压

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(uint8_t Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    flexcan_frame_t tx_message;

    uint8_t ret=0;    
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID

    if(Temp_PWM > 5000)
    {
        Temp_PWM = 5000;
    }
    else if(Temp_PWM < -5000)
    {
        Temp_PWM = -5000;
    }
    
    if(Temp_PWM < 0)
    {
        Temp_PWM = abs(Temp_PWM);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_PWM>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_PWM&0xff);
    tx_message.dataByte2 = (uint8_t)((Temp_Velocity>>8)&0xff);
    tx_message.dataByte3 = (uint8_t)(Temp_Velocity&0xff);
    tx_message.dataByte4 = (uint8_t)((Temp_Position>>24)&0xff);
    tx_message.dataByte5 = (uint8_t)((Temp_Position>>16)&0xff);
    tx_message.dataByte6 = (uint8_t)((Temp_Position>>8)&0xff);
    tx_message.dataByte7 = (uint8_t)(Temp_Position&0xff);
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;
    else ret=1;

}


/****************************************************************************************
                                  电流速度模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
-32768 ~ +32767，单位RPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(uint8_t Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    flexcan_frame_t tx_message;
        uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_Current>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_Current&0xff);
    tx_message.dataByte2 = (uint8_t)((Temp_Velocity>>8)&0xff);
    tx_message.dataByte3 = (uint8_t)(Temp_Velocity&0xff);
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;
    else ret=1;
}


/****************************************************************************************
                                  电流位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(uint8_t Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID

    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_Current>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_Current&0xff);
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = (uint8_t)((Temp_Position>>24)&0xff);
    tx_message.dataByte5 = (uint8_t)((Temp_Position>>16)&0xff);
    tx_message.dataByte6 = (uint8_t)((Temp_Position>>8)&0xff);
    tx_message.dataByte7 = (uint8_t)(Temp_Position&0xff);
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) 
			ret=0;
    else 
			ret=1;

}


/****************************************************************************************
                                  电流速度位置模式下的数据指令
Group   取值范围 0-7

Number  取值范围 0-15，其中Number==0时，为广播发送

temp_current的取值范围如下：
0 ~ +32767，单位mA

temp_velocity的取值范围如下：
0 ~ +32767，单位RPM

temp_position的取值范围如下：
-2147483648~+2147483647，单位qc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(uint8_t Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID
    
    if(Temp_Current < 0)
    {
        Temp_Current = abs(Temp_Current);
    }
    
    if(Temp_Velocity < 0)
    {
        Temp_Velocity = abs(Temp_Velocity);
    }
    
    tx_message.dataByte0 = (uint8_t)((Temp_Current>>8)&0xff);
    tx_message.dataByte1 = (uint8_t)(Temp_Current&0xff);
    tx_message.dataByte2 = (uint8_t)((Temp_Velocity>>8)&0xff);
    tx_message.dataByte3 = (uint8_t)(Temp_Velocity&0xff);
    tx_message.dataByte4 = (uint8_t)((Temp_Position>>24)&0xff);
    tx_message.dataByte5 = (uint8_t)((Temp_Position>>16)&0xff);
    tx_message.dataByte6 = (uint8_t)((Temp_Position>>8)&0xff);
    tx_message.dataByte7 = (uint8_t)(Temp_Position&0xff);
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) 
			ret=0;
    else 
			ret=1;


}

/****************************************************************************************
                                      配置指令
Temp_Time1的取值范围: 0 ~ 255，为0时候，为关闭电流速度位置反馈功能
Temp_Time2的取值范围: 0 ~ 255，为0时候，为关闭限位信号反馈功能
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(uint8_t Number,uint8_t Temp_Time1,uint8_t Temp_Time2)
{
    unsigned short can_id = 0x00A;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //标准帧
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;
    
    tx_message.dataByte0 = Temp_Time1;
    tx_message.dataByte1 = Temp_Time2;
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) 
			ret=0;
    else 
			ret=1;

}

/****************************************************************************************
                                      在线检测
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(uint8_t Number)
{
    unsigned short can_id = 0x00F;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;
    tx_message.type = kFLEXCAN_FrameTypeData;  //数据帧
    tx_message.length = 0x08;          //帧长度为8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //帧ID为传入参数的CAN_ID
    
    tx_message.dataByte0 = 0x55;
    tx_message.dataByte1 = 0x55;
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) 
		ret=0;
    else 
		ret=1;

}



