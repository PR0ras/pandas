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
	FLEXCAN_EnableMbInterrupts(TS_CAN,1<<RX_MESSAGE_BUFFER_NUM);  //ʹ��RX��Ϣ�����ж�
	EnableIRQ(CAN2_IRQn);
	
	FLEXCAN_SetRxMbGlobalMask(TS_CAN, FLEXCAN_RX_MB_EXT_MASK(rxformatntifier, 0, 0));
	mbConfig.format = kFLEXCAN_FrameFormatStandard;
  mbConfig.type = kFLEXCAN_FrameTypeData;
  mbConfig.id = FLEXCAN_ID_STD(rxformatntifier);
	
	FLEXCAN_SetRxMbConfig(TS_CAN, RX_MESSAGE_BUFFER_NUM, &mbConfig, true);
	FLEXCAN_SetTxMbConfig(TS_CAN, TX_MESSAGE_BUFFER_NUM, true);
}  


//CAN2�жϷ�����
void CAN2_IRQHandler(void)
{
    if (FLEXCAN_GetMbStatusFlags(TS_CAN,1<<RX_MESSAGE_BUFFER_NUM))    //�ж�CAN2��RX��Ϣ�����Ƿ��յ�����
    {
        FLEXCAN_ClearMbStatusFlags(CAN2,1<<RX_MESSAGE_BUFFER_NUM);  //����жϱ�־λ
        FLEXCAN_ReadRxMb(TS_CAN,RX_MESSAGE_BUFFER_NUM,&can2_rxframe); //��ȡ����
        rxComplete=true;                                       //��Ƕ�ȡ���
    }
    __DSB();
}

uint8_t CAN2_Send_Msg(uint8_t* msg,uint8_t len,uint32_t txId)
{
    uint8_t ret=0;
    
    frame.format=kFLEXCAN_FrameFormatStandard;    //��׼��ʽ
    frame.type=kFLEXCAN_FrameTypeData;            //����֡
    frame.id=FLEXCAN_ID_STD(txId);           //��׼ID 
    frame.length=len;                             //����8
    
    //��������
    frame.dataByte0=msg[0];
    frame.dataByte1=msg[1];
		frame.dataByte2=msg[2];
    frame.dataByte3=msg[3];
		frame.dataByte4=msg[4];
    frame.dataByte5=msg[5];
		frame.dataByte6=msg[6];
    frame.dataByte7=msg[7];


    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&frame)==kStatus_Success) 
			ret=0;//�������ݣ���������	
    else 
			ret=1;
        
    return ret;
}

//can�ڽ������ݲ�ѯ
//buf:���ݻ�����;	 
//����ֵ:0,�����ݱ��յ�;
//		 ����,���յ����ݳ���;
uint8_t CAN2_Receive_Msg(uint8_t *buf)
{	  
    uint8_t datalen=0;

    if(rxComplete==true)  //�������
    {
        rxComplete=false;
        
        //��������
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
//������ֲ


/****************************************************************************************
                                       ��λָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����
*****************************************************************************************/
void CAN_RoboModule_DRV_Reset(uint8_t Number)
{
    unsigned short can_id = 0x000;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length=0x08;         //֡����Ϊ8
    
    if(Number<=15)
    {
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id=FLEXCAN_ID_STD(can_id);      //֡IDΪ���������CAN_ID
    
    tx_message.dataByte0 = 0x55;
    tx_message.dataByte1 = 0x55;
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
  
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;//�������ݣ���������	
    else ret=1;

}

/****************************************************************************************
                                     ģʽѡ��ָ��
Group   ȡֵ��Χ 0-7
Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

Mode    ȡֵ��Χ

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
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID
    
    tx_message.dataByte0 = Mode;
    tx_message.dataByte1 = 0x55;
    tx_message.dataByte2 = 0x55;
    tx_message.dataByte3 = 0x55;
    tx_message.dataByte4 = 0x55;
    tx_message.dataByte5 = 0x55;
    tx_message.dataByte6 = 0x55;
    tx_message.dataByte7 = 0x55;
    
    if(FLEXCAN_TransferSendBlocking(TS_CAN,TX_MESSAGE_BUFFER_NUM,&tx_message)==kStatus_Success) ret=0;//�������ݣ���������	
    else ret=1;
		

}

/****************************************************************************************
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
-5000 ~ +5000����ֵ5000������temp_pwm = ��5000ʱ����������ѹΪ��Դ��ѹ

*****************************************************************************************/
void CAN_RoboModule_DRV_OpenLoop_Mode(uint8_t Number,short Temp_PWM)
{
    unsigned short can_id = 0x002;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID

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
                                   ����ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_current��ȡֵ��Χ���£�
-32768 ~ +32767����λmA

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Mode(uint8_t Number,short Temp_PWM,short Temp_Current)
{
    unsigned short can_id = 0x003;
    flexcan_frame_t tx_message;
        uint8_t ret=0;
	
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID

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
                                   �ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Mode(uint8_t Number,short Temp_PWM,short Temp_Velocity)
{
    unsigned short can_id = 0x004;
    flexcan_frame_t tx_message;
    uint8_t ret=0;    
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID

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
                                   λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Position_Mode(uint8_t Number,short Temp_PWM,long Temp_Position)
{
    unsigned short can_id = 0x005;
    flexcan_frame_t tx_message;

    uint8_t ret=0;    
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID

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
                                  �ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_pwm��ȡֵ��Χ���£�
0 ~ +5000����ֵ5000������temp_pwm = 5000ʱ����������ѹΪ��Դ��ѹ

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc
*****************************************************************************************/
void CAN_RoboModule_DRV_Velocity_Position_Mode(uint8_t Number,short Temp_PWM,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x006;
    flexcan_frame_t tx_message;

    uint8_t ret=0;    
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID

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
                                  �����ٶ�ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
-32768 ~ +32767����λRPM

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Mode(uint8_t Number,short Temp_Current,short Temp_Velocity)
{
    unsigned short can_id = 0x007;
    flexcan_frame_t tx_message;
        uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID
    
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
                                  ����λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Position_Mode(uint8_t Number,short Temp_Current,long Temp_Position)
{
    unsigned short can_id = 0x008;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID

    
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
                                  �����ٶ�λ��ģʽ�µ�����ָ��
Group   ȡֵ��Χ 0-7

Number  ȡֵ��Χ 0-15������Number==0ʱ��Ϊ�㲥����

temp_current��ȡֵ��Χ���£�
0 ~ +32767����λmA

temp_velocity��ȡֵ��Χ���£�
0 ~ +32767����λRPM

temp_position��ȡֵ��Χ���£�
-2147483648~+2147483647����λqc

*****************************************************************************************/
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(uint8_t Number,short Temp_Current,short Temp_Velocity,long Temp_Position)
{
    unsigned short can_id = 0x009;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID
    
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
                                      ����ָ��
Temp_Time1��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�رյ����ٶ�λ�÷�������
Temp_Time2��ȡֵ��Χ: 0 ~ 255��Ϊ0ʱ��Ϊ�ر���λ�źŷ�������
*****************************************************************************************/
void CAN_RoboModule_DRV_Config(uint8_t Number,uint8_t Temp_Time1,uint8_t Temp_Time2)
{
    unsigned short can_id = 0x00A;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;    //��׼֡
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
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
                                      ���߼��
*****************************************************************************************/
void CAN_RoboModule_DRV_Online_Check(uint8_t Number)
{
    unsigned short can_id = 0x00F;
    flexcan_frame_t tx_message;
    uint8_t ret=0;
    tx_message.format = kFLEXCAN_FrameFormatStandard;
    tx_message.type = kFLEXCAN_FrameTypeData;  //����֡
    tx_message.length = 0x08;          //֡����Ϊ8
    
    if(Number<=15)
    {
        
        can_id |= Number<<4;
    }
    else
    {
        return;
    }
    
    tx_message.id =FLEXCAN_ID_STD(can_id);;      //֡IDΪ���������CAN_ID
    
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



