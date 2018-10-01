#include "bsp_pit.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_gpio.h"  
float vx,vy,vz;
pit_config_t pit_config;
	uint16_t count_stop;
void PIT_CH0_Int_Init(uint32_t ldval)
{
	gpio_pin_config_t led_config;
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03,0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04,0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,0x10B0);
	led_config.direction=kGPIO_DigitalOutput;	//���
	led_config.interruptMode=kGPIO_NoIntmode;	//��ʹ���жϹ���
	led_config.outputLogic=1;					//Ĭ�ϸߵ�ƽ��LED�ƹر�
	
	GPIO_PinInit(GPIO1,02,&led_config); 
	GPIO_PinInit(GPIO1,03,&led_config); 
	GPIO_PinInit(GPIO1,04,&led_config); 
//	GPIO_PinInit(GPIO1,05,&led_config); 
	GPIO_PinInit(GPIO1,10,&led_config); 
	
	//digitalLo(GPIO1,24);
	digitalLo(GPIO1,03);
	
    PIT_GetDefaultConfig(&pit_config);  //��ʼ��ΪĬ������
    pit_config.enableRunInDebug=true;   //����ģʽ��PIT��������
    PIT_Init(PIT,&pit_config);          //��ʼ��PIT��ʱ��
    
    PIT_SetTimerPeriod(PIT,kPIT_Chnl_0,ldval);//���õ���ʱ��ʼֵ
    PIT_EnableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);//ʹ���ж�
    RT1052_NVIC_SetPriority(PIT_IRQn,6,0);	//��ռ���ȼ�6�������ȼ�0
		EnableIRQ(PIT_IRQn);	                //ʹ��PIT�ж�
//    PIT_StartTimer(PIT,kPIT_Chnl_0);        //��PIT
}

//PIT�жϷ�����
void PIT_IRQHandler(void)
{
	static uint32_t count;
    //PIT CH0�ж�
    if((PIT_GetStatusFlags(PIT,kPIT_Chnl_0)&kPIT_TimerFlag)==kPIT_TimerFlag)
    {
        //digitalToggle(GPIO1,2);
//			if(count++>=1000)
//			{
//				vx=0.0;
//				vy=0.0;
//				vz=0.0;
//			}
			
//			PIT_StopTimer(PIT,kPIT_Chnl_0);
        PIT_ClearStatusFlags(PIT,kPIT_Chnl_0,kPIT_TimerFlag);//����жϱ�־λ
    }
    __DSB();				//����ͬ������ָ��
}

void change_hight(uint8_t fangxiang,uint16_t temp)
{

	if(fangxiang)
		digitalLo(GPIO1,03)
	else
		digitalHi(GPIO1,03);
	
	count_stop=temp;
	PIT_StartTimer(PIT,kPIT_Chnl_0);        //��PIT
}