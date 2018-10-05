#include "bsp_pit.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_gpio.h"  
extern float vx,vy,vz;
pit_config_t pit_config;
	volatile uint16_t count_stop;
	volatile uint32_t sec=0;	
void PIT_CH0_Int_Init(uint32_t ldval)
{
	gpio_pin_config_t led_config;
//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,0);
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03,0);
//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04,0);
//	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,0);
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_02_GPIO1_IO02,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_03_GPIO1_IO03,0x10B0);
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_04_GPIO1_IO04,0x10B0);
//	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_05_GPIO1_IO05,0x10B0);
	led_config.direction=kGPIO_DigitalOutput;	//���
	led_config.interruptMode=kGPIO_NoIntmode;	//��ʹ���жϹ���
	led_config.outputLogic=1;					//Ĭ�ϸߵ�ƽ��LED�ƹر�
	
//	GPIO_PinInit(GPIO1,02,&led_config); 
//	GPIO_PinInit(GPIO1,03,&led_config); 
//	GPIO_PinInit(GPIO1,04,&led_config); 
//	GPIO_PinInit(GPIO1,05,&led_config); 
//	GPIO_PinInit(GPIO1,10,&led_config); 
	
	//digitalLo(GPIO1,24);
//	digitalLo(GPIO1,03);
	
    PIT_GetDefaultConfig(&pit_config);  //��ʼ��ΪĬ������
    pit_config.enableRunInDebug=true;   //����ģʽ��PIT��������
    PIT_Init(PIT,&pit_config);          //��ʼ��PIT��ʱ��
    
    PIT_SetTimerPeriod(PIT,kPIT_Chnl_0,ldval);//���õ���ʱ��ʼֵ
    PIT_EnableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);//ʹ���ж�
	
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_1,66000);//���õ���ʱ��ʼֵ
    PIT_EnableInterrupts(PIT,kPIT_Chnl_1,kPIT_TimerInterruptEnable);//ʹ���ж�
	
    RT1052_NVIC_SetPriority(PIT_IRQn,6,0);	//��ռ���ȼ�6�������ȼ�0
	EnableIRQ(PIT_IRQn);	                //ʹ��PIT�ж�
 
}

//PIT�жϷ�����
void PIT_IRQHandler(void)
{
	static uint32_t count=0;

    //PIT CH0�ж�
    if((PIT_GetStatusFlags(PIT,kPIT_Chnl_0)&kPIT_TimerFlag)==kPIT_TimerFlag)
    {
        digitalToggle(GPIO1,03);
			if(count++>=count_stop)
			{	
				count=0;	
				count_stop=0;				
				PIT_StopTimer(PIT,kPIT_Chnl_0);
			}
        PIT_ClearStatusFlags(PIT,kPIT_Chnl_0,kPIT_TimerFlag);//����жϱ�־λ
    }
	
	if((PIT_GetStatusFlags(PIT,kPIT_Chnl_1)&kPIT_TimerFlag)==kPIT_TimerFlag)
	{
		sec++;	
		PIT_ClearStatusFlags(PIT,kPIT_Chnl_1,kPIT_TimerFlag);//����жϱ�־λ
	}
}

void change_hight(int32_t temp)
{

	if(temp>0)
	{
		digitalLo(GPIO1,02);
		count_stop = temp;
	}
	else
	{
		digitalHi(GPIO1,02);
		count_stop=-temp;
	}
	PIT_StartTimer(PIT,kPIT_Chnl_0);        //��PIT
}