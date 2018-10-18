#include "bsp_pit.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_gpio.h"  
#include "fsl_gpt.h"
extern float vx,vy,vz;

 int32_t count_stop;
	 uint32_t sec=0;
uint32_t umsec1=0,umsec2=0,umsec3=0;
extern uint32_t mm;	 
	pit_config_t pit_config;
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
	led_config.direction=kGPIO_DigitalOutput;	//输出
	led_config.interruptMode=kGPIO_NoIntmode;	//不使用中断功能
	led_config.outputLogic=1;					//默认高电平，LED灯关闭
	
	GPIO_PinInit(GPIO1,02,&led_config); 
	GPIO_PinInit(GPIO1,03,&led_config); 
//	GPIO_PinInit(GPIO1,04,&led_config); 
//	GPIO_PinInit(GPIO1,05,&led_config); 
//	GPIO_PinInit(GPIO1,10,&led_config); 
	
	//digitalLo(GPIO1,24);
	digitalLo(GPIO1,03);
	
    PIT_GetDefaultConfig(&pit_config);  //初始化为默认配置
    pit_config.enableRunInDebug=true;   //调试模式下PIT继续运行
    PIT_Init(PIT,&pit_config);          //初始化PIT定时器
    
//    PIT_SetTimerPeriod(PIT,kPIT_Chnl_0,ldval);//设置倒计时初始值
//    PIT_EnableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);//使能中断
	
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_1,66000);//设置倒计时初始值
    PIT_EnableInterrupts(PIT,kPIT_Chnl_1,kPIT_TimerInterruptEnable);//使能中断
	
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_2,660);//设置倒计时初始值
    PIT_EnableInterrupts(PIT,kPIT_Chnl_2,kPIT_TimerInterruptEnable);//使能中断
	
    RT1052_NVIC_SetPriority(PIT_IRQn,6,0);	//抢占优先级6，子优先级0
		EnableIRQ(PIT_IRQn);	                //使能PIT中断
		       //打开PIT
		PIT_StartTimer(PIT,kPIT_Chnl_2);
		PIT_StartTimer(PIT,kPIT_Chnl_1);
}
void GPT1_Int_Init(uint32_t ocrx)
{
	gpt_config_t gpt1_onfig;
	GPT_GetDefaultConfig(&gpt1_onfig);	//先初始化GPT1为默认值
	gpt1_onfig.clockSource=kGPT_ClockSource_Periph;	//初始化时钟源perclk_clk_root
	gpt1_onfig.divider=(uint16_t)1;	        //设置分频值
	GPT_Init(GPT1,&gpt1_onfig);
	
	GPT_SetOutputCompareValue(GPT1,kGPT_OutputCompare_Channel1,ocrx);	    //设置比较计数值
	GPT_EnableInterrupts(GPT1,kGPT_OutputCompare1InterruptEnable);			//使能GPT比较通道1中断
	RT1052_NVIC_SetPriority(GPT1_IRQn,5,0);									//抢占优先级5，子优先级0
	EnableIRQ(GPT1_IRQn);	//使能GPT1中断
	//GPT_StartTimer(GPT1);	//开始定时器	
}
void GPT1_IRQHandler(void) 
{
	static uint32_t count;
    //OCR1中断
    if(GPT_GetStatusFlags(GPT1,kGPT_OutputCompare1Flag))
    {
		digitalToggle(GPIO1,03);
        if(count++>=count_stop)
			{	
				count=0;	
				count_stop=0;				
				GPT_StopTimer(GPT1);
			}
        GPT_ClearStatusFlags(GPT1,kGPT_OutputCompare1Flag);//清除中断标志位
    }
	__DSB();				//数据同步屏蔽指令
}
//PIT中断服务函数
void PIT_IRQHandler(void)
{
//	static uint32_t count;

//    //PIT CH0中断
//    if((PIT_GetStatusFlags(PIT,kPIT_Chnl_0)&kPIT_TimerFlag)==kPIT_TimerFlag)
//    {
//        digitalToggle(GPIO1,03);
//			if(count++>=count_stop)
//			{	
//				count=0;	
//				count_stop=0;				
//				PIT_StopTimer(PIT,kPIT_Chnl_0);
//			}
//        PIT_ClearStatusFlags(PIT,kPIT_Chnl_0,kPIT_TimerFlag);//清楚中断标志位
//    }
	 //PIT CH1中断
	if((PIT_GetStatusFlags(PIT,kPIT_Chnl_1)&kPIT_TimerFlag)==kPIT_TimerFlag)
	{
		sec++;	
		PIT_ClearStatusFlags(PIT,kPIT_Chnl_1,kPIT_TimerFlag);//清楚中断标志位
	}
	
	if((PIT_GetStatusFlags(PIT,kPIT_Chnl_2)&kPIT_TimerFlag)==kPIT_TimerFlag)
    {
			umsec1++;
			umsec2++;
      PIT_ClearStatusFlags(PIT,kPIT_Chnl_2,kPIT_TimerFlag);//清楚中断标志位
    }
	 __DSB();				//数据同步屏蔽指令
}

void change_height(int32_t temp)
{
	static int32_t height=0,d_height=0,height_o=0;
	
	height=temp;
	d_height=height-height_o;
	if(d_height>0)
	{
		digitalLo(GPIO1,02);
		count_stop = d_height;
	}
	else
	{
		digitalHi(GPIO1,02);
		count_stop=-d_height;
	}

	
	height_o=height;
	GPT_StartTimer(GPT1);
}