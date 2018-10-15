#include "bsp_pit.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_gpio.h"  
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
    
    PIT_SetTimerPeriod(PIT,kPIT_Chnl_0,ldval);//设置倒计时初始值
    PIT_EnableInterrupts(PIT,kPIT_Chnl_0,kPIT_TimerInterruptEnable);//使能中断
	
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_1,66000);//设置倒计时初始值
    PIT_EnableInterrupts(PIT,kPIT_Chnl_1,kPIT_TimerInterruptEnable);//使能中断
	
	PIT_SetTimerPeriod(PIT,kPIT_Chnl_2,660);//设置倒计时初始值
    PIT_EnableInterrupts(PIT,kPIT_Chnl_2,kPIT_TimerInterruptEnable);//使能中断
	
    RT1052_NVIC_SetPriority(PIT_IRQn,6,0);	//抢占优先级6，子优先级0
		EnableIRQ(PIT_IRQn);	                //使能PIT中断
		PIT_StartTimer(PIT,kPIT_Chnl_1);        //打开PIT
		PIT_StartTimer(PIT,kPIT_Chnl_2);
}

//PIT中断服务函数
void PIT_IRQHandler(void)
{
	static uint32_t count;

    //PIT CH0中断
    if((PIT_GetStatusFlags(PIT,kPIT_Chnl_0)&kPIT_TimerFlag)==kPIT_TimerFlag)
    {
        digitalToggle(GPIO1,03);
			if(count++>=count_stop)
			{	
				count=0;	
				count_stop=0;				
				PIT_StopTimer(PIT,kPIT_Chnl_0);
			}
        PIT_ClearStatusFlags(PIT,kPIT_Chnl_0,kPIT_TimerFlag);//清楚中断标志位
    }
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
	delay_ms(1);
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
	PIT_StartTimer(PIT,kPIT_Chnl_0);        //打开PIT
	
	height_o=height;
}