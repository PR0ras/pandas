#include "bsp_flexpwm.h"
#include "fsl_debug_console.h"

pwm_config_t pwm2sm3_config;    //PWM2模块3配置结构体

void PWM2_SM3_PWMAB_Init(uint16_t psc,uint32_t fre,uint8_t duty)
{
	//RGB light
	gpio_pin_config_t led_config;
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_GPIO1_IO24,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_GPIO1_IO25,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,0);	//RGB_RED
	
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_08_GPIO1_IO24,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B1_09_GPIO1_IO25,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_AD_B0_10_GPIO1_IO10,0x10B0);
	
	led_config.direction=kGPIO_DigitalOutput;	//输出
	led_config.interruptMode=kGPIO_NoIntmode;	//不使用中断功能
	led_config.outputLogic=1;					//默认高电平，LED灯关闭
	
	GPIO_PinInit(GPIO1,24,&led_config); 
	GPIO_PinInit(GPIO1,25,&led_config); 
	GPIO_PinInit(GPIO1,10,&led_config); 
	
	//digitalLo(GPIO1,24);
	digitalLo(GPIO1,25);
	
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal;
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //分频
    
	//IO功能设置
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_07_FLEXPWM2_PWMB00,0);   
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_06_FLEXPWM2_PWMA00,0U);
	//配置IO引脚GPIO_SD_B1_02和GPIO_SD_B1_03的功能
	//低转换速度,驱动能力为R0/6,速度为100Mhz，关闭开路功能，使能pull/keepr
	//选择keeper功能，下拉100K Ohm，关闭Hyst
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_07_FLEXPWM2_PWMB00,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_06_FLEXPWM2_PWMA00,0x10B0u);
    
    //初始化PWM2 模块3的通道B
    PWM_GetDefaultConfig(&pwm2sm3_config);              //先初始化为默认配置
    pwm2sm3_config.clockSource=kPWM_BusClock;           //时钟源为IP BUS=IPG_CLK_ROOT=150MHz
    pwm2sm3_config.prescale=pwm_prescale;               //设置分频
    pwm2sm3_config.reloadLogic=kPWM_ReloadPwmFullCycle; //全周期更新
    pwm2sm3_config.pairOperation=kPWM_Independent;      //PMWA PWMB独立模式
	PWM_Init(PWM2,kPWM_Module_0,&pwm2sm3_config);       //初始化PWM2模块3

    //屏蔽故障检测功能
    PWM2->SM[0].DISMAP[0]=0;     
    
    //设置PWM2B通道
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
    //PWMB
    pwm_ignal.pwmChannel=kPWM_PwmB;                     //PWM通道B
    pwm_ignal.level=kPWM_HighTrue;                      //高电平有效
    pwm_ignal.dutyCyclePercent=duty;                    //占空比

    //设置PWM2，中央对齐模式
    PWM_SetupPwm(PWM2,kPWM_Module_0,&pwm_ignal,1,kPWM_CenterAligned,fre,sourceclock);
	pwm_ignal.pwmChannel=kPWM_PwmA; 
    PWM_SetupPwm(PWM2,kPWM_Module_0,&pwm_ignal,1,kPWM_CenterAligned,fre,sourceclock);
	
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_0,true);    //设置PWM的load ok位
    PWM_StartTimer(PWM2,kPWM_Control_Module_0);         //开启定时器
}

//更新PWM2占空比
//duty:占空比
void PWM2_SM3_DutySet(uint8_t duty) 
{
    PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_0,kPWM_PwmB,kPWM_CenterAligned,duty); //更新PWMB占空比
	PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_0,kPWM_PwmA,kPWM_CenterAligned,duty); //更新PWMB占空比
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_0,true);    //设置PWM的load ok位
}

void DUO_Init(void)
{
	gpio_pin_config_t led_config;
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_GPIO1_IO24,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_GPIO1_IO25,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_GPIO1_IO26,0);	//RGB_RED
	led_config.direction=kGPIO_DigitalOutput;	//输出
	led_config.interruptMode=kGPIO_NoIntmode;	//不使用中断功能
	led_config.outputLogic=0;					//默认高电平，LED灯关闭
	GPIO_PinInit(GPIO1,24,&led_config); 
	GPIO_PinInit(GPIO1,25,&led_config); 
	GPIO_PinInit(GPIO1,26,&led_config);
	
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_06_FLEXPWM2_PWMA00,0U);                                  
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_07_FLEXPWM2_PWMB00,0U);
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_08_FLEXPWM2_PWMA01,0U);                                   
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_09_FLEXPWM2_PWMB01,0U);
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_10_FLEXPWM2_PWMA02,0U);
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_11_FLEXPWM2_PWMB02,0U);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_06_FLEXPWM2_PWMA00,0x10B0u); 		
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_07_FLEXPWM2_PWMB00,0x10B0u); 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_08_FLEXPWM2_PWMA01,0x10B0u); 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_09_FLEXPWM2_PWMB01,0x10B0u); 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_10_FLEXPWM2_PWMA02,0x10B0u); 
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_11_FLEXPWM2_PWMB02,0x10B0u); 			
	
	PWM_bsp_Init(PWM2,kPWM_Module_0,kPWM_Control_Module_0,5);
	//PWM_bsp_Init(PWM2,kPWM_Module_0,kPWM_Control_Module_0,kPWM_PwmB,50);
	PWM_bsp_Init(PWM2,kPWM_Module_1,kPWM_Control_Module_1,5);
	//PWM_bsp_Init(PWM2,kPWM_Module_1,kPWM_Control_Module_1,kPWM_PwmB,50);
	PWM_bsp_Init(PWM2,kPWM_Module_2,kPWM_Control_Module_2,5);
	//PWM_bsp_Init(PWM2,kPWM_Module_2,kPWM_Control_Module_2,kPWM_PwmB,50);
}

void change_duty(uint8_t num,uint8_t duty)
{
	switch(num)
	{
		case 1:
						PWMx_SMx_DutySet(PWM2,kPWM_Module_0,1,duty);
						break;
		case 2:
						PWMx_SMx_DutySet(PWM2,kPWM_Module_0,2,duty);
						break;
		case 3:
						PWMx_SMx_DutySet(PWM2,kPWM_Module_1,1,duty);
						break;
		case 4:
						PWMx_SMx_DutySet(PWM2,kPWM_Module_1,2,duty);
						break;
		case 5:
						PWMx_SMx_DutySet(PWM2,kPWM_Module_2,1,duty);
						break;
		case 6:
						PWMx_SMx_DutySet(PWM2,kPWM_Module_2,2,duty);
						break;
	}
}	

void PWM_bsp_Init(PWM_Type *Pwmx,pwm_submodule_t Module_x,uint8_t subModulesToUpdate,uint32_t freq)
{
	uint16_t psc=7;  //预分频器,0~7,表示2^psc分频.
	pwm_config_t pwm_init_structure;
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal[2];
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //分频
    

    //初始化Pwmx 模块N的通道X
  PWM_GetDefaultConfig(&pwm_init_structure);              //先初始化为默认配置
  pwm_init_structure.clockSource=kPWM_BusClock;           //时钟源为IP BUS=IPG_CLK_ROOT=150MHz
  pwm_init_structure.prescale=pwm_prescale;               //设置分频
  pwm_init_structure.reloadLogic=kPWM_ReloadPwmFullCycle; //全周期更新
  pwm_init_structure.pairOperation=kPWM_Independent;      //PMWA PWMB独立模式
	PWM_Init(Pwmx,Module_x,&pwm_init_structure);       //初始化Pwmx模块3

    //屏蔽故障检测功能
  Pwmx->SM[0].DISMAP[0]=0; 
	Pwmx->SM[1].DISMAP[0]=0;
	Pwmx->SM[2].DISMAP[0]=0;
    
    //设置Pwmx通道
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
    
  pwm_ignal[0].pwmChannel=kPWM_PwmA;                     //PWMA通道
  pwm_ignal[0].level=kPWM_HighTrue;                      //高电平有效
  pwm_ignal[0].dutyCyclePercent=50;                    //占空比
	pwm_ignal[1].pwmChannel=kPWM_PwmB;                     //PWM通道
	pwm_ignal[1].level=kPWM_HighTrue;                      //高电平有效
  pwm_ignal[1].dutyCyclePercent=50;                    //占空比
	//设置Pwmx，中央对齐模式
  PWM_SetupPwm(Pwmx,Module_x,pwm_ignal,2,kPWM_CenterAligned,freq,sourceclock);

  PWM_SetPwmLdok(Pwmx,subModulesToUpdate,true);    //设置PWM的load ok位
  PWM_StartTimer(Pwmx,subModulesToUpdate);         //开启定时器

}

void PWMx_SMx_DutySet(PWM_Type *Pwmx,pwm_submodule_t Module_x,uint8_t Chanal, uint8_t duty) 
{
	switch(Chanal)
	{
		case 1:
			PWM_UpdatePwmDutycycle(Pwmx,Module_x,kPWM_PwmA,kPWM_CenterAligned,duty); //更新PWMB占空比
			break;
		case 2:
			PWM_UpdatePwmDutycycle(Pwmx,Module_x,kPWM_PwmB,kPWM_CenterAligned,duty); //更新PWMB占空比
			break;
	}
    //PWM_UpdatePwmDutycycle(Pwmx,Module_x,pwmSignal,kPWM_CenterAligned,duty); //更新PWMB占空比
}
