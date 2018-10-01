#include "bsp_flexpwm.h"
#include "fsl_debug_console.h"

pwm_config_t pwm2sm3_config;    //PWM2ģ��3���ýṹ��

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
	
	led_config.direction=kGPIO_DigitalOutput;	//���
	led_config.interruptMode=kGPIO_NoIntmode;	//��ʹ���жϹ���
	led_config.outputLogic=1;					//Ĭ�ϸߵ�ƽ��LED�ƹر�
	
	GPIO_PinInit(GPIO1,24,&led_config); 
	GPIO_PinInit(GPIO1,25,&led_config); 
	GPIO_PinInit(GPIO1,10,&led_config); 
	
	//digitalLo(GPIO1,24);
	digitalLo(GPIO1,25);
	
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal;
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //��Ƶ
    
	//IO��������
	IOMUXC_SetPinMux(IOMUXC_GPIO_B0_07_FLEXPWM2_PWMB00,0);   
    IOMUXC_SetPinMux(IOMUXC_GPIO_B0_06_FLEXPWM2_PWMA00,0U);
	//����IO����GPIO_SD_B1_02��GPIO_SD_B1_03�Ĺ���
	//��ת���ٶ�,��������ΪR0/6,�ٶ�Ϊ100Mhz���رտ�·���ܣ�ʹ��pull/keepr
	//ѡ��keeper���ܣ�����100K Ohm���ر�Hyst
    IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_07_FLEXPWM2_PWMB00,0x10B0);
	IOMUXC_SetPinConfig(IOMUXC_GPIO_B0_06_FLEXPWM2_PWMA00,0x10B0u);
    
    //��ʼ��PWM2 ģ��3��ͨ��B
    PWM_GetDefaultConfig(&pwm2sm3_config);              //�ȳ�ʼ��ΪĬ������
    pwm2sm3_config.clockSource=kPWM_BusClock;           //ʱ��ԴΪIP BUS=IPG_CLK_ROOT=150MHz
    pwm2sm3_config.prescale=pwm_prescale;               //���÷�Ƶ
    pwm2sm3_config.reloadLogic=kPWM_ReloadPwmFullCycle; //ȫ���ڸ���
    pwm2sm3_config.pairOperation=kPWM_Independent;      //PMWA PWMB����ģʽ
	PWM_Init(PWM2,kPWM_Module_0,&pwm2sm3_config);       //��ʼ��PWM2ģ��3

    //���ι��ϼ�⹦��
    PWM2->SM[0].DISMAP[0]=0;     
    
    //����PWM2Bͨ��
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
    //PWMB
    pwm_ignal.pwmChannel=kPWM_PwmB;                     //PWMͨ��B
    pwm_ignal.level=kPWM_HighTrue;                      //�ߵ�ƽ��Ч
    pwm_ignal.dutyCyclePercent=duty;                    //ռ�ձ�

    //����PWM2���������ģʽ
    PWM_SetupPwm(PWM2,kPWM_Module_0,&pwm_ignal,1,kPWM_CenterAligned,fre,sourceclock);
	pwm_ignal.pwmChannel=kPWM_PwmA; 
    PWM_SetupPwm(PWM2,kPWM_Module_0,&pwm_ignal,1,kPWM_CenterAligned,fre,sourceclock);
	
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_0,true);    //����PWM��load okλ
    PWM_StartTimer(PWM2,kPWM_Control_Module_0);         //������ʱ��
}

//����PWM2ռ�ձ�
//duty:ռ�ձ�
void PWM2_SM3_DutySet(uint8_t duty) 
{
    PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_0,kPWM_PwmB,kPWM_CenterAligned,duty); //����PWMBռ�ձ�
	PWM_UpdatePwmDutycycle(PWM2,kPWM_Module_0,kPWM_PwmA,kPWM_CenterAligned,duty); //����PWMBռ�ձ�
    PWM_SetPwmLdok(PWM2,kPWM_Control_Module_0,true);    //����PWM��load okλ
}

void DUO_Init(void)
{
	gpio_pin_config_t led_config;
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_08_GPIO1_IO24,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_09_GPIO1_IO25,0);	//RGB_RED
	IOMUXC_SetPinMux(IOMUXC_GPIO_AD_B1_10_GPIO1_IO26,0);	//RGB_RED
	led_config.direction=kGPIO_DigitalOutput;	//���
	led_config.interruptMode=kGPIO_NoIntmode;	//��ʹ���жϹ���
	led_config.outputLogic=0;					//Ĭ�ϸߵ�ƽ��LED�ƹر�
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
	uint16_t psc=7;  //Ԥ��Ƶ��,0~7,��ʾ2^psc��Ƶ.
	pwm_config_t pwm_init_structure;
    uint32_t sourceclock;
    pwm_signal_param_t pwm_ignal[2];
    pwm_clock_prescale_t pwm_prescale=(pwm_clock_prescale_t)psc; //��Ƶ
    

    //��ʼ��Pwmx ģ��N��ͨ��X
  PWM_GetDefaultConfig(&pwm_init_structure);              //�ȳ�ʼ��ΪĬ������
  pwm_init_structure.clockSource=kPWM_BusClock;           //ʱ��ԴΪIP BUS=IPG_CLK_ROOT=150MHz
  pwm_init_structure.prescale=pwm_prescale;               //���÷�Ƶ
  pwm_init_structure.reloadLogic=kPWM_ReloadPwmFullCycle; //ȫ���ڸ���
  pwm_init_structure.pairOperation=kPWM_Independent;      //PMWA PWMB����ģʽ
	PWM_Init(Pwmx,Module_x,&pwm_init_structure);       //��ʼ��Pwmxģ��3

    //���ι��ϼ�⹦��
  Pwmx->SM[0].DISMAP[0]=0; 
	Pwmx->SM[1].DISMAP[0]=0;
	Pwmx->SM[2].DISMAP[0]=0;
    
    //����Pwmxͨ��
    sourceclock=CLOCK_GetFreq(kCLOCK_IpgClk);
    
  pwm_ignal[0].pwmChannel=kPWM_PwmA;                     //PWMAͨ��
  pwm_ignal[0].level=kPWM_HighTrue;                      //�ߵ�ƽ��Ч
  pwm_ignal[0].dutyCyclePercent=50;                    //ռ�ձ�
	pwm_ignal[1].pwmChannel=kPWM_PwmB;                     //PWMͨ��
	pwm_ignal[1].level=kPWM_HighTrue;                      //�ߵ�ƽ��Ч
  pwm_ignal[1].dutyCyclePercent=50;                    //ռ�ձ�
	//����Pwmx���������ģʽ
  PWM_SetupPwm(Pwmx,Module_x,pwm_ignal,2,kPWM_CenterAligned,freq,sourceclock);

  PWM_SetPwmLdok(Pwmx,subModulesToUpdate,true);    //����PWM��load okλ
  PWM_StartTimer(Pwmx,subModulesToUpdate);         //������ʱ��

}

void PWMx_SMx_DutySet(PWM_Type *Pwmx,pwm_submodule_t Module_x,uint8_t Chanal, uint8_t duty) 
{
	switch(Chanal)
	{
		case 1:
			PWM_UpdatePwmDutycycle(Pwmx,Module_x,kPWM_PwmA,kPWM_CenterAligned,duty); //����PWMBռ�ձ�
			break;
		case 2:
			PWM_UpdatePwmDutycycle(Pwmx,Module_x,kPWM_PwmB,kPWM_CenterAligned,duty); //����PWMBռ�ձ�
			break;
	}
    //PWM_UpdatePwmDutycycle(Pwmx,Module_x,pwmSignal,kPWM_CenterAligned,duty); //����PWMBռ�ձ�
}
