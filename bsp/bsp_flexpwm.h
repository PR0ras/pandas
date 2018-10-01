#ifndef BSP_FLEXPWM_H
#define BSP_FLEXPWM_H

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_gpio.h"  
#include "fsl_pwm.h"

//////////////////////////////////////////////////////////////////////////////////	 
//本程序只供学习使用，未经作者许可，不得用于其它任何用途
//ALIENTEK I.MX RT1052开发板
//FLEXPWM驱动代码	   
//正点原子@ALIENTEK
//技术论坛:www.openedv.com
//创建日期:2018/1/3
//版本：V1.0
//版权所有，盗版必究。
//Copyright(C) 广州市星翼电子科技有限公司 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define digitalHi(p,i)      {p->DR |= (1U << i);}   //输出为高电平    
#define digitalLo(p,i)      {p->DR &= ~(1U << i);}  //输出低电平
#define digitalToggle(p,i)  {p->DR ^= (1U<<i);}     //输出反转状态

//void PWM2_SM3_PWMAB_Init(uint16_t psc,uint32_t fre,uint8_t duty);	//PWM2 SM3 PWM输出初始化函数
//void PWM2_SM3_DutySet(uint8_t duty) ;     //设置PWM2_SM3两个通道的占空比

void DUO_Init(void);
void change_duty(uint8_t num,uint8_t duty);
void PWM_bsp_Init(PWM_Type *Pwmx,pwm_submodule_t Module_x,
									uint8_t subModulesToUpdate,
									uint32_t freq);

void PWMx_SMx_DutySet(PWM_Type *Pwmx,pwm_submodule_t Module_x,
											uint8_t Chanal,
											uint8_t duty) ;
#endif
