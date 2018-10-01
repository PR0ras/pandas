#ifndef BSP_FLEXPWM_H
#define BSP_FLEXPWM_H

#include "fsl_common.h"
#include "fsl_iomuxc.h"
#include "pin_mux.h"
#include "fsl_gpio.h"  
#include "fsl_pwm.h"

//////////////////////////////////////////////////////////////////////////////////	 
//������ֻ��ѧϰʹ�ã�δ��������ɣ��������������κ���;
//ALIENTEK I.MX RT1052������
//FLEXPWM��������	   
//����ԭ��@ALIENTEK
//������̳:www.openedv.com
//��������:2018/1/3
//�汾��V1.0
//��Ȩ���У�����ؾ���
//Copyright(C) ������������ӿƼ����޹�˾ 2014-2024
//All rights reserved									  
////////////////////////////////////////////////////////////////////////////////// 	

#define digitalHi(p,i)      {p->DR |= (1U << i);}   //���Ϊ�ߵ�ƽ    
#define digitalLo(p,i)      {p->DR &= ~(1U << i);}  //����͵�ƽ
#define digitalToggle(p,i)  {p->DR ^= (1U<<i);}     //�����ת״̬

//void PWM2_SM3_PWMAB_Init(uint16_t psc,uint32_t fre,uint8_t duty);	//PWM2 SM3 PWM�����ʼ������
//void PWM2_SM3_DutySet(uint8_t duty) ;     //����PWM2_SM3����ͨ����ռ�ձ�

void DUO_Init(void);
void change_duty(uint8_t num,uint8_t duty);
void PWM_bsp_Init(PWM_Type *Pwmx,pwm_submodule_t Module_x,
									uint8_t subModulesToUpdate,
									uint32_t freq);

void PWMx_SMx_DutySet(PWM_Type *Pwmx,pwm_submodule_t Module_x,
											uint8_t Chanal,
											uint8_t duty) ;
#endif
