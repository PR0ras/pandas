#ifndef _BSP_PIT_H
#define _BSP_PIT_H

#define digitalHi(p,i)      {p->DR |= (1U << i);}   //输出为高电平    
#define digitalLo(p,i)      {p->DR &= ~(1U << i);}  //输出低电平
#define digitalToggle(p,i)  {p->DR ^= (1U<<i);}     //输出反转状态

#include "fsl_common.h"
#include "fsl_pit.h"

void PIT_CH0_Int_Init(uint32_t ldval);
void change_hight(int32_t temp);
#endif