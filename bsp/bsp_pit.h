#ifndef _BSP_PIT_H
#define _BSP_PIT_H

#define digitalHi(p,i)      {p->DR |= (1U << i);}   //���Ϊ�ߵ�ƽ    
#define digitalLo(p,i)      {p->DR &= ~(1U << i);}  //����͵�ƽ
#define digitalToggle(p,i)  {p->DR ^= (1U<<i);}     //�����ת״̬

#include "fsl_common.h"
#include "fsl_pit.h"

void PIT_CH0_Int_Init(uint32_t ldval);
void change_hight(int32_t temp);
#endif