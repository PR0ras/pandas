#ifndef __BSP_RED_H
#define __BSP_RED_H

#include "fsl_common.h"

void red_gpio_init();

void RED_IRQHandler(void);
	
static void RED_Interrupt_Config(void);

static void RED_IOMUXC_PAD_Config(void);
static void RED_GPIO_Mode_Config(void);

#endif


