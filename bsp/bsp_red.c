#include "bsp_red.h"
#include "fsl_iomuxc.h"
#include "fsl_gpio.h" 
#include "board.h"
#include "fsl_debug_console.h"
#include "bsp_pit.h"

extern uint32_t umsec1,umsec2,umsec3;
uint32_t mm1=0,mm2=0;
extern uint8_t key1;
static void RED_IOMUXC_PAD_Config(void)
{
  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_05_GPIO2_IO05,           /* GPIO_B0_06 is configured as GPIO2_IO06 */
      0U);  
	IOMUXC_SetPinConfig(
      IOMUXC_GPIO_B0_05_GPIO2_IO05,        /* GPIO_AD_B0_12 PAD functional properties : */
      0x10B0u); 
	  IOMUXC_SetPinMux(
      IOMUXC_GPIO_B0_07_GPIO2_IO07,           /* GPIO_B0_06 is configured as GPIO2_IO06 */
      0U);  
	IOMUXC_SetPinConfig(
      IOMUXC_GPIO_B0_07_GPIO2_IO07,        /* GPIO_AD_B0_12 PAD functional properties : */
      0x10B0u); 
}

static void RED_Interrupt_Config(void)
{
	  /* ��IOMUXC_SNVS ʱ�� */
  CLOCK_EnableClock(kCLOCK_IomuxcSnvs);    

	RT1052_NVIC_SetPriority(GPIO2_Combined_0_15_IRQn,6,2);	
  /* ����GPIO�˿��ж� */
  EnableIRQ(GPIO2_Combined_0_15_IRQn);
  
  /* ����GPIO�˿�ĳ�����ŵ��ж� */
	  GPIO_PortEnableInterrupts(GPIO2, 
                            1U << 7U);  	
  GPIO_PortEnableInterrupts(GPIO2, 
                            1U << 5U);   

}
static void RED_GPIO_Mode_Config(void)
{
	  /* ����Ϊ����ģʽ���͵�ƽ�жϣ�����ͨ��GPIO_PinInit������������ */
  gpio_pin_config_t RED_config;
  
  /** ���İ�İ�����GPIO���� **/       
  RED_config.direction = kGPIO_DigitalInput;    //����ģʽ
  RED_config.outputLogic =  1;                  //Ĭ�ϸߵ�ƽ������ģʽʱ��Ч��
  RED_config.interruptMode = kGPIO_IntRisingEdge;                              //�͵�ƽ�����ж�
  
  /* ��ʼ�� KEY GPIO. */
  GPIO_PinInit(GPIO2, 5U, &RED_config);
  GPIO_PinInit(GPIO2, 7U, &RED_config);
}
void red_gpio_init()
{
	RED_IOMUXC_PAD_Config();
	RED_GPIO_Mode_Config();
	RED_Interrupt_Config();

}

void GPIO2_Combined_0_15_IRQHandler()
{
	static uint8_t measure_ready1=0,measure_ready2=0;
	    /* ����жϱ�־λ */
    
//PRINTF("gpio_interrupt\r\n");  
//	umsec++;
	if((GPIO_PortGetInterruptFlags(GPIO2)&(1U<<5U))==1U<<5U)
	{
	if(measure_ready1)
		{  
			measure_ready1=0;
			mm1=umsec1+20;
			GPIO_PinSetInterruptConfig(GPIO2,5U,kGPIO_IntRisingEdge);
		}
		else
		{
			measure_ready1=1;
			GPIO_PinSetInterruptConfig(GPIO2,5U,kGPIO_IntFallingEdge);
			umsec1=0;
		}
		GPIO_PortClearInterruptFlags(GPIO2,
																	1U << 5U);
	}
	
	if((GPIO_PortGetInterruptFlags(GPIO2)&(1U<<7U))==1U<<7U)
	{
	if(measure_ready2)
		{  
			measure_ready2=0;
			mm2=umsec2+20;
			GPIO_PinSetInterruptConfig(GPIO2,7U,kGPIO_IntRisingEdge);
		}
		else
		{
			measure_ready2=1;	
			umsec2=0;
			GPIO_PinSetInterruptConfig(GPIO2,7U,kGPIO_IntFallingEdge);
		
		}
		GPIO_PortClearInterruptFlags(GPIO2,
																1U << 7U);
	}
	
	
	//��������
		if((GPIO_PortGetInterruptFlags(GPIO2)&(1U<<3U))==1U<<3U)
	{
		key1=1;
		GPIO_PortClearInterruptFlags(GPIO2,
																	1U << 3U);
	}

}





