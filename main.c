#include "fsl_debug_console.h"
#include "board.h"
#include "pin_mux.h"
#include "clock_config.h"
#include "bsp_flexcan.h"
#include "bsp_key.h"
#include "bsp_uart.h"
#include "bsp_flexpwm.h"
#include "bsp_pit.h"
#include "arm_math.h"
#include "fsl_gpt.h"
#include "control.h"	
#include "math.h"
#include "bsp_led.h"
#include "bsp_key_it.h"
/*******************************************************************
 * Prototypes
 *******************************************************************/
 #define assign(x,y,z) vx=x;vy=y;vz=z;vx_o+=x;vy_o+=y;vz_o+=z;
 #define threshould_xy(a,b) ((a>b-10)&&(a<b+10))
 #define threshould_z(a,b)  ((a>b-1)&&(a<b+1))
 #define threshould_s(a,b)  ((a>b-100)&&(a<b+100)) 
 
	uint8_t res,rxff=0;
	extern uint8_t key;
	extern uint8_t lpuartrx[8];
	int32_t move_x,move_y,move_z,move_v,t_0,Hei;
	uint16_t run_node=0,roa_node=0;
	extern uint32_t sec;
	uint32_t now_time=0;
	uint8_t tim;
	uint16_t gear=1000;
	float vx,vy,vz;
	float vx_o=0,vy_o=0,vz_o=0;
	
	extern float pos_x;
	extern float pos_y;
	extern float zangle;
	extern float xangle;
	extern float yangle;
	extern float w_z;
	extern float wd_x,wd_y;
	extern uint8_t rxflag,rxflag1,data;
	uint8_t once=1,twice=1,thrice=1,back=1;
	float px,py,pz;
	static float
		  DST_X[30]={ -1376.0 , -2327.0 , -3613.0 , -4166.0 ,-3335.0 ,-1416.0 ,	-452,	1244,	1771,	-245},
		  DST_Y[30]={ -340.0  ,  225.0  ,  1284.0 ,  2298.0 , 1949.0 ,  487.0 ,	2528,	3013,	5017,	5382},
		  DST_Z[30]={    0    ,    53.0 ,    0.0  ,  136.0  , -160.0 ,  -24.0 ,	-16 ,	45  ,      0,     60};
	static int32_t 
		  DST_H[30]={    0    ,    0    ,    0    ,  150000 , 9000 ,  -24 },
		  DST_S[30]={    0    ,    0    ,    0    ,  0 , 3000 ,  3000 };
/*******************************************************************
 * Code
 *******************************************************************/
/**
  * @brief  ������
  * @param  ��
  * @retval ��
  */
		  
//void yigedunzi()
//{
//	if(twice)
//	{
//		Analysis(0,0,0);
//		change_height(50000);
//		delay_ms(1000);
//		change_height(90000);
//		delay_ms(1000);
//		change_height(150000);
//		delay_ms(1000);	
//		
//		twice=0;
//		back=1;
//	}
//	if(back)
//	{
//		if(thrice)
//		{
//			now_time = sec;
//			thrice=0;
//		}
//		if(threshould_s(sec,now_time+DST_S[4]))//��ʱ500ms
//		{
//			once=1;
//			twice=1;
//			thrice=1;
//			back=0;
//			run_node++;	
//		}
//		else
//		aaa(DST_X[run_node-1],DST_Y[run_node-1]);	
//	}
//}
void yigedunzi()
{
	if(!thrice)
	{
		if(!twice)
		{
			now_time = sec;
			twice=1;
			back=0;
		}
		if(threshould_s(sec,now_time + 1000))//��ʱ500ms
		{
			once=1;
			twice=1;
			thrice=1;
			back=1;
			run_node++;	
		}

	}//����һ��
	else
	{
		if(!twice)
		{
			change_height(50000);
			delay_ms(1000);
			change_height(90000);
			delay_ms(1000);
			change_height(150000);
			delay_ms(1000);	
			thrice=0;	
		}
		twice=0;
		vx=0;
		vy=0;
		vz=0;
	}
}

void lianggedunzi()
{
	if((!thrice)&&back)
	{
		if(twice)
		{
			now_time = sec;
			twice=0;
			back=1;
		}
		if(threshould_s(sec,now_time + 500))//��ʱ500ms
		{
		once=1;
		twice=1;
		back=0;
		run_node++;	
		}
	}//����һ��
	else
	{
		if((!twice)&&thrice)
		{
			change_height(50000);
			delay_ms(1000);
			change_height(130000);
			delay_ms(1000);
			change_height(150000);
			delay_ms(1000);		
			thrice=0;
			back=1;
		}
		twice=0;
		vx=0;
		vy=0;
		vz=0;
	}
}

int main(void)
{
    /* ��ʼ���ڴ汣����Ԫ */
    BOARD_ConfigMPU();
    /* ��ʼ������������ */
    BOARD_InitPins();
    /* ��ʼ��������ʱ�� */
    BOARD_BootClockRUN();
    /* ��ʼ�����Դ��� */
    BOARD_InitDebugConsole();
    /* ��ӡϵͳʱ�� */
    //PRINTF("CPU:             %d Hz\r\n", CLOCK_GetFreq(kCLOCK_CpuClk));
    /* �����������Ĵ���^_^. */
	
//		vx=0.0;
//		vy=0.0;
//		vz=1000;//2370.0;
		uart_Init();
		uint8_t i=0,key_flag=0;
		uint16_t vel=0;
		
//		change_hight(-100000);
		Key_IT_GPIO_Config();
		LED_GPIO_Config();
		PIT_CH0_Int_Init(500);
//		DUO_Init();
//		delay_ms(500);	

	my_canInit();

	PIT_StartTimer(PIT,kPIT_Chnl_1);        //��PIT
    while(1) 
    {
		if(rxflag)
		{ 
			updateWD();
			PRINTF("t0.txt=\"%d\"",twice );
			END_SEND();
			PRINTF("t1.txt=\"%d\"",thrice);
			END_SEND();
			PRINTF("t2.txt=\"%f\"",zangle);
			END_SEND();
			PRINTF("t7.txt=\"%d\"",run_node);
			END_SEND();
			PRINTF("t8.txt=\"%d\"",back);
			END_SEND();
			PRINTF("t9.txt=\"%d\"",now_time);
			END_SEND();
			PRINTF("t10.txt=\"%d\"",sec);
			END_SEND();
			//PRINTF("x= %f ",wd_x);
			//PRINTF("y= %f ",wd_y);
			//PRINTF("z= %f ",zangle);
			//PRINTF("node= %d ",run_node);
			//PRINTF("dX= %f ", vx);
			//PRINTF("dy= %f ", vy);

			//PRINTF("\r\n");
			rxflag=0;
			rxff=1;
		}

			if(g_KeyDown[CORE_BOARD_WAUP_KEY_ID])
		  {
			  /* ��΢��ʱ */
			  delay(100);
			  /* �ȴ����������ͷ� ���ߵ�ƽ��*/
			  if(1 == GPIO_PinRead(CORE_BOARD_WAUP_KEY_GPIO, CORE_BOARD_WAUP_KEY_GPIO_PIN))
			  {
				  /* ��תLED�ƣ����������Ϣ */
				CORE_BOARD_LED_TOGGLE;
				 Hei=130000;
				change_height(Hei);
			  }
			  /* �������ñ�־λ */
			  g_KeyDown[CORE_BOARD_WAUP_KEY_ID] = false; 
		  }
		  
		  /* MODE�����ı�־ */
		  /* ��g_KeyDownΪtrue�������������� */
		  if(g_KeyDown[CORE_BOARD_MODE_KEY_ID])
		  {
			  delay(100);
			  if(1 == GPIO_PinRead(CORE_BOARD_MODE_KEY_GPIO, CORE_BOARD_MODE_KEY_GPIO_PIN))
			  {
				  CORE_BOARD_LED_TOGGLE;
				  Hei=220000;
				  change_height(Hei);
			  }
			  g_KeyDown[CORE_BOARD_MODE_KEY_ID] = false; 
		 }
				
	//	if(threshould_s(sec,2000))
	//		run_node=1;
	//	if(threshould_s(sec,9000))
	//		run_node=2;	
		if(rxff==1)
		{
			if(back)
				aaa(DST_X[run_node],DST_Y[run_node]);
			else
				aaa(DST_X[run_node-1],DST_Y[run_node-1]);
	//	if(threshould_z(zangle,DST_Z[run_node]))
	//		vz=0;	
	//	else if(once)
	//	{
	//		if(DST_Z[run_node]-zangle>0)
	//			vz=60.0;
	//		else
	//			vz=-60.0;
	//		once=0;		
	//	}
	//	else;
			switch(run_node)//��ʼ����
			{
				case 0:
					vz=0;
					break;
				case 2:			
					vz=0;
					break;
				case 4:
					if(once)
					{
						vz=60.0;
						once=0;
						change_height(150000);
					}					
					if(threshould_z(zangle,DST_Z[run_node]))
						vz=0;
					break;
				default :
					if(once)
					{
						vz=60.0;
						once=0;
					}
					if(threshould_z(zangle,DST_Z[run_node]))
						vz=0;
					break;
				}
			if(!back||threshould_xy(wd_x,DST_X[run_node])&&threshould_xy(wd_y,DST_Y[run_node]))
			{
				switch(run_node)//��ͣ����
				{
					case 4:
						yigedunzi();
						break;
					case 5:
						yigedunzi();
						break;
	//				case 6:
	//					lianggedunzi();
	//					break;
	//				case 7:
	//					lianggedunzi();
	//					break;
	//				case 9:
	//					lianggedunzi();
	//					break;
					case 12:
						vx=0;
						vy=0;
						vz=0;
					default :
						back=1;
						once=1;
						twice=1;
						thrice=1;
						run_node++;
					break;
				}
			}
			Analysis(vx,vy,vz);
			rxff=0;
		}
	}
		
}
/****************************END OF FILE**********************/
