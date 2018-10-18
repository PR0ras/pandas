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
#include "bsp_red.h"
/*******************************************************************
 * Prototypes
 *******************************************************************/
#define threshould_xy(a,b) ((a>b-7)&&(a<b+7))
#define threshould_z(a,b)  ((a>b-0.5)&&(a<b+1))
#define threshould_s(a,b)  ((a>b-50)&&(a<b+50)) 
#define spcnum 8
#define angle_speed 80
enum 
{
	H1 = 0,
	H2 = 1,
	W2 = 2,
	W1 = 3,
	N1 = 4,
	N2 = 5,
	N3 = 6,
	N4 = 7,
	N5 = 8,
}Myenum;
void yigedunzi(void);
void lianggedunzi(void);
void END_node(void);
void timer_delay(uint16_t tim);
uint8_t priority[10]={1,1,1,1,1,1,1,1,1,1},ftmp=0,DZflag=20;
uint8_t res,rxff=0;
extern uint8_t key;
uint8_t KEY1=0,KEY2=0;
uint16_t key1_affirm=0,key2_affirm=0;
extern uint8_t lpuartrx[8];
int32_t move_x,move_y,move_z,move_v,t_0,Hei=0;
uint16_t run_node=0,roa_node=0;
extern uint32_t sec;
uint32_t now_time=0;
uint32_t bk_tim=700;
uint8_t dog1=0,dog2=0,dog3=0;
float gear0=2500.0;
float vx,vy,vz,vx_old=0,vy_old=0,keyflag=1;
float vx_o=0,vy_o=0,vz_o=0;
extern uint32_t mm1,mm2;
extern float pos_x;
extern float pos_y;
extern float zangle;
extern float xangle;
extern float yangle;
extern float w_z;
extern float wd_x,wd_y;
uint8_t key1,key2,key3;
extern uint8_t rxflag,rxflag1,data;
float d_z;
uint8_t back=1;
float XMDZ[2][spcnum] ={{-2372.0, -4142, -2981, -1332.0,-505.0, 988.0,-316.0, 675},
						{ 437.0 ,2814.0, 2035 ,   417.0,2956.0,3381.0,5228.0,5063}},px,py,pz;
float
//	  DST_X[30]={ -1467  , XMDZ[0][H1] , -3754.0 , -4142.0 ,-2660.0 , -3225.5 , -1332.0, -352.0,  1354, 911 , 2140 ,  -316 ,417,675,0,-4720},
//	  DST_Y[30]={ -294.0 ,   437.0 ,  1329.0 ,  2814.0 , 2308.0 ,  1884.5 ,   417.0, 2974.0,  2423, 3347 , 5557 , 5228,5341,5063,0,-172},
////	  DST_Z[30]={    0   ,    51.0 ,    51.0 ,   138.0 , 132.0  ,  131.0  ,  -147.0,   83.0, 144.0, 142.0,-142.0, -142,-138,-138,0,175},
	  DST_Z[30]={ 0 };
//	  XMDZ[2][10] ={{-2987,0},{-1416,0},{-552,0},{1051,0},{-126,0}};
int32_t
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
//	vx=0.0;
//	vy=0.0;
//	vz=1000;//2370.0;
	float
	  DST_X[30]={ -1467  , XMDZ[0][H1] , -3754.0 , XMDZ[0][H2] ,-2660.0 , XMDZ[0][W2] , XMDZ[0][W1], XMDZ[0][N1],  1354, XMDZ[0][N2] , 2140 , XMDZ[0][N3],  417 ,  0 ,-4720},
	  DST_Y[30]={ -294.0 , XMDZ[1][H1] ,  1329.0 , XMDZ[1][H2] , 2308.0 , XMDZ[1][W2] , XMDZ[1][W1], XMDZ[1][N1],  2423, XMDZ[1][N2] , 5557 , XMDZ[1][N3], 5341 , 0 ,-172};
	uint8_t hha[7]={0x0d,0x0a,0x00,0x00,0x00,0x0a,0x0d};
	uart_Init();
	Key_IT_GPIO_Config();
	touch_key_init();
	LED_GPIO_Config();
	PIT_CH0_Int_Init(500);
	GPT1_Int_Init(750);
	red_gpio_init();
	uint8_t ii=0;
//		delay_ms(500);	
	my_canInit();
	
	//WFJian(90,0);
	//CAN_RoboModule_DRV_Velocity_Mode(0,5000,0 );
	
	//change_height(180000);
	//���
	hha[2]=0;
	hha[3]=0;
	hha[4]=0;
	LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
	LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
	key=0;
    while(1) 
    {
		key1=GPIO_PinRead(GPIO2, 27);
		key2=GPIO_PinRead(GPIO2, 3);//��ѯ�������صĵ�ƽ
		//������ȷ�ϰ���
		if(!key1)
		{
			if(!key1&&(key1_affirm++)>10000)//�ȴ�100��ѭ��
			{
				KEY1=1;
				key1_affirm=0;
			}
			__DSB();
		}
		else
			key1_affirm=0;
		
		if(!key2)
		{
			if(!key2&&(key2_affirm++)>10000)
			{
				KEY1=1;
				key2_affirm=0;
			}
			__DSB();
		}
		else
			key2_affirm=0;
		
		if(rxflag)//��ȷ�յ���λ������
		{ 
			updateWD();//����������µ���������	
			PRINTF("t0.txt=\"%d\"",Hei );
			END_SEND();//��ʾ����ָ��
			PRINTF("t1.txt=\"%f\"",wd_y);
			END_SEND();
			PRINTF("t2.txt=\"%f\"",zangle);
			END_SEND();
//			PRINTF("t7.txt=\"%d\"",KEY1);
//			END_SEND();
//			PRINTF("t8.txt=\"%d\"",KEY2);
//			END_SEND();
			//PRINTF("x= %f ",wd_x);
			//PRINTF("y= %f ",wd_y);
			//PRINTF("z= %f ",zangle);
			//PRINTF("node= %d ",run_node);
			//PRINTF("dX= %f ", vx);
			//PRINTF("dy= %f ", vy);
			//PRINTF("\r\n");
			rxflag=0;
			rxff=1;//��ʼ�������ִ꣬���˶��㷨������
		}

			if(key==1)//���İ尴��WAUP
		  {
			  hha[2]=0;
			  hha[3]=0;
			  hha[4]=0;
			  digitalToggle(GPIO1,9);
			  LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
			  //delay(20);
//			  if(1 == GPIO_PinRead(CORE_BOARD_WAUP_KEY_GPIO, CORE_BOARD_WAUP_KEY_GPIO_PIN))
//			  {
				  /* ��תLED�ƣ����������Ϣ */
				//CORE_BOARD_LED_TOGGLE;
//				 Hei+=10000;
//				change_height(89000);
//			  }
			  /* �������ñ�־λ */
			  key = 0; 
		  }

		  if(key==2)//���İ尴��MODE
		  {
			  hha[2]=1;
			  hha[3]=1;
			  hha[4]=1;
			  digitalToggle(GPIO1,9);
			  LPUART_WriteBlocking(LPUART3, hha, sizeof(hha));
			  //delay(20);
//			  if(1 == GPIO_PinRead(CORE_BOARD_MODE_KEY_GPIO, CORE_BOARD_MODE_KEY_GPIO_PIN))
//			  {
				  //CORE_BOARD_LED_TOGGLE;
//			  if(Hei)
//				  Hei-=10000;
//				  change_height(130000);
//			  }
			  key = 0; 
		 }
		  
		if(rxff==1)//�˶�����
		{
			for(ii=0;ii<spcnum;ii++)
			{
				if(DST_X[run_node]==XMDZ[0][ii])//�ж��ֽڵ�Ŀ�������Ƿ�Ϊ��������
					DZflag=ii;//�ڼ�����������
			}

			if(back)//�����˶�
				bbb(DST_X[run_node],DST_Y[run_node]);
			else//����һ��
				aaa(DST_X[run_node-1],DST_Y[run_node-1]);
			
			if(priority[5])
			{
				sec=0;
				priority[5]=0;
				now_time = sec;
			}
			else if(threshould_s(sec,now_time + 300))
			{
				priority[0]=0;
				__DSB();
			}

			if((abs(DST_X[run_node]-wd_x)>150.0)&&(abs(DST_Y[run_node]-wd_y)>150.0))
				{
					switch(DZflag)
					{
						case H1:
							angleAnalysis(3);
							break;
						case H2:
							angleAnalysis(1);
							break;
						case W2:
							angleAnalysis(3);
							break;
						case W1:
							angleAnalysis(3);
							break;
						case N1:
							angleAnalysis(1);
							break;
						case N2:
							angleAnalysis(1);
							break;
						case N3:
							angleAnalysis(3);
							break;
						case N4:
							angleAnalysis(3);
							break;
					}
				}
				
			if(priority[0]!=1)//һ���˶��ڵ�ִֻ��һ��
			{//�Ƕ�����ת�������ٶ�ͳһ��ֵ60��/��
				d_z=DST_Z[run_node ]-zangle;
				if(d_z>0)
				{
					if(abs(d_z)>180)
						vz=-angle_speed;
					else
						vz=angle_speed;
				}
				else
				{
					if(abs(d_z)>180)
						vz=angle_speed;
					else
						vz=-angle_speed;
				}
				
				vx_old=vx;
				vy_old=vy;
				priority[0]=1;

				if(run_node==3)//�ڵ�3��Ҫ��߽��ٶ�
					vz=angle_speed*1.5f;
			}
			
			if(threshould_z(zangle,DST_Z[run_node]))
				vz=0;		//����Ŀ��ǶȺ���ٶ���0	
			if(DZflag==W2)
				vz=0;
			if(run_node&&priority[1])//��¼�µ�ǰǰ������ִֻ��һ�Σ�
				{
					vx_old=vx;
					vy_old=vy;
					priority[1]=0;
				}
			if(DZflag!=20&&DZflag!=H1&&DZflag!=H2)//��������δ��������ǰ��
			{
				if(!KEY1)
				{
					vy=vy_old;
					vx=vx_old;//��֮ǰ�ķ������ǰ��
					keyflag=1;
				}
				else
					keyflag=0;
				__DSB();
			}
			
			
		if(!keyflag||!back||(threshould_xy(wd_x,DST_X[run_node])&&threshould_xy(wd_y,DST_Y[run_node])))//X,Y�����㵱ǰ�ڵ�����
			switch(DZflag)//����㴦��
			{
				case H1:
					clthrD();
					priority[9]=0;
					break;
				case H2:
					cloneD();
					delay_ms(500);
					change_height(180000);//��ߵ�
					delay_ms(300);
					priority[9]=0;
					break;
				case W2:
					priority[8]=0;//���ô�������
					yigedunzi();
					break;
				case W1:
					yigedunzi();
					break;
				case N1:
					lianggedunzi();
					break;
				case N2:
					lianggedunzi();
					break;
				case N3:
					lianggedunzi();
					break;
				case N4:
					lianggedunzi();
					break;
				case 8:
					if(!priority[8]||KEY1)
					{
						priority[9]=0;
					}
					break;
				case 20:
					if(run_node==14)
					{
						vx=0;
						vy=0;
						vz=0;
					}
					else
						priority[9]=0;//������һ�ڵ�
			}
			if(!priority[9])//�жϽڵ���ֹ��־λ
				END_node();
			Analysis(vx,vy,vz);//������õ����������������
			rxff=0;
		}
	}
		
}
	
void yigedunzi(void)
{
	if(!priority[3])
	{
		if(!priority[2])
		{

			sec=0;
			now_time = sec;
			priority[2]=1;
			back=0;
		}
		else if(threshould_s(sec,now_time + bk_tim))//��ʱ500ms
		{
			priority[9]=0;
		}
	}//����һ��
	else
	{
		if(!priority[2])
		{

			change_height(65000);
			delay_ms(700);
			if(dog1)
				oponeD();
			if(dog2)
				optwoD();
			if(dog3)
				opthrD();
			//delay_ms(100);
			delay_ms(200);
			change_height(89000);
			//delay_ms(500);
			delay_ms(700);
			if(dog1)
				cloneD();
			if(dog2)
				cltwoD();
			if(dog3)
				clthrD();
//			delay_ms(700);
			delay_ms(700);
			change_height(180000);
			//delay_ms(100);	
			delay_ms(800);
			priority[3]=0;			
		}
		
		if(!priority[8]||KEY1)
		{
			priority[2]=0;
			vx=0;
			vy=0;
			vz=0;
		}
	}
}

void lianggedunzi()
{
	if(!priority[3])
	{
		if(!priority[2])
		{
			sec=0;
			now_time = sec;
			priority[2]=1;
			back=0;
		}
		else if(threshould_s(sec,now_time + bk_tim))//��ʱ500ms
		{
			priority[9]=0;
		}
	}//����һ��
	else
	{
		if(!priority[2])
		{
			change_height(65000);
//			delay_ms(500);
			delay_ms(700);
			if(dog1)
				oponeD();
			if(dog2)
				optwoD();
			if(dog3)
				opthrD();
//			delay_ms(100);
			delay_ms(200);
			change_height(130000);
//			delay_ms(500);
			delay_ms(800);
			if(dog1)
				cloneD();
			if(dog2)
				cltwoD();
			if(dog3)
				clthrD();
//			delay_ms(700);
			delay_ms(700);
			change_height(180000);
//			delay_ms(700);		
			delay_ms(800);
			priority[3]=0;	
		}
		if(KEY1)
			{
				priority[2]=0;
				vx=0;
				vy=0;
				vz=0;
			}
	}
}

void END_node(void)
{
	for(ftmp=0;ftmp<10;ftmp++)
	{
		priority[ftmp]=1;
	}
	back=1;
	dog1=0;
	dog2=0;
	dog3=0;
	KEY1=0;
	keyflag=1;
	DZflag=20;
	DST_Z[run_node+1]=DST_Z[run_node];
	run_node++;	
	
	__DSB();
	
}

void timer_delay(uint16_t tim)
{
		sec=0;
	now_time = sec;
	while(sec<now_time + tim)
		;
}

/****************************END OF FILE**********************/
