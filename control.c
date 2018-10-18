#include "control.h"	
#include "arm_math.h"
#include "bsp_flexcan.h"
#include "fsl_debug_console.h"
#include "bsp_uart.h"
#include "fsl_lpuart.h"
#include "bsp_red.h"
#define threshould_x(a,b) ((a>b-2)&&(a<b+4))
#define threshould_y(a,b) ((a>b-2)&&(a<b+4))
#define X_PARAMETER          (0.5f)               
#define Y_PARAMETER           (sqrt(3)/2.f)      
#define L_PARAMETER           (1364.0f)  
#define Kxy_mm                (217.2996f)
#define k_Vz									(20.38f)//(13.66f)
extern uint32_t mm;	 
extern  float ax,ay,rx,ry,juli,jdc;
extern float pos_x;
extern float pos_y;
extern float zangle;
extern float vx,vy,vz,gear0,DST_Z[30];
extern uint8_t dog1,dog2,dog3;
extern uint16_t run_node;
float gear=0.0;
float wd1_x=0.0,wd1_y=0.0,kx=129.8,bx=0.0,ky=-129.9,by=130.0,wd_x=0,wd_y=0;
uint8_t Flag_Target,Flag_Change;                             //��ر�־λ
uint8_t temp1,dja[7]={0x0d,0x0a,0x00,0x00,0x00,0x0a,0x0d};  //��ʱ����
float Voltage_Count,Voltage_All;  //��ѹ������ر���
float Gyro_K=0.004;       //�����Ǳ���ϵ��
extern uint16_t run_node;
extern uint32_t mm1,mm2;
float DIST_X=0,DIST_Y=0;
void updateWD(void)
{
//	wd_x=pos_x-(arm_sin_f32(zangle*PI/180)*kx+bx);
//	wd_y=pos_y-(arm_cos_f32(zangle*PI/180)*ky+by);
	wd_x=-pos_x;
	wd_y=-pos_y;
}
/**************************************************************************
�������ܣ�С���˶���ѧģ��
��ڲ�����X Y Z �����ٶȻ���λ��
����  ֵ����
**************************************************************************/
void Analysis(float Vx,float Vy,float Vz)
{
	float rad=zangle*PI/180.0f;
	static float Lx=0,Ly=0,Lz=0,dx=0,dy=0,Vx1,Vy1;
	static	int32_t Target_1=0,Target_2=0,Target_3=0;
	//Vx��Vy���룬Vx1��Vy1����gear�����ٶ�
	if((Vx==0)&&(Vy==0))
	{Vx1=0;Vy1=0;}
	else if((Vx==0)&&(Vy!=0))
	{
		if(Vy>0)
			Vy1=gear;
		else if(Vy<0)
			Vy1=-gear;
	}
	else
	{	
		Vy1=arm_sin_f32(atan(Vy/Vx))*gear;
		Vx1=arm_cos_f32(atan(Vy/Vx))*gear;
		if(Vx<0)
		{
			Vy1=-Vy1;
			Vx1=-Vx1;
		}
	}
	dx=Vx1-wd1_x;
	dy=Vy1-wd1_y;
	Lx=dx*arm_cos_f32(rad)+dy*arm_sin_f32(rad);
	Ly=dy*arm_cos_f32(rad)-dx*arm_sin_f32(rad); 
	
//	float hg=80.0;
//	int32_t lk=0;
//	lk=hg;
	
	Target_1   = (int32_t)(Lx + k_Vz*Vz);//+gyroz*Gyro_K;
	Target_2   = (int32_t)(-X_PARAMETER*Lx + Y_PARAMETER*Ly + k_Vz*Vz);//+gyroz*Gyro_K;
	Target_3   = (int32_t)(-X_PARAMETER*Lx - Y_PARAMETER*Ly + k_Vz*Vz);//+gyroz*Gyro_K;

	CAN_RoboModule_DRV_Velocity_Mode(1,5000,Target_1);
	CAN_RoboModule_DRV_Velocity_Mode(2,5000,Target_2);
	CAN_RoboModule_DRV_Velocity_Mode(3,5000,Target_3);
}	 				

void aaa(float x,float y)
{
	static float err_x=0,l_err_x=0,ln_err_x=0,dX=0,
				 err_y=0,l_err_y=0,ln_err_y=0,dY=0;
	static float kp,ki,kd,index;
		kp=4.25;
		ki=0.125;
		kd=0.32;
		if((abs(err_x)+abs(err_y))>200.0)
		{
			index=0.0;
		}
		else if((abs(err_x)+abs(err_y))<25.0)
		{
			index=1.0;
		}		
		
		err_x=x-wd_x ;               
		ln_err_x+=index*err_x;
		dX=kp*err_x+ki*index*ln_err_x+kd*(err_x-l_err_x);
		l_err_x=err_x;
		
		err_y=y-wd_y;               
		ln_err_y+=index*err_y;		
		dY=kp*err_y+ki*index*ln_err_y+kd*(err_y-l_err_y);		
		l_err_y=err_y;

	
	    vx=dX;
		vy=dY;
		
	switch((threshould_x(wd_x,x)*1)+(threshould_y(wd_y,y)*2))
	{
		case 1:
			vx=0;
			break;
		case 2:
			vy=0;
			break;
		case 3:
			vx=0;
			vy=0;
			break;
	}
	
		gear=sqrt(vx*vx+vy*vy);
		if(gear>gear0)
			gear=gear0;
		delay_ms(2);
}

void bbb(float x,float y)
{
	static float dx=0,dy=0;
	dx=x-wd_x;
	dy=y-wd_y;
	DIST_X=x;
	DIST_Y=y;
	gear=gear0;
	vx=dx;
	vy=dy;

	if((abs(dx)<400.0)||(abs(dy)<300.0))
	{
		gear=gear0*0.8f;
	}
	 if((abs(dx)<250.0)||(abs(dy)<150.0))
	{
		gear=gear0*0.5;
	}
		if((abs(dx)<50.0)&&(abs(dy)<50.0))
	{
		gear=500;
	}
		switch(run_node)
	{
//		case 2:
//			gear=1000;
//			break;
		case 3:
			gear=1000;
			break;
		case 4:
			gear=1000;
			break;
		case 5:
			gear=1000;
			break;		
	}
	switch((threshould_x(wd_x,x)*1)+(threshould_y(wd_y,y)*2))
	{
		case 1:
			vx=0;
			break;
		case 2:
			vy=0;
			break;
		case 3:
			vx=0;
			vy=0;
			break;
	}
	__DSB();
	//delay_ms(1);
			
}
void WFJian(float fi,float Vz)
{
	float rad=fi*PI/180.0f; 
	float Lx=0,Ly=0,Lz=0,dx=0,dy=0,Vx1,Vy1;
	Lx=1000;
	Ly=1000;
	static	int32_t WTarget_1=0,WTarget_2=0,WTarget_3=0;
	
	WTarget_1   = (int32_t)(Lx*arm_cos_f32(rad) + k_Vz*Vz);//+gyroz*Gyro_K;
	WTarget_2   = (int32_t)(-X_PARAMETER*Lx*arm_cos_f32(rad) + Y_PARAMETER*Ly*arm_sin_f32(rad) + k_Vz*Vz);//+gyroz*Gyro_K;
	WTarget_3   = (int32_t)(-X_PARAMETER*Lx*arm_sin_f32(rad) - Y_PARAMETER*Ly*arm_cos_f32(rad) + k_Vz*Vz);//+gyroz*Gyro_K;

	CAN_RoboModule_DRV_Velocity_Mode(1,5000,WTarget_1);
	CAN_RoboModule_DRV_Velocity_Mode(2,5000,WTarget_2);
	CAN_RoboModule_DRV_Velocity_Mode(3,5000,WTarget_3);
}
/**************************************************************************
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
uint32_t myabs(long int a)
{ 		   
	  uint32_t temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}

void oponeD(void)
{
	dja[2]=0;
    LPUART_WriteBlocking(LPUART3, dja, sizeof(dja));
}
void optwoD(void)
{
	dja[3]=0;
	LPUART_WriteBlocking(LPUART3, dja, sizeof(dja));
}
void opthrD(void)
{
	dja[4]=0;
	LPUART_WriteBlocking(LPUART3, dja, sizeof(dja));
}
void cloneD(void)
{
	dja[2]=1;
	LPUART_WriteBlocking(LPUART3, dja, sizeof(dja));
}
void cltwoD(void)
{
	dja[3]=1;
	LPUART_WriteBlocking(LPUART3, dja, sizeof(dja));
}
void clthrD(void)
{
	dja[4]=1;
	LPUART_WriteBlocking(LPUART3, dja, sizeof(dja));
}
void angleAnalysis(uint8_t hand)
{
	static float Xtmp=0,Ytmp=0,Ztmp=0,zangle1=0,mmp=0;
	Xtmp=DIST_X-wd_x;
	Ytmp=DIST_Y-wd_y; 
	Ztmp=0;
	if(Ytmp<0)
		Ztmp=180;
	zangle1=-((atan(Xtmp/Ytmp)/PI)*180+Ztmp);
	
	if(zangle1>180.0f)
		zangle1-=360;
	if(zangle1<-180.0f)
		zangle1+=360;
//	PRINTF("t7.txt=\"%d\"",run_node);
//	END_SEND();
//	PRINTF("t8.txt=\"%f\"",zangle1);
//	END_SEND();

	
	switch(hand)
	{
		case 1:
			dog1=1;
			mmp=zangle1+120.0f;
			if(mmp>180.0f)
				mmp-=360;
			if(mmp<-180.0f)
				mmp+=360;
			break;
		case 2:
			dog2=1;
			mmp=zangle1 - 120.0f;
			if(mmp>180.0f)
			
			mmp-=360;
			if(mmp<-180.0f)
				mmp+=360;
			break;
		case 3:
			dog3=1;
			mmp=zangle1;
			if(mmp>180.0f)
				mmp-=360;
			if(mmp<-180.0f)
				mmp+=360;
			break;
	}	
	DST_Z[run_node]=mmp;
	__DSB();
//	PRINTF("t9.txt=\"%f\"",mmp);
//	END_SEND();
//	PRINTF("t10.txt=\"%f\"",DST_Z[run_node]);
//	END_SEND();
	
}