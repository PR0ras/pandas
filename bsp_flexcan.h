#ifndef __BSP_FLEXCAN_H
#define __BSP_FLEXCAN_H

#include "fsl_flexcan.h"


#define OpenLoop_Mode                       0x01
#define Current_Mode                        0x02
#define Velocity_Mode                       0x03
#define Position_Mode                       0x04
#define Velocity_Position_Mode              0x05
#define Current_Velocity_Mode               0x06
#define Current_Position_Mode               0x07
#define Current_Velocity_Position_Mode      0x08

void CAN1_Configuration(void);

void my_canInit();
void CAN_RoboModule_DRV_Reset(uint8_t Number);
void CAN_RoboModule_DRV_Mode_Choice(uint8_t Number,uint8_t Mode);
void CAN_RoboModule_DRV_OpenLoop_Mode(uint8_t Number,short Temp_PWM);
void CAN_RoboModule_DRV_Current_Mode(uint8_t Number,short Temp_PWM,short Temp_Current);
void CAN_RoboModule_DRV_Velocity_Mode(uint8_t Number,short Temp_PWM,short Temp_Velocity);
void CAN_RoboModule_DRV_Position_Mode(uint8_t Number,short Temp_PWM,long Temp_Position);
void CAN_RoboModule_DRV_Velocity_Position_Mode(uint8_t Number,short Temp_PWM,short Temp_Velocity,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Mode(uint8_t Number,short Temp_Current,short Temp_Velocity);
void CAN_RoboModule_DRV_Current_Position_Mode(uint8_t Number,short Temp_Current,long Temp_Position);
void CAN_RoboModule_DRV_Current_Velocity_Position_Mode(uint8_t Number,short Temp_Current,short Temp_Velocity,long Temp_Position);

void CAN_RoboModule_DRV_Config(uint8_t Number,uint8_t Temp_Time,uint8_t Ctl1_Ctl2);
void CAN_RoboModule_DRV_Online_Check(uint8_t Number);


void flexcanInit(void);
uint8_t CAN2_Receive_Msg(uint8_t *buf);
uint8_t CAN2_Send_Msg(uint8_t* msg,uint8_t len,uint32_t txId);
void CAN2_IRQHandler(void);



#endif /* __BSP_LED_H */
