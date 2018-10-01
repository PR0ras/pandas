#ifndef __CONTROL_H
#define __CONTROL_H
#include "fsl_common.h"
#define PI 3.14159
void Analysis(float Vx,float Vy,float Vz);
void move(int32_t target_1,int32_t target_2,int32_t target_3,int16_t tt);
void updateWD(void);
#endif
