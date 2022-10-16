#ifndef __TIMER_H
#define __TIMER_H
#include "sys.h"

extern u8 swing_flag;
/*
extern float angle_pitch[1000],angle_roll[1000],angle_yall[1000];
extern short w_x[1000],w_y[1000],w_z[1000];
*/
void TIM3_Init(u16 arr,u16 psc);
void TIM3_IRQHandler(void);
void TIM2_CH1_PWM_Init(u16 arr,u16 psc); 
#endif
