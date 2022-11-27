/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PID_H
#define __AX_PID_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP功能函数
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current);   //PID控制函数，电机A
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current);   //PID控制函数，电机B
int16_t AX_PID_MotorVelocityCtlC(int16_t spd_target, int16_t spd_current);   //PID控制函数，电机C
int16_t AX_PID_MotorVelocityCtlD(int16_t spd_target, int16_t spd_current);   //PID控制函数，电机D

int16_t AX_PID_MotorVelocityCtlB_plus(int16_t spd_target, int16_t spd_current);

#endif


