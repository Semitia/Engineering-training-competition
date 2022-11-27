/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __AX_PID_H
#define __AX_PID_H

/* Includes ------------------------------------------------------------------*/	 
#include "stm32f10x.h"

//OpenCRP���ܺ���
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������A
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������B
int16_t AX_PID_MotorVelocityCtlC(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������C
int16_t AX_PID_MotorVelocityCtlD(int16_t spd_target, int16_t spd_current);   //PID���ƺ��������D

int16_t AX_PID_MotorVelocityCtlB_plus(int16_t spd_target, int16_t spd_current);

#endif


