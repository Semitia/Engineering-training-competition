#ifndef __AX_KINEMATICS_H
#define __AX_KINEMATICS_H


/* Includes ------------------------------------------------------------------*/	

#include "stm32f10x.h"


#define ENCODER_MAX 32767        
#define ENCODER_MIN -32768 

#define WHEEL_TRACK             0.213	 //轮距
#define WHEEL_DIAMETER          0.1221  //轮径
#define ENCODER_RESOLUTION      2618.18     //编码器分辨率
#define PID_RATE                50      //PID频率
#define ROBOT_LINEAR_SPEED_LIMIT 5000   //机器人线速度限值 m/s*1000
#define ROBOT_ANGULAR_SPEED_LIMIT 5000  //机器人角速度限值 rad/s*1000

#define ENCODER_LOW_WRAP  ((ENCODER_MAX - ENCODER_MIN)*0.3+ENCODER_MIN)
#define ENCODER_HIGH_WRAP ((ENCODER_MAX - ENCODER_MIN)*0.7+ENCODER_MIN)
#define PI 3.1415926

#define ANGLE_CAL(X) (PI-((X)-1500)/2000*PI)

//ROBOT功能函数
void AX_Kinematics_Init(int16_t* robot_params);  //参数初始化
void AX_Kinematics_Forward(int16_t* input, int16_t* output); //正解(ForwardKinematics)：轮子编码值->底盘三轴里程计坐标
void AX_Kinematics_Inverse(int16_t* input, int16_t* output); //逆解(InverseKinematics)：底盘三轴速度->轮子速度
void Mecanum_Forward(int16_t* input, int16_t* output);
void Mecanum_Inverse(int16_t* input, int16_t* output);

#endif
