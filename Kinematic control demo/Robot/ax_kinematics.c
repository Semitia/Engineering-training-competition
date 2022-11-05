/**			                                                    
		   ____                    _____ _____  _____        XTARK@塔克创新
		  / __ \                  / ____|  __ \|  __ \  
		 | |  | |_ __   ___ _ __ | |    | |__) | |__) |
		 | |  | | '_ \ / _ \ '_ \| |    |  _  /|  ___/ 
		 | |__| | |_) |  __/ | | | |____| | \ \| |     
		  \____/| .__/ \___|_| |_|\_____|_|  \_\_|     
		    		| |                                    
		    		|_|  OpenCRP 树莓派 专用ROS机器人控制器                                   
									 
  ****************************************************************************** 
  *           
  * 版权所有： XTARK@塔克创新  版权所有，盗版必究
  * 官网网站： www.xtark.cn
  * 淘宝店铺： https://shop246676508.taobao.com  
  * 塔克媒体： www.cnblogs.com/xtark（博客）
	* 塔克微信： 微信公众号：塔克创新（获取最新资讯）
  *      
  ******************************************************************************
  * @作  者  Musk Han@XTARK
  * @版  本  V1.0
  * @日  期  2019-8-8
  * @内  容  机器人运动学解析
  *
  ******************************************************************************
  * @说  明
  *  
  * 
  ******************************************************************************
  */


#include "ax_kinematics.h"
#include <stdio.h>
#include "ax_delay.h"
#include <math.h>


//变量定义
int32_t  current_count[4] = {0};
double    ticks_per_meter = 500;
double   linear_correction_factor = 1.0;
double   angular_correction_factor = 1.0;
int32_t  wheel_mult[4] = {0};
float  wheel_track_cali = 0.3;

extern int16_t robot_odom[6];
extern int16_t robot_target_speed[3];

/**
  * @简  述  机器人运动参数设置
  * @参  数  无
  * @返回值  无
  */
void AX_Kinematics_Init(int16_t* robot_params)
{

	linear_correction_factor    = (float)robot_params[0]/1000;
  angular_correction_factor   = (float)robot_params[1]/1000;
	wheel_track_cali = (float)WHEEL_TRACK/angular_correction_factor;


	robot_odom[0]  = 0;
	robot_odom[1]  = 0;
	robot_odom[2]  = 0;

	ticks_per_meter    = (float)ENCODER_RESOLUTION/((float)WHEEL_DIAMETER*3.1415926*linear_correction_factor);		
}

/**
  * @简  述  逆向运动学解析，底盘三轴速度->轮子速度
  * @参  数  input:  robot_target_speed[]  机器人三轴速度 m/s*1000
  *          output：ax_encoder_delta_target[] 电机期望速度 count
  * @返回值  无
  */
void AX_Kinematics_Inverse(int16_t* input, int16_t* output)
{
	float x_speed   = ((float)input[0])/1000;
	float yaw_speed = ((float)input[2])/1000;
	
	//printf("X_Speed if %f\r\n",x_speed);
	
	static float wheel_velocity[4] = {0};

		wheel_velocity[0] = (x_speed-yaw_speed*wheel_track_cali);
		wheel_velocity[1] = x_speed+yaw_speed*wheel_track_cali;
		wheel_velocity[2] = wheel_velocity[0];
		wheel_velocity[3] = wheel_velocity[1];


	output[0] = (int16_t)(wheel_velocity[0] * ticks_per_meter/PID_RATE);
	output[1] = (int16_t)(wheel_velocity[1] * ticks_per_meter/PID_RATE);
	output[2] = output[0];
	output[3] = output[1];
}

/**
  * @简  述  正向运动学解析，轮子编码值->底盘三轴里程计坐标
  * @参  数  input: ax_encoder[]  编码器累加值
  *          output: robot_odom[] 三轴里程计 x y yaw
  * @返回值  无
  */
void AX_Kinematics_Forward(int16_t* input, int16_t* output)
{
		static double delta_count[2];  
		static double delta_v_ave[3];
		static double delta_v_integral[2];
		static int16_t recv_count[2];
	
	recv_count[0] = -input[0];
	recv_count[1] = input[1];

	
		//编码器计数溢出处理
	for(int i=0;i<2;i++)
	{
			if(recv_count[i] < ENCODER_LOW_WRAP && current_count[i] > ENCODER_HIGH_WRAP)
				wheel_mult[i]++;
			else if(recv_count[i] > ENCODER_HIGH_WRAP && current_count[i] < ENCODER_LOW_WRAP)
				wheel_mult[i]--;
			else
				wheel_mult[i]=0;
	}

	//将编码器数值转化为前进的距离，单位m
	for(int i=0;i<2;i++)
	{	
			delta_count[i] = 1.0*((float)recv_count[i] + wheel_mult[i]*(ENCODER_MAX-ENCODER_MIN)-current_count[i])/ticks_per_meter;
			current_count[i] = recv_count[i];
	}
		//计算底盘x轴变化距离m与Yaw轴朝向变化rad
		delta_v_ave[0] = ((delta_count[1]-delta_count[0])/2.0);  // 线速度
		delta_v_ave[1] = ((delta_count[0]+delta_count[1])/(2*wheel_track_cali));  //角速度
		//计算底盘坐标系下的x轴与Yaw轴的速度
		delta_v_integral[0] = cos(delta_v_ave[1])*delta_v_ave[0];
	  delta_v_integral[1] = -sin(delta_v_ave[1])*delta_v_ave[0];

		//积分计算里程计坐标系(odom_frame)下的机器人X,Y,Yaw轴坐标
		output[0] += (int16_t)((cos((double)output[2]/1000)*delta_v_integral[0] - sin((double)output[2]/1000)*delta_v_integral[1])*1000);
		output[1] += (int16_t)((sin((double)output[2]/1000)*delta_v_integral[0] + cos((double)output[2]/1000)*delta_v_integral[1])*1000);
		output[2] += (int16_t)(delta_v_ave[1]*1000);
		
    //Yaw轴坐标变化范围控制-2Π -> 2Π
		if(output[2] > PI*1000)
			output[2] -= 2*PI*1000;
		else if(output[2] < -PI*1000)
			output[2] += 2*PI*1000;
		
		//发送机器人X轴Yaw轴速度反馈
		output[3] = (int16_t)(delta_v_ave[0]*1000);
		output[4] = 0;
		output[5] = (int16_t)(delta_v_ave[1]*1000);
}

/******************* (C) 版权 2019 XTARK **************************************/

/**

 */
void Mecanum_Forward(int16_t* input, int16_t* output)
{
	static double delta_count[4];  
	static double delta_v_ave[3];
	static double delta_v_integral[2];
	static int16_t recv_count[4];
		
	recv_count[0] = -input[0];
	recv_count[1] = input[1];
	recv_count[2] = -input[2];
	recv_count[3] = input[3];
	
	for(int i=0;i<4;i++)
	{
		if(recv_count[i] < ENCODER_LOW_WRAP && current_count[i] > ENCODER_HIGH_WRAP)
			wheel_mult[i]++;
		else if(recv_count[i] > ENCODER_HIGH_WRAP && current_count[i] < ENCODER_LOW_WRAP)
			wheel_mult[i]--;
		else
			wheel_mult[i]=0;
	}
	
	for(int i=0;i<4;i++)
	{
		delta_count[i] = 1.0*(recv_count[i] + wheel_mult[i]*(ENCODER_MAX - ENCODER_MIN)-current_count[i])/ticks_per_meter;
		current_count[i] = recv_count[i];
	}
	
	delta_v_ave[0] = (delta_count[3] - delta_count[2])/2.0;
	delta_v_ave[1] = (delta_count[2] - delta_count[0])/2.0;
	delta_v_ave[2] = (delta_count[2] + delta_count[1])/(2*wheel_track_cali);
	
	delta_v_integral[0] =  cos(delta_v_ave[2])*delta_v_ave[0] - sin(delta_v_ave[2])*delta_v_ave[1];
	delta_v_integral[1] = -sin(delta_v_ave[2])*delta_v_ave[0] - cos(delta_v_ave[2])*delta_v_ave[1];
	
	delta_v_integral[0] =  cos(delta_v_ave[2])*delta_v_ave[0] - sin(delta_v_ave[2])*delta_v_ave[1];
	delta_v_integral[1] = -sin(delta_v_ave[2])*delta_v_ave[0] - cos(delta_v_ave[2])*delta_v_ave[1];
	
	output[0] += (int16_t)((cos((double)output[2]/1000)*delta_v_integral[0] - sin((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[1] += (int16_t)((sin((double)output[2]/1000)*delta_v_integral[0] + cos((double)output[2]/1000)*delta_v_integral[1])*1000);
	output[2] += (int16_t)(delta_v_ave[2]*1000);

	if(output[2] > PI*1000) output[2] -= 2*PI*1000;
	else if(output[2] < -PI*1000) output[2] += 2*PI*1000;
	
	output[3] = (int16_t)( delta_v_ave[0]*1000);
	output[4] = (int16_t)(-delta_v_ave[1]*1000);
	output[5] = (int16_t)( delta_v_ave[2]*1000);
	
	return;
}

/**
	
 */
void Mecanum_Inverse(int16_t* input, int16_t* output)
{
	//printf("DEBUG: Mecanum_Inverse\r\n");
	float x_speed = ((float)input[0])/1000;
	float y_speed = ((float)input[1])/1000;
	float yaw_speed = ((float)input[2])/1000;
	static float wheel_velocity[4] = {0};
	//printf("X:%f, Y%f, w:%f\r\n",x_speed,y_speed,yaw_speed);
	
	wheel_velocity[0] = -y_speed + x_speed - (wheel_track_cali)*yaw_speed;
	wheel_velocity[1] =  y_speed + x_speed + (wheel_track_cali)*yaw_speed;	
	wheel_velocity[2] =  y_speed + x_speed - (wheel_track_cali)*yaw_speed;	
	wheel_velocity[3] = -y_speed + x_speed + (wheel_track_cali)*yaw_speed;	
	//printf("Wheelvelocity: %f, %f, %f, %f\r\n",wheel_velocity[0],wheel_velocity[1],wheel_velocity[2],wheel_velocity[3]);
	
	output[0] = (int16_t)(wheel_velocity[0] * ticks_per_meter/PID_RATE);
	output[1] = (int16_t)(wheel_velocity[1] * ticks_per_meter/PID_RATE);
  output[2] = (int16_t)(wheel_velocity[2] * ticks_per_meter/PID_RATE);
	output[3] = (int16_t)(wheel_velocity[3] * ticks_per_meter/PID_RATE);
	return;
}

