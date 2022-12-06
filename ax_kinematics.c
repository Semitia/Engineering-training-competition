#include "ax_kinematics.h"
#include <stdio.h>
#include "ax_delay.h"
#include <math.h>

#define LINE_FIX 0.2
#define ANGLE_FIX 0.01

//变量定义
int32_t  current_count[4] = {0};
double    ticks_per_meter = 500;
double   linear_correction_factor = 1.0;
double   angular_correction_factor = 1.0;
int32_t  wheel_mult[4] = {0};
float  wheel_track_cali = 0.3;

extern int16_t robot_odom[6];
extern int16_t robot_target_speed[3];
enum move_state{stop, forward_back, left_right, spin, arm};
enum move_state state;
u8 _state;
extern double position[3];

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


void my_forward(int16_t* input)
{
	switch(state)
	{
		case stop:
		{
			break;
		}
		case forward_back:
		{
			double len = LINE_FIX * (input[1]+input[3])/2;
			position[0] += len*cos(position[2]);
			position[1] += len*sin(position[2]);
			break;
		}
		case left_right:
		{
			double len = LINE_FIX * (input[1]+input[3])/2;
			position[0] += len*sin(position[2]);
			position[1] += len*cos(position[2]);
			break;
		}
		case spin:
		{
			double ang = ANGLE_FIX * (input[0]-input[1]+input[3])/3;
			position[2] += ang;
			if(position[2] > 6.2832) {position[2] -= 6.2832;}
			break;
		}
		case arm:
		{
			break;
		}
	}

	return;
}

/**
麦克纳姆论正解算
 */
void Mecanum_Forward(int16_t* input, int16_t* output)
{
	//printf("input: %d,%d,%d,%d\r\n",input[0],input[1],input[2],input[3]);
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

void my_inverse(int16_t* input, int16_t* output)
{
	//update the state
	u8 i;
	if(robot_target_speed[0]==666 && robot_target_speed[1]==666 ) state = arm;
	else if(robot_target_speed[0]==0 && robot_target_speed[1]==0 && robot_target_speed[2]==0) state = stop;//stop
	else if(robot_target_speed[0]==0 && robot_target_speed[2]==0) state = forward_back;//forward
	else if(robot_target_speed[1]==0 && robot_target_speed[2]==0) state = left_right;//
	else if(robot_target_speed[0]==0 && robot_target_speed[1]==0) state = spin;//spin
	_state = state;
	
	switch(state)
	{
		case stop:
			for(i=0; i<4; i++) output[i] = 0;
			break;
		case forward_back:
			for(i=0; i<4; i++) output[i] = input[1]/10;
			break;
		case left_right:
			output[0]=-input[0]/10;
			output[1]= input[0]/10;
			output[2]=-input[0]/10;
			output[3]= input[0]/10;
			break;
		case spin:
			output[0]= input[2]/20;
			output[1]=-input[2]/20;
			output[2]=-input[2]/20;
			output[3]= input[2]/20;
			break;
		case arm:
			for(i=0; i<4; i++) output[i] = 0;
			break;
	}
	
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
	
	//update the state
	if(robot_target_speed[0]==0 && robot_target_speed[1]==0 && robot_target_speed[2]==0) state = stop;//stop
	else if(robot_target_speed[0]==0 && robot_target_speed[2]==0) state = forward_back;//forward
	else if(robot_target_speed[0]==0 && robot_target_speed[1]==0) state = spin;//reverse clock
	_state = state;
	
	output[0] = (int16_t)(wheel_velocity[0] * ticks_per_meter/PID_RATE);
	output[1] = (int16_t)(wheel_velocity[1] * ticks_per_meter/PID_RATE);
  output[2] = (int16_t)(wheel_velocity[2] * ticks_per_meter/PID_RATE);
	output[3] = (int16_t)(wheel_velocity[3] * ticks_per_meter/PID_RATE);
	return;
}

