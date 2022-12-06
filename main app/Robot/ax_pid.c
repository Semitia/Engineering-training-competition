#include "ax_pid.h"
#include "ax_delay.h"
#include "ax_uart_db.h"
#include "ax_uart_pi.h"
#include "DataScope_DP.h"

#define PID_SCALE  0.1f  //PID缩放系数
#define PID_INTEGRAL_UP 1000  //积分上限

#define I_up 80
#define I_low 55

/*
int16_t ax_motor_kp=600;  //电机转速PID-P
int16_t ax_motor_ki=0;    //电机转速PID-I
int16_t ax_motor_kd=400;  //电机转速PID-D
*/
int16_t ax_motor_kp=150; 
int16_t ax_motor_ki=120;    //电机转速PID-I
int16_t ax_motor_kd=80; 

u8 Send_Count,s;


int16_t my_abs(int16_t x)
{
	if(x >= 0) {return x;}
	else {return -x;}
}


int16_t AX_PID_MotorVelocityCtlA_plus(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out, last_spd_target=0;
	static int32_t bias,bias_last,bias_integral = 0;
	float bias_f,I_f;
	
	//积分清除
	if(spd_target != last_spd_target) {bias_integral = 0;}
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	if(my_abs(bias) < I_low)
	{ bias_integral += (bias + bias_last)/2; }
	else if(my_abs(bias) < I_up)
	{ bias_integral += ((my_abs(bias) - I_low)/(I_up - I_low))*((bias +  bias_last)/2 ); }
	
  //抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
	
/*
		bias_f = (float)bias;
		DataScope_Get_Channel_Data(bias_f,1);
		Send_Count=DataScope_Data_Generate(1);
		for( s = 0 ; s < Send_Count; s++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[s]; 
		}
*/
	
	//返回PWM控制值
	return motor_pwm_out;
}

int16_t AX_PID_MotorVelocityCtlB_plus(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out, last_spd_target=0;
	static int32_t bias,bias_last,bias_integral = 0;
	float bias_f,I_f;
	
	//积分清除
	if(spd_target != last_spd_target) {bias_integral = 0;}
	last_spd_target = spd_target;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	if(my_abs(bias) < I_low)
	{ bias_integral += (bias + bias_last)/2; }
	else if(my_abs(bias) < I_up)
	{ bias_integral += ((my_abs(bias) - I_low)/(I_up - I_low))*((bias +  bias_last)/2 ); }
	
  //抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
	
/*
		bias_f = (float)bias;
		DataScope_Get_Channel_Data(bias_f,1);
		Send_Count=DataScope_Data_Generate(1);
		for( s = 0 ; s < Send_Count; s++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[s]; 
		}
*/
	
	
	//返回PWM控制值
	return motor_pwm_out;
}

int16_t AX_PID_MotorVelocityCtlC_plus(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out, last_spd_target=0;
	static int32_t bias,bias_last,bias_integral = 0;
	float bias_f,I_f;
	
	//积分清除
	if(spd_target != last_spd_target) {bias_integral = 0;}
	last_spd_target = spd_target;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	if(my_abs(bias) < I_low)
	{ bias_integral += (bias + bias_last)/2; }
	else if(my_abs(bias) < I_up)
	{ bias_integral += ((my_abs(bias) - I_low)/(I_up - I_low))*((bias +  bias_last)/2 ); }
	
  //抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
	
/*
		bias_f = (float)bias;
		DataScope_Get_Channel_Data(bias_f,1);
		Send_Count=DataScope_Data_Generate(1);
		for( s = 0 ; s < Send_Count; s++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[s]; 
		}
*/
	
	//返回PWM控制值
	return motor_pwm_out;
}

int16_t AX_PID_MotorVelocityCtlD_plus(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out, last_spd_target=0;
	static int32_t bias,bias_last,bias_integral = 0;
	float bias_f,I_f;
	
	//积分清除
	if(spd_target != last_spd_target) {bias_integral = 0;}
	last_spd_target = spd_target;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	if(my_abs(bias) < I_low)
	{ bias_integral += (bias + bias_last)/2; }
	else if(my_abs(bias) < I_up)
	{ bias_integral += ((my_abs(bias) - I_low)/(I_up - I_low))*((bias +  bias_last)/2 ); }
	
  //抗积分饱和
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID计算电机输出PWM值
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//记录上次偏差
	bias_last = bias;
	
	//限制最大输出
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
	
/*
		bias_f = (float)bias;
		DataScope_Get_Channel_Data(bias_f,1);
		Send_Count=DataScope_Data_Generate(1);
		for( s = 0 ; s < Send_Count; s++) 
		{
		while((USART1->SR&0X40)==0);  
		USART1->DR = DataScope_OutPut_Buffer[s]; 
		}
*/
	//printf("input:%d, currentspeed:%d, output:%d\r\n",spd_target,spd_current,motor_pwm_out);
	
	//返回PWM控制值
	return motor_pwm_out;
}


