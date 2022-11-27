#include "ax_pid.h"
#include "ax_delay.h"
#include "ax_uart_db.h"
#include "ax_uart_pi.h"

#define PID_SCALE  0.1f  //PID缩放系数
#define PID_INTEGRAL_UP 1000  //积分上限

#define I_up 80
#define I_low 55

/*
int16_t ax_motor_kp=600;  //电机转速PID-P
int16_t ax_motor_ki=0;    //电机转速PID-I
int16_t ax_motor_kd=400;  //电机转速PID-D
*/
int16_t ax_motor_kp=350; 
int16_t ax_motor_ki=120;    //电机转速PID-I
int16_t ax_motor_kd=80; 

/**
  * @简  述  电机A PID控制函数
  * @参  数  spd_target:编码器速度目标值 ,范围（±250）
  *          spd_current: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out = 0;
	static int32_t bias = 0,bias_last = 0,bias_integral = 0;

	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
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
  
	
	//printf("PID_A: tar:%d; cur:%d; output:%d\r\n",spd_target,spd_current,motor_pwm_out);
	//返回PWM控制值
	return motor_pwm_out;
}	

/**
  * @简  述  电机B PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
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
	
	//printf("PID_B: tar:%d; cur:%d; output:%d\r\n",spd_target,spd_current,motor_pwm_out);
	
	//返回PWM控制值
	return motor_pwm_out;
}

int16_t my_abs(int16_t x)
{
	if(x >= 0) {return x;}
	else {return -x;}
}
	
int16_t AX_PID_MotorVelocityCtlB_plus(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out, last_spd_target=0;
	static int32_t bias,bias_last,bias_integral = 0;
	
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
	
	//printf("PID_B: tar:%d; cur:%d; output:%d\r\n",spd_target,spd_current,motor_pwm_out);
	
	//返回PWM控制值
	return motor_pwm_out;
}


/**
  * @简  述  电机C PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlC(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral = bias;
	
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

	//返回PWM控制值
	return motor_pwm_out;
}

/**
  * @简  述  电机D PID控制函数
  * @参  数  spd_target:编码器速度目标值 
  *          spd_target: 编码器速度当前值
  * @返回值  电机PWM速度
  */
int16_t AX_PID_MotorVelocityCtlD(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//获得偏差值
	bias = spd_target - spd_current;
	
	//计算偏差累加值
	bias_integral += bias;
	
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

	//返回PWM控制值
	return motor_pwm_out;
}

/******************* (C) 版权 2019 XTARK **************************************/
