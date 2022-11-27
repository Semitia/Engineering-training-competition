#include "ax_pid.h"
#include "ax_delay.h"
#include "ax_uart_db.h"
#include "ax_uart_pi.h"

#define PID_SCALE  0.1f  //PID����ϵ��
#define PID_INTEGRAL_UP 1000  //��������

#define I_up 80
#define I_low 55

/*
int16_t ax_motor_kp=600;  //���ת��PID-P
int16_t ax_motor_ki=0;    //���ת��PID-I
int16_t ax_motor_kd=400;  //���ת��PID-D
*/
int16_t ax_motor_kp=350; 
int16_t ax_motor_ki=120;    //���ת��PID-I
int16_t ax_motor_kd=80; 

/**
  * @��  ��  ���A PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ ,��Χ����250��
  *          spd_current: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_PID_MotorVelocityCtlA(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out = 0;
	static int32_t bias = 0,bias_last = 0,bias_integral = 0;

	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral += bias;
	
	//�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
  
	
	//printf("PID_A: tar:%d; cur:%d; output:%d\r\n",spd_target,spd_current,motor_pwm_out);
	//����PWM����ֵ
	return motor_pwm_out;
}	

/**
  * @��  ��  ���B PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_PID_MotorVelocityCtlB(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral += bias;
	
  //�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
	
	//printf("PID_B: tar:%d; cur:%d; output:%d\r\n",spd_target,spd_current,motor_pwm_out);
	
	//����PWM����ֵ
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
	
	//�������
	if(spd_target != last_spd_target) {bias_integral = 0;}
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	if(my_abs(bias) < I_low)
	{ bias_integral += (bias + bias_last)/2; }
	else if(my_abs(bias) < I_up)
	{ bias_integral += ((my_abs(bias) - I_low)/(I_up - I_low))*((bias +  bias_last)/2 ); }
	
  //�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;
	
	//printf("PID_B: tar:%d; cur:%d; output:%d\r\n",spd_target,spd_current,motor_pwm_out);
	
	//����PWM����ֵ
	return motor_pwm_out;
}


/**
  * @��  ��  ���C PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_PID_MotorVelocityCtlC(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral = bias;
	
  //�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;

	//����PWM����ֵ
	return motor_pwm_out;
}

/**
  * @��  ��  ���D PID���ƺ���
  * @��  ��  spd_target:�������ٶ�Ŀ��ֵ 
  *          spd_target: �������ٶȵ�ǰֵ
  * @����ֵ  ���PWM�ٶ�
  */
int16_t AX_PID_MotorVelocityCtlD(int16_t spd_target, int16_t spd_current)
{
	static int16_t motor_pwm_out;
	static int32_t bias,bias_last,bias_integral = 0;
	
	//���ƫ��ֵ
	bias = spd_target - spd_current;
	
	//����ƫ���ۼ�ֵ
	bias_integral += bias;
	
  //�����ֱ���
	if(bias_integral>PID_INTEGRAL_UP)bias_integral = PID_INTEGRAL_UP;
	if(bias_integral<-PID_INTEGRAL_UP)bias_integral = -PID_INTEGRAL_UP;
	
	//PID���������PWMֵ
	motor_pwm_out = ax_motor_kp*bias*PID_SCALE + ax_motor_kd*(bias-bias_last)*PID_SCALE + ax_motor_ki*bias_integral*PID_SCALE;
	
	//��¼�ϴ�ƫ��
	bias_last = bias;
	
	//����������
	if(motor_pwm_out > 2000)
		motor_pwm_out = 2000;
	if(motor_pwm_out < -2000)
		motor_pwm_out = -2000;

	//����PWM����ֵ
	return motor_pwm_out;
}

/******************* (C) ��Ȩ 2019 XTARK **************************************/
