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
  * @日  期  2019-7-26
  * @内  容  机器人控制主函数
  * 
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "stm32f10x.h"
#include <stdio.h>
#include <math.h>   

#include "ax_sys.h"    //系统设置
#include "ax_delay.h"  //软件延时
#include "ax_led.h"    //LED灯控制
#include "ax_vin.h"    //输入电压检测
#include "ax_uart_db.h"  //调试串口
#include "ax_uart_pi.h"  //树莓派串口
#include "ax_motor.h"    //直流电机调速控制
#include "ax_encoder.h"  //编码器控制
#include "ax_tim.h"      //定时器
#include "ax_mpu6050.h"  //IMU加速度陀螺仪测量
#include "ax_mpu6050_dmp.h"  //DMP功能函数
#include "ax_pid.h"        //PID控制
#include "ax_kinematics.h" //运动学解析


#define ENCODER_MID_VALUE  30000  
#define VBAT_VOL_CHG    1050 
#define VBAT_VOL_OFF    990   


int16_t ax_encoder[4];	//编码器累加值
int16_t ax_encoder_delta[4];	//编码器变化值
int16_t ax_encoder_delta_target[4] = {0};  //编码器目标变化值
int16_t robot_odom[6] = {0}; //里程计数据，绝对值和变化值，x y yaw dx dy dyaw
int16_t ax_motor_pwm[4];  //电机PWM
uint16_t ax_bat_vol;  //电池电压
int16_t mpu_data[10];  //陀螺仪，加速度，姿态角

int16_t robot_target_speed[3] = {0};  //机器人目标速度 X Y Yaw
int16_t robot_params[2] = {1000,1000};  //机器人参数

u16 count=0;
u8 flag=0;


//主要函数声明
void AX_ROBOT_GetImuData(void);  //读取MPU6050数据
void AX_ROBOT_MoveCtl(void);  //机器人运动控制函数  
void AX_ROBOT_BatteryManagement(void);  //机器人电池管理
void AX_ROBOT_SendDataToPi(void);  //机器人发送数据到树莓派

/**
  * @简  述  程序主函数
  * @参  数  无
  * @返回值  无
  */
int main(void)
{
	uint8_t cnt = 1;  //定时器计数

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);   //设置中断分组

	//机器人初始化
	AX_MOTOR_Init(10);
	AX_DELAY_Init();  
	AX_JTAG_Set(JTAG_SWD_DISABLE);     
	AX_JTAG_Set(SWD_ENABLE);          
	AX_LED_Init();
	AX_VIN_Init();
	AX_UART_DB_Init(115200);
	AX_UART_PI_Init(115200);

	//编码器初始化
	AX_ENCODER_AB_Init(ENCODER_MID_VALUE*2);  
	AX_ENCODER_CD_Init(ENCODER_MID_VALUE*2);  
	AX_ENCODER_EF_Init(ENCODER_MID_VALUE*2);  
	AX_ENCODER_GH_Init(ENCODER_MID_VALUE*2);  
	AX_ENCODER_AB_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_CD_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_EF_SetCounter(ENCODER_MID_VALUE); 
	AX_ENCODER_GH_SetCounter(ENCODER_MID_VALUE);
	
  //开机提示信息
	AX_LED_Red_On();
	AX_LED_Green_On();
	AX_Delayms(100);	
	AX_LED_Red_Off();
	AX_LED_Green_Off();
	AX_Delayms(300);

	AX_MPU6050_Init();    //MPU6050初始化
	AX_MPU6050_DMP_Init();	//DMP初始化
	AX_TIM6_Init(10000);
  AX_TIM6_Cmd(ENABLE);
	
	
  //绿灯点亮，提示运行
  AX_LED_Green_On();
	robot_target_speed[0]=500;

	while(1)
	{
		//100HZ控制频率
		if(AX_TIM6_CheckIrqStatus())
		{		
			//机器人获取PMU6050数据
			AX_ROBOT_GetImuData();
			
			//50HZ执行频率
			if(cnt%2 == 0)
			{
				
				//机器人运动控制
				AX_ROBOT_MoveCtl();
			
				//机器人电量管理
				//AX_ROBOT_BatteryManagement();
			
				//机器人发送数据到树莓派
				AX_ROBOT_SendDataToPi();
			}
			
			//计数器累加
			cnt++;
		}
	}
}

/**
  * @简  述  机器人获取MPU6050 加速度陀螺仪姿态数据
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_GetImuData(void)
{
	  //变量定义，舵机角度
    AX_MPU6050_DMP_GetData(mpu_data);
	
}
/**
  * @简  述  机器人运动控制函数
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_MoveCtl(void)
{ 
	  //robot_target_speed[0] = 500;
	  //获取编码器变化值
		ax_encoder_delta[0] = (AX_ENCODER_AB_GetCounter()-ENCODER_MID_VALUE);
		ax_encoder_delta[1] = -(AX_ENCODER_CD_GetCounter()  -ENCODER_MID_VALUE);
		ax_encoder_delta[2] = (AX_ENCODER_EF_GetCounter()-ENCODER_MID_VALUE);
		ax_encoder_delta[3] = -(AX_ENCODER_GH_GetCounter()  -ENCODER_MID_VALUE);
		
	  //设置编码器中间值
		AX_ENCODER_AB_SetCounter(ENCODER_MID_VALUE);
		AX_ENCODER_CD_SetCounter(ENCODER_MID_VALUE);
		AX_ENCODER_EF_SetCounter(ENCODER_MID_VALUE);
		AX_ENCODER_GH_SetCounter(ENCODER_MID_VALUE);
		
	  //计算编码器累加值
		ax_encoder[0] = ax_encoder[0] + ax_encoder_delta[0];
		ax_encoder[1] = ax_encoder[1] + ax_encoder_delta[1];
		ax_encoder[2] = ax_encoder[2] + ax_encoder_delta[2];
		ax_encoder[3] = ax_encoder[3] + ax_encoder_delta[3];
		
		//麦克纳姆轮运动学解析
		Mecanum_Forward(ax_encoder, robot_odom);
		Mecanum_Inverse(robot_target_speed, ax_encoder_delta_target); //逆向运动学解析

		//电机PID速度控制
		ax_motor_pwm[0] = AX_PID_MotorVelocityCtlA(ax_encoder_delta_target[0], ax_encoder_delta[0]);   
		ax_motor_pwm[1] = AX_PID_MotorVelocityCtlB(ax_encoder_delta_target[1], ax_encoder_delta[1]);   
		ax_motor_pwm[2] = AX_PID_MotorVelocityCtlC(ax_encoder_delta_target[2], ax_encoder_delta[2]);   
		ax_motor_pwm[3] = AX_PID_MotorVelocityCtlD(ax_encoder_delta_target[3], ax_encoder_delta[3]);  
					
		count++;
		if(count%20==0)
		{
			//count = 0;
			printf("count: %d\r\n",count);
			printf("target_speed: X:%d; Y:%d; w:%d\r\n",robot_target_speed[0],robot_target_speed[1],robot_target_speed[2]);
			printf("four wheels: %d; %d; %d;% d\r\n",ax_encoder_delta_target[0],ax_encoder_delta_target[1],ax_encoder_delta_target[2],ax_encoder_delta_target[3]);
			printf("set_PWM: %d; %d; %d; %d\r\n",ax_motor_pwm[0],ax_motor_pwm[1],ax_motor_pwm[2],ax_motor_pwm[3]);
		}
		
		if(count>=500)
		{
			count = 0;
			if(flag) robot_target_speed[0] = 500;
			else robot_target_speed[0] = 0;
			
			flag = !flag;
		}
		
		AX_MOTOR_A_SetSpeed(ax_motor_pwm[0]);
		AX_MOTOR_B_SetSpeed(ax_motor_pwm[1]);  
		AX_MOTOR_C_SetSpeed(-ax_motor_pwm[2]);
		AX_MOTOR_D_SetSpeed(-ax_motor_pwm[3]); 
		
}

/**
  * @简  述  机器人电池管理
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_BatteryManagement(void)
{
	//计数变量
	static uint16_t ax_bat_vol_cnt = 0;   
	
	//采集电池电压
	ax_bat_vol = AX_VIN_GetVol_X100();			
	
	if(ax_bat_vol < VBAT_VOL_CHG)  //1050
	{

		AX_LED_Red_Toggle();
		
		if(ax_bat_vol < VBAT_VOL_OFF) //990
		{
			ax_bat_vol_cnt++;
      
			//电压持续小于VBAT_VOL_OFF 4秒，进入停机保护状态
			if(ax_bat_vol_cnt > 200 )
			{
				
				//红灯常亮，进入停机状态，电机停止转动
				AX_LED_Red_On();
				AX_LED_Green_Off();
				AX_MOTOR_A_SetSpeed(0);  
				AX_MOTOR_B_SetSpeed(0);  
				AX_MOTOR_C_SetSpeed(0);  
				AX_MOTOR_D_SetSpeed(0);  
				
				//报警
				while(1)
				{	
					AX_Delayms(100);
				}								
			}
		}
		else
		{
			ax_bat_vol_cnt = 0;
		}
	}
	else
	{
		AX_LED_Red_Off();
	}
}


/**
  * @简  述  机器人发送数据到树莓派
  * @参  数  无
  * @返回值  无
  */
void AX_ROBOT_SendDataToPi(void)
{
	    //串口发送数据
			static uint8_t comdata[60]; 			
	
			//陀螺仪角速度 = (ax_gyro/32768) * 2000 ?s
			comdata[0] = (u8)( mpu_data[0] >> 8 );  
			comdata[1] = (u8)( mpu_data[0] );
			comdata[2] = (u8)( mpu_data[1] >> 8 );
			comdata[3] = (u8)( mpu_data[1] );
			comdata[4] = (u8)( mpu_data[2] >> 8 );
			comdata[5] = (u8)( mpu_data[2] );
			
			//加速度 = (ax_acc/32768) * 2G  
			comdata[6] = (u8)( mpu_data[3] >> 8 );
			comdata[7] = (u8)( mpu_data[3] );
			comdata[8] = (u8)( mpu_data[4] >> 8 );
			comdata[9] = (u8)( mpu_data[4] );
			comdata[10] = (u8)( mpu_data[5] >> 8 );
			comdata[11] = (u8)( mpu_data[5] );
			
			//姿态角度 = (ax_angle/100)
			comdata[12] = (u8)( mpu_data[6] >> 8 ); 
			comdata[13] = (u8)( mpu_data[6] );
			comdata[14] = (u8)( mpu_data[7] >> 8 );
			comdata[15] = (u8)( mpu_data[7] );
			comdata[16] = (u8)( mpu_data[8] >> 8 );
			comdata[17] = (u8)( mpu_data[8] );
			
			//里程计坐标 x(m) y(m) yaw(rad)  odom_frame
			comdata[18] = (u8)( robot_odom[0] >> 8 );
			comdata[19] = (u8)( robot_odom[0] );
			comdata[20] = (u8)( robot_odom[1] >> 8 );
			comdata[21] = (u8)( robot_odom[1] );
			comdata[22] = (u8)( robot_odom[2] >> 8 );
			comdata[23] = (u8)( robot_odom[2] );
			
			//里程计坐标变化量  d_x(m) d_y(m) d_yaw(rad)  base_frame
			comdata[24] = (u8)( robot_odom[3] >> 8 );
			comdata[25] = (u8)( robot_odom[3] );
			comdata[26] = (u8)( robot_odom[4] >> 8 );
			comdata[27] = (u8)( robot_odom[4] );
			comdata[28] = (u8)( robot_odom[5] >> 8 );
			comdata[29] = (u8)( robot_odom[5] );
		
		  //编码器当前值和目标值
			comdata[30] = (u8)( ax_encoder_delta[0] >> 8 );  
			comdata[31] = (u8)( ax_encoder_delta[0] );
			comdata[32] = (u8)( ax_encoder_delta[1] >> 8 );
			comdata[33] = (u8)( ax_encoder_delta[1] );
			comdata[34] = (u8)( ax_encoder_delta[2] >> 8 );
			comdata[35] = (u8)( ax_encoder_delta[2] );
			comdata[36] = (u8)( ax_encoder_delta[3] >> 8 );
			comdata[37] = (u8)( ax_encoder_delta[3] );
			
			comdata[38] = (u8)( ax_encoder_delta_target[0] >> 8 );  
			comdata[39] = (u8)( ax_encoder_delta_target[0] );
			comdata[40] = (u8)( ax_encoder_delta_target[1] >> 8 );
			comdata[41] = (u8)( ax_encoder_delta_target[1] );
			comdata[42] = (u8)( ax_encoder_delta_target[2] >> 8 );
			comdata[43] = (u8)( ax_encoder_delta_target[2] );
			comdata[44] = (u8)( ax_encoder_delta_target[3] >> 8 );
			comdata[45] = (u8)( ax_encoder_delta_target[3] );
			
			//编码器
			comdata[46] = (u8)( ax_bat_vol >> 8 );
			comdata[47] = (u8)( ax_bat_vol );
				
			//发送串口数据
			AX_UART_PI_SendPacket(comdata, 48, 0x06);
}

/******************* (C) 版权 2019 XTARK **************************************/

