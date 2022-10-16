#include "led.h"
#include "delay.h"
#include "sys.h"
#include "usart.h"
#include "timer.h"

const uint16_t PWM_angle0 = 1950;
const uint16_t PWM_angle1 = 1870;
const uint16_t PWM_angle2 = 1810;
const uint16_t PWM_angle3 = 1750;


u8 lenth;

static u8 get_value(u8 x)
{
	if( x>='0' && x<='9') return (x-'0');
	else return 10;
}

 int main(void)
 {	
	u8 t;
	u8 len;	
	u16 times=0; 
 
	delay_init();	    	 //延时函数初始化	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// 设置中断优先级分组2
	uart_init(9600);	 //串口初始化为9600
	LED_Init();		  	 //初始化与LED连接的硬件接口 
  TIM2_CH1_PWM_Init(1999,719); //1950~1750 对应0~180°（最大量程）;计算角度对应PWM：1950-200*(x/180)
	 
	 
	while(1)
	{
		u8 tem_angle=0;//角度模式
		u8 i;
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//得到此次接收到的数据长度
			
			for(i=0;i<len;i++)//解析数据
			{
				u8 tem_value = get_value(USART_RX_BUF[i]);
				if(tem_value == 10) {break;}//结束
				else {tem_angle = tem_angle*10 + tem_value;}
			}
			
			printf("tem_angle:%d\r\n",tem_angle);
			
			switch (tem_angle)
			{
				case 0:
					TIM_SetCompare1(TIM2,PWM_angle0);
					break;
				case 1:
					TIM_SetCompare1(TIM2,PWM_angle1);
					break;
				case 2:
					TIM_SetCompare1(TIM2,PWM_angle2);
					break;
				case 3:
					TIM_SetCompare1(TIM2,PWM_angle3);
					break;
			}
			USART_RX_STA=0;
		}
		else
		{
			times++;
			if(times%200==0)printf("Please input the data\r\n");  
			if(times%100==0)LED0=!LED0;//闪烁LED,提示系统正在运行.
			delay_ms(10);   
		}
	}	 
}


