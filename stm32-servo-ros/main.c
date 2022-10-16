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
 
	delay_init();	    	 //��ʱ������ʼ��	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);// �����ж����ȼ�����2
	uart_init(9600);	 //���ڳ�ʼ��Ϊ9600
	LED_Init();		  	 //��ʼ����LED���ӵ�Ӳ���ӿ� 
  TIM2_CH1_PWM_Init(1999,719); //1950~1750 ��Ӧ0~180�㣨������̣�;����Ƕȶ�ӦPWM��1950-200*(x/180)
	 
	 
	while(1)
	{
		u8 tem_angle=0;//�Ƕ�ģʽ
		u8 i;
		if(USART_RX_STA&0x8000)
		{					   
			len=USART_RX_STA&0x3fff;//�õ��˴ν��յ������ݳ���
			
			for(i=0;i<len;i++)//��������
			{
				u8 tem_value = get_value(USART_RX_BUF[i]);
				if(tem_value == 10) {break;}//����
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
			if(times%100==0)LED0=!LED0;//��˸LED,��ʾϵͳ��������.
			delay_ms(10);   
		}
	}	 
}


