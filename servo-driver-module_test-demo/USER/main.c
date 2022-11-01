#include "stm32f10x.h"
#include "delay.h"
#include "sys.h"
#include "pca9685.h"
#include "usart.h"
//140 307 480

int main(void) 
{	
  delay_init();
	pca_reset1();
	pca_reset2();
	pca_setfreq1(50);
	pca_setfreq2(50);
	while(1)
	{
		/*
		pca_setpwm1(0,0,250);
		delay_ms(500);
		pca_setpwm1(0,0,450);
		delay_ms(1200);
		pca_setpwm1(0,0,250);
		delay_ms(400);


		pca_setpwm1(1,0,307);
		delay_ms(500);
		pca_setpwm1(1,0,140);
		delay_ms(1200);
		pca_setpwm1(1,0,307);
		delay_ms(400);

		pca_setpwm1(15,0,180);
		delay_ms(500);
		pca_setpwm1(15,0,380);
		delay_ms(1200);
		pca_setpwm1(15,0,180);
		delay_ms(400);

		pca_setpwm1(3,0,140);
		delay_ms(500);
		pca_setpwm1(3,0,307);
		delay_ms(1200);
		pca_setpwm1(3,0,140);
		delay_ms(400);
		*/
		u16 pwm;
		for(pwm = 150; pwm <= 480; pwm+=20)
		{
			u8 i;
			for(i=0;i<=15;i++)
			{
				pca_setpwm1(i,0,pwm);
			}
			delay_ms(500);
		}
		//printf("PWM SET: %d\r\n",pwm);
	}
	
}

