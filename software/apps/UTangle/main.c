#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <math.h>

#include "nrf_delay.h"
#include "ultrasonic.h"

int main(void)
{
	virtual_timer_init();
	float l = 0.352;
	
	while (1)
	{
		nrf_delay_ms(500);

		float back = distB();
		float right = distR();
		float theta = atan((right+0.5*l)/(back+0.5*l))*(180/M_PI);
		printf("Right: %.1f, Back: %.1f, Theta: %.1f\n", right,back,theta);
	}
}
