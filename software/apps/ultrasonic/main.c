#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "nrf_delay.h"
#include "ultrasonic.h"

int main(void)
{
	virtual_timer_init();
	while (1)
	{
		nrf_delay_ms(500);
		printf("Right: %f, Front: %f, Back: %f\n", distR(),distF(),distB());
	}
}
