#include "gpio.h"

#define GPIO_BASE 0x50000000

GPIO_Type* GPIO = (GPIO_Type*)GPIO_BASE;

// Inputs: 
//  gpio_num - gpio number 0-31
//  dir - gpio direction (INPUT, OUTPUT)
void gpio_config(uint8_t gpio_num, gpio_direction_t dir) {
	// INPUT
	if (dir == 0) {
		GPIO->PIN_CNF[gpio_num] &= ~3;
	}
	// OUTPUT
	else {
		GPIO->PIN_CNF[gpio_num] |= 3;
	}
}

// Set gpio_num high
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_set(uint8_t gpio_num) {
	GPIO->OUT |= (1 << gpio_num);
}

// Set gpio_num low
// Inputs: 
//  gpio_num - gpio number 0-31
void gpio_clear(uint8_t gpio_num) {
	GPIO->OUT &= ~(1 << gpio_num);
}

// Inputs: 
//  gpio_num - gpio number 0-31
bool gpio_read(uint8_t gpio_num) {
    // should return pin state
    return (GPIO->IN >> (gpio_num))&1;
}
