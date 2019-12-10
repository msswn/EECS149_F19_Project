// Blink app
//
// Blinks the LEDs on Buckler

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_log.h"
#include "nrf_gpio.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_serial.h"

#include "buckler.h"
#include "gpio.h"

uint32_t timeout = 1000000L;

static uint32_t MicrosDiff(uint32_t begin, uint32_t end) {
	return end-begin;
}

static uint32_t pulseIn(uint8_t pin) {
	uint32_t begin = read_timer();

	// wait for previous pulse to end
	while (gpio_read(pin)) {} // if (MicrosDiff(begin,read_timer()) >= timeout) return 0;

	// wait for pulse to start
	while (!gpio_read(pin)) {} // if (MicrosDiff(begin, read_timer()) >= timeout) return 0;
	uint32_t pulseBegin = read_timer();

	// wait for pulse to stop
	while (gpio_read(pin)) {} // if (MicrosDiff(begin, read_timer()) >= timeout) return 0;
	uint32_t pulseEnd = read_timer();

	return MicrosDiff(pulseBegin, pulseEnd);
}

float get_distance(uint8_t pin) {
	gpio_config(pin, OUTPUT);
	gpio_clear(pin);
	nrf_delay_us(2);
	gpio_set(pin);
	nrf_delay_us(15);
	gpio_clear(pin);
	gpio_config(pin,INPUT);
	uint32_t duration;
	duration = pulseIn(pin);
	float RangeInCentimeters;
	RangeInCentimeters = duration/29.0f/2.0f;
	return RangeInCentimeters;
}

float distF() {
	return get_distance(2);
}

float distR() {
	return get_distance(4);
}

float distL() {
	return get_distance(6);
}

float distB() {
	return get_distance(19);
}

