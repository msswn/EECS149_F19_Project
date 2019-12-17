// BLE Service Template
//
// Creates a service for changing LED state over BLE

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>

#include "app_error.h"
#include "app_timer.h"
#include "nrf.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include "nrf_pwr_mgmt.h"
#include "nrf_drv_spi.h"

#include "buckler.h"
#include "display.h"
#include "kobukiActuator.h"
#include "kobukiSensorPoll.h"
#include "kobukiSensorTypes.h"
#include "kobukiUtilities.h"
#include "mpu9250.h"
#include "simple_ble.h"

typedef enum
{
  OFF,
  DRIVING
} robot_state_t;

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
    // c0:98:e5:49:xx:xx
    .platform_id = 0x49,     // used as 4th octect in device BLE address
    .device_id = 0x0011,     // TODO: replace with your lab bench number
    .adv_name = "EE149 LED", // used in advertisements if there is room
    .adv_interval = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
    .min_conn_interval = MSEC_TO_UNITS(500, UNIT_1_25_MS),
    .max_conn_interval = MSEC_TO_UNITS(1000, UNIT_1_25_MS),
};

// 32e61089-2b22-4db5-a914-43ce41986c70
static simple_ble_service_t led_service = {{.uuid128 = {0x70, 0x6C, 0x98, 0x41, 0xCE, 0x43, 0x14, 0xA9,
                                                        0xB5, 0x4D, 0x22, 0x2B, 0x89, 0x10, 0xE6, 0x32}}};

static simple_ble_char_t led_state_char = {.uuid16 = 0x108a};
static bool led_state = true;
char buf[16];

// global variables
KobukiSensors_t sensors = {0};

robot_state_t state = OFF;

/*******************************************************************************
 *   State for this application
 ******************************************************************************/
// Main application state
simple_ble_app_t *simple_ble_app;

void ble_evt_write(ble_evt_t const *p_ble_evt)
{
  if (simple_ble_is_char_event(p_ble_evt, &led_state_char))
  {
    if (state != OFF)
    {
      state = OFF;
    }
    else
    {
      state = DRIVING;
    }
  }
}

int main(void)
{

  // Initialize

  // initialize display
  nrf_drv_spi_t spi_instance = NRF_DRV_SPI_INSTANCE(1);
  nrf_drv_spi_config_t spi_config = {
      .sck_pin = BUCKLER_LCD_SCLK,
      .mosi_pin = BUCKLER_LCD_MOSI,
      .miso_pin = BUCKLER_LCD_MISO,
      .ss_pin = BUCKLER_LCD_CS,
      .irq_priority = NRFX_SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
      .orc = 0,
      .frequency = NRF_DRV_SPI_FREQ_4M,
      .mode = NRF_DRV_SPI_MODE_2,
      .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST};

  ret_code_t error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  printf("Display initialized!\n");

  // Setup LED GPIO
  nrf_gpio_cfg_output(BUCKLER_LED0);

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&led_service);

  simple_ble_add_characteristic(1, 1, 0, 0,
                                sizeof(led_state), (uint8_t *)&led_state,
                                &led_service, &led_state_char);

  // simple_ble_add_characteristic(1, 1, 0, 0,
  //     sizeof(buf), (char*)&buf,
  //     &led_service, &led_state_char);

  // Start Advertising
  simple_ble_adv_only_name();

  while (1)
  {
    int status = kobukiSensorPoll(&sensors);
    //kobukiSensorPoll(&sensors);

    // TODO: complete state machine
    switch (state)
    {
    case OFF:
    {

      state = OFF;
      // perform state-specific actions here
      kobukiDriveDirect(0, 0);
      display_write("OFF", DISPLAY_LINE_1);
      break; // each case needs to end with break!
    }
    case DRIVING:
    {

      state = DRIVING;
      // perform state-specific actions here
      display_write("DRIVING", DISPLAY_LINE_1);
      break; // each case needs to end with break!
    }
    }
  }
}
