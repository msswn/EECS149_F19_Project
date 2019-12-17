// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

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

#include "states.h"

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

static simple_ble_char_t forward_char = {.uuid16 = 0xeda3};
bool forward;
static simple_ble_char_t backward_char = {.uuid16 = 0xeda4};
bool backward;
static simple_ble_char_t left_char = {.uuid16 = 0xeda5};
bool left;
static simple_ble_char_t right_char = {.uuid16 = 0xeda6};
bool right;
// global variables
KobukiSensors_t sensors = {0};

states state = OFF;

// Intervals for advertising and connections
static simple_ble_config_t ble_config = {
        // c0:98:e5:49:xx:xx
        .platform_id       = 0x49,    // used as 4th octect in device BLE address
        .device_id         = 0x0011, // TODO: replace with your lab bench number
        .adv_name          = "KOBUKI", // used in advertisements if there is room
        .adv_interval      = MSEC_TO_UNITS(1000, UNIT_0_625_MS),
        .min_conn_interval = MSEC_TO_UNITS(100, UNIT_1_25_MS),
        .max_conn_interval = MSEC_TO_UNITS(200, UNIT_1_25_MS),
};

//4607eda0-f65e-4d59-a9ff-84420d87a4ca
static simple_ble_service_t robot_service = {{
    .uuid128 = {0xca,0xa4,0x87,0x0d,0x42,0x84,0xff,0xA9,
                0x59,0x4D,0x5e,0xf6,0xa0,0xed,0x07,0x46}
}};

// TODO: Declare characteristics and variables for your service

simple_ble_app_t* simple_ble_app;

void ble_evt_write(ble_evt_t const* p_ble_evt) {
    // TODO: logic for each characteristic and related state changes
	if (simple_ble_is_char_event(p_ble_evt, &forward_char)) {
		if(forward){
			printf("Moving forwards\n");
			state = FORWARD;
			display_write("FORWARD", DISPLAY_LINE_1);
		}
		else{
			printf("%i\n",forward);
			printf("OFF _> FOrward\n");
			state = OFF;
		}
	}
	if (simple_ble_is_char_event(p_ble_evt, &backward_char)) {
		if(backward){
			printf("Moving backwards\n");
			state = BACKWARD;
			display_write("BACKWARD", DISPLAY_LINE_1);
		}
		else{
			printf("OFF -> BACK\n");
			state = OFF;
		}
	}
	if (simple_ble_is_char_event(p_ble_evt, &right_char)) {

		if(right){
			printf("Moving right\n");
			state = RIGHT;
			display_write("RIGHT", DISPLAY_LINE_1);
		}
		else{
			printf("OFF-> RIGHT\n");
			state = OFF;
		}
	}
	if (simple_ble_is_char_event(p_ble_evt, &left_char)) {
		if(left){
			printf("Moving left\n");
			state = LEFT;
			display_write("LEFT", DISPLAY_LINE_1);
		}
		else{
			printf("OFF -> LEFT\n");
			state = OFF;
		}
	}
	

}

void print_state(states current_state){
	switch(current_state){
	case OFF:
		display_write("OFF", DISPLAY_LINE_0);
		break;
    }
}

int main(void) {
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // Setup BLE
  simple_ble_app = simple_ble_init(&ble_config);

  simple_ble_add_service(&robot_service);

  // TODO: Register your characteristics
  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(forward), &forward,
      &robot_service, &forward_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(forward), &backward,
      &robot_service, &backward_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(forward), &right,
      &robot_service, &right_char);

  simple_ble_add_characteristic(1, 1, 0, 0,
      sizeof(forward), &left,
      &robot_service, &left_char);

  // Start Advertising
  simple_ble_adv_only_name();

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

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
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    int status = kobukiSensorPoll(&sensors);
    //kobukiSensorPoll(&sensors);

    // TODO: complete state machine
    switch(state) {
      case OFF: {
        
          state = OFF;
          // perform state-specific actions here
          kobukiDriveDirect(0, 0);
      
        break; // each case needs to end with break!
      }
      case RIGHT: {
        
      
          state = RIGHT;
          // perform state-specific actions here
          kobukiDriveDirect(100, -0);
        
        break; // each case needs to end with break!

      }
      case LEFT: {
      

  
          state = LEFT;
          // perform state-specific actions here
          kobukiDriveDirect(-100, 100);
        
        break; // each case needs to end with break!

      }
      case FORWARD: {
        

      
    
        state = FORWARD;
          // perform state-specific actions here
        kobukiDriveDirect(100, 100);
      
        break; // each case needs to end with break!

      }
      case BACKWARD: {
        
          state = BACKWARD;
          // perform state-specific actions here
          kobukiDriveDirect(-100, -100);
        break; // each case needs to end with break!

      }
    }
    // TODO: complete state machine
    // switch(state) {
    //   case OFF: {
    //     print_state(state);

    //     // transition logic
    //     if (is_button_pressed(&sensors)) {
    //       //state = NEWSTATE;
    //     } else {
    //       state = OFF;
    //       // perform state-specific actions here
    //       kobukiDriveDirect(0, 0);
    //     }
    //     break; // each case needs to end with break!
    //   }
    // }
  }
}

