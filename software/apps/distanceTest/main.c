// Robot Template app
//
// Framework for creating applications that control the Kobuki robot

#include <math.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <time.h>

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

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum
{
  OFF,
  DRIVING,
} robot_state_t;

int16_t vel = 50;
float lastDist;

static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder, float dt)
{

  if (current_encoder < previous_encoder)
  {
    current_encoder += 2 ^ 16;
  }
  const float CONVERSION = 0.08529;
  float distance = CONVERSION * (current_encoder - previous_encoder);
  if (distance - lastDist > vel / dt * 1.25)
  {
    distance = lastDist;
  }
  lastDist = distance;
  return distance;
}

int main(void)
{
  ret_code_t error_code = NRF_SUCCESS;

  // initialize RTT library
  error_code = NRF_LOG_INIT(NULL);
  APP_ERROR_CHECK(error_code);
  NRF_LOG_DEFAULT_BACKENDS_INIT();
  printf("Log initialized!\n");

  // initialize LEDs
  nrf_gpio_pin_dir_set(23, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(24, NRF_GPIO_PIN_DIR_OUTPUT);
  nrf_gpio_pin_dir_set(25, NRF_GPIO_PIN_DIR_OUTPUT);

  // initialize timer
  virtual_timer_init();

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
  error_code = nrf_drv_spi_init(&spi_instance, &spi_config, NULL, NULL);
  APP_ERROR_CHECK(error_code);
  display_init(&spi_instance);
  display_write("Hello, Human!", DISPLAY_LINE_0);
  display_write("", DISPLAY_LINE_1);
  printf("Display initialized!\n");

  // initialize i2c master (two wire interface)
  nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
  i2c_config.scl = BUCKLER_SENSORS_SCL;
  i2c_config.sda = BUCKLER_SENSORS_SDA;
  i2c_config.frequency = NRF_TWIM_FREQ_100K;
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  // mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  float KL = 0.1;
  float KR = 0.1;
  float KdL = 0.1;
  float KdR = 0.1;
  float Kw = 0;
  float Ki = 0;

  int16_t velL;
  int16_t velR;
  uint16_t start_encoder_L;
  uint16_t start_encoder_R;
  uint32_t timer_start;
  float curr_time;
  float last_time;
  float lasteL;
  float lasteR;
  float derivL;
  float derivR;
  int loop;
  float intErrorL;
  float intErrorR;
  float lastEncoderL;
  float lastEncoderR;

  // loop forever, running state machine
  while (1)
  {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(1);

    // handle states
    switch (state)
    {
    case OFF:
    {
      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = DRIVING;
        start_encoder_L = sensors.leftWheelEncoder;
        start_encoder_R = sensors.rightWheelEncoder;
        timer_start = read_timer();
        loop = 0;
        derivL = 0;
        derivR = 0;
        intErrorL = 0;
        intErrorR = 0;
        lasteL = 0;
        lasteR = 0;
        last_time = 0;
        lastDist = 0;
        lastEncoderL = sensors.leftWheelEncoder;
        lastEncoderR = sensors.rightWheelEncoder;
      }
      else
      {
        // perform state-specific actions here
        display_write("OFF", DISPLAY_LINE_0);
        display_write(" ", DISPLAY_LINE_1);
        kobukiDriveDirect(0, 0);
        state = OFF;
      }
      break; // each case needs to end with break!
    }

    case DRIVING:
    {
      curr_time = (float)(read_timer() - timer_start) / 1000000;
      float dt = curr_time - last_time;
      // Compute left and right actual velocity
      float encoderL = sensors.leftWheelEncoder;
      float encoderR = sensors.rightWheelEncoder;
      float distL = measure_distance(encoderL, lastEncoderL, dt);
      float distR = measure_distance(encoderR, lastEncoderR, dt);
      float distTravel = measure_distance(encoderL,start_encoder_L,dt);

      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = OFF;
      }
      else if (distTravel > 0.5)
      {
        state = OFF;
      }
      else
      {
        // Compute current time and change in time

        // // Compute desired distance
        // float distD = vel*curr_time;
        // // Compute left and right actual distance
        // float distL = measure_distance(sensors.leftWheelEncoder, start_encoder_L,dt);
        // float distR = measure_distance(sensors.rightWheelEncoder, start_encoder_R,dt);
        // // Compute distance errors
        // float eL = distD-distL;
        // float eR = distD-distR;

        float velActualL = distL / dt;
        float velActualR = distR / dt;
        printf("%i\n", dt);

        if (abs(velActualL) > vel * 1.1)
        {
          velActualL = vel;
        }

        if (abs(velActualR) > vel * 1.1)
        {
          velActualR = vel;
        }

        // Compute left and right velocity error
        float eL = vel - velActualL;
        float eR = vel - velActualR;

        // Compute derivative term
        float curr_derivative_L = (eL - lasteL) / dt;
        float curr_derivative_R = (eR - lasteR) / dt;
        derivL += curr_derivative_L;
        derivR += curr_derivative_R;
        float edL = derivL / loop;
        float edR = derivR / loop;

        // Compute integral term
        intErrorL = Kw * intErrorL + eL;
        intErrorR = Kw * intErrorR + eR;

        // Update terms
        lastEncoderL = encoderL;
        lastEncoderR = encoderR;
        lasteL = eL;
        lasteR = eR;
        last_time = curr_time;

        // Compute input
        velL = (int16_t)vel + KL * eL + KdL * edL + Ki * intErrorL;
        velR = (int16_t)vel + KR * eR + KdR * edR + Ki * intErrorR;
        // velL = (int16_t) vel + KL*eL;
        // velR = (int16_t) vel + KR*eR;
        char buf1[16];
        char buf2[16];
        snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
        snprintf(buf2, 16, "distD:%.1f", vel);
        display_write(buf1, DISPLAY_LINE_0);
        display_write(buf2, DISPLAY_LINE_1);
        kobukiDriveDirect((int16_t)velL, (int16_t)velR);
        state = DRIVING;
        loop += 1;
      }
      break; // each case needs to end with break!
    }

      // add other cases here
    }
  }
}
