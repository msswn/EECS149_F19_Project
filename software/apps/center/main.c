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
#include "virtual_timer.h"
#include "ultrasonic.h"

// I2C manager
// NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum
{
    OFF,
    CENTER
} robot_state_t;

// float lastDist;

// static float measure_distance(uint16_t current_encoder, uint16_t previous_encoder, float dt)
// {
//   if (current_encoder < previous_encoder)
//   {
//     current_encoder += 2 ^ 16;
//   }
//   const float CONVERSION = 0.08529;
//   float distance = CONVERSION * (current_encoder - previous_encoder);
//   if (distance - lastDist > vel / dt * 1.25)
//   {
//     distance = lastDist;
//   }
//   return distance;
// }

// static float measure_distance_reverse(uint16_t current_encoder, uint16_t previous_encoder, float dt)
// {
//   if (current_encoder > previous_encoder)
//   {
//     previous_encoder += 2 ^ 16;
//   }
//   const float CONVERSION = 0.08529;
//   float distance = fabs(CONVERSION * (previous_encoder - current_encoder));
//   if (distance - lastDist > vel / dt * 1.25)
//   {
//     distance = lastDist;
//   }
//   lastDist = distance;
//   return distance;
// }

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

    // initialize timer library
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
    printf("Display initialized!\n");

    // initialize i2c master (two wire interface)
    nrf_drv_twi_config_t i2c_config = NRF_DRV_TWI_DEFAULT_CONFIG;
    i2c_config.scl = BUCKLER_SENSORS_SCL;
    i2c_config.sda = BUCKLER_SENSORS_SDA;
    i2c_config.frequency = NRF_TWIM_FREQ_100K;
    // error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
    // APP_ERROR_CHECK(error_code);
    // mpu9250_init(&twi_mngr_instance);
    // printf("IMU initialized!\n");

    // initialize Kobuki
    kobukiInit();
    printf("Kobuki initialized!\n");

    // configure initial state
    robot_state_t state = OFF;
    KobukiSensors_t sensors = {0};

    // Control gains
    float Kp = 1.0f;

    // Kobuki parameters
    float l = 0.352;

    float velOld;
    float vel;
    float distB;

    // loop forever, running state machine
    while (1)
    {
        // read sensors from robot
        kobukiSensorPoll(&sensors);

        // delay before continuing
        // Note: removing this delay will make responses quicker, but will result
        //  in printf's in this loop breaking JTAG
        nrf_delay_ms(1);

        float distBOld = distB;
        float distF = distFront();
        float distB = 0.5 * (distBack() + distBOld);
        float distD = 0.5 * (distF + distB);
        // Compute distance error
        float e = distF - distB;
        // handle states
        switch (state)
        {
        case OFF:
        {
            // transition logic
            if (is_button_pressed(&sensors))
            {
                state = CENTER;
            }
            else
            { // snprintf(buf2,16,"distDL:%.1f",distDL);
                // perform state-specific actions here
                display_write("OFF", DISPLAY_LINE_0);
                display_write(" ", DISPLAY_LINE_1);
                kobukiDriveDirect(0, 0);
                state = OFF;
            }
            break; // each case needs to end with break!
        }

        case CENTER:
        {
            // transition logic
            if (is_button_pressed(&sensors))
            {
                state = OFF;
            }
            else if (-1 < e && e < 1)
            {
                state = OFF;
            }
            else
            {
                // Compute input
                vel = Kp * e * 10;
                if (vel - velOld > 20)
                {
                    vel = velOld + 5;
                }
                char buf1[16];
                char buf2[16];
                snprintf(buf1, 16, "error: %f cm", e);
                snprintf(buf2, 16, "F: %.0f, B: %.0f", distF, distB);
                display_write(buf1, DISPLAY_LINE_0);
                display_write(buf2, DISPLAY_LINE_1);
                kobukiDriveDirect((int16_t)vel, (int16_t)vel);
                velOld = vel;
                state = CENTER;
            }
            break; // each case needs to end with break!
        }
            // add other cases here
        }
    }
}
