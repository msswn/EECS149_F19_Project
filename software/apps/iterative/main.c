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
  C1,
  SEG,
  C2,
  C3,
  C4,
  C5,
  C6,
  S2
} robot_state_t;

int16_t vel = 100;
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
  return distance;
}

static float measure_distance_reverse(uint16_t current_encoder, uint16_t previous_encoder, float dt)
{
  if (current_encoder > previous_encoder)
  {
    previous_encoder += 2 ^ 16;
  }
  const float CONVERSION = 0.08529;
  float distance = fabs(CONVERSION * (previous_encoder - current_encoder));
  if (distance - lastDist > vel / dt * 1.25)
  {
    distance = lastDist;
  }
  lastDist = distance;
  return distance;
}

static float norm(float v[2])
{
  return sqrt(pow(v[0], 2) + pow(v[1], 2));
}

static float dot(float v1[2], float v2[2])
{
  return v1[0] * v2[0] + v1[1] * v2[1];
}

static float theta(float xc, float yc, float r, float xs, float ys, float xe, float ye)
{
  float u[2] = {r, 0};
  float vs[2] = {xs - xc, ys - yc};
  float ve[2] = {xe - xc, ye - yc};
  float ths = acos(dot(u, vs) / (norm(u) * norm(vs)));
  float the = acos(dot(u, ve) / (norm(u) * norm(ve)));
  if (ve[1] < 0)
  {
    ths = 2 * M_PI - ths;
    the = 2 * M_PI - the;
  }
  // printf("ths:%f, the:%f\n",ths,the);
  return the - ths;
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

  // Control parameters
  float KL = 2;
  float KR = 2;
  float KdL = 1;
  float KdR = 1;
  float Kw = 1;
  float Ki = 1;

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

  // Waypoints
  float f1[2] = {0, 0};
  float f2[2] = {0, 0};
  float f4[2] = {0, 0};
  float f5[2] = {0, 0};
  float f6[2] = {0, 0};
  float f7[2] = {0, 0};
  float f8[2] = {0, 0};
  float f9[2] = {0, 0};

  // Distances
  float magS;
  float magC1;
  float magC2;
  float magC3;
  float magC4;
  float magC5;
  float magC6;
  float magS2;

  // Kobuki specs
  float l = 0.352;
  float w = 0.230;
  float d1 = l / 2.0f + 0.01;
  float Rmin = l / tan(50 * M_PI / 180);
  // Outer and inner velocity ratio
  float velO = vel * (Rmin + w / 2) / Rmin;
  float velI = vel * (Rmin - w / 2) / Rmin;

  // parking spot parameters
  // float spotLength = l + 3 * d1;
  float spotLength = 0.902;
  float spotWidth = l * 1.25;
  float spotBack = -spotLength / 2.0f;
  float spotFront = spotLength / 2.0f;
  float spotLeft = spotWidth / 2.0f;
  float spotRight = -spotWidth / 2.0f;
  float spotCenter[2] = {0, 0};

  // initial Kobuki position
  float xi = 0.5;
  float yi = 0.5;

  // desired final position
  float xf = spotBack;
  float yf = 0.0f;

  // initial position offset
  float S = xi - xf;
  float H = yi - yf;

  // boolean flags
  bool backCollide = 0;
  bool frontCollide = 0;

  // Path calculation
  float k = (S * (H - 2 * Rmin) + sqrt(4 * pow(Rmin, 2) * (pow(S, 2) + pow(H, 2)) - 16 * pow(Rmin, 3) * H)) / (pow(S, 2) - 4 * pow(Rmin, 2));
  float m = Rmin * (1 - sqrt(1 + pow(k, 2))) + yf;
  f1[0] = (S - k / sqrt(1 + pow(k, 2)) * Rmin) + xf;
  f1[1] = (H - (1 - 1 / sqrt(1 + pow(k, 2))) * Rmin) + yf;
  f2[0] = (k / sqrt(1 + pow(k, 2)) * Rmin) + xf;
  f2[1] = ((1 - 1 / sqrt(1 + pow(k, 2))) * Rmin) + yf;

  // Check if SEG exceeds parking spot
  if (f2[0] < spotBack + d1)
  {
    backCollide = 1;
    f2[0] = spotBack + d1;
    f2[1] = k * (spotBack + d1 - xf) + m + yf;
    float del[2] = {f1[0] - f2[0], f1[1] - f2[1]};
    magS = norm(del);

    // Calculate first correction turn
    float dS[2];
    dS[0] = f1[0] - f2[0];
    dS[1] = f1[1] - f2[1];
    float dSn[2];
    dSn[0] = dS[1] / norm(dS);
    dSn[1] = -dS[0] / norm(dS);
    float cx = Rmin * dSn[0] + f2[0];
    float cy = Rmin * dSn[1] + f2[1];
    f5[0] = cx - (f2[0] - cx);
    f5[1] = cy + f2[1] - cy;
    float th5 = theta(cx, cy, Rmin, f5[0], f5[1], f2[0], f2[1]);
    magC3 = Rmin * th5;
  }
  else
  {
    float newO[2] = {spotBack + d1, 0};
    // Vehicle stops before collision
    f4[0] = 0 + newO[0];
    f4[1] = Rmin - sqrt(pow(Rmin, 2) - pow(d1, 2)) + newO[1];

    // Calculate first correction turn
    float dS[2];
    float dSn[2];
    dS[0] = f4[0] - xf;
    dS[1] = f4[1] - (Rmin + yf);
    dSn[0] = dS[1] / norm(dS);
    dSn[1] = -dS[0] / norm(dS);
    float cx = Rmin * dSn[0] + f4[0];
    // float cy = Rmin * dSn[1] + f4[1];
    f5[0] = cx + cx - f4[0];
    f5[1] = f4[1];
  }

  // Calculate second correction turn
  float f6xd = f5[0] + sqrt(pow(Rmin, 2) - pow(Rmin - f5[1], 2));
  float f6y = 0;
  f6[0] = f6xd;
  f6[1] = f6y;

  // Check for collision during second correction turn
  if (f6[0] >= spotFront - d1)
  {
    frontCollide = 1;
    float f6x = spotFront - d1;
    f6y = Rmin - sqrt(pow(Rmin, 2) - pow(f6x - f6xd, 2));
    f6[0] = f6x;
    f6[1] = f6y;
    float th6 = theta(f6xd, f6y + Rmin, Rmin, f5[0], f5[1], f6[0], f6[1]);
    magC4 = Rmin * th6;
    float cx = f6xd;
    float cy = Rmin;
    cx = f6[0] - (f6xd - f6[0]);
    cy = f6[1] - (Rmin - f6[1]);
    f7[0] = cx - (f6[0] - cx);
    f7[1] = cy + f6[1] - cy;
    float th7 = theta(cx, cy, Rmin, f6[0], f6[1], f7[0], f7[1]);
    magC5 = Rmin * th7;
    f8[0] = f7[0] - sqrt(pow(Rmin, 2) - pow(Rmin - f7[1], 2));
    f8[1] = 0;
    float th8 = theta(f8[0], f8[1] + Rmin, Rmin, f8[0], f8[1], f7[0], f7[1]);
    magC6 = Rmin * th8;
    f9[0] = spotCenter[0];
    f9[1] = spotCenter[1];
    float del[2] = {f8[0] - f9[0], f8[1] - f9[1]};
    magS2 = norm(del);
  }
  else
  {
    float th6 = theta(f6xd, f6y+Rmin, Rmin, f5[0],f5[1],f6[0],f6[1]);
    magC4 = Rmin*th6;
    float cx = f6xd;
    float cy = Rmin;
    f9[0] = spotCenter[0];
    f9[1] = spotCenter[1];
  }

  printf("f1: %f, %f; f2: %f,%f; f4: %f,%f; f5: %f,%f; f6: %f,%f; f7: %f,%f; f8: %f,%f; f9: %f,%f;\n", f1[0], f1[1], f2[0], f2[1], f4[0], f4[1], f5[0], f5[1], f6[0], f6[1], f7[0], f7[1], f8[0], f8[1], f9[0], f9[1]);

  float th1 = theta(S, H - Rmin, Rmin, S, H, f1[0], f1[1]);
  float th2 = theta(0, Rmin, Rmin, 0, 0, f2[0], f2[1]);
  // printf("th1: %f, th2: %f\n", th1, th2);

  magC1 = fabs(Rmin * th1);

  magC2 = fabs(Rmin * th2);

  // Deadlines to reach waypoint
  float t1 = magC1 * 1000 / vel;
  float t2 = magS * 1000 / vel;
  float t3 = magC2 * 1000 / vel;
  float t4 = magC3 * 1000 / vel;
  float t5 = magC4 * 1000 / vel;
  float t6 = magC5 * 1000 / vel;
  float t7 = magC6 * 1000 / vel;
  float t8 = magS2 * 1000 / vel;
  printf("magC1: %f,magS: %f, magC2: %f, magC3: %f, magC4: %f, magC5: %f, magC6: %f, magS2: %f\n", magC1, magS, magC2, magC3, magC4, magC5, magC6, magS2);
  // printf("t1: %f,t2: %f, t3: %f, t4: %f, t5: %f, t6: %f, t7: %f, t8: %f\n", t1, t2, t3, t4, t5, t6, t7, t8);

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
        state = C1;
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

    case C1:
    {
      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = OFF;
      }
      else if (curr_time >= t1)
      {
        state = SEG;
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
        curr_time = 0;
      }
      else
      {
        // Compute current time and change in time
        curr_time = (float)(read_timer() - timer_start) / 1000000;
        float dt = curr_time - last_time;

        // Compute desired distance
        float distDL = velO * curr_time;
        float distDR = velI * curr_time;
        // Compute left and right actual distance
        float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L, dt);
        float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R, dt);
        // Compute distance errors
        float eL = distDL - distL;
        float eR = distDR - distR;

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
        lasteL = eL;
        lasteR = eR;
        last_time = curr_time;

        // Compute input
        velL = (int16_t)-velO - KL * eL - KdL * edL - Ki * intErrorL;
        velR = (int16_t)-velI - KR * eR - KdR * edR - Ki * intErrorR;
        char buf1[16];
        // char buf2[16];
        snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
        // snprintf(buf2,16,"distDL:%.1f",distDL);
        display_write(buf1, DISPLAY_LINE_0);
        // display_write(buf2,DISPLAY_LINE_1);
        // display_write("C1", DISPLAY_LINE_0);
        char buf2[16];
        snprintf(buf2, 16, "Time: %f s", curr_time);
        display_write(buf2, DISPLAY_LINE_1);
        kobukiDriveDirect((int16_t)velL, (int16_t)velR);
        state = C1;
        loop += 1;
      }
      break; // each case needs to end with break!
    }

    case SEG:
    {
      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = OFF;
      }
      else if (curr_time >= t2)
      {
        state = C3;
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
        curr_time = 0;
      }
      else
      {
        // Compute current time and change in time
        curr_time = (float)(read_timer() - timer_start) / 1000000;
        float dt = curr_time - last_time;

        // Compute desired distance
        float distDL = vel * curr_time;
        float distDR = vel * curr_time;
        // Compute left and right actual distance
        float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L, dt);
        float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R, dt);
        // Compute distance errors
        float eL = distDL - distL;
        float eR = distDR - distR;

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
        lasteL = eL;
        lasteR = eR;
        last_time = curr_time;

        // Compute input
        velL = (int16_t)-vel - KL * eL - KdL * edL - Ki * intErrorL;
        velR = (int16_t)-vel - KR * eR - KdR * edR - Ki * intErrorR;
        char buf1[16];
        // char buf2[16];
        snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
        // snprintf(buf2,16,"distDL:%.1f",distDL);
        display_write("SEG", DISPLAY_LINE_0);
        // display_write(buf2,DISPLAY_LINE_1);
        // display_write("SEG", DISPLAY_LINE_0);
        char buf2[16];
        snprintf(buf2, 16, "Time: %f s", curr_time);
        display_write(buf2, DISPLAY_LINE_1);
        kobukiDriveDirect((int16_t)velL, (int16_t)velR);
        state = SEG;
        loop += 1;
      }
      break; // each case needs to end with break!
    }

    case C2:
    {
      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = OFF;
      }
      else if (curr_time >= t3)
      {
        state = OFF;
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
        curr_time = 0;
      }
      else
      {
        // Compute current time and change in time
        curr_time = (float)(read_timer() - timer_start) / 1000000;
        float dt = curr_time - last_time;

        // Compute desired distance
        float distDL = velI * curr_time;
        float distDR = velO * curr_time;
        // Compute left and right actual distance
        float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L, dt);
        float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R, dt);
        // Compute distance errors
        float eL = distDL - distL;
        float eR = distDR - distR;

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
        lasteL = eL;
        lasteR = eR;
        last_time = curr_time;

        // Compute input
        velL = (int16_t)-velI - KL * eL - KdL * edL - Ki * intErrorL;
        velR = (int16_t)-velO - KR * eR - KdR * edR - Ki * intErrorR;
        char buf1[16];
        // char buf2[16];
        snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
        // snprintf(buf2,16,"distDL:%.1f",distDL);
        display_write(C2, DISPLAY_LINE_0);
        // display_write(buf2,DISPLAY_LINE_1);
        // display_write("C2", DISPLAY_LINE_0);
        char buf2[16];
        snprintf(buf2, 16, "Time: %f s", curr_time);
        display_write(buf2, DISPLAY_LINE_1);
        kobukiDriveDirect((int16_t)velL, (int16_t)velR);
        state = C2;
        loop += 1;
      }
      break; // each case needs to end with break!
    }

    case C3:
    {
      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = OFF;
      }
      else if (curr_time >= t4)
      {
        state = C4;
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
        curr_time = 0;
      }
      else
      {
        // Compute current time and change in time
        curr_time = (float)(read_timer() - timer_start) / 1000000;
        float dt = curr_time - last_time;

        // Compute desired distance
        float distDL = velO * curr_time;
        float distDR = velI * curr_time;
        // Compute left and right actual distance
        float distL = measure_distance(sensors.leftWheelEncoder, start_encoder_L, dt);
        float distR = measure_distance(sensors.rightWheelEncoder, start_encoder_R, dt);
        // Compute distance errors
        float eL = distDL - distL;
        float eR = distDR - distR;

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
        lasteL = eL;
        lasteR = eR;
        last_time = curr_time;

        // Compute input
        velL = (int16_t)velO + KL * eL + KdL * edL + Ki * intErrorL;
        velR = (int16_t)velI + KR * eR + KdR * edR + Ki * intErrorR;
        char buf1[16];
        // char buf2[16];
        snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
        // snprintf(buf2,16,"distDL:%.1f",distDL);
        display_write("C3", DISPLAY_LINE_0);
        // display_write(buf2,DISPLAY_LINE_1);
        // display_write("C2", DISPLAY_LINE_0);
        char buf2[16];
        snprintf(buf2, 16, "Time: %f s", curr_time);
        display_write(buf2, DISPLAY_LINE_1);
        kobukiDriveDirect((int16_t)velL, (int16_t)velR);
        state = C3;
        loop += 1;
      }
      break; // each case needs to end with break!
    }

    case C4:
    {
      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = OFF;
      }
      else if (curr_time >= t5)
      {
        state = S2;
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
        curr_time = 0;
      }
      else
      {
        // Compute current time and change in time
        curr_time = (float)(read_timer() - timer_start) / 1000000;
        float dt = curr_time - last_time;

        // Compute desired distance
        float distDL = velI * curr_time;
        float distDR = velO * curr_time;
        // Compute left and right actual distance
        float distL = measure_distance(sensors.leftWheelEncoder, start_encoder_L, dt);
        float distR = measure_distance(sensors.rightWheelEncoder, start_encoder_R, dt);
        // Compute distance errors
        float eL = distDL - distL;
        float eR = distDR - distR;

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
        lasteL = eL;
        lasteR = eR;
        last_time = curr_time;

        // Compute input
        velL = (int16_t)velI + KL * eL + KdL * edL + Ki * intErrorL;
        velR = (int16_t)velO + KR * eR + KdR * edR + Ki * intErrorR;
        char buf1[16];
        // char buf2[16];
        snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
        // snprintf(buf2,16,"distDL:%.1f",distDL);
        display_write("C3", DISPLAY_LINE_0);
        // display_write(buf2,DISPLAY_LINE_1);
        // display_write("C2", DISPLAY_LINE_0);
        char buf2[16];
        snprintf(buf2, 16, "Time: %f s", curr_time);
        display_write(buf2, DISPLAY_LINE_1);
        kobukiDriveDirect((int16_t)velL, (int16_t)velR);
        state = C4;
        loop += 1;
      }
      break; // each case needs to end with break!
    }

      // case C5:
      // {
      //   // transition logic
      //   if (is_button_pressed(&sensors))
      //   {
      //     state = OFF;
      //   }
      //   else if (curr_time >= t6)
      //   {
      //     state = C6;
      //     start_encoder_L = sensors.leftWheelEncoder;
      //     start_encoder_R = sensors.rightWheelEncoder;
      //     timer_start = read_timer();
      //     loop = 0;
      //     derivL = 0;
      //     derivR = 0;
      //     intErrorL = 0;
      //     intErrorR = 0;
      //     lasteL = 0;
      //     lasteR = 0;
      //     last_time = 0;
      //     lastDist = 0;
      //     curr_time = 0;
      //   }
      //   else
      //   {
      //     // Compute current time and change in time
      //     curr_time = (float)(read_timer() - timer_start) / 1000000;
      //     float dt = curr_time - last_time;

      //     // Compute desired distance
      //     float distDL = velO * curr_time;
      //     float distDR = velI * curr_time;
      //     // Compute left and right actual distance
      //     float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L, dt);
      //     float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R, dt);
      //     // Compute distance errors
      //     float eL = distDL - distL;
      //     float eR = distDR - distR;

      //     // Compute derivative term
      //     float curr_derivative_L = (eL - lasteL) / dt;
      //     float curr_derivative_R = (eR - lasteR) / dt;
      //     derivL += curr_derivative_L;
      //     derivR += curr_derivative_R;
      //     float edL = derivL / loop;
      //     float edR = derivR / loop;

      //     // Compute integral term
      //     intErrorL = Kw * intErrorL + eL;
      //     intErrorR = Kw * intErrorR + eR;

      //     // Update terms
      //     lasteL = eL;
      //     lasteR = eR;
      //     last_time = curr_time;

      //     // Compute input
      //     velL = (int16_t)-velO - KL * eL - KdL * edL - Ki * intErrorL;
      //     velR = (int16_t)-velI - KR * eR - KdR * edR - Ki * intErrorR;
      //     char buf1[16];
      //     // char buf2[16];
      //     snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
      //     // snprintf(buf2,16,"distDL:%.1f",distDL);
      //     display_write("C6", DISPLAY_LINE_0);
      //     // display_write(buf2,DISPLAY_LINE_1);
      //     // display_write("C1", DISPLAY_LINE_0);
      //     char buf2[16];
      //     snprintf(buf2, 16, "Time: %f s", curr_time);
      //     display_write(buf2, DISPLAY_LINE_1);
      //     kobukiDriveDirect((int16_t)velL, (int16_t)velR);
      //     state = C5;
      //     loop += 1;
      //   }
      //   break; // each case needs to end with break!
      // }
      // case C6:
      // {
      //   // transition logic
      //   if (is_button_pressed(&sensors))
      //   {
      //     state = OFF;
      //   }
      //   else if (curr_time >= t7)
      //   {
      //     state = S2;
      //     start_encoder_L = sensors.leftWheelEncoder;
      //     start_encoder_R = sensors.rightWheelEncoder;
      //     timer_start = read_timer();
      //     loop = 0;
      //     derivL = 0;
      //     derivR = 0;
      //     intErrorL = 0;
      //     intErrorR = 0;
      //     lasteL = 0;
      //     lasteR = 0;
      //     last_time = 0;
      //     lastDist = 0;
      //     curr_time = 0;
      //   }
      //   else
      //   {
      //     // Compute current time and change in time
      //     curr_time = (float)(read_timer() - timer_start) / 1000000;
      //     float dt = curr_time - last_time;

      //     // Compute desired distance
      //     float distDL = velI * curr_time;
      //     float distDR = velO * curr_time;
      //     // Compute left and right actual distance
      //     float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L, dt);
      //     float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R, dt);
      //     // Compute distance errors
      //     float eL = distDL - distL;
      //     float eR = distDR - distR;

      //     // Compute derivative term
      //     float curr_derivative_L = (eL - lasteL) / dt;
      //     float curr_derivative_R = (eR - lasteR) / dt;
      //     derivL += curr_derivative_L;
      //     derivR += curr_derivative_R;
      //     float edL = derivL / loop;
      //     float edR = derivR / loop;

      //     // Compute integral term
      //     intErrorL = Kw * intErrorL + eL;
      //     intErrorR = Kw * intErrorR + eR;

      //     // Update terms
      //     lasteL = eL;
      //     lasteR = eR;
      //     last_time = curr_time;

      //     // Compute input
      //     velL = (int16_t)-velI - KL * eL - KdL * edL - Ki * intErrorL;
      //     velR = (int16_t)-velO - KR * eR - KdR * edR - Ki * intErrorR;
      //     char buf1[16];
      //     // char buf2[16];
      //     snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
      //     // snprintf(buf2,16,"distDL:%.1f",distDL);
      //     display_write("C6", DISPLAY_LINE_0);
      //     // display_write(buf2,DISPLAY_LINE_1);
      //     // display_write("C1", DISPLAY_LINE_0);
      //     char buf2[16];
      //     snprintf(buf2, 16, "Time: %f s", curr_time);
      //     display_write(buf2, DISPLAY_LINE_1);
      //     kobukiDriveDirect((int16_t)velL, (int16_t)velR);
      //     state = C6;
      //     loop += 1;
      //   }
      //   break; // each case needs to end with break!
      // }

    case S2:
    {
      // transition logic
      if (is_button_pressed(&sensors))
      {
        state = OFF;
      }
      else if (curr_time >= t8)
      {
        state = OFF;
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
        curr_time = 0;
      }
      else
      {
        // Compute current time and change in time
        curr_time = (float)(read_timer() - timer_start) / 1000000;
        float dt = curr_time - last_time;

        // Compute desired distance
        float distDL = vel * curr_time;
        float distDR = vel * curr_time;
        // Compute left and right actual distance
        float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L, dt);
        float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R, dt);
        // Compute distance errors
        float eL = distDL - distL;
        float eR = distDR - distR;

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
        lasteL = eL;
        lasteR = eR;
        last_time = curr_time;

        // Compute input
        velL = (int16_t)-vel - KL * eL - KdL * edL - Ki * intErrorL;
        velR = (int16_t)-vel - KR * eR - KdR * edR - Ki * intErrorR;
        char buf1[16];
        // char buf2[16];
        snprintf(buf1, 16, "L:%.1f,R:%.1f", eL, eR);
        // snprintf(buf2,16,"distDL:%.1f",distDL);
        display_write("S2", DISPLAY_LINE_0);
        // display_write(buf2,DISPLAY_LINE_1);
        // display_write("SEG", DISPLAY_LINE_0);
        char buf2[16];
        snprintf(buf2, 16, "Time: %f s", curr_time);
        display_write(buf2, DISPLAY_LINE_1);
        kobukiDriveDirect((int16_t)velL, (int16_t)velR);
        state = S2;
        loop += 1;
      }
      break; // each case needs to end with break!
    }
      // add other cases here
    }
  }
}
