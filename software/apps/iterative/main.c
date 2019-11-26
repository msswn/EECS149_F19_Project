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

// I2C manager
NRF_TWI_MNGR_DEF(twi_mngr_instance, 5, 0);

typedef enum {
  OFF,
  C1,
  SEG,
  C2,
  C3,
  C4
} robot_state_t;

int16_t vel = 100;
float lastDist;

static float measure_distance_reverse(uint16_t current_encoder, uint16_t previous_encoder,float dt){
	if(current_encoder > previous_encoder){
		previous_encoder += 2^16;
	} 
	const float CONVERSION = 0.08529;
	float distance = fabs(CONVERSION*(previous_encoder - current_encoder));
  if (distance-lastDist > vel/dt*1.25) {
    distance = lastDist;
  }
  lastDist = distance;
	return distance;
}

static float norm(float v[2]){
	return sqrt(pow(v[0],2)+pow(v[1],2));
}

static float dot(float v1[2],float v2[2]){
	return v1[0]*v2[0]+v1[1]*v2[1];
}

static float theta(float xc, float yc, float r, float xs, float ys, float xe, float ye){
	float u[2] = {r,0};
	float vs[2] = {xs-xc, ys-yc};
	float ve[2] = {xe-xc, ye-yc};
	float ths = acos(dot(u,vs)/(norm(u)*norm(vs)));
	float the = acos(dot(u,ve)/(norm(u)*norm(ve)));
	if (ve[2] < 0) {
		ths = 2*M_PI-ths;
		the = 2*M_PI-the;
	}
	// printf("ths:%f, the:%f\n",ths,the);
	return the-ths;
}


int main(void) {
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
    .bit_order = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST
  };
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
  error_code = nrf_twi_mngr_init(&twi_mngr_instance, &i2c_config);
  APP_ERROR_CHECK(error_code);
  mpu9250_init(&twi_mngr_instance);
  printf("IMU initialized!\n");

  // initialize Kobuki
  kobukiInit();
  printf("Kobuki initialized!\n");

  // configure initial state
  robot_state_t state = OFF;
  KobukiSensors_t sensors = {0};

  // Control parameters
  float KL = 1;
  float KR = 1;
  float KdL = 0.25;
  float KdR = 0.25;
  float Kw = 1;
  float Ki = 0.5;

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
  
  // current Kobuki position
  float S = 0.6;
  float H = 0.5;

  // Kobuki specs
  float l = 0.352;
  float w = 0.230;
  float Rmin = l/tan(50*M_PI/180);
  // Outer and inner velocity ratio
  float velO = vel*(Rmin+w/2)/Rmin;
  float velI = vel*(Rmin-w/2)/Rmin;

  // Path calculation
  float k = (S*(H-2*Rmin)+sqrt(4*pow(Rmin,2)*(pow(S,2)+pow(H,2))-16*pow(Rmin,3)*H))/(pow(S,2)-4*pow(Rmin,2));
  float m = Rmin*(1-sqrt(1+pow(k,2)));
  float f1[2] = {S-k/sqrt(1+pow(k,2))*Rmin,H-(1-1/sqrt(1+pow(k,2)))*Rmin};
  float f2[2] = {k/sqrt(1+pow(k,2))*Rmin, (1-1/sqrt(1+pow(k,2)))*Rmin};
  
  float th1 = theta(S, H-Rmin, Rmin, S, H, f1[0], f1[1]);
  float th2 = theta(0, Rmin, Rmin, 0, 0, f2[0], f2[1]);
  printf("th1: %f, th2: %f\n",th1,th2);
  float del[2] = {f1[0]-f2[0],f1[1]-f2[1]};
  float magC1 = fabs(Rmin*th1);
  float magS = norm(del);
  float magC2 = fabs(Rmin*th2);
  printf("k: %f, m: %f, f1:[%f,%f], f2:[%f,%f] \nmagC1:%f, magS:%f, magC2:%f \n",k,m,f1[0],f1[1],f2[0],f2[1],magC1,magS,magC2);

  // Deadlines to reach waypoint
  float t1 = magC1*1000/vel;
  float t2 = S*1000/vel;
  float t3 = magC2*1000/vel;
  printf("t1: %f,t2: %f, t3: %f",t1,t2,t3);

  // loop forever, running state machine
  while (1) {
    // read sensors from robot
    kobukiSensorPoll(&sensors);

    // delay before continuing
    // Note: removing this delay will make responses quicker, but will result
    //  in printf's in this loop breaking JTAG
    nrf_delay_ms(1);

    // handle states
    switch(state) {
      case OFF: {
        // transition logic
        if (is_button_pressed(&sensors)) {
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
        } else {
          // perform state-specific actions here
          display_write("OFF", DISPLAY_LINE_0);
          display_write(" ",DISPLAY_LINE_1);
          kobukiDriveDirect(0,0);
          state = OFF;
        }
        break; // each case needs to end with break!
      }

      case C1: {
        // transition logic
         if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (curr_time >= t1) {
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
        } else {
          // Compute current time and change in time
          curr_time = (float) (read_timer()-timer_start)/1000000;
          float dt = curr_time-last_time;
          
          // Compute desired distance
          float distDL = velO*curr_time;
          float distDR = velI*curr_time;
          // Compute left and right actual distance
          float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L,dt);
          float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R,dt);
          // Compute distance errors
          float eL = distDL-distL;
          float eR = distDR-distR;

          // Compute derivative term
          float curr_derivative_L = (eL-lasteL)/dt;
          float curr_derivative_R = (eR-lasteR)/dt;
          derivL += curr_derivative_L;
          derivR += curr_derivative_R;
          float edL = derivL/loop;
          float edR = derivR/loop;

          // Compute integral term
          intErrorL = Kw*intErrorL + eL;
          intErrorR = Kw*intErrorR + eR;

          // Update terms
          lasteL = eL;
          lasteR = eR;
          last_time = curr_time;
          
          // Compute input
          velL = (int16_t) -velO-KL*eL-KdL*edL-Ki*intErrorL;
          velR = (int16_t) -velI-KR*eR-KdR*edR-Ki*intErrorR;
          char buf1[16];
          // char buf2[16];
          snprintf(buf1,16,"L:%.1f,R:%.1f",eL,eR);
          // snprintf(buf2,16,"distDL:%.1f",distDL);
          display_write(buf1,DISPLAY_LINE_0);
          // display_write(buf2,DISPLAY_LINE_1);
          // display_write("C1", DISPLAY_LINE_0);
          char buf2[16];
          snprintf(buf2,16,"Time: %f s",curr_time);
          display_write(buf2,DISPLAY_LINE_1);
          kobukiDriveDirect((int16_t) velL,(int16_t) velR);
          state = C1;
          loop += 1;
        }
        break; // each case needs to end with break!
      }

      case SEG: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (sensors.bumps_wheelDrops.bumpLeft | sensors.bumps_wheelDrops.bumpRight | sensors.bumps_wheelDrops.bumpCenter) {
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
        else if (curr_time >= t2) {
          state = C2;
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
        } else {
          // Compute current time and change in time
          curr_time = (float) (read_timer()-timer_start)/1000000;
          float dt = curr_time-last_time;
          
          // Compute desired distance
          float distDL = vel*curr_time;
          float distDR = vel*curr_time;
          // Compute left and right actual distance
          float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L,dt);
          float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R,dt);
          // Compute distance errors
          float eL = distDL-distL;
          float eR = distDR-distR;

          // Compute derivative term
          float curr_derivative_L = (eL-lasteL)/dt;
          float curr_derivative_R = (eR-lasteR)/dt;
          derivL += curr_derivative_L;
          derivR += curr_derivative_R;
          float edL = derivL/loop;
          float edR = derivR/loop;

          // Compute integral term
          intErrorL = Kw*intErrorL + eL;
          intErrorR = Kw*intErrorR + eR;

          // Update terms
          lasteL = eL;
          lasteR = eR;
          last_time = curr_time;
          
          // Compute input
          velL = (int16_t) -vel-KL*eL-KdL*edL-Ki*intErrorL;
          velR = (int16_t) -vel-KR*eR-KdR*edR-Ki*intErrorR;
          char buf1[16];
          // char buf2[16];
          snprintf(buf1,16,"L:%.1f,R:%.1f",eL,eR);
          // snprintf(buf2,16,"distDL:%.1f",distDL);
          display_write(buf1,DISPLAY_LINE_0);
          // display_write(buf2,DISPLAY_LINE_1);
          // display_write("SEG", DISPLAY_LINE_0);
          char buf2[16];
          snprintf(buf2,16,"Time: %f s",curr_time);
          display_write(buf2,DISPLAY_LINE_1);
          kobukiDriveDirect((int16_t) velL,(int16_t) velR);
          state = SEG;
          loop += 1;
        }
        break; // each case needs to end with break!
      }

      case C2: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (curr_time >= t3) {
          state = OFF;
        } else {
          // Compute current time and change in time
          curr_time = (float) (read_timer()-timer_start)/1000000;
          float dt = curr_time-last_time;
          
          // Compute desired distance
          float distDL = velI*curr_time;
          float distDR = velO*curr_time;
          // Compute left and right actual distance
          float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L,dt);
          float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R,dt);
          // Compute distance errors
          float eL = distDL-distL;
          float eR = distDR-distR;

          // Compute derivative term
          float curr_derivative_L = (eL-lasteL)/dt;
          float curr_derivative_R = (eR-lasteR)/dt;
          derivL += curr_derivative_L;
          derivR += curr_derivative_R;
          float edL = derivL/loop;
          float edR = derivR/loop;

          // Compute integral term
          intErrorL = Kw*intErrorL + eL;
          intErrorR = Kw*intErrorR + eR;

          // Update terms
          lasteL = eL;
          lasteR = eR;
          last_time = curr_time;
          
          // Compute input
          velL = (int16_t) -velI-KL*eL-KdL*edL-Ki*intErrorL;
          velR = (int16_t) -velO-KR*eR-KdR*edR-Ki*intErrorR;
          char buf1[16];
          // char buf2[16];
          snprintf(buf1,16,"L:%.1f,R:%.1f",eL,eR);
          // snprintf(buf2,16,"distDL:%.1f",distDL);
          display_write(buf1,DISPLAY_LINE_0);
          // display_write(buf2,DISPLAY_LINE_1);
          // display_write("C2", DISPLAY_LINE_0);
          char buf2[16];
          snprintf(buf2,16,"Time: %f s",curr_time);
          display_write(buf2,DISPLAY_LINE_1);
          kobukiDriveDirect((int16_t) velL,(int16_t) velR);
          state = C2;
          loop += 1;
        }
        break; // each case needs to end with break!
      }

      case C3: {
        // transition logic
        if (is_button_pressed(&sensors)) {
          state = OFF;
        } else if (curr_time >= t3) {
          state = OFF;
        } else {
          // Compute current time and change in time
          curr_time = (float) (read_timer()-timer_start)/1000000;
          float dt = curr_time-last_time;
          
          // Compute desired distance
          float distDL = velI*curr_time;
          float distDR = velO*curr_time;
          // Compute left and right actual distance
          float distL = measure_distance_reverse(sensors.leftWheelEncoder, start_encoder_L,dt);
          float distR = measure_distance_reverse(sensors.rightWheelEncoder, start_encoder_R,dt);
          // Compute distance errors
          float eL = distDL-distL;
          float eR = distDR-distR;

          // Compute derivative term
          float curr_derivative_L = (eL-lasteL)/dt;
          float curr_derivative_R = (eR-lasteR)/dt;
          derivL += curr_derivative_L;
          derivR += curr_derivative_R;
          float edL = derivL/loop;
          float edR = derivR/loop;

          // Compute integral term
          intErrorL = Kw*intErrorL + eL;
          intErrorR = Kw*intErrorR + eR;

          // Update terms
          lasteL = eL;
          lasteR = eR;
          last_time = curr_time;
          
          // Compute input
          velL = (int16_t) -velI-KL*eL-KdL*edL-Ki*intErrorL;
          velR = (int16_t) -velO-KR*eR-KdR*edR-Ki*intErrorR;
          char buf1[16];
          // char buf2[16];
          snprintf(buf1,16,"L:%.1f,R:%.1f",eL,eR);
          // snprintf(buf2,16,"distDL:%.1f",distDL);
          display_write(buf1,DISPLAY_LINE_0);
          // display_write(buf2,DISPLAY_LINE_1);
          // display_write("C2", DISPLAY_LINE_0);
          char buf2[16];
          snprintf(buf2,16,"Time: %f s",curr_time);
          display_write(buf2,DISPLAY_LINE_1);
          kobukiDriveDirect((int16_t) velL,(int16_t) velR);
          state = C2;
          loop += 1;
        }
        break; // each case needs to end with break!
      }

      // add other cases here

    }
  }
}

