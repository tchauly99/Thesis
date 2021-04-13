/*
 * maipp.h
 *
 *  Created on: Feb 2, 2021
 *      Author: chau
 */

#ifndef INC_MAINPP_H_
#define INC_MAINPP_H_

#include "main.h"

#ifdef CODE_CPP

#define LINEAR 0
#define ANGULAR 1
#define LEFT 0
#define RIGHT 1
#define FORWARD 0
#define REVERSE 1
#define WHEEL_RADIUS 0.05  //meter
#define WHEEL_SEPERATION 0.33 //meter
#define RAD2DEG(x)      (x * 180 / PI)
#define IMU_HEADER 0x0A
#define IMU_FOOTER 0x0D
#define IMU_600USD_SIZE 80

#include "stdio.h"
#include "test.h"
#include "THESIS_MADGWICK.h"
#include "ROS_SETUP.h"


enum FLAG_MS{
	TWENTY,
	SIXTY,
	ONE_HUNDRED,
	TWO_HUNDRED,
	NUMBER_OF_FLAG
};

typedef struct{
	float roll;
	float pitch;
	float yaw;
}IMU_value;

typedef struct{
	char roll[5];
	char pitch[5];
	char yaw[5];
}IMU_prevalue;

typedef struct{
	float x;
	float y;
	float z;
}IMU_sensor_value;

typedef struct{
	char x[5];
	char y[5];
	char z[5];
}IMU_sensor_prevalue;

extern TIM_HandleTypeDef htim2; //encoder1
extern TIM_HandleTypeDef htim3; //encoder2
extern TIM_HandleTypeDef htim4; //pwm1
extern TIM_HandleTypeDef htim9; //timer interrupt

extern UART_HandleTypeDef huart5;
extern DMA_HandleTypeDef hdma_uart5_rx;

uint8_t imu_receive_buffer[IMU_600USD_SIZE*2];
uint8_t imu_temp[IMU_600USD_SIZE*2];
uint8_t imu_cache[IMU_600USD_SIZE];
IMU_value imu_value;
IMU_prevalue imu_prevalue;
IMU_sensor_prevalue IMU_premag;
IMU_sensor_value IMU_mag;
uint8_t IMU_index=0;
uint8_t serial_buffer[4];
uint8_t serial_cache[4];

bool flag_b[NUMBER_OF_FLAG];

char log_msg[100];

float alpha = 0.15, beta=0.1; //0.002 0.998  //0.055 0.08

uint8_t cnt=0;
uint8_t cnt_1=0;

float goal_cmd_vel[2] = {0.0, 0.0}; //from ros command in linear and angular value
float goal_wheel_vel_PID[2] = {0.0, 0.0};
//float pre_goal_wheel_vel[2] = {0.0, 0.0};

float delta_theta, theta, last_theta = 0.0;
float robot_speed;

uint8_t buffer[3];

float quat[4];

ak8963_soft_iron_corr_t soft_iron={1.1992, 1.20312, 1.1601};
//ak8963_soft_iron_scale_t Soft_iron={{0.0035, 0.0001,-0.0001}, {0.0001, 0.0034, -0.0001}, {-0.0001, -0.0001, 0.0035}};
//ak8963_hard_iron_bias_t Hard_iron={32.9256, 255.7876, -172.9932};
ak8963_soft_iron_scale_t Soft_iron={{0.0053, 0.0003, -0.0001}, {0.0003, 0.0052, 0.0000}, {-0.0001, 0.0000, 0.0056}};
ak8963_hard_iron_bias_t Hard_iron={117.0628, 423.1777, -119.8184};
ScaledData_Def Accel_bias_disp, Gyro_bias_disp;
//uint8_t Data_glob[2];
void IMU_Config(void);
void Low_pass_filter(float* Current, float* Pre, float Alpha);
//void set_goal_vel(uint8_t Wheel);
void Control_motor(void);
void Get_encoder(void);
void get_speed_100ms(void);
void get_odom(void);
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim);
char hello[] = "Hello!!";
void led_toggle (void);
void waitfor_SerialLink(bool isConnected);
void send_logmsg(void);
void update_Variable(bool isConnected);
void update_quaternion(void);
void IMU_600USD_GetSample(void);
void IMU_600USD_Read(void);


MPU_ConfigTypeDef mpu_config;
ScaledData_Def Accel_scaled_disp;
ScaledData_Def Gyro_scaled_disp;
ScaledData_Def Mag_scaled_disp;
class Encoder{
public:
	int16_t delta_encoder_counter=0;
	float encoder=0, round_encoder=0, encoder_speed=0, encoder_speed_inmeter=0, pulse_per_round=0;
	float goal_wheel_vel=0.0, pre_goal_wheel_vel=0.0;
	uint16_t pre_counter=0.0;
	float pre_encoder = 0.0;
	volatile float duty=0;
	uint8_t wheel=0;
	uint32_t pwm_channel=0;
	//static float test;
	double Kp, Ki, Kd;
	double uk=0, uk_1=0, uk_2=0, ek=0, ek_1=0, ek_2=0;
	double u_max = 70; //70
	double u_min = 10; //10    0-200
	double T=0.06;
	Encoder (float pulse_per_round, uint8_t wheel, double Kp, double Ki, double Kd) : pulse_per_round(pulse_per_round), wheel(wheel), Kp(Kp), Ki(Ki), Kd(Kd){
		if(this->wheel == LEFT) pwm_channel = TIM_CHANNEL_1;
		else pwm_channel = TIM_CHANNEL_2;
	}
	void get_speed(void);
	void get_encoder(void);
	void set_dir(uint8_t dir);
	void set_PWM(void);
	void PID_control(void);
};
#endif

#endif /* INC_MAINPP_H_ */
