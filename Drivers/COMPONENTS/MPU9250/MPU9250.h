/*
 * MPU9250.h
 *
 *  Created on: Jan 26, 2021
 *      Author: tchauly99
 */

#ifndef MPU9250_MPU9250_H_
#define MPU9250_MPU9250_H_

#include "TJ_MPU6050.h"
#include "ak8963.h"
#include "MPU9250_RegMap.h"
#include <stdlib.h>
#include <stdio.h>
#include <iostream>

extern UART_HandleTypeDef huart4;

typedef struct{
	int32_t x;
	int32_t y;
	int32_t z;
}Sensor_Bias_Def;

extern I2C_HandleTypeDef hi2c1;
extern MPU_ConfigTypeDef mpu_config;

typedef struct sample{
	float x;
	float y;
	float z;
}sample_def;

void MPU9250_Config(void);
void MPU9250_Get_Scaled(ScaledData_Def* accel_scaled, ScaledData_Def* gyro_scaled);
void MPU9250_AG_Calib_Config(void);
void MPU9250_Accel_WriteRegister(Sensor_Bias_Def* accel_bias_st);
void MPU9250_Gyro_WriteRegister(Sensor_Bias_Def* gyro_bias);
void MPU9250_AG_Calibrate(ScaledData_Def* accel_bias_disp, ScaledData_Def* gyro_bias_disp);
void MPU9250_AK8963_Config(void);
void MPU9250_AK8963_AutoCalib(void);
void MPU9250_AK8963_GetMag_Scaled(ScaledData_Def* mag_scaled);
void MPU9250_AK8963_Set_Hard_Soft_Iron(ak8963_soft_iron_scale_t *soft_iron_p, ak8963_hard_iron_bias_t hard_iron_p);
void MPU9250_AK8963_GetSample(void);
//void MPU9250_AK8963_send_uart(float* f);
void MPU9250_Bypass_En(void);
void MPU9250_Bypass_Dis(void);

#endif /* MPU9250_MPU9250_H_ */
