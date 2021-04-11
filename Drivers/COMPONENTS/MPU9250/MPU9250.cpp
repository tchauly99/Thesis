/*
 * MPU9250.H
 *
 *  Created on: Feb 26, 2021
 *      Author: chau
 */

#include "MPU9250.h"

ak8963_handle_t ak8963_handle_p;
ak8963_cfg_t ak8963_config;

void MPU9250_Config(void){
	MPU6050_Init(&hi2c1);
	mpu_config.Accel_Full_Scale = AFS_SEL_2g;
	mpu_config.ClockSource = Internal_8MHz;
	mpu_config.CONFIG_DLPF = DLPF_184A_188G_Hz;
	mpu_config.Gyro_Full_Scale = FS_SEL_250;
	mpu_config.Sleep_Mode_Bit = 0;
	MPU6050_Config(&mpu_config);
}
void MPU9250_Bypass_En(void){
	MPU6050_Bypass_En();
}
void MPU9250_Bypass_Dis(void){
	MPU6050_Bypass_Dis();
}
void MPU9250_Get_Scaled(ScaledData_Def* accel_scaled, ScaledData_Def* gyro_scaled){
	//MPU9250_Bypass_En();
	MPU6050_Get_Accel_Scale(accel_scaled);
	MPU6050_Get_Gyro_Scale(gyro_scaled);
	//MPU9250_Bypass_Dis();
}
void MPU9250_AG_Calibrate(ScaledData_Def* accel_bias_disp, ScaledData_Def* gyro_bias_disp){
	uint16_t  Gyro_sensitivity  = 131;   // = 131 LSB/degrees/sec
	uint16_t  Accel_sensitivity = 16384;  // = 16384 LSB/g
	uint16_t packet_count, fifo_count;
	uint8_t data[12];

	Sensor_Bias_Def Accel_bias, Gyro_bias;
	MPU6050_Init(&hi2c1);
	MPU9250_AG_Calib_Config();
	HAL_Delay(100); // accumulate 40 samples in 40 milliseconds = 480 bytes
	I2C_Write8(FIFO_EN, 0x00);        // Disable gyro and accelerometer sensors for FIFO
	I2C_Read(FIFO_COUNTH, &data[0], 2); // read FIFO sample count
	//I2C_Read(FIFO_COUNTH, data_glob, 2);
	fifo_count = ((uint16_t)data[0] << 8) | data[1];
	packet_count = fifo_count / 12;// How many sets of full gyro and accelerometer data for averaging
	for(uint16_t i=0; i<packet_count; i++){
		RawData_Def Accel_bias_temp, Gyro_bias_temp;
		//int16_t accel_temp[3] = {0, 0, 0}, gyro_temp[3] = {0, 0, 0};
		I2C_Read(FIFO_R_W, &data[0], 12); // read data for averaging
		Accel_bias_temp.x = (int16_t) (((int16_t)data[0] << 8) | data[1]  ) ;  // Form signed 16-bit integer for each sample in FIFO
		Accel_bias_temp.y = (int16_t) (((int16_t)data[2] << 8) | data[3]  ) ;
		Accel_bias_temp.z = (int16_t) (((int16_t)data[4] << 8) | data[5]  ) ;
		Gyro_bias_temp.x  = (int16_t) (((int16_t)data[6] << 8) | data[7]  ) ;
		Gyro_bias_temp.y  = (int16_t) (((int16_t)data[8] << 8) | data[9]  ) ;
		Gyro_bias_temp.z  = (int16_t) (((int16_t)data[10] << 8) | data[11]) ;

		Accel_bias.x += (int32_t) Accel_bias_temp.x; // Sum individual signed 16-bit biases to get accumulated signed 32-bit biases
		Accel_bias.y += (int32_t) Accel_bias_temp.y;
		Accel_bias.z += (int32_t) Accel_bias_temp.z;

		Gyro_bias.x  += (int32_t) Gyro_bias_temp.x;
		Gyro_bias.y  += (int32_t) Gyro_bias_temp.y;
		Gyro_bias.z  += (int32_t) Gyro_bias_temp.z;
	}
	Accel_bias.x /= (int32_t)packet_count;
	Accel_bias.y /= (int32_t)packet_count;
	Accel_bias.z /= (int32_t)packet_count;
	Gyro_bias.x /= (int32_t)packet_count;
	Gyro_bias.y /= (int32_t)packet_count;
	Gyro_bias.z /= (int32_t)packet_count;
	// Remove gravity from the z-axis accelerometer bias calculation
	if(Accel_bias.z > 0L){
		Accel_bias.z -= (int32_t) Accel_sensitivity;
	}
	else {
		Accel_bias.z += (int32_t) Accel_sensitivity;
	}
	MPU9250_Accel_WriteRegister(&Accel_bias);
	MPU9250_Gyro_WriteRegister(&Gyro_bias);

	accel_bias_disp->x = (float)Accel_bias.x/(float)Accel_sensitivity;
	accel_bias_disp->y = (float)Accel_bias.y/(float)Accel_sensitivity;
	accel_bias_disp->z = (float)Accel_bias.z/(float)Accel_sensitivity;
	gyro_bias_disp->x = (float)Gyro_bias.x/(float)Gyro_sensitivity;
	gyro_bias_disp->y = (float)Gyro_bias.y/(float)Gyro_sensitivity;
	gyro_bias_disp->z = (float)Gyro_bias.z/(float)Gyro_sensitivity;

}
void MPU9250_AG_Calib_Config(void){
	// reset device
	I2C_Write8(PWR_MGMT_1, 0x80); // Write a one to bit 7 reset bit; toggle reset device
	HAL_Delay(100);

	// get stable time source; Auto select clock source to be PLL gyroscope reference if ready
	// else use the internal oscillator, bits 2:0 = 001
	I2C_Write8(PWR_MGMT_1, 0x01);
	I2C_Write8(PWR_MGMT_2, 0x00);
	HAL_Delay(200);

	// Configure device for bias calculation
	I2C_Write8(INT_ENABLE, 0x00);   // Disable all interrupts
	I2C_Write8(FIFO_EN, 0x00);      // Disable FIFO
	I2C_Write8(PWR_MGMT_1, 0x00);   // Turn on internal clock source
	I2C_Write8(I2C_MST_CTRL, 0x00); // Disable I2C master
	I2C_Write8(USER_CTRL, 0x00);    // Disable FIFO and I2C master modes
	I2C_Write8(USER_CTRL, 0x0C);    // Reset FIFO and DMP
	HAL_Delay(15);

	// Configure MPU6050 gyro and accelerometer for bias calculation
	I2C_Write8(MPU_CONFIG, 0x01);      // Set low-pass filter to 188 Hz
	I2C_Write8(SMPLRT_DIV, 0x00);  // Set sample rate to 1 kHz
	I2C_Write8(GYRO_CONFIG, 0x00);  // Set gyro full-scale to 250 degrees per second, maximum sensitivity
	I2C_Write8(ACCEL_CONFIG, 0x00); // Set accelerometer full-scale to 2 g, maximum sensitivity

	// Configure FIFO to capture accelerometer and gyro data for bias calculation
	I2C_Write8(USER_CTRL, 0x40);   // Enable FIFO
	I2C_Write8(FIFO_EN, 0x78);     // Enable gyro and accelerometer sensors for FIFO  (max size 512 bytes in MPU-9150)
}
void MPU9250_Accel_WriteRegister(Sensor_Bias_Def* accel_bias_st){
	uint8_t data_read[2];
	uint8_t data_write[6];
	int16_t accel_bias_factory[3] = {0, 0, 0};
	int16_t mask_bit[3] = {1, 1, 1};  // Define array to hold mask bit for each accelerometer bias axis
	int16_t accel_bias[3];
	accel_bias[0] = accel_bias_st->x;
	accel_bias[1] = accel_bias_st->y;
	accel_bias[2] = accel_bias_st->z;

	I2C_Read(XA_OFFSET_H, &data_read[0], 2);  // Read factory accelerometer trim values
	accel_bias_factory[0] = ((int16_t)data_read[0] << 8) | data_read[1];
	I2C_Read(YA_OFFSET_H, &data_read[0], 2);
	accel_bias_factory[1] = ((int16_t)data_read[0] << 8) | data_read[1];
	I2C_Read(ZA_OFFSET_H, &data_read[0], 2);
	accel_bias_factory[2] = ((int16_t)data_read[0] << 8) | data_read[1];

	for (int i = 0; i < 3; i++) {
		if (accel_bias_factory[i] % 2) {
			mask_bit[i] = 0;
		}
		accel_bias_factory[i] -= (int16_t)accel_bias[i] >> 3;  // Subtract calculated averaged accelerometer bias scaled to 2048 LSB/g
		if (mask_bit[i]) {
			accel_bias_factory[i] &= ~mask_bit[i];  // Preserve temperature compensation bit
		} else {
			accel_bias_factory[i] |= 0x0001;  // Preserve temperature compensation bit
		}
	}
	data_write[0] = (accel_bias_factory[0] >> 8) & 0xFF;
	data_write[1] = (accel_bias_factory[0]) & 0xFF;
	data_write[2] = (accel_bias_factory[1] >> 8) & 0xFF;
	data_write[3] = (accel_bias_factory[1]) & 0xFF;
	data_write[4] = (accel_bias_factory[2] >> 8) & 0xFF;
	data_write[5] = (accel_bias_factory[2]) & 0xFF;

	// Push accelerometer biases to hardware registers
	I2C_Write8(XA_OFFSET_H, data_write[0]);
	I2C_Write8(XA_OFFSET_L, data_write[1]);
	I2C_Write8(YA_OFFSET_H, data_write[2]);
	I2C_Write8(YA_OFFSET_L, data_write[3]);
	I2C_Write8(ZA_OFFSET_H, data_write[4]);
	I2C_Write8(ZA_OFFSET_L, data_write[5]);
}
void MPU9250_Gyro_WriteRegister(Sensor_Bias_Def* gyro_bias){
	uint8_t data_write[6];
	data_write[0] = (-gyro_bias->x / 4  >> 8) & 0xFF; // Divide by 4 to get 32.9 LSB per deg/s to conform to expected bias input format
	data_write[1] = (-gyro_bias->x / 4)       & 0xFF; // Biases are additive, so change sign on calculated average gyro biases
	data_write[2] = (-gyro_bias->y / 4  >> 8) & 0xFF;
	data_write[3] = (-gyro_bias->y / 4)       & 0xFF;
	data_write[4] = (-gyro_bias->z / 4  >> 8) & 0xFF;
	data_write[5] = (-gyro_bias->z / 4)       & 0xFF;
	// Push gyro biases to hardware registers
	I2C_Write8(XG_OFFSET_H, data_write[0]);
	I2C_Write8(XG_OFFSET_L, data_write[1]);
	I2C_Write8(YG_OFFSET_H, data_write[2]);
	I2C_Write8(YG_OFFSET_L, data_write[3]);
	I2C_Write8(ZG_OFFSET_H, data_write[4]);
	I2C_Write8(ZG_OFFSET_L, data_write[5]);
}
void MPU9250_AK8963_Config(void){
	ak8963_config.opr_mode = AK8963_MODE_CONT_MEASUREMENT_2;
	ak8963_config.mfs_sel = AK8963_MFS_16BIT;
	ak8963_config.if_protocol = AK8963_IF_I2C;
	ak8963_handle_p = ak8963_init(&ak8963_config, &hi2c1);
}

void MPU9250_AK8963_AutoCalib(void){
	ak8963_auto_calib(ak8963_handle_p);
}
void MPU9250_AK8963_GetMag_Scaled(ScaledData_Def* mag_scaled){
	ak8963_scale_data_t mag_scaled_;
	ak8963_get_mag_scale(ak8963_handle_p, &mag_scaled_);
	/*Transfer magnetometer frame to gyro frame*/
	mag_scaled->x = mag_scaled_.y_axis;
	mag_scaled->y = mag_scaled_.x_axis;
	mag_scaled->z = -mag_scaled_.z_axis;
}
void MPU9250_AK8963_Set_Hard_Soft_Iron(ak8963_soft_iron_scale_t *soft_iron, ak8963_hard_iron_bias_t hard_iron){
	ak8963_set_soft_iron_scale(ak8963_handle_p, soft_iron);
	ak8963_set_hard_iron_bias(ak8963_handle_p, hard_iron);
}
void MPU9250_AK8963_GetSample(void){
	int i, sample_count;
	sample_count = 2000;
	int16_t Sample[3];
	char buffer[50];
	int n=0;
	for (i = 0; i < sample_count + 100; i++)            /*!< Dismiss 100 first value */
	    {
	        if (i > 100 && i <= (sample_count + 100))
	        {
	            ak8963_raw_data_t mag_raw;
	            ak8963_get_mag_raw(ak8963_handle_p, &mag_raw);
	            Sample[0] = (mag_raw.x_axis);
	            Sample[1] = (mag_raw.y_axis);
	            Sample[2] = (mag_raw.z_axis);
	            n = sprintf(buffer,"%10d, %10d, %10d \n", Sample[0], Sample[1], Sample[2]);
	            HAL_UART_Transmit(&huart4, (uint8_t*)buffer, n, 100);
	            //MPU9250_AK8963_send_uart(Sample);
	        }
	        HAL_Delay(15);
	    }

}
//void MPU9250_AK8963_send_uart(float* f){
//	char c=0;
//	uint8_t nguyen=0, du=0;
//	float ff;
//	for(int k=0; k<3; k++){
//		du=f[k];
//		ff = f[k]-(float)du;
//		for (int i=0; i<4; i++){
//			nguyen = du/pow(10, 3-i);
//			du = du - nguyen*pow(10, 3-i);
//			nguyen+=48;
//			HAL_UART_Transmit(&huart4, &nguyen, 1, 100);
//		}
//		du = (int)(ff*10000);
//		c = '.';
//		nguyen = (uint8_t)c;
//		HAL_UART_Transmit(&huart4, &nguyen, 1, 100);
//		for (int i=0; i<4; i++){
//			nguyen = du/pow(10, 3-i);
//			du = du - nguyen*pow(10, 3-i);
//			nguyen+=48;
//			HAL_UART_Transmit(&huart4, &nguyen, 1, 100);
//		}
//		c='\n';
//		nguyen=(uint8_t)c;
//		HAL_UART_Transmit(&huart4, &nguyen, 1, 100);
//	}
//	c='\n';
//	nguyen=(uint8_t)c;
//	HAL_UART_Transmit(&huart4, &nguyen, 1, 100);
//}
