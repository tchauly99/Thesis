/*
 * MADGWICK.cpp
 *
 *  Created on: Feb 26, 2021
 *      Author: tchauly99
 */

#include <THESIS_MADGWICK.h>

madgwick_t m = {0.5, 17.0, 1, 0.0, 0.0, 0.0};
madgwick_handle_t madgwick_handle=&m;



void MADGWICK_Quaternion_Update(void){
	ScaledData_Def accel_scaled;
	ScaledData_Def gyro_scaled;
	ScaledData_Def mag_scaled;
	MPU9250_Get_Scaled(&accel_scaled, &gyro_scaled);
	memcpy(&Accel_scaled_disp, &accel_scaled, sizeof(accel_scaled));
	memcpy(&Gyro_scaled_disp, &gyro_scaled, sizeof(gyro_scaled));
	MPU9250_AK8963_GetMag_Scaled(&mag_scaled);
	memcpy(&Mag_scaled_disp, &mag_scaled, sizeof(mag_scaled));

	madgwick_update_9dof(madgwick_handle,
			 DEG2RAD(gyro_scaled.x),
			 DEG2RAD(gyro_scaled.y),
			 DEG2RAD(gyro_scaled.z),
			 accel_scaled.x/10.0,
			 accel_scaled.y/10.0,
			 accel_scaled.z/10.0,
			 mag_scaled.x,
			 mag_scaled.y,
			 mag_scaled.z);
//	madgwick_update_6dof(madgwick_handle,
//				 DEG2RAD(gyro_scaled.x),
//				 DEG2RAD(gyro_scaled.y),
//				 DEG2RAD(gyro_scaled.z),
//				 accel_scaled.x/10.0,
//				 accel_scaled.y/10.0,
//				 accel_scaled.z/10.0);
}

void MADGWICK_Get_Quaternion(float *quat){
	madgwick_quat_data_t quat_data;
	madgwick_get_quaternion(madgwick_handle, &quat_data);
	quat[0] = quat_data.q0;
	quat[1] = quat_data.q1;
	quat[2] = quat_data.q2;
	quat[3] = quat_data.q3;
}
