/*
 * MADGWICK.h
 *
 *  Created on: Feb 26, 2021
 *      Author: chau
 */

#ifndef COMPONENTS_MADGWICK_THESIS_MADGWICK_H_
#define COMPONENTS_MADGWICK_THESIS_MADGWICK_H_

#include "madgwick.h"
#include "MPU9250.h"

#define PI                  3.14159265359
#define DEG2RAD(x)      (x * PI / 180.0)

extern madgwick_handle_t madgwick_handle;
extern ScaledData_Def Accel_scaled_disp;
extern ScaledData_Def Gyro_scaled_disp;
extern ScaledData_Def Mag_scaled_disp;

void MADGWICK_Quaternion_Update(void);
void MADGWICK_Get_Quaternion(float *quat);

#endif /* COMPONENTS_MADGWICK_THESIS_MADGWICK_H_ */
