/*
 * roshihi.h
 *
 *  Created on: Feb 2, 2021
 *      Author: chau
 */

#ifndef ROSSERIAL_INCLUDE_ROSHIHI_H_
#define ROSSERIAL_INCLUDE_ROSHIHI_H_
#include "ros/node_handle.h"
#include "STM32Hardware.h"

namespace ros
{
typedef NodeHandle_<STM32Hardware, 25, 25, BUF_SIZE, BUF_SIZE> NodeHandle;
}
#endif /* ROSSERIAL_INCLUDE_ROSHIHI_H_ */
