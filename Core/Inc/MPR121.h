/*
 * MPR121.h
 *
 *  Created on: Aug 26, 2022
 *      Author: ParasolkaJeck
 */

#ifndef MPR121_H_
#define MPR121_H_

#include "main.h"
#include "stm32f4xx_hal.h"

#define MPR_I2C hi2c1
#define MPR_I2C_ADDR 0x4C<<	1


uint8_t MPR121_check_conection();


#endif /* MPR121_H_ */
