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
#define MPR_I2C_ADDR 0xB4

#define MPR_ELE0_7_TOUCH_STATUS 	0x00
#define MPR_ELE8_11_TOUCH_STATUS 	0x01


uint8_t MPR121_check_conection();

uint8_t I2C_Scaner();

void I2C_read_register_value(uint16_t dev_addr, uint8_t reg_addr, uint8_t *value);
void I2C_read_16_register_value(uint16_t dev_addr, uint8_t reg_addr, uint16_t *value);

#endif /* MPR121_H_ */
