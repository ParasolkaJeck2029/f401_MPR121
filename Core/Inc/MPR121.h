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

/*======MPR121 I2C config=======*/
#define MPR_I2C hi2c1
#define MPR_I2C_ADDR 0xB4

/*======MPR121 registers address=======*/
#define MPR_ELE0_7_TOUCH_STATUS 	0x00
#define MPR_ELE8_11_TOUCH_STATUS 	0x01
#define MPR_OOR0_7__STATUS			0x02
#define MPR_OOR8_11__STATUS			0x03
#define MPR_ELE_FILTRED_DATA_START	0x04
#define MPR_ELE_BASELINE_VAl_START	0x1E
#define MPR_ELEPROX_BASELINE_VAL	0x2A
#define MPR_ELE0_TOUCH_THRESHOLD	0x41
#define MPR_ELE0_RELEASE_THRESHOLD	0x42
#define MPR_AUTOCONFIG_CONTROL_0	0x7B
#define MPR_AUTOCONFIG_CONTROL_1	0x7C
#define MPR_AUTOCONFIG_USL			0x7B
#define MPR_AUTOCONFIG_LSL			0x7C
#define MPR_AUTOCONFIG_TARGET_LEVEL	0x7B

/*=====Check connection to MPR121, if return != 0, check config or run scanner======*/
uint8_t MPR121_check_conection();

/*=====Function return first i2c device=====*/
uint8_t I2C_Scanner();

/*=====Reading any i2c device registers =======*/
void I2C_read_register_value(uint16_t dev_addr, uint8_t reg_addr, uint8_t *value);
void I2C_read_16_register_value(uint16_t dev_addr, uint8_t reg_addr, uint16_t *value);

/*=====Writing any i2c device registers =======*/
void I2C_write_register_value(uint16_t dev_addr, uint8_t reg_addr, uint8_t *value);
//void I2C_read_16_register_value(uint16_t dev_addr, uint8_t reg_addr, uint16_t *value);

/*=====Reading MPR121 registers =======*/
void MPR121_Read_register(uint8_t reg_addr, uint8_t * result);
void MPR121_Read_register_16(uint8_t reg_addr, uint16_t * result);

/*=====Writing MPR121 registers =======*/
void MPR121_Write_register(uint8_t reg_addr, uint8_t * value);
void MPR121_Write_register_16(uint8_t reg_addr, uint16_t * value);

void MPR121_Set_AUTO_TARGET(uint8_t target_value);
void MPR121_Set_threshold_value(uint8_t touch,uint8_t release);
#endif /* MPR121_H_ */
