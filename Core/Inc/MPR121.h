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
#define MPR_ELE0_7_TOUCH_STATUS 		0x00
#define MPR_ELE8_11_TOUCH_STATUS	 	0x01
#define MPR_OOR0_7__STATUS				0x02
#define MPR_OOR8_11__STATUS				0x03
#define MPR_ELE_FILTRED_DATA_START		0x04
#define MPR_ELE_BASELINE_VAl_START		0x1E
#define MPR_ELEPROX_BASELINE_VAL		0x2A
#define MPR_MHD_RISING					0x2B
#define MPR_NHD_RISING					0x2C
#define MPR_NCL_RISING					0x2D
#define MPR_FDL_RISING					0x2E
#define MPR_MHD_FALLING					0x2F
#define MPR_NHD_FALLING					0x30
#define MPR_NCL_FALLING					0x31
#define MPR_FDL_FALLING					0x32
#define MPR_NHD_TOUCHED					0x33
#define MPR_NCL_TOUCHED					0x34
#define MPR_FDL_TOUCHED					0x35
#define MPR_ELEPTOX_MHD_RISING			0x36
#define MPR_ELEPTOX_NHD_RISING			0x37
#define MPR_ELEPTOX_NCL_RISING			0x38
#define MPR_ELEPTOX_FDL_RISING			0x39
#define MPR_ELEPTOX_MHD_FALLING			0x3A
#define MPR_ELEPTOX_NHD_FALLING			0x3B
#define MPR_ELEPTOX_NCL_FALLING			0x3C
#define MPR_ELEPTOX_FDL_FALLING			0x3D
#define MPR_ELEPTOX_NHD_TOUCHED			0x3E
#define MPR_ELEPTOX_NCL_TOUCHED			0x3F
#define MPR_ELEPTOX_FDL_TOUCHED			0x40
#define MPR_ELE0_TOUCH_THRESHOLD		0x41
#define MPR_ELE0_RELEASE_THRESHOLD		0x42
#define MPR_ELEPROX_TOUCH_THRESHOLD		0x59
#define MPR_ELEPROX_RELEASE_THRESHOLD	0x5A
#define MPR_DEBOUNSE_TOUCH_RELEASE		0x5B
#define MPR_AFE_CONFIG					0x5C
#define MPR_FILTER_CONFIG				0x5D
#define MPR_ELE_CONFIG					0x5E
#define MPR_ELE0_CURRENT				0x5F
#define MPR_ELEPROX_ELE_CURRENT			0x6B
#define MPR_ELE0_1_CHARGE_TIME			0x6C
#define MPR_ELEPROX_CHARGE_TIME			0x72
#define MPR_GPIO_CNTL_REGISTER0			0x73
#define MPR_GPIO_CNTL_REGISTER1			0x74
#define MPR_GPIO_DATA_REGISTER			0x75
#define MPR_GPIO_DIRECTION_REGISTER		0x76
#define MPR_GPIO_ENABLE_REGISTER		0x77
#define MPR_GPIO_DATA_SET_REGISTER		0x78
#define MPR_GPIO_DATA_CLEAR_REGISTER	0x79
#define MPR_GPIO_DATA_TOGGLE_REGISTER	0x7A
#define MPR_AUTOCONFIG_CONTROL_0		0x7B
#define MPR_AUTOCONFIG_CONTROL_1		0x7C
#define MPR_AUTOCONFIG_USL				0x7B
#define MPR_AUTOCONFIG_LSL				0x7C
#define MPR_AUTOCONFIG_TARGET_LEVEL		0x7B
#define MPR_SOFTRESET					0x80
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

void MPR121_Set_threshold_value(uint8_t touch,uint8_t release);
void MPR121_Set_charging_current(uint8_t current);
void MPR121_Get_charging_current(uint8_t *result);
/*======Autoconfig functions=======*/
void MPR121_Set_AUTO_TARGET(uint8_t target_value);
void MPR121_Set_upperLimit(uint8_t upperLimit_value);
void MPR121_Set_lowerLimit(uint8_t lowerLimit_value);

void MPR121_Set_sampling_time(uint8_t samp_time);
void MPT121_Set_debounce(uint8_t touch, uint8_t release);
uint8_t MPR121_init();

uint16_t MPR121_read_buttons_status();
uint8_t MPR121_read_one_button(uint8_t button_nomer);
void MPR121_read_array_buttons(uint8_t * b_array);

uint8_t MPR121_Get_over_current_flag();
#endif /* MPR121_H_ */
