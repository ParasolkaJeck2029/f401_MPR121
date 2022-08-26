/*
 * MPR121.c
 *
 *  Created on: Aug 26, 2022
 *      Author: ParasolkaJeck
 */

#include "MPR121.h"

extern I2C_HandleTypeDef MPR_I2C;

uint8_t MPR121_check_conection(){
	HAL_StatusTypeDef res;
	res = HAL_I2C_IsDeviceReady(&MPR_I2C, MPR_I2C_ADDR, 3, 1000);
	return res;
}

uint8_t I2C_Scanner(){
	HAL_StatusTypeDef res;
	for (uint8_t addr = 0; addr < 0xFF; addr++){
		res = HAL_I2C_IsDeviceReady(&MPR_I2C, addr, 3, 1000);
		if (res == HAL_OK){
			return addr;
		}
	}
	return res;
}

void I2C_read_register_value(uint16_t dev_addr, uint8_t reg_addr, uint8_t *value){
	HAL_I2C_Mem_Read(&MPR_I2C, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, value, 1, 1000);
}

void I2C_read_16_register_value(uint16_t dev_addr, uint8_t reg_addr, uint16_t *value){
	HAL_I2C_Mem_Read(&MPR_I2C, dev_addr, reg_addr, I2C_MEMADD_SIZE_16BIT, value, 2, 1000);
}

void MPR121_Read_register(uint8_t reg_addr, uint8_t * result){
	I2C_read_register_value(MPR_I2C_ADDR, reg_addr, result);
}
void MPR121_Read_register_16(uint8_t reg_addr, uint16_t * result){
	I2C_read_16_register_value(MPR_I2C_ADDR, reg_addr, result);
}


void I2C_write_register_value(uint16_t dev_addr, uint8_t reg_addr, uint8_t *value){
	HAL_I2C_Mem_Write(&MPR_I2C, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, value, 1, 1000);
}

void MPR121_Write_register(uint8_t reg_addr, uint8_t * value){
	I2C_write_register_value(MPR_I2C_ADDR, reg_addr, value);
}
void MPR121_Write_register_16(uint8_t reg_addr, uint16_t * value){
	/*will be added later*/
}

void MPR121_Set_AUTO_TARGET(uint8_t target_value){
	MPR121_Write_register(MPR_AUTOCONFIG_TARGET_LEVEL, &target_value);
}
void MPR121_Set_upperLimit(uint8_t upperLimit_value){
	MPR121_Write_register(MPR_AUTOCONFIG_USL, &upperLimit_value);
}
void MPR121_Set_lowerLimit(uint8_t lowerLimit_value){
	MPR121_Write_register(MPR_AUTOCONFIG_LSL, &lowerLimit_value);
}

void MPR121_Set_threshold_value(uint8_t touch,uint8_t release){
	for (uint8_t i = 0; i < 12; i++){
		MPR121_Write_register(MPR_ELE0_TOUCH_THRESHOLD + i * 2, &touch);
		MPR121_Write_register(MPR_ELE0_RELEASE_THRESHOLD + i * 2, &release);
	}
}
void MPR121_Set_charging_current(uint8_t current){
	uint8_t register_value;
	MPR121_Read_register(MPR_AFE_CONFIG, &register_value);
	uint8_t new_value = (register_value & 0b11100000) | current;
	MPR121_Write_register(MPR_AFE_CONFIG, &new_value);
}
void MPR121_Get_charging_current(uint8_t *result){
	uint8_t register_value;
	MPR121_Read_register(MPR_AFE_CONFIG, &register_value);
	register_value = register_value & 0b00011111;
	*result = register_value;
}
