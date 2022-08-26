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
