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
	HAL_I2C_Mem_Read(&MPR_I2C, dev_addr, reg_addr, I2C_MEMADD_SIZE_8BIT, (uint8_t *)value, 2, 1000);
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

uint8_t MPR121_init(){
	HAL_StatusTypeDef res = HAL_OK;
	res = MPR121_check_conection();
	if (res != HAL_OK){
		return res;
		printf("Error connection\r\n");
	}
	struct {
		uint8_t MHD_RISING;
		uint8_t NHD_RISING;
		uint8_t NCL_RISING;
		uint8_t FDL_RISING;
		uint8_t MHD_FALLING;
		uint8_t NHD_FALLING;
		uint8_t NCL_FALLING;
		uint8_t FDL_FALLING;
		uint8_t FILTER_CONFIG;
		uint8_t ELE_CONFIG;
		uint8_t AUTO_CONFIG0;
	}init_config_value;

	init_config_value.MHD_RISING = 0x01;
	init_config_value.NHD_RISING = 0x01;
	init_config_value.NCL_RISING = 0x0E;
	init_config_value.FDL_RISING = 0x00;

	init_config_value.MHD_FALLING = 0x01;
	init_config_value.NHD_FALLING = 0x05;
	init_config_value.NCL_FALLING = 0x01;
	init_config_value.FDL_FALLING = 0x02;

	init_config_value.FILTER_CONFIG = 0x04;
	init_config_value.ELE_CONFIG = 0x0C;
	init_config_value.AUTO_CONFIG0 = 0x0B;


	/*=====Section A of datasheet======*/
	MPR121_Write_register(MPR_MHD_RISING, &init_config_value.MHD_RISING);
	MPR121_Write_register(MPR_NHD_RISING, &init_config_value.NHD_RISING);
	MPR121_Write_register(MPR_NCL_RISING, &init_config_value.NCL_RISING);
	MPR121_Write_register(MPR_FDL_RISING, &init_config_value.FDL_RISING);
	/*=====Section B of datasheet======*/
	MPR121_Write_register(MPR_MHD_FALLING, &init_config_value.MHD_FALLING);
	MPR121_Write_register(MPR_NHD_FALLING, &init_config_value.NHD_FALLING);
	MPR121_Write_register(MPR_NCL_FALLING, &init_config_value.NCL_FALLING);
	MPR121_Write_register(MPR_FDL_FALLING, &init_config_value.FDL_FALLING);
	/*=====Section C of datasheet======*/
	MPR121_Set_threshold_value(0x0F, 0x0A);
	/*=====Section D of datasheet======*/
	MPR121_Write_register(MPR_FILTER_CONFIG, &init_config_value.FILTER_CONFIG);
	/*=====Section E of datasheet======*/
	MPR121_Write_register(MPR_ELE_CONFIG, &init_config_value.ELE_CONFIG);
	/*=====Section F of datasheet======*/
	MPR121_Write_register(MPR_AUTOCONFIG_CONTROL_0, &init_config_value.AUTO_CONFIG0);
	//MPR121_Set_threshold_value(12, 6);
	MPR121_Set_AUTO_TARGET(180);
	MPR121_Set_lowerLimit(130);
	MPR121_Set_upperLimit(200);

	MPR121_Set_charging_current(16);

	return res;
}

uint16_t MPR121_read_buttons_status(){
	uint16_t buttons_registers;
	MPR121_Read_register_16(MPR_ELE0_7_TOUCH_STATUS, &buttons_registers);
	return buttons_registers & 0x0FFF;
}

