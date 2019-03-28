
/* 
* This file is part of VL53L1 Platform 
* 
* Copyright (c) 2016, STMicroelectronics - All Rights Reserved 
* 
* License terms: BSD 3-clause "New" or "Revised" License. 
* 
* Redistribution and use in source and binary forms, with or without 
* modification, are permitted provided that the following conditions are met: 
* 
* 1. Redistributions of source code must retain the above copyright notice, this 
* list of conditions and the following disclaimer. 
* 
* 2. Redistributions in binary form must reproduce the above copyright notice, 
* this list of conditions and the following disclaimer in the documentation 
* and/or other materials provided with the distribution. 
* 
* 3. Neither the name of the copyright holder nor the names of its contributors 
* may be used to endorse or promote products derived from this software 
* without specific prior written permission. 
* 
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" 
* AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE 
* IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
* DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE 
* FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL 
* DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
* SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER 
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, 
* OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
* OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE. 
* 
*/

#include "vl53l1_platform.h"
#include <string.h>
#include <time.h>
#include <math.h>

#define BASE_TIMEOUT 10
#define BYTE_TIMEOUT 1

uint8_t I2C_buffer[256];

int I2C_write(VL53L1_DEV Dev, uint8_t *data, uint16_t size){
	int status;
	int timeout = BASE_TIMEOUT + size*BYTE_TIMEOUT;
	status = HAL_I2C_Master_Transmit(Dev->I2cHandle, Dev->I2cDevAddr, data, size, timeout);

	return status;
}

int I2C_read(VL53L1_DEV Dev, uint8_t *data, uint16_t size){
	int status;
	int timeout = BASE_TIMEOUT + size*BYTE_TIMEOUT;
	status = HAL_I2C_Master_Receive(Dev->I2cHandle, Dev->I2cDevAddr, data, size, timeout);

	return status;
}

int8_t VL53L1_WrByte(VL53L1_DEV Dev, uint16_t index, uint8_t data) {

	int8_t status = 0;
	status = HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->I2cDevAddr, index, 1, &data, 1, BASE_TIMEOUT);

	return status; // to be implemented
}

int8_t VL53L1_WrWord(VL53L1_DEV Dev, uint16_t index, uint16_t data) {

	int8_t status = 0;
	uint8_t tmp [2];
	*tmp = data>>8;
	tmp[1] = data & 0x00FF;
	status = HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->I2cDevAddr, index, 1, tmp, 2, BASE_TIMEOUT+2*BYTE_TIMEOUT);

	return status; // to be implemented
}

int8_t VL53L1_WrDWord(VL53L1_DEV Dev, uint16_t index, uint32_t data) {

	int8_t status = 0;
	uint8_t tmp [4];
	*tmp = (data>>24) & 0xFF;
	tmp[1] = (data>>16) & 0xFF;
	tmp[2] = (data>>8) & 0xFF;
	tmp[3] = (data>>0) & 0xFF;

	status = HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->I2cDevAddr, index, 1, tmp, 4, BASE_TIMEOUT+4*BYTE_TIMEOUT);

	return status; // to be implemented
}

int8_t VL53L1_RdByte(VL53L1_DEV Dev, uint16_t index, uint8_t *data) {

	int8_t status = 0;
	status = HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->I2cDevAddr, index, 1, data, 1, BASE_TIMEOUT);

	return status; // to be implemented
}

int8_t VL53L1_RdWord(VL53L1_DEV Dev, uint16_t index, uint16_t *data) {

	int8_t status = 0;
	uint8_t tmp[2];
	status = HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->I2cDevAddr, index, 1, tmp, 2, BASE_TIMEOUT+2*BYTE_TIMEOUT);

	*data = (tmp[0]<<8 | tmp[1]);
	return status; // to be implemented
}

int8_t VL53L1_RdDWord(VL53L1_DEV Dev, uint16_t index, uint32_t *data) {

	int8_t status = 0;
	uint8_t tmp[4];
	status = HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->I2cDevAddr, index, 1, tmp, 4, BASE_TIMEOUT+4*BYTE_TIMEOUT);

	*data = (tmp[0]<<24) | (tmp[1]<<16) | (tmp[2])<<8 | tmp[3];
	return status; // to be implemented
}

int8_t VL53L1_WaitMs(VL53L1_Dev_t *pdev, int32_t wait_ms){
	HAL_Delay(wait_ms); // nie wiem nie mogę tego znaleźć po co to jest to nigdzie nie jest użyte XD
	return 0; // to be implemented
}
