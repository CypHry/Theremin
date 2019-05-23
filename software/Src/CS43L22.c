/*
 * CS43L22.C
 *
 *  Created on: May 20, 2019
 *      Author: bum
 */

#include <CS43L22.h>

void CS43L22_Init(CS43L22_DEV Dev)
{
	uint8_t control[10]={0};
	HAL_StatusTypeDef status_ = HAL_ERROR;
	uint8_t id=0;
	uint8_t power=0;



	status_ = HAL_I2C_IsDeviceReady(Dev->I2cHandle, Dev->deviceAddr,10, 10);
	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_CHIPID_ADDR, 1, &id, 1, 10);

	/* Keep Codec powered OFF */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_POWER_CTL1,1, 0x01, 1, 10);
//	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_POWER_CTL1,1, &power, 1, 10);

	/* Output Device */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_POWER_CTL2,1, 0xAF, 1, 10);
//	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_POWER_CTL2,1, &control[1], 1, 10);


	/* Clock configuration: Auto detection */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_CLOCKING_CTL,1, 0x81, 1, 10);
//	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_CLOCKING_CTL,1, &control[2], 1, 10);


	/* 24bit I2S data	 */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_INTERFACE_CTL1,1, 0b00010101, 1, 10);
//	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_INTERFACE_CTL1,1, &control[3], 1, 10);




	/* Set the Master volume */ //TODO
	//	counter += cs43l22_SetVolume(DeviceAddr, Volume);
	//	counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MASTER_A_VOL, convertedvol - 0xE7);
	//	counter += CODEC_IO_Write(DeviceAddr, CS43L22_REG_MASTER_B_VOL, convertedvol - 0xE7);

	/* Additional configuration for the CODEC. These configurations are done to reduce
	    the time needed for the Codec to power off. If these configurations are removed,
	    then a long delay should be added between powering off the Codec and switching
	    off the I2S peripheral MCLK clock (which is the operating clock for Codec).
	    If this delay is not inserted, then the codec will not shut down properly and
	    it results in high noise after shut down. */

	/* Disable the analog soft ramp */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_ANALOG_ZC_SR_SETT,1, 0x00, 1, 10);
	//HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_ANALOG_ZC_SR_SETT,1, &control[4], 1, 10);


	/* Disable the digital soft ramp */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_MISC_CTL,1, 0x04, 1, 10);
	//HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_MISC_CTL,1, &control[5], 1, 10);


	/* Disable the limiter attack level */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_LIMIT_CTL1,1, 0x00, 1, 10);
	//HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_LIMIT_CTL1,1, &control[6], 1, 10);


	/* Adjust Bass and Treble levels */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_TONE_CTL,1, 0x0F, 1, 10);
	//HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_TONE_CTL,1, &control[7], 1, 10);


	/* Adjust PCM volume level */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_PCMA_VOL,1, 0x0A, 1, 10);
	//HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_PCMA_VOL,1, &control[8], 1, 10);

	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_PCMB_VOL,1, 0x0A, 1, 10);
	//HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_PCMB_VOL,1, &control[9], 1, 10);


	/* Required Initilization Settings */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, 0x00,1, 0x99, 1, 10);
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, 0x80,1, 0x47, 1, 10);
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, 0x32,1, 0b10111011, 1, 10);
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, 0x32,1, 0b00111011, 1, 10);
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, 0x00,1, 0x00, 1, 10);

}

void CS43L22_Play(CS43L22_DEV Dev)
{
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_MISC_CTL,1, 0x06, 1, 10);

	/* Mute OFF */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_HEADPHONE_A_VOL,1, 0x00, 1, 10);
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_HEADPHONE_B_VOL,1, 0x00, 1, 10);
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_POWER_CTL2,1, 0xAF, 1, 10);

	/* Power on Codec */
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->deviceAddr, CS43L22_REG_POWER_CTL1,1, 0x9E, 1, 10);
}

