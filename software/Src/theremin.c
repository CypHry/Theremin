/*
 * theremin.c
 *
 *  Created on: 20 May 2019
 *      Author: cyprian
 */


#include "theremin.h"

thereminStatus thereminInit(VL53L1X_DEV Dev1, VL53L1X_DEV Dev2)
{
	HAL_GPIO_WritePin(XSHUT_Pin, XSHUT_GPIO_Port, GPIO_PIN_RESET);

	if(VL53L1X_Init(Dev1) == VL53L1X_Error)
		return thereminError;

	VL53L1X_SetAddress(Dev1, VL53L1X_2ND_ADDRESS);

	HAL_GPIO_WritePin(XSHUT_Pin, XSHUT_GPIO_Port, GPIO_PIN_SET);

	if(VL53L1X_Init(Dev2) == VL53L1X_Error)
		return thereminError;

	 VL53L1X_startContinuous(Dev1, 50);
	 VL53L1X_startContinuous(Dev2, 50);

	return thereminOK;
}
