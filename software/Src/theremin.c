/*
 * theremin.c
 *
 *  Created on: 20 May 2019
 *      Author: cyprian
 */


#include "theremin.h"

thereminStatus thereminInit(VL53L1X_DEV Dev1, VL53L1X_DEV Dev2)
{
	if(VL53L1X_init(Dev1) == VL53L1X_ERROR)
		return thereminError_Dev1;

	VL53L1X_setAddress(Dev1, VL53L1X_2ND_ADDRESS);

	HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);

	HAL_Delay(100);

	if(VL53L1X_init(Dev2) == VL53L1X_ERROR)
		return thereminError_Dev2;

	 VL53L1X_startContinuous(Dev1, 50);
	 VL53L1X_startContinuous(Dev2, 50);

	return thereminOK;
}
