/*
 * theremin.c
 *
 *  Created on: 20 May 2019
 *      Author: cyprian
 */


#include "theremin.h"

thereminStatus thereminInit(VL53L1X_DEV Dev1, VL53L1X_DEV Dev2)
{

	 HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
	 HAL_Delay(100);

	 if(VL53L1X_init(Dev1) == VL53L1X_ERROR)
	 		return thereminError_Dev1;
	 HAL_Delay(100);

	 HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_RESET);
	 HAL_Delay(100);

	 VL53L1X_setAddress(dev1, VL53L1X_2ND_ADDRESS);
	 HAL_Delay(100);

	 HAL_GPIO_WritePin(XSHUT_GPIO_Port, XSHUT_Pin, GPIO_PIN_SET);
	 HAL_Delay(100);

	 if(VL53L1X_init(Dev2) == VL53L1X_ERROR)
	 		return thereminError_Dev2;
	 HAL_Delay(100);

	 VL53L1X_startContinuous(dev1, 50);
	 VL53L1X_startContinuous(dev2, 50);
	 HAL_Delay(100);

	return thereminOK;
}
