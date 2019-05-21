/*
 * theremin.h
 *
 *  Created on: 20 May 2019
 *      Author: cyprian
 */

#ifndef THEREMIN_H_
#define THEREMIN_H_

#include "VL53L1X.h"
#include "gen_sinewave.h"


typedef enum {
	thereminOK		= 0,
	thereminError	= 1
} thereminStatus;

thereminStatus thereminInit(VL53L1X_DEV Dev1, VL53L1X_DEV Dev2);

#endif /* THEREMIN_H_ */
