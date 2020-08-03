/*
 * VL53L1X.h
 *
 *  Created on: 12 Apr 2019
 *      Author: cyprian
 */

#ifndef VL53L1X_H_
#define VL53L1X_H_

#include "VL53L1X_register_map.h"
#include "stm32l4xx_hal.h"
#include <stdbool.h>

#define VL53L1X_ADDRESS_DEFAULT		(0x29<<1)
#define VL53L1X_2ND_ADDRESS			54
#define	VL53L1X_TIMING_GUARD		4528
#define VL53L1X_TARGET_RATE			0x0A00
#define BASE_TIMEOUT 10
#define BYTE_TIMEOUT 1


typedef enum
{
	Short,
	Medium,
	Long,
	Unknown
} DistanceMode;

typedef struct
{
	uint8_t   I2cDevAddr;
	I2C_HandleTypeDef *I2cHandle;
	uint16_t	fast_osc_frequency;
	uint16_t	osc_calibrate_val;
	DistanceMode distance_mode;
	bool calibrated;
	uint8_t saved_vhv_init;
	uint8_t saved_vhv_timeout;
} VL53L1X_Dev_t;
typedef VL53L1X_Dev_t* VL53L1X_DEV;

typedef enum
{
	RangeValid 					= 	0,
	SigmaFail 					= 	1,
	SignalFail 					= 	2,
	RangeValidMinRangeClipped	= 	3,
	OutOfBoundsFail 			= 	4,
    HardwareFail              	=   5,
    RangeValidNoWrapCheckFail 	=   6,
    WrapTargetFail            	=   7,
	ProcessingFail            	=   8,
    XtalkSignalFail           	=   9,
    SynchronizationInt          =  10,
    MinRangeFail              	=  13,
    None                      	= 255,
} RangeStatus;

typedef struct
{
  uint16_t range_mm;
  RangeStatus range_status;
  float peak_signal_count_rate_MCPS;
  float ambient_count_rate_MCPS;
} RangingData;

typedef struct
{
    uint8_t range_status;
    uint16_t dss_actual_effective_spads_sd0;
    uint16_t ambient_count_rate_mcps_sd0;
    uint16_t final_crosstalk_corrected_range_mm_sd0;
    uint16_t peak_signal_count_rate_crosstalk_corrected_mcps_sd0;
} ResultBuffer;

typedef enum
{
	VL53L1X_OK = 0,
	VL53L1X_ERROR = 1,
	VL53L1X_TIMEOUT = 2
}VL53L1X_Status;

VL53L1X_Status VL53L1X_init(VL53L1X_DEV Dev);

void VL53L1X_setAddress(VL53L1X_DEV Dev, uint8_t new_addr);

void VL53L1X_writeReg(VL53L1X_DEV Dev, uint16_t reg, uint8_t value);
void VL53L1X_writeReg16Bit(VL53L1X_DEV Dev, uint16_t reg, uint16_t value);
void VL53L1X_writeReg32Bit(VL53L1X_DEV Dev, uint16_t reg, uint32_t value);
uint8_t VL53L1X_readReg(VL53L1X_DEV Dev, uint16_t reg);
uint16_t VL53L1X_readReg16Bit(VL53L1X_DEV Dev, uint16_t reg);
uint32_t VL53L1X_readReg32Bit(VL53L1X_DEV Dev, uint16_t reg);

VL53L1X_Status VL53L1X_setDistanceMode(VL53L1X_DEV Dev, DistanceMode mode);

VL53L1X_Status VL53L1X_setMeasurementTimingBudget(VL53L1X_DEV Dev, uint32_t budget_us);
uint32_t VL53L1X_getMeasurementTimingBudget(VL53L1X_DEV Dev);
uint32_t VL53L1X_calcMacroPeriod(VL53L1X_DEV Dev, uint8_t vcsel_period);
uint32_t VL53L1X_timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us);
uint32_t VL53L1X_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us);

uint32_t VL53L1X_decodeTimeout(uint16_t reg_val);
uint16_t VL53L1X_encodeTimeout(uint32_t timeout_mclks);

void VL53L1X_startContinuous(VL53L1X_DEV Dev, uint32_t period_ms);
void VL53L1X_stopContinuous(VL53L1X_DEV Dev);

VL53L1X_Status VL53L1X_dataReady(VL53L1X_DEV Dev);

void VL53L1X_updateDSS(VL53L1X_DEV Dev, ResultBuffer* results);
uint16_t VL53L1X_read(VL53L1X_DEV Dev, RangingData* data, ResultBuffer* results);

void VL53L1X_setupManualCalibration(VL53L1X_DEV Dev);
void VL53L1X_readResults(VL53L1X_DEV Dev, ResultBuffer* results);
void VL53L1X_getRangingData(RangingData* data, ResultBuffer* results);
float VL53L1X_countRateFixedToFloat(uint16_t count_rate_fixed);


#endif /* VL53L1X_H_ */
