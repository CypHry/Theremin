/*
 * VL53L1X.c
 *
 *  Created on: 12 Apr 2019
 *      Author: cyprian
 */

#include "VL53L1X.h"



void VL53L1X_writeReg(VL53L1X_DEV Dev, uint16_t reg, uint8_t value)
{
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->I2cDevAddr, reg, 2, &value, 1, BASE_TIMEOUT);
}

void VL53L1X_writeReg16Bit(VL53L1X_DEV Dev, uint16_t reg, uint16_t value)
{
	uint8_t tmp [2];
	*tmp = value>>8;
	tmp[1] = value & 0x00FF;
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->I2cDevAddr, reg, 2, tmp, 2, BASE_TIMEOUT+2*BYTE_TIMEOUT);
}
void VL53L1X_writeReg32Bit(VL53L1X_DEV Dev, uint16_t reg, uint32_t value)
{
	uint8_t tmp [4];
	*tmp = (value>>24) & 0xFF;
	tmp[1] = (value>>16) & 0xFF;
	tmp[2] = (value>>8) & 0xFF;
	tmp[3] = (value>0) & 0xFF;
	HAL_I2C_Mem_Write(Dev->I2cHandle, Dev->I2cDevAddr, reg, 2, tmp, 4, BASE_TIMEOUT+4*BYTE_TIMEOUT);
}

uint8_t VL53L1X_readReg(VL53L1X_DEV Dev, uint16_t reg)
{
	uint8_t value;
	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->I2cDevAddr, reg, 2, &value, 1, BASE_TIMEOUT);
	return value;
}

uint16_t VL53L1X_readReg16Bit(VL53L1X_DEV Dev, uint16_t reg)
{
	uint8_t tmp[2];
	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->I2cDevAddr, reg, 2, tmp, 2, BASE_TIMEOUT+2*BYTE_TIMEOUT);
	return tmp[0]<<8 | tmp[1];
}

uint32_t VL53L1X_readReg32Bit(VL53L1X_DEV Dev, uint16_t reg)
{
	uint8_t tmp[4];
	HAL_I2C_Mem_Read(Dev->I2cHandle, Dev->I2cDevAddr, reg, 2, tmp, 4, BASE_TIMEOUT+4*BYTE_TIMEOUT);
	return (tmp[0]<<24) | (tmp[1]<<16) | (tmp[2])<<8 | tmp[3];
}


VL53L1X_Status VL53L1X_init(VL53L1X_DEV Dev)
{
	Dev->calibrated = false;
	Dev->fast_osc_frequency = 0;
	Dev->osc_calibrate_val = 0;
	Dev->distance_mode = Unknown;
	Dev->saved_vhv_init = 0;
	Dev->saved_vhv_timeout = 0;
	Dev->I2cDevAddr = VL53L1X_ADDRESS_DEFAULT;

	VL53L1X_writeReg(Dev, VL53L1_SOFT_RESET, 0x00);
	HAL_Delay(100);
	VL53L1X_writeReg(Dev, VL53L1_SOFT_RESET, 0x01);

	HAL_Delay(1000);


	if (VL53L1X_readReg16Bit(Dev, VL53L1_IDENTIFICATION__MODEL_ID) != 0xEACC)
		return VL53L1X_ERROR;

	uint8_t i = 0;
	while((VL53L1X_readReg(Dev, VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x01) == 0 && i < 50)
	{
		HAL_Delay(10);
		i++;
	}
	if((VL53L1X_readReg(Dev, VL53L1_FIRMWARE__SYSTEM_STATUS) & 0x01) == 0)
		return VL53L1X_TIMEOUT;

	Dev->fast_osc_frequency = VL53L1X_readReg16Bit(Dev, VL53L1_OSC_MEASURED__FAST_OSC__FREQUENCY);
	Dev->osc_calibrate_val = VL53L1X_readReg16Bit(Dev, VL53L1_RESULT__OSC_CALIBRATE_VAL);

	VL53L1X_writeReg16Bit(Dev, VL53L1_DSS_CONFIG__TARGET_TOTAL_RATE_MCPS, VL53L1X_TARGET_RATE);
	VL53L1X_writeReg(Dev, VL53L1_GPIO__TIO_HV_STATUS, 0x02);
	VL53L1X_writeReg(Dev, VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_PULSE_WIDTH_NS, 8);
	VL53L1X_writeReg(Dev, VL53L1_SIGMA_ESTIMATOR__EFFECTIVE_AMBIENT_WIDTH_NS, 16);
	VL53L1X_writeReg(Dev, VL53L1_ALGO__CROSSTALK_COMPENSATION_VALID_HEIGHT_MM, 0x01);
	VL53L1X_writeReg(Dev, VL53L1_ALGO__RANGE_IGNORE_VALID_HEIGHT_MM, 0xFF);
	VL53L1X_writeReg(Dev, VL53L1_ALGO__RANGE_MIN_CLIP, 0);
	VL53L1X_writeReg(Dev, VL53L1_ALGO__CONSISTENCY_CHECK__TOLERANCE, 2);
	VL53L1X_writeReg16Bit(Dev, VL53L1_SYSTEM__THRESH_RATE_HIGH, 0x0000);
	VL53L1X_writeReg16Bit(Dev, VL53L1_SYSTEM__THRESH_RATE_LOW, 0x0000);
	VL53L1X_writeReg(Dev, VL53L1_DSS_CONFIG__APERTURE_ATTENUATION, 0x38);
	VL53L1X_writeReg16Bit(Dev, VL53L1_RANGE_CONFIG__SIGMA_THRESH, 360);
	VL53L1X_writeReg16Bit(Dev, VL53L1_RANGE_CONFIG__MIN_COUNT_RATE_RTN_LIMIT_MCPS, 192);
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_0, 0x01);
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD_1, 0x01);
	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__QUANTIFIER, 2);
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__GROUPED_PARAMETER_HOLD, 0x00);
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__SEED_CONFIG, 1);
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__SEQUENCE_CONFIG, 0x8B);
	VL53L1X_writeReg16Bit(Dev, VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 200 << 8);
	VL53L1X_writeReg(Dev, VL53L1_DSS_CONFIG__ROI_MODE_CONTROL, 2);

	VL53L1X_setDistanceMode(Dev, Short);
	VL53L1X_setMeasurementTimingBudget(Dev, 50000);
	VL53L1X_writeReg16Bit(Dev, VL53L1_ALGO__PART_TO_PART_RANGE_OFFSET_MM,
			VL53L1X_readReg16Bit(Dev, VL53L1_MM_CONFIG__OUTER_OFFSET_MM) * 4);

	return VL53L1X_OK;
}

void VL53L1X_setAddress(VL53L1X_DEV Dev, uint8_t new_addr)
{
	VL53L1X_writeReg(Dev, VL53L1_I2C_SLAVE__DEVICE_ADDRESS, new_addr & 0x7F);
	Dev->I2cDevAddr = new_addr<<1;
}

VL53L1X_Status VL53L1X_setDistanceMode(VL53L1X_DEV Dev, DistanceMode mode)
{
	uint32_t budget_us = VL53L1X_getMeasurementTimingBudget(Dev);

	switch (mode)
	  {
	    case Short:
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x07);
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x05);
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x38);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__WOI_SD0, 0x07);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__WOI_SD1, 0x05);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 6);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 6);

	    	break;

	    case Medium:
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0B);
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x09);
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0x78);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__WOI_SD0, 0x0B);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__WOI_SD1, 0x09);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 10);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 10);

	    	break;

	    case Long:
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A, 0x0F);
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B, 0x0D);
	    	VL53L1X_writeReg(Dev, VL53L1_RANGE_CONFIG__VALID_PHASE_HIGH, 0xB8);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__WOI_SD0, 0x0F);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__WOI_SD1, 0x0D);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__INITIAL_PHASE_SD0, 14);
	    	VL53L1X_writeReg(Dev, VL53L1_SD_CONFIG__INITIAL_PHASE_SD1, 14);

	    	break;

	    default:
	      return VL53L1X_ERROR;
	  }
	VL53L1X_setMeasurementTimingBudget(Dev, budget_us);

	Dev->distance_mode = mode;


	return VL53L1X_OK;
}

uint32_t VL53L1X_calcMacroPeriod(VL53L1X_DEV Dev, uint8_t vcsel_period)
{
	uint32_t pll_period_us = ((uint32_t)0x01 << 30) / Dev->fast_osc_frequency;
	uint8_t vcsel_period_pclks = (vcsel_period + 1) << 1;
	uint32_t macro_period_us = (uint32_t)2304 * pll_period_us;
	macro_period_us >>= 6;
	macro_period_us *= vcsel_period_pclks;
	macro_period_us >>= 6;

	return macro_period_us;
}

uint32_t VL53L1X_getMeasurementTimingBudget(VL53L1X_DEV Dev)
{
	uint32_t macro_period_us = VL53L1X_calcMacroPeriod(Dev, VL53L1X_readReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));

	uint32_t range_config_timeout_us = VL53L1X_timeoutMclksToMicroseconds(VL53L1X_decodeTimeout(
	  VL53L1X_readReg16Bit(Dev, VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI)), macro_period_us);

	return  2 * range_config_timeout_us + VL53L1X_TIMING_GUARD;
}


VL53L1X_Status VL53L1X_setMeasurementTimingBudget(VL53L1X_DEV Dev, uint32_t budget_us)
{
	if(budget_us < VL53L1X_TIMING_GUARD)
		return VL53L1X_ERROR;

	uint32_t range_config_timeout_us = budget_us -= VL53L1X_TIMING_GUARD;
	if (range_config_timeout_us > 1100000)
		return VL53L1X_ERROR;

	range_config_timeout_us /= 2;
	uint32_t macro_period_us;

	macro_period_us = VL53L1X_calcMacroPeriod(Dev, VL53L1X_readReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_A));
	uint32_t phasecal_timeout_mclks = VL53L1X_timeoutMicrosecondsToMclks(1000, macro_period_us);
	if (phasecal_timeout_mclks > 0xFF)
		phasecal_timeout_mclks = 0xFF;

	VL53L1X_writeReg(Dev, VL53L1_PHASECAL_CONFIG__TIMEOUT_MACROP, phasecal_timeout_mclks);
	VL53L1X_writeReg16Bit(Dev, VL53L1_MM_CONFIG__TIMEOUT_MACROP_A_HI, VL53L1X_encodeTimeout(
			VL53L1X_timeoutMicrosecondsToMclks(1, macro_period_us)));
	VL53L1X_writeReg16Bit(Dev, VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_A_HI, VL53L1X_encodeTimeout(
	    VL53L1X_timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));

	macro_period_us = VL53L1X_calcMacroPeriod(Dev, VL53L1X_readReg(Dev, VL53L1_RANGE_CONFIG__VCSEL_PERIOD_B));

	VL53L1X_writeReg16Bit(Dev, VL53L1_MM_CONFIG__TIMEOUT_MACROP_B_HI, VL53L1X_encodeTimeout(
	    VL53L1X_timeoutMicrosecondsToMclks(1, macro_period_us)));

	VL53L1X_writeReg16Bit(Dev, VL53L1_RANGE_CONFIG__TIMEOUT_MACROP_B_HI, VL53L1X_encodeTimeout(
	    VL53L1X_timeoutMicrosecondsToMclks(range_config_timeout_us, macro_period_us)));


	return VL53L1X_OK;
}

uint32_t VL53L1X_timeoutMclksToMicroseconds(uint32_t timeout_mclks, uint32_t macro_period_us)
{
	  return ((uint64_t)timeout_mclks * macro_period_us + 0x800) >> 12;
}
uint32_t VL53L1X_timeoutMicrosecondsToMclks(uint32_t timeout_us, uint32_t macro_period_us)
{
	  return (((uint32_t)timeout_us << 12) + (macro_period_us >> 1)) / macro_period_us;
}

uint32_t VL53L1X_decodeTimeout(uint16_t reg_val)
{
	  return ((uint32_t)(reg_val & 0xFF) << (reg_val >> 8)) + 1;

}

uint16_t VL53L1X_encodeTimeout(uint32_t timeout_mclks)
{
	 uint32_t ls_byte = 0;
	 uint16_t ms_byte = 0;

	 if (timeout_mclks > 0)
	 {
		 ls_byte = timeout_mclks - 1;

		 while ((ls_byte & 0xFFFFFF00) > 0)
		 {
			 ls_byte >>= 1;
			 ms_byte++;
		 }

		 return (ms_byte << 8) | (ls_byte & 0xFF);
	 }
	 	 else
	 		 return 0;
}


void VL53L1X_startContinuous(VL53L1X_DEV Dev, uint32_t period_ms)
{
	VL53L1X_writeReg32Bit(Dev, VL53L1_SYSTEM__INTERMEASUREMENT_PERIOD, period_ms * Dev->osc_calibrate_val);
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__MODE_START, 0x40);
}


void VL53L1X_stopContinuous(VL53L1X_DEV Dev)
{
	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__MODE_START, 0x80);
	Dev->calibrated = false;
	if (Dev->saved_vhv_init != 0)
	    VL53L1X_writeReg(Dev, VL53L1_VHV_CONFIG__INIT, Dev->saved_vhv_init);
	if (Dev->saved_vhv_timeout != 0)
	    VL53L1X_writeReg(Dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND, Dev->saved_vhv_timeout);

	VL53L1X_writeReg(Dev, VL53L1_PHASECAL_CONFIG__OVERRIDE, 0x00);
}

VL53L1X_Status VL53L1X_dataReady(VL53L1X_DEV Dev)
{
	if ((VL53L1X_readReg(Dev, VL53L1_GPIO__TIO_HV_STATUS) & 0x01) == 0)
		return VL53L1X_OK;
	return VL53L1X_ERROR;
}

void VL53L1X_updateDSS(VL53L1X_DEV Dev, ResultBuffer* results)
{
	  uint16_t spadCount = results->dss_actual_effective_spads_sd0;

	  if (spadCount != 0)
	  {
	    uint32_t totalRatePerSpad =
	      (uint32_t)results->peak_signal_count_rate_crosstalk_corrected_mcps_sd0 +results->ambient_count_rate_mcps_sd0;

	    if (totalRatePerSpad > 0xFFFF)
	    	totalRatePerSpad = 0xFFFF;

	    totalRatePerSpad <<= 16;

	    totalRatePerSpad /= spadCount;

	    if (totalRatePerSpad != 0)
	    {
	      uint32_t requiredSpads = ((uint32_t)VL53L1X_TARGET_RATE << 16) / totalRatePerSpad;

	      if (requiredSpads > 0xFFFF)
	    	  requiredSpads = 0xFFFF;

	      VL53L1X_writeReg16Bit(Dev, VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, requiredSpads);

	      return;
	    }
	  }

	   VL53L1X_writeReg16Bit(Dev, VL53L1_DSS_CONFIG__MANUAL_EFFECTIVE_SPADS_SELECT, 0x8000);
}

uint16_t VL53L1X_read(VL53L1X_DEV Dev, RangingData* data, ResultBuffer* results)
{
	while(VL53L1X_dataReady(Dev) != VL53L1X_OK);

	VL53L1X_readResults(Dev, results);

	if(!Dev->calibrated)
	{
		VL53L1X_setupManualCalibration(Dev);
		Dev->calibrated = true;
	}
	VL53L1X_updateDSS(Dev, results);
	VL53L1X_getRangingData(data, results);

	VL53L1X_writeReg(Dev, VL53L1_SYSTEM__INTERRUPT_CLEAR, 0x01);

	return data->range_mm;
}

void VL53L1X_setupManualCalibration(VL53L1X_DEV Dev)
{
	 Dev->saved_vhv_init = VL53L1X_readReg(Dev, VL53L1_VHV_CONFIG__INIT);
	 Dev->saved_vhv_timeout = VL53L1X_readReg(Dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND);
	 VL53L1X_writeReg(Dev, VL53L1_VHV_CONFIG__INIT, Dev->saved_vhv_init & 0x7F);
	 VL53L1X_writeReg(Dev, VL53L1_VHV_CONFIG__TIMEOUT_MACROP_LOOP_BOUND,(Dev->saved_vhv_timeout & 0x03) + (3 << 2));
	 VL53L1X_writeReg(Dev, VL53L1_PHASECAL_CONFIG__OVERRIDE, 0x01);
	 VL53L1X_writeReg(Dev, VL53L1_CAL_CONFIG__VCSEL_START, VL53L1X_readReg(Dev, VL53L1_PHASECAL_RESULT__VCSEL_START));
}

void VL53L1X_readResults(VL53L1X_DEV Dev, ResultBuffer* results)
{
	results->range_status = VL53L1X_readReg(Dev, VL53L1_RESULT__RANGE_STATUS);
	results->final_crosstalk_corrected_range_mm_sd0 = VL53L1X_readReg16Bit(Dev, VL53L1_RESULT__FINAL_CROSSTALK_CORRECTED_RANGE_MM_SD0);
	results->ambient_count_rate_mcps_sd0 = VL53L1X_readReg16Bit(Dev, VL53L1_RESULT__AMBIENT_COUNT_RATE_MCPS_SD0);
	results->dss_actual_effective_spads_sd0 = VL53L1X_readReg16Bit(Dev, VL53L1_RESULT__DSS_ACTUAL_EFFECTIVE_SPADS_SD0);
	results->peak_signal_count_rate_crosstalk_corrected_mcps_sd0 =
			VL53L1X_readReg16Bit(Dev, VL53L1_RESULT__PEAK_SIGNAL_COUNT_RATE_CROSSTALK_CORRECTED_MCPS_SD0);
}

void VL53L1X_getRangingData(RangingData* data, ResultBuffer* results)
{
	uint16_t range = results->final_crosstalk_corrected_range_mm_sd0;
	data->range_mm = ((uint32_t)range * 2011 + 0x0400) / 0x0800;

	 switch(results->range_status)
	  {
	    case 17:
	    case 2:
	    case 1:
	    case 3:
	      data->range_status = HardwareFail;
	      break;

	    case 13:
	      data->range_status = MinRangeFail;
	      break;

	    case 18:
	      data->range_status = SynchronizationInt;
	      break;

	    case 5:
	      data->range_status =  OutOfBoundsFail;
	      break;

	    case 4:
	      data->range_status = SignalFail;
	      break;

	    case 6:
	      data->range_status = SignalFail;
	      break;

	    case 7:
	      data->range_status = WrapTargetFail;
	      break;

	    case 12:
	      data->range_status = XtalkSignalFail;
	      break;

	    case 8:
	      data->range_status = RangeValidMinRangeClipped;
	      break;

	    case 9:
	        data->range_status = RangeValid;
	      break;

	    default:
	      data->range_status = None;
	  }
	  data->peak_signal_count_rate_MCPS = VL53L1X_countRateFixedToFloat(results->peak_signal_count_rate_crosstalk_corrected_mcps_sd0);
	  data->ambient_count_rate_MCPS = VL53L1X_countRateFixedToFloat(results->ambient_count_rate_mcps_sd0);
}


float VL53L1X_countRateFixedToFloat(uint16_t count_rate_fixed)
{
	return (float)count_rate_fixed / (1 << 7);
}




