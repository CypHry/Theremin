/*
 * gen_sinewave.h
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */
//
#ifndef GEN_SINEWAVE_H_
#define GEN_SINEWAVE_H_



#include <math.h>
#include "stm32l4xx.h"
#include <stdint.h>
#include "arm_math.h"

#define DAC_OUTPUT_RATE 420000	// The update rate of the DAC in Hz
#define DAC_SAMPLES_SIZE 10000	// The size of the DAC output buffer (Mult of 10)

typedef struct {
	uint16_t* dacData;
	uint8_t bin;
	uint16_t freq;
	uint8_t	amp;
	uint16_t offset;
}sineInfo;

void Fill_DAC_Half_Buffer(sineInfo* appState);

#endif /* GEN_SINEWAVE_H_ */
