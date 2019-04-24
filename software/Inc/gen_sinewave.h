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

extern uint32_t lookup[];

typedef struct
{
	uint16_t amp;
	uint16_t freq;
	uint32_t* data;
}SineWave;

typedef SineWave *SineWaveHandler;

//SineWave sin;
//SineWaveHandler hsin = &sin;

void SineWave_init(SineWaveHandler);


#endif
