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

uint8_t sinewave[] = {128, 176, 217, 234, 255, 245, 217, 176, 128, 79, 38, 10, 1, 10, 38, 79};

typedef struct
{
	uint16_t amp;
	uint16_t freq;
	uint8_t* data;
}SineWave;

typedef SineWave *SineWaveHandler;

//SineWave sin;
//SineWaveHandler hsin = &sin;
