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
#include "VL53L1X.h"
#include "main.h"


#define AUDIO_FREQ 8000
#define LOOKUP_MAXSIZE 8000
#define MAX12BIT 4095
#define MAX12BIT_2 (MAX12BIT>>1)
#define MAX8BIT 255
#define MAX8BIT_2 (MAX8BIT>>1)
#define MAX16BIT  65535
#define MAX16BIT_2 (MAX16BIT>>1)


extern int16_t lookup[2*LOOKUP_MAXSIZE];
extern int16_t prepare[2*LOOKUP_MAXSIZE];
extern int16_t sampleShow;

typedef struct
{
	float32_t amp;
	uint16_t freq;
	uint16_t* data;
	uint32_t sampleNum;
}SineWave;

typedef SineWave *SineWaveHandler;

//SineWave sin;
//SineWaveHandler hsin = &sin;

void SineWave_init(SineWaveHandler hsin);
void SineWave_generate(SineWaveHandler hsin, RangingData *dataFreq, RangingData *dataAmp);
//void SineWave_adjustFreq(SineWaveHandler hsin,  TIM_HandleTypeDef *htim);


#endif
