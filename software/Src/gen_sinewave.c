/*
 * gen_sinewave.c
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */

#include "gen_sinewave.h"

#define RANGE_MAX(R, L, H) R<L ? L : R<H ? R : H

int16_t lookup[2*LOOKUP_MAXSIZE]={0};
int16_t prepare[2*LOOKUP_MAXSIZE]={0};

void SineWave_init(SineWaveHandler hsin)
{
	hsin->amp = 1.0;
	hsin->freq = 1000; //range
}

void SineWave_generate(SineWaveHandler hsin, RangingData *dataFreq, RangingData *dataAmp)
{
	hsin->freq = RANGE_MAX(dataFreq->range_mm, 40, 400);
	float32_t temp = dataAmp->range_mm*0.001;
	hsin->amp = RANGE_MAX(temp, 0.1, 0.4);
	hsin->amp *=2;
	hsin->sampleNum = hsin->freq;
	float32_t step = 2.0*PI/hsin->sampleNum;
	hsin->sampleNum = 2*hsin->sampleNum;
	float32_t pos = 0;
	float32_t sample;
	for(uint32_t i=0; i<hsin->sampleNum; i+=2)
	{
		sample = hsin->amp*((arm_sin_f32(pos))*(INT16_MAX));
		prepare[i]= (uint16_t)sample;
		prepare[i+1] = prepare[i];
		pos+=step;
	}
}

