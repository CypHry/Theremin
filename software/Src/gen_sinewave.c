/*
 * gen_sinewave.c
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */

#include "gen_sinewave.h"



uint16_t lookup[LOOKUP_MAXSIZE]={0};

void SineWave_init(SineWaveHandler hsin)
{
	hsin->amp = 1.0;
	hsin->freq = 1000; //range
}

void SineWave_generate(SineWaveHandler hsin, RangingData *data)
{
//	memcpy(hsin->data, lookup, LOOKUP_SIZE);
//	hsin->amp = 1.0*data->range_mm/1700;
//	hsin->freq = 1.0*data->range_mm;
	hsin->amp = 1;
	hsin->freq = data->range_mm%500;
	hsin->sampleNum = AUDIO_FREQ/hsin->freq;

	float32_t step = 2.0*PI/hsin->sampleNum;
	float32_t pos = 0;
	float32_t sample;
	for(int i=0; i<hsin->sampleNum; i++)
	{
		sample = hsin->amp*((arm_sin_f32(pos)+1)*(UINT16_MAX>>1));
		lookup[i]= (uint16_t)sample;
		//sampleShow = lookup[i];
		pos+=step;
	}
	//hsin->data = lookup;
}

//void SineWave_adjustFreq(SineWaveHandler hsin,  TIM_HandleTypeDef *htim)
//{
//	if(hsin->freq > 20000)
//	{
//		hsin->freq = 20000;
//	}
//
//	if(hsin->freq < 50)
//	{
//		hsin->freq = 50;
//	}
//
//	uint32_t tim_period = TIM_CLOCK/(hsin->freq*(TIM_PRESCALER+1)*LOOKUP_SIZE);
//
//	__HAL_TIM_SET_AUTORELOAD(htim, tim_period);
//
//	uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(htim);
//}

//void adjustAmplitude(int_8 amplitude)
//{
//	if(amplitude > 16)
//	{
//		amplitude = 16;
//	}
//
//	for(uint8_t i=0; i<SAMPLES_NUMBER; ++i)
//	{
//		sine_wave[i]*=amplitude;
//	}
//}



