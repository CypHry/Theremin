/*
 * gen_sinewave.c
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */

#include "gen_sinewave.h"


uint32_t lookup[] = {128, 176, 217, 234, 255, 245, 217, 176, 128, 79, 38, 10, 1, 10, 38, 79};


void SineWave_init(SineWaveHandler hsin)
{
	hsin->data=lookup;
}

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



