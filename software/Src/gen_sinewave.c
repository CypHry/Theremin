/*
 * gen_sinewave.c
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */

#include "gen_sinewave.h"



SineWave_init(SineWaveHandler hsin)
{
	hsin->data=sinewave;
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



