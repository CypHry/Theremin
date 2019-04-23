/*
 * gen_sinewave.c
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */

#include "gen_sinewave.h"



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

uint16_t outPos = 0;
uint16_t lastBinFreq = 0;
uint16_t lastFreq = 0;

void Fill_DAC_Half_Buffer(sineInfo* appState) {
	uint16_t *activeSample = appState->dacData+
								((appState->bin-1)*DAC_SAMPLES_SIZE);
	uint32_t i = 0;
	float32_t scaleVal=(appState->freq*M_TWOPI)/DAC_OUTPUT_RATE;

	//float32_t slope = (2*appState->amp)/M_PI;;

	uint16_t offset = 2*appState->offset;

	uint16_t reset = DAC_OUTPUT_RATE/appState->freq;
	float32_t step = DAC_OUTPUT_RATE*1.0f/appState->freq;

		scaleVal *= (step/reset);	// correct for floating point / integer error
		while (i < DAC_SAMPLES_SIZE)
		{
			// calculate the sin of 2*pi*f/dac rate
			*activeSample++ = (uint16_t)(arm_sin_f32(scaleVal*outPos++)*appState->amp+offset);
			// check to see if we're near a multiple of 2*pi
			if ((outPos > 10) && (outPos % reset == 0))
			{
				outPos = 0;
			}

			i++;
		}

}

