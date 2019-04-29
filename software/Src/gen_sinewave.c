/*
 * gen_sinewave.c
 *
 *  Created on: Apr 12, 2019
 *      Author: bum
 */

#include "gen_sinewave.h"



uint16_t lookup[LOOKUP_SIZE]={0};
//{0x800,0x832,0x864,0x897,0x8c9,0x8fb,0x92d,0x95f,
//0x990,0x9c2,0x9f3,0xa24,0xa54,0xa84,0xab4,0xae3,
//0xb12,0xb40,0xb6e,0xb9b,0xbc8,0xbf4,0xc20,0xc4b,
//0xc75,0xc9e,0xcc7,0xcef,0xd17,0xd3d,0xd63,0xd88,
//0xdac,0xdcf,0xdf1,0xe12,0xe33,0xe52,0xe71,0xe8e,
//0xeaa,0xec6,0xee0,0xef9,0xf11,0xf28,0xf3e,0xf53,
//0xf67,0xf79,0xf8b,0xf9b,0xfaa,0xfb8,0xfc4,0xfd0,
//0xfda,0xfe3,0xfea,0xff1,0xff6,0xffa,0xffd,0xfff,
//0xfff,0xffe,0xffc,0xff8,0xff4,0xfee,0xfe7,0xfde,
//0xfd5,0xfca,0xfbe,0xfb1,0xfa2,0xf93,0xf82,0xf70,
//0xf5d,0xf49,0xf33,0xf1d,0xf05,0xeed,0xed3,0xeb8,
//0xe9c,0xe7f,0xe61,0xe43,0xe23,0xe02,0xde0,0xdbd,
//0xd9a,0xd75,0xd50,0xd2a,0xd03,0xcdb,0xcb3,0xc8a,
//0xc60,0xc35,0xc0a,0xbde,0xbb2,0xb85,0xb57,0xb29,
//0xafb,0xacc,0xa9c,0xa6c,0xa3c,0xa0b,0x9da,0x9a9,
//0x978,0x946,0x914,0x8e2,0x8b0,0x87e,0x84b,0x819,
//0x7e6,0x7b4,0x781,0x74f,0x71d,0x6eb,0x6b9,0x687,
//0x656,0x625,0x5f4,0x5c3,0x593,0x563,0x533,0x504,
//0x4d6,0x4a8,0x47a,0x44d,0x421,0x3f5,0x3ca,0x39f,
//0x375,0x34c,0x324,0x2fc,0x2d5,0x2af,0x28a,0x265,
//0x242,0x21f,0x1fd,0x1dc,0x1bc,0x19e,0x180,0x163,
//0x147,0x12c,0x112,0xfa,0xe2,0xcc,0xb6,0xa2,
//0x8f,0x7d,0x6c,0x5d,0x4e,0x41,0x35,0x2a,
//0x21,0x18,0x11,0xb,0x7,0x3,0x1,0x0,
//0x0,0x2,0x5,0x9,0xe,0x15,0x1c,0x25,
//0x2f,0x3b,0x47,0x55,0x64,0x74,0x86,0x98,
//0xac,0xc1,0xd7,0xee,0x106,0x11f,0x139,0x155,
//0x171,0x18e,0x1ad,0x1cc,0x1ed,0x20e,0x230,0x253,
//0x277,0x29c,0x2c2,0x2e8,0x310,0x338,0x361,0x38a,
//0x3b4,0x3df,0x40b,0x437,0x464,0x491,0x4bf,0x4ed,
//0x51c,0x54b,0x57b,0x5ab,0x5db,0x60c,0x63d,0x66f,
//0x6a0,0x6d2,0x704,0x736,0x768,0x79b,0x7cd,0x800};

void SineWave_init(SineWaveHandler hsin)
{
	hsin->amp = 1.0;
	hsin->freq = 10000;
}

void SineWave_generate(SineWaveHandler hsin)
{
//	memcpy(hsin->data, lookup, LOOKUP_SIZE);

	float32_t step = 2*PI/LOOKUP_SIZE;
	float32_t pos = 0;
	float32_t sample;
	for(int i=0; i<LOOKUP_SIZE; i++)
	{
		sample = hsin->amp*((arm_sin_f32(pos)+1)*(MAX12BIT_2));
		lookup[i]= (uint16_t)sample;
		pos+=step;
	}
	//hsin->data = lookup;
}

void SineWave_adjustFreq(SineWaveHandler hsin,  TIM_HandleTypeDef *htim)
{
	if(hsin->freq > 20000)
	{
		hsin->freq = 20000;
	}

	if(hsin->freq < 50)
	{
		hsin->freq = 50;
	}

	uint32_t tim_period = TIM_CLOCK/(hsin->freq*(TIM_PRESCALER+1)*LOOKUP_SIZE);

	__HAL_TIM_SET_AUTORELOAD(htim, tim_period);

	uint32_t autoreload = __HAL_TIM_GET_AUTORELOAD(htim);
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



