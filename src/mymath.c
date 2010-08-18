#include <math.h>

#define ADC_BITS 10
#define WORD 16

float intToFloat(unsigned int x);
unsigned int floatToInt(float x);

float intToFloat(unsigned int x) {
	return ((float)(x) / (float)(2^WORD - 1)) * (int)(VRef);
}

unsigned int floatToInt(float x) {
	return (unsigned int)(x * (2^WORD - 1));
}