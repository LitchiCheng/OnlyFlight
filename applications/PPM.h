#ifndef PPM_H
#define PPM_H
#include "stm32f1xx_hal.h"
#include <rthw.h>
#include <rtdevice.h>

void initPPMThread();

struct remote_t {
	uint8_t channel1;
	uint8_t channel2;
	uint8_t channel3;
	uint8_t channel4;
};

#endif 
