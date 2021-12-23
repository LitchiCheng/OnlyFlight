#ifndef PPM_H
#define PPM_H
#include "stm32f1xx_hal.h"
#include <rthw.h>
#include <rtdevice.h>

void initPPMThread();

typedef struct {
	uint8_t channel1;
	uint8_t channel2;
	uint8_t channel3;
	uint8_t channel4;
}remote_t;

enum{
	FRONT_RIGHT,
	FRONT_LEFT,
	REAR_LEFT,
	REAR_RIGHT
};

#endif 
