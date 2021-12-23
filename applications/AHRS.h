#ifndef AHRS_H
#define AHRS_H

#include "drv_mpu6050.h"
#include "stdbool.h"
#include "math.h"

void initAHRSThread();

typedef struct 
{
    float pitch;
    float roll;
} AHRS_t;


#endif //
