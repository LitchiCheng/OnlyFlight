#ifndef PWM_H
#define PWM_H

#include <rtthread.h>
#include <board.h>
#include <rtdevice.h>

#define PWM_PERIOD 500000  //周期的单位ns,0.5ms

int initPwmThread(void);

typedef struct
{
    char* name;
    struct rt_device_pwm *dev;
    uint8_t channel;
    uint32_t throttle;
}AeroFoil;

typedef struct 
{
    uint32_t thr[4];
}Throttle_t;

#define AEROFOIL_1  \
{                   \
    .name = "pwm1", \
    .dev = NULL,    \
    .channel = 1,   \
    .throttle = 0   \
}
#define AEROFOIL_2  \
{                   \
    .name = "pwm1", \
    .dev = NULL,    \
    .channel = 4,   \
    .throttle = 0   \
}
#define AEROFOIL_3  \
{                   \
    .name = "pwm4", \
    .dev = NULL,    \
    .channel = 1,   \
    .throttle = 0   \
}
#define AEROFOIL_4  \
{                   \
    .name = "pwm4", \
    .dev = NULL,    \
    .channel = 2,   \
    .throttle = 0   \
}

#endif //PWM_H