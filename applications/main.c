#include <rthw.h>
#include <rtthread.h>
#include "math.h"

#include "AHRS.h"
#include "PPM.h"
#include "pwm.h"
#include "stablization.h"

 
int main(void)
{ 
    initAHRSThread();
	//initPPMThread();
	initStablization();
    initPwmThread();
	
    while (1){
        rt_thread_mdelay(10);
    }
    return RT_EOK;
}

