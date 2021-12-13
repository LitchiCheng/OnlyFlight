#include <rthw.h>
#include <rtthread.h>
#include "math.h"

#include "AHRS.h"
#include "PPM.h"
 
int main(void)
{ 
    initAHRSThread();
	initPPMThread();
    while (1){
		rt_thread_mdelay(10);
    }
    return RT_EOK;
}