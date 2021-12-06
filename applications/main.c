#include <rthw.h>
#include <rtthread.h>
#include "math.h"

#include "AHRS.h"
 
int main(void)
{ 
    initAHRSThread();
    while (1){
				rt_thread_mdelay(10);
    }
    return RT_EOK;
}