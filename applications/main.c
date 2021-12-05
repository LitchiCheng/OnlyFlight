#include <rthw.h>
#include <rtthread.h>
#include "math.h"

#include "AHRS.h"
 
int main(void)
{ 
    initAHRSThread();

    while (1){
        //led
    }
    return RT_EOK;
}