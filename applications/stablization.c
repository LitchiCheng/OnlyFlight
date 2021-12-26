#include "pid.h"
#include "stablization.h"
#include "AHRS.h"
#include "pwm.h"

extern rt_mq_t ahrs_mq_t;
rt_mq_t af_mq_t = RT_NULL;

void statble_thread_entry(void *parameter)
{
    pid_t pid_set[4];
    pid_set[FRONT_RIGHT] = *(getPid(1000,0,0,PWM_PERIOD,20000));
    pid_set[FRONT_LEFT] = *(getPid(1000,0,0,PWM_PERIOD,20000));
    pid_set[REAR_RIGHT] = *(getPid(1000,0,0,PWM_PERIOD,20000));
    pid_set[REAR_LEFT] = *(getPid(1000,0,0,PWM_PERIOD,20000));
    Throttle_t tt;
    AHRS_t pAhrs;
    while(1)
    {
		if(ahrs_mq_t != RT_NULL){
			rt_mq_recv(ahrs_mq_t, &pAhrs, sizeof(AHRS_t), RT_WAITING_NO);
			pidProcess(&pAhrs, pid_set, &tt);
		}
		//rt_kprintf("throttle %d|%d|%d|%d|%f|%f\r\n", af[0].throttle,af[1].throttle,af[2].throttle,af[3].throttle, pAhrs.pitch, pAhrs.roll);
		//rt_kprintf("throttle %f|%f\r\n", pAhrs.pitch, pAhrs.roll);
		rt_mq_send(af_mq_t, &tt, sizeof(tt));
        rt_thread_delay(rt_tick_from_millisecond(2));    
    }
}

void initStablization(){
    rt_thread_t tid;
    af_mq_t = rt_mq_create("af_mq", sizeof(Throttle_t),2*sizeof(Throttle_t), RT_IPC_FLAG_FIFO);
	if(af_mq_t == RT_NULL){
		rt_kprintf("af_mq_t create failed!\r\n");
	}
    tid = rt_thread_create("stablization", 
                            statble_thread_entry,
                            RT_NULL,
                            1024,
                            24,
                            20);
    if (tid == RT_NULL)
    {
				rt_kprintf("stable thread create failed!\r\n");
        return;
    }
    rt_thread_startup(tid);
}