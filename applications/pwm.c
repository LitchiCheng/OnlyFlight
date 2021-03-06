#include "pwm.h"
#include "PPM.h"

rt_mq_t remote_mq_t = RT_NULL;
extern rt_mq_t af_mq_t;

static AeroFoil aerofoil_obj[] = {
    AEROFOIL_1,
    AEROFOIL_2,
    AEROFOIL_3,
    AEROFOIL_4
};

void pwm_entry(void *pra)
{
    rt_uint32_t pwm_pulse = 0;
    while(1){
		Throttle_t tt;
        if(af_mq_t != NULL){
            if (rt_mq_recv(af_mq_t, &tt, sizeof(tt), RT_WAITING_FOREVER) == RT_EOK){
                for (size_t i = 0; i < sizeof(aerofoil_obj) / sizeof(aerofoil_obj[0]); i++){
                    rt_pwm_set(aerofoil_obj[i].dev, aerofoil_obj[i].channel, PWM_PERIOD, tt.thr[i]);
                }
            }
        }     
    }   
}
 
int initPwmThread(void)
{
    rt_thread_t  pwm_thread;
	rt_err_t result;
	remote_mq_t = rt_mq_create("remote_mq", sizeof(remote_t),10*sizeof(remote_t), RT_IPC_FLAG_FIFO);
    
    for (size_t i = 0; i < sizeof(aerofoil_obj) / sizeof(aerofoil_obj[0]); i++){
        //初始化pwm设备
        aerofoil_obj[i].dev = (struct rt_device_pwm *)rt_device_find(aerofoil_obj[i].name);
        if(aerofoil_obj[i].dev == RT_NULL){
            rt_kprintf("cannot find pwmdev %s\n", aerofoil_obj[0].name);
            return RT_ERROR;
        }
        rt_pwm_set(aerofoil_obj[i].dev, aerofoil_obj[i].channel, PWM_PERIOD, 0);//初始默认值
        rt_pwm_enable(aerofoil_obj[i].dev, aerofoil_obj[i].channel);
    }
    
    //创建使用pwm设备的线程
    pwm_thread = rt_thread_create("pwm_thread",pwm_entry,0,512,10,10);
    if(pwm_thread == RT_NULL){
        rt_kprintf("create pwm thread failed\n");
        return RT_ERROR;
    }
    rt_thread_startup(pwm_thread);
    return RT_EOK;
}
//MSH_CMD_EXPORT(pwm_sample,pwm sample);





