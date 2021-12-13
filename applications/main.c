#include <rthw.h>
#include <rtthread.h>
#include "math.h"

#include "AHRS.h"
#include "PPM.h"

#include <rtthread.h>
#include <rtdevice.h>
#include <board.h>
 
#define PWM1_DEVICE_NAME "pwm4"
#define PWM1_DEVICE_CHANNEL 1
 
 
#define PWM1_PERIOD 500000  //周期的单位ns,0.5ms
#define PWM1_STEP   20000   //pwm变化量
static struct rt_device_pwm  *pwm_dev;
 
void pwm_led_entry(void *pra)
{
   static rt_uint32_t pwm_pulse = 0;
   static int  dir = 0;
    
    while(1)
    {
        switch(dir)
        {
            case 0: pwm_pulse += PWM1_STEP;
                pwm_pulse = (pwm_pulse >= PWM1_PERIOD)? PWM1_PERIOD : pwm_pulse;
                if(pwm_pulse == PWM1_PERIOD)dir = 1; 
            
        break;
        
            case 1:pwm_pulse -= PWM1_STEP;
                pwm_pulse = (pwm_pulse <= 0)? 0 : pwm_pulse;
                if(pwm_pulse == 0)dir = 0;  
        break;
        
            default:
                break;
        }
        rt_pwm_set(pwm_dev,PWM1_DEVICE_CHANNEL,PWM1_PERIOD,pwm_pulse);
        rt_thread_mdelay(50);         
    }   
}
 
int pwm_sample(void)
{
    static rt_thread_t  pwm_led_thread;
    
    
    //初始化pwm设备
    pwm_dev = (struct rt_device_pwm *)rt_device_find(PWM1_DEVICE_NAME);
    if(pwm_dev == RT_NULL)
    {
        rt_kprintf("cannot find pwmdev\n");
        return RT_ERROR;
    }
    
    rt_pwm_set(pwm_dev,PWM1_DEVICE_CHANNEL,PWM1_PERIOD,0);//初始默认值
    rt_pwm_enable(pwm_dev,PWM1_DEVICE_CHANNEL);
    
    
	//创建一个使用pwm设备的线程，如点亮led的线程
    pwm_led_thread = rt_thread_create("led_thread",pwm_led_entry,0,1024,10,10);
    
    if(pwm_led_thread == RT_NULL)
    {
        rt_kprintf("create pwm thread failed\n");
        return RT_ERROR;
    }
    
    rt_thread_startup(pwm_led_thread);
    
    return RT_EOK;
 
}
MSH_CMD_EXPORT(pwm_sample,pwm sample);
 
int main(void)
{ 
    //initAHRSThread();
	//initPPMThread();
    while (1){
		rt_thread_mdelay(10);
    }
    return RT_EOK;
}

