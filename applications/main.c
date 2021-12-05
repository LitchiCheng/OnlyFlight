#include <rthw.h>
#include <rtthread.h>
#include "math.h"
#include "drv_mpu6050.h"
 
void mpu6050_thread_entry(void *parameter)
{
    rt_int16_t temp;        //??
    rt_int16_t gx,gy,gz;    //?????
    rt_int16_t ax,ay,az;    //?????
    
    rt_err_t ret;
    double angle = 0;
    while(1)
    {
        ret = mpu6050_temperature_get(&temp);
        if (ret != RT_EOK)
        {
            rt_kprintf("mpu6050 : get temperature error\r\n");
        }            
        ret = mpu6050_accelerometer_get(&ax, &ay, &az);
        if (ret != RT_EOK)
        {
            rt_kprintf("mpu6050 : get acc error\r\n");
        } 
        ret = mpu6050_gyroscope_get(&gx, &gy, &gz);
        if (ret != RT_EOK)
        {
            rt_kprintf("mpu6050 : get gyro error\r\n");
        } 
        if (ret == RT_EOK)
        {
            //rt_kprintf("mpu6050: temperature=%-6d gx=%-6d gy=%-6d gz=%-6d ax=%-6d ay=%-6d az=%-6d\r\n",temp/100,gx,gy,gz,ax,ay,az);
        }
				double gz_f = gz;
				if(fabs(gz_f) < 30){
						gz_f = 0;
				}
				angle += (gz_f / 16) * 0.005;
				uint64_t angele_k = (uint64_t)angle;
				rt_kprintf("angle is %f\r\n", angle);
				//char dd[20];
				//sprintf(dd, "angle is %f\r\n", angle);
        rt_thread_delay(rt_tick_from_millisecond(5));    
    }
}
 
int main(void)
{ 
    rt_thread_t tid;
    
    tid = rt_thread_create("mpu6050", 
                            mpu6050_thread_entry,
                            RT_NULL,
                            1024,
                            24,
                            20);
    if (tid == RT_NULL)
    {
        return -RT_ERROR;
    }
    
    rt_thread_startup(tid);
    
    return RT_EOK;
}