#include "AHRS.h"

void mpu6050_thread_entry(void *parameter)
{
    rt_int16_t temp;        //??
    rt_int16_t gx,gy,gz;    //?????
    rt_int16_t ax,ay,az;    //?????
    
    rt_err_t ret;
    double angle = 0;
		rt_tick_t last_tick = rt_tick_get();
		bool first_in = true;
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
        }else{
					
					if(first_in){
							first_in = false;
							last_tick = rt_tick_get();
					}
				} 
        if (ret == RT_EOK)
        {
            //rt_kprintf("mpu6050: temperature=%-6d gx=%-6d gy=%-6d gz=%-6d ax=%-6d ay=%-6d az=%-6d\r\n",temp/100,gx,gy,gz,ax,ay,az);
        }
				
				double gy_f = gy;
				if(fabs(gy_f) < 30){
						gy_f = 0;
				}
				angle += (gy_f / 16.4) * (rt_tick_get()-last_tick / RT_TICK_PER_SECOND);
				last_tick = rt_tick_get();
				
				rt_kprintf("pitch is %f angleX is %f\r\n", angle, atan2((ax / 16384.0),(az / 16384.0))*180/3.14);
        rt_thread_delay(rt_tick_from_millisecond(5));    
    }
}

void initAHRSThread(){
    rt_thread_t tid;
    
    tid = rt_thread_create("mpu6050", 
                            mpu6050_thread_entry,
                            RT_NULL,
                            1024,
                            24,
                            20);
    if (tid == RT_NULL)
    {
        return;
    }
    
    rt_thread_startup(tid);
}