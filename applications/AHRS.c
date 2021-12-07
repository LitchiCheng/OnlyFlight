#include "AHRS.h"

#define OFFSET_CNT  400
#define STILL_CNT   400

#define PI  3.14159265357
#define max(a,b) (a>b?a:b)
#define RAD2DEG(r)  (r*180/PI)

rt_int16_t gx_off,gy_off,gz_off;
rt_int16_t ax_off,ay_off,az_off;

rt_int16_t gx_max,gy_max,gz_max;
rt_int16_t ax_max,ay_max,az_max;

void calibOffset(rt_int16_t* gx_off, rt_int16_t* gy_off, rt_int16_t* gz_off,
                    rt_int16_t* ax_off, rt_int16_t* ay_off, rt_int16_t* az_off){
    rt_int64_t gx_sum = 0;
    rt_int64_t gy_sum = 0;
    rt_int64_t gz_sum = 0;
    rt_int64_t ax_sum = 0;
    rt_int64_t ay_sum = 0;
    rt_int64_t az_sum = 0;
    for(int i=0; i<OFFSET_CNT; i++){
        rt_int16_t gx,gy,gz;
        rt_int16_t ax,ay,az;
        mpu6050_accelerometer_get(&ax, &ay, &az);
        mpu6050_gyroscope_get(&gx, &gy, &gz);
        gx_sum += gx;
        gy_sum += gy;
        gz_sum += gz;
        ax_sum += ax;
        ay_sum += ay;
        az_sum += az;
        rt_thread_delay(rt_tick_from_millisecond(5));
    }
    *gx_off = gx_sum / OFFSET_CNT;
    *gy_off = gy_sum / OFFSET_CNT;
    *gz_off = gz_sum / OFFSET_CNT;
    *ax_off = ax_sum / OFFSET_CNT;
    *ay_off = ay_sum / OFFSET_CNT;
    *az_off = az_sum / OFFSET_CNT;
    rt_kprintf("%d|%d|%d|%d|%d|%d\r\n", gx_sum,gy_sum,gz_sum,az_sum,ay_sum,az_sum);
    rt_kprintf("%d|%d|%d|%d|%d|%d\r\n", *gx_off,*gy_off,*gz_off,*ax_off,*ay_off,*az_off);
    rt_thread_delay(rt_tick_from_millisecond(1000));
}
										
void calibStillMax(rt_int16_t *gx_max, rt_int16_t *gy_max, rt_int16_t* gz_max,
                    rt_int16_t *ax_max, rt_int16_t* ay_max, rt_int16_t* az_max){
    for(int i=0; i<STILL_CNT; i++){
        rt_int16_t gx,gy,gz;
        rt_int16_t ax,ay,az;
        mpu6050_accelerometer_get(&ax, &ay, &az);
        mpu6050_gyroscope_get(&gx, &gy, &gz);
        *gx_max = max(fabs(gx-gx_off), fabs(*gx_max));
        *gy_max = max(fabs(gy-gy_off), fabs(*gy_max));
        *gz_max = max(fabs(gz-gz_off), fabs(*gz_max));
        *ax_max = max(fabs(ax-ax_off), fabs(*ax_max));
        *ay_max = max(fabs(ay-ay_off), fabs(*ay_max));
        *az_max = max(fabs(az-az_off), fabs(*az_max));
        rt_thread_delay(rt_tick_from_millisecond(5));
    }
    rt_kprintf("%d|%d|%d|%d|%d|%d\r\n", *gx_max,*gy_max,*gz_max,*ax_max,*ay_max,*az_max);
    rt_thread_delay(rt_tick_from_millisecond(1000));
}

void mpu6050_thread_entry(void *parameter)
{
    rt_int16_t temp;
    rt_int16_t gx,gy,gz;
    rt_int16_t ax,ay,az;
    

    calibOffset(&gx_off,&gy_off,&gz_off,&ax_off,&ay_off,&az_off);
    calibStillMax(&gx_max,&gy_max,&gz_max,&ax_max,&ay_max,&az_max);
    
    rt_err_t ret;
    double angle = 0;
    rt_tick_t last_tick = rt_tick_get();
    bool first_in = true;
    while(1)
    {
        ret = mpu6050_temperature_get(&temp);
        if (ret != RT_EOK){
            rt_kprintf("mpu6050 : get temperature error\r\n");
        }            
        ret = mpu6050_accelerometer_get(&ax, &ay, &az);
        if (ret != RT_EOK){
            rt_kprintf("mpu6050 : get acc error\r\n");
        }
        ret = mpu6050_gyroscope_get(&gx, &gy, &gz);
        if (ret != RT_EOK){
            rt_kprintf("mpu6050 : get gyro error\r\n");
        }else{	
            if(first_in){
                first_in = false;
                last_tick = rt_tick_get();
            }
        } 
        if (ret == RT_EOK){
            //rt_kprintf("%f|%f|%f|%f|%f|%f\r\n",gx/16.4,gy/16.4,gz/16.4,ax/16384.0,ay/16384.0,az/16384.0);
            //rt_kprintf("%d|%d|%d|%d|%d|%d\r\n",gx,gy,gz,ax,ay,az);
						//rt_kprintf("%d|%d|%d|%d|%d|%d\r\n",gx-gx_off,gy-gy_off,gz-gz_off,ax-ax_off,ay-ay_off,az-az_off);
        }
				
        gx = gx - gx_off;
				gy = gy - gy_off;
				gz = gz - gz_off;
//				ax = ax - ax_off;
//				ay = ay - ay_off;
//				az = az - az_off;
        if(fabs(gy) < gy_max){
					gy = 0;
        }
				double time_elapse = (double)(rt_tick_get()-last_tick) / RT_TICK_PER_SECOND;
        angle += (gy / 16.4) * time_elapse;
        last_tick = rt_tick_get();
        
        rt_kprintf("pitch is %f angleX is %f angleY is %f\r\n",angle, RAD2DEG(atan2((ax / 16384.0),(az / 16384.0))), RAD2DEG(atan2((ay / 16384.0),(az / 16384.0))));
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
				rt_kprintf("AHRS thread create failed!\r\n");
        return;
    }
    rt_thread_startup(tid);
}