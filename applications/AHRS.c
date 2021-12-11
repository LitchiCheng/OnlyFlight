#include "AHRS.h"

#define OFFSET_CNT  400
#define STILL_CNT   400

#define MOVING_FILT_CNT 50

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

double movingAverageFilter(rt_int16_t* data_set, int size, rt_int16_t new_data){
    double sum = 0;
    for(int i=0; i < size - 1; i++){
        sum+=data_set[i+1];
        data_set[i] = data_set[i+1];
    }
    data_set[size-1] = new_data;
    double mean = (sum+new_data) / size;
    return mean;
}
								
double complementaryFilter1d(double a_pitch, double g_pitch, double alpha){
	return (a_pitch * alpha + g_pitch * (1-alpha));
}

float kalman_filter(float angle_m,float gyro_m,float kalman_filter_angle, float kalman_filter_angle_dot, float dt)
{

    //滤波参数
    // float  dt = 0.005;   //卡尔曼采样时间
    float  P[2][2]    = {{1,0},{0,1}};
    float  Pdot[4]    = {0,0,0,0};
    float  Q_angle = 0.001;//角度数据置信度,陀螺仪协方差
    float  Q_gyro = 0.005;     //角速度数据置信度，陀螺仪飘移噪声协方差
    float  R_angle = 0.5;    //加速度计协方差
    char     C_0 = 1;
    float  q_bias = 0,angle_err = 0; //q_bias为陀螺仪飘移
    float  PCt_0 = 0,PCt_1 = 0,E = 0;
    float  K_0 = 0,  K_1 = 0,  t_0 = 0,  t_1 = 0;

    kalman_filter_angle += (gyro_m - q_bias) * dt;    //卡尔曼预测方程，认为每次飘移相同，

    Pdot[0] = Q_angle - P[0][1] - P[1][0];
    Pdot[1] =- P[1][1];
    Pdot[2] =- P[1][1];
    Pdot[3] = Q_gyro;

    P[0][0] += Pdot[0] * dt;
    P[0][1] += Pdot[1] * dt;
    P[1][0] += Pdot[2] * dt;
    P[1][1] += Pdot[3] * dt;

    PCt_0 = C_0 * P[0][0];     //矩阵乘法中间变量
    PCt_1 = C_0 * P[1][0];

    E = R_angle + C_0 * PCt_0;     //分母

    K_0 = PCt_0 / E;   //增益值
    K_1 = PCt_1 / E;

    angle_err = angle_m - kalman_filter_angle;    
    kalman_filter_angle += K_0 * angle_err; //对状态的卡尔曼估计，最优角度
    q_bias += K_1 * angle_err;
    kalman_filter_angle_dot = gyro_m-q_bias;//最优角速度

    t_0 = PCt_0;     //矩阵计算中间变量
    t_1 = C_0 * P[0][1];

    P[0][0] -= K_0 * t_0;
    P[0][1] -= K_0 * t_1;
    P[1][0] -= K_1 * t_0;
    P[1][1] -= K_1 * t_1;

    return kalman_filter_angle;
}

void mpu6050_thread_entry(void *parameter)
{
    rt_int16_t temp;
    rt_int16_t gx,gy,gz;
    rt_int16_t ax,ay,az;
    
    rt_int16_t gx_filter[MOVING_FILT_CNT];
    rt_int16_t gy_filter[MOVING_FILT_CNT];
    rt_int16_t gz_filter[MOVING_FILT_CNT];
    rt_int16_t ax_filter[MOVING_FILT_CNT];
    rt_int16_t ay_filter[MOVING_FILT_CNT];
    rt_int16_t az_filter[MOVING_FILT_CNT];
    for(int i=0; i<MOVING_FILT_CNT; i++){
        gx_filter[i] = 0;
        gy_filter[i] = 0;
        gz_filter[i] = 0;
        ax_filter[i] = 0;
        ay_filter[i] = 0;
        az_filter[i] = 0;
    }

    calibOffset(&gx_off,&gy_off,&gz_off,&ax_off,&ay_off,&az_off);
    calibStillMax(&gx_max,&gy_max,&gz_max,&ax_max,&ay_max,&az_max);
    
    rt_err_t ret;
    double gyro_pitch = 0;
    double gyro_roll = 0;
    float gyro_pitch_filter = 0;
    float gyro_pitch_dot_filter = 0; 
    float gyro_roll_filter = 0;
    float gyro_roll_dot_filter = 0; 
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
        // ax = ax - ax_off;
        // ay = ay - ay_off;
        // az = az - az_off;
        gx = movingAverageFilter(gx_filter, MOVING_FILT_CNT, gx);
        gy = movingAverageFilter(gy_filter, MOVING_FILT_CNT, gy);
        gz = movingAverageFilter(gz_filter, MOVING_FILT_CNT, gz);
        ax = movingAverageFilter(ax_filter, MOVING_FILT_CNT, ax);
        ay = movingAverageFilter(ay_filter, MOVING_FILT_CNT, ay);
        az = movingAverageFilter(az_filter, MOVING_FILT_CNT, az);
		
//        gx = (fabs(gx) < gx_max) ? 0 : gx;
//        gy = (fabs(gy) < gy_max) ? 0 : gy;
//        gz = (fabs(gz) < gz_max) ? 0 : gz;

        double time_elapse = (double)(rt_tick_get()-last_tick) / RT_TICK_PER_SECOND;
        gyro_pitch += (gy / 16.4) * time_elapse;
        gyro_roll += (gx / 16.4) * time_elapse;
        last_tick = rt_tick_get();
				
        double acc_pitch = RAD2DEG(atan2((ax / 16384.0),(az / 16384.0)));
        double acc_roll = RAD2DEG(atan2((ay / 16384.0),(az / 16384.0)));
//        double kalman_pitch = kalman_filter(acc_pitch, -gyro_pitch, gyro_pitch_filter, gyro_pitch_dot_filter, time_elapse);
//        double kalman_roll = kalman_filter(acc_roll, -gyro_roll, gyro_roll_filter, gyro_roll_dot_filter, time_elapse);
				static double c_pitch_filter = 0;
				static double c_roll_filter = 0;
				c_pitch_filter = complementaryFilter1d(acc_pitch, c_pitch_filter, 0.80);
				c_roll_filter = complementaryFilter1d(acc_roll, c_roll_filter, 0.80);
        rt_kprintf("%f|%f|%f|%f|%f|%f\r\n",-gyro_pitch, acc_pitch, c_pitch_filter,-gyro_roll, acc_roll, c_roll_filter);
        rt_thread_delay(rt_tick_from_millisecond(5));    
    }
}

void initAHRSThread(){
    rt_thread_t tid;
    
    tid = rt_thread_create("mpu6050", 
                            mpu6050_thread_entry,
                            RT_NULL,
                            2048,
                            24,
                            20);
    if (tid == RT_NULL)
    {
				rt_kprintf("AHRS thread create failed!\r\n");
        return;
    }
    rt_thread_startup(tid);
}