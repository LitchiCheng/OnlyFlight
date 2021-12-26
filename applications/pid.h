#pragma once
#include <math.h>

#include "PPM.h"
#include "pwm.h"
#include "AHRS.h"
#include "rtthread.h"

typedef struct
{
	float P;
	float I;
	float D;
    float err_sum;
    float last_err;
    rt_tick_t last_t;
    float max;
    float min;
    bool first;
} pid_t;

static pid_t* getPid(float P, float I, float D, float max, float min){
    pid_t *pPid = (pid_t *)calloc(1, sizeof(pid_t));
    pPid->P = P;
    pPid->I = I;
    pPid->D = D;
    pPid->max = max;
    pPid->min = min;
    pPid->err_sum = 0;
    pPid->last_err = 0;
    pPid->last_t = rt_tick_get();
    pPid->first = true;
    return pPid;
}

static void delPid(pid_t* p){
    if(p != NULL){
        free(p);
    }
}

static void pidProcess(AHRS_t* pAhrs, pid_t *pPid, Throttle_t *pAf){
	int32_t rl_value = 0;
	int32_t rr_value = 0;
	int32_t fl_value = 0;
	int32_t fr_value = 0;
    if(fabs(pAhrs->pitch) < 90){
        //if(pAhrs->pitch > 0){
            pPid[REAR_LEFT].err_sum += pAhrs->pitch;
            rl_value += (int32_t)(pPid[REAR_LEFT].P * pAhrs->pitch 
                        + pPid[REAR_LEFT].I * pPid[REAR_LEFT].err_sum 
                        + pPid[REAR_LEFT].D * ((pAhrs->pitch-pPid[REAR_LEFT].last_err) / (rt_tick_get()-pPid[REAR_LEFT].last_t)));
            pPid[REAR_LEFT].last_err = pAhrs->pitch;
            pPid[REAR_LEFT].last_t = rt_tick_get();
            

            pPid[REAR_RIGHT].err_sum += pAhrs->pitch;
            rr_value += (int32_t)(pPid[REAR_RIGHT].P * pAhrs->pitch 
                        + pPid[REAR_RIGHT].I * pPid[REAR_RIGHT].err_sum 
                        + pPid[REAR_RIGHT].D * ((pAhrs->pitch-pPid[REAR_RIGHT].last_err) / (rt_tick_get()-pPid[REAR_RIGHT].last_t)));
            pPid[REAR_RIGHT].last_err = pAhrs->pitch;
            pPid[REAR_RIGHT].last_t = rt_tick_get();
        

            pPid[FRONT_LEFT].err_sum += pAhrs->pitch;
            fl_value -= (int32_t)(pPid[FRONT_LEFT].P * pAhrs->pitch 
                        + pPid[FRONT_LEFT].I * pPid[FRONT_LEFT].err_sum 
                        + pPid[FRONT_LEFT].D * ((pAhrs->pitch-pPid[FRONT_LEFT].last_err) / (rt_tick_get()-pPid[FRONT_LEFT].last_t)));
            pPid[FRONT_LEFT].last_err = pAhrs->pitch;
            pPid[FRONT_LEFT].last_t = rt_tick_get();

            pPid[FRONT_RIGHT].err_sum += pAhrs->pitch;
            fr_value -= (int32_t)(pPid[FRONT_RIGHT].P * pAhrs->pitch 
                        + pPid[FRONT_RIGHT].I * pPid[FRONT_RIGHT].err_sum 
                        + pPid[FRONT_RIGHT].D * ((pAhrs->pitch-pPid[FRONT_RIGHT].last_err) / (rt_tick_get()-pPid[FRONT_RIGHT].last_t)));
            pPid[FRONT_RIGHT].last_err = pAhrs->pitch;
            pPid[FRONT_RIGHT].last_t = rt_tick_get();
//        }else if(pAhrs->pitch < 0){
//            pPid[REAR_LEFT].err_sum += pAhrs->pitch;
//            rl_value -= (int32_t)(pPid[REAR_LEFT].P * pAhrs->pitch 
//                        + pPid[REAR_LEFT].I * pPid[REAR_LEFT].err_sum 
//                        + pPid[REAR_LEFT].D * ((pAhrs->pitch-pPid[REAR_LEFT].last_err) / (rt_tick_get()-pPid[REAR_LEFT].last_t)));
//            pPid[REAR_LEFT].last_err = pAhrs->pitch;
//            pPid[REAR_LEFT].last_t = rt_tick_get();
//            

//            pPid[REAR_RIGHT].err_sum += pAhrs->pitch;
//            rr_value -= (int32_t)(pPid[REAR_RIGHT].P * pAhrs->pitch 
//                        + pPid[REAR_RIGHT].I * pPid[REAR_RIGHT].err_sum 
//                        + pPid[REAR_RIGHT].D * ((pAhrs->pitch-pPid[REAR_RIGHT].last_err) / (rt_tick_get()-pPid[REAR_RIGHT].last_t)));
//            pPid[REAR_RIGHT].last_err = pAhrs->pitch;
//            pPid[REAR_RIGHT].last_t = rt_tick_get();
//        

//            pPid[FRONT_LEFT].err_sum += pAhrs->pitch;
//            fl_value += (int32_t)(pPid[FRONT_LEFT].P * pAhrs->pitch 
//                        + pPid[FRONT_LEFT].I * pPid[FRONT_LEFT].err_sum 
//                        + pPid[FRONT_LEFT].D * ((pAhrs->pitch-pPid[FRONT_LEFT].last_err) / (rt_tick_get()-pPid[FRONT_LEFT].last_t)));
//            pPid[FRONT_LEFT].last_err = pAhrs->pitch;
//            pPid[FRONT_LEFT].last_t = rt_tick_get();

//            pPid[FRONT_RIGHT].err_sum += pAhrs->pitch;
//            fr_value += (int32_t)(pPid[FRONT_RIGHT].P * pAhrs->pitch 
//                        + pPid[FRONT_RIGHT].I * pPid[FRONT_RIGHT].err_sum 
//                        + pPid[FRONT_RIGHT].D * ((pAhrs->pitch-pPid[FRONT_RIGHT].last_err) / (rt_tick_get()-pPid[FRONT_RIGHT].last_t)));
//            pPid[FRONT_RIGHT].last_err = pAhrs->pitch;
//            pPid[FRONT_RIGHT].last_t = rt_tick_get();
//        }
        
       
    }
    
    if(fabs(pAhrs->roll) < 90){
        //if(pAhrs->roll  > 0){
            pPid[REAR_LEFT].err_sum += pAhrs->roll;
            rl_value -= (int32_t)(pPid[REAR_LEFT].P * pAhrs->roll 
                        + pPid[REAR_LEFT].I * pPid[REAR_LEFT].err_sum 
                        + pPid[REAR_LEFT].D * ((pAhrs->roll-pPid[REAR_LEFT].last_err) / (rt_tick_get()-pPid[REAR_LEFT].last_t)));
            pPid[REAR_LEFT].last_err = pAhrs->roll;
            pPid[REAR_LEFT].last_t = rt_tick_get();

            pPid[FRONT_LEFT].err_sum += pAhrs->roll;
            fl_value -= (int32_t)(pPid[FRONT_LEFT].P * pAhrs->roll 
                        + pPid[FRONT_LEFT].I * pPid[FRONT_LEFT].err_sum 
                        + pPid[FRONT_LEFT].D * ((pAhrs->roll-pPid[FRONT_LEFT].last_err) / (rt_tick_get()-pPid[FRONT_LEFT].last_t)));
            pPid[FRONT_LEFT].last_err = pAhrs->roll;
            pPid[FRONT_LEFT].last_t = rt_tick_get();

            pPid[REAR_RIGHT].err_sum += pAhrs->roll;
            rr_value += (int32_t)(pPid[REAR_RIGHT].P * pAhrs->roll 
                        + pPid[REAR_RIGHT].I * pPid[REAR_RIGHT].err_sum 
                        + pPid[REAR_RIGHT].D * ((pAhrs->roll-pPid[REAR_RIGHT].last_err) / (rt_tick_get()-pPid[REAR_RIGHT].last_t)));
            pPid[REAR_RIGHT].last_err = pAhrs->roll;
            pPid[REAR_RIGHT].last_t = rt_tick_get();

            pPid[FRONT_RIGHT].err_sum += pAhrs->roll;
            fr_value += (int32_t)(pPid[FRONT_RIGHT].P * pAhrs->roll 
                        + pPid[FRONT_RIGHT].I * pPid[FRONT_RIGHT].err_sum 
                        + pPid[FRONT_RIGHT].D * ((pAhrs->roll-pPid[FRONT_RIGHT].last_err) / (rt_tick_get()-pPid[FRONT_RIGHT].last_t)));
            pPid[FRONT_RIGHT].last_err = pAhrs->roll;
            pPid[FRONT_RIGHT].last_t = rt_tick_get();
//        }else if(pAhrs->roll < 0){
//            pPid[REAR_LEFT].err_sum += pAhrs->roll;
//            rl_value += (int32_t)(pPid[REAR_LEFT].P * pAhrs->roll 
//                        + pPid[REAR_LEFT].I * pPid[REAR_LEFT].err_sum 
//                        + pPid[REAR_LEFT].D * ((pAhrs->roll-pPid[REAR_LEFT].last_err) / (rt_tick_get()-pPid[REAR_LEFT].last_t)));
//            pPid[REAR_LEFT].last_err = pAhrs->roll;
//            pPid[REAR_LEFT].last_t = rt_tick_get();

//            pPid[FRONT_LEFT].err_sum += pAhrs->roll;
//            fl_value += (int32_t)(pPid[FRONT_LEFT].P * pAhrs->roll 
//                        + pPid[FRONT_LEFT].I * pPid[FRONT_LEFT].err_sum 
//                        + pPid[FRONT_LEFT].D * ((pAhrs->roll-pPid[FRONT_LEFT].last_err) / (rt_tick_get()-pPid[FRONT_LEFT].last_t)));
//            pPid[FRONT_LEFT].last_err = pAhrs->roll;
//            pPid[FRONT_LEFT].last_t = rt_tick_get();

//            pPid[REAR_RIGHT].err_sum += pAhrs->roll;
//            rr_value -= (int32_t)(pPid[REAR_RIGHT].P * pAhrs->roll 
//                        + pPid[REAR_RIGHT].I * pPid[REAR_RIGHT].err_sum 
//                        + pPid[REAR_RIGHT].D * ((pAhrs->roll-pPid[REAR_RIGHT].last_err) / (rt_tick_get()-pPid[REAR_RIGHT].last_t)));
//            pPid[REAR_RIGHT].last_err = pAhrs->roll;
//            pPid[REAR_RIGHT].last_t = rt_tick_get();

//            pPid[FRONT_RIGHT].err_sum += pAhrs->roll;
//            fr_value -= (int32_t)(pPid[FRONT_RIGHT].P * pAhrs->roll 
//                        + pPid[FRONT_RIGHT].I * pPid[FRONT_RIGHT].err_sum 
//                        + pPid[FRONT_RIGHT].D * ((pAhrs->roll-pPid[FRONT_RIGHT].last_err) / (rt_tick_get()-pPid[FRONT_RIGHT].last_t)));
//            pPid[FRONT_RIGHT].last_err = pAhrs->roll;
//            pPid[FRONT_RIGHT].last_t = rt_tick_get();
//        }
        
    }
	pAf->thr[REAR_LEFT] += rl_value;
	pAf->thr[REAR_LEFT] = pAf->thr[REAR_LEFT] > pPid[REAR_LEFT].max ? pPid[REAR_LEFT].max : pAf->thr[REAR_LEFT];
	pAf->thr[REAR_LEFT] = pAf->thr[REAR_LEFT] < pPid[REAR_LEFT].min ? pPid[REAR_LEFT].min : pAf->thr[REAR_LEFT];
	
	pAf->thr[REAR_RIGHT] += rr_value;
	pAf->thr[REAR_RIGHT] = pAf->thr[REAR_RIGHT] > pPid[REAR_RIGHT].max ? pPid[REAR_RIGHT].max : pAf->thr[REAR_RIGHT];
	pAf->thr[REAR_RIGHT] = pAf->thr[REAR_RIGHT] < pPid[REAR_RIGHT].min ? pPid[REAR_RIGHT].min : pAf->thr[REAR_RIGHT];
	
	pAf->thr[FRONT_LEFT] += fl_value;
	pAf->thr[FRONT_LEFT] = pAf->thr[FRONT_LEFT] > pPid[FRONT_LEFT].max ? pPid[FRONT_LEFT].max : pAf->thr[FRONT_LEFT];
	pAf->thr[FRONT_LEFT] = pAf->thr[FRONT_LEFT] < pPid[FRONT_LEFT].min ? pPid[FRONT_LEFT].min : pAf->thr[FRONT_LEFT];
	
	pAf->thr[FRONT_RIGHT] += fr_value;
	pAf->thr[FRONT_RIGHT] = pAf->thr[FRONT_RIGHT] > pPid[FRONT_RIGHT].max ? pPid[FRONT_RIGHT].max : pAf->thr[FRONT_RIGHT];
	pAf->thr[FRONT_RIGHT] = pAf->thr[FRONT_RIGHT] < pPid[FRONT_RIGHT].min ? pPid[FRONT_RIGHT].min : pAf->thr[FRONT_RIGHT];
	
	rt_kprintf("1ahrs %f|%f pid %d|%d|%d|%d|\r\n", pAhrs->pitch, pAhrs->roll, fr_value, fl_value, rl_value, rr_value);
	rt_kprintf("2ahrs %f|%f pid %d|%d|%d|%d|\r\n", pAhrs->pitch, pAhrs->roll, pAf->thr[FRONT_RIGHT], pAf->thr[FRONT_LEFT], pAf->thr[REAR_LEFT], pAf->thr[REAR_RIGHT]);

}


