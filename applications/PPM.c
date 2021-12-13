#include "PPM.h"
#include "stm32f1xx_hal.h"
#include "drv_common.h"

TIM_HandleTypeDef htim2;

#define MAX_PPM_CHANNELS (8)
#define MAX_PULSE_WAIT_TIMEOUT_USEC (3000)

int ppm_channel_timing[MAX_PPM_CHANNELS] = { 0 };

// indicates the system is just booted up
int is_first_start = 1;
// indicates that the first channel pulse is found
int is_in_sync = 0;

int current_channel = 0 ;
 
void TIM2_IRQHandler(void)
{
    rt_interrupt_enter();
    HAL_TIM_IRQHandler(&htim2);
    rt_interrupt_leave();
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim)
{
    if(htim->Instance == TIM2) {
        uint32_t usec = __HAL_TIM_GET_COMPARE(&htim2, TIM_CHANNEL_2);
        ////		// on first start just resetting counter
        if(!is_first_start) {
            if(is_in_sync) {
                if(MAX_PULSE_WAIT_TIMEOUT_USEC <= usec) {
                    current_channel = 0;
                }else {
                    ppm_channel_timing[current_channel] = usec;
                    current_channel++;
                }
            }else {// waiting for first pulse
                if(MAX_PULSE_WAIT_TIMEOUT_USEC <= usec) {
                    is_in_sync = 1;
                }
            }
        }
        __HAL_TIM_SET_COUNTER(&htim2, 0);
        is_first_start = 0;
 	}
}

// Adjust these values if the cursor in the computer jumps at the end points of sticks
// typically it should be 1000 and 2000
#define PPM_MIN (950)
#define PPM_MAX (2000)
#define HID_MAX_VALUE (255)

#define PPM_SCALE(X) ((X - PPM_MIN) * HID_MAX_VALUE / (PPM_MAX - PPM_MIN))

#define CHAN_X (3)
#define CHAN_Y (2)

#define CHAN_RX (0)
#define CHAN_Z (1)

 struct gamepadHID_t {
     uint8_t x;
     uint8_t y;
     uint8_t z;
     uint8_t rx;
 };

 /* USER CODE END 0 */

 /**
   * @brief TIM2 Initialization Function
   * @param None
   * @retval None
   */
 static void MX_TIM2_Init(void)
 {

   /* USER CODE BEGIN TIM2_Init 0 */

   /* USER CODE END TIM2_Init 0 */

   TIM_ClockConfigTypeDef sClockSourceConfig = {0};
   TIM_MasterConfigTypeDef sMasterConfig = {0};
   TIM_IC_InitTypeDef sConfigIC = {0};

   /* USER CODE BEGIN TIM2_Init 1 */

   /* USER CODE END TIM2_Init 1 */
   htim2.Instance = TIM2;
   htim2.Init.Prescaler = 48;
   htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
   htim2.Init.Period = 65535;
   htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
   htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
   if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
   {
	Error_Handler();
   }
   sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
   if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
   {
     Error_Handler();
   }
   if (HAL_TIM_IC_Init(&htim2) != HAL_OK)
   {
     Error_Handler();
   }
   sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
   sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
   if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
   {
     Error_Handler();
   }
   sConfigIC.ICPolarity = TIM_INPUTCHANNELPOLARITY_RISING;
   sConfigIC.ICSelection = TIM_ICSELECTION_DIRECTTI;
   sConfigIC.ICPrescaler = TIM_ICPSC_DIV1;
   sConfigIC.ICFilter = 0;
   if (HAL_TIM_IC_ConfigChannel(&htim2, &sConfigIC, TIM_CHANNEL_2) != HAL_OK)
   {
     Error_Handler();
   }
   /* USER CODE BEGIN TIM2_Init 2 */

   /* USER CODE END TIM2_Init 2 */

 }

 void ppm_thread_entry(void *parameter)
 {
     MX_TIM2_Init();
     struct gamepadHID_t report = { 0 };
     HAL_TIM_IC_Start_IT(&htim2, TIM_CHANNEL_2);
     while (1)
     {
         report.x = PPM_SCALE(ppm_channel_timing[CHAN_X]);
         report.y = PPM_SCALE(ppm_channel_timing[CHAN_Y]);
         report.z = PPM_SCALE(ppm_channel_timing[CHAN_Z]);
         report.rx = PPM_SCALE(ppm_channel_timing[CHAN_RX]);
         rt_kprintf("x %d y %d z %d\r\n", ppm_channel_timing[CHAN_X],ppm_channel_timing[CHAN_Y],ppm_channel_timing[CHAN_Z]);
 				 rt_thread_mdelay(10);
     }
 }

 void initPPMThread()
 {
     rt_thread_t tid;
    
     tid = rt_thread_create("ppm", 
                             ppm_thread_entry,
                             RT_NULL,
                             1024,
                             23,
                             20);
     if (tid == RT_NULL){
				rt_kprintf("ppm thread create failed!\r\n");
				return;
     }
     rt_thread_startup(tid);
 }