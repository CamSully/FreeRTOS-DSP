/*
 * FreeRTOS Kernel V10.3.1
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */
/* Scheduler includes. */
#include <stdio.h>
#include "FreeRTOS.h"
#include "task.h"

/* App includes. */
#include "app_main.h"
#include "stm32l4xx_hal.h"
#include "init486_def.h"

void TIM4_Config(uint16_t);
static void init_dac(uint16_t, enum Num_Channels_Out);
void light_green_led(void*);
void light_red_led(void*);



// CWS FUNCTION BEGIN
void light_red_led(void *p) 
{    
    // Turn on red LED.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_SET);
    
    // Delay 1 second.
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Turn off red LED.
    HAL_GPIO_WritePin(GPIOB, GPIO_PIN_2, GPIO_PIN_RESET);
    
    xTaskCreate(light_green_led, "green_led", 1024, NULL, 1, NULL);
    vTaskDelete(NULL);
    
    while(1);
}
// CWS FUNCTION END



// CWS FUNCTION BEGIN
void light_green_led(void *p) 
{  
    // Turn on green LED.
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_SET);
    
    // Delay 1 second.
    vTaskDelay(pdMS_TO_TICKS(1000));
    
    // Turn off green LED.
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_8, GPIO_PIN_RESET);
    
    xTaskCreate(light_red_led, "red_led", 1024, NULL, 1, NULL);
    vTaskDelete(NULL);
    
    while(1);
}
// CWS FUNCTION END


static DAC_HandleTypeDef DacHandle;
enum Num_Channels_Out Output_Configuration;
enum Num_Channels_In Input_Configuration;

void app_main( void )
{
        // Initialize green and red LEDs.
        GPIO_InitTypeDef GPIO_InitStruct;
        __GPIOB_CLK_ENABLE();
        __GPIOE_CLK_ENABLE();
  
        GPIO_InitStruct.Pin = GPIO_PIN_2;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);
        
        GPIO_InitStruct.Pin = GPIO_PIN_8;
        GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
        GPIO_InitStruct.Speed = GPIO_SPEED_LOW;
        HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);
                
        // DSP Init
        uint16_t fs = 1667;      // 48 ksps == 1667.
        TIM4_Config(fs);
        init_dac(fs, STEREO_OUT);

        
        // CWS - create task here.
        /* Params:
            - Pointer to entry function (function name).
            - Name for task
            - Stack depth (num words)
            - Task param
            - Task priority
            - Handle (optional, can be NULL).
        */
        
        TaskHandle_t red_handle = NULL;
        
        xTaskCreate(light_red_led, "red_led", 1024, NULL, 1, &red_handle);
        // xTaskCreate(light_green_led, "green_led", 1024, NULL, 1, &green_handle);
        
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Should not get here. */
	for( ;; );
}
/*-----------------------------------------------------------*/

// CWS - from init486.c
static void TIM4_Config(uint16_t fs)
{
        static TIM_HandleTypeDef htim;
        TIM_MasterConfigTypeDef  sMasterConfig;
          
        htim.Instance = TIM4;
          
        /* Number of timer counts per ADC sample....  The timer 4 clock frequency is
        * the APB1 Bus frequency: 80 MHz.  For example, To get 50 ksps, we're
        * counting (50 MHz)/(50 ksps) = 1600 counts
        */
        htim.Init.Period = fs;
        htim.Init.Prescaler = 0;
        htim.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
        htim.Init.CounterMode = TIM_COUNTERMODE_UP;
        HAL_TIM_Base_Init(&htim);
        
        sMasterConfig.MasterOutputTrigger = TIM_TRGO_UPDATE;
        sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
         
        HAL_TIMEx_MasterConfigSynchronization(&htim, &sMasterConfig);
          
        /*##-2- Enable TIM peripheral counter ######################################*/
        HAL_TIM_Base_Start(&htim);
}


// CWS - from init486.c
static void init_dac(uint16_t fs, enum Num_Channels_Out chanout) {
  static DAC_ChannelConfTypeDef DacConfig;

  DacHandle.Instance = DAC;

  /*
   * DAC Channel 2...  Trigger with Timer 4 TRG0 Event
   * Note that HAL_DAC_Init() calls HAL_DAC_MspInit() (The MPU-specific initialization
   * routine).  This function must be re-defined  below in order to associate
   * the correct DMA stream with the DAC.  Similarly, HAL_DAC_DeInit() calls
   * HAL_DAC_MspDeInit()... which we're on the hook to define.
   */
  if(HAL_DAC_Init(&DacHandle) != HAL_OK)  flagerror(DAC_CONFIG_ERROR);

  DacConfig.DAC_Trigger = DAC_TRIGGER_T4_TRGO;
  DacConfig.DAC_OutputBuffer = DAC_OUTPUTBUFFER_ENABLE;  // Output Buffer Amp
  DacConfig.DAC_UserTrimming = DAC_TRIMMING_FACTORY;
  DacConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_DISABLE;
  DacConfig.DAC_SampleAndHold = DAC_SAMPLEANDHOLD_DISABLE;

  if (chanout == MONO_OUT) {
    if(HAL_DAC_ConfigChannel(&DacHandle, &DacConfig, DAC_CHANNEL_2) != HAL_OK)
      flagerror(DAC_CONFIG_ERROR);

    if( HAL_DAC_Start_DMA(&DacHandle,
                          DAC_CHANNEL_2,
                          (uint32_t *)DAC_Output_Buffer,
                          ADC_Buffer_Size,
                          DAC_ALIGN_12B_L)
        != HAL_OK) flagerror(DAC_CONFIG_ERROR);

  } else if (chanout == STEREO_OUT) {

   // DAC Channel 2 is the same as the Mono case... Channel 2 drives PA5 directly
   if(HAL_DAC_ConfigChannel(&DacHandle, &DacConfig, DAC_CHANNEL_2) != HAL_OK)
     flagerror(DAC_CONFIG_ERROR);

   // DAC Channel 1 is directed to opamp1, which can drive PA3
   // (This is because PA4 is not extended to the connector on the discovery board,
   //  and PA3 is extended.  The opamp is used to get the dac stignal to PA3.)
   opamp_init();
   DacConfig.DAC_ConnectOnChipPeripheral = DAC_CHIPCONNECT_ENABLE;
   if(HAL_DAC_ConfigChannel(&DacHandle, &DacConfig, DAC_CHANNEL_1) != HAL_OK)
     flagerror(DAC_CONFIG_ERROR);

    /*
    * To start the DAC in dual-sample mode, use a hacked up version of
    * HAL_DAC_Start_DMA()...  The HAL library does not seem to support the
    * dual mode very well!
    */
   if( Hacked_HAL_DAC_DualStart_DMA(&DacHandle,
                          (uint32_t *)DAC_Output_Buffer,
                          ADC_Buffer_Size)
        != HAL_OK) flagerror(DAC_CONFIG_ERROR);
 } else {
   flagerror(DAC_CONFIG_ERROR);
  }
}




// CWS - had to add this because it was originally defined in mpu_demo.c.
void vHandleMemoryFault(void)
{
  while(1);
}

void vApplicationStackOverflowHook( TaskHandle_t pxTask, char *pcTaskName )
{
	/* If configCHECK_FOR_STACK_OVERFLOW is set to either 1 or 2 then this
	function will automatically get called if a task overflows its stack. */
	( void ) pxTask;
	( void ) pcTaskName;
	for( ;; );
}
/*-----------------------------------------------------------*/

void vApplicationMallocFailedHook( void )
{
	/* If configUSE_MALLOC_FAILED_HOOK is set to 1 then this function will
	be called automatically if a call to pvPortMalloc() fails.  pvPortMalloc()
	is called automatically when a task, queue or semaphore is created. */
	for( ;; );
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION is set to 1, so the application must provide an
implementation of vApplicationGetIdleTaskMemory() to provide the memory that is
used by the Idle task. */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
/* If the buffers to be provided to the Idle task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xIdleTaskTCB;
static StackType_t uxIdleTaskStack[ configMINIMAL_STACK_SIZE ];

	/* Pass out a pointer to the StaticTask_t structure in which the Idle task's
	state will be stored. */
	*ppxIdleTaskTCBBuffer = &xIdleTaskTCB;

	/* Pass out the array that will be used as the Idle task's stack. */
	*ppxIdleTaskStackBuffer = uxIdleTaskStack;

	/* Pass out the size of the array pointed to by *ppxIdleTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
}
/*-----------------------------------------------------------*/

/* configUSE_STATIC_ALLOCATION and configUSE_TIMERS are both set to 1, so the
application must provide an implementation of vApplicationGetTimerTaskMemory()
to provide the memory that is used by the Timer service task. */
void vApplicationGetTimerTaskMemory( StaticTask_t **ppxTimerTaskTCBBuffer, StackType_t **ppxTimerTaskStackBuffer, uint32_t *pulTimerTaskStackSize )
{
/* If the buffers to be provided to the Timer task are declared inside this
function then they must be declared static - otherwise they will be allocated on
the stack and so not exists after this function exits. */
static StaticTask_t xTimerTaskTCB;
static StackType_t uxTimerTaskStack[ configTIMER_TASK_STACK_DEPTH ];

	/* Pass out a pointer to the StaticTask_t structure in which the Timer
	task's state will be stored. */
	*ppxTimerTaskTCBBuffer = &xTimerTaskTCB;

	/* Pass out the array that will be used as the Timer task's stack. */
	*ppxTimerTaskStackBuffer = uxTimerTaskStack;

	/* Pass out the size of the array pointed to by *ppxTimerTaskStackBuffer.
	Note that, as the array is necessarily of type StackType_t,
	configMINIMAL_STACK_SIZE is specified in words, not bytes. */
	*pulTimerTaskStackSize = configTIMER_TASK_STACK_DEPTH;
}
/*-----------------------------------------------------------*/
