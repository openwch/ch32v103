/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2020/12/1
* Description        : Main program body.
*******************************************************************************/

/*
 *@Note
 串口打印调试例程：
 USART1_Tx(PA9)。
 本例程演示 PA11，PA12 点灯。

*/

#include "debug.h"
#include "Config.h"
#include "FreeRTOS.h"
#include "task.h"
/* Global typedef */

/* Global define */
//-------------任务相关入口参数-------------
#define TASK1_TASK_PRIO     2           //---优先级---
#define TASK1_STK_SIZE      128         //----任务栈长度，需要注意是与M3不同，任务栈同时也需要给中断使用----
TaskHandle_t Task1Task_Handler;         //---任务句柄---
#define TASK2_TASK_PRIO     2
#define TASK2_STK_SIZE      128
TaskHandle_t Task2Task_Handler;

/* Global Variable */

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
void task1_task(void *pvParameters)
{
    while(1)
    {
        GPIO_SetBits(GPIOA, GPIO_Pin_12);
        vTaskDelay(700);
        GPIO_ResetBits(GPIOA, GPIO_Pin_12);
        vTaskDelay(700);
    }
}
void task2_task(void *pvParameters)
{
    while(1)
    {
        GPIO_ResetBits(GPIOA, GPIO_Pin_11);
        vTaskDelay(700);
        GPIO_SetBits(GPIOA, GPIO_Pin_11);
        vTaskDelay(700);
    }
}

int main(void)
{
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);    //------建议使用分组1或直接操作优先级寄存器------
	NVIC_HaltPushCfg(DISABLE);          //-----硬件压栈关闭-----
	init_keyAndLed();                   //-----本例程中，仅作为初始化LED-----
    //---------------创建两个任务--------------------
	xTaskCreate((TaskFunction_t )task2_task,
	                    (const char*    )"task2_task",
	                    (uint16_t       )TASK2_STK_SIZE,
	                    (void*          )NULL,
	                    (UBaseType_t    )TASK2_TASK_PRIO,
	                    (TaskHandle_t*  )&Task2Task_Handler);

	xTaskCreate((TaskFunction_t )task1_task,
                    (const char*    )"task1_task",
                    (uint16_t       )TASK1_STK_SIZE,
                    (void*          )NULL,
                    (UBaseType_t    )TASK1_TASK_PRIO,
                    (TaskHandle_t*  )&Task1Task_Handler);
	//--------启动调度器--------
	vTaskStartScheduler();
	while(1)
	{
	    ;
	}
}

