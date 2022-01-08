/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/12/1
 * Description        : Main program body.
 *******************************************************************************/

#include "debug.h"
#include "Config.h"

/*********************************************************************
 * @fn      init_timer
 *
 * @brief   timer初始化
 *
 * @return  none
 */
void init_timer(void)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure;
    NVIC_InitTypeDef        NVIC_InitStructure;

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_TIM4, ENABLE);
    //--TIMER4--
    TIM_TimeBaseInitStructure.TIM_Period = 2000 - 1;
    TIM_TimeBaseInitStructure.TIM_Prescaler = 36 - 1;           //36分频
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1; //1分频
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM4, &TIM_TimeBaseInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM4_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 7;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
    TIM_ClearITPendingBit(TIM4, TIM_IT_Update);
    TIM_ITConfig(TIM4, TIM_IT_Update, ENABLE);

    TIM_Cmd(TIM4, ENABLE);
}
