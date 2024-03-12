/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2024/02/21
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/**
 * @note
 * The code initializes the RTC (Real-Time Clock) and TIM1 (Timer 1) modules, waits for a calibration
 * flag to be set, and then prints the calibration value and adjust the calibration register
 */
#include "debug.h"

/* Global define */
#define LSI_VALUE 40000
/* Global Variable */

/* Exported_Functions */
volatile uint8_t SampleFlag = 0;
volatile uint32_t rtc_cnt = 0;

void TIM1_UP_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
void TIM1_UP_IRQHandler()
{
    rtc_cnt = RTC_GetCounter();
    SampleFlag = 1;
    RTC_SetCounter(0);
    TIM_ClearITPendingBit(TIM1, TIM_IT_Update);
}

/*********************************************************************
 * @fn      RTC_Init
 *
 * @brief   Initializes RTC collection.
 *
 * @return  1 - Init Fail
 *          0 - Init Success
 */
u8 RTC_Init(void)
{
    u8 temp = 0;
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);

    /* Is it the first configuration */

    BKP_DeInit();
    RCC_LSICmd(ENABLE);
    while (RCC_GetFlagStatus(RCC_FLAG_HSIRDY) == RESET && temp < 250)
    {
        temp++;
        Delay_Ms(20);
    }
    if (temp >= 250)
        return 1;
    RCC_RTCCLKConfig(RCC_RTCCLKSource_LSI);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForLastTask();
    RTC_WaitForSynchro();
    RTC_ITConfig(RTC_IT_SEC, ENABLE);
    RTC_WaitForLastTask();
    RTC_EnterConfigMode();
    RTC_SetPrescaler(0);
    RTC_WaitForLastTask();
    RTC_ExitConfigMode();
    BKP_WriteBackupRegister(BKP_DR1, 0XA1A1);
    return 0;
}

/*********************************************************************
 * @fn      TIM1_Base_Init
 *
 * @brief   Initializes TIM1 output compare.
 *
 * @param   arr - the period value.
 *          psc - the prescaler value.
 *          ccp - the pulse value.
 *
 * @return  none
 */
void TIM1_Base_Init(u16 arr, u16 psc)
{
    TIM_TimeBaseInitTypeDef TIM_TimeBaseInitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);

    TIM_TimeBaseInitStructure.TIM_Period = arr;
    TIM_TimeBaseInitStructure.TIM_Prescaler = psc;
    TIM_TimeBaseInitStructure.TIM_ClockDivision = TIM_CKD_DIV1;
    TIM_TimeBaseInitStructure.TIM_CounterMode = TIM_CounterMode_Up;
    TIM_TimeBaseInit(TIM1, &TIM_TimeBaseInitStructure);

    NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_Init(&NVIC_InitStructure);

    TIM_ITConfig(TIM1, TIM_IT_Update, ENABLE);
    RTC_SetCounter(0);

    TIM_Cmd(TIM1, ENABLE);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf("ChipID:%08x\r\n", DBGMCU_GetCHIPID());
    printf("RTC Test\r\n");

    /* The above code is initializing the TIM1 timer and the RTC (Real-Time Clock) module. It then
    waits for a calibration flag to be set. Once the flag is set, it prints the calibration value. */
    RCC_ClocksTypeDef RCC_Clocks;
    RCC_GetClocksFreq(&RCC_Clocks);
    printf("TIM1 Freq: %d\r\n", RCC_Clocks.PCLK2_Frequency);
    RTC_Init();
    Delay_Ms(100);
    TIM1_Base_Init(10000, RCC_Clocks.PCLK2_Frequency / 100000 - 1);
    /* The code is performing a calibration of the HSI (High-Speed Internal) clock frequency. */
    static uint32_t Meas_Freq = 0;

    SampleFlag = 0;
    while (!SampleFlag);
    Delay_Ms(100);
    Meas_Freq = (uint32_t)((float)LSI_VALUE * (float)rtc_cnt/(float)(LSI_VALUE / 10));

    printf("%d\r\n",Meas_Freq);

    RTC_SetCounter(Meas_Freq);
    WWDG_SetPrescaler(Meas_Freq);

    while (1)
    {
        Delay_Ms(1000);
    }
}
