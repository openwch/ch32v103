/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2021/06/06
* Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
/*
 *@Note
 *HSE frequency check routine:
 *HSE value -the frequency of HSE
 *MCO(PA8)	- outputs the HSE clock
	 
*/

#include "debug.h"

u8 value;
uint32_t HSEFrequencyMhz;
#define  F1M                      1000000
#define  CHECK_ERR_COUNT          (8)
#define  RTC_DIV_VAL              (128)
#define  RTC_COUNT_DELAY          (100)
#define  HSEClock        (8000000)
#define  RTC_CHANGE_TIMEOUT       ((HSEClock/F1M)*RTC_DIV_VAL)
#define  RTC_COUNT_MAX_TIMEOUT    ((HSEClock/F1M)*RTC_DIV_VAL*RTC_COUNT_DELAY)
#define  PPL2_ENABLE              1
#define  HSE_ENABLE               0

/*********************************************************************
 * @fn      HSE_FrequencyCheck
 *
 * @brief   Checking HSE Frequency  Values.
 *
 * @return  HSEFrequencyMhz - HSE Frequency
 */
uint8_t HSE_FrequencyCheck(void) {
    uint32_t rtc_val;
    uint32_t tick_val;
    uint32_t HSEFrequency, HSEFrequency_x;
    uint8_t count;

    Delay_Init();
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR | RCC_APB1Periph_BKP, ENABLE);
    PWR_BackupAccessCmd(ENABLE);
    BKP_DeInit();
    RCC_RTCCLKConfig(RCC_RTCCLKSource_HSE_Div128);
    RCC_RTCCLKCmd(ENABLE);
    RTC_WaitForLastTask();
    RTC_WaitForSynchro();
    RTC_WaitForLastTask();
    RTC_EnterConfigMode();
    RTC_SetPrescaler(0);
    RTC_WaitForLastTask();
    RTC_ExitConfigMode();
    while( RCC->BDCTLR & (1 << 15)!=(1 << 15)) {

    }
    count = CHECK_ERR_COUNT;
    while(1) {
        SysTick->CTLR = 0;
        SysTick->CNTL0 = 0;
        SysTick->CNTL1 = 0;
        SysTick->CNTL2 = 0;
        SysTick->CNTL3 = 0;
        SysTick->CTLR =1;

        rtc_val = RTC_GetCounter();

        while( rtc_val == RTC_GetCounter()) {
            if( SysTick->CNTL0+(SysTick->CNTL1<<8)+(SysTick->CNTL2<<16)+(SysTick->CNTL3<<24) > RTC_CHANGE_TIMEOUT ) {
                return 0;
            }
        }
        SysTick->CTLR = 0;
        SysTick->CNTL0 = 0;
        SysTick->CNTL1 = 0;
        SysTick->CNTL2 = 0;
        SysTick->CNTL3 = 0;

        SysTick->CTLR = 1;

        rtc_val += (RTC_COUNT_DELAY);
        while( rtc_val >= RTC_GetCounter() ) {
            if( SysTick->CNTL0+(SysTick->CNTL1<<8)+(SysTick->CNTL2<<16)+(SysTick->CNTL3<<24) > RTC_COUNT_MAX_TIMEOUT ) {
                return 0;
            }
        }

        tick_val = SysTick->CNTL0+(SysTick->CNTL1<<8)+(SysTick->CNTL2<<16)+(SysTick->CNTL3<<24);
        SysTick->CTLR = 0;
        HSEFrequency = (HSEClock)/(tick_val/(RTC_DIV_VAL/8))*RTC_COUNT_DELAY;
        HSEFrequencyMhz = (HSEFrequency+(F1M/2))/F1M;

        /* �ظ�һ�� */
        SysTick->CTLR = 0;
        SysTick->CNTL0 = 0;
        SysTick->CNTL1 = 0;
        SysTick->CNTL2 = 0;
        SysTick->CNTL3 = 0;
        SysTick->CTLR = 1;
        rtc_val = RTC_GetCounter();
        while( rtc_val == RTC_GetCounter() ) {
            if( SysTick->CNTL0+(SysTick->CNTL1<<8)+(SysTick->CNTL2<<16)+(SysTick->CNTL3<<24) > RTC_CHANGE_TIMEOUT ) {
                return 0;
            }
        }
        SysTick->CTLR = 0;
        SysTick->CNTL0 = 0;
        SysTick->CNTL1 = 0;
        SysTick->CNTL2 = 0;
        SysTick->CNTL3 = 0;
        SysTick->CTLR = 1;
        rtc_val += (RTC_COUNT_DELAY);
        while( rtc_val >= RTC_GetCounter() ) {
            if( SysTick->CNTL0+(SysTick->CNTL1<<8)+(SysTick->CNTL2<<16)+(SysTick->CNTL3<<24) > RTC_COUNT_MAX_TIMEOUT ) {
                return 0;
            }
        }
        tick_val = SysTick->CNTL0+(SysTick->CNTL1<<8)+(SysTick->CNTL2<<16)+(SysTick->CNTL3<<24);

        SysTick->CTLR = 0;

        HSEFrequency = (HSEClock)/(tick_val/(RTC_DIV_VAL/8))*RTC_COUNT_DELAY;

        HSEFrequency_x = (HSEFrequency+(F1M/2))/F1M;


        if( HSEFrequencyMhz == HSEFrequency_x ) {
            break;
        }
        if( --count == 0 ) {
            HSEFrequencyMhz = 253;
            printf("HSE check error.\n");
            break;
        }
    }
    return HSEFrequencyMhz;
}

/*********************************************************************
 * @fn      SetSysClockTo48
 *
 * @brief   Sets System clock frequency to 48MHz and configure HCLK, PCLK2 and PCLK1 prescalers.
 *
 * @return  none
 */
static void SetSysClockTo48(void)
{
    __IO uint32_t StartUpCounter = 0, HSEStatus = 0;

    RCC->CTLR |= ((uint32_t)RCC_HSEON);
    /* Wait till HSE is ready and if Time out is reached exit */
    do
    {
        HSEStatus = RCC->CTLR & RCC_HSERDY;
        StartUpCounter++;
    }while((HSEStatus == 0) && (StartUpCounter != HSE_STARTUP_TIMEOUT));

    if((RCC->CTLR & RCC_HSERDY) != RESET)
    {
        HSEStatus = (uint32_t) 0x00;
        value = HSE_FrequencyCheck();
        if ((value >= 3) && (value <= 25)) {
            HSEStatus = (uint32_t) 0x01;
        }
    }
    else
    {
        HSEStatus = (uint32_t)0x00;
    }

    if(HSEStatus == (uint32_t)0x01)
    {
        /* Enable Prefetch Buffer */
        FLASH->ACTLR |= FLASH_ACTLR_PRFTBE;

        /* Flash 1 wait state */
        FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
        FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_1;

        /* HCLK = SYSCLK */
        RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
        /* PCLK2 = HCLK */
        RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
        /* PCLK1 = HCLK */
        RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV2;

        /*  PLL configuration: PLLCLK = HSE * 6 = 48 MHz */
        RCC->CFGR0 &= (uint32_t)((uint32_t) ~(RCC_PLLSRC | RCC_PLLXTPRE | RCC_PLLMULL));
        RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSE | RCC_PLLMULL6);

        /* Enable PLL */
        RCC->CTLR |= RCC_PLLON;
        /* Wait till PLL is ready */
        while((RCC->CTLR & RCC_PLLRDY) == 0)
        {
        }
        /* Select PLL as system clock source */
        RCC->CFGR0 &= (uint32_t)((uint32_t) ~(RCC_SW));
        RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
        /* Wait till PLL is used as system clock source */
        while((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
        {
        }
    }
    else
    {
        EXTEN->EXTEN_CTR |=EXTEN_PLL_HSI_PRE;
        /* Enable Prefetch Buffer */
        FLASH->ACTLR |= FLASH_ACTLR_PRFTBE;

        /* Flash 2 wait state */
        FLASH->ACTLR &= (uint32_t)((uint32_t)~FLASH_ACTLR_LATENCY);
        FLASH->ACTLR |= (uint32_t)FLASH_ACTLR_LATENCY_2;

        /* HCLK = SYSCLK */
        RCC->CFGR0 |= (uint32_t)RCC_HPRE_DIV1;
        /* PCLK2 = HCLK */
        RCC->CFGR0 |= (uint32_t)RCC_PPRE2_DIV1;
        /* PCLK1 = HCLK */
        RCC->CFGR0 |= (uint32_t)RCC_PPRE1_DIV2;

        /*  PLL configuration: PLLCLK = HSE * 9 = 72 MHz */
        RCC->CFGR0 &= (uint32_t)((uint32_t) ~(RCC_PLLSRC | RCC_PLLXTPRE |
                        RCC_PLLMULL));
        RCC->CFGR0 |= (uint32_t)(RCC_PLLSRC_HSI_Div2 | RCC_PLLMULL6);
        /* Enable PLL */
        RCC->CTLR |= RCC_PLLON;
        /* Wait till PLL is ready */
        while((RCC->CTLR & RCC_PLLRDY) == 0)
        {
        }
        /* Select PLL as system clock source */
        RCC->CFGR0 &= (uint32_t)((uint32_t) ~(RCC_SW));
        RCC->CFGR0 |= (uint32_t)RCC_SW_PLL;
        /* Wait till PLL is used as system clock source */
        while((RCC->CFGR0 & (uint32_t)RCC_SWS) != (uint32_t)0x08)
        {
        }
    }
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
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);
    SetSysClockTo48();
    USART_Printf_Init(115200);
    SystemCoreClockUpdate();
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf(" HSE  value:%02d\r\n", value);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    /* MCO Output GPIOA 8 */
    RCC_MCOConfig(RCC_MCO_HSE);

    while(1);
}
