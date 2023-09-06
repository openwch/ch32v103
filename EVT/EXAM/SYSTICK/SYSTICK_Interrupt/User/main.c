/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/
/*
 *@Note
 Systick interrupt:
 USART1_Tx(PA9).
*/

#include "debug.h"
/* Global typedef */
/* Global define */
/* Global Variable */

uint32_t counter;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));


/*********************************************************************
 * @fn      SYSTICK_Init_Config
 *
 * @brief   SYSTICK_Init_Config.
 *
 * @return  none
 */
void SYSTICK_Init_Config(u_int32_t ticks)
{
    SysTick->CTLR = 0x0000;//关闭系统计数器

    SysTick->CNTL0 = 0;
    SysTick->CNTL1 = 0;
    SysTick->CNTL2 = 0;
    SysTick->CNTL3 = 0;

    SysTick->CNTH0 = 0;
    SysTick->CNTH1 = 0;
    SysTick->CNTH2 = 0;
    SysTick->CNTH3 = 0;

    SysTick->CMPLR0 = (u8)(ticks & 0xFF);
    SysTick->CMPLR1 = (u8)(ticks >> 8);
    SysTick->CMPLR2 = (u8)(ticks >> 16);
    SysTick->CMPLR3 = (u8)(ticks >> 24);

    SysTick->CMPHR0 = 0;
    SysTick->CMPHR1 = 0;
    SysTick->CMPHR2 = 0;
    SysTick->CMPHR3 = 0;

    NVIC_SetPriority(SysTicK_IRQn, 15);
    NVIC_EnableIRQ(SysTicK_IRQn);

    SysTick->CTLR = (1<<0);
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
    SystemCoreClockUpdate();
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    USART_Printf_Init( 115200 );
    printf("SystemClk:%d\r\n",SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID());
    SYSTICK_Init_Config(SystemCoreClock-1);
    while(1)
    {
    }
}

/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   SysTick_Handler
 *
 * @return  none
 */
void SysTick_Handler(void)
   {
    printf("welcome to WCH\r\n");
    SysTick->CNTL0 = 0;
    SysTick->CNTL1 = 0;
    SysTick->CNTL2 = 0;
    SysTick->CNTL3 = 0;

    SysTick->CNTH0 = 0;
    SysTick->CNTH1 = 0;
    SysTick->CNTH2 = 0;
    SysTick->CNTH3 = 0;

    counter++;
    Delay_Ms(250);
    printf("Counter:%d\r\n",counter);

   }
