/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2026/10/15
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *VTF IRQ interrupt routine:
 *This example is used to demonstrate VTF IRQ
 */
#include "debug.h"

/* Global define */

/* Global Variable */
volatile uint64_t time = 0;

void SysTick_Handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      Interrupt_VTF_Init
 *
 * @brief   Initializes VTF.
 *
 * @return  none
 */
void Interrupt_VTF_Init(void)
{
    NVIC_SetPriority(SysTicK_IRQn, 15);
    NVIC_SetFastIRQ((u32)SysTick_Handler, SysTick_IRQn, 0);
    NVIC_EnableIRQ(SysTick_IRQn);
}

/*********************************************************************
 * @fn      Systick_Init
 *
 * @brief   Initializes Systick.
 *
 * @return  none
 */
void Systick_Init(void)
{
    SysTick->CTLR=0;
    SysTick->CNTL0=0;
    SysTick->CNTL1=0;
    SysTick->CNTL2=0;
    SysTick->CNTL3=0;
    SysTick->CNTH0=0;
    SysTick->CNTH1=0;
    SysTick->CNTH2=0;
    SysTick->CNTH3=0;
    SysTick->CMPLR0=0x20;
    SysTick->CMPLR1=0;
    SysTick->CMPLR2=0;
    SysTick->CMPLR3=0;
    SysTick->CMPHR0=0;
    SysTick->CMPHR1=0;
    SysTick->CMPHR2=0;
    SysTick->CMPHR3=0;
    SysTick->CTLR=0x1;
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
    Delay_Init();
#if (SDI_PRINT == SDI_PR_OPEN)
    SDI_Printf_Enable();
#else
    USART_Printf_Init(115200);
#endif
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("Interrupt VTF Test\r\n");

    Interrupt_VTF_Init();
    Systick_Init();

    while(1)
    {

    }
}

/*********************************************************************
 * @fn      SysTick_Handler
 *
 * @brief   This function handles SysTick exception.
 *
 * @return  none
 */
void SysTick_Handler(void)
{
    time=(((uint64_t)(*(vu32*)(0xE000F008)))<<32)|*(vu32*)(0xE000F004);
    SysTick->CTLR=0;
    #if 0
    printf("delta time:%d\r\n",time-0x20);
    #endif
}
