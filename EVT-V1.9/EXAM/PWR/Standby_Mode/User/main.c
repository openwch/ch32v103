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
 low power consumption, standby mode routine:
 WKUP(PA0)
 This routine demonstrates that WFI enters the standby mode, the rising edge of the
 WKUP (PA0) pin exits the standby mode,Program reset after wake-up.
 Note: In order to reduce power consumption as much as possible, it is recommended
 to set the unused GPIO to pull-down mode.

*/

#include "debug.h"

/* Global define */

/* Global Variable */

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

    /* To reduce power consumption, unused GPIO should be set as 
	 * pull-up or pull-down input. 
	 */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB |
                               RCC_APB2Periph_GPIOC,
                           ENABLE);
    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_All;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;

    GPIO_Init(GPIOA, &GPIO_InitStructure);
    GPIO_Init(GPIOB, &GPIO_InitStructure);
    GPIO_Init(GPIOC, &GPIO_InitStructure);

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    printf("Standby Mode Test\r\n");

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_PWR, ENABLE);

    if(PWR_GetFlagStatus(PWR_FLAG_WU) == SET)
    {
        printf("\r\n Standby wake up reset \r\n");
    }
    else
    {
        printf("\r\n Power on reset \r\n");
        PWR_WakeUpPinCmd(ENABLE);
        PWR_EnterSTANDBYMode();
    }

    printf("\r\n ########## \r\n");
    while(1)
    {
        Delay_Ms(1000);
        printf("Run in main\r\n");
    }
}
