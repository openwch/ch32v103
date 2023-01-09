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
 Independent watchdog routine:
 This routine demonstrates the pull-down input of the PA0 pin. When the input is high,
 it will feed the dog. If you don't feed the dog,Then trigger independent watchdog
 reset after 3.2s.

*/

#include "debug.h"

#define KEY0    GPIO_ReadInputDataBit(GPIOA, GPIO_Pin_0) //PA0

/*********************************************************************
 * @fn      KEY_Init
 *
 * @brief   Initializes KEY GPIO.
 *
 * @return  none
 */
void KEY_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      KEY_PRESS
 *
 * @brief   Key processing funcation.
 *
 * @return  0 - Press the key.
 *          1 - Release Key.
 */
u8 KEY_PRESS(void)
{
    if(KEY0 == 1)
    {
        Delay_Ms(10);
        return 1;
    }

    return 0;
}

/*********************************************************************
 * @fn      IWDG_Init
 *
 * @brief   Initializes IWDG.
 *
 * @param   IWDG_Prescaler: specifies the IWDG Prescaler value.
 *            IWDG_Prescaler_4: IWDG prescaler set to 4.
 *            IWDG_Prescaler_8: IWDG prescaler set to 8.
 *            IWDG_Prescaler_16: IWDG prescaler set to 16.
 *            IWDG_Prescaler_32: IWDG prescaler set to 32.
 *            IWDG_Prescaler_64: IWDG prescaler set to 64.
 *            IWDG_Prescaler_128: IWDG prescaler set to 128.
 *            IWDG_Prescaler_256: IWDG prescaler set to 256.
 *          Reload: specifies the IWDG Reload value.
 *            This parameter must be a number between 0 and 0x0FFF.
 *
 * @return  none
 */
void IWDG_Feed_Init(u16 prer, u16 rlr)
{
    IWDG_WriteAccessCmd(IWDG_WriteAccess_Enable);
    IWDG_SetPrescaler(prer);
    IWDG_SetReload(rlr);
    IWDG_ReloadCounter();
    IWDG_Enable();
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
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    Delay_Init();
    KEY_Init();
    printf("IWDG test...\r\n");
    Delay_Ms(1000);

    IWDG_Feed_Init(IWDG_Prescaler_32, 4000); // 3.2s IWDG reset

    while(1)
    {
        if(KEY_PRESS() == 1) //PA0
        {
            printf("Feed dog\r\n");
            IWDG_ReloadCounter(); //Feed dog
            Delay_Ms(10);
        }
    }
}
