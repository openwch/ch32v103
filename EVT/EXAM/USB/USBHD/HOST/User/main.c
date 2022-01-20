/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 *@Note
  USB设备的简易枚举过程例程：
  USBHDM(PA11)、USBHDP(PA12)。

*/

#include "debug.h"
#include "string.h"

/* Global define */

/* Global Variable */
__attribute__((aligned(4))) UINT8 RxBuffer[MAX_PACKET_SIZE]; // IN, must even address
__attribute__((aligned(4))) UINT8 TxBuffer[MAX_PACKET_SIZE]; // OUT, must even address
__attribute__((aligned(4))) UINT8 Com_Buffer[128];

/*********************************************************************
 * @fn      Set_USBConfig
 *
 * @brief   Set USB clock.
 *
 * @return  none
 */
void USBHD_ClockCmd(UINT32 RCC_USBCLKSource, FunctionalState NewState)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, NewState);
    EXTEN->EXTEN_CTR |= EXTEN_USBHD_IO_EN;
    RCC_USBCLKConfig(RCC_USBCLKSource); //USBclk=PLLclk/1.5=48Mhz
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHD, NewState);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
    UINT8 s;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    printf("USBHD   HOST Test\r\n");
    USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5, ENABLE);
    pHOST_RX_RAM_Addr = RxBuffer;
    pHOST_TX_RAM_Addr = TxBuffer;
    USB_HostInit();
    printf("Wait Device In\r\n");

    while(1)
    {
        s = ERR_SUCCESS;

        if(R8_USB_INT_FG & RB_UIF_DETECT)
        {
            R8_USB_INT_FG = RB_UIF_DETECT;

            s = AnalyzeRootHub();
            if(s == ERR_USB_CONNECT)
            {
                printf("New Device In\r\n");
                FoundNewDev = 1;
            }
            if(s == ERR_USB_DISCON)
            {
                printf("Device Out\r\n");
            }
        }

        if(FoundNewDev || s == ERR_USB_CONNECT)
        {
            FoundNewDev = 0;
            Delay_Ms(200);
            s = InitRootDevice(Com_Buffer);
            if(s == ERR_SUCCESS)
                printf("Device Enum Succeed\r\n");
            else
                printf("Device Enum Failed\r\n");
        }
    }
}
