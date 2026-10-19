/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.2
 * Date               : 2026/10/15
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 * IAP routine: this routine support USB and UART mode,
 * and you can choose the command method or the IO method to jump to the APP .
 * Key  parameters: CalAddr - address in flash (same in APP), note that this address needs to be unused.
 *                  CheckNum - The value of 'CalAddr' that needs to be modified.
 * Tips :the routine need IAP software version 1.50.
 */

#include "debug.h"
#include "string.h"
#include "iap.h"
#include "ch32v10x_usbfs_device.h"
extern vu8 End_Flag;

#define UPGRADE_MODE_COMMAND   0
#define UPGRADE_MODE_IO        1

#define UPGRADE_MODE   UPGRADE_MODE_COMMAND

/*********************************************************************
 * @fn      IAP_2_APP
 *
 * @brief   IAP_2_APP program.
 *
 * @return  none
 */
void IAP_2_APP(void)
{
    R8_USB_CTRL&=~RB_UC_DEV_PU_EN;
    R8_USB_CTRL|=RB_UC_CLR_ALL|RB_UC_RESET_SIE;
    NVIC_DisableIRQ( USBFS_IRQn );
    USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5, DISABLE);
    Delay_Ms(50);
    printf("jump APP\r\n");
    GPIO_DeInit( GPIOA);
    GPIO_DeInit( GPIOB);
    USART_DeInit(USART3);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, DISABLE);
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOB,DISABLE);
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3,DISABLE);
    Delay_Ms(10);
    NVIC_EnableIRQ(Software_IRQn);
    NVIC_SetPendingIRQ(Software_IRQn);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void) {
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%u\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("IAP\r\n");

#if UPGRADE_MODE == UPGRADE_MODE_COMMAND
    if(*(uint32_t*)FLASH_Base  != 0xFFFFFFFF )
    {
        if(*(uint32_t*)CalAddr != CheckNum)
        {
            IAP_2_APP();
            while(1)
            {
            }
        }
    }
#elif UPGRADE_MODE == UPGRADE_MODE_IO
    if(PA0_Check() == 0)
    {
        IAP_2_APP();
        while(1)
        {
        }
    }
#endif

    USART3_CFG(460800);
    pEP0_RAM_Addr = EP0_Databuf;
    pEP1_RAM_Addr = EP1_Databuf;
    pEP2_RAM_Addr = EP2_Databuf;
    pEP3_RAM_Addr = EP3_Databuf;
    pEP4_RAM_Addr = EP4_Databuf;
    pEP5_RAM_Addr = EP5_Databuf;
    pEP6_RAM_Addr = EP6_Databuf;
    pEP7_RAM_Addr = EP7_Databuf;

    USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5, ENABLE);
    UINT8 VDD_Voltage=PWR_VDD_SupplyVoltage();
    if( VDD_Voltage == PWR_VDD_5V)
        {
            EXTEN->EXTEN_CTR |= EXTEN_USB_5V_SEL;
        }
        else
        {
            EXTEN->EXTEN_CTR &= ~EXTEN_USB_5V_SEL;
        }
    USB_DeviceInit();
    NVIC_EnableIRQ(USBFS_IRQn);

    while(1)
    {
        if( USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET){
            UART_Rx_Deal();
        }
#if UPGRADE_MODE == UPGRADE_MODE_COMMAND
        if (End_Flag)
         {
            IAP_2_APP();
            while(1)
            {
            }
         }
#endif
    }

}
