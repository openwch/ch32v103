/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.2
* Date               : 2026/10/15
* Description        : Main program body.
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * APP routine: this routine support USB and UART mode,
 * and you can choose the command method to jump to the IAP .
 * Key  parameters: CalAddr - address in flash(same in IAP), note that this address needs to be unused.
 *                  CheckNum - The value of 'CalAddr' that needs to be modified.
 * Tips :the routine need IAP software version 1.50.
 */

#include "debug.h"
#include "string.h"
#include "iap.h"
#include "ch32v10x_usbfs_device.h"

/*********************************************************************
 * @fn      GPIO_Toggle_INIT
 *
 * @brief   Initializes GPIOA.2
 *
 * @return  none
 */
void GPIO_Toggle_INIT(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}

/*********************************************************************
 * @fn      APP_2_IAP
 *
 * @brief   APP_2_IAP program.
 *
 * @return  none
 */
void APP_2_IAP(void)
{
    NVIC_SystemReset();
    while(1)
    {
    }
}

/*********************************************************************
 * @fn      USART3_IT_CFG
 *
 * @brief   USART3 IT configuration.
 *
 * @return  none
 */
void USART3_IT_CFG(void)
{
    NVIC_InitTypeDef  NVIC_InitStructure = {0};

    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
    USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    USART_Cmd(USART3, ENABLE);
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
	u8 i =0;
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
    printf("SystemClk:%u\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("APP\r\n");
	GPIO_Toggle_INIT();

	//USB-USART ENABLE
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
    USART3_IT_CFG();
	while(1)
	{
        Delay_Ms(250);
        GPIO_WriteBit(GPIOA, GPIO_Pin_2, (i == 0) ? (i = Bit_SET) : (i = Bit_RESET));
        if(*(uint32_t*)CalAddr == CheckNum)
        {
             Delay_Ms(10);
             APP_2_IAP();
             while(1)
             {
             }
        }
	}
}
void USART3_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
/*********************************************************************
 * @fn      USART3_IRQHandler
 *
 * @brief   This function handles USART3 global interrupt request.
 *
 * @return  none
 */
void USART3_IRQHandler(void)
{
    if( USART_GetFlagStatus(USART3, USART_FLAG_RXNE) != RESET){

        UART_Rx_Deal();
    }
}


