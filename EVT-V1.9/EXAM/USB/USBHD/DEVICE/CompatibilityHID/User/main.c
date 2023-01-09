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

/* @Note
 * Compatibility HID Example:
 * This program provides examples of the pass-through of USB-HID data and serial port
 *  data based on compatibility HID device. And the data returned by Get_Report request is
 *  the data sent by the last Set_Report request.Speed of UART1/2 is 115200bps.
 *
 * Interrupt Transfers:
 *   UART2_RX   ---> Endpoint2
 *   Endpoint1  ---> UART2_TX
 *
 *   Note that the first byte is the valid data length and the remaining bytes are
 *   the transmission data for interrupt Transfers.
 *
 * Control Transfers:
 *   Set_Report ---> UART1_TX
 *   Get_Report <--- last Set_Report packet
 *
 *  */

#include "debug.h"
#include "string.h"
#include "ch32v10x_usbfs_device.h"
#include "usbd_compatibility_hid.h"

/******************************************************************************/
/* Global define */

/******************************************************************************/
/* Global Variables */

/*********************************************************************
 * @fn      Var_Init
 *
 * @brief   Software parameter initialization
 *
 * @return  none
 */
void Var_Init(void)
{
    uint16_t i;
    RingBuffer_Comm.LoadPtr = 0;
    RingBuffer_Comm.StopFlag = 0;
    RingBuffer_Comm.DealPtr = 0;
    RingBuffer_Comm.RemainPack = 0;
    for(i=0; i<DEF_Ring_Buffer_Max_Blks; i++)
    {
        RingBuffer_Comm.PackLen[i] = 0;
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
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("USBHD Compatibility HID Example\r\n");
    printf("SystemClk:%d\r\n",SystemCoreClock);

    /* Variables init */
    Var_Init();

    /* UART2 init */
    UART2_Init();
    UART2_DMA_Init();

    /* Usb init */
    USBHD_RCC_Init();
    USBHD_Device_Init(ENABLE);
    
    /* Timer init */
    TIM2_Init();

    while(1)
    {
        if(USBHD_DevEnumStatus)
        {
            UART2_Rx_Service();
            UART2_Tx_Service();
            HID_Set_Report_Deal();
        }
    }

}
