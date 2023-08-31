/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2019/10/15
* Description        : Main program body.
********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 * @Note
 * This example demonstrates the enumeration process of a USB host to a device that
 * supports MTP and PTP protocols, and reads its files.
*/

/*
 * @Note
 * The USBHD module uses the system clock as the clock source, so the SystemCoreClock can
 * only be set to 72MHz or 48MHz.
 */

/*******************************************************************************/
/* Header File */
#include "usb_host_config.h"

/*********************************************************************
 * @fn      main
 *
 * @brief   Main function.
 *
 * @return  none
 */
int main( void )
{
    /* Initialize system configuration */
    Delay_Init( );
    USART_Printf_Init( 115200 );
    DUG_PRINTF( "SystemClk:%d\r\n", SystemCoreClock );
    DUG_PRINTF( "USBHD HOST KM Test\r\n" );

    /* Configure USB clock and initialize USB host */
#if DEF_USBHD_PORT_EN
    USBHD_RCC_Init( );
    USBHD_Host_Init( ENABLE , PWR_VDD_SupplyVoltage());
    memset( &RootHubDev.bStatus, 0, sizeof( ROOT_HUB_DEVICE ) );
    memset( &HostCtl[ DEF_TOTAL_ROOT_HUB * DEF_ONE_USB_SUP_DEV_TOTAL ].InterfaceNum, 0, DEF_ONE_USB_SUP_DEV_TOTAL * sizeof( HOST_CTL ) );
#endif
    
    while( 1 )
    {
        USBH_MainDeal( );
    }
}
