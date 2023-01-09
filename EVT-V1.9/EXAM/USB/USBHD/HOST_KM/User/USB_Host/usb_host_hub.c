/********************************** (C) COPYRIGHT  *******************************
 * File Name          : usb_host_hub.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/29
 * Description        : 
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
 
 
/********************************************************************************/
/* Header File */
#include "usb_host_config.h"

/*********************************************************************
 * @fn      HUB_ClearPortFeature
 *
 * @brief
 *
 * @para
 *
 * @return  none
 */
uint8_t HUB_GetPortStatus( uint8_t hub_ep0_size, uint8_t hub_port, uint8_t *pbuf )
{
    uint16_t len;
    
    memcpy( pUSBHD_SetupRequest, GetPortStatus, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wIndex = (uint16_t)hub_port;
    return USBHDH_CtrlTransfer( hub_ep0_size, pbuf, &len );
}

/*********************************************************************
 * @fn      HUB_ClearPortFeature
 *
 * @brief
 *
 * @para
 *
 * @return  none
 */
uint8_t HUB_ClearPortFeature( uint8_t hub_ep0_size, uint8_t hub_port, uint8_t selector )
{
    memcpy( pUSBHD_SetupRequest, ClearPortFeature, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wValue = (uint16_t)selector;
    pUSBHD_SetupRequest->wIndex = (uint16_t)hub_port;
    return USBHDH_CtrlTransfer( hub_ep0_size, NULL, NULL );
}

/*********************************************************************
 * @fn      HUB_SetPortFeature
 *
 * @brief
 *
 * @para
 *
 * @return  none
 */
uint8_t HUB_SetPortFeature( uint8_t hub_ep0_size, uint8_t hub_port, uint8_t selector )
{
    memcpy( pUSBHD_SetupRequest, SetPortFeature, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wValue = (uint16_t)selector;
    pUSBHD_SetupRequest->wIndex = (uint16_t)hub_port;
    return USBHDH_CtrlTransfer( hub_ep0_size, NULL, NULL );
}

/*********************************************************************
 * @fn      HUB_GetClassDevDescr
 *
 * @brief
 *
 * @para
 *
 * @return  none
 */
uint8_t HUB_GetClassDevDescr( uint8_t hub_ep0_size, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t s;
    
    memcpy( pUSBHD_SetupRequest, GetHubDescr, sizeof( USB_SETUP_REQ ) );
    s = USBHDH_CtrlTransfer( hub_ep0_size, pbuf, plen );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    else
    {
        pUSBHD_SetupRequest->wLength = *plen = (uint16_t)pbuf[ 0 ];
        s = USBHDH_CtrlTransfer( hub_ep0_size, pbuf, plen );
    }

    return s;
}
