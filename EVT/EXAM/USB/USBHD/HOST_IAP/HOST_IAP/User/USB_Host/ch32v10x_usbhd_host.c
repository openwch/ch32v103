/********************************* (C) COPYRIGHT  *******************************
 * File Name          : ch32v10x_usbhd_host.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/09/01
 * Description        :
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


/*******************************************************************************/
/* Header File */
#include "usb_host_config.h"

/*******************************************************************************/
/* Variable Definition */
__attribute__((aligned(4))) uint8_t  RxBuffer[ MAX_PACKET_SIZE ];           // IN, must even address
__attribute__((aligned(4))) uint8_t  TxBuffer[ MAX_PACKET_SIZE ];           // OUT, must even address

/*********************************************************************
 * @fn      USBHD_RCC_Init
 *
 * @brief   Set USB clock.
 *
 * @para    RCC_USBCLKSource: USB clock source.
 *          NewState: Desired state.
 *
 * @return  none
 */
void USBHD_RCC_Init( void )
{
    RCC_APB2PeriphClockCmd( RCC_APB2Periph_GPIOA, ENABLE );
    EXTEN->EXTEN_CTR |= EXTEN_USBHD_IO_EN;
    if( SystemCoreClock == 72000000 ) 
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_1Div5 );
    }
    else if( SystemCoreClock == 48000000 ) 
    {
        RCC_USBCLKConfig( RCC_USBCLKSource_PLLCLK_Div1 );
    }
    RCC_AHBPeriphClockCmd( RCC_AHBPeriph_USBHD, ENABLE );
}

/*********************************************************************
 * @fn      USBHD_Host_Init
 *
 * @brief   Initialize USB host.
 *
 * @return  none
 */
void USBHD_Host_Init( FunctionalState sta , PWR_VDD VDD_Voltage)
{
    if( VDD_Voltage == PWR_VDD_5V)
    {
        EXTEN->EXTEN_CTR |= EXTEN_USB_5V_SEL;
    }
    else
    {
        EXTEN->EXTEN_CTR &= ~EXTEN_USB_5V_SEL;
    }

    if( sta )
    {
        R8_USB_CTRL = RB_UC_HOST_MODE;
        R8_UHOST_CTRL = 0x00;
        R8_USB_DEV_AD = 0x00;
        R8_UH_EP_MOD = RB_UH_EP_TX_EN | RB_UH_EP_RX_EN;
        R16_UH_RX_DMA = (uint16_t)(uint32_t)USBHD_RX_Buf;
        R16_UH_TX_DMA = (uint16_t)(uint32_t)USBHD_TX_Buf;

        R8_UH_RX_CTRL = 0x00;
        R8_UH_TX_CTRL = 0x00;
        R8_USB_CTRL = RB_UC_HOST_MODE | RB_UC_INT_BUSY | RB_UC_DMA_EN;
        R8_UH_SETUP = RB_UH_SOF_EN;
        R8_USB_INT_FG = 0xFF;
        R8_USB_INT_EN = RB_UIE_TRANSFER | RB_UIE_DETECT;
    }
    else
    {
        R8_USB_CTRL = RB_UC_RESET_SIE | RB_UC_CLR_ALL;
    }
}

/*********************************************************************
 * @fn      USBH_Detect
 *
 * @brief   Detecting equipment plugging and unplugging On Fs Port
 *
 * @return  none
 */
uint8_t USBHDH_CheckRootHubPortStatus( uint8_t dev_sta )
{
    /* Detect USB devices plugged or unplugged */
    if( R8_USB_INT_FG & RB_UIF_DETECT )
    {
        R8_USB_INT_FG = RB_UIF_DETECT; // Clear flag
        if( R8_USB_MIS_ST & RB_UMS_DEV_ATTACH ) // Detect that the USB device has been connected to the port
        {
            if( ( dev_sta == ROOT_DEV_DISCONNECT ) || ( ( dev_sta != ROOT_DEV_FAILED ) && ( USBHDH_CheckRootHubPortEnable( ) == 0x00 ) ) )
            {
                return ROOT_DEV_CONNECTED;
            }
            else
            {
                return ROOT_DEV_FAILED;
            }
        }
        else
        {
            return ROOT_DEV_DISCONNECT;
        }
    }
    else
    {
        return ROOT_DEV_FAILED;
    }
}

/*********************************************************************
 * @fn      USBHDH_CheckPortStatus
 *
 * @brief   Check the USB host port status.
 *
 * @return  Enable or disable
 */
uint8_t USBHDH_CheckRootHubPortEnable( void )
{
    return ( R8_UHOST_CTRL & RB_UH_PORT_EN );
}

/*********************************************************************
 * @fn      USBHDH_CheckDevSpeed
 *
 * @brief   Check the USB host port speed.
 *
 * @return  Low speed or full speed
 */
uint8_t USBHDH_CheckRootHubPortSpeed( void )
{
    return ( R8_USB_MIS_ST & RB_UMS_DM_LEVEL? USB_LOW_SPEED : USB_FULL_SPEED );
}

/*********************************************************************
 * @fn      USBH_SetSelfAddr
 *
 * @brief   Set the USB device address.
 *
 * @para    addr: USB device address.
 *
 * @return  none
 */
void USBHDH_SetSelfAddr( uint8_t addr )
{
    R8_USB_DEV_AD = ( R8_USB_DEV_AD & RB_UDA_GP_BIT ) | ( addr & MASK_USB_ADDR );
}

/*********************************************************************
 * @fn      USBH_SetSelfSpeed
 *
 * @brief   Set USB speed.
 *
 * @para    speed: USB speed.
 *
 * @return  none
 */
void USBHDH_SetSelfSpeed( uint8_t speed )
{
    if( speed )
    {
        /* Full speed */
        R8_USB_CTRL &= ~RB_UC_LOW_SPEED;
        R8_UH_SETUP &= ~RB_UH_PRE_PID_EN;
    }
    else
    {
        /* Low speed */
        R8_USB_CTRL |= RB_UC_LOW_SPEED;
    }
}

/*********************************************************************
 * @fn      USBH_ResetRootHubPort
 *
 * @brief   Reset USB host port.
 *
 * @para    index: USB host port.
 *          mod: Reset host port operating mode.
 *               0=reset and wait end, 1=begin reset, 2=end reset
 *
 * @return  none
 */
void USBHDH_ResetRootHubPort( uint8_t mode )
{
    USBHDH_SetSelfAddr( 0x00 );
    USBHDH_SetSelfSpeed( 1 );
    if( mode <= 1 )
    {
        R8_UHOST_CTRL = ( R8_UHOST_CTRL & ~RB_UH_LOW_SPEED ) | RB_UH_BUS_RESET;
    }
    if( mode == 0 )
    {
        Delay_Ms( DEF_BUS_RESET_TIME ); // Reset time from 10mS to 20mS
    }
    if( mode != 1 )
    {
        R8_UHOST_CTRL = R8_UHOST_CTRL & ~RB_UH_BUS_RESET;
    }
    Delay_Ms( 2 );
    R8_USB_INT_FG = RB_UIF_DETECT;
}

/*********************************************************************
 * @fn      USBH_EnableRootHubPort
 *
 * @brief   Enable host port.
 *
 * @para    index: USB host port.
 *
 * @return  none
 */
uint8_t USBHDH_EnableRootHubPort( uint8_t *pspeed )
{
    if( R8_USB_MIS_ST & RB_UMS_DEV_ATTACH )
    {
        if( USBHDH_CheckRootHubPortEnable( ) == 0x00 )
        {
            *pspeed = USBHDH_CheckRootHubPortSpeed( );
            if( *pspeed == 0 )
            {
                R8_UHOST_CTRL |= RB_UH_LOW_SPEED;
            }
        }
        R8_UHOST_CTRL |= RB_UH_PORT_EN; // Enable port

        return ERR_SUCCESS;
    }

    return ERR_USB_DISCON;
}

/*********************************************************************
 * @fn      USBH_Transact
 *
 * @brief   Perform USB transaction.
 *
 * @para    endp_pid: Endpoint number.
 *          tog:
 *          timeout: Timeout time.
 *
 * @return  none
 */
uint8_t USBHDH_Transact( uint8_t endp_pid, uint8_t endp_tog, uint32_t timeout )
{
    uint8_t   r, trans_rerty;
    uint16_t  i;

    R8_UH_RX_CTRL = R8_UH_TX_CTRL = endp_tog;
    trans_rerty = 0;
    do
    {
        R8_UH_EP_PID = endp_pid; // Specify token PID and endpoint number
        R8_USB_INT_FG = RB_UIF_TRANSFER; // Allow transmission
        for( i = DEF_WAIT_USB_TOUT_200US; ( i != 0 ) && ( ( R8_USB_INT_FG & RB_UIF_TRANSFER ) == 0 ); i-- )
        {
            Delay_Us( 40 );
        }
        R8_UH_EP_PID = 0x00; // Stop USB transfer
        if( ( R8_USB_INT_FG & RB_UIF_TRANSFER ) == 0 )
        {
            return ERR_USB_UNKNOWN;
        }
        else
        {
            /* Complete transfer */
            if( R8_USB_INT_ST & RB_UIS_TOG_OK )
            {
                return ERR_SUCCESS;
            }
            r = R8_USB_INT_ST & MASK_UIS_H_RES; // USB device answer status
            if( r == USB_PID_STALL )
            {
                return ( r | ERR_USB_TRANSFER );
            }
            if( r == USB_PID_NAK )
            {
                if( timeout == 0 )
                {
                    return ( r | ERR_USB_TRANSFER );
                }
                if( timeout < 0xFFFF )
                {
                    timeout--;
                }
                --trans_rerty;
            }
            else switch ( endp_pid >> 4 )
            {
                case USB_PID_SETUP:
                case USB_PID_OUT:
                    if( r )
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break; 
                case USB_PID_IN:
                    if( ( r == USB_PID_DATA0 ) && ( r == USB_PID_DATA1 ) ) 
                    {  
                        ;
                    }  
                    else if( r ) 
                    {
                        return ( r | ERR_USB_TRANSFER );
                    }
                    break;
                default:
                    return ERR_USB_UNKNOWN;
            }
        }
        Delay_Us( 15 );
        if( R8_USB_INT_ST & RB_UIF_DETECT )
        {
            Delay_Us( 200 );
            if( USBHDH_CheckRootHubPortEnable( ) == 0 )
            {
                return ERR_USB_DISCON; // USB device disconnect event
            }
        }
    }while( ++trans_rerty < 10 );

    return ERR_USB_TRANSFER;  // Reply timeout
}

/*********************************************************************
 * @fn      USBH_CtrlTransfer
 *
 * @brief
 *
 * @return  none
 */
uint8_t USBHDH_CtrlTransfer( uint8_t ep0_size, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;
    uint16_t rem_len, rx_len, rx_cnt, tx_cnt;
    uint16_t *Plen;
    Plen = plen;
    s = 0;

    if( Plen )
    {
        *Plen = 0;
    }

    R8_UH_TX_LEN = sizeof( USB_SETUP_REQ );
    s = USBHDH_Transact( USB_PID_SETUP << 4 | 0x00, 0x00, DEF_CTRL_TRANS_TIMEOVER_CNT );  // SETUP stage, 200mS timeout
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    R8_UH_RX_CTRL = R8_UH_TX_CTRL = RB_UH_R_TOG | RB_UH_R_AUTO_TOG | RB_UH_T_TOG | RB_UH_T_AUTO_TOG;  // Default DATA1
    R8_UH_TX_LEN = 0x01; // The default no-data state stage is IN
    rem_len = pUSBHD_SetupRequest->wLength;
    if( rem_len && pbuf )
    {
        if( pUSBHD_SetupRequest->bRequestType & USB_REQ_TYP_IN )
        {
            /* Receive data */
            while( rem_len )
            {
                Delay_Us( 100 );
                s = USBHDH_Transact( USB_PID_IN << 4 | 0x00, R8_UH_RX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT );  // IN
                if( s != ERR_SUCCESS )
                {
                    return s;
                }
                rx_len = R16_USB_RX_LEN < rem_len ? R16_USB_RX_LEN : rem_len;
                rem_len -= rx_len;
                if( Plen )
                {
                    *Plen += rx_len; // The total length of the actual successful transmission and reception
                }
                for( rx_cnt = 0; rx_cnt != rx_len; rx_cnt++ )
                {
                    *pbuf = USBHD_RX_Buf[ rx_cnt ];
                    pbuf++;
                }

                if( R16_USB_RX_LEN == 0 || ( R16_USB_RX_LEN & ( ep0_size - 1 ) ) )
                {
                    break; // Short package
                }
            }
            R8_UH_TX_LEN = 0x00; // Status stage is OUT
        }
        else
        {
            /* Send data */
            while( rem_len )
            {
                Delay_Us( 100 );
                R8_UH_TX_LEN = rem_len >= ep0_size ? ep0_size : rem_len;
                for( tx_cnt = 0; tx_cnt != R8_UH_TX_LEN; tx_cnt++ )
                {
                    USBHD_TX_Buf[ tx_cnt ] = *pbuf;
                    pbuf++;
                }
                s = USBHDH_Transact( USB_PID_OUT << 4 | 0x00, R8_UH_TX_CTRL, DEF_CTRL_TRANS_TIMEOVER_CNT ); // OUT
                if( s != ERR_SUCCESS )
                {
                    return( s );
                }
                rem_len -= R8_UH_TX_LEN;
                if( Plen )
                {
                    *Plen += R8_UH_TX_LEN; // The total length of the actual successful transmission and reception
                }
            }
        }
    }
    Delay_Us( 100 );
    s = USBHDH_Transact( ( R8_UH_TX_LEN ? USB_PID_IN << 4 | 0x00: USB_PID_OUT << 4 | 0x00 ), RB_UH_R_TOG | RB_UH_T_TOG, DEF_CTRL_TRANS_TIMEOVER_CNT );  /* STATUS�׶� */
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    if( R8_UH_TX_LEN == 0 )
    {
        return ERR_SUCCESS;
    }
    if( R16_USB_RX_LEN == 0 )
    {
        return ERR_SUCCESS;
    }
    return ERR_USB_BUF_OVER;
}

/*********************************************************************
 * @fn      USBH_GetDeviceDescr
 *
 * @brief
 *
 * @para
 *
 * @return  none
 */
uint8_t USBHDH_GetDeviceDescr( uint8_t *pep0_size, uint8_t *pbuf )
{
    uint8_t  s;
    uint16_t len;

    *pep0_size = DEFAULT_ENDP0_SIZE;
    memcpy( pUSBHD_SetupRequest, SetupGetDevDesc, sizeof( USB_SETUP_REQ ) );
    s = USBHDH_CtrlTransfer( *pep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    *pep0_size = ( (PUSB_DEV_DESCR)pbuf )->bMaxPacketSize0;
    if( len < ( (PUSB_SETUP_REQ)SetupGetDevDesc )->wLength )
    {
        return ERR_USB_BUF_OVER;
    }
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBH_GetConfigDescr
 *
 * @brief
 *
 * @para
 *
 * @return  none
 */
uint8_t USBHDH_GetConfigDescr( uint8_t ep0_size, uint8_t *pbuf, uint16_t buf_len, uint16_t *pcfg_len )
{
    uint8_t  s;

    /* Get the string descriptor of the first 4 bytes */
    memcpy( pUSBHD_SetupRequest, SetupGetCfgDesc, sizeof( USB_SETUP_REQ ) );
    s = USBHDH_CtrlTransfer( ep0_size, pbuf, pcfg_len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    if( *pcfg_len < ( (PUSB_SETUP_REQ)SetupGetCfgDesc )->wLength )
    {
        return ERR_USB_BUF_OVER;
    }

    /* Get the complete string descriptor */
    *pcfg_len = ((PUSB_CFG_DESCR)pbuf)->wTotalLength;
    if( *pcfg_len > buf_len  )
    {
        *pcfg_len = buf_len;
    }
    memcpy( pUSBHD_SetupRequest, SetupGetCfgDesc, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wLength = *pcfg_len;
    s = USBHDH_CtrlTransfer( ep0_size, pbuf, pcfg_len );
    return s;
}

/*********************************************************************
 * @fn      USBH_GetStrDescr
 *
 * @brief
 *
 * @para
 *
 * @return  none
 */
uint8_t USBHDH_GetStrDescr( uint8_t ep0_size, uint8_t str_num, uint8_t *pbuf )
{
    uint8_t  s;
    uint16_t len;

    /* Get the string descriptor of the first 4 bytes */
    memcpy( pUSBHD_SetupRequest, SetupGetStrDesc, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wValue = ( (uint16_t)USB_DESCR_TYP_STRING << 8 ) | str_num;
    s = USBHDH_CtrlTransfer( ep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }

    /* Get the complete string descriptor */
    len = pbuf[ 0 ];
    memcpy( pUSBHD_SetupRequest, SetupGetStrDesc, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wValue = ( (uint16_t)USB_DESCR_TYP_STRING << 8 ) | str_num;
    pUSBHD_SetupRequest->wLength = len;
    s = USBHDH_CtrlTransfer( ep0_size, pbuf, &len );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBH_SetUsbAddress
 *
 * @brief   Set USB device address.
 *
 * @para    addr: Device address.
 *
 * @return  none
 */
uint8_t USBHDH_SetUsbAddress( uint8_t ep0_size, uint8_t addr )
{
    uint8_t  s;

    memcpy( pUSBHD_SetupRequest, SetupSetAddr, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wValue = (uint16_t)addr;
    s = USBHDH_CtrlTransfer( ep0_size, NULL, NULL );
    if( s != ERR_SUCCESS )
    {
        return s;
    }
    USBHDH_SetSelfAddr( addr );
    Delay_Ms( DEF_BUS_RESET_TIME >> 1 ); // Wait for the USB device to complete its operation.
    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      USBH_SetUsbConfig
 *
 * @brief   Set USB configuration.
 *
 * @para    cfg: Configuration value.
 *
 * @return  none
 */
uint8_t USBHDH_SetUsbConfig( uint8_t ep0_size, uint8_t cfg_val )
{
    memcpy( pUSBHD_SetupRequest, SetupSetConfig, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wValue = (uint16_t)cfg_val;
    return USBHDH_CtrlTransfer( ep0_size, NULL, NULL );
}

/*********************************************************************
 * @fn      USBH_ClearEndpStall
 *
 * @brief   Clear endpoint stall.
 *
 * @para    endp: Endpoint number.
 *
 * @return  none
 */
uint8_t USBHDH_ClearEndpStall( uint8_t ep0_size, uint8_t endp_num )
{
    memcpy( pUSBHD_SetupRequest, SetupClearEndpStall, sizeof( USB_SETUP_REQ ) );
    pUSBHD_SetupRequest->wIndex = (uint16_t)endp_num;
    return USBHDH_CtrlTransfer( ep0_size, NULL, NULL );
}

/*********************************************************************
 * @fn      USBFSH_GetEndpData
 *
 * @brief   Detecting equipment plugging and unplugging On Fs Port
 *
 * @return  none
 */
uint8_t USBHDH_GetEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t *plen )
{
    uint8_t  s;
    
    s = USBHDH_Transact( ( USB_PID_IN << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *pendp_tog ^= RB_UH_R_TOG | RB_UH_T_TOG;
        *plen = R16_USB_RX_LEN;
        memcpy( pbuf, USBHD_RX_Buf, *plen );
    }
    
    return s;
}

/*********************************************************************
 * @fn      USBHDH_SendEndpData
 *
 * @brief   Clear endpoint stall.
 *
 * @para    endp: Endpoint number.
 *
 * @return  none
 */
uint8_t USBHDH_SendEndpData( uint8_t endp_num, uint8_t *pendp_tog, uint8_t *pbuf, uint16_t len )
{
    uint8_t  s;
    
    memcpy( USBHD_TX_Buf, pbuf, len );
    R8_UH_TX_LEN = len;
    
    s = USBHDH_Transact( ( USB_PID_OUT << 4 ) | endp_num, *pendp_tog, 0 );
    if( s == ERR_SUCCESS )
    {
        *pendp_tog ^= RB_UH_R_TOG | RB_UH_T_TOG;
    }
  
    return s;
}
