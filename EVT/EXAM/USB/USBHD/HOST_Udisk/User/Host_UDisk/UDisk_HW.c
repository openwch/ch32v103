/********************************** (C) COPYRIGHT *******************************
* File Name          : UDisk_HW.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2022/09/01
* Description        : USB full-speed port host operation functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*******************************************************************************/
/* Header File */
#include "Udisk_Operation.h"

/*******************************************************************************/
/* Variable Definition */
__attribute__((aligned(4)))  uint8_t  Com_Buffer[ DEF_COM_BUF_LEN ];     // even address , used for host enumcation and udisk operation
__attribute__((aligned(4)))  uint8_t  DevDesc_Buf[ 18 ];                 // Device Descriptor Buffer
struct   _ROOT_HUB_DEVICE RootHubDev;
volatile uint8_t  UDisk_Opeation_Flag = 0;

/* For Udisk Lib */
uint8_t *pHOST_RX_RAM_Addr;
uint8_t *pHOST_TX_RAM_Addr;

/*********************************************************************
 * @fn      mStopIfError
 *
 * @brief   Checking the operation status, displaying the error code and stopping if there is an error
 *          input : iError - Error code input
 *
 * @return  none
 */
void mStopIfError( uint8_t iError )
{
    if ( iError == ERR_SUCCESS )
    {
        /* operation success, return */
        return;
    }
    /* Display the errors */
    DUG_PRINTF( "Error:%02x\r\n", iError );
    /* After encountering an error, you should analyze the error code and CHRV3DiskStatus status, for example,
     * call CHRV3DiskReady to check whether the current USB disk is connected or not,
     * if the disk is disconnected then wait for the disk to be plugged in again and operate again,
     * Suggested steps to follow after an error:
     *     1, call CHRV3DiskReady once, if successful, then continue the operation, such as Open, Read/Write, etc.
     *     2If CHRV3DiskReady is not successful, then the operation will be forced to start from the beginning.
     */
    while(1)
    {  }
}

/*********************************************************************
 * @fn      Udisk_USBH_Initialization
 *
 * @brief   USB Host Udisk process Initialization, include usb-host initialization
 *          usb libs initialization, udisk-related values Initialization 
            and the others
 *
 * @para    none
 *
 * @return  none
 */
void Udisk_USBH_Initialization( void )
{
	/* USB Host Initialization */
    DUG_PRINTF( "USB Host Initialization. \r\n" );
    USBHD_RCC_Init( );
    pHOST_TX_RAM_Addr = USBHD_TX_Buf;
    pHOST_RX_RAM_Addr = USBHD_RX_Buf;
    USBHD_Host_Init( ENABLE , PWR_VDD_SupplyVoltage());
	
	/* USB Libs Initialization */
    printf( "UDisk library Initialization. \r\n" );
    CHRV3LibInit( );
}

/*********************************************************************
 * @fn      Udisk_USBH_EnumRootDevice
 *
 * @brief   Generally enumerate a device connected to host port.
 *
 * @para    index: USB host port
 *
 * @return  Enumeration result
 */
uint8_t Udisk_USBH_EnumRootDevice( uint8_t usb_port )
{
    uint8_t  s;
    uint8_t  enum_cnt;
    uint8_t  cfg_val;
    uint16_t i;
    uint16_t len;

    DUG_PRINTF( "Enum:\r\n" );

    enum_cnt = 0;
ENUM_START:
    /* Delay and wait for the device to stabilize */
    Delay_Ms( 100 );
    enum_cnt++;
    Delay_Ms( 8 << enum_cnt );

    /* Reset the USB device and wait for the USB device to reconnect */
    USBHDH_ResetRootHubPort( 0 );
    for( i = 0, s = 0; i < DEF_RE_ATTACH_TIMEOUT; i++ )
    {
        if( USBHDH_EnableRootHubPort( &RootHubDev.bSpeed ) == ERR_SUCCESS )
        {
            i = 0;
            s++;
            if( s > 6 )
            {
                break;
            }
        }
        Delay_Ms( 1 );
    }
    if( i )
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return( ERR_USB_DISCON );
    }

    /* Select USB speed */
    USBHDH_SetSelfSpeed( RootHubDev.bSpeed );

    /* Get USB device device descriptor */
    DUG_PRINTF("Get DevDesc: ");
    s = USBHDH_GetDeviceDescr( &RootHubDev.bEp0MaxPks, DevDesc_Buf );
    if( s == ERR_SUCCESS )
    {
        /* Print USB device device descriptor */
#if DEF_DEBUG_PRINTF
        for( i = 0; i < 18; i++ )
        {
            DUG_PRINTF( "%02x ", DevDesc_Buf[ i ] );
        }
        DUG_PRINTF("\n");
#endif
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        DUG_PRINTF( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return DEF_DEV_DESCR_GETFAIL;
    }

    /* Set the USB device address */
    DUG_PRINTF("Set DevAddr: ");
    s = USBHDH_SetUsbAddress( RootHubDev.bEp0MaxPks, USB_DEVICE_ADDR );
    if( s == ERR_SUCCESS )
    {
        DUG_PRINTF( "OK\n" );
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        DUG_PRINTF( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return DEF_DEV_ADDR_SETFAIL;
    }
    Delay_Ms( 5 );

    /* Get the USB device configuration descriptor */
    DUG_PRINTF("Get CfgDesc: ");
    s = USBHDH_GetConfigDescr( RootHubDev.bEp0MaxPks, Com_Buffer, DEF_COM_BUF_LEN, &len );
    if( s == ERR_SUCCESS )
    {
        cfg_val = ( (PUSB_CFG_DESCR)Com_Buffer )->bConfigurationValue;

        /* Print USB device configuration descriptor  */
#if DEF_DEBUG_PRINTF
        for( i = 0; i < len; i++ )
        {
            DUG_PRINTF( "%02x ", Com_Buffer[ i ] );
        }
        DUG_PRINTF("\n");
#endif
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        DUG_PRINTF( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return( DEF_CFG_DESCR_GETFAIL );
    }

    /* Set USB device configuration value */
    DUG_PRINTF("Set Cfg: ");
    s = USBHDH_SetUsbConfig( RootHubDev.bEp0MaxPks, cfg_val );
    if( s == ERR_SUCCESS )
    {
        DUG_PRINTF( "OK\n" );
    }
    else
    {
        /* Determine whether the maximum number of retries has been reached, and retry if not reached */
        DUG_PRINTF( "Err(%02x)\n", s );
        if( enum_cnt <= 5 )
        {
            goto ENUM_START;
        }
        return ERR_USB_UNSUPPORT;
    }

    return ERR_SUCCESS;
}

/*********************************************************************
 * @fn      UDisk_USBH_PreDeal
 *
 * @brief   usb host preemption operations,
  *         including detecting device insertion and enumerating device information
 *
 * @return  none
 */
uint8_t UDisk_USBH_PreDeal( void )
{
    uint8_t ret;
    ret = USBHDH_CheckRootHubPortStatus( RootHubDev.bStatus );
    if( ret == ROOT_DEV_CONNECTED )
    {
        DUG_PRINTF("USB Dev In.\n");
        USBHDH_CheckRootHubPortSpeed( );
        RootHubDev.bStatus = ROOT_DEV_CONNECTED; // Set connection status
        RootHubDev.DeviceIndex = DEF_USB_PORT_HD;

        /* Enumerate root device */
        ret = Udisk_USBH_EnumRootDevice( 0 );
        if( ret == ERR_SUCCESS )
        {
            DUG_PRINTF( "USB Device Enumeration Succeed\r\n" );
            return ERR_SUCCESS;
        }
        else
        {
            DUG_PRINTF( "USB Device Enumeration ERR %02x.\r\n",ret);
            return ERR_USB_UNAVAILABLE;
        }
    }
    else if( ret == ROOT_DEV_DISCONNECT )
    {
        DUG_PRINTF("USB Dev Out.\n");
        /* Clear parameters */
        USBHD_Host_Init( DISABLE , PWR_VDD_SupplyVoltage());
        memset( &RootHubDev.bStatus, 0, sizeof( struct _ROOT_HUB_DEVICE ) );
        UDisk_Opeation_Flag = 1;
        CHRV3DiskStatus = DISK_UNKNOWN;
        return ERR_USB_DISCON;
    }
    return ERR_USB_UNKNOWN;
}

/*********************************************************************
 * @fn      UDisk_USBH_DiskReady
 *
 * @brief   Check if UDisk is Ready and update status in 'CHRV3DiskStatus'
 *
 * @return  none
 */
uint8_t UDisk_USBH_DiskReady( void )
{
    uint8_t  ret, i;

    /* Detect USB Device & Enumeration processing */
    ret = UDisk_USBH_PreDeal( );
    if( ret == ERR_SUCCESS )
    {
        /* Wait for uDisk Ready */
        CHRV3DiskStatus = DISK_USB_ADDR;
        for ( i = 0; i != 10; i ++ )
        {
              DUG_PRINTF( "Wait Disk Ready...\r\n" );
              ret = CHRV3DiskReady( );
              if ( ret == ERR_SUCCESS )
              {
                  /* Disk Ready */
                  DUG_PRINTF( "Disk Ready Code:%02x.\r\n", ret );
                  DUG_PRINTF( "CHRV3DiskStatus:%02x\n", CHRV3DiskStatus);
                  UDisk_Opeation_Flag = 1;
                  return DISK_READY;
              }
              else
              {
                  DUG_PRINTF("Not Ready Code :%02x.\r\n", ret);
                  DUG_PRINTF("CHRV3DiskStatus:%02x.\n", CHRV3DiskStatus);
              }
              Delay_Ms( 50 );
          }
    }
    return CHRV3DiskStatus;
}

