/********************************** (C) COPYRIGHT *******************************
 * File Name          : CH32V103UFI.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2014/09/09
 * Description        : USB-flash File Interface for CHRV3.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 *******************************************************************************/
/* CHRV3 U disk host file system interface, supports: FAT12/FAT16/FAT32 */

//#define DISK_BASE_BUF_LEN		512	/* The default disk data buffer size is512 bytes((selectable as2048 or even4096 to supportU disks with large sectors),if0, defining the buffer in this file is prohibited; the application specifies it inpDISK_BASE_BUF */
/* If the disk data buffer needs to be reused to saveRAM,, thenDISK_BASE_BUF_LEN can be defined as0 to disable defining the buffer in this file,, and the application shall, before callingCHRV3LibInit, put the start address of the shared buffer into thepDISK_BASE_BUF variable */

//#define NO_DEFAULT_ACCESS_SECTOR	    1		/* Disable the default disk sector read/write routines,replace them with self-written routines below */
//#define NO_DEFAULT_DISK_CONNECT		1		/* Disable the default disk connection check routine,replace it with a self-written routine below */
//#define NO_DEFAULT_FILE_ENUMER		1		/* Disable the default file name enumeration callback routine,replace it with a self-written routine below */

#include "debug.h"
#include "CHRV3UFI.h"
#include "ch32v10x_usbfs_host.h"
#include "usb_host_config.h"

UINT8 USBHostTransact( UINT8 endp_pid, UINT8 tog, UINT32 timeout )
{
    uint8_t ret;
    ret = USBFSH_Transact( endp_pid, tog, timeout );
    return ret;
}

UINT8 HostCtrlTransfer( PUINT8 DataBuf, PUINT8 RetLen )
{
    uint8_t  ret;
    uint16_t retlen;
    retlen = (uint16_t)(*RetLen);
    ret = USBFSH_CtrlTransfer( RootHubDev.bEp0MaxPks, DataBuf, &retlen );
    return ret;
}

void CopySetupReqPkg( PCCHAR pReqPkt )
{
    uint8_t i;

    for(i = 0; i != sizeof(USB_SETUP_REQ); i++)
    {
        ((char *)pUSBFS_SetupRequest)[i] = *pReqPkt;
        pReqPkt++;
    }
}

UINT8 CtrlGetDeviceDescrTB( void )
{
    uint8_t ret;
    ret = USBFSH_GetDeviceDescr( &RootHubDev.bEp0MaxPks, TxBuffer );
    return ret;
}

UINT8 CtrlGetConfigDescrTB( void )
{
    uint16_t len;
    uint8_t  ret;
    ret = USBFSH_GetConfigDescr( RootHubDev.bEp0MaxPks, TxBuffer, 256, &len );
    return ret;
}

UINT8 CtrlSetUsbConfig( UINT8 cfg )
{
    uint8_t ret;
    ret = USBFSH_SetUsbConfig( RootHubDev.bEp0MaxPks, cfg );
    return ret;
}

UINT8 CtrlSetUsbAddress( UINT8 addr )
{
    uint8_t ret;
    ret = USBFSH_SetUsbAddress( RootHubDev.bEp0MaxPks, addr );
    return ret;
}

UINT8 CtrlClearEndpStall( UINT8 endp )
{
    uint8_t ret;
    ret = USBFSH_ClearEndpStall( RootHubDev.bEp0MaxPks, endp );
    return ret;
}

#ifndef FOR_ROOT_UDISK_ONLY
UINT8 CtrlGetHubDescr( void )
{

}

UINT8 HubGetPortStatus( UINT8 HubPortIndex )
{

}

UINT8 HubSetPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt )
{

}

UINT8 HubClearPortFeature( UINT8 HubPortIndex, UINT8 FeatureSelt )
{

}
#endif

CMD_PARAM_I	mCmdParam;						/* Command parameters */
#if		DISK_BASE_BUF_LEN > 0
//UINT8	DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((at(BA_RAM+SZ_RAM/2)));	/* ExternalRAM disk data buffer,, with a length of one sector */
UINT8	DISK_BASE_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4)));	/* ExternalRAM disk data buffer,, with a length of one sector */
//UINT8	DISK_FAT_BUF[ DISK_BASE_BUF_LEN ] __attribute__((aligned (4)));	/* ExternalRAM diskFAT data buffer,, with a length of one sector */
#endif

/* The following routines can be modified as needed */
#ifndef	NO_DEFAULT_ACCESS_SECTOR		/* DefineNO_DEFAULT_ACCESS_SECTOR in the application to disable the default disk sector read/write routines,, then replace them with self-written routines */
//if ( use_external_interface ) // Replace theU disk sector low-level read/write routines
//{
//    CHRV3vSectorSize=512;     // Set the actual sector size,it must be a multiple of512,this value is the sector size of the disk
//    CHRV3vSectorSizeB=9;      // Set the shift count of the actual sector size,512 corresponds to9,1024 corresponds to10,2048 corresponds to11
//    CHRV3DiskStatus=DISK_MOUNTED;  // Force the block device connection to succeed(only the file system remains to be analyzed)
//}

UINT8	CHRV3ReadSector( UINT8 SectCount, PUINT8 DataBuf )  /* Read multiple sectors of data from the disk into the buffer */
{
	UINT8	retry;
//	if ( use_external_interface ) return( extReadSector( CHRV3vLbaCurrent, SectCount, DataBuf ) );  /* External interface */
	for( retry = 0; retry < 3; retry ++ ) {  /* Retry on error */
		pCBW -> mCBW_DataLen = (UINT32)SectCount << CHRV3vSectorSizeB;  /* Data transfer length */
		pCBW -> mCBW_Flag = 0x80;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_READ10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (UINT8)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (UINT8)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (UINT8)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (UINT8)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* Execute commands based onBulkOnly protocol */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* Disk operation error */
}

#ifdef	EN_DISK_WRITE
UINT8	CHRV3WriteSector( UINT8 SectCount, PUINT8 DataBuf )  /* Write the multi-sector data block in the buffer to the disk */
{
	UINT8	retry;
//	if ( use_external_interface ) return( extWriteSector( CHRV3vLbaCurrent, SectCount, DataBuf ) );  /* External interface */
	for( retry = 0; retry < 3; retry ++ ) {  /* Retry on error */
		pCBW -> mCBW_DataLen = (UINT32)SectCount << CHRV3vSectorSizeB;  /* Data transfer length */
		pCBW -> mCBW_Flag = 0x00;
		pCBW -> mCBW_LUN = CHRV3vCurrentLun;
		pCBW -> mCBW_CB_Len = 10;
		pCBW -> mCBW_CB_Buf[ 0 ] = SPC_CMD_WRITE10;
		pCBW -> mCBW_CB_Buf[ 1 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 2 ] = (UINT8)( CHRV3vLbaCurrent >> 24 );
		pCBW -> mCBW_CB_Buf[ 3 ] = (UINT8)( CHRV3vLbaCurrent >> 16 );
		pCBW -> mCBW_CB_Buf[ 4 ] = (UINT8)( CHRV3vLbaCurrent >> 8 );
		pCBW -> mCBW_CB_Buf[ 5 ] = (UINT8)( CHRV3vLbaCurrent );
		pCBW -> mCBW_CB_Buf[ 6 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 7 ] = 0x00;
		pCBW -> mCBW_CB_Buf[ 8 ] = SectCount;
		pCBW -> mCBW_CB_Buf[ 9 ] = 0x00;
		CHRV3BulkOnlyCmd( DataBuf );  /* Execute commands based onBulkOnly protocol */
		if ( CHRV3IntStatus == ERR_SUCCESS ) {
			Delay_Us( 200 );  /* Delay after write operation */
			return( ERR_SUCCESS );
		}
		CHRV3IntStatus = CHRV3AnalyzeError( retry );
		if ( CHRV3IntStatus != ERR_SUCCESS ) {
			return( CHRV3IntStatus );
		}
	}
	return( CHRV3IntStatus = ERR_USB_DISK_ERR );  /* Disk operation error */
}
#endif
#endif  // NO_DEFAULT_ACCESS_SECTOR

#ifndef	NO_DEFAULT_DISK_CONNECT			/* DefineNO_DEFAULT_DISK_CONNECT in the application to disable the default disk connection check routine,, then replace it with a self-written routine */

/*
Convention: USBUSB device address allocation rules(seeUSB_DEVICE_ADDR)
Address value  device position
0x02    Built-inRoot-HUB0 subordinateUSB device or external HUBHUB
0x03    Built-inRoot-HUB1 subordinateUSB device or external HUBHUB
0x1x    Built-inRoot-HUB0 subordinate externalHUB portx subordinateUSB device,x, x ranges 1~n1~n
0x2x    Built-inRoot-HUB1 subordinate externalHUB portx subordinateUSB device,x, x ranges 1~n1~n
*/

#define		UHUB_DEV_ADDR	R8_USB_DEV_AD
#define		UHUB_MIS_STAT	R8_USB_MIS_ST
#define		UHUB_HOST_CTRL	R8_UHOST_CTRL
#define		UHUB_INT_FLAG	R8_USB_INT_FG
#define		bUMS_ATTACH		RB_UMS_DEV_ATTACH
#define		bUMS_SUSPEND	RB_UMS_SUSPEND

/* Check whether the disk is connected */
UINT8 CHRV3DiskConnect( void )
{
	UINT8	ums, devaddr;
	UHUB_DEV_ADDR = UHUB_DEV_ADDR & 0x7F;
	ums = UHUB_MIS_STAT;
	devaddr = UHUB_DEV_ADDR;
	if ( devaddr == USB_DEVICE_ADDR )
	{  /* Built-inRoot-HUB subordinateUSB device */
		if ( ums & bUMS_ATTACH )
		{  /* Built-inRoot-HUB subordinateUSB device present */
			if ( ( ums & bUMS_SUSPEND ) == 0 )
			{
			    /* Built-inRoot-HUB subordinateUSB device present and not unplugged */
				return( ERR_SUCCESS );  /* USB device already connected and not unplugged */
			}
			else
			{  /* Built-inRoot-HUB subordinateUSB device present */
//mDiskConnect:
				CHRV3DiskStatus = DISK_CONNECT;  /* Was disconnected before */
				return( ERR_SUCCESS );  /* ExternalHUB orUSB device connected, or reconnected after disconnection */
			}
		}
		else
		{
		    /* USB device disconnected */
mDiskDisconn:
			CHRV3DiskStatus = DISK_DISCONNECT;
			return( ERR_USB_DISCON );
		}
	}
	else
	{
		goto mDiskDisconn;
	}
}
#endif  // NO_DEFAULT_DISK_CONNECT

#ifndef	NO_DEFAULT_FILE_ENUMER			/* DefineNO_DEFAULT_FILE_ENUMER in the application to disable the default file name enumeration callback routine,, then replace it with a self-written routine */
void xFileNameEnumer( void )			/* File name enumeration callback routine */
{
/* If the specified enumeration indexCHRV3vFileSize is0xFFFFFFFF, callFileOpen afterwards; every time a file is foundFileOpen this callback routine will be called,
    callback routinexFileNameEnumer after returning,FileOpen decrementsCHRV3vFileSize and enumeration continues until no more files or directories are found. Recommended practice: 
   Before callingFileOpen, define a global variable as0, whenFileOpen calls back into this routine; fromCHRV3vFdtOffset get the structureFAT_DIR_INFO,
   AnalyzeDIR_Attr andDIR_Name in the structure to check whether it is the wanted file or directory name; record the info and increment the global counter,
   WhenFileOpen returns, check the return value: if it isERR_MISS_FILE orERR_FOUND_NAME, treat it as success; the global variable holds the number of valid files found.
   If, in this callback routine,xFileNameEnumer setsCHRV3vFileSize to1, thenFileOpen can be notified to end the search early. Below is a callback routine example */
#if		0
	UINT8			i;
	UINT16			FileCount;
	PX_FAT_DIR_INFO	pFileDir;
	PUINT8			NameBuf;
	pFileDir = (PX_FAT_DIR_INFO)( pDISK_BASE_BUF + CHRV3vFdtOffset );  /* CurrentFDT's start address */
	FileCount = (UINT16)( 0xFFFFFFFF - CHRV3vFileSize );  /* The current enumeration index of the file name,CHRV3vFileSize, initially0xFFFFFFFF,, decremented after a file name is found */
	if ( FileCount < sizeof( FILE_DATA_BUF ) / 12 ) {  /* Check whether the buffer is large enough to hold,, assuming each file name takes12 bytes */
		NameBuf = & FILE_DATA_BUF[ FileCount * 12 ];  /* Calculate the buffer address for saving the current file name */
		for ( i = 0; i < 11; i ++ ) NameBuf[ i ] = pFileDir -> DIR_Name[ i ];  /* Copy the file name,, length11 characters,, spaces not handled */
//		if ( pFileDir -> DIR_Attr & ATTR_DIRECTORY ) NameBuf[ i ] = 1;  /* Check whether it is a directory name */
		NameBuf[ i ] = 0;  /* File name terminator */
	}
#endif
}
#endif  // NO_DEFAULT_FILE_ENUMER

UINT8	CHRV3LibInit( void )  /* Initialize theCHRV3 library,; return on successful operation0 */
{
	if ( CHRV3GetVer( ) < CHRV3_LIB_VER ) return( 0xFF );  /* Get the version number of the current subroutine library,return an error if the version is too low */
#if		DISK_BASE_BUF_LEN > 0
	pDISK_BASE_BUF = & DISK_BASE_BUF[0];  /* Pointing to the externalRAM disk data buffer */
	pDISK_FAT_BUF = & DISK_BASE_BUF[0];  /* Pointing to the externalRAM diskFAT data buffer,, can share withpDISK_BASE_BUF to saveRAM */
//	pDISK_FAT_BUF = & DISK_FAT_BUF[0];  /* Pointing to the externalRAM diskFAT data buffer,, independent ofpDISK_BASE_BUF for higher speed */
/* To improve file access speed,, callCHRV3LibInit in the main program, then re-,pointpDISK_FAT_BUF to another independently allocated buffer of the same size aspDISK_BASE_BUF */
#endif
	CHRV3DiskStatus = DISK_UNKNOWN;  /* Unknown status */
	CHRV3vSectorSizeB = 9;  /* The sectors of the default physical disk are512B */
	CHRV3vSectorSize = 512;  // The sectors of the default physical disk are512B,this value is the sector size of the disk
	CHRV3vStartLba = 0;  /* Automatic analysis by defaultFDD andHDD */
	CHRV3vPacketSize = 64;  /* USBMaximum packet length of the storage device:64@FS,512@HS/SS,, initialized by the application,, after enumerating theU drive: if high-speed or super-speed, update it to512 */
	pTX_DMA_A_REG = (PUINT32)&R16_UH_TX_DMA;  /* Pointing to the transmitDMA address register,, initialized by the application */
	pRX_DMA_A_REG = (PUINT32)&R16_UH_RX_DMA;  /* Pointing to the receiveDMA address register,, initialized by the application */
	pTX_LEN_REG = (PUINT16)&R8_UH_TX_LEN;  /* Pointing to the transmit length register,, initialized by the application */
	pRX_LEN_REG = (PUINT16)&R16_USB_RX_LEN;  /* Pointing to the receive length register,, initialized by the application */

//CHRV3vRootPort = 0;  /* USB host selection( (similar toRoot-hub root hub port selection)) */
	return( ERR_SUCCESS );
}

void mDelaymS( UINT16 n )
{
	Delay_Ms(n);
}

