/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v10x_usbfs_device.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : header file for ch32v10x_usbfs_device.c
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/


#ifndef __CH32V10x_USBFS_DEVICE_H_
#define __CH32V10x_USBFS_DEVICE_H_

/*******************************************************************************/
/* Header File */
#include "string.h"
#include "debug.h"
#include "usbd_desc.h"
#include "ch32v10x_usb.h"

/*******************************************************************************/
/* Macro Definition */
#define pEP0_RAM_Addr                 USBHD_EP0_Buf

/* end-point number */
#define DEF_UEP_IN                    0x80
#define DEF_UEP_OUT                   0x00
#define DEF_UEP_BUSY                  0x01
#define DEF_UEP_FREE                  0x00
#define DEF_UEP_NUM                   0x08
#define DEF_UEP0                      0x00
#define DEF_UEP1                      0x01
#define DEF_UEP2                      0x02
#define DEF_UEP3                      0x03
#define DEF_UEP4                      0x04
#define DEF_UEP5                      0x05
#define DEF_UEP6                      0x06
#define DEF_UEP7                      0x07

#define USBHD_UEP_MOD_BASE            0x4002340C
#define USBHD_UEP_DMA_BASE            0x40023410
#define USBHD_UEP_LEN_BASE            0x40023430
#define USBHD_UEP_CTL_BASE            0x40023432
#define USBHD_UEP_RX_EN               0x08
#define USBHD_UEP_TX_EN               0x04
#define USBHD_UEP_BUF_MOD             0x01
#define DEF_UEP_DMA_LOAD              0 /* Direct the DMA address to the data to be processed */
#define DEF_UEP_CPY_LOAD              1 /* Use memcpy to move data to a buffer */
#define USBHD_UEP_MOD( N )            (*((volatile uint8_t *)( USBHD_UEP_MOD_BASE + N )))
#define USBHD_UEP_CTRL( N )           (*((volatile uint8_t *)( USBHD_UEP_CTL_BASE + N * 0x04 )))
#define USBHD_UEP_DMA( N )            (*((volatile uint32_t *)( USBHD_UEP_DMA_BASE + N * 0x04 )))
#define USBHD_UEP_BUF( N )            ((uint8_t *)(*((volatile uint32_t *)( USBHD_UEP_DMA_BASE + N * 0x04 ))) + 0x20000000)
#define USBHD_UEP_TLEN( N )           (*((volatile uint16_t *)( USBHD_UEP_LEN_BASE + N * 0x04 )))

/******************************************************************************/
/* Variable Declaration */

/* Global */
extern const    uint8_t  *pUSBHD_Descr;

/* Setup Request */
extern volatile uint8_t  USBHD_SetupReqCode;
extern volatile uint8_t  USBHD_SetupReqType;
extern volatile uint16_t USBHD_SetupReqValue;
extern volatile uint16_t USBHD_SetupReqIndex;
extern volatile uint16_t USBHD_SetupReqLen;

/* USB Device Status */
extern volatile uint8_t  USBHD_DevConfig;
extern volatile uint8_t  USBHD_DevAddr;
extern volatile uint8_t  USBHD_DevSleepStatus;
extern volatile uint8_t  USBHD_DevEnumStatus;

/* HID Class Command */
extern volatile uint8_t  USBHD_HidIdle[ 2 ];
extern volatile uint8_t  USBHD_HidProtocol[ 2 ];

/* Endpoint Buffer */
extern __attribute__ ((aligned(4))) uint8_t USBHD_EP0_Buf[ DEF_USBD_UEP0_SIZE ];      //ep0(64)
extern __attribute__ ((aligned(4))) uint8_t USBHD_EP1_Buf[ DEF_USBD_EP1_FS_SIZE ];    //ep1_in(64)
extern __attribute__ ((aligned(4))) uint8_t USBHD_EP2_Buf[ DEF_USBD_EP2_FS_SIZE ];    //ep2_in(64)

/* USB IN Endpoint Busy Flag */
extern volatile uint8_t  USBHD_Endp_Busy[ DEF_UEP_NUM ];

/* Setup Request Packets */
#define pUSBHD_SetupReqPak                 ((PUSB_SETUP_REQ)USBHD_EP0_Buf)

/******************************************************************************/
/* Function Declaration */
extern void USBHD_RCC_Init( void );
extern void USBHD_Device_Endp_Init( void );
extern void USBHD_Device_Init( FunctionalState sta , PWR_VDD VDD_Voltage);
extern uint8_t USBHD_Endp_DataUp( uint8_t endp, uint8_t *pbuf, uint16_t len, uint8_t mod );
extern void USBHD_Send_Resume( void );


#endif /* __CH32V10X_USB_DEVICE_H_ */
