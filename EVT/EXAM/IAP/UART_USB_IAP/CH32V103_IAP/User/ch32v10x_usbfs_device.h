/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v10x_usbfs_device.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/12/03
 * Description        : This file provides all the USBFS firmware functions.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

#ifndef __CH32V10X_USBFS_DEVICE_H_
#define __CH32V10X_USBFS_DEVICE_H_

#include "debug.h"
#include "string.h"

/* Endpoint Buffer */
extern __attribute__ ((aligned(4))) UINT8 EP0_Databuf[64]; //ep0(64)
extern __attribute__ ((aligned(4))) UINT8 EP1_Databuf[64 + 64];//ep1_out(64)+ep1_in(64)
extern __attribute__ ((aligned(4))) UINT8 EP2_Databuf[64 + 64];//ep2_out(64)+ep2_in(64)
extern __attribute__ ((aligned(4))) UINT8 EP3_Databuf[64 + 64];//ep3_out(64)+ep3_in(64)
extern __attribute__ ((aligned(4))) UINT8 EP4_Databuf[64 + 64];//ep4_out(64)+ep4_in(64)
extern __attribute__ ((aligned(4))) UINT8 EP5_Databuf[64 + 64];//ep5_out(64)+ep5_in(64)
extern __attribute__ ((aligned(4))) UINT8 EP6_Databuf[64 + 64];//ep6_out(64)+ep6_in(64)
extern __attribute__ ((aligned(4))) UINT8 EP7_Databuf[64 + 64];//ep7_out(64)+ep7_in(64)



void   USBHD_ClockCmd(UINT32 RCC_USBCLKSource, FunctionalState NewState);

#endif /* __CH32V10X_USBFS_DEVICE_H_ */
