/********************************** (C) COPYRIGHT  *******************************
 * File Name          : debug.h
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : This file contains all the functions prototypes for UART
 *                      Printf , Delay functions.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/
#ifndef __DEBUG_H
#define __DEBUG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "ch32v10x.h"

/* HID功能开关，1为关闭HID功能，0为打开HID功能 */
#define ch32v10x_usb_hid       0

/* UART Printf Definition */
#define DEBUG_UART1    1
#define DEBUG_UART2    2
#define DEBUG_UART3    3

/* DEBUG UATR Definition */
#define DEBUG          DEBUG_UART1
//#define DEBUG   DEBUG_UART2
//#define DEBUG   DEBUG_UART3

void Delay_Init(void);
void Delay_Us(uint32_t n);
void Delay_Ms(uint32_t n);
void USART_Printf_Init(uint32_t baudrate);

#ifdef __cplusplus
}
#endif

#endif /* __DEBUG_H */
