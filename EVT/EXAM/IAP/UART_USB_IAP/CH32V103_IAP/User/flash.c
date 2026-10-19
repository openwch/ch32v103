/********************************** (C) COPYRIGHT  *******************************
* File Name          : iap.c
* Author             : WCH
* Version            : V1.0.2
* Date               : 2026/10/15
* Description        : CH32F103  fast program
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "flash.h"
#include "string.h"

u32 Verify_buf[32];


/*********************************************************************
 * @fn      CH32_IAP_Program
 *
 * @brief   adr - 128Byte stand
 *          buf - 128Byte stand
 *
 * @return  none
 */
void CH32_IAP_Program(u32 adr, u32* buf)
{
    FLASH_ROM_WRITE(adr, buf, 128);
}

