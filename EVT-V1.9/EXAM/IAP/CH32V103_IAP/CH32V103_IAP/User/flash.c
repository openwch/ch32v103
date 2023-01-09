/********************************** (C) COPYRIGHT  *******************************
* File Name          : iap.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2020/12/16
* Description        : CH32F103  fast program
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "flash.h"
#include "string.h"


u32 Verity_buf[32];

/*********************************************************************
 * @fn      CH32_IAP_ERASE
 *
 * @brief   Start_adr:  128Byte stand
 *          End_adr:    128Byte stand
 *
 * @return  none
 */
void CH32_IAP_ERASE(u32 Start_adr, u32 End_adr)
{
    u16 i;
    u32 len;

    Start_adr &= 0xFFFFFF80;
    End_adr &= 0xFFFFFF80;

    len = (End_adr - Start_adr)/128;


    for(i=0; i<len; i++){
        FLASH_ErasePage_Fast(Start_adr + 128*i);
    }
}

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
    adr &= 0xFFFFFF80;

    FLASH_BufReset();
    FLASH_BufLoad(adr, buf[0], buf[1], buf[2], buf[3]);
    FLASH_BufLoad(adr + 0x10, buf[4], buf[5], buf[6], buf[7]);
    FLASH_BufLoad(adr + 0x20, buf[8], buf[9], buf[10], buf[11]);
    FLASH_BufLoad(adr + 0x30, buf[12], buf[13], buf[14], buf[15]);
    FLASH_BufLoad(adr + 0x40, buf[16], buf[17], buf[18], buf[19]);
    FLASH_BufLoad(adr + 0x50, buf[20], buf[21], buf[22], buf[23]);
    FLASH_BufLoad(adr + 0x60, buf[24], buf[25], buf[26], buf[27]);
    FLASH_BufLoad(adr + 0x70, buf[28], buf[29], buf[30], buf[31]);

    FLASH_ProgramPage_Fast(adr);
}

