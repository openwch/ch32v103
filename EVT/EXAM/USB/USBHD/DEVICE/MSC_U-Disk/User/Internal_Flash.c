/********************************** (C) COPYRIGHT *******************************
 * File Name          : Internal_Flash.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/08
 * Description        : Internal Flash program
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "ch32v10x.h"
#include "Internal_Flash.h"
#include "ch32v10x_flash.h"

void IFlash_Prog_512(uint32_t address,uint32_t *pbuf)
{
    uint8_t i, j;
    if (address < IFLASH_UDISK_START_ADDR || (address + 511) > IFLASH_UDISK_END_ADDR )
    {
        printf("Error Address %x\n",address);
        return;
    }
    address &= 0x00FFFFFF;
    address |= 0x08000000;
    NVIC_DisableIRQ(USBHD_IRQn);
    FLASH_Unlock_Fast();
    for (j = 0; j < 4; j++)
    {
        FLASH_ErasePage_Fast(address);
        FLASH_BufReset();
        for (i = 0; i < 8; i++)
        {
            FLASH_BufLoad(address + 16 * i, *pbuf, *(pbuf + 1), *(pbuf + 2), *(pbuf + 3));
            pbuf += 4;
        }
        /* Program 128 */
        FLASH_ProgramPage_Fast(address);
        address += 128;
    }
    FLASH_Lock_Fast();
    NVIC_EnableIRQ(USBHD_IRQn);
}
