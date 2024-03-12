/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2024/01/05
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *FLASH erase / read / write, and fast programming:
 *Includes Standard Erase and Program, Fast Erase and Program.
 *
 */

#include "debug.h"

/* Global define */
typedef enum
{
    FAILED = 0,
    PASSED = !FAILED
} TestStatus;
#define PAGE_WRITE_START_ADDR          ((uint32_t)0x0800F000) /* Start from 60K */
#define PAGE_WRITE_END_ADDR            ((uint32_t)0x08010000) /* End at 63K */
#define FLASH_PAGE_SIZE                1024
#define FLASH_PAGES_TO_BE_PROTECTED    FLASH_WRProt_Pages60to63

/* Global Variable */
uint32_t              EraseCounter = 0x0, Address = 0x0;
uint16_t              Data = 0xAAAA;
uint32_t              WRPR_Value = 0xFFFFFFFF, ProtectedPages = 0x0;
uint32_t              NbrOfPage;
volatile FLASH_Status FLASHStatus = FLASH_COMPLETE;
volatile TestStatus MemoryProgramStatus = PASSED;
volatile TestStatus MemoryEraseStatus = PASSED;

#define Fadr    (0x0800E000)
#define Fsize   ((((128*4))>>2))
u32 buf[Fsize];

/*********************************************************************
 * @fn      Flash_Test
 *
 * @brief   Flash Program Test.
 *
 * @return  none
 */
void Flash_Test(void)
{
    FLASH_Unlock();
    WRPR_Value = FLASH_GetWriteProtectionOptionByte();

    NbrOfPage = (PAGE_WRITE_END_ADDR - PAGE_WRITE_START_ADDR) / FLASH_PAGE_SIZE;

    if((WRPR_Value & FLASH_PAGES_TO_BE_PROTECTED) != 0x00)
    {
        FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);

        for(EraseCounter = 0; (EraseCounter < NbrOfPage) && (FLASHStatus == FLASH_COMPLETE); EraseCounter++)
        {
            FLASHStatus = FLASH_ErasePage(PAGE_WRITE_START_ADDR + (FLASH_PAGE_SIZE * EraseCounter));
            if(FLASHStatus != FLASH_COMPLETE)
            {
                printf("FLASH Erase ERR at Page%d\r\n", EraseCounter + 60);
                return;
            }
            printf("FLASH Erase Page%d...\r\n", EraseCounter + 60);
        }

        Address = PAGE_WRITE_START_ADDR;
        printf("Erase Cheking...\r\n");
        while((Address < PAGE_WRITE_END_ADDR) && (MemoryEraseStatus != FAILED))
        {
            if((*(__IO uint16_t *)Address) != 0xFFFF)
            {
                MemoryEraseStatus = FAILED;
            }
            Address += 2;
        }
        if(MemoryEraseStatus == FAILED)
        {
            printf("Erase Flash FAIL!\r\n");
            printf("\r\n");
        }
        else
        {
            printf("Erase Flash PASS!\r\n");
            printf("\r\n");
        }

        Address = PAGE_WRITE_START_ADDR;
        printf("Programing...\r\n");
        while((Address < PAGE_WRITE_END_ADDR) && (FLASHStatus == FLASH_COMPLETE))
        {
            FLASHStatus = FLASH_ProgramHalfWord(Address, Data);
            Address = Address + 2;
        }

        Address = PAGE_WRITE_START_ADDR;
        printf("Program Cheking...\r\n");
        while((Address < PAGE_WRITE_END_ADDR) && (MemoryProgramStatus != FAILED))
        {
            if((*(__IO uint16_t *)Address) != Data)
            {
                MemoryProgramStatus = FAILED;
            }
            Address += 2;
        }
        if(MemoryProgramStatus == FAILED)
        {
            printf("Memory Program FAIL!\r\n");
            printf("\r\n");
        }
        else
        {
            printf("Memory Program PASS!\r\n");
            printf("\r\n");
        }
    }
    else
    {
        MemoryProgramStatus = FAILED;
        printf("Error to program the flash : The desired pages are write protected\r\n");
    }

    FLASH_Lock();
}

/*********************************************************************
 * @fn      Flash_Test_Fast
 *
 * @brief   Flash Fast Program Test.
 *
 * @return  none
 */
void Flash_Test_Fast(void)
{
    u32 i;
    u8 Verify_Flag = 0;
    FLASH_Status s;

    for(i = 0; i < Fsize; i++){
        buf[i] = i;
    }

    printf("Read flash\r\n");
    for(i=0; i<Fsize; i++){
        printf("adr-%08x v-%08x\r\n", Fadr +4*i, *(u32*)(Fadr +4*i));
    }

    s = FLASH_ROM_ERASE(Fadr, Fsize*4);
    if(s != FLASH_COMPLETE)
    {
        printf("check FLASH_ADR_RANGE_ERROR FLASH_ALIGN_ERROR or FLASH_OP_RANGE_ERROR\r\n");
        return;
    }

    printf("Erase flash\r\n");
    for(i=0; i<Fsize; i++){
        printf("adr-%08x v-%08x\r\n", Fadr +4*i, *(u32*)(Fadr +4*i));
    }

    s = FLASH_ROM_WRITE(Fadr,  buf, Fsize*4);
    if(s != FLASH_COMPLETE)
    {
        printf("check FLASH_ADR_RANGE_ERROR FLASH_ALIGN_ERROR or FLASH_OP_RANGE_ERROR\r\n");
        return;
    }

    printf("Write flash\r\n");
    for(i=0; i<Fsize; i++){
        printf("adr-%08x v-%08x\r\n", Fadr +4*i, *(u32*)(Fadr +4*i));
    }

    for(i = 0; i < Fsize; i++){
        if(buf[i] == *(u32 *)(Fadr + 4 * i))
        {
            Verify_Flag = 0;
        }
        else
        {
            Verify_Flag = 1;
            break;
        }
    }

    if(Verify_Flag)
        printf("%d Byte Verify Fail\r\n", (Fsize*4));
    else
        printf("%d Byte Verify Suc\r\n", (Fsize*4));
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main(void)
{
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
    SystemCoreClockUpdate();
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );
    printf("Flash Program Test\r\n");

    Flash_Test();
    Flash_Test_Fast();

    while(1);
}
