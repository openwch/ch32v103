/********************************** (C) COPYRIGHT  *******************************
 * File Name          : ch32v10x_flash.h
 * Author             : WCH
 * Version            : V1.0.1
 * Date               : 2024/01/02
 * Description        : This file contains all the functions prototypes for the FLASH
 *                      firmware library.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#ifndef __CH32V10x_FLASH_H
#define __CH32V10x_FLASH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "ch32v10x.h"

/* FLASH Status */
typedef enum
{
    FLASH_BUSY = 1,
    FLASH_ERROR_PG,
    FLASH_ERROR_WRP,
    FLASH_COMPLETE,
    FLASH_TIMEOUT,
    FLASH_OP_RANGE_ERROR = 0xFD,
    FLASH_ALIGN_ERROR = 0xFE,
    FLASH_ADR_RANGE_ERROR = 0xFF,
} FLASH_Status;

/* Flash_Latency */
#define FLASH_Latency_0                  ((uint32_t)0x00000000) /* FLASH Zero Latency cycle */
#define FLASH_Latency_1                  ((uint32_t)0x00000001) /* FLASH One Latency cycle */
#define FLASH_Latency_2                  ((uint32_t)0x00000002) /* FLASH Two Latency cycles */

/* Half_Cycle_Enable_Disable */
#define FLASH_HalfCycleAccess_Enable     ((uint32_t)0x00000008) /* FLASH Half Cycle Enable */
#define FLASH_HalfCycleAccess_Disable    ((uint32_t)0x00000000) /* FLASH Half Cycle Disable */

/* Prefetch_Buffer_Enable_Disable */
#define FLASH_PrefetchBuffer_Enable      ((uint32_t)0x00000010) /* FLASH Prefetch Buffer Enable */
#define FLASH_PrefetchBuffer_Disable     ((uint32_t)0x00000000) /* FLASH Prefetch Buffer Disable */

/* Write Protect */
#define FLASH_WRProt_Pages0to3           ((uint32_t)0x00000001) /* Write protection of the standard page 0 to 3 ,1K bytes/standard page */
#define FLASH_WRProt_Pages4to7           ((uint32_t)0x00000002) /* Write protection of the standard page 4 to 7 ,1K bytes/standard page */
#define FLASH_WRProt_Pages8to11          ((uint32_t)0x00000004) /* Write protection of the standard page 8 to 11 ,1K bytes/standard page */
#define FLASH_WRProt_Pages12to15         ((uint32_t)0x00000008) /* Write protection of the standard page 12 to 15 ,1K bytes/standard page */
#define FLASH_WRProt_Pages16to19         ((uint32_t)0x00000010) /* Write protection of the standard page 16 to 19 ,1K bytes/standard page */
#define FLASH_WRProt_Pages20to23         ((uint32_t)0x00000020) /* Write protection of the standard page 20 to 23 ,1K bytes/standard page */
#define FLASH_WRProt_Pages24to27         ((uint32_t)0x00000040) /* Write protection of the standard page 24 to 27 ,1K bytes/standard page */
#define FLASH_WRProt_Pages28to31         ((uint32_t)0x00000080) /* Write protection of the standard page 28 to 31 ,1K bytes/standard page */
#define FLASH_WRProt_Pages32to35         ((uint32_t)0x00000100) /* Write protection of the standard page 32 to 35 ,1K bytes/standard page */
#define FLASH_WRProt_Pages36to39         ((uint32_t)0x00000200) /* Write protection of the standard page 36 to 39 ,1K bytes/standard page */
#define FLASH_WRProt_Pages40to43         ((uint32_t)0x00000400) /* Write protection of the standard page 40 to 43 ,1K bytes/standard page */
#define FLASH_WRProt_Pages44to47         ((uint32_t)0x00000800) /* Write protection of the standard page 44 to 47 ,1K bytes/standard page */
#define FLASH_WRProt_Pages48to51         ((uint32_t)0x00001000) /* Write protection of the standard page 48 to 51 ,1K bytes/standard page */
#define FLASH_WRProt_Pages52to55         ((uint32_t)0x00002000) /* Write protection of the standard page 52 to 55 ,1K bytes/standard page */
#define FLASH_WRProt_Pages56to59         ((uint32_t)0x00004000) /* Write protection of the standard page 56 to 59 ,1K bytes/standard page */
#define FLASH_WRProt_Pages60to63         ((uint32_t)0x00008000) /* Write protection of the standard page 60 to 63 ,1K bytes/standard page */
#define FLASH_WRProt_Pages64to67         ((uint32_t)0x00010000) /* Write protection of the standard page 64 to 67 ,1K bytes/standard page */
#define FLASH_WRProt_Pages68to71         ((uint32_t)0x00020000) /* Write protection of the standard page 68 to 71 ,1K bytes/standard page */
#define FLASH_WRProt_Pages72to75         ((uint32_t)0x00040000) /* Write protection of the standard page 72 to 75 ,1K bytes/standard page */
#define FLASH_WRProt_Pages76to79         ((uint32_t)0x00080000) /* Write protection of the standard page 76 to 79 ,1K bytes/standard page */
#define FLASH_WRProt_Pages80to83         ((uint32_t)0x00100000) /* Write protection of the standard page 80 to 83 ,1K bytes/standard page */
#define FLASH_WRProt_Pages84to87         ((uint32_t)0x00200000) /* Write protection of the standard page 84 to 87 ,1K bytes/standard page */
#define FLASH_WRProt_Pages88to91         ((uint32_t)0x00400000) /* Write protection of the standard page 88 to 91 ,1K bytes/standard page */
#define FLASH_WRProt_Pages92to95         ((uint32_t)0x00800000) /* Write protection of the standard page 92 to 95 ,1K bytes/standard page */
#define FLASH_WRProt_Pages96to99         ((uint32_t)0x01000000) /* Write protection of the standard page 96 to 99 ,1K bytes/standard page */
#define FLASH_WRProt_Pages100to103       ((uint32_t)0x02000000) /* Write protection of the standard page 100 to 103 ,1K bytes/standard page */
#define FLASH_WRProt_Pages104to107       ((uint32_t)0x04000000) /* Write protection of the standard page 104 to 107 ,1K bytes/standard page */
#define FLASH_WRProt_Pages108to111       ((uint32_t)0x08000000) /* Write protection of the standard page 108 to 111 ,1K bytes/standard page */
#define FLASH_WRProt_Pages112to115       ((uint32_t)0x10000000) /* Write protection of the standard page 112 to 115 ,1K bytes/standard page */
#define FLASH_WRProt_Pages116to119       ((uint32_t)0x20000000) /* Write protection of the standard page 115 to 119 ,1K bytes/standard page */
#define FLASH_WRProt_Pages120to123       ((uint32_t)0x40000000) /* Write protection of the standard page 120 to 123 ,1K bytes/standard page */
#define FLASH_WRProt_Pages124to127       ((uint32_t)0x80000000) /* Write protection of the standard page 124 to 127 ,1K bytes/standard page */

#define FLASH_WRProt_AllPages            ((uint32_t)0xFFFFFFFF) /* Write protection of all Pages */

/* Option_Bytes_IWatchdog */
#define OB_IWDG_SW                       ((uint16_t)0x0001) /* Software IWDG selected */
#define OB_IWDG_HW                       ((uint16_t)0x0000) /* Hardware IWDG selected */

/* Option_Bytes_nRST_STOP */
#define OB_STOP_NoRST                    ((uint16_t)0x0002) /* No reset generated when entering in STOP */
#define OB_STOP_RST                      ((uint16_t)0x0000) /* Reset generated when entering in STOP */

/* Option_Bytes_nRST_STDBY  */
#define OB_STDBY_NoRST                   ((uint16_t)0x0004) /* No reset generated when entering in STANDBY */
#define OB_STDBY_RST                     ((uint16_t)0x0000) /* Reset generated when entering in STANDBY */

/* FLASH_Interrupts */
#define FLASH_IT_ERROR                   ((uint32_t)0x00000400) /* FPEC error interrupt source */
#define FLASH_IT_EOP                     ((uint32_t)0x00001000) /* End of FLASH Operation Interrupt source */
#define FLASH_IT_BANK1_ERROR             FLASH_IT_ERROR         /* FPEC BANK1 error interrupt source */
#define FLASH_IT_BANK1_EOP               FLASH_IT_EOP           /* End of FLASH BANK1 Operation Interrupt source */

/* FLASH_Flags */
#define FLASH_FLAG_BSY                   ((uint32_t)0x00000001) /* FLASH Busy flag */
#define FLASH_FLAG_EOP                   ((uint32_t)0x00000020) /* FLASH End of Operation flag */
#define FLASH_FLAG_PGERR                 ((uint32_t)0x00000004) /* FLASH Program error flag */
#define FLASH_FLAG_WRPRTERR              ((uint32_t)0x00000010) /* FLASH Write protected error flag */
#define FLASH_FLAG_OPTERR                ((uint32_t)0x00000001) /* FLASH Option Byte error flag */

#define FLASH_FLAG_BANK1_BSY             FLASH_FLAG_BSY       /* FLASH BANK1 Busy flag*/
#define FLASH_FLAG_BANK1_EOP             FLASH_FLAG_EOP       /* FLASH BANK1 End of Operation flag */
#define FLASH_FLAG_BANK1_PGERR           FLASH_FLAG_PGERR     /* FLASH BANK1 Program error flag */
#define FLASH_FLAG_BANK1_WRPRTERR        FLASH_FLAG_WRPRTERR  /* FLASH BANK1 Write protected error flag */

/*Functions used for all CH32V10x devices*/
void         FLASH_SetLatency(uint32_t FLASH_Latency);
void         FLASH_HalfCycleAccessCmd(uint32_t FLASH_HalfCycleAccess);
void         FLASH_PrefetchBufferCmd(uint32_t FLASH_PrefetchBuffer);
void         FLASH_Unlock(void);
void         FLASH_Lock(void);
FLASH_Status FLASH_ErasePage(uint32_t Page_Address);
FLASH_Status FLASH_EraseAllPages(void);
FLASH_Status FLASH_EraseOptionBytes(void);
FLASH_Status FLASH_ProgramWord(uint32_t Address, uint32_t Data);
FLASH_Status FLASH_ProgramHalfWord(uint32_t Address, uint16_t Data);
FLASH_Status FLASH_ProgramOptionByteData(uint32_t Address, uint8_t Data);
FLASH_Status FLASH_EnableWriteProtection(uint32_t FLASH_Pages);
FLASH_Status FLASH_ReadOutProtection(FunctionalState NewState);
FLASH_Status FLASH_UserOptionByteConfig(uint16_t OB_IWDG, uint16_t OB_STOP, uint16_t OB_STDBY);
uint32_t     FLASH_GetUserOptionByte(void);
uint32_t     FLASH_GetWriteProtectionOptionByte(void);
FlagStatus   FLASH_GetReadOutProtectionStatus(void);
FlagStatus   FLASH_GetPrefetchBufferStatus(void);
void         FLASH_ITConfig(uint32_t FLASH_IT, FunctionalState NewState);
FlagStatus   FLASH_GetFlagStatus(uint32_t FLASH_FLAG);
void         FLASH_ClearFlag(uint32_t FLASH_FLAG);
FLASH_Status FLASH_GetStatus(void);
FLASH_Status FLASH_WaitForLastOperation(uint32_t Timeout);
void         FLASH_Unlock_Fast(void);
void         FLASH_Lock_Fast(void);
void         FLASH_BufReset(void);
void         FLASH_BufLoad(uint32_t Address, uint32_t Data0, uint32_t Data1, uint32_t Data2, uint32_t Data3);
void         FLASH_ErasePage_Fast(uint32_t Page_Address);
void         FLASH_ProgramPage_Fast(uint32_t Page_Address);

/* New function used for all CH32V10x devices */
void         FLASH_UnlockBank1(void);
void         FLASH_LockBank1(void);
FLASH_Status FLASH_EraseAllBank1Pages(void);
FLASH_Status FLASH_GetBank1Status(void);
FLASH_Status FLASH_WaitForLastBank1Operation(uint32_t Timeout);
FLASH_Status FLASH_ROM_ERASE(uint32_t StartAddr, uint32_t Length);
FLASH_Status FLASH_ROM_WRITE(uint32_t StartAddr, uint32_t *pbuf, uint32_t Length);

#ifdef __cplusplus
}
#endif

#endif /* __CH32V10x_FLASH_H */
