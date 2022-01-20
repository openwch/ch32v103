/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main program body.
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * SPDX-License-Identifier: Apache-2.0
 *******************************************************************************/

/*
 *@Note
  U盘文件枚举例程：
  USBHDM(PA11)、USBHDP(PA12)。
  本例程演示 U盘文件系统，枚举根目录或者指定目下的文件。
  注：文件系统支持 FAT12/FAT16/FAT32。

*/

#include "debug.h"
#include "string.h"
#include "CHRV3UFI.h"
/* Global define */

/* Global Variable */
__attribute__((aligned(4))) UINT8 RxBuffer[MAX_PACKET_SIZE]; // IN, must even address
__attribute__((aligned(4))) UINT8 TxBuffer[MAX_PACKET_SIZE]; // OUT, must even address
__attribute__((aligned(4))) UINT8 Com_Buffer[128];

UINT8 buf[100]; //长度可以根据应用自己指定

/* 检查操作状态,如果错误则显示错误代码并停机 */
void mStopIfError(UINT8 iError)
{
    if(iError == ERR_SUCCESS)
    {
        return; /* 操作成功 */
    }
    printf("Error: %02X\r\n", (UINT16)iError); /* 显示错误 */
    /* 遇到错误后,应该分析错误码以及CHRV3DiskStatus状态,例如调用CHRV3DiskReady查询当前U盘是否连接,如果U盘已断开那么就重新等待U盘插上再操作,
       建议出错后的处理步骤:
       1、调用一次CHRV3DiskReady,成功则继续操作,例如Open,Read/Write等
       2、如果CHRV3DiskReady不成功,那么强行将从头开始操作(等待U盘连接，CHRV3DiskReady等) */
    while(1)
    {
    }
}

/*********************************************************************
 * @fn      Set_USBConfig
 *
 * @brief   Set USB clock.
 *
 * @return  none
 */
void USBHD_ClockCmd(UINT32 RCC_USBCLKSource, FunctionalState NewState)
{
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, NewState);
    EXTEN->EXTEN_CTR |= EXTEN_USBHD_IO_EN;
    RCC_USBCLKConfig(RCC_USBCLKSource); //USBclk=PLLclk/1.5=48Mhz
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHD, NewState);
}

/*********************************************************************
 * @fn      main
 *
 * @brief   Main program.
 *
 * @return  none
 */
int main()
{
    UINT8  s, i;
    PUINT8 pCodeStr;
    UINT16 j;

    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    Delay_Init();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);

    printf("USBHD HOST Test\r\n");
    USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5, ENABLE);
    pHOST_RX_RAM_Addr = RxBuffer;
    pHOST_TX_RAM_Addr = TxBuffer;
    USB_HostInit();
    CHRV3LibInit();
    printf("Wait Device In\r\n");

    while(1)
    {
        s = ERR_SUCCESS;

        if(R8_USB_INT_FG & RB_UIF_DETECT)
        {
            R8_USB_INT_FG = RB_UIF_DETECT;

            s = AnalyzeRootHub();
            if(s == ERR_USB_CONNECT)
            {
                printf("New Device In\r\n");
                FoundNewDev = 1;
            }
            if(s == ERR_USB_DISCON)
            {
                printf("Device Out\r\n");
            }
        }

        if(FoundNewDev || s == ERR_USB_CONNECT)
        {
            FoundNewDev = 0;
            Delay_Ms(200);
            s = InitRootDevice(Com_Buffer);
            if(s == ERR_SUCCESS)
            {
                // U盘操作流程：USB总线复位、U盘连接、获取设备描述符和设置USB地址、可选的获取配置描述符，之后到达此处，由CH103子程序库继续完成后续工作
                CHRV3DiskStatus = DISK_USB_ADDR;
                for(i = 0; i != 10; i++){
                    printf("Wait DiskReady\r\n");
                    s = CHRV3DiskReady(); //等待U盘准备好
                    if(s == ERR_SUCCESS)
                    {
                        break;
                    }
                    else
                    {
                        printf("%02x\r\n", (UINT16)s);
                    }
                    Delay_Ms(50);
                }
                if(CHRV3DiskStatus >= DISK_MOUNTED)
                {
                    /* 读文件 */
                    printf("Open\r\n");
                    strcpy((PCHAR)mCmdParam.Open.mPathName, "/C51/CH103HFT.C"); //设置要操作的文件名和路径
                    s = CHRV3FileOpen();                                        //打开文件
                    if(s == ERR_MISS_DIR)
                    {
                        printf("不存在该文件夹则列出根目录所有文件\r\n");
                        pCodeStr = (PUINT8) "/*";
                    }
                    else
                    {
                        pCodeStr = (PUINT8) "/C51/*"; //列出\C51子目录下的的文件
                    }

                    printf("List file %s\r\n", pCodeStr);
                    for(j = 0; j < 10000; j++){ //限定10000个文件,实际上没有限制
                        strcpy((PCHAR)mCmdParam.Open.mPathName, (PCCHAR)pCodeStr); //搜索文件名,*为通配符,适用于所有文件或者子目录
                        i = strlen((PCHAR)mCmdParam.Open.mPathName);
                        mCmdParam.Open.mPathName[i] = 0xFF; //根据字符串长度将结束符替换为搜索的序号,从0到254,如果是0xFF即255则说明搜索序号在CHRV3vFileSize变量中
                        CHRV3vFileSize = j;                 //指定搜索/枚举的序号
                        i = CHRV3FileOpen();                //打开文件,如果文件名中含有通配符*,则为搜索文件而不打开
                        /* CHRV3FileEnum 与 CHRV3FileOpen 的唯一区别是当后者返回ERR_FOUND_NAME时那么对应于前者返回ERR_SUCCESS */
                        if(i == ERR_MISS_FILE)
                        {
                            break; //再也搜索不到匹配的文件,已经没有匹配的文件名
                        }
                        if(i == ERR_FOUND_NAME) //搜索到与通配符相匹配的文件名,文件名及其完整路径在命令缓冲区中
                        {
                            printf("  match file %04d#: %s\r\n", (unsigned int)j, mCmdParam.Open.mPathName); /* 显示序号和搜索到的匹配文件名或者子目录名 */
                            continue;                                                                        /* 继续搜索下一个匹配的文件名,下次搜索时序号会加1 */
                        }
                        else //出错
                        {
                            mStopIfError(i);
                            break;
                        }
                    }
                    i = CHRV3FileClose(); //关闭文件
                    printf("U盘演示完成\r\n");
                }
                else
                {
                    printf("U盘没有准备好 ERR =%02X\r\n", (UINT16)s);
                }
            }
        }
        Delay_Ms(100);  // 模拟单片机做其它事
        SetUsbSpeed(1); // 默认为全速
    }
}
