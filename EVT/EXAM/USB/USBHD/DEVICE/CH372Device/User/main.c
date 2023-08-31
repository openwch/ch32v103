/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2022/08/20
 * Description        : Main program body.
*********************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/

/*
 *@Note
  Example routines for emulating custom USB devices (CH372 devices).
  This routine demonstrates the use of the USBHD-FS to emulate a custom device,
  the CH372, with endpoints 1/3/5 downlinked and uploaded via endpoints 2/4/6 respectively
  where endpoint 1/2 is implemented via a ring buffer and the data is not inverted,
  and endpoints 3/4 and 5/6 are directly copied and inverted for upload.
  The device can be operated using Bushund or other upper computer software.
  Note: This routine needs to be demonstrated in conjunction with the host software.
*/

#include "ch32v10x_usbfs_device.h"
#include "debug.h"

/*********************************************************************
 * @fn      Var_Init
 *
 * @brief   Software parameter initialisation
 *
 * @return  none
 */
void Var_Init(void)
{
    uint16_t i;
    RingBuffer_Comm.LoadPtr = 0;
    RingBuffer_Comm.StopFlag = 0;
    RingBuffer_Comm.DealPtr = 0;
    RingBuffer_Comm.RemainPack = 0;
    for(i=0; i<DEF_Ring_Buffer_Max_Blks; i++)
    {
        RingBuffer_Comm.PackLen[i] = 0;
    }
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
    uint8_t ret;
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);
	printf("USBHD Device Test\r\n");

	/* Variables init */
	Var_Init( );

	/* Usb Init */
	USBHD_RCC_Init( );
    USBHD_Device_Init( ENABLE , PWR_VDD_SupplyVoltage());

	while(1)
	{
	    /* Determine if enumeration is complete, perform data transfer if completed */
	    if( USBHD_DevEnumStatus )
	    {
	        /* Data Transfer */
	        if( RingBuffer_Comm.RemainPack )
	        {
	            ret = USBHD_Endp_DataUp(DEF_UEP2, &Data_Buffer[(RingBuffer_Comm.DealPtr) * DEF_USBD_FS_PACK_SIZE], RingBuffer_Comm.PackLen[RingBuffer_Comm.DealPtr], DEF_UEP_DMA_LOAD);
	            if( ret == 0 )
	            {
	                NVIC_DisableIRQ(USBHD_IRQn);
	                RingBuffer_Comm.RemainPack--;
	                RingBuffer_Comm.DealPtr++;
	                if(RingBuffer_Comm.DealPtr == DEF_Ring_Buffer_Max_Blks)
	                {
	                    RingBuffer_Comm.DealPtr = 0;
	                }
	                NVIC_EnableIRQ(USBHD_IRQn);
	            }
	        }

	        /* Monitor whether the remaining space is available for further downloads */
	        if(RingBuffer_Comm.RemainPack < (DEF_Ring_Buffer_Max_Blks - DEF_RING_BUFFER_RESTART))
	        {
	            if(RingBuffer_Comm.StopFlag)
	            {
	                RingBuffer_Comm.StopFlag = 0;
	                R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_R_RES) | UEP_R_RES_ACK;
	            }
	        }
	    }
	}
}
