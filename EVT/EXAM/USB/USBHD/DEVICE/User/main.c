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
 模拟自定义USB设备（CH372设备）例程：
 USBHDM(PA11)、USBHDP(PA12)。
 本例程演示使用 USBD 模拟自定义设备 CH372，和上位机通信。
 
 注：本例程需与上位机软件配合演示。

*/

#include "debug.h"
#include "string.h"

/* Global define */


/* Global Variable */
#define DevEP0SIZE	0x40

/* Device Descriptor */
const UINT8  MyDevDescr[] = {
	0x12, 0x01, 0x10, 0x01,0xFF, 0x80, 0x55, DevEP0SIZE,
	0x48, 0x43, 0x37, 0x55,
	0x00, 0x01, 0x01, 0x02, 0x00, 0x01,
};

/* Configration Descriptor */
const UINT8  MyCfgDescr[] = {
	0x09, 0x02, 0x74, 0x00, 0x01, 0x01, 0x00, 0x80, 0x32,
	0x09, 0x04, 0x00, 0x00, 0x0E, 0xFF, 0x80, 0x55, 0x00,
	0x07, 0x05, 0x87, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x07, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x86, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x06, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x85, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x05, 0x02, 0x40, 0x00, 0x00,	
	0x07, 0x05, 0x84, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x04, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x83, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x03, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x82, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x02, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x81, 0x02, 0x40, 0x00, 0x00,
	0x07, 0x05, 0x01, 0x02, 0x40, 0x00, 0x00,
};

/* Language Descriptor */
const UINT8  MyLangDescr[] = { 0x04, 0x03, 0x09, 0x04 };

/* Manufactor Descriptor */
const UINT8  MyManuInfo[] = { 0x0E, 0x03, 'w', 0, 'c', 0, 'h', 0, '.', 0, 'c', 0, 'n', 0 };

/* Product Information */
const UINT8  MyProdInfo[] = { 0x0C, 0x03, 'C', 0, 'H', 0, '1', 0, '0', 0, 'x', 0 };

/**********************************************************/
UINT8   DevConfig;
UINT8   SetupReqCode;
UINT16  SetupReqLen;
const UINT8 *pDescr;

/* Endpoint Buffer */
__attribute__ ((aligned(4))) UINT8 EP0_Databuf[64];	//ep0(64)
__attribute__ ((aligned(4))) UINT8 EP1_Databuf[64+64];	//ep1_out(64)+ep1_in(64)
__attribute__ ((aligned(4))) UINT8 EP2_Databuf[64+64];	//ep2_out(64)+ep2_in(64)
__attribute__ ((aligned(4))) UINT8 EP3_Databuf[64+64];	//ep3_out(64)+ep3_in(64)
__attribute__ ((aligned(4))) UINT8 EP4_Databuf[64+64];	//ep4_out(64)+ep4_in(64)
__attribute__ ((aligned(4))) UINT8 EP5_Databuf[64+64];	//ep5_out(64)+ep5_in(64)
__attribute__ ((aligned(4))) UINT8 EP6_Databuf[64+64];	//ep6_out(64)+ep6_in(64)
__attribute__ ((aligned(4))) UINT8 EP7_Databuf[64+64];	//ep7_out(64)+ep7_in(64)

void USBHD_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      Set_USBConfig
 *
 * @brief   Set USB clock.
 *
 * @return  none
 */
void USBHD_ClockCmd(UINT32 RCC_USBCLKSource,FunctionalState NewState)
{
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, NewState);
	EXTEN->EXTEN_CTR |= EXTEN_USBHD_IO_EN;
	RCC_USBCLKConfig(RCC_USBCLKSource);             //USBclk=PLLclk/1.5=48Mhz
	RCC_AHBPeriphClockCmd(RCC_AHBPeriph_USBHD,NewState);
}

/*********************************************************************
 * @fn      USB_DevTransProcess
 *
 * @brief   USB device transfer process.
 *
 * @return  none
 */
void USB_DevTransProcess( void )
{
	UINT8  len, chtype;
	UINT8  intflag, errflag = 0;

	intflag = R8_USB_INT_FG;

	if( intflag & RB_UIF_TRANSFER )
	{
		switch ( R8_USB_INT_ST & MASK_UIS_TOKEN)
		{
			case UIS_TOKEN_SETUP:
				R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_NAK;
				len = R8_USB_RX_LEN;

				if ( len == sizeof( USB_SETUP_REQ ) )
				{
					SetupReqLen = pSetupReqPak->wLength;
					SetupReqCode = pSetupReqPak->bRequest;
					chtype = pSetupReqPak->bRequestType;

					len = 0;
					errflag = 0;
					if ( ( pSetupReqPak->bRequestType & USB_REQ_TYP_MASK ) != USB_REQ_TYP_STANDARD )
					{
						errflag = 0xFF;
					}
					else
					{
						switch( SetupReqCode )
						{
							case USB_GET_DESCRIPTOR:
							{
								switch( ((pSetupReqPak->wValue)>>8) )
								{
									case USB_DESCR_TYP_DEVICE:
										pDescr = MyDevDescr;
										len = MyDevDescr[0];
										break;

									case USB_DESCR_TYP_CONFIG:
										pDescr = MyCfgDescr;
										len = MyCfgDescr[2];
										break;

									case USB_DESCR_TYP_STRING:
										switch( (pSetupReqPak->wValue)&0xff )
										{
											case 1:
												pDescr = MyManuInfo;
												len = MyManuInfo[0];
												break;

											case 2:
												pDescr = MyProdInfo;
												len = MyProdInfo[0];
												break;

											case 0:
												pDescr = MyLangDescr;
												len = MyLangDescr[0];
												break;

											default:
												errflag = 0xFF;
												break;
										}
										break;

									default :
										errflag = 0xff;
										break;
								}

								if( SetupReqLen>len )	SetupReqLen = len;
								len = (SetupReqLen >= DevEP0SIZE) ? DevEP0SIZE : SetupReqLen;
								memcpy( pEP0_DataBuf, pDescr, len );
								pDescr += len;
							}
								break;

							case USB_SET_ADDRESS:
								SetupReqLen = (pSetupReqPak->wValue)&0xff;
								break;

							case USB_GET_CONFIGURATION:
								pEP0_DataBuf[0] = DevConfig;
								if ( SetupReqLen > 1 ) SetupReqLen = 1;
								break;

							case USB_SET_CONFIGURATION:
								DevConfig = (pSetupReqPak->wValue)&0xff;
								break;

							case USB_CLEAR_FEATURE:
								if ( ( pSetupReqPak->bRequestType & USB_REQ_RECIP_MASK ) == USB_REQ_RECIP_ENDP )
								{
									switch( (pSetupReqPak->wIndex)&0xff )
									{
									case 0x82:
										R8_UEP2_CTRL = (R8_UEP2_CTRL & ~( RB_UEP_T_TOG|MASK_UEP_T_RES )) | UEP_T_RES_NAK;
										break;

									case 0x02:
										R8_UEP2_CTRL = (R8_UEP2_CTRL & ~( RB_UEP_R_TOG|MASK_UEP_R_RES )) | UEP_R_RES_ACK;
										break;

									case 0x81:
										R8_UEP1_CTRL = (R8_UEP1_CTRL & ~( RB_UEP_T_TOG|MASK_UEP_T_RES )) | UEP_T_RES_NAK;
										break;

									case 0x01:
										R8_UEP1_CTRL = (R8_UEP1_CTRL & ~( RB_UEP_R_TOG|MASK_UEP_R_RES )) | UEP_R_RES_ACK;
										break;

									default:
										errflag = 0xFF;
										break;

									}
								}
								else	errflag = 0xFF;
								break;

							case USB_GET_INTERFACE:
								pEP0_DataBuf[0] = 0x00;
								if ( SetupReqLen > 1 ) SetupReqLen = 1;
								break;

							case USB_GET_STATUS:
								pEP0_DataBuf[0] = 0x00;
								pEP0_DataBuf[1] = 0x00;
								if ( SetupReqLen > 2 ) SetupReqLen = 2;
								break;

							default:
								errflag = 0xff;
								break;
						}
					}
				}
				else	errflag = 0xff;

				if( errflag == 0xff)
				{
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_STALL | UEP_T_RES_STALL;
				}
				else
				{
					if( chtype & 0x80 )
					{
						len = (SetupReqLen>DevEP0SIZE) ? DevEP0SIZE : SetupReqLen;
						SetupReqLen -= len;
					}
					else  len = 0;

					R8_UEP0_T_LEN = len;
					R8_UEP0_CTRL = RB_UEP_R_TOG | RB_UEP_T_TOG | UEP_R_RES_ACK | UEP_T_RES_ACK;
				}
				break;

			case UIS_TOKEN_IN:
				switch ( R8_USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
				{
					case UIS_TOKEN_IN:
						switch( SetupReqCode )
						{
							case USB_GET_DESCRIPTOR:
									len = SetupReqLen >= DevEP0SIZE ? DevEP0SIZE : SetupReqLen;
									memcpy( pEP0_DataBuf, pDescr, len );
									SetupReqLen -= len;
									pDescr += len;
									R8_UEP0_T_LEN = len;
									R8_UEP0_CTRL ^= RB_UEP_T_TOG;
									break;

							case USB_SET_ADDRESS:
									R8_USB_DEV_AD = (R8_USB_DEV_AD&RB_UDA_GP_BIT) | SetupReqLen;
									R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
									break;

							default:
									R8_UEP0_T_LEN = 0;
									R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
									break;

						}
						break;

				case UIS_TOKEN_IN | 1:
					R8_UEP1_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP1_CTRL = (R8_UEP1_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
					break;

				case UIS_TOKEN_IN | 2:
					R8_UEP2_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP2_CTRL = (R8_UEP2_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
					break;

				case UIS_TOKEN_IN | 3:
					R8_UEP3_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP3_CTRL = (R8_UEP3_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
					break;

				case UIS_TOKEN_IN | 4:
					R8_UEP4_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP4_CTRL = (R8_UEP4_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
					break;

				case UIS_TOKEN_IN | 5:
					R8_UEP5_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP5_CTRL = (R8_UEP5_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
					break;

				case UIS_TOKEN_IN | 6:
					R8_UEP6_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP6_CTRL = (R8_UEP6_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
					break;

				case UIS_TOKEN_IN | 7:
					R8_UEP7_CTRL ^=  RB_UEP_T_TOG;
					R8_UEP7_CTRL = (R8_UEP7_CTRL & ~MASK_UEP_T_RES) | UEP_T_RES_NAK;
					break;

				default :
					break;

				}
				break;

			case UIS_TOKEN_OUT:
				switch ( R8_USB_INT_ST & ( MASK_UIS_TOKEN | MASK_UIS_ENDP ) )
				{
					case UIS_TOKEN_OUT:
							len = R8_USB_RX_LEN;
							break;

					case UIS_TOKEN_OUT | 1:
						if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
						{
							R8_UEP1_CTRL ^= RB_UEP_R_TOG;
							len = R8_USB_RX_LEN;
							DevEP1_OUT_Deal( len );
						}
						break;

					case UIS_TOKEN_OUT | 2:
						if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
						{
							R8_UEP2_CTRL ^= RB_UEP_R_TOG;
							len = R8_USB_RX_LEN;
							DevEP2_OUT_Deal( len );
						}
						break;

					case UIS_TOKEN_OUT | 3:
						if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
						{
							R8_UEP3_CTRL ^= RB_UEP_R_TOG;
							len = R8_USB_RX_LEN;
							DevEP3_OUT_Deal( len );
						}
						break;

					case UIS_TOKEN_OUT | 4:
						if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
						{
							R8_UEP4_CTRL ^= RB_UEP_R_TOG;
							len = R8_USB_RX_LEN;
							DevEP4_OUT_Deal( len );
						}
						break;

					case UIS_TOKEN_OUT | 5:
						if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
						{
							R8_UEP5_CTRL ^= RB_UEP_R_TOG;
							len = R8_USB_RX_LEN;
							DevEP5_OUT_Deal( len );
						}
						break;

					case UIS_TOKEN_OUT | 6:
						if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
						{
							R8_UEP6_CTRL ^= RB_UEP_R_TOG;
							len = R8_USB_RX_LEN;
							DevEP6_OUT_Deal( len );
						}
						break;

					case UIS_TOKEN_OUT | 7:
						if ( R8_USB_INT_ST & RB_UIS_TOG_OK )
						{
							R8_UEP7_CTRL ^= RB_UEP_R_TOG;
							len = R8_USB_RX_LEN;
							DevEP7_OUT_Deal( len );
						}
						break;
				}

				break;

			case UIS_TOKEN_SOF:

				break;

			default :
				break;

		}


		R8_USB_INT_FG = RB_UIF_TRANSFER;
	}
	else if( intflag & RB_UIF_BUS_RST )
	{
		R8_USB_DEV_AD = 0;
		R8_UEP0_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP1_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP2_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP3_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP4_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP5_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP6_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;
		R8_UEP7_CTRL = UEP_R_RES_ACK | UEP_T_RES_NAK;		
		
		R8_USB_INT_FG |= RB_UIF_BUS_RST;
	}
	else if( intflag & RB_UIF_SUSPEND )
	{
		if ( R8_USB_MIS_ST & RB_UMS_SUSPEND ) {;}
		else{;}
		R8_USB_INT_FG = RB_UIF_SUSPEND;
	}
	else
	{
		R8_USB_INT_FG = intflag;
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
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
	Delay_Init();
	USART_Printf_Init(115200);
	printf("SystemClk:%d\r\n",SystemCoreClock);

	printf("USBHD Device Test\r\n");
	pEP0_RAM_Addr = EP0_Databuf;
	pEP1_RAM_Addr = EP1_Databuf;
	pEP2_RAM_Addr = EP2_Databuf;
	pEP3_RAM_Addr = EP3_Databuf;
	pEP4_RAM_Addr = EP4_Databuf;
	pEP5_RAM_Addr = EP5_Databuf;
	pEP6_RAM_Addr = EP6_Databuf;
	pEP7_RAM_Addr = EP7_Databuf;	
	
	USBHD_ClockCmd(RCC_USBCLKSource_PLLCLK_1Div5,ENABLE);
	USB_DeviceInit();
	NVIC_EnableIRQ(USBHD_IRQn);

	while(1)
  {

  }

}

/*********************************************************************
 * @fn      DevEP1_OUT_Deal
 *
 * @brief   Deal device Endpoint 1 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void DevEP1_OUT_Deal( UINT8 l )
{
	UINT8 i;

	for(i=0; i<l; i++)
	{
		pEP1_IN_DataBuf[i] = ~pEP1_OUT_DataBuf[i];
	}

	DevEP1_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP2_OUT_Deal
 *
 * @brief   Deal device Endpoint 2 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void DevEP2_OUT_Deal( UINT8 l )
{
	UINT8 i;

	for(i=0; i<l; i++)
	{
		pEP2_IN_DataBuf[i] = ~pEP2_OUT_DataBuf[i];
	}

	DevEP2_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP3_OUT_Deal
 *
 * @brief   Deal device Endpoint 3 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void DevEP3_OUT_Deal( UINT8 l )
{
	UINT8 i;

	for(i=0; i<l; i++)
	{
		pEP3_IN_DataBuf[i] = ~pEP3_OUT_DataBuf[i];
	}

	DevEP3_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP4_OUT_Deal
 *
 * @brief   Deal device Endpoint 4 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void DevEP4_OUT_Deal( UINT8 l )
{
	UINT8 i;

	for(i=0; i<l; i++)
	{
		pEP4_IN_DataBuf[i] = ~pEP4_OUT_DataBuf[i];
	}

	DevEP4_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP5_OUT_Deal
 *
 * @brief   Deal device Endpoint 5 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void DevEP5_OUT_Deal( UINT8 l )
{
	UINT8 i;

	for(i=0; i<l; i++)
	{
		pEP5_IN_DataBuf[i] = ~pEP5_OUT_DataBuf[i];
	}

	DevEP5_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP6_OUT_Deal
 *
 * @brief   Deal device Endpoint 6 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void DevEP6_OUT_Deal( UINT8 l )
{
	UINT8 i;

	for(i=0; i<l; i++)
	{
		pEP6_IN_DataBuf[i] = ~pEP6_OUT_DataBuf[i];
	}

	DevEP6_IN_Deal( l );
}

/*********************************************************************
 * @fn      DevEP7_OUT_Deal
 *
 * @brief   Deal device Endpoint 7 OUT.
 *
 * @param   l: Data length.
 *
 * @return  none
 */
void DevEP7_OUT_Deal( UINT8 l )
{
	UINT8 i;

	for(i=0; i<l; i++)
	{
		pEP7_IN_DataBuf[i] = ~pEP7_OUT_DataBuf[i];
	}

	DevEP7_IN_Deal( l );
}

/*********************************************************************
 * @fn      USB_IRQHandler
 *
 * @brief   This function handles USB exception.
 *
 * @return  none
 */
void USBHD_IRQHandler (void)
{
	USB_DevTransProcess();
}

