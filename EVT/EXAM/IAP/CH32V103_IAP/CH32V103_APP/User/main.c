/********************************** (C) COPYRIGHT *******************************
* File Name          : main.c
* Author             : WCH
* Version            : V1.0.0
* Date               : 2020/04/30
* Description        : Main program body.
*******************************************************************************
* Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
* Attention: This software (modified or not) and binary are used for 
* microcontroller manufactured by Nanjing Qinheng Microelectronics.
*******************************************************************************/
#include "debug.h"

/*@Note
 *bootloader:0x08000000 - 0x08005000  20K
 *UserCode:  0x08005000 - 0x08010000  44K
 *
 */


/*******************************************************************************
* Function Name  : GPIO_Toggle_INIT
* Description    : Initializes GPIOA.0
* Input          : None
* Return         : None
*******************************************************************************/
void GPIO_PA2(void)
{
  GPIO_InitTypeDef  GPIO_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA,ENABLE);
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed=GPIO_Speed_50MHz;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
}


/*******************************************************************************
* Function Name  : main
* Description    : Main program.
* Input          : None
* Return         : None
*******************************************************************************/
int main(void)
{
	u16 i;
	SystemCoreClockUpdate();
	Delay_Init();
	USART_Printf_Init(115200);
	GPIO_PA2();

	printf("UserCode test start...\r\n");
	for(i=0; i<10; i++){
        printf("UserCode test%d\r\n", i);
        GPIO_SetBits(GPIOA, GPIO_Pin_2);
        Delay_Ms(200);
        GPIO_ResetBits(GPIOA, GPIO_Pin_2);
        Delay_Ms(300);
	}

	printf("Soft RESET...\r\n");
	printf("\r\n");
	Delay_Ms(800);

	//soft - 0x00 run t0 BootLoader
	NVIC_SystemReset();

	while(1);
}







