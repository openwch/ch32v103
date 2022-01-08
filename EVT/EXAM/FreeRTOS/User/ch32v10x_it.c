/********************************** (C) COPYRIGHT *******************************
 * File Name          : ch32v10x_it.c
 * Author             : WCH
 * Version            : V1.0.0
 * Date               : 2020/04/30
 * Description        : Main Interrupt Service Routines.
 *******************************************************************************/
#include "ch32v10x_it.h"
#include "Config.h"
//#include "riscv_encoding.h"

void NMI_Handler(void) __attribute__((interrupt()));
//void HardFault_Handler(void) __attribute__((interrupt()));
//void TIM4_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void ADC1_2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void USART1_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//void SW_handler(void) __attribute__((interrupt("WCH-Interrupt-fast")));
//volatile uint16_t ADC_val;
/*********************************************************************
 * @fn      SW_handler
 *
 * @brief   This function handles SW exception.
 *
 * @return  none
 */
//void SW_handler(void)
//{
//    sysPar.sysCounter++;
//    NVIC_ClearPendingIRQ(Software_IRQn);
//}
/*********************************************************************
 * @fn      NMI_Handler
 *
 * @brief   This function handles NMI exception.
 *
 * @return  none
 */
void NMI_Handler(void)
{
    ;
}
/*********************************************************************
 * @fn      HardFault_Handler
 *
 * @brief   This function handles Hard Fault exception.
 *
 * @return  none
 */
//void HardFault_Handler(void)
//{
//  while (1)
//  {
//      ;
//  }
//    //---sp+4---
//    //unsigned long epc = read_csr(mepc);
//    //write_csr(mepc, epc+4);
//    //NVIC->CFGR = 0xBCAF0020;
//}
/*********************************************************************
 * @fn       TIM4_IRQHandler
 *
 * @brief   TIM4中断服务函数(sysPar.sysCounter计数作为系统调度用)
 *
 * @return  none
 */
//void TIM4_IRQHandler(void)
//{
//    if( TIM_GetITStatus( TIM4, TIM_IT_Update) != RESET )
//    {
//        sysPar.sysCounter++;
//    }
//    TIM_ClearITPendingBit( TIM4, TIM_IT_Update);    //清除中断标志位
//}
