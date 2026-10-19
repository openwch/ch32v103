/********************************** (C) COPYRIGHT *******************************
 * File Name          : main.c
 * Author             : WCH
 * Version            : V1.0.2
 * Date               : 2026/10/15
 * Description        : Main program body.
 *********************************************************************************
 * Copyright (c) 2021 Nanjing Qinheng Microelectronics Co., Ltd.
 * Attention: This software (modified or not) and binary are used for 
 * microcontroller manufactured by Nanjing Qinheng Microelectronics.
 *******************************************************************************/

/*
 *@Note
 *External lines trigger ADC conversion routine:
 *ADC channel 2 (PA2) - injection group channel, external trigger pin (PA15)
 *high level triggers EXTI line 15 event,In this mode, an ADC conversion is
 *triggered by an event on EXTI line 15, and a JEOC interrupt is generated
 *after the conversion is completed.
 *Note: Take 3.3V as an example.
 */

#include "debug.h"

/* Global Variable */
vs16 Calibration_Val = 0;
vu8 IntResponse = 0;
vu16 ADC_val;
/*********************************************************************
 * @fn      ADC_Function_Init
 *
 * @brief   Initializes ADC collection.
 *
 * @return  none
 */
void ADC_Function_Init(void)
{
    ADC_InitTypeDef  ADC_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};
    NVIC_InitTypeDef NVIC_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE);
    RCC_ADCCLKConfig(RCC_PCLK2_Div8);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    ADC_DeInit(ADC1);
    ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
    ADC_InitStructure.ADC_ScanConvMode = DISABLE;
    ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;
    ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
    ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
    ADC_InitStructure.ADC_NbrOfChannel = 1;
    ADC_Init(ADC1, &ADC_InitStructure);

    ADC_InjectedSequencerLengthConfig(ADC1, 1);
    ADC_ExternalTrigInjectedConvConfig(ADC1, ADC_ExternalTrigInjecConv_Ext_IT15_TIM8_CC4);
    ADC_InjectedChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_239Cycles5);
    ADC_ExternalTrigInjectedConvCmd(ADC1, ENABLE);

    NVIC_InitStructure.NVIC_IRQChannel = ADC1_2_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);

    ADC_ITConfig(ADC1, ADC_IT_JEOC, ENABLE);
    ADC_Cmd(ADC1, ENABLE);

    ADC_ResetCalibration(ADC1);
    while(ADC_GetResetCalibrationStatus(ADC1));
        
    ADC_StartCalibration(ADC1);
    while(ADC_GetCalibrationStatus(ADC1));
    Calibration_Val = Get_CalibrationValue(ADC1);
}

/*********************************************************************
 * @fn      EXTI_Event_Init
 *
 * @brief   Initializes EXTI.
 *
 * @return  none
 */
void EXTI_Event_Init(void)
{
    EXTI_InitTypeDef EXTI_InitStructure = {0};
    GPIO_InitTypeDef GPIO_InitStructure = {0};

    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);

    GPIO_EXTILineConfig(GPIO_PortSourceGPIOA, GPIO_PinSource15);

    GPIO_InitStructure.GPIO_Pin = GPIO_Pin_15;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;
    GPIO_Init(GPIOA, &GPIO_InitStructure);

    EXTI_InitStructure.EXTI_Line = EXTI_Line15;
    EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Event;
    EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising;
    EXTI_InitStructure.EXTI_LineCmd = ENABLE;
    EXTI_Init(&EXTI_InitStructure);
}

/*********************************************************************
 * @fn      Get_ConversionVal_3_3V
 *
 * @brief   Get Conversion Value Under 3.3V.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal_3_3V(s16 val)
{
    int32_t y;
    y = 6 * (val + Calibration_Val) / 1000 - 12;
    if(val == 0 || val == 4095)
        return val;
    else
    {
        if((val + Calibration_Val - y) < 0)
            return 0;
        if((Calibration_Val + val - y) > 4095||val==4095)
            return 4095;
        return (val + Calibration_Val);
    }
}

/*********************************************************************
 * @fn      Get_ConversionVal_5V
 *
 * @brief   Get Conversion Value Under 5V.
 *
 * @param   val - Sampling value
 *
 * @return  val+Calibrattion_Val - Conversion Value.
 */
u16 Get_ConversionVal_5V(s16 val)
{
    int32_t y;
    y = 4 * (val + Calibration_Val) / 1000 - 8;
    if(val == 0 || val == 4095)
        return val;
    else
    {
        if((val + Calibration_Val - y) < 0)
            return 0;
        if((Calibration_Val + val - y) > 4095||val==4095)
            return 4095;
        return (val + Calibration_Val);
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
    SystemCoreClockUpdate();
    USART_Printf_Init(115200);
    printf("SystemClk:%d\r\n", SystemCoreClock);
    printf( "ChipID:%08x\r\n", DBGMCU_GetCHIPID() );

    ADC_Function_Init();
    printf("CalibrattionValue:%d\n", Calibration_Val);

    EXTI_Event_Init();

    while(1)
    {
        if(IntResponse == 1)
        {
           printf("ADC Extline trigger conversion...\r\n"); 
           printf("JADC%04d\r\n", Get_ConversionVal_3_3V(ADC_val));
           IntResponse = 0;
        }
    }
}

void ADC1_2_IRQHandler(void) __attribute__((interrupt("WCH-Interrupt-fast")));

/*********************************************************************
 * @fn      ADC1_2_IRQHandler
 *
 * @brief   ADC1_2 Interrupt Service Function.
 *
 * @return  none
 */
void ADC1_2_IRQHandler(void)
{

    if(ADC_GetITStatus(ADC1, ADC_IT_JEOC))
    {
        IntResponse = 1;
        ADC_val = ADC_GetInjectedConversionValue(ADC1, ADC_InjectedChannel_1);
    }

    ADC_ClearITPendingBit(ADC1, ADC_IT_JEOC);
}
