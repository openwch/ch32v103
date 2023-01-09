/*
 * Copyright (c) 2013-2020, Huawei Technologies Co., Ltd. All rights reserved.
 * Copyright (c) 2020-2021 Huawei Device Co., Ltd. All rights reserved.
 * Copyright (c) 2021 Nuclei Limited. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 *    conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 *    of conditions and the following disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 *    to endorse or promote products derived from this software without specific prior written
 *    permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "los_timer.h"
#include "los_config.h"
#include "los_tick.h"
#include "los_reg.h"
#include "los_sched.h"
#include "core_riscv.h"
#include "los_arch_interrupt.h"
#include "los_arch_timer.h"


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

#define configKERNEL_INTERRUPT_PRIORITY         0

//#define SYSTICK_TICK_CONST  (OS_SYS_CLOCK / LOSCFG_BASE_CORE_TICK_PER_SECOND)

static OS_TICK_HANDLER systick_handler = (OS_TICK_HANDLER)NULL;

extern UINT32 g_intCount;

WEAK UINT32 HalTickStart(OS_TICK_HANDLER handler)
{
    g_sysClock = OS_SYS_CLOCK;
    g_cyclesPerTick = g_sysClock/LOSCFG_BASE_CORE_TICK_PER_SECOND;
    g_intCount = 0;

    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_EnableIRQ(Software_IRQn);
    NVIC_SetPriority(SysTicK_IRQn,0xff);
    NVIC_SetPriority(Software_IRQn,0xff);
    systick_handler = handler;

    SysTick->CTLR=0;
    SysTick->CNTL0=0;SysTick->CNTL1=0;SysTick->CNTL2=0;SysTick->CNTL3=0;
    SysTick->CNTH0=0;SysTick->CNTH1=0;SysTick->CNTH2=0;SysTick->CNTH3=0;

    SysTick->CMPLR0=(UINT8)(g_cyclesPerTick-1);
    SysTick->CMPLR1=(UINT8)((g_cyclesPerTick-1)>>8);
    SysTick->CMPLR2=(UINT8)((g_cyclesPerTick-1)>>16);
    SysTick->CMPLR3=(UINT8)((g_cyclesPerTick-1)>>24);

    SysTick->CMPHR0=0;SysTick->CMPHR1=0;SysTick->CMPHR2=0;SysTick->CMPHR3=0;
    SysTick->CTLR=0x1;

    return LOS_OK; /* never return */
}



void SysTick_Handler(void) __attribute__((interrupt()));
#define HalTickSysTickHandler  SysTick_Handler
void HalTickSysTickHandler( void )
{
    /* Do systick handler registered in HalTickStart. */
    if ((void *)systick_handler != NULL) {
        systick_handler();
    }

}

WEAK VOID HalSysTickReload(UINT64 nextResponseTime)
{
    UINT32 tickL=0,tickH=0;
    SysTick->CTLR &= ~(1<<0);

    tickL=(UINT32)(nextResponseTime-1);
    tickH=(UINT32)((nextResponseTime-1)>>32);

    SysTick->CNTL0=0;SysTick->CNTL1=0;SysTick->CNTL2=0;SysTick->CNTL3=0;
    SysTick->CNTH0=0;SysTick->CNTH1=0;SysTick->CNTH2=0;SysTick->CNTH3=0;

    SysTick->CMPLR0=(UINT8)(tickL);
    SysTick->CMPLR1=(UINT8)(tickL>>8);
    SysTick->CMPLR2=(UINT8)(tickL>>16);
    SysTick->CMPLR3=(UINT8)(tickL>>24);

    SysTick->CMPHR0=(UINT8)(tickH);
    SysTick->CMPHR1=(UINT8)(tickH>>8);
    SysTick->CMPHR2=(UINT8)(tickH>>16);
    SysTick->CMPHR3=(UINT8)(tickH>>24);
    NVIC_ClearPendingIRQ(SysTicK_IRQn);

    SysTick->CTLR |= (1<<0);
}

WEAK UINT64 HalGetTickCycle(UINT64 *period)
{
    UINT64 ticks=0;
    UINT32 intSave = LOS_IntLock();

    ticks= (*(__IO UINT32 *)0xe000f008);
    ticks= (ticks<<32) | (*(__IO UINT32 *)0xe000f004);


    *period = (*(__IO UINT64*)0xe000f00c);

    LOS_IntRestore(intSave);
    return ticks;
}

WEAK VOID HalTickLock(VOID)
{
    SysTick->CTLR &= ~(1<<0);
}

WEAK VOID HalTickUnlock(VOID)
{
    SysTick->CTLR |= (1<<0);
}

UINT32 HalEnterSleep(VOID)
{
   /* need sleep ? */
//    __WFI();

    return LOS_OK;
}

#ifdef __cplusplus
#if __cplusplus
}
#endif /* __cplusplus */
#endif /* __cplusplus */
