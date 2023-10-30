/*----------------------------------------------------------------------------
 * Tencent is pleased to support the open source community by making TencentOS
 * available.
 *
 * Copyright (C) 2019 THL A29 Limited, a Tencent company. All rights reserved.
 * If you have downloaded a copy of the TencentOS binary from Tencent, please
 * note that the TencentOS binary is licensed under the BSD 3-Clause License.
 *
 * If you have downloaded a copy of the TencentOS source code from Tencent,
 * please note that TencentOS source code is licensed under the BSD 3-Clause
 * License, except for the third-party components listed below which are
 * subject to different license terms. Your integration of TencentOS into your
 * own projects may require compliance with the BSD 3-Clause License, as well
 * as the other licenses applicable to the third-party components included
 * within TencentOS.
 *---------------------------------------------------------------------------*/

#include <tos_k.h>
#include "ch32v10x.h"
#include "core_riscv.h"



__PORT__ void port_int_disable(void)
{
    asm("csrw mstatus, %0" : :"r"(0x1800));
}

__PORT__ void port_int_enable(void)
{
    asm("csrw mstatus, %0" : :"r"(0x1888));
}

__PORT__ cpu_cpsr_t port_cpsr_save(void)
{
    cpu_cpsr_t value=0;
    asm("csrrw %0, mstatus, %1":"=r"(value):"r"(0x1800));
    return value;
}
__PORT__ void port_cpsr_restore(cpu_cpsr_t cpsr)
{
  asm("csrw mstatus, %0": :"r"(cpsr));
}


__PORT__ void port_cpu_reset(void)
{
    NVIC_SystemReset();
}

/* clear soft interrupt */
__PORT__ void sw_clearpend(void)
{
    NVIC_ClearPendingIRQ(Software_IRQn);
}

/* trigger software interrupt */
__PORT__ void port_context_switch(void)
{
    NVIC_SetPendingIRQ(Software_IRQn);
}

/* trigger software interrupt */
__PORT__ void port_irq_context_switch(void)
{
    NVIC_SetPendingIRQ(Software_IRQn);
}

__PORT__ void port_systick_config(uint32_t cycle_per_tick)
{
    SysTick->CTLR=0;

    SysTick->CNTL0=0;
    SysTick->CNTL1=0;
    SysTick->CNTL2=0;
    SysTick->CNTL3=0;
    SysTick->CNTH0=0;
    SysTick->CNTH1=0;
    SysTick->CNTH2=0;
    SysTick->CNTH3=0;
    SysTick->CMPLR0=(uint8_t)(cycle_per_tick-1);
    SysTick->CMPLR1=(uint8_t)((cycle_per_tick-1)>>8);
    SysTick->CMPLR2=(uint8_t)((cycle_per_tick-1)>>16);
    SysTick->CMPLR3=(uint8_t)((cycle_per_tick-1)>>24);

    SysTick->CMPHR0=0;
    SysTick->CMPHR1=0;
    SysTick->CMPHR2=0;
    SysTick->CMPHR3=0;

    SysTick->CTLR=0x1;
}

__PORT__ void port_systick_priority_set(uint32_t prio)
{
    NVIC_SetPriority(SysTicK_IRQn, prio);
}

__PORT__ void port_cpu_init()
{
    /* V103 need to disable HPE */
    NVIC_HaltPushCfg(DISABLE);
    NVIC_SetPriority(Software_IRQn,0xf0);
    NVIC_EnableIRQ(SysTicK_IRQn);
    NVIC_EnableIRQ(Software_IRQn);
}


void SysTick_Handler(void) __attribute__((interrupt()));
void SysTick_Handler(void)
{
    GET_INT_SP();   /* 切换中断栈 */
    SysTick->CTLR=0x0;
    SysTick->CNTL0=0;
    SysTick->CNTL1=0;
    SysTick->CNTL2=0;
    SysTick->CNTL3=0;
    SysTick->CNTH0=0;
    SysTick->CNTH1=0;
    SysTick->CNTH2=0;
    SysTick->CNTH3=0;
    SysTick->CTLR=0x1;
    if (tos_knl_is_running())
    {
      tos_knl_irq_enter();
      tos_tick_handler();
      tos_knl_irq_leave();
    }
    FREE_INT_SP(); /* 释放中断栈 */
}


#if TOS_CFG_TICKLESS_EN > 0u
__PORT__ k_time_t port_systick_max_delay_millisecond(void)
{
    k_time_t max_millisecond;
    uint32_t max_cycle;

    max_cycle = 0xffffffff; // systick 是64位，这里用低32位
    /* V103 Systick clock is HCLK/8 */
    max_millisecond = (k_time_t)((uint64_t)max_cycle * K_TIME_MILLISEC_PER_SEC / TOS_CFG_CPU_CLOCK * 8); // CLOCK: cycle per second
    return max_millisecond;
}

__PORT__ void port_systick_resume(void)
{
    SysTick->CTLR |= 1;
}

__PORT__ void port_systick_suspend(void)
{
    SysTick->CTLR &= ~(1<<0);
}

__PORT__ k_cycle_t port_systick_max_reload_cycle(void)
{
    return 0xffffffff;
}

__PORT__ void port_systick_reload(uint32_t cycle_per_tick)
{
    port_systick_config(cycle_per_tick);
}

__PORT__ void port_systick_pending_reset(void)
{
    PFIC->IPRR[0] |= (1<<12); //clear pend
}

#endif


#if TOS_CFG_PWR_MGR_EN > 0u
__PORT__ void port_sleep_mode_enter(void)
{
    /* only CPU sleep */
    PFIC->SCTLR |= (1<<2);
    __WFI();
    PFIC->SCTLR &= ~(1<<2);
}

__PORT__ void port_stop_mode_enter(void)
{
    PWR_EnterSTOPMode(PWR_Regulator_ON, PWR_STOPEntry_WFI);
}

__PORT__ void port_standby_mode_enter(void)
{
    PWR_EnterSTANDBYMode();
}

#endif







