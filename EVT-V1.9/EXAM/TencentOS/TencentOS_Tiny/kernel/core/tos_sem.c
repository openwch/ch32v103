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

#include "tos_k.h"

#if TOS_CFG_SEM_EN > 0u

__API__ k_err_t tos_sem_create_max(k_sem_t *sem, k_sem_cnt_t init_count, k_sem_cnt_t max_count)
{
    TOS_PTR_SANITY_CHECK(sem);
    
    if (unlikely(init_count > max_count)) {
        init_count = max_count;
    }
    
    sem->count      = init_count;
    sem->count_max  = max_count;
    
    pend_object_init(&sem->pend_obj);
    TOS_OBJ_INIT(sem, KNL_OBJ_TYPE_SEMAPHORE);
    
    knl_object_alloc_set_static(&sem->knl_obj);

    return K_ERR_NONE;
}

__API__ k_err_t tos_sem_create(k_sem_t *sem, k_sem_cnt_t init_count)
{
    return tos_sem_create_max(sem, init_count, (k_sem_cnt_t)-1);
}

__API__ k_err_t tos_sem_create_max_dyn(k_sem_t **sem, k_sem_cnt_t init_count, k_sem_cnt_t max_count)
{
    k_sem_t *the_sem;

    TOS_IN_IRQ_CHECK();
    TOS_PTR_SANITY_CHECK(sem);

    if (unlikely(init_count > max_count)) {
        init_count = max_count;
    }

    the_sem = tos_mmheap_calloc(1, sizeof(k_sem_t));
    if (!the_sem) {
        return K_ERR_SEM_OUT_OF_MEMORY;
    }

    the_sem->count = init_count;
    the_sem->count_max = max_count;

    pend_object_init(&the_sem->pend_obj);
    TOS_OBJ_INIT(the_sem, KNL_OBJ_TYPE_SEMAPHORE);
    
    knl_object_alloc_set_dynamic(&the_sem->knl_obj);

    *sem = the_sem;

    return K_ERR_NONE;
}

__API__ k_err_t tos_sem_create_dyn(k_sem_t **sem, k_sem_cnt_t init_count)
{
    return tos_sem_create_max_dyn(sem, init_count, (k_sem_cnt_t)-1);
}

__API__ k_err_t tos_sem_destroy(k_sem_t *sem)
{
    TOS_CPU_CPSR_ALLOC();

    TOS_PTR_SANITY_CHECK(sem);
    TOS_OBJ_VERIFY(sem, KNL_OBJ_TYPE_SEMAPHORE);

    TOS_CPU_INT_DISABLE();

    pend_wakeup_all(&sem->pend_obj, PEND_STATE_DESTROY);

    pend_object_deinit(&sem->pend_obj);

    if (knl_object_alloc_is_dynamic(&sem->knl_obj)) {
        TOS_OBJ_DEINIT(sem);
        tos_mmheap_free(sem);
    } else {
        TOS_OBJ_DEINIT(sem);
    }

    TOS_CPU_INT_ENABLE();
    knl_sched();

    return K_ERR_NONE;
}

__STATIC__ k_err_t sem_do_post(k_sem_t *sem, opt_post_t opt)
{
    TOS_CPU_CPSR_ALLOC();

    TOS_PTR_SANITY_CHECK(sem);
    TOS_OBJ_VERIFY(sem, KNL_OBJ_TYPE_SEMAPHORE);

    TOS_CPU_INT_DISABLE();

    if (sem->count == sem->count_max) {
        TOS_CPU_INT_ENABLE();
        return K_ERR_SEM_OVERFLOW;
    }

    if (pend_is_nopending(&sem->pend_obj)) {
        ++sem->count;
        TOS_CPU_INT_ENABLE();
        return K_ERR_NONE;
    }

    pend_wakeup(&sem->pend_obj, PEND_STATE_POST, opt);

    TOS_CPU_INT_ENABLE();
    knl_sched();

    return K_ERR_NONE;
}

__API__ k_err_t tos_sem_post(k_sem_t *sem)
{
    return sem_do_post(sem, OPT_POST_ONE);
}

__API__ k_err_t tos_sem_post_all(k_sem_t *sem)
{
    return sem_do_post(sem, OPT_POST_ALL);
}

__API__ k_err_t tos_sem_pend(k_sem_t *sem, k_tick_t timeout)
{
    TOS_CPU_CPSR_ALLOC();

    TOS_IN_IRQ_CHECK();
    TOS_PTR_SANITY_CHECK(sem);
    TOS_OBJ_VERIFY(sem, KNL_OBJ_TYPE_SEMAPHORE);

    TOS_CPU_INT_DISABLE();

    if (sem->count > (k_sem_cnt_t)0u) {
        --sem->count;
        TOS_CPU_INT_ENABLE();
        return K_ERR_NONE;
    }

    if (timeout == TOS_TIME_NOWAIT) { // no wait, return immediately
        TOS_CPU_INT_ENABLE();
        return K_ERR_PEND_NOWAIT;
    }

    if (knl_is_sched_locked()) {
        TOS_CPU_INT_ENABLE();
        return K_ERR_PEND_SCHED_LOCKED;
    }

    pend_task_block(k_curr_task, &sem->pend_obj, timeout);

    TOS_CPU_INT_ENABLE();
    knl_sched();

    return pend_state2errno(k_curr_task->pend_state);
}

#endif // TOS_CFG_SEM_EN

