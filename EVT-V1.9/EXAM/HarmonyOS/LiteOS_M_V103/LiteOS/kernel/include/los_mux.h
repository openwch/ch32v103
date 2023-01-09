/*
 * Copyright (c) 2013-2019 Huawei Technologies Co., Ltd. All rights reserved.
 * Copyright (c) 2020-2021 Huawei Device Co., Ltd. All rights reserved.
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

/**
 * @defgroup los_mux Mutex
 * @ingroup kernel
 */

#ifndef _LOS_MUX_H
#define _LOS_MUX_H

#include "los_task.h"


#ifdef __cplusplus
#if __cplusplus
extern "C" {
#endif /* __cplusplus */
#endif /* __cplusplus */

/**
 * @ingroup los_mux
 * Mutex error code: The memory request fails.
 *
 * Value: 0x02001d00
 *
 * Solution: Decrease the number of mutexes defined by LOSCFG_BASE_IPC_MUX_LIMIT.
 */
#define LOS_ERRNO_MUX_NO_MEMORY     LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x00)

/**
 * @ingroup los_mux
 * Mutex error code: The mutex is not usable.
 *
 * Value: 0x02001d01
 *
 * Solution: Check whether the mutex ID and the mutex state are applicable for the current operation.
 */
#define LOS_ERRNO_MUX_INVALID       LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x01)

/**
* @ingroup los_mux
* Mutex error code: Null pointer.
*
* Value: 0x02001d02
*
* Solution: Check whether the input parameter is usable.
*/
#define LOS_ERRNO_MUX_PTR_NULL      LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x02)

/**
* @ingroup los_mux
* Mutex error code: No mutex is available and the mutex request fails.
*
* Value: 0x02001d03
*
* Solution: Increase the number of mutexes defined by LOSCFG_BASE_IPC_MUX_LIMIT.
*/
#define LOS_ERRNO_MUX_ALL_BUSY      LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x03)

/**
* @ingroup los_mux
* Mutex error code: The mutex fails to be locked in non-blocking mode because it is locked by another thread.
*
* Value: 0x02001d04
*
* Solution: Lock the mutex after it is unlocked by the thread that owns it, or set a waiting time.
*/
#define LOS_ERRNO_MUX_UNAVAILABLE   LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x04)

/**
* @ingroup los_mux
* Mutex error code: The mutex is being locked during an interrupt.
*
* Value: 0x02001d05
*
* Solution: Check whether the mutex is being locked during an interrupt.
*/
#define LOS_ERRNO_MUX_PEND_INTERR   LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x05)

/**
* @ingroup los_mux
* Mutex error code: A thread locks a mutex after waiting for the mutex to be unlocked by another thread
* when the task scheduling is disabled.
*
* Value: 0x02001d06
*
* Solution: Check whether the task scheduling is disabled, or set uwtimeout to 0, which means that the
* thread will not wait for the mutex to become available.
*/
#define LOS_ERRNO_MUX_PEND_IN_LOCK  LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x06)

/**
* @ingroup los_mux
* Mutex error code: The mutex locking times out.
*
* Value: 0x02001d07
*
* Solution: Increase the waiting time or set the waiting time to LOS_WAIT_FOREVER (forever-blocking mode).
*/
#define LOS_ERRNO_MUX_TIMEOUT       LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x07)

/**
 * @ingroup los_mux
 *
 * Value: 0x02001d08
 * Not in use temporarily.
 */
#define LOS_ERRNO_MUX_OVERFLOW      LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x08)

/**
* @ingroup los_mux
* Mutex error code: The mutex to be deleted is being locked.
*
* Value: 0x02001d09
*
* Solution: Delete the mutex after it is unlocked.
*/
#define LOS_ERRNO_MUX_PENDED        LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x09)

/**
 * @ingroup los_mux
 *
 * Value: 0x02001d0A
 * Not in use temporarily.
 */
#define LOS_ERRNO_MUX_GET_COUNT_ERR LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x0A)

/**
 * @ingroup los_mux
 *
 * Value: 0x02001d0B
 * Not in use temporarily.
 */
#define LOS_ERRNO_MUX_REG_ERROR     LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x0B)

/**
 * @ingroup los_mux
 *
 * Mutex error code: LOS_ERRNO_MUX_MAXNUM_ZERO is zero.
 * Value: 0x02001d0C
 *
 * Solution: LOS_ERRNO_MUX_MAXNUM_ZERO should not be zero.
 */
#define LOS_ERRNO_MUX_MAXNUM_ZERO   LOS_ERRNO_OS_ERROR(LOS_MOD_MUX, 0x0C)

/**
 * @ingroup los_mux
 * @brief Create a mutex.
 *
 * @par Description:
 * This API is used to create a mutex. A mutex handle is assigned to muxHandle when the mutex is created successfully.
 * Return LOS_OK on creating successful, return specific error code otherwise.
 * @attention
 * <ul>
 * <li>The total number of mutexes is pre-configured. If there are no available mutexes, the mutex creation fails.</li>
 * </ul>
 *
 * @param muxHandle   [OUT] Handle pointer of the successfully created mutex. The value of handle should be in
 * [0, LOSCFG_BASE_IPC_MUX_LIMIT - 1].
 *
 * @retval #LOS_ERRNO_MUX_PTR_NULL           The muxHandle pointer is NULL.
 * @retval #LOS_ERRNO_MUX_ALL_BUSY           No available mutex.
 * @retval #LOS_OK                           The mutex is successfully created.
 * @par Dependency:
 * <ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_MuxDelete
 */
extern UINT32 LOS_MuxCreate(UINT32 *muxHandle);

/**
 * @ingroup los_mux
 * @brief Delete a mutex.
 *
 * @par Description:
 * This API is used to delete a specified mutex. Return LOS_OK on deleting successfully, return specific error code
 * otherwise.
 * @attention
 * <ul>
 * <li>The specific mutex should be created firstly.</li>
 * <li>The mutex can be deleted successfully only if no other tasks pend on it.</li>
 * </ul>
 *
 * @param muxHandle   [IN] Handle of the mutex to be deleted. The value of handle should be in
 * [0, LOSCFG_BASE_IPC_MUX_LIMIT - 1].
 *
 * @retval #LOS_ERRNO_MUX_INVALID            Invalid handle or mutex in use.
 * @retval #LOS_ERRNO_MUX_PENDED             Tasks pended on this mutex.
 * @retval #LOS_OK                           The mutex is successfully deleted.
 * @par Dependency:
 * <ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_MuxCreate
 */
extern UINT32 LOS_MuxDelete(UINT32 muxHandle);

/**
 * @ingroup los_mux
 * @brief Wait to lock a mutex.
 *
 * @par Description:
 * This API is used to wait for a specified period of time to lock a mutex.
 * @attention
 * <ul>
 * <li>The specific mutex should be created firstly.</li>
 * <li>The function fails if the mutex that is waited on is already locked by another thread when the task scheduling
 * is disabled.</li>
 * <li>Do not wait on a mutex during an interrupt.</li>
 * <li>The priority inheritance protocol is supported. If a higher-priority thread is waiting on a mutex, it changes
 * the priority of the thread that owns the mutex to avoid priority inversion.</li>
 * <li>A recursive mutex can be locked more than once by the same thread.</li>
 * </ul>
 *
 * @param muxHandle    [IN] Handle of the mutex to be waited on.  The value of handle should be
 * in [0, LOSCFG_BASE_IPC_MUX_LIMIT - 1].
 * @param timeout      [IN] Waiting time. The value range is [0, LOS_WAIT_FOREVER](unit: Tick).
 *
 * @retval #LOS_ERRNO_MUX_INVALID            The mutex state (for example, the mutex does not exist or is not in use)
 *                                           is not applicable for the current operation.
 * @retval #LOS_ERRNO_MUX_UNAVAILABLE        The mutex fails to be locked because it is locked by another thread and
 * a period of time is not set for waiting for the mutex to become available.
 * @retval #LOS_ERRNO_MUX_PEND_INTERR        The mutex is being locked during an interrupt.
 * @retval #LOS_ERRNO_MUX_PEND_IN_LOCK       The mutex is waited on when the task scheduling is disabled.
 * @retval #LOS_ERRNO_MUX_TIMEOUT            The mutex waiting times out.
 * @retval #LOS_OK                           The mutex is successfully locked.
 * @par Dependency:
 * <ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_MuxCreate | LOS_MuxPost
 */
extern UINT32 LOS_MuxPend(UINT32 muxHandle, UINT32 timeout);

/**
 * @ingroup los_mux
 * @brief Release a mutex.
 *
 * @par Description:
 * This API is used to release a specified mutex.
 * @attention
 * <ul>
 * <li>The specific mutex should be created firstly.</li>
 * <li>Do not release a mutex during an interrupt.</li>
 * <li>If a recursive mutex is locked for many times, it must be unlocked for the same times to be released.</li>
 * </ul>
 *
 * @param muxHandle    [IN] Handle of the mutex to be released. The value of handle should be in
 * [0, LOSCFG_BASE_IPC_MUX_LIMIT - 1].
 *
 * @retval #LOS_ERRNO_MUX_INVALID            The mutex state (for example, the mutex does not exist or is not in use
 * or owned by other thread) is not applicable for the current operation.
 * @retval #LOS_ERRNO_MUX_PEND_INTERR        The mutex is being released during an interrupt.
 * @retval #LOS_OK                           The mutex is successfully released.
 * @par Dependency:
 * <ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_MuxCreate | LOS_MuxPend
 */
extern UINT32 LOS_MuxPost(UINT32 muxHandle);

/**
 * @ingroup los_mux
 * Mutex object.
 */
typedef struct {
    UINT8 muxStat;       /**< State OS_MUX_UNUSED,OS_MUX_USED  */
    UINT16 muxCount;     /**< Times of locking a mutex */
    UINT32 muxID;        /**< Handle ID */
    LOS_DL_LIST muxList; /**< Mutex linked list */
    LosTaskCB *owner;    /**< The current thread that is locking a mutex */
    UINT16 priority;     /**< Priority of the thread that is locking a mutex */
} LosMuxCB;

/**
 * @ingroup los_mux
 * Mutex state: not in use.
 */
#define OS_MUX_UNUSED 0

/**
 * @ingroup los_mux
 * Mutex state: in use.
 */
#define OS_MUX_USED   1

extern LosMuxCB *g_allMux;

/**
 * @ingroup los_mux
 * Obtain the pointer to a mutex object of the mutex that has a specified handle.
 */
#define GET_MUX(muxid) (((LosMuxCB *)g_allMux) + (muxid))

/**
 * @ingroup los_mux
 * @brief Initializes the mutex.
 *
 * @par Description:
 * This API is used to initializes the mutex.
 * @attention
 * <ul>
 * <li>None.</li>
 * </ul>
 *
 * @param None.
 *
 * @retval UINT32     Initialization result.
 * @par Dependency:
 * <ul><li>los_mux.h: the header file that contains the API declaration.</li></ul>
 * @see LOS_MuxDelete
 */
extern UINT32 OsMuxInit(VOID);

/**
 * @ingroup los_mux
 * Obtain the pointer to the linked list in the mutex pointed to by a specified pointer.
 */
#define GET_MUX_LIST(ptr) LOS_DL_LIST_ENTRY(ptr, LosMuxCB, muxList)

#ifdef __cplusplus
#if __cplusplus
}
#endif
#endif /* __cplusplus */

#endif /* _LOS_MUX_H */
