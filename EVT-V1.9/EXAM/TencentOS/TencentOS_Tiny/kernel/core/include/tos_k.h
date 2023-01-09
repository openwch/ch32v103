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

#ifndef _TOS_K_H_
#define  _TOS_K_H_

#include <tos_compiler.h>
#include <tos_kerr.h>
#include <tos_cpu_def.h>
#include <tos_config.h>
#include <tos_config_default.h>
#include <port_config.h>
#include <tos_config_check.h>
#include <tos_ktypes.h>
#include <tos_cpu_types.h>
#include <port.h>
#include <tos_cpu.h>
#include <tos_fault.h>
#include <tos_klib.h>
#include <tos_list.h>
#include <tos_slist.h>
#include <tos_pend.h>
#include <tos_sys.h>
#include <tos_bitmap.h>
#include <tos_ring_queue.h>
#include <tos_char_fifo.h>
#include <tos_mail_queue.h>
#include <tos_message_queue.h>
#include <tos_binary_heap.h>
#include <tos_priority_queue.h>
#include <tos_priority_mail_queue.h>
#include <tos_priority_message_queue.h>
#include <tos_task.h>
#include <tos_robin.h>
#include <tos_mutex.h>
#include <tos_sem.h>
#include <tos_event.h>
#include <tos_barrier.h>
#include <tos_completion.h>
#include <tos_countdownlatch.h>
#include <tos_rwlock.h>
#include <tos_timer.h>
#include <tos_time.h>
#include <tos_stopwatch.h>
#include <tos_mmblk.h>
#include <tos_mmheap.h>
#include <tos_tick.h>
#include <tos_sched.h>
#if TOS_CFG_PWR_MGR_EN > 0u
#include <tos_pm.h>
#if TOS_CFG_TICKLESS_EN > 0u
#include <tos_tickless.h>
#endif
#endif
#include <tos_global.h>
#include <tos_version.h>

#endif /* _TOS_K_H_ */

