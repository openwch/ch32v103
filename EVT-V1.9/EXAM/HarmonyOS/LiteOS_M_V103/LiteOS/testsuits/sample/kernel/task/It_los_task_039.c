/*
 * Copyright (c) 2013-2019 Huawei Technologies Co., Ltd. All rights reserved.
 * Copyright (c) 2020-2021 Huawei Device Co., Ltd. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice, this list of
 * conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice, this list
 * of conditions and the following disclaimer in the documentation and/or other materials
 * provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors may be used
 * to endorse or promote products derived from this software without specific prior written
 * permission.
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

#include "osTest.h"
#include "It_los_task.h"


static VOID TaskF01(VOID)
{
    g_testCount++;

    ICUNIT_ASSERT_EQUAL_VOID(g_testCount, 1, g_testCount);
}

static UINT32 TestCase(VOID)
{
    UINT32 ret;
    UINT8 index;
    CHAR acName[TASK_NAME_NUM] = "Tsk";
    TSK_INIT_PARAM_S task1 = { 0 };
    UINT32 testTaskID[LOSCFG_BASE_CORE_TSK_LIMIT];

    task1.pfnTaskEntry = (TSK_ENTRY_FUNC)TaskF01;
    task1.uwStackSize = LOSCFG_BASE_CORE_TSK_MIN_STACK_SIZE;
    task1.usTaskPrio = TASK_PRIO_TEST + 1;
    g_testCount = 0;

    LOS_TaskLock();

    for (index = 0; index < LOSCFG_BASE_CORE_TSK_LIMIT - TASK_EXISTED_D_NUM + 1; index++) {
        task1.usTaskPrio = index % OS_TASK_PRIORITY_LOWEST;
        task1.pcName = acName;
        task1.uwResved = LOS_TASK_STATUS_DETACHED;
        ret = LOS_TaskCreate(&testTaskID[index], &task1);
        ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);
    }

    task1.usTaskPrio = index % OS_TASK_PRIORITY_LOWEST;
    task1.pcName = "TskA";
    task1.uwResved = LOS_TASK_STATUS_DETACHED;
    ret = LOS_TaskCreate(&testTaskID[index], &task1);
    ICUNIT_GOTO_EQUAL(ret, LOS_ERRNO_TSK_TCB_UNAVAILABLE, ret, EXIT);
EXIT:

    for (index = 0; index < LOSCFG_BASE_CORE_TSK_LIMIT - TASK_EXISTED_D_NUM + 1; index++) {
        ret = LOS_TaskDelete(testTaskID[index]);
        ICUNIT_ASSERT_EQUAL(ret, LOS_OK, ret);
    }

    LOS_TaskUnlock();

    return LOS_OK;
}

VOID ItLosTask039(VOID) // IT_Layer_ModuleORFeature_No
{
    TEST_ADD_CASE("ItLosTask039", TestCase, TEST_LOS, TEST_TASK, TEST_LEVEL1, TEST_FUNCTION);
}

