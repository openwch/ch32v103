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
#include "It_los_event.h"

static VOID TaskF02(VOID)
{
    UINT32 ret;

    g_testCount++;

    ret = LOS_TaskSuspend(g_testTaskID01);
    ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

    ret = LOS_EventWrite(&g_pevent, 0x11);
    ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

    g_testCount++;

EXIT:
    LOS_TaskDelete(g_testTaskID02);
}

static VOID TaskF01(VOID)
{
    UINT32 ret;

    g_testCount++;

    ret = LOS_EventRead(&g_pevent, 0x11, LOS_WAITMODE_AND, LOS_WAIT_FOREVER);
    ICUNIT_GOTO_EQUAL(ret, g_pevent.uwEventID, ret, EXIT);

    g_testCount++;

EXIT:
    LOS_TaskDelete(g_testTaskID01);
}

static UINT32 Testcase(VOID)
{
    UINT32 ret;
    TSK_INIT_PARAM_S task1;
    (void)memset_s(&task1, sizeof(TSK_INIT_PARAM_S), 0, sizeof(TSK_INIT_PARAM_S));
    task1.pfnTaskEntry = (TSK_ENTRY_FUNC)TaskF01;
    task1.pcName = "EvtTsk22A";
    task1.uwStackSize = TASK_STACK_SIZE_TEST;
    task1.usTaskPrio = TASK_PRIO_TEST - 2; // 2, set new task priority, it is higher than the test task.
    task1.uwResved = LOS_TASK_STATUS_DETACHED;

    g_testCount = 0;
    LOS_EventInit(&g_pevent);

    ret = LOS_TaskCreate(&g_testTaskID01, &task1);
    ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

    ICUNIT_ASSERT_EQUAL(g_testCount, 1, g_testCount);
    g_testCount++;

    task1.pfnTaskEntry = (TSK_ENTRY_FUNC)TaskF02;
    task1.pcName = "EvtTsk22B";
    task1.uwStackSize = TASK_STACK_SIZE_TEST;
    task1.usTaskPrio = TASK_PRIO_TEST - 1;
    task1.uwResved = LOS_TASK_STATUS_DETACHED;

    ret = LOS_TaskCreate(&g_testTaskID02, &task1);
    ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT1);

    ICUNIT_GOTO_EQUAL(g_testCount, 4, g_testCount, EXIT1); // 4, Here, assert that g_testCount is equal to 4.
    g_testCount++;

    ret = LOS_TaskResume(g_testTaskID01);
    ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT1);

    ICUNIT_GOTO_EQUAL(g_testCount, 6, g_testCount, EXIT1); // 6, Here, assert that g_testCount is equal to 6.

EXIT1:
    LOS_TaskDelete(g_testTaskID02);

EXIT:
    LOS_TaskDelete(g_testTaskID01);

    return LOS_OK;
}

VOID ItLosEvent022(VOID)
{
    TEST_ADD_CASE("ItLosEvent022", Testcase, TEST_LOS, TEST_EVENT, TEST_LEVEL0, TEST_FUNCTION);
}
