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
#include "It_los_mux.h"
#include "los_config.h"
#include "los_sem.h"


static VOID TaskFuncC(VOID)
{
    UINT32 ret;
    g_testCount++;

    ICUNIT_ASSERT_EQUAL_VOID(g_testCount, 3, g_testCount); // 3, Here, assert that g_testCount is equal to 3.

    ret = LOS_MuxPend(g_mutexTest1, LOS_WAIT_FOREVER);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    ret = LOS_MuxPost(g_mutexTest1);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    // 5, Here, assert that priority is equal to 5.
    ICUNIT_ASSERT_EQUAL_VOID(g_losTask.runTask->priority, 5, g_losTask.runTask->priority);
    // 3, Here, assert that g_testCount is equal to 3.
    ICUNIT_ASSERT_EQUAL_VOID(g_testCount, 3, g_testCount);

    g_testCount++;
}

static VOID TaskFuncB(VOID)
{
    UINT32 ret;
    g_testCount++;

    ICUNIT_ASSERT_EQUAL_VOID(g_testCount, 2, g_testCount); // 2, Here, assert that g_testCount is equal to 2.

    ret = LOS_MuxPend(g_mutexTest1, LOS_WAIT_FOREVER);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    ret = LOS_MuxPost(g_mutexTest1);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    // 8, Here, assert that priority is equal to 8.
    ICUNIT_ASSERT_EQUAL_VOID(g_losTask.runTask->priority, 8, g_losTask.runTask->priority);

    // 4, Here, assert that g_testCount is equal to 4.
    ICUNIT_ASSERT_EQUAL_VOID(g_testCount, 4, g_testCount);

    g_testCount++;
}

static VOID TaskFuncA(VOID)
{
    UINT32 ret;
    TSK_INIT_PARAM_S task1, task2;
    g_testCount++;

    ICUNIT_ASSERT_EQUAL_VOID(g_testCount, 1, g_testCount); // 1, Here, assert that g_testCount is equal to 1.

    ret = LOS_MuxPend(g_mutexTest1, 0);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    task1.pfnTaskEntry = (TSK_ENTRY_FUNC)TaskFuncB;
    // 8, Set the priority according to the task purpose,a smaller number means a higher priority.
    task1.usTaskPrio = 8;
    task1.pcName = "TaskB";
    task1.uwStackSize = TASK_STACK_SIZE_TEST;
    task1.uwResved = LOS_TASK_STATUS_DETACHED;

    ret = LOS_TaskCreate(&g_testTaskID02, &task1);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    task2.pfnTaskEntry = (TSK_ENTRY_FUNC)TaskFuncC;
    // 5, Set the priority according to the task purpose,a smaller number means a higher priority.
    task2.usTaskPrio = 5;
    task2.pcName = "TaskC";
    task2.uwStackSize = TASK_STACK_SIZE_TEST;
    task2.uwResved = LOS_TASK_STATUS_DETACHED;

    ret = LOS_TaskCreate(&g_testTaskID03, &task2);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    LOS_TaskDelay(5); // 5, set delay time.

    ret = LOS_MuxPost(g_mutexTest1);
    ICUNIT_ASSERT_EQUAL_VOID(ret, LOS_OK, ret);

    // 10, Here, assert that priority is equal to 10.
    ICUNIT_ASSERT_EQUAL_VOID(g_losTask.runTask->priority, 10, g_losTask.runTask->priority);
    // 5, Here, assert that g_testCount is equal to 5.
    ICUNIT_ASSERT_EQUAL_VOID(g_testCount, 5, g_testCount);
}

static UINT32 Testcase(VOID)
{
    UINT32 ret;
    TSK_INIT_PARAM_S task;
    g_testCount = 0;

    ret = LOS_MuxCreate(&g_mutexTest1);
    ICUNIT_ASSERT_EQUAL(ret, LOS_OK, ret);

    task.pfnTaskEntry = (TSK_ENTRY_FUNC)TaskFuncA;
    // 10, Set the priority according to the task purpose,a smaller number means a higher priority.
    task.usTaskPrio = 10;
    task.pcName = "TaskA";
    task.uwStackSize = TASK_STACK_SIZE_TEST;
    task.uwResved = LOS_TASK_STATUS_DETACHED;

    ret = LOS_TaskCreate(&g_testTaskID01, &task);
    ICUNIT_ASSERT_EQUAL(ret, LOS_OK, ret);

    ICUNIT_ASSERT_EQUAL(g_testCount, 3, g_testCount); // 3, Here, assert that g_testCount is equal to 3.

    LOS_TaskDelay(10); // 10, set delay time.

    ret = LOS_MuxDelete(g_mutexTest1);
    ICUNIT_ASSERT_EQUAL(ret, LOS_OK, ret);

    // 25, Here, assert that priority is equal to 25.
    ICUNIT_ASSERT_EQUAL(g_losTask.runTask->priority, 25, g_losTask.runTask->priority);

    ICUNIT_ASSERT_EQUAL(g_testCount, 5, g_testCount); // 5, Here, assert that g_testCount is equal to 5.
    return LOS_OK;
}

VOID ItLosMux033(void)
{
    TEST_ADD_CASE("ItLosMux033", Testcase, TEST_LOS, TEST_MUX, TEST_LEVEL1, TEST_FUNCTION);
}

