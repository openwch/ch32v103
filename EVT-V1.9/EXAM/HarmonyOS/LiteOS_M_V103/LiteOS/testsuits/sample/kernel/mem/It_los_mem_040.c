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
#include "It_los_mem.h"


static UINT32 TestCase(VOID)
{
    UINT32 ret;
    UINT32 memused0, memused1, memused2, tempsize;
    UINT32 size = 0x100;
    void *p = NULL;
    void *p1 = NULL;

    MemStart();

    MemInit();

    memused2 = memused0 = LOS_MemTotalUsedGet(g_memPool);

    p = LOS_MemAlloc((void *)g_memPool, size);
    ICUNIT_GOTO_NOT_EQUAL(p, NULL, p, EXIT);

    memused1 = LOS_MemTotalUsedGet(g_memPool);

    tempsize = memused1 - memused0;
    ICUNIT_GOTO_EQUAL(tempsize, size + LOS_MEM_NODE_HEAD_SIZE, tempsize, EXIT);

    memused0 = LOS_MemTotalUsedGet(g_memPool);

    p1 = LOS_MemAlloc((void *)g_memPool, size);
    ICUNIT_GOTO_NOT_EQUAL(p, NULL, p, EXIT);

    memused1 = LOS_MemTotalUsedGet(g_memPool);
    tempsize = memused1 - memused0;
    ICUNIT_GOTO_EQUAL(tempsize, size + LOS_MEM_NODE_HEAD_SIZE, tempsize, EXIT);

    ret = LOS_MemFree((void *)g_memPool, p);
    ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

    ret = LOS_MemFree((void *)g_memPool, p1);
    ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

    memused1 = LOS_MemTotalUsedGet(g_memPool);
    ICUNIT_GOTO_EQUAL(memused1, memused2, memused1, EXIT);

EXIT:
    MemFree();
    MemEnd();
    return LOS_OK;
}

VOID ItLosMem040(void)
{
    TEST_ADD_CASE("ItLosMem040", TestCase, TEST_LOS, TEST_MEM, TEST_LEVEL1, TEST_FUNCTION);
}

