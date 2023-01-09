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
#include "It_los_swtmr.h"


static VOID Case1(UINT32 arg)
{
    if (arg != 0xffff) {
        return;
    }

    g_testCount++;
    return;
}

static UINT32 Testcase(VOID)
{
    UINT32 ret;
    UINT32 i;
    UINT32 swTmrID;

    g_testCount = 0;

    for (i = 1; i < SWTMR_LOOP_NUM; i++) {
        // 2, Timeout interval of a periodic software timer.
        ret = LOS_SwtmrCreate(2, LOS_SWTMR_MODE_ONCE, Case1, &swTmrID, 0xffff
#if (LOSCFG_BASE_CORE_SWTMR_ALIGN == 1)
            ,
            OS_SWTMR_ROUSES_ALLOW, OS_SWTMR_ALIGN_INSENSITIVE
#endif
        );
        ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

        ret = LOS_SwtmrStart(swTmrID);
        ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

        ret = LOS_TaskDelay(4); // 4, set delay time.
        ICUNIT_GOTO_EQUAL(ret, LOS_OK, ret, EXIT);

        ICUNIT_ASSERT_EQUAL(g_testCount, i, g_testCount);
#if SELF_DELETED
        ret = LOS_SwtmrDelete(swTmrID);
        ICUNIT_ASSERT_EQUAL(ret, LOS_OK, ret);
#endif
    }

    return LOS_OK;

EXIT:
    ret = LOS_SwtmrDelete(swTmrID);
    ICUNIT_ASSERT_EQUAL(ret, LOS_OK, ret);

    return LOS_OK;
}

VOID ItLosSwtmr025() // IT_Layer_ModuleORFeature_No
{
    TEST_ADD_CASE("ItLosSwtmr025", Testcase, TEST_LOS, TEST_SWTMR, TEST_LEVEL3, TEST_FUNCTION);
}

