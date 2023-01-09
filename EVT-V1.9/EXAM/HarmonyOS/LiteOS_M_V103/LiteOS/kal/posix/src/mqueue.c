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

#include "mqueue_impl.h"
#include "time_internal.h"

/* GLOBALS */
STATIC struct mqarray g_queueTable[LOSCFG_BASE_IPC_QUEUE_LIMIT];
STATIC pthread_mutex_t g_mqueueMutex = PTHREAD_RECURSIVE_MUTEX_INITIALIZER_NP;

int MapMqErrno(UINT32 err)
{
    if (err == LOS_OK) {
        return ENOERR;
    }
    switch (err) {
        case LOS_ERRNO_QUEUE_INVALID:
        case LOS_ERRNO_QUEUE_WRITE_PTR_NULL:
        case LOS_ERRNO_QUEUE_WRITESIZE_ISZERO:
        case LOS_ERRNO_QUEUE_SIZE_TOO_BIG:
        case LOS_ERRNO_QUEUE_CREAT_PTR_NULL:
        case LOS_ERRNO_QUEUE_PARA_ISZERO:
        case LOS_ERRNO_QUEUE_WRITE_SIZE_TOO_BIG:
            errno = EINVAL;
            break;
        case LOS_ERRNO_QUEUE_ISFULL:
        case LOS_ERRNO_QUEUE_ISEMPTY:
            errno = EAGAIN;
            break;
        case LOS_ERRNO_QUEUE_CREATE_NO_MEMORY:
            errno = ENOSPC;
            break;
        case LOS_ERRNO_QUEUE_TIMEOUT:
            errno = ETIMEDOUT;
            break;
        case LOS_ERRNO_QUEUE_CB_UNAVAILABLE:
            errno = ENFILE;
            break;
        case LOS_ERRNO_QUEUE_READ_IN_INTERRUPT:
        case LOS_ERRNO_QUEUE_WRITE_IN_INTERRUPT:
            errno = EINTR;
            break;
        default:
            errno = EINVAL;
            break;
    }
    return errno;
}

STATIC INLINE INT32 MqNameCheck(const CHAR *mqName)
{
    if (mqName == NULL) {
        errno = EINVAL;
        return -1;
    }

    if (strlen(mqName) == 0) {
        errno = EINVAL;
        return -1;
    }

    if (strlen(mqName) > (PATH_MAX - 1)) {
        errno = ENAMETOOLONG;
        return -1;
    }
    return 0;
}

STATIC INLINE UINT32 GetMqueueCBByID(UINT32 queueID, LosQueueCB **queueCB)
{
    LosQueueCB *tmpQueueCB = NULL;
    if (queueCB == NULL) {
        errno = EINVAL;
        return LOS_ERRNO_QUEUE_READ_PTR_NULL;
    }
    tmpQueueCB = GET_QUEUE_HANDLE(queueID);
    if ((GET_QUEUE_INDEX(queueID) >= LOSCFG_BASE_IPC_QUEUE_LIMIT) || (tmpQueueCB->queueID != queueID)) {
        return LOS_ERRNO_QUEUE_INVALID;
    }
    *queueCB = tmpQueueCB;

    return LOS_OK;
}

STATIC INLINE struct mqarray *GetMqueueCBByName(const CHAR *name)
{
    UINT32 index;
    UINT32 mylen = strlen(name);

    for (index = 0; index < LOSCFG_BASE_IPC_QUEUE_LIMIT; index++) {
        if ((g_queueTable[index].mq_name == NULL) || (strlen(g_queueTable[index].mq_name) != mylen)) {
            continue;
        }

        if (strncmp(name, (const CHAR *)(g_queueTable[index].mq_name), mylen) == 0) {
            return &(g_queueTable[index]);
        }
    }

    return NULL;
}

STATIC INT32 DoMqueueDelete(struct mqarray *mqueueCB)
{
    UINT32 ret;

    if (mqueueCB->mq_name != NULL) {
        LOS_MemFree(OS_SYS_MEM_ADDR, mqueueCB->mq_name);
        mqueueCB->mq_name = NULL;
    }

    mqueueCB->mqcb = NULL;

    ret = LOS_QueueDelete(mqueueCB->mq_id);
    switch (ret) {
        case LOS_OK:
            return 0;
        case LOS_ERRNO_QUEUE_NOT_FOUND:
        case LOS_ERRNO_QUEUE_NOT_CREATE:
        case LOS_ERRNO_QUEUE_IN_TSKUSE:
        case LOS_ERRNO_QUEUE_IN_TSKWRITE:
            errno = EAGAIN;
            return -1;
        default:
            errno = EINVAL;
            return -1;
    }
}

STATIC int SaveMqueueName(const CHAR *mqName, struct mqarray *mqueueCB)
{
    size_t nameLen;

    nameLen = strlen(mqName); /* sys_mq_open has checked name and name length */
    mqueueCB->mq_name = (char *)LOS_MemAlloc(OS_SYS_MEM_ADDR, nameLen + 1);
    if (mqueueCB->mq_name == NULL) {
        errno = ENOMEM;
        return LOS_NOK;
    }

    if (strncpy_s(mqueueCB->mq_name, (nameLen + 1), mqName, nameLen) != EOK) {
        LOS_MemFree(OS_SYS_MEM_ADDR, mqueueCB->mq_name);
        mqueueCB->mq_name = NULL;
        errno = EINVAL;
        return LOS_NOK;
    }
    mqueueCB->mq_name[nameLen] = '\0';
    return LOS_OK;
}

STATIC struct mqpersonal *DoMqueueCreate(const struct mq_attr *attr, const CHAR *mqName, INT32 openFlag)
{
    struct mqarray *mqueueCB = NULL;
    UINT32 mqueueID;

    UINT32 err = LOS_QueueCreate(NULL, attr->mq_maxmsg, &mqueueID, 0, attr->mq_msgsize);
    if (MapMqErrno(err) != ENOERR) {
        goto ERROUT;
    }

    if (g_queueTable[GET_QUEUE_INDEX(mqueueID)].mqcb == NULL) {
        mqueueCB = &(g_queueTable[GET_QUEUE_INDEX(mqueueID)]);
        mqueueCB->mq_id = mqueueID;
    }

    if (mqueueCB == NULL) {
        errno = EINVAL;
        goto ERROUT;
    }

    if (SaveMqueueName(mqName, mqueueCB) != LOS_OK) {
        goto ERROUT;
    }

    if (GetMqueueCBByID(mqueueCB->mq_id, &(mqueueCB->mqcb)) != LOS_OK) {
        errno = ENOSPC;
        goto ERROUT;
    }

    mqueueCB->mq_personal = (struct mqpersonal *)LOS_MemAlloc(OS_SYS_MEM_ADDR, sizeof(struct mqpersonal));
    if (mqueueCB->mq_personal == NULL) {
        (VOID)LOS_QueueDelete(mqueueCB->mq_id);
        mqueueCB->mqcb->queue = NULL;
        mqueueCB->mqcb = NULL;
        errno = ENOSPC;
        goto ERROUT;
    }

    mqueueCB->unlinkflag = FALSE;
    mqueueCB->mq_personal->mq_status = MQ_USE_MAGIC;
    mqueueCB->mq_personal->mq_next = NULL;
    mqueueCB->mq_personal->mq_posixdes = mqueueCB;
    mqueueCB->mq_personal->mq_flags = (INT32)((UINT32)openFlag | ((UINT32)attr->mq_flags & (UINT32)FNONBLOCK));

    return mqueueCB->mq_personal;
ERROUT:

    if ((mqueueCB != NULL) && (mqueueCB->mq_name != NULL)) {
        LOS_MemFree(OS_SYS_MEM_ADDR, mqueueCB->mq_name);
        mqueueCB->mq_name = NULL;
    }
    return (struct mqpersonal *)-1;
}

STATIC struct mqpersonal *DoMqueueOpen(struct mqarray *mqueueCB, INT32 openFlag)
{
    struct mqpersonal *privateMqPersonal = NULL;

    /* already have the same name of g_squeuetable */
    if (mqueueCB->unlinkflag == TRUE) {
        errno = EINVAL;
        goto ERROUT;
    }
    /* alloc mqprivate and add to mqarray */
    privateMqPersonal = (struct mqpersonal *)LOS_MemAlloc(OS_SYS_MEM_ADDR, sizeof(struct mqpersonal));
    if (privateMqPersonal == NULL) {
        errno = ENOSPC;
        goto ERROUT;
    }

    privateMqPersonal->mq_next = mqueueCB->mq_personal;
    mqueueCB->mq_personal = privateMqPersonal;

    privateMqPersonal->mq_posixdes = mqueueCB;
    privateMqPersonal->mq_flags = openFlag;
    privateMqPersonal->mq_status = MQ_USE_MAGIC;

    return privateMqPersonal;

ERROUT:
    return (struct mqpersonal *)-1;
}

mqd_t mq_open(const char *mqName, int openFlag, ...)
{
    struct mqarray *mqueueCB = NULL;
    struct mqpersonal *privateMqPersonal = (struct mqpersonal *)-1;
    struct mq_attr *attr = NULL;
    struct mq_attr defaultAttr = { 0, MQ_MAX_MSG_NUM, MQ_MAX_MSG_LEN, 0, {0} };

    va_list ap;

    if (MqNameCheck(mqName) == -1) {
        return (mqd_t)-1;
    }

    (VOID)pthread_mutex_lock(&g_mqueueMutex);
    mqueueCB = GetMqueueCBByName(mqName);
    if ((UINT32)openFlag & (UINT32)O_CREAT) {
        if (mqueueCB != NULL) {
            if (((UINT32)openFlag & (UINT32)O_EXCL)) {
                errno = EEXIST;
                goto OUT;
            }
            privateMqPersonal = DoMqueueOpen(mqueueCB, openFlag);
        } else {
            va_start(ap, openFlag);
            (VOID)va_arg(ap, int);
            attr = va_arg(ap, struct mq_attr *);
            va_end(ap);

            if (attr != NULL) {
                (VOID)memcpy_s(&defaultAttr, sizeof(struct mq_attr), attr, sizeof(struct mq_attr));
                if ((defaultAttr.mq_maxmsg < 0) || (defaultAttr.mq_maxmsg > (long int)USHRT_MAX) ||
                    (defaultAttr.mq_msgsize < 0) || (defaultAttr.mq_msgsize > (long int)(USHRT_MAX - sizeof(UINT32)))) {
                    errno = EINVAL;
                    goto OUT;
                }
            }
            privateMqPersonal = DoMqueueCreate(&defaultAttr, mqName, openFlag);
        }
    } else {
        if (mqueueCB == NULL) {
            errno = ENOENT;
            goto OUT;
        }
        privateMqPersonal = DoMqueueOpen(mqueueCB, openFlag);
    }

OUT:
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
    return (mqd_t)privateMqPersonal;
}

int mq_close(mqd_t personal)
{
    INT32 ret = 0;
    struct mqarray *mqueueCB = NULL;
    struct mqpersonal *privateMqPersonal = NULL;
    struct mqpersonal *tmp = NULL;

    (VOID)pthread_mutex_lock(&g_mqueueMutex);
    privateMqPersonal = (struct mqpersonal *)personal;
    if (privateMqPersonal->mq_status != MQ_USE_MAGIC) {
        errno = EBADF;
        goto OUT_UNLOCK;
    }

    mqueueCB = privateMqPersonal->mq_posixdes;
    if (mqueueCB->mq_personal == NULL) {
        errno = EBADF;
        goto OUT_UNLOCK;
    }

    /* find the personal and remove */
    if (mqueueCB->mq_personal == privateMqPersonal) {
        mqueueCB->mq_personal = privateMqPersonal->mq_next;
    } else {
        for (tmp = mqueueCB->mq_personal; tmp->mq_next != NULL; tmp = tmp->mq_next) {
            if (tmp->mq_next == privateMqPersonal) {
                break;
            }
        }
        if (tmp->mq_next == NULL) {
            errno = EBADF;
            goto OUT_UNLOCK;
        }
        tmp->mq_next = privateMqPersonal->mq_next;
    }
    /* flag no use */
    privateMqPersonal->mq_status = 0;

    /* free the personal */
    ret = LOS_MemFree(OS_SYS_MEM_ADDR, privateMqPersonal);
    if (ret != LOS_OK) {
        errno = EFAULT;
        ret = -1;
        goto OUT_UNLOCK;
    }

    if ((mqueueCB->unlinkflag == TRUE) && (mqueueCB->mq_personal == NULL)) {
        ret = DoMqueueDelete(mqueueCB);
    }
OUT_UNLOCK:
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
    return ret;
}

int OsMqGetAttr(mqd_t personal, struct mq_attr *mqAttr)
{
    struct mqarray *mqueueCB = NULL;
    struct mqpersonal *privateMqPersonal = NULL;

    if (mqAttr == NULL) {
        errno = EINVAL;
        return -1;
    }

    (VOID)pthread_mutex_lock(&g_mqueueMutex);
    privateMqPersonal = (struct mqpersonal *)personal;
    if (privateMqPersonal->mq_status != MQ_USE_MAGIC) {
        errno = EBADF;
        (VOID)pthread_mutex_unlock(&g_mqueueMutex);
        return -1;
    }

    mqueueCB = privateMqPersonal->mq_posixdes;
    mqAttr->mq_maxmsg = mqueueCB->mqcb->queueLen;
    mqAttr->mq_msgsize = mqueueCB->mqcb->queueSize - sizeof(UINT32);
    mqAttr->mq_curmsgs = mqueueCB->mqcb->readWriteableCnt[OS_QUEUE_READ];
    mqAttr->mq_flags = privateMqPersonal->mq_flags;
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
    return 0;
}

int OsMqSetAttr(mqd_t personal, const struct mq_attr *mqSetAttr, struct mq_attr *mqOldAttr)
{
    struct mqpersonal *privateMqPersonal = NULL;

    if (mqSetAttr == NULL) {
        errno = EINVAL;
        return -1;
    }

    (VOID)pthread_mutex_lock(&g_mqueueMutex);
    privateMqPersonal = (struct mqpersonal *)personal;
    if (privateMqPersonal->mq_status != MQ_USE_MAGIC) {
        errno = EBADF;
        (VOID)pthread_mutex_unlock(&g_mqueueMutex);
        return -1;
    }

    if (mqOldAttr != NULL) {
        (VOID)OsMqGetAttr((mqd_t)privateMqPersonal, mqOldAttr);
    }

    privateMqPersonal->mq_flags = (INT32)((UINT32)privateMqPersonal->mq_flags & (UINT32)(~FNONBLOCK)); /* clear */
    if (((UINT32)mqSetAttr->mq_flags & (UINT32)FNONBLOCK) == (UINT32)FNONBLOCK) {
        privateMqPersonal->mq_flags = (INT32)((UINT32)privateMqPersonal->mq_flags | (UINT32)FNONBLOCK);
    }
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
    return 0;
}

int mq_getsetattr(mqd_t mqd, const struct mq_attr *new, struct mq_attr *old)
{
    if (new == NULL) {
        return OsMqGetAttr(mqd, old);
    }
    return OsMqSetAttr(mqd, new, old);
}

int mq_getattr(mqd_t mqd, struct mq_attr *attr)
{
    return mq_getsetattr(mqd, 0, attr);
}

int mq_setattr(mqd_t mqd, const struct mq_attr *new, struct mq_attr *old)
{
    return mq_getsetattr(mqd, new, old);
}

int mq_unlink(const char *mqName)
{
    INT32 ret = 0;
    struct mqarray *mqueueCB = NULL;

    if (MqNameCheck(mqName) == -1) {
        return -1;
    }

    (VOID)pthread_mutex_lock(&g_mqueueMutex);
    mqueueCB = GetMqueueCBByName(mqName);
    if (mqueueCB == NULL) {
        errno = ENOENT;
        goto ERROUT_UNLOCK;
    }

    if (mqueueCB->mq_personal != NULL) {
        mqueueCB->unlinkflag = TRUE;
    } else {
        ret = DoMqueueDelete(mqueueCB);
    }

    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
    return ret;

ERROUT_UNLOCK:
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
    return -1;
}

STATIC INT32 ConvertTimeout(long flags, const struct timespec *absTimeout, UINT64 *ticks)
{
    if ((UINT32)flags & (UINT32)FNONBLOCK) {
        *ticks = LOS_NO_WAIT;
        return 0;
    }

    if (absTimeout == NULL) {
        *ticks = LOS_WAIT_FOREVER;
        return 0;
    }

    if (!ValidTimeSpec(absTimeout)) {
        errno = EINVAL;
        return -1;
    }

    *ticks = OsTimeSpec2Tick(absTimeout);
    return 0;
}

STATIC INLINE BOOL MqParamCheck(mqd_t personal, const char *msg, size_t msgLen)
{
    (void)personal;
    if ((msg == NULL) || (msgLen == 0)) {
        errno = EINVAL;
        return FALSE;
    }
    return TRUE;
}

#define OS_MQ_GOTO_ERROUT_UNLOCK_IF(expr, errcode) \
    if (expr) {                        \
        errno = errcode;                 \
        goto ERROUT_UNLOCK;                     \
    }
#define OS_MQ_GOTO_ERROUT_IF(expr, errcode) \
    if (expr) {                        \
        errno = errcode;                 \
        goto ERROUT;                     \
    }
int mq_timedsend(mqd_t personal, const char *msg, size_t msgLen, unsigned int msgPrio,
                 const struct timespec *absTimeout)
{
    UINT32 mqueueID, err;
    UINT64 absTicks;
    struct mqarray *mqueueCB = NULL;
    struct mqpersonal *privateMqPersonal = NULL;

    OS_MQ_GOTO_ERROUT_IF(!MqParamCheck(personal, msg, msgLen), errno);

    OS_MQ_GOTO_ERROUT_IF(msgPrio > (MQ_PRIO_MAX - 1), EINVAL);

    (VOID)pthread_mutex_lock(&g_mqueueMutex);
    privateMqPersonal = (struct mqpersonal *)personal;
    OS_MQ_GOTO_ERROUT_UNLOCK_IF(privateMqPersonal->mq_status != MQ_USE_MAGIC, EBADF);

    mqueueCB = privateMqPersonal->mq_posixdes;
    OS_MQ_GOTO_ERROUT_UNLOCK_IF(msgLen > (size_t)(mqueueCB->mqcb->queueSize - sizeof(UINT32)), EMSGSIZE);

    OS_MQ_GOTO_ERROUT_UNLOCK_IF((((UINT32)privateMqPersonal->mq_flags & (UINT32)O_WRONLY) != (UINT32)O_WRONLY) &&
                                (((UINT32)privateMqPersonal->mq_flags & (UINT32)O_RDWR) != (UINT32)O_RDWR),
                                EBADF);

    OS_MQ_GOTO_ERROUT_UNLOCK_IF(ConvertTimeout(privateMqPersonal->mq_flags, absTimeout, &absTicks) == -1, errno);
    mqueueID = mqueueCB->mq_id;
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);

    err = LOS_QueueWriteCopy(mqueueID, (VOID *)msg, (UINT32)msgLen, (UINT32)absTicks);
    if (MapMqErrno(err) != ENOERR) {
        goto ERROUT;
    }
    return 0;
ERROUT_UNLOCK:
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
ERROUT:
    return -1;
}

ssize_t mq_timedreceive(mqd_t personal, char *msg, size_t msgLen, unsigned int *msgPrio,
                        const struct timespec *absTimeout)
{
    UINT32 mqueueID, err;
    UINT32 receiveLen;
    UINT64 absTicks;
    struct mqarray *mqueueCB = NULL;
    struct mqpersonal *privateMqPersonal = NULL;

    if (!MqParamCheck(personal, msg, msgLen)) {
        goto ERROUT;
    }

    if (msgPrio != NULL) {
        *msgPrio = 0;
    }

    (VOID)pthread_mutex_lock(&g_mqueueMutex);
    privateMqPersonal = (struct mqpersonal *)personal;
    if (privateMqPersonal->mq_status != MQ_USE_MAGIC) {
        errno = EBADF;
        goto ERROUT_UNLOCK;
    }

    mqueueCB = privateMqPersonal->mq_posixdes;
    if (msgLen < (size_t)(mqueueCB->mqcb->queueSize - sizeof(UINT32))) {
        errno = EMSGSIZE;
        goto ERROUT_UNLOCK;
    }

    if (((UINT32)privateMqPersonal->mq_flags & (UINT32)O_WRONLY) == (UINT32)O_WRONLY) {
        errno = EBADF;
        goto ERROUT_UNLOCK;
    }

    if (ConvertTimeout(privateMqPersonal->mq_flags, absTimeout, &absTicks) == -1) {
        goto ERROUT_UNLOCK;
    }

    receiveLen = msgLen;
    mqueueID = mqueueCB->mq_id;
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);

    err = LOS_QueueReadCopy(mqueueID, (VOID *)msg, &receiveLen, (UINT32)absTicks);
    if (MapMqErrno(err) == ENOERR) {
        return (ssize_t)receiveLen;
    } else {
        goto ERROUT;
    }

ERROUT_UNLOCK:
    (VOID)pthread_mutex_unlock(&g_mqueueMutex);
ERROUT:
    return -1;
}

/* not support the prio */
int mq_send(mqd_t personal, const char *msg_ptr, size_t msg_len, unsigned int msg_prio)
{
    return mq_timedsend(personal, msg_ptr, msg_len, msg_prio, NULL);
}

ssize_t mq_receive(mqd_t personal, char *msg_ptr, size_t msg_len, unsigned int *msg_prio)
{
    return mq_timedreceive(personal, msg_ptr, msg_len, msg_prio, NULL);
}

