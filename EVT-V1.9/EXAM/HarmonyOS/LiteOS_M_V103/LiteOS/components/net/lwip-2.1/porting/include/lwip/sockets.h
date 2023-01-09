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

#ifndef _LWIP_PORTING_SOCKETS_H_
#define _LWIP_PORTING_SOCKETS_H_

#include <sys/socket.h>
#include <poll.h>
#include <netinet/tcp.h>
#include <netinet/in.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <limits.h>
#include <fcntl.h>
#include_next <lwip/sockets.h>
#include <fatfs.h>

#ifdef __cplusplus
extern "C" {
#endif

#if FD_SETSIZE < (LWIP_SOCKET_OFFSET + MEMP_NUM_NETCONN)
#error "external FD_SETSIZE too small for number of sockets"
#else
#define LWIP_SELECT_MAXNFDS FD_SETSIZE
#endif

#if IOV_MAX > 0xFFFF
#error "IOV_MAX larger than supported by LwIP"
#endif

#if LWIP_UDP && LWIP_UDPLITE
#define UDPLITE_SEND_CSCOV 0x01 /* sender checksum coverage */
#define UDPLITE_RECV_CSCOV 0x02 /* minimal receiver checksum coverage */
#endif

// For BSD 4.4 socket sa_len compatibility
#define DF_NADDR(addr)
#define SA_LEN(addr, _)  (IP_IS_V4_VAL(addr) ? sizeof(struct sockaddr_in) : sizeof(struct sockaddr_in6))
#define sa_len sa_data[0] * 0 + SA_LEN(naddr, _)
#define sin_len sin_zero[0]
#define sin6_len sin6_addr.s6_addr[0]

// for sockets.c, TCP_KEEPALIVE is not supported currently
#define TCP_KEEPALIVE   0xFF
#define SIN_ZERO_LEN    8

int closesocket(int sockfd);
int ioctlsocket(int s, long cmd, void *argp);

#ifdef __cplusplus
}
#endif

#endif /* _LWIP_PORTING_SOCKETS_H_ */
