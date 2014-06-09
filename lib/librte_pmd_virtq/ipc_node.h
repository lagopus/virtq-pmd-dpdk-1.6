/*-
 *   BSD LICENSE
 *
 *   Copyright (C) 2014 Nippon Telegraph and Telephone Corporation.
 *   All rights reserved.
 *
 *   Redistribution and use in source and binary forms, with or without
 *   modification, are permitted provided that the following conditions
 *   are met:
 *
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in
 *       the documentation and/or other materials provided with the
 *       distribution.
 *
 *   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef	_IPC_NODE_H_
#define	_IPC_NODE_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <sys/un.h>

#define NODE_NAME_SIZE			(64)
#define VTNET_MAXQ	3

typedef enum {
	IPC_MESSAGE_TYPE_INVALID,
	IPC_MESSAGE_TYPE_GUEST_FEATURES, /* Config device (s->c) */
	IPC_MESSAGE_TYPE_PFN,		/* Config queue (s->c) */
	IPC_MESSAGE_TYPE_RESET,		/* Reset device (s->c) */
	IPC_MESSAGE_TYPE_KICK,		/* Kick (s<->c) */
	IPC_MESSAGE_TYPE_RECONFIG,	/* Reconfigure device & queue (s->c) */
} ipc_msg_type;

/*
 * ipc local data
 *
 * The structure is used for managing connectd clients and servers.
 */
typedef struct _ipc_node {
	/* node info */
	int fd;			/* file descriptor to access this node */

	int ram_fd;		/* fd to access physical memory */
	uint32_t pfn[VTNET_MAXQ - 1];
				/* queue addr */
	uint32_t negotiated_caps;
	uint64_t ram_size;	/* size of physical memory */
	uint32_t nid;		/* node id */
	uint32_t peer_features;	/* peer ipc features */
	uint8_t	client_kicked;	/* kicked flag for a client */
} ipc_node;

int ipc_get_fd(ipc_node *);

int ipc_node_init(ipc_node *, uint32_t, int, int, uint64_t);
int ipc_socket(void);
int ipc_connect(int, const char *, size_t);
int ipc_bind(int, const char *, size_t);
int ipc_listen(int);
int ipc_accept(int);

int ipc_server_init_sequence(ipc_node *, uint32_t *, int *, uint64_t *,
			     uint32_t *);
int ipc_server_recv(ipc_node *, ipc_msg_type *, uint16_t *, uint32_t *,
		    uint32_t *);
int ipc_server_kick(ipc_node *, uint16_t);

int ipc_client_init_sequence(ipc_node *, uint32_t, int);
int ipc_client_recv(ipc_node *, ipc_msg_type *, uint16_t *);
int ipc_client_kick(ipc_node *, uint16_t);
int ipc_client_guest_features(ipc_node *, uint32_t);
int ipc_client_pfn(ipc_node *, uint16_t, uint32_t);
int ipc_client_reset(ipc_node *);
int ipc_client_reconfigure(ipc_node *node);
int ipc_client_wait_for_end(ipc_node *node);

#ifdef __cplusplus
}
#endif

#endif /* _IPC_NODE_H_*/
