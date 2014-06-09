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

#include <stdio.h>
#include <errno.h>
#include <sys/socket.h>

#include "ipc_node.h"

#define IPC_FEATURES_VER0	0x1
#define IPC_FEATURES		IPC_FEATURES_VER0

/*
 * Queue definitions.
 */
#define VTNET_RXQ	0
#define VTNET_TXQ	1
#define VTNET_CTLQ	2	/* NB: not yet supported */

enum ipc_command {
	IPC_COMMAND_INIT,	/* VER0 */
	IPC_COMMAND_CONFIGURE,	/* VER0 */
	IPC_COMMAND_FEATURES,	/* VER0 */
	IPC_COMMAND_KICK,	/* VER0 */
	IPC_COMMAND_RESET,	/* VER0 */
};

struct ipc_command_init {
	uint32_t cmd;
	struct {
		uint32_t nid;
		uint64_t ram_size;
		uint32_t lowmem_limit;
	} body;
} __attribute__((__packed__));

struct ipc_command_configure {
	uint32_t cmd;
	struct {
		uint16_t queue_index;
		uint32_t pfn;
		uint8_t reconfigure_flag;
	} body;
} __attribute__((__packed__));

struct ipc_command_features {
	uint32_t cmd;
	struct {
		uint32_t features;
	} body;
} __attribute__((__packed__));

struct ipc_command_reset {
	uint32_t cmd;
} __attribute__((__packed__));

struct ipc_command_kick {
	uint32_t cmd;
	struct {
		uint16_t queue_index;
	} body;
} __attribute__((__packed__));

int ipc_get_fd(ipc_node *node)
{
	return node->fd;
}

static ssize_t ipc_write_fixed_size(int fd, void *buf,
		ssize_t remain, int ram_fd)
{
	struct iovec iov;
	char cmsgbuf[CMSG_SPACE(sizeof(int))];
	struct cmsghdr *cmsg = (struct cmsghdr *)cmsgbuf;
	ssize_t total;
	int *p;
	struct msghdr msg;

	iov.iov_base = buf;
	iov.iov_len = remain;

	cmsg->cmsg_len = CMSG_LEN(sizeof(int));
	cmsg->cmsg_level = SOL_SOCKET;
	cmsg->cmsg_type = SCM_RIGHTS;
	p = (int *)CMSG_DATA(cmsg);
	*p = ram_fd;

	memset(&msg, 0, sizeof(msg));
	msg.msg_name = NULL;
	msg.msg_namelen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_flags = 0;

	if (ram_fd >= 0) {
		msg.msg_control = cmsgbuf;
		msg.msg_controllen = sizeof(cmsgbuf);
	} else {
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
	}

	total = sendmsg(fd, &msg, MSG_NOSIGNAL);
	if (total != remain) {
		errno = EFAULT;
		return -errno;
	}

	return total;

}

static ssize_t ipc_read_fixed_size(int fd, void *buf,
		ssize_t remain, int *ram_fd)
{
	struct msghdr msg;
	struct iovec iov;
	char cmsgbuf[CMSG_SPACE(sizeof(int))];
	ssize_t total;
	struct cmsghdr *cmsg;
	int *p;

	iov.iov_base = buf;
	iov.iov_len = remain;
	msg.msg_name = NULL;
	msg.msg_namelen = 0;
	msg.msg_iov = &iov;
	msg.msg_iovlen = 1;
	msg.msg_flags = 0;

	if (ram_fd) {
		msg.msg_control = cmsgbuf;
		msg.msg_controllen = sizeof(cmsgbuf);
	} else {
		msg.msg_control = NULL;
		msg.msg_controllen = 0;
	}

	total = recvmsg(fd, &msg, MSG_WAITALL);
	if (total != remain) {
		errno = EFAULT;
		return -errno;
	}

	if (ram_fd) {
		cmsg = (struct cmsghdr *)cmsgbuf;
		p = (int *)CMSG_DATA(cmsg);
		*ram_fd = *p;
	}

	return total;
}

int ipc_node_init(ipc_node *n, uint32_t nid, int fd, int ram_fd,
		  uint64_t ram_size)
{
	bzero(n, sizeof(ipc_node));
	n->fd = fd;
	n->ram_fd = ram_fd;
	n->nid = nid;
	n->ram_size = ram_size;
	return 0;
}

int ipc_socket(void)
{
	return socket(AF_UNIX, SOCK_STREAM, 0);
}

int ipc_connect(int fd, const char *path, size_t len)
{
	struct sockaddr_un caddr;

	bzero(&caddr, sizeof(caddr));
	caddr.sun_family = AF_LOCAL;
	strncpy((char *)&caddr.sun_path, path, len);

	return connect(fd, (struct sockaddr *)&caddr, sizeof(caddr));
}

int ipc_bind(int fd, const char *path, size_t len)
{
	struct sockaddr_un saddr;

	bzero(&saddr, sizeof(saddr));
	saddr.sun_family = AF_UNIX;
	strncpy((char *)&saddr.sun_path, path, len);
	return bind(fd, (const struct sockaddr *)&saddr, sizeof(saddr));
}

int ipc_listen(int fd)
{
	return listen(fd, 0);
}

int ipc_accept(int fd)
{
	struct sockaddr_un caddr;
	socklen_t addrlen = 0;

	/* accept */
	return accept(fd, (struct sockaddr *)&caddr, &addrlen);
}

static int ipc_send_kick(ipc_node *node, uint16_t curq)
{
	struct ipc_command_kick data;

	if (!(node->peer_features & IPC_FEATURES_VER0)) {
		errno = ENOSYS;
		return -errno;
	}
	data.cmd = IPC_COMMAND_KICK;
	data.body.queue_index = curq;
	return ipc_write_fixed_size(node->fd, &data, sizeof(data), -1);
}

int ipc_server_kick(ipc_node *node, uint16_t curq)
{
	return ipc_send_kick(node, curq);
}

int ipc_client_kick(ipc_node *node, uint16_t curq)
{
	node->client_kicked = 1;
	return ipc_send_kick(node, curq);
}

int ipc_client_guest_features(ipc_node *node, uint32_t negotiated_caps)
{
	struct ipc_command_features data;

	node->negotiated_caps = negotiated_caps; /* save features */
	if (!(node->peer_features & IPC_FEATURES_VER0)) {
		errno = ENOSYS;
		return -errno;
	}
	data.cmd = IPC_COMMAND_FEATURES;
	data.body.features = negotiated_caps;
	return ipc_write_fixed_size(node->fd, &data, sizeof(data), -1);
}

int ipc_client_pfn(ipc_node *node, uint16_t curq, uint32_t pfn)
{
	struct ipc_command_configure data;

	if (curq >= VTNET_MAXQ - 1) {
		errno = EINVAL;
		return -EINVAL;
	}
	node->pfn[curq] = pfn;	/* save pfn */
	if (!(node->peer_features & IPC_FEATURES_VER0)) {
		errno = ENOSYS;
		return -errno;
	}
	data.cmd = IPC_COMMAND_CONFIGURE;
	data.body.queue_index = curq;
	data.body.pfn = pfn;
	data.body.reconfigure_flag = 0;
	return ipc_write_fixed_size(node->fd, &data, sizeof(data), -1);
}

int ipc_client_reset(ipc_node *node)
{
	uint16_t queue_index;
	struct ipc_command_reset data;

	if (!(node->peer_features & IPC_FEATURES_VER0)) {
		errno = ENOSYS;
		return -errno;
	}
	node->client_kicked = 0;
	node->negotiated_caps = 0;
	for (queue_index = 0; queue_index < VTNET_MAXQ - 1; queue_index++)
		node->pfn[queue_index] = 0;
	data.cmd = IPC_COMMAND_RESET;
	return ipc_write_fixed_size(node->fd, &data, sizeof(data), -1);
}

int ipc_server_init_sequence(ipc_node *node, uint32_t *nid, int *ram_fd,
			     uint64_t *ram_size, uint32_t *lowmem_limit)
{
	int ret;
	int fd = node->fd;
	uint32_t my_features = IPC_FEATURES;
	struct ipc_command_init data;

	/* receive features that client supports */
	ret = ipc_read_fixed_size(fd, &node->peer_features,
				  sizeof(node->peer_features), NULL);
	if (ret < 0)
		return ret;
	/* send features that server supports */
	ret = ipc_write_fixed_size(fd, &my_features, sizeof(my_features), -1);
	if (ret < 0)
		return ret;
	/* receive init command */
	ret = ipc_read_fixed_size(fd, &data, sizeof(data), ram_fd);
	if (ret < 0)
		return ret;
	if (data.cmd != IPC_COMMAND_INIT) {
		/* FIXME: Close the received file descriptor */
		/* Currently ipc_read_fixed_size does not check that a
		 * file descriptor was received or not */
		fprintf(stderr, "Invalid command in init sequence\n");
		errno = EINVAL;
		return -errno;
	}
	*nid = data.body.nid;
	*ram_size = data.body.ram_size;
	*lowmem_limit = data.body.lowmem_limit;

	return 0;
}

int ipc_client_init_sequence(ipc_node *node, uint32_t lowmem_limit, int fd)
{
	int ret;
	struct ipc_command_init data;
	uint32_t my_features = IPC_FEATURES;

	/* send features that client supports */
	ret = ipc_write_fixed_size(fd, &my_features, sizeof(my_features), -1);
	if (ret < 0)
		return ret;
	/* receive features that server supports */
	ret = ipc_read_fixed_size(fd, &node->peer_features,
				  sizeof(node->peer_features), NULL);
	if (ret < 0)
		return ret;
	/* features compatibility check */
	if (!(my_features & node->peer_features)) {
		fprintf(stderr, "compatibility check failed\n");
		errno = EFAULT;
		return -errno;
	}

	/* send init command */
	if (!(node->peer_features & IPC_FEATURES_VER0)) {
		errno = ENOSYS;
		return -errno;
	}
	data.cmd = IPC_COMMAND_INIT;
	data.body.nid = node->nid;
	data.body.ram_size = node->ram_size;
	data.body.lowmem_limit = lowmem_limit;
	return ipc_write_fixed_size(fd, &data, sizeof(data), node->ram_fd);
}

int ipc_server_recv(ipc_node *node, ipc_msg_type *type, uint16_t *curq,
		    uint32_t *negotiated_caps, uint32_t *pfn)
{
	int ret;
	uint32_t cmd;
	int fd = node->fd;

	ret = ipc_read_fixed_size(fd, &cmd, sizeof(cmd), NULL);
	if (ret < 0)
		return ret;
	switch ((enum ipc_command)cmd) {
	case IPC_COMMAND_INIT:
		break;
	case IPC_COMMAND_CONFIGURE:
	{
		struct ipc_command_configure data;

		ret = ipc_read_fixed_size(fd, &data.body, sizeof(data.body),
					  NULL);
		if (ret < 0)
			return ret;
		if (data.body.reconfigure_flag)
			*type = IPC_MESSAGE_TYPE_RECONFIG;
		else
			*type = IPC_MESSAGE_TYPE_PFN;
		*curq = data.body.queue_index;
		*pfn = data.body.pfn;
		return 0;
	}
	case IPC_COMMAND_FEATURES:
	{
		struct ipc_command_features data;

		ret = ipc_read_fixed_size(fd, &data.body, sizeof(data.body),
					  NULL);
		if (ret < 0)
			return ret;
		*type = IPC_MESSAGE_TYPE_GUEST_FEATURES;
		*negotiated_caps = data.body.features;
		return 0;
	}
	case IPC_COMMAND_KICK:
	{
		struct ipc_command_kick data;

		ret = ipc_read_fixed_size(fd, &data.body, sizeof(data.body),
					  NULL);
		if (ret < 0)
			return ret;
		*type = IPC_MESSAGE_TYPE_KICK;
		*curq = data.body.queue_index;
		return 0;
	}
	case IPC_COMMAND_RESET:
		*type = IPC_MESSAGE_TYPE_RESET;
		return 0;
	}
	*type = IPC_MESSAGE_TYPE_INVALID;
	return 0;
}

int ipc_client_recv(ipc_node *node, ipc_msg_type *type, uint16_t *curq)
{
	int ret;
	uint32_t cmd;
	int fd = node->fd;

	ret = ipc_read_fixed_size(fd, &cmd, sizeof(cmd), NULL);
	if (ret < 0)
		return ret;
	switch ((enum ipc_command)cmd) {
	case IPC_COMMAND_INIT:
	case IPC_COMMAND_CONFIGURE:
	case IPC_COMMAND_FEATURES:
		break;
	case IPC_COMMAND_KICK:
	{
		struct ipc_command_kick data;

		ret = ipc_read_fixed_size(fd, &data.body, sizeof(data.body),
					  NULL);
		if (ret < 0)
			return ret;
		*type = IPC_MESSAGE_TYPE_KICK;
		*curq = data.body.queue_index;
		return 0;
	}
	case IPC_COMMAND_RESET:
		break;
	}
	*type = IPC_MESSAGE_TYPE_INVALID;
	return 0;
}

int ipc_client_reconfigure(ipc_node *node)
{
	int ret;
	uint16_t queue_index;
	uint32_t pfn;
	int fd = node->fd;

	if (!(node->peer_features & IPC_FEATURES_VER0)) {
		errno = ENOSYS;
		return -errno;
	}
	for (;;) {
		struct ipc_command_features data;

		data.cmd = IPC_COMMAND_FEATURES;
		data.body.features = node->negotiated_caps;
		ret = ipc_write_fixed_size(fd, &data, sizeof(data), -1);
		if (ret < 0)
			return ret;
		/* If the value is changed by other function, send
		 * again. */
		if (data.body.features == node->negotiated_caps)
			break;
	}
	for (queue_index = 0; queue_index < VTNET_MAXQ - 1; queue_index++) {
		struct ipc_command_configure data;

		for (;;) {
			pfn = node->pfn[queue_index];
			data.cmd = IPC_COMMAND_CONFIGURE;
			data.body.queue_index = queue_index;
			data.body.pfn = pfn;
			data.body.reconfigure_flag = node->client_kicked;
			ret = ipc_write_fixed_size(fd, &data, sizeof(data), -1);
			if (ret < 0)
				return ret;
			/* If the value is changed by other function,
			 * send again. */
			if (pfn == node->pfn[queue_index])
				break;
		}
	}
	if (node->client_kicked) {
		for (queue_index = 0; queue_index < VTNET_MAXQ - 1;
		     queue_index++) {
			ret = ipc_send_kick(node, queue_index);
			if (ret < 0)
				return ret;
		}
	}
	return 0;
}

int ipc_client_wait_for_end(ipc_node *node)
{
	int ret;
	ipc_msg_type type;
	uint16_t curq;

	do
		ret = ipc_client_recv(node, &type, &curq);
	while (ret >= 0);
	return 0;
}
