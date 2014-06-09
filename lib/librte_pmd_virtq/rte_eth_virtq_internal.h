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

#ifndef _RTE_ETH_VIRTQ_INTERNAL_H_
#define _RTE_ETH_VIRTQ_INTERNAL_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <pthread.h>
#include <rte_ether.h>

#include "rte_eth_virtq_virtio.h"
#include "rte_eth_virtq_vmmapi.h"
#include "rte_eth_virtq_pci_virtio_net.h"

struct virtio_softc;

enum virtq_dev_status {
	DEV_STOP = 1,	/* device not started */
	DEV_START,	/* device start */
	DEV_OPEN,	/* ipc socket is opened */
};

enum virtq_queue_status {
	VQ_STOP = 1,	/* virtqueue not started */
	VQ_KICK,	/* virtqueue is configured and kicked */
	VQ_ACCESS	/* virtqueue is accessed by PMD */
};

struct virtq_dev {
	int id;
	const char *name;
	pthread_t th;
	unsigned lcore_id;
	struct ether_addr *eth_addr;
	rte_atomic32_t status;
	rte_atomic32_t vq_status[VTNET_MAXQ - 1];

	ipc_node qemu;
	struct virtio_softc *vs;
	struct vmctx *ctx;
	struct rte_eth_dev *edev;
	struct pci_devinst *pdi;
	int ram_fd;
	uint64_t ram_size;
	uint32_t lowmem_limit;
	uint8_t resetting;
	uint16_t rxhdr_len;
	uint8_t vrh_buf_val;
};

struct pmd_queue {
	struct virtq_dev *vdev;
	struct rte_mempool *mb_pool;

	volatile unsigned long rx_pkts;
	volatile unsigned long tx_pkts;
	volatile unsigned long rx_errs;
	volatile unsigned long tx_errs;
	volatile unsigned long rx_bytes;
	volatile unsigned long tx_bytes;
};

#ifdef __cplusplus
}
#endif

#endif
