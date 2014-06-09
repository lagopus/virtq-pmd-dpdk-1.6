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

#ifndef	_RTE_ETH_VIRTQ_PCI_VIRTIO_NET_H_
#define	_RTE_ETH_VIRTQ_PCI_VIRTIO_NET_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <rte_mbuf.h>
#include "rte_eth_virtq_virtio.h"
#include "rte_eth_virtq_pci_emul.h"

/*
 * Queue definitions.
 */
#define VTNET_RXQ	0
#define VTNET_TXQ	1
#define VTNET_CTLQ	2	/* NB: not yet supported */
#define VTNET_MAXQ	3

/*
 * Host capabilities.  Note that we only offer a few of these.
 */
#define	VIRTIO_NET_F_CSUM	(1 <<  0) /* host handles partial cksum */
#define	VIRTIO_NET_F_GUEST_CSUM	(1 <<  1) /* guest handles partial cksum */
#define	VIRTIO_NET_F_MAC	(1 <<  5) /* host supplies MAC */
#define	VIRTIO_NET_F_GSO_DEPREC	(1 <<  6) /* deprecated: host handles GSO */
#define	VIRTIO_NET_F_GUEST_TSO4	(1 <<  7) /* guest can rcv TSOv4 */
#define	VIRTIO_NET_F_GUEST_TSO6	(1 <<  8) /* guest can rcv TSOv6 */
#define	VIRTIO_NET_F_GUEST_ECN	(1 <<  9) /* guest can rcv TSO with ECN */
#define	VIRTIO_NET_F_GUEST_UFO	(1 << 10) /* guest can rcv UFO */
#define	VIRTIO_NET_F_HOST_TSO4	(1 << 11) /* host can rcv TSOv4 */
#define	VIRTIO_NET_F_HOST_TSO6	(1 << 12) /* host can rcv TSOv6 */
#define	VIRTIO_NET_F_HOST_ECN	(1 << 13) /* host can rcv TSO with ECN */
#define	VIRTIO_NET_F_HOST_UFO	(1 << 14) /* host can rcv UFO */
#define	VIRTIO_NET_F_MRG_RXBUF	(1 << 15) /* host can merge RX buffers */
#define	VIRTIO_NET_F_STATUS	(1 << 16) /* config status field available */
#define	VIRTIO_NET_F_CTRL_VQ	(1 << 17) /* control channel available */
#define	VIRTIO_NET_F_CTRL_RX	(1 << 18) /* control channel RX mode support */
#define	VIRTIO_NET_F_CTRL_VLAN	(1 << 19) /* control channel VLAN filtering */
#define	VIRTIO_NET_F_GUEST_ANNOUNCE \
				(1 << 21) /* guest can send gratuitous pkts */

/*
 * Fixed network header size
 */
struct virtio_net_rxhdr {
	uint8_t vrh_flags;
	uint8_t vrh_gso_type;
	uint16_t vrh_hdr_len;
	uint16_t vrh_gso_size;
	uint16_t vrh_csum_start;
	uint16_t vrh_csum_offset;
	uint16_t vrh_bufs;
} __attribute__((__packed__));

int pci_vtnet_init(struct pci_devinst **pdev, void **dev);
void pci_vtnet_close(void *dev);
inline uint64_t pci_vtnet_proctx(void *dev, struct vqueue_info *vq,
		int send, struct rte_mempool *mb_pool, struct rte_mbuf **mbuf);
uint16_t virtq_pmd_virtq_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs);
uint16_t virtq_pmd_virtq_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs);

#ifdef __cplusplus
}
#endif

#endif /* _PCI_VIRTIO_NET_H_ */
