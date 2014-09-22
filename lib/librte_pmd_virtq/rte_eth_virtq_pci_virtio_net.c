/*-
 *   BSD LICENSE
 *
 *   Copyright (c) 2011 NetApp, Inc.
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

#include <sys/socket.h>
#include <unistd.h>
#include <assert.h>

#include <rte_mbuf.h>

#include "rte_eth_virtq_pci_virtio_net.h"
#include "rte_eth_virtq_pci_emul.h"
#include "rte_eth_virtq_virtio.h"
#include "rte_eth_virtq_internal.h"

#define VTNET_RINGSZ	256

#define VTNET_MAXSEGS	32

/*
 * PCI config-space "registers"
 */
struct virtio_net_config {
	uint8_t  mac[6];
	uint16_t status;
} __attribute__((__packed__));

/*
 * Per-device softc
#include <rte_ethdev.h>
 */
struct pci_vtnet_softc {
	struct virtio_softc vsc_vs;
	struct vqueue_info vsc_queues[VTNET_MAXQ - 1];
	pthread_mutex_t vsc_mtx;

	int		vsc_tapfd;
	volatile int	resetting;	/* set and checked outside lock */
};

static void pci_vtnet_reset(void *);
/* static void pci_vtnet_notify(void *, struct vqueue_info *); */

static struct virtio_consts vtnet_vi_consts = {
	"vtnet",		/* our name */
	VTNET_MAXQ - 1,		/* we currently support 2 virtqueues */
	sizeof(struct virtio_net_config), /* config reg size */
	pci_vtnet_reset,	/* reset */
	NULL,			/* device-wide qnotify -- not used */
};

static void
pci_vtnet_reset(void *vsc)
{
	struct pci_vtnet_softc *sc = vsc;

	sc->resetting = 1;

	/* now reset rings, MSI-X vectors, and negotiated capabilities */
	vi_reset_dev(&sc->vsc_vs);

	sc->resetting = 0;
}

static inline struct rte_mbuf *
rte_rxmbuf_alloc(struct rte_mempool *mp)
{
	struct rte_mbuf *m;

	m = __rte_mbuf_raw_alloc(mp);
	__rte_mbuf_sanity_check_raw(m, RTE_MBUF_PKT, 0);

	return m;
}

uint16_t
virtq_pmd_virtq_rx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	struct pmd_queue *rx_queue = q;
	struct virtq_dev *vdev = rx_queue->vdev;
	uint16_t num_rx = 0;
	rte_atomic32_t *vq_status;
	uint64_t total_bytes = 0;

	struct pci_vtnet_softc *sc;
	struct vqueue_info *vq;
	struct rte_mbuf *mbuf;

	if (unlikely(vdev == NULL || nb_bufs == 0 || vdev->resetting))
		return 0;

	vq_status = &vdev->vq_status[VTNET_RXQ];

	/* Here is the status check to access virtqueue */
	if (unlikely(rte_atomic32_read(vq_status) != VQ_KICK))
			return 0;
	rte_atomic32_set(vq_status, VQ_ACCESS);
	/* double check is needed because the vq_status might be
	 * changed by other threads */
	if (unlikely(rte_atomic32_read(vq_status) != VQ_ACCESS))
		return 0;

	sc = (struct pci_vtnet_softc *)rx_queue->vdev->vs;
	/* rx thread of PMD should access the tx queue of a guest virtqueue */
	vq = &sc->vsc_queues[VTNET_TXQ];

	vq_startchains(vq);
	while (vq_has_descs(vq)) {
		/*
		 * Run through entries, placing them into
		 * iovecs and sending when an end-of-packet is found
		 */
		total_bytes += pci_vtnet_proctx(
				sc, vq, 1, rx_queue->mb_pool, &mbuf, rx_queue->in_port);
		if (mbuf == NULL)
			break;

		bufs[num_rx] = mbuf;
		num_rx++;

		if (unlikely(num_rx >= nb_bufs))
			break;
	}

	/*
	 * Generate an interrupt if needed.
	 */
	vq_endchains(vq, 1);

	rx_queue->rx_pkts += num_rx;
	rx_queue->rx_bytes += total_bytes;

	rte_atomic32_set(vq_status, VQ_KICK);

	return num_rx;
}

inline uint64_t
pci_vtnet_proctx(void *dev, struct vqueue_info *vq,
		int send, struct rte_mempool *mb_pool,
		struct rte_mbuf **mbuf, uint8_t in_port)
{
	struct pci_vtnet_softc *sc __rte_unused = (struct pci_vtnet_softc *)dev;
	struct iovec iov[VTNET_MAXSEGS + 1];
	int i, n;
	uint16_t plen, tlen, buf_size;
	struct rte_pktmbuf_pool_private *mbp_priv;
	uint8_t *dst;

	/*
	 * Obtain chain of descriptors.  The first one is
	 * really the header descriptor, so we need to sum
	 * up two lengths: packet length and transfer length.
	 */
	n = vq_getchain(vq, iov, VTNET_MAXSEGS, NULL);
	assert(n >= 1 && n <= VTNET_MAXSEGS);
	plen = 0;
	tlen = iov[0].iov_len;
	for (i = 1; i < n; i++) {
		plen += iov[i].iov_len;
		tlen += iov[i].iov_len;
	}

	if (send) {
		*mbuf = rte_pktmbuf_alloc(mb_pool);
		if (unlikely(!(*mbuf)))
			return 0;

		mbp_priv = (struct rte_pktmbuf_pool_private *)
			((char *)mb_pool + sizeof(struct rte_mempool));
		buf_size = (uint16_t) (mbp_priv->mbuf_data_room_size -
				RTE_PKTMBUF_HEADROOM);
		if (buf_size >= plen) {
			dst = (*mbuf)->pkt.data;
			for (i = 1; i < n; i++) {
				rte_memcpy(dst,
					iov[i].iov_base, iov[i].iov_len);
				dst += iov[i].iov_len;
			}
			(*mbuf)->pkt.data_len = plen;
			(*mbuf)->pkt.pkt_len = (*mbuf)->pkt.data_len;
			(*mbuf)->pkt.in_port = in_port;
			(*mbuf)->pkt.nb_segs = 1;
			(*mbuf)->pkt.next = NULL;
		} else
			return 0;
	}

	/* chain is processed, release it and set tlen */
	vq_relchain(vq, tlen);

	return (uint64_t)plen;
}

uint16_t
virtq_pmd_virtq_tx(void *q, struct rte_mbuf **bufs, uint16_t nb_bufs)
{
	struct pmd_queue *tx_queue = q;
	struct virtq_dev *vdev = tx_queue->vdev;
	uint16_t num_tx = 0;
	rte_atomic32_t *vq_status;

	struct pci_vtnet_softc *sc;
	struct vqueue_info *vq;
	struct rte_mbuf *mbuf;
	struct virtio_net_rxhdr *vrx;
	uint64_t rxhdr_len;
	uint64_t total_bytes = 0;
	struct iovec iov;
	uint8_t *buf, *vrh_buf;
	uint8_t vrh_buf_val;
	int len;

	if (unlikely(vdev == NULL || nb_bufs == 0 || vdev->resetting))
		return 0;

	vq_status = &vdev->vq_status[VTNET_TXQ];

	/* Here is the status check to access virtqueue */
	if (unlikely(rte_atomic32_read(vq_status) != VQ_KICK))
		return 0;
	rte_atomic32_set(vq_status, VQ_ACCESS);
	/* double check is needed because the vq_status might be
	 * changed by other threads */
	if (unlikely(rte_atomic32_read(vq_status) != VQ_ACCESS))
		return 0;

	sc = (struct pci_vtnet_softc *)tx_queue->vdev->vs;
	/* tx thread of PMD should access the rx queue of a guest virtqueue */
	vq = &sc->vsc_queues[VTNET_RXQ];
	rxhdr_len = vdev->rxhdr_len;
	vrh_buf_val = vdev->vrh_buf_val;

	vq_startchains(vq);

	while ((vq_has_descs(vq)) && (nb_bufs > num_tx)) {
		/*
		 * Get descriptor chain, which should have just
		 * one descriptor in it.
		 * ??? allow guests to use multiple descs?
		 */
		assert(vq_getchain(vq, &iov, 1, NULL) == 1);

		/*
		 * Get a pointer to the rx header, and use the
		 * data immediately following it for the packet buffer.
		 */
		vrx = iov.iov_base;
		buf = (uint8_t *)((uint64_t)iov.iov_base + rxhdr_len);
		vrh_buf = (uint8_t *)((uint64_t)buf - 2);

		mbuf = bufs[num_tx];
		len = iov.iov_len - rxhdr_len;
		if (len >= mbuf->pkt.data_len) {
			rte_memcpy(buf, mbuf->pkt.data, mbuf->pkt.data_len);
			rte_pktmbuf_free(mbuf);
		} else
			break;

		/*
		 * The only valid field in the rx packet header is the
		 * number of buffers, which is always 1 without TSO
		 * support.
		 */
		memset(vrx, 0, rxhdr_len);
		*vrh_buf = vrh_buf_val;

		/*
		 * Release this chain and handle more chains.
		 */
		vq_relchain(vq, mbuf->pkt.data_len + rxhdr_len);

		total_bytes += mbuf->pkt.data_len;
		num_tx++;
	}

	/*
	 * Generate an interrupt if needed.
	 */
	vq_endchains(vq, 1);

	tx_queue->tx_pkts += num_tx;
	tx_queue->tx_errs += nb_bufs - num_tx;
	tx_queue->tx_bytes += total_bytes;

	rte_atomic32_set(vq_status, VQ_KICK);

	return num_tx;
}

void pci_vtnet_close(void *dev)
{
	struct virtio_softc *vs;
	struct pci_devinst *vs_pi;
	struct pci_vtnet_softc *sc;

	vs = (struct virtio_softc *)dev;
	sc = (struct pci_vtnet_softc *)dev;
	vs_pi = vs->vs_pi;

	close(sc->vsc_tapfd);
	vi_reset_dev(vs);
	free(vs_pi);
	free(sc);
}

int pci_vtnet_init(struct pci_devinst **pdev, void **dev)
{
	char *devname __rte_unused;
	int mac_provided __rte_unused;
	struct pci_devinst *pi;
	struct pci_vtnet_softc *sc;
	struct pci_vtnet_softc **vdev;

	*pdev = malloc(sizeof(struct pci_devinst));
	pi = *pdev;
	bzero(pi, sizeof(struct pci_devinst));

	sc = malloc(sizeof(struct pci_vtnet_softc));
	memset(sc, 0, sizeof(struct pci_vtnet_softc));
	vdev = (struct pci_vtnet_softc **)(struct virtio_softc **)dev;
	*vdev = sc;

	pthread_mutex_init(&sc->vsc_mtx, NULL);

	vi_softc_linkup(&sc->vsc_vs, &vtnet_vi_consts, sc, pi, sc->vsc_queues);
	sc->vsc_queues[VTNET_RXQ].vq_qsize = VTNET_RINGSZ;
	sc->vsc_queues[VTNET_RXQ].vq_notify = NULL;
	sc->vsc_queues[VTNET_TXQ].vq_qsize = VTNET_RINGSZ;
	sc->vsc_queues[VTNET_TXQ].vq_notify = NULL;
#ifdef notyet
	sc->vsc_queues[VTNET_CTLQ].vq_qsize = VTNET_RINGSZ;
	sc->vsc_queues[VTNET_CTLQ].vq_notify = pci_vtnet_ping_ctlq;
#endif

	/*
	 * Attempt to open the tap device and read the MAC address
	 * if specified
	 */
	mac_provided = 0;
	sc->vsc_tapfd = -1;

	sc->resetting = 0;

	return 0;
}
