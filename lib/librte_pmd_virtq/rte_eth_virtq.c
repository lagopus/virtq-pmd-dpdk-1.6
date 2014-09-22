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

#include <unistd.h>
#include <sys/socket.h>

#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_cycles.h>

#include "rte_eth_virtq.h"
#include "rte_eth_pcap_arg_parser.h"
#include "rte_eth_virtq_internal.h"
#include "rte_eth_virtq_pci_virtio_net.h"
#include "rte_eth_virtq_thread.h"

#define ETH_VIRTQ_MAC_ADDR_ARG		"mac"
#define ETH_VIRTQ_LCORE_ID_ARG		"lcore_id"

/* #define DEBUG_PRINT_ENABLED */
#if defined(DEBUG_PRINT_ENABLED)
#define dbg(format, args...) \
	printf("%s(%d): " format, __func__, __LINE__, ##args)
#else
#define dbg(format, args...) ((void)0)
#endif

static struct virtq_dev *virtq_dev[RTE_MAX_ETHPORTS];
static uint8_t virtq_dev_num;

struct virtq_connection_accepter {
	const char *name;
	int fd;
	pthread_t th;
	pthread_attr_t attr;
} virtq_accepter;

struct pmd_internals {
	unsigned nb_rx_queues;
	unsigned nb_tx_queues;

	struct pmd_queue rx_pmd_queues[RTE_PMD_RING_MAX_RX_RINGS];
	struct pmd_queue tx_pmd_queues[RTE_PMD_RING_MAX_TX_RINGS];
};

static const char *drivername = "VirtQueue PMD";
static struct rte_eth_link pmd_link = {
		.link_speed = 10000,
		.link_duplex = ETH_LINK_FULL_DUPLEX,
		.link_status = 0
};

const char *virtq_valid_arguments[] = {
	ETH_VIRTQ_MAC_ADDR_ARG,
	ETH_VIRTQ_LCORE_ID_ARG,
	NULL
};

static int
eth_dev_configure(struct rte_eth_dev *dev __rte_unused) { return 0; }

/**
 * Atomically reads the link status information from global
 * structure rte_eth_dev.
 *
 * @param dev
 *   - Pointer to the structure rte_eth_dev to read from.
 *   - Pointer to the buffer to be saved with the link status.
 *
 * @return
 *   - On success, zero.
 *   - On failure, negative value.
 */
static inline int
eth_link_atomic_read_status(struct rte_eth_dev *dev,
				struct rte_eth_link *link)
{
	struct rte_eth_link *dst = link;
	struct rte_eth_link *src = &(dev->data->dev_link);

	if (rte_atomic64_cmpset((uint64_t *)dst, *(uint64_t *)dst,
					*(uint64_t *)src) == 0)
		return -1;

	return 0;
}

/**
 * Atomically writes the link status information into global
 * structure rte_eth_dev.
 *
 * @param dev
 *   - Pointer to the structure rte_eth_dev to read from.
 *   - Pointer to the buffer to be saved with the link status.
 *
 * @return
 *   - On success, zero.
 *   - On failure, negative value.
 */
static inline int
eth_link_atomic_write_status(struct rte_eth_dev *dev,
				struct rte_eth_link *link)
{
	struct rte_eth_link *dst = &(dev->data->dev_link);
	struct rte_eth_link *src = link;

	if (rte_atomic64_cmpset((uint64_t *)dst, *(uint64_t *)dst,
					*(uint64_t *)src) == 0)
		return -1;

	return 0;
}

static void eth_dev_stop(struct rte_eth_dev *dev);
static int
eth_link_update(struct rte_eth_dev *dev,
		int wait_to_complete __rte_unused)
{
	struct virtq_dev *vdev;
	struct pmd_internals *internals = dev->data->dev_private;
	struct rte_eth_link link, old;

	if (internals->nb_rx_queues < 1)
		return -1;

	/* every element of rx_pmd_queues[] has a same vdev value. */
	vdev = internals->rx_pmd_queues[0].vdev;
	if (vdev == NULL)
		return -1;

	memset(&link, 0, sizeof(link));
	eth_link_atomic_read_status(dev, &link);
	old = link;

	if ((rte_atomic32_read(&vdev->status) == DEV_OPEN) &&
		(rte_atomic32_read(&vdev->vq_status[VTNET_RXQ]) != VQ_STOP) &&
		(rte_atomic32_read(&vdev->vq_status[VTNET_TXQ]) != VQ_STOP)) {
		if (link.link_status == 0) { /* link_status is 'down' */
			/* change link_status */
			link.link_status = 1;
			eth_link_atomic_write_status(dev, &link);
		}
	} else {
		if (link.link_status == 1) { /* link_status is 'up' */
			/* change link_status */
			link.link_status = 0;
			eth_link_atomic_write_status(dev, &link);
		}
	}

	/* every element of rx_pmd_queues[] has a same vdev value. */
	/* not changed */
	if (old.link_status == link.link_status)
		return -1;

	/* changed */
	return 0;
}

static inline int
eth_dev_cmpset_status(struct virtq_dev *vdev, enum virtq_dev_status exp,
				enum virtq_dev_status src)
{
	if ((rte_atomic32_cmpset((uint32_t *)&vdev->status,
				(uint32_t)exp,
				(uint32_t)src)) == 0)
		return -1;

	return 0;
}

static int
eth_dev_start(struct rte_eth_dev *dev)
{
	struct virtq_dev *vdev;
	struct pmd_internals *internals = dev->data->dev_private;

	if (internals->nb_rx_queues < 1)
		return -1;

	/* every element of rx_pmd_queues[] has a same vdev value. */
	vdev = internals->rx_pmd_queues[0].vdev;
	if (vdev == NULL)
		return -1;

	dbg("'virtq%d' device is started\n", vdev->id);
	/* change device status */
	if (eth_dev_cmpset_status(vdev, DEV_STOP, DEV_START))
		return -1;

	return 0;
}

static void
eth_dev_stop(struct rte_eth_dev *dev)
{
	struct virtq_dev *vdev;
	struct pmd_internals *internals = dev->data->dev_private;

	if (internals->nb_rx_queues < 1)
		return;

	/* every element of rx_pmd_queues[] has a same vdev value. */
	vdev = internals->rx_pmd_queues[0].vdev;
	if (vdev == NULL)
		return;

	dbg("'virtq%d' device is stopped\n", vdev->id);
	/* stop event handler */
	if (rte_atomic32_read(&vdev->status) == DEV_OPEN) {
		shutdown(ipc_get_fd(vdev->vs->ipc), SHUT_RDWR);
		while (rte_atomic32_read(&vdev->status) != DEV_START)
			rte_delay_ms(10);
	}

	/* change device status */
	if (eth_dev_cmpset_status(vdev, DEV_START, DEV_STOP))
		return;

	/* close socket */
	if ((vdev->vs) && (vdev->vs->ipc))
		close(ipc_get_fd(vdev->vs->ipc));

	/* link status update */
	eth_link_update(vdev->edev, 0);
}

static int
eth_rx_queue_setup(struct rte_eth_dev *dev, uint16_t rx_queue_id,
			uint16_t nb_rx_desc __rte_unused,
			unsigned int socket_id __rte_unused,
			const struct rte_eth_rxconf *rx_conf __rte_unused,
			struct rte_mempool *mb_pool)
{
	struct pmd_internals *internals = dev->data->dev_private;
	struct pmd_queue *rx_queue = &internals->rx_pmd_queues[rx_queue_id];

	rx_queue->mb_pool = mb_pool;
	dev->data->rx_queues[rx_queue_id] = rx_queue;
	rx_queue->in_port = dev->data->port_id;
	return 0;
}

static int
eth_tx_queue_setup(struct rte_eth_dev *dev, uint16_t tx_queue_id,
			uint16_t nb_tx_desc __rte_unused,
			unsigned int socket_id __rte_unused,
			const struct rte_eth_txconf *tx_conf __rte_unused)
{
	struct pmd_internals *internals = dev->data->dev_private;
	struct pmd_queue *tx_queue = &internals->tx_pmd_queues[tx_queue_id];

	dev->data->tx_queues[tx_queue_id] = tx_queue;
	return 0;
}


static void
eth_dev_info(struct rte_eth_dev *dev,
		struct rte_eth_dev_info *dev_info)
{
	struct pmd_internals *internals = dev->data->dev_private;

	dev_info->driver_name = drivername;
	dev_info->max_mac_addrs = 1; /* TBW */
	dev_info->max_rx_pktlen = (uint32_t)-1;
	dev_info->max_rx_queues = (uint16_t)internals->nb_rx_queues;
	dev_info->max_tx_queues = (uint16_t)internals->nb_tx_queues;
	dev_info->min_rx_bufsize = 0;
	dev_info->pci_dev = NULL;
}

static void
eth_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *igb_stats)
{
	unsigned i;
	unsigned long rx_total_pkts = 0, tx_total_pkts = 0;
	unsigned long rx_total_errs = 0, tx_total_errs = 0;
	unsigned long rx_total_bytes = 0, tx_total_bytes = 0;
	const struct pmd_internals *internal = dev->data->dev_private;

	memset(igb_stats, 0, sizeof(*igb_stats));
	for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS &&
			i < internal->nb_rx_queues; i++) {
		igb_stats->q_ipackets[i] = internal->rx_pmd_queues[i].rx_pkts;
		igb_stats->q_errors[i] = internal->rx_pmd_queues[i].rx_errs;
		igb_stats->q_ibytes[i] = internal->rx_pmd_queues[i].rx_bytes;
		rx_total_pkts += igb_stats->q_ipackets[i];
		rx_total_errs += internal->rx_pmd_queues[i].rx_errs;
		rx_total_bytes += igb_stats->q_ibytes[i];
	}

	for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS &&
			i < internal->nb_tx_queues; i++) {
		igb_stats->q_opackets[i] = internal->tx_pmd_queues[i].tx_pkts;
		igb_stats->q_errors[i] = internal->tx_pmd_queues[i].tx_errs;
		igb_stats->q_obytes[i] = internal->tx_pmd_queues[i].tx_bytes;
		tx_total_pkts += igb_stats->q_opackets[i];
		tx_total_errs += internal->rx_pmd_queues[i].tx_errs;
		tx_total_bytes += igb_stats->q_obytes[i];
	}

	igb_stats->ipackets = rx_total_pkts;
	igb_stats->opackets = tx_total_pkts;
	igb_stats->ierrors = rx_total_errs;
	igb_stats->oerrors = tx_total_errs;
	igb_stats->ibytes = rx_total_bytes;
	igb_stats->obytes = tx_total_bytes;
}

static void
eth_stats_reset(struct rte_eth_dev *dev)
{
	unsigned i;
	struct pmd_internals *internal = dev->data->dev_private;

	for (i = 0; i < internal->nb_rx_queues; i++) {
		internal->rx_pmd_queues[i].rx_pkts = 0;
		internal->rx_pmd_queues[i].rx_errs = 0;
		internal->rx_pmd_queues[i].rx_bytes = 0;
	}
	for (i = 0; i < internal->nb_tx_queues; i++) {
		internal->tx_pmd_queues[i].tx_pkts = 0;
		internal->tx_pmd_queues[i].tx_errs = 0;
		internal->tx_pmd_queues[i].tx_bytes = 0;
	}
}

static void
eth_queue_release(void *q __rte_unused) { ; }

static struct eth_dev_ops ops = {
		.dev_start = eth_dev_start,
		.dev_stop = eth_dev_stop,
		.dev_configure = eth_dev_configure,
		.dev_infos_get = eth_dev_info,
		.rx_queue_setup = eth_rx_queue_setup,
		.tx_queue_setup = eth_tx_queue_setup,
		.rx_queue_release = eth_queue_release,
		.tx_queue_release = eth_queue_release,
		.link_update = eth_link_update,
		.stats_get = eth_stats_get,
		.stats_reset = eth_stats_reset,
};

static void
eth_virtq_close_event_handler(struct virtq_dev *vdev)
{
	int fd;

	if (vdev == NULL)
		return;

	(*vdev->vs->vs_vc->vc_reset)((void *)vdev->vs);
	vm_destroy(vdev->ctx);
	pci_vtnet_close(vdev->vs);
	fd = ipc_get_fd(vdev->vs->ipc);
	close(fd);
}

static struct virtq_dev *
eth_virtq_init_event_handler(int fd)
{
	struct virtq_dev *vdev;
	struct virtio_softc *vs;
	ipc_node qemu;
	uint32_t nid, lowmem_limit;
	uint64_t ram_size;
	char name[] = "virtq_client";
	int ram_fd;

	/* allocate client */
	if (ipc_node_init(&qemu,
				0, /* nid */
				fd,
				0, /* ram_fd */
				0 /* ram_size*/))
		goto err1;

	if (ipc_server_init_sequence(&qemu, &nid, &ram_fd, &ram_size,
				&lowmem_limit) < 0)
		goto err1;

	if ((nid >= RTE_MAX_ETHPORTS) || (nid > virtq_dev_num))
		goto err1;

	/* set vdev */
	vdev = virtq_dev[nid];

	/* change device status */
	if (eth_dev_cmpset_status(vdev, DEV_START, DEV_OPEN))
		goto err1;

	/* fill vdev structure */
	vdev->qemu = qemu;
	vdev->ram_fd = ram_fd;
	vdev->ram_size = ram_size;
	vdev->lowmem_limit = lowmem_limit;
	vdev->th = pthread_self();

	/* init pci device */
	if (pci_vtnet_init(&vdev->pdi, (void **)&vdev->vs))
		goto err2;
	vs = vdev->vs;
	vs->ipc = &vdev->qemu;

	/* open vm memory */
	vdev->ctx = vm_open(name, ram_fd, lowmem_limit);
	if (vdev->ctx == NULL)
		goto err3;

	if (vm_sneak_memory(vdev->ctx, VM_MMAP_ALL))
		goto err4;
	vi_set_physical_mem(vs, vdev->ctx);

	return vdev;

err4:
	vm_destroy(vdev->ctx);
err3:
	pci_vtnet_close(vdev->vs);
err2:
	eth_dev_cmpset_status(vdev, DEV_OPEN, DEV_START);
err1:
	close(fd);

	return NULL;
}

static inline void
wait_vq_access_finish(rte_atomic32_t *status, uint8_t *resetting)
{
	*resetting = 1;
	do {
		rte_atomic32_set(status, VQ_STOP);
		rte_delay_ms(100);
	} while (rte_atomic32_read(status) != VQ_STOP);
	*resetting = 0;
}

static void *
eth_virtq_event_handler(void *data)
{
	int cfd = (int)(int64_t)data;
	struct virtq_dev *vdev;

	/* initialize the event handler */
	vdev = eth_virtq_init_event_handler(cfd);
	if (vdev == NULL) {
		/* eth_virtq_close_event_handler_XXXX will be called
		 * device status should be DEV_START */
		pthread_exit(NULL);
	}

	/* set cpu affinity */
	virtq_thread_set_affinity(vdev->lcore_id);

	/* Now device status is DEV_OPEN, but virtqueues are still stopped */
	rte_atomic32_set(&vdev->vq_status[VTNET_RXQ], VQ_STOP);
	rte_atomic32_set(&vdev->vq_status[VTNET_RXQ], VQ_STOP);

	/* event loop */
	for (;;) {
		ipc_node *qemu = &vdev->qemu;
		uint16_t curq;
		ipc_msg_type mtype;
		uint32_t pfn, negotiated_caps;

		if (ipc_server_recv(qemu, &mtype, &curq, &negotiated_caps,
					&pfn) < 0)
			goto do_close;

		switch (mtype) {
		case IPC_MESSAGE_TYPE_KICK:
			dbg("KICK\n");
			/* set status */
			rte_atomic32_set(&vdev->vq_status[VTNET_TXQ], VQ_KICK);
			rte_atomic32_set(&vdev->vq_status[VTNET_RXQ], VQ_KICK);
			break;
		case IPC_MESSAGE_TYPE_GUEST_FEATURES:
			dbg("FEATURE\n");
			vdev->vs->vs_negotiated_caps = negotiated_caps;
			/* size of struct virtio_net_rxhdr isn't constant */
			if (negotiated_caps & VIRTIO_NET_F_MRG_RXBUF) {
				vdev->rxhdr_len =
					sizeof(struct virtio_net_rxhdr);
				vdev->vrh_buf_val = 1;
			} else {
				vdev->rxhdr_len =
					sizeof(struct virtio_net_rxhdr) - 2;
				vdev->vrh_buf_val = 0;
			}
			break;
		case IPC_MESSAGE_TYPE_PFN:
			dbg("PFN\n");
			/* set pfn */
			vdev->vs->vs_curq = curq;
			vi_vq_init(vdev->vs, pfn);
			break;
		case IPC_MESSAGE_TYPE_RECONFIG:
			dbg("Reconfigure queue%u pfn is 0x%x\n",
				curq, pfn);
			/* reconfigure a queue */
			vdev->vs->vs_curq = curq;
			vi_vq_reconf(vdev->vs, pfn);
			break;
		case IPC_MESSAGE_TYPE_RESET:
			dbg("receive reset message\n");
			goto do_close;
		default:
			fprintf(stderr, "invalid ipc message type\n");
			/* set status */
			goto do_close;
		}
	}

do_close:
	/* virtqueues might be accessed by tx/rx threads,
	 * so wait until virtqueue accesses are finished */
	wait_vq_access_finish(&vdev->vq_status[VTNET_TXQ], &vdev->resetting);
	wait_vq_access_finish(&vdev->vq_status[VTNET_RXQ], &vdev->resetting);

	/* change device status */
	rte_atomic32_set(&vdev->status, DEV_START);

	/* eth_virtq_close_event_handler_XXXX will be called */
	eth_virtq_close_event_handler(vdev);

	pthread_exit(NULL);
}

static void *
eth_virtq_accepter(void *data __rte_unused)
{
	pthread_t th;
	pthread_attr_t attr;
	int cfd;

	/* init pthread attribute */
	if (pthread_attr_init(&attr))
		return NULL;

	/* set detach state */
	if (pthread_attr_setdetachstate(&attr,
				PTHREAD_CREATE_DETACHED))
		return NULL;

	for (;;) {
		/* accept */
		cfd = ipc_accept(virtq_accepter.fd);
		if (cfd < 0)
			break;

		/* create thread to handle events */
		if (pthread_create(&th, &attr, eth_virtq_event_handler,
					(void *)(int64_t)cfd))
			break;
	}

	return NULL;
}

/* This function isn't MT safe */
static int
eth_virtq_create_accepter(void)
{
	/* sanity check */
	if (virtq_accepter.name != NULL)
		return -1;

	virtq_accepter.name = (const char *)getenv("RTE_PMD_VIRTQ_CONNECTOR");
	if (virtq_accepter.name == NULL)
		return -1;

	/* open a socket to accept connections from QEMU */
	virtq_accepter.fd = ipc_socket();
	if (virtq_accepter.fd < 0)
		return -1;

	unlink(virtq_accepter.name);

	/* bind */
	if (ipc_bind(virtq_accepter.fd, virtq_accepter.name,
				strlen(virtq_accepter.name)))
		return -1;

	/* listen */
	if (ipc_listen(virtq_accepter.fd))
		return -1;

	/* init pthread attribute */
	if (pthread_attr_init(&virtq_accepter.attr))
		return -1;

	/* set detach state */
	if (pthread_attr_setdetachstate(&virtq_accepter.attr,
				PTHREAD_CREATE_DETACHED))
		return -1;

	/* create thread to receive data from the client */
	if (pthread_create(&virtq_accepter.th,
			&virtq_accepter.attr, eth_virtq_accepter, NULL))
		return -1;

	return 0;
}

static int
eth_from_virtqs(struct virtq_dev *vdev,
		const unsigned nb_rx_queues,
		const unsigned nb_tx_queues,
		const unsigned numa_node)
{
	struct rte_eth_dev_data *data = NULL;
	struct rte_pci_device *pci_dev = NULL;
	struct pmd_internals *internals = NULL;
	struct rte_eth_dev *eth_dev = NULL;
	unsigned i;

	/* do some paramter checking */
	if (vdev == NULL && nb_rx_queues > 0)
		goto error;
	if (vdev == NULL && nb_tx_queues > 0)
		goto error;

	RTE_LOG(INFO, PMD,
		"Creating virtqueue-backed ethdev on numa socket %u\n",
		numa_node);

	/* now do all data allocation - for eth_dev structure, dummy pci driver
	 * and internal (private) data
	 */
	data = rte_zmalloc_socket(NULL, sizeof(*data), 0, numa_node);
	if (data == NULL)
		goto error;

	pci_dev = rte_zmalloc_socket(NULL, sizeof(*pci_dev), 0, numa_node);
	if (pci_dev == NULL)
		goto error;

	internals = rte_zmalloc_socket(NULL, sizeof(*internals), 0, numa_node);
	if (internals == NULL)
		goto error;

	/* reserve an ethdev entry */
	eth_dev = rte_eth_dev_allocate();
	if (eth_dev == NULL)
		goto error;

	/* now put it all together
	 * - store queue data in internals,
	 * - store numa_node info in pci_driver
	 * - point eth_dev_data to internals and pci_driver
	 * - and point eth_dev structure to new eth_dev_data structure
	 */
	/* NOTE: we'll replace the data element, of originally allocated eth_dev
	 * so the virtqs are local per-process */

	internals->nb_rx_queues = nb_rx_queues;
	internals->nb_tx_queues = nb_tx_queues;
	for (i = 0; i < nb_rx_queues; i++)
		internals->rx_pmd_queues[i].vdev = vdev;

	for (i = 0; i < nb_tx_queues; i++)
		internals->tx_pmd_queues[i].vdev = vdev;

	pci_dev->numa_node = numa_node;

	data->dev_private = internals;
	data->port_id = eth_dev->data->port_id;
	data->nb_rx_queues = (uint16_t)nb_rx_queues;
	data->nb_tx_queues = (uint16_t)nb_tx_queues;
	data->dev_link = pmd_link;
	data->mac_addrs = vdev->eth_addr;

	eth_dev->data = data;
	eth_dev->dev_ops = &ops;
	eth_dev->pci_dev = pci_dev;

	/* finally assign rx and tx ops */
	eth_dev->rx_pkt_burst = virtq_pmd_virtq_rx;
	eth_dev->tx_pkt_burst = virtq_pmd_virtq_tx;

	vdev->edev = eth_dev;

	return 0;

error:
	if (data)
		rte_free(data);
	if (pci_dev)
		rte_free(pci_dev);
	if (internals)
		rte_free(internals);
	return -1;
}

/* The function is based on my_eth_aton of lirte_cmdline */
static inline int
get_eth_addr(char *value, void *extra_args)
{
	int i;
	const char *a = value;
	char *end;
	unsigned long o[ETHER_ADDR_LEN];
	struct ether_addr *ether_addr = extra_args;

	i = 0;
	do {
		errno = 0;
		o[i] = strtoul(a, &end, 16);
		if (errno != 0 || end == a || (end[0] != ':' && end[0] != 0))
			return -1;
		a = end + 1;
	} while (++i != sizeof(o) / sizeof(o[0]) && end[0] != 0);

	/* Junk at the end of line */
	if (end[0] != 0)
		return -1;

	/* Support the format XX:XX:XX:XX:XX:XX */
	if (i == ETHER_ADDR_LEN) {
		while (i-- != 0) {
			if (o[i] > UINT8_MAX)
				return -1;
			ether_addr->addr_bytes[i] = (uint8_t)o[i];
		}
	/* Support the format XXXX:XXXX:XXXX */
	} else if (i == ETHER_ADDR_LEN / 2) {
		while (i-- != 0) {
			if (o[i] > UINT16_MAX)
				return -1;
			ether_addr->addr_bytes[i * 2] =
				(uint8_t)(o[i] >> 8);
			ether_addr->addr_bytes[i * 2 + 1] =
				(uint8_t)(o[i] & 0xff);
		}
	/* unknown format */
	} else
		return -1;

	return 0;
}

static inline int
get_cpu_flag(char *value, void *extra_args)
{
	const char *a = value;
	unsigned *lcore_id = extra_args;

	*lcore_id = (unsigned)strtoul(a, NULL, 0);
	if (*lcore_id == UINT_MAX)
		return -1;

	return 0;
}

static inline int
get_device_id(const char *name)
{
	const char *p = &name[strlen(name) - 1];

	return (int)rte_str_to_size(p);
}

static int
eth_dev_virtq_create(const char *name, struct ether_addr *eth_addr,
		unsigned lcore_id, const unsigned numa_node)
{
	struct virtq_dev *tmp;

	/* create a thread to accept connections from QEMU */
	if (strncmp(name, "eth_virtq0", strlen("eth_virtq0")) == 0) {
		if (eth_virtq_create_accepter())
			return -1;
	} else { /* eth_virtqX */
		/* eth_virtq0 should be initialized before */
		if (virtq_dev[0] == NULL)
			return -1;
	}

	/* Create virtqueue device */
	tmp = rte_zmalloc_socket(NULL, sizeof(*tmp), 0, numa_node);
	if (tmp == NULL)
		return -1;

	/* fill vdev */
	tmp->id = get_device_id(name);
	tmp->name = name;
	tmp->eth_addr = eth_addr;
	tmp->lcore_id = lcore_id;
	rte_atomic32_init(&tmp->status);
	rte_atomic32_set(&tmp->status, DEV_STOP);

	/* create ether device */
	if (eth_from_virtqs(tmp, 1, 1, numa_node))
		return -1;

	virtq_dev[tmp->id] = tmp;

	virtq_dev_num = RTE_MAX(virtq_dev_num, tmp->id);

	return 0;
}

int
rte_pmd_virtq_init(const char *name, const char *params)
{
	int ret;
	struct args_dict dict;
	struct ether_addr *eth_addr;
	unsigned lcore_id = 0;

	eth_addr = rte_zmalloc(NULL, sizeof(struct ether_addr), 0);
	if (eth_addr == NULL)
		return -1;

	rte_eth_pcap_init_args_dict(&dict);

	rte_eth_pcap_parse_args(&dict, name,
				params, virtq_valid_arguments);

	ret = rte_eth_pcap_post_process_arguments(&dict, ETH_VIRTQ_MAC_ADDR_ARG,
				&get_eth_addr, eth_addr);
	if (ret < 0)
		return -1;

	ret = rte_eth_pcap_post_process_arguments(&dict, ETH_VIRTQ_LCORE_ID_ARG,
				&get_cpu_flag, &lcore_id);
	if (ret < 0)
		return -1;

	if (eth_dev_virtq_create(name, eth_addr, lcore_id, rte_socket_id()))
		return -1;

	return 0;
}
