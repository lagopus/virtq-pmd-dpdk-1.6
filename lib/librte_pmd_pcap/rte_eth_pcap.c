/*-
 *   BSD LICENSE
 * 
 *   Copyright(c) 2010-2014 Intel Corporation. All rights reserved.
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
 *     * Neither the name of Intel Corporation nor the names of its
 *       contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
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

#include <time.h>
#include <rte_mbuf.h>
#include <rte_ethdev.h>
#include <rte_malloc.h>
#include <rte_memcpy.h>
#include <rte_string_fns.h>
#include <rte_cycles.h>

#include "rte_eth_pcap.h"
#include "rte_eth_pcap_arg_parser.h"

#define RTE_ETH_PCAP_SNAPSHOT_LEN 65535
#define RTE_ETH_PCAP_SNAPLEN 4096
#define RTE_ETH_PCAP_PROMISC 1
#define RTE_ETH_PCAP_TIMEOUT -1
#define RTE_ETH_PCAP_MBUFS 64
#define ETH_PCAP_RX_PCAP_ARG  	"rx_pcap"
#define ETH_PCAP_TX_PCAP_ARG  	"tx_pcap"
#define ETH_PCAP_RX_IFACE_ARG 	"rx_iface"
#define ETH_PCAP_TX_IFACE_ARG	"tx_iface"
#define ETH_PCAP_IFACE_ARG		"iface"

static char errbuf[PCAP_ERRBUF_SIZE];
static struct timeval start_time;
static uint64_t start_cycles;
static uint64_t hz;

struct pcap_rx_queue {
	pcap_t *pcap;
	struct rte_mempool *mb_pool;
	volatile unsigned long rx_pkts;
	volatile unsigned long err_pkts;
};

struct pcap_tx_queue {
	pcap_dumper_t *dumper;
	pcap_t *pcap;
	volatile unsigned long tx_pkts;
	volatile unsigned long err_pkts;
};

struct rx_pcaps {
	unsigned num_of_rx;
	pcap_t *pcaps[RTE_PMD_RING_MAX_RX_RINGS];
};

struct tx_pcaps {
	unsigned num_of_tx;
	pcap_dumper_t *dumpers[RTE_PMD_RING_MAX_TX_RINGS];
	pcap_t *pcaps[RTE_PMD_RING_MAX_RX_RINGS];
};

struct pmd_internals {
	unsigned nb_rx_queues;
	unsigned nb_tx_queues;

	struct pcap_rx_queue rx_queue[RTE_PMD_RING_MAX_RX_RINGS];
	struct pcap_tx_queue tx_queue[RTE_PMD_RING_MAX_TX_RINGS];
};

const char *valid_arguments[] = {
	ETH_PCAP_RX_PCAP_ARG,
	ETH_PCAP_TX_PCAP_ARG,
	ETH_PCAP_RX_IFACE_ARG,
	ETH_PCAP_TX_IFACE_ARG,
	ETH_PCAP_IFACE_ARG,
	NULL
};

static struct ether_addr eth_addr = { .addr_bytes = { 0, 0, 0, 0x1, 0x2, 0x3 } };
static const char *drivername = "Pcap PMD";
static struct rte_eth_link pmd_link = {
		.link_speed = 10000,
		.link_duplex = ETH_LINK_FULL_DUPLEX,
		.link_status = 0
};


static uint16_t
eth_pcap_rx(void *queue,
		struct rte_mbuf **bufs,
		uint16_t nb_pkts)
{
	unsigned i;
	struct pcap_pkthdr header;
	const u_char *packet;
	struct rte_mbuf *mbuf;
	struct pcap_rx_queue *pcap_q = queue;
	struct rte_pktmbuf_pool_private *mbp_priv;
	uint16_t num_rx = 0;
	uint16_t buf_size;

	if (unlikely(pcap_q->pcap == NULL || nb_pkts == 0))
		return 0;

	/* Reads the given number of packets from the pcap file one by one
	 * and copies the packet data into a newly allocated mbuf to return.
	 */
	for (i = 0; i < nb_pkts; i++) {
		/* Get the next PCAP packet */
		packet = pcap_next(pcap_q->pcap, &header);
		if (unlikely(packet == NULL))
			break;
		else 
			mbuf = rte_pktmbuf_alloc(pcap_q->mb_pool);
		if (unlikely(mbuf == NULL))
			break;

		/* Now get the space available for data in the mbuf */
		mbp_priv =  rte_mempool_get_priv(pcap_q->mb_pool);
		buf_size = (uint16_t) (mbp_priv->mbuf_data_room_size -
				RTE_PKTMBUF_HEADROOM);

		if (header.len <= buf_size) {
			/* pcap packet will fit in the mbuf, go ahead and copy */
			rte_memcpy(mbuf->pkt.data, packet, header.len);
			mbuf->pkt.data_len = (uint16_t)header.len;
			mbuf->pkt.pkt_len = mbuf->pkt.data_len;
			bufs[i] = mbuf;
			num_rx++;
		} else {
			/* pcap packet will not fit in the mbuf, so drop packet */
			RTE_LOG(ERR, PMD, 
					"PCAP packet %d bytes will not fit in mbuf (%d bytes)\n",
					header.len, buf_size);
			rte_pktmbuf_free(mbuf);
		}
	}
	pcap_q->rx_pkts += num_rx;
	return num_rx;
}

static inline void
calculate_timestamp(struct timeval *ts) {
	uint64_t cycles;
	struct timeval cur_time;

	cycles = rte_get_timer_cycles() - start_cycles;
	cur_time.tv_sec = cycles / hz;
	cur_time.tv_usec = (cycles % hz) * 10e6 / hz;
	timeradd(&start_time, &cur_time, ts);
}

/*
 * Callback to handle writing packets to a pcap file.
 */
static uint16_t
eth_pcap_tx_dumper(void *queue,
		struct rte_mbuf **bufs,
		uint16_t nb_pkts)
{
	unsigned i;
	struct rte_mbuf *mbuf;
	struct pcap_tx_queue *dumper_q = queue;
	uint16_t num_tx = 0;
	struct pcap_pkthdr header;

	if (dumper_q->dumper == NULL || nb_pkts == 0)
		return 0;

	/* writes the nb_pkts packets to the previously opened pcap file dumper */
	for (i = 0; i < nb_pkts; i++) {
		mbuf = bufs[i];
		calculate_timestamp(&header.ts);
		header.len = mbuf->pkt.data_len;
		header.caplen = header.len;
		pcap_dump((u_char*) dumper_q->dumper, &header, mbuf->pkt.data);
		rte_pktmbuf_free(mbuf);
		num_tx++;
	}

	/*
	 * Since there's no place to hook a callback when the forwarding
	 * process stops and to make sure the pcap file is actually written,
	 * we flush the pcap dumper within each burst.
	 */
	pcap_dump_flush(dumper_q->dumper);
	dumper_q->tx_pkts += num_tx;
	dumper_q->err_pkts += nb_pkts - num_tx;
	return num_tx;
}

/*
 * Callback to handle sending packets through a real NIC.
 */
static uint16_t
eth_pcap_tx(void *queue,
		struct rte_mbuf **bufs,
		uint16_t nb_pkts)
{
	unsigned i;
	int ret;
	struct rte_mbuf *mbuf;
	struct pcap_tx_queue *tx_queue = queue;
	uint16_t num_tx = 0;

	if (unlikely(nb_pkts == 0 || tx_queue->pcap == NULL))
		return 0;

	for (i = 0; i < nb_pkts; i++) {
		mbuf = bufs[i];
		ret = pcap_sendpacket(tx_queue->pcap, (u_char*) mbuf->pkt.data,
				mbuf->pkt.data_len);
		if(likely(!ret))
			num_tx++;
		rte_pktmbuf_free(mbuf);
	}

	tx_queue->tx_pkts += num_tx;
	tx_queue->err_pkts += nb_pkts - num_tx;
	return num_tx;
}

static int
eth_dev_start(struct rte_eth_dev *dev)
{
	dev->data->dev_link.link_status = 1;
	return 0;
}

/*
 * This function gets called when the current port gets stopped.
 * Is the only place for us to close all the tx streams dumpers.
 * If not called the dumpers will be flushed within each tx burst.
 */
static void
eth_dev_stop(struct rte_eth_dev *dev)
{
	unsigned i;
	pcap_dumper_t *dumper;
	pcap_t *pcap;
	struct pmd_internals *internals = dev->data->dev_private;

	for (i = 0; i < internals->nb_tx_queues; i++) {
		dumper = internals->tx_queue[i].dumper;
		if(dumper != NULL)
			pcap_dump_close(dumper);
		pcap = internals->tx_queue[i].pcap;
		if(pcap != NULL)
			pcap_close(pcap);
	}

	dev->data->dev_link.link_status = 0;
}

static int
eth_dev_configure(struct rte_eth_dev *dev __rte_unused)
{
	return 0;
}

static void
eth_dev_info(struct rte_eth_dev *dev,
		struct rte_eth_dev_info *dev_info)
{
	struct pmd_internals *internals = dev->data->dev_private;
	dev_info->driver_name = drivername;
	dev_info->max_mac_addrs = 1;
	dev_info->max_rx_pktlen = (uint32_t) -1;
	dev_info->max_rx_queues = (uint16_t)internals->nb_rx_queues;
	dev_info->max_tx_queues = (uint16_t)internals->nb_tx_queues;
	dev_info->min_rx_bufsize = 0;
	dev_info->pci_dev = NULL;
}

static void
eth_stats_get(struct rte_eth_dev *dev,
		struct rte_eth_stats *igb_stats)
{
	unsigned i;
	unsigned long rx_total = 0, tx_total = 0, tx_err_total = 0;
	const struct pmd_internals *internal = dev->data->dev_private;

	memset(igb_stats, 0, sizeof(*igb_stats));
	for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS && i < internal->nb_rx_queues;
			i++) {
		igb_stats->q_ipackets[i] = internal->rx_queue[i].rx_pkts;
		rx_total += igb_stats->q_ipackets[i];
	}

	for (i = 0; i < RTE_ETHDEV_QUEUE_STAT_CNTRS && i < internal->nb_tx_queues;
			i++) {
		igb_stats->q_opackets[i] = internal->tx_queue[i].tx_pkts;
		igb_stats->q_errors[i] = internal->tx_queue[i].err_pkts;
		tx_total += igb_stats->q_opackets[i];
		tx_err_total += igb_stats->q_errors[i];
	}

	igb_stats->ipackets = rx_total;
	igb_stats->opackets = tx_total;
	igb_stats->oerrors = tx_err_total;
}

static void
eth_stats_reset(struct rte_eth_dev *dev)
{
	unsigned i;
	struct pmd_internals *internal = dev->data->dev_private;
	for (i = 0; i < internal->nb_rx_queues; i++)
		internal->rx_queue[i].rx_pkts = 0;
	for (i = 0; i < internal->nb_tx_queues; i++) {
		internal->tx_queue[i].tx_pkts = 0;
		internal->tx_queue[i].err_pkts = 0;
	}
}

static void
eth_dev_close(struct rte_eth_dev *dev __rte_unused)
{
}

static void
eth_queue_release(void *q __rte_unused)
{
}

static int
eth_link_update(struct rte_eth_dev *dev __rte_unused,
		int wait_to_complete __rte_unused)
{
	return 0;
}

static int
eth_rx_queue_setup(struct rte_eth_dev *dev,
		uint16_t rx_queue_id,
		uint16_t nb_rx_desc __rte_unused,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_rxconf *rx_conf __rte_unused,
		struct rte_mempool *mb_pool)
{
	struct pmd_internals *internals = dev->data->dev_private;
	struct pcap_rx_queue *pcap_q = &internals->rx_queue[rx_queue_id];
	pcap_q->mb_pool = mb_pool;
	dev->data->rx_queues[rx_queue_id] = pcap_q;
	return 0;
}

static int
eth_tx_queue_setup(struct rte_eth_dev *dev,
		uint16_t tx_queue_id,
		uint16_t nb_tx_desc __rte_unused,
		unsigned int socket_id __rte_unused,
		const struct rte_eth_txconf *tx_conf __rte_unused)
{

	struct pmd_internals *internals = dev->data->dev_private;
	dev->data->tx_queues[tx_queue_id] = &internals->tx_queue[tx_queue_id];
	return 0;
}

static struct eth_dev_ops ops = {
		.dev_start = eth_dev_start,
		.dev_stop =	eth_dev_stop,
		.dev_close = eth_dev_close,
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

/*
 * Function handler that opens the pcap file for reading a stores a
 * reference of it for use it later on.
 */
static int
open_rx_pcap(char *value, void *extra_args)
{
	unsigned i;
	char *pcap_filename = value;
	struct rx_pcaps *pcaps = extra_args;
	pcap_t *rx_pcap;

	for (i = 0; i < pcaps->num_of_rx; i++) {
		if ((rx_pcap = pcap_open_offline(pcap_filename, errbuf)) == NULL) {
			RTE_LOG(ERR, PMD, "Couldn't open %s: %s\n", pcap_filename, errbuf);
			return -1;
		}
		pcaps->pcaps[i] = rx_pcap;
	}

	return 0;
}

/*
 * Opens a pcap file for writing and stores a reference to it
 * for use it later on.
 */
static int
open_tx_pcap(char *value, void *extra_args)
{
	unsigned i;
	char *pcap_filename = value;
	struct tx_pcaps *dumpers = extra_args;
	pcap_t *tx_pcap;
	pcap_dumper_t *dumper;

	for (i = 0; i < dumpers->num_of_tx; i++) {
		/*
		 * We need to create a dummy empty pcap_t to use it
		 * with pcap_dump_open(). We create big enough an Ethernet
		 * pcap holder.
		 */
		if ((tx_pcap = pcap_open_dead(DLT_EN10MB, RTE_ETH_PCAP_SNAPSHOT_LEN))
				== NULL) {
			RTE_LOG(ERR, PMD, "Couldn't create dead pcap\n");
			return -1;
		}

		/* The dumper is created using the previous pcap_t reference */
		if ((dumper = pcap_dump_open(tx_pcap, pcap_filename)) == NULL) {
			RTE_LOG(ERR, PMD, "Couldn't open %s for writing.\n", pcap_filename);
			return -1;
		}
		dumpers->dumpers[i] = dumper;
	}

	return 0;
}

/*
 * pcap_open_live wrapper function
 */
static inline int
open_iface_live(const char *iface, pcap_t **pcap) {
	*pcap = pcap_open_live(iface, RTE_ETH_PCAP_SNAPLEN,
			RTE_ETH_PCAP_PROMISC, RTE_ETH_PCAP_TIMEOUT, errbuf);

	if (*pcap == NULL) {
		RTE_LOG(ERR, PMD, "Couldn't open %s: %s\n", iface, errbuf);
		return -1;
	}
	return 0;
}

/*
 * Opens an interface for reading and writing
 */
static inline int
open_rx_tx_iface(char *value, void *extra_args)
{
	const char *iface = value;
	pcap_t **pcap = extra_args;

	if(open_iface_live(iface, pcap) < 0)
		return -1;
	return 0;
}

/*
 * Opens a NIC for reading packets from it
 */
static inline int
open_rx_iface(char *value, void *extra_args)
{
	unsigned i;
	const char *iface = value;
	struct rx_pcaps *pcaps = extra_args;
	pcap_t *pcap = NULL;

	for (i = 0; i < pcaps->num_of_rx; i++) {
		if(open_iface_live(iface, &pcap) < 0)
			return -1;
		pcaps->pcaps[i] = pcap;
	}

	return 0;
}

/*
 * Opens a NIC for writing packets to it
 */
static inline int
open_tx_iface(char *value, void *extra_args)
{
	unsigned i;
	const char *iface = value;
	struct tx_pcaps *pcaps = extra_args;
	pcap_t *pcap;

	for (i = 0; i < pcaps->num_of_tx; i++) {
		if(open_iface_live(iface, &pcap) < 0)
			return -1;
		pcaps->pcaps[i] = pcap;
	}

	return 0;
}


static int
rte_pmd_init_internals(const unsigned nb_rx_queues,
		const unsigned nb_tx_queues,
		const unsigned numa_node,
		struct pmd_internals **internals,
		struct rte_eth_dev **eth_dev)
{
	struct rte_eth_dev_data *data = NULL;
	struct rte_pci_device *pci_dev = NULL;

	RTE_LOG(INFO, PMD,
			"Creating pcap-backed ethdev on numa socket %u\n", numa_node);

	/* now do all data allocation - for eth_dev structure, dummy pci driver
	 * and internal (private) data
	 */
	data = rte_zmalloc_socket(NULL, sizeof(*data), 0, numa_node);
	if (data == NULL)
		goto error;

	pci_dev = rte_zmalloc_socket(NULL, sizeof(*pci_dev), 0, numa_node);
	if (pci_dev == NULL)
		goto error;

	*internals = rte_zmalloc_socket(NULL, sizeof(**internals), 0, numa_node);
	if (*internals == NULL)
		goto error;

	/* reserve an ethdev entry */
	*eth_dev = rte_eth_dev_allocate();
	if (*eth_dev == NULL)
		goto error;

	/* now put it all together
	 * - store queue data in internals,
	 * - store numa_node info in pci_driver
	 * - point eth_dev_data to internals and pci_driver
	 * - and point eth_dev structure to new eth_dev_data structure
	 */
	/* NOTE: we'll replace the data element, of originally allocated eth_dev
	 * so the rings are local per-process */

	(*internals)->nb_rx_queues = nb_rx_queues;
	(*internals)->nb_tx_queues = nb_tx_queues;

	pci_dev->numa_node = numa_node;

	data->dev_private = *internals;
	data->port_id = (*eth_dev)->data->port_id;
	data->nb_rx_queues = (uint16_t)nb_rx_queues;
	data->nb_tx_queues = (uint16_t)nb_tx_queues;
	data->dev_link = pmd_link;
	data->mac_addrs = &eth_addr;

	(*eth_dev)->data = data;
	(*eth_dev)->dev_ops = &ops;
	(*eth_dev)->pci_dev = pci_dev;

	return 0;

	error: if (data)
		rte_free(data);
	if (pci_dev)
		rte_free(pci_dev);
	if (*internals)
		rte_free(*internals);
	return -1;
}

int
rte_eth_from_pcaps_n_dumpers(pcap_t * const rx_queues[],
		const unsigned nb_rx_queues,
		pcap_dumper_t * const tx_queues[],
		const unsigned nb_tx_queues,
		const unsigned numa_node)
{
	struct pmd_internals *internals = NULL;
	struct rte_eth_dev *eth_dev = NULL;
	unsigned i;

	/* do some parameter checking */
	if (rx_queues == NULL && nb_rx_queues > 0)
		return -1;
	if (tx_queues == NULL && nb_tx_queues > 0)
		return -1;

	if (rte_pmd_init_internals(nb_rx_queues, nb_tx_queues, numa_node,
			&internals, &eth_dev) < 0)
		return -1;

	for (i = 0; i < nb_rx_queues; i++) {
		internals->rx_queue->pcap = rx_queues[i];
	}
	for (i = 0; i < nb_tx_queues; i++) {
		internals->tx_queue->dumper = tx_queues[i];
	}

	eth_dev->rx_pkt_burst = eth_pcap_rx;
	eth_dev->tx_pkt_burst = eth_pcap_tx_dumper;

	return 0;
}

int
rte_eth_from_pcaps(pcap_t * const rx_queues[],
		const unsigned nb_rx_queues,
		pcap_t * const tx_queues[],
		const unsigned nb_tx_queues,
		const unsigned numa_node)
{
	struct pmd_internals *internals = NULL;
	struct rte_eth_dev *eth_dev = NULL;
	unsigned i;

	/* do some parameter checking */
	if (rx_queues == NULL && nb_rx_queues > 0)
		return -1;
	if (tx_queues == NULL && nb_tx_queues > 0)
		return -1;

	if (rte_pmd_init_internals(nb_rx_queues, nb_tx_queues, numa_node,
			&internals, &eth_dev) < 0)
		return -1;

	for (i = 0; i < nb_rx_queues; i++) {
		internals->rx_queue->pcap = rx_queues[i];
	}
	for (i = 0; i < nb_tx_queues; i++) {
		internals->tx_queue->pcap = tx_queues[i];
	}

	eth_dev->rx_pkt_burst = eth_pcap_rx;
	eth_dev->tx_pkt_burst = eth_pcap_tx;

	return 0;
}


int
rte_pmd_pcap_init(const char *name, const char *params)
{
	unsigned numa_node, using_dumpers = 0;
	int ret;
	struct args_dict dict;
	struct rx_pcaps pcaps;
	struct tx_pcaps dumpers;

	rte_eth_pcap_init_args_dict(&dict);

	numa_node = rte_socket_id();

	gettimeofday(&start_time, NULL);
	start_cycles = rte_get_timer_cycles();
	hz = rte_get_timer_hz();

	if (rte_eth_pcap_parse_args(&dict, name, params, valid_arguments) < 0)
		return -1;

	/*
	 * If iface argument is passed we open the NICs and use them for
	 * reading / writing
	 */
	if (rte_eth_pcap_num_of_args(&dict, ETH_PCAP_IFACE_ARG) == 1) {

		ret = rte_eth_pcap_post_process_arguments(&dict, ETH_PCAP_IFACE_ARG,
				&open_rx_tx_iface, &pcaps.pcaps[0]);
		if (ret < 0)
			return -1;

		return rte_eth_from_pcaps(pcaps.pcaps, 1, pcaps.pcaps, 1, numa_node);
	}

	/*
	 * We check whether we want to open a RX stream from a real NIC or a
	 * pcap file
	 */
	if ((pcaps.num_of_rx = rte_eth_pcap_num_of_args(&dict, ETH_PCAP_RX_PCAP_ARG))) {
		ret = rte_eth_pcap_post_process_arguments(&dict, ETH_PCAP_RX_PCAP_ARG,
				&open_rx_pcap, &pcaps);
	} else {
		pcaps.num_of_rx = rte_eth_pcap_num_of_args(&dict,
				ETH_PCAP_RX_IFACE_ARG);
		ret = rte_eth_pcap_post_process_arguments(&dict, ETH_PCAP_RX_IFACE_ARG,
				&open_rx_iface, &pcaps);
	}

	if (ret < 0)
		return -1;

	/*
	 * We check whether we want to open a TX stream to a real NIC or a
	 * pcap file
	 */
	if ((dumpers.num_of_tx = rte_eth_pcap_num_of_args(&dict,
			ETH_PCAP_TX_PCAP_ARG))) {
		ret = rte_eth_pcap_post_process_arguments(&dict, ETH_PCAP_TX_PCAP_ARG,
				&open_tx_pcap, &dumpers);
		using_dumpers = 1;
	} else {
		dumpers.num_of_tx = rte_eth_pcap_num_of_args(&dict,
				ETH_PCAP_TX_IFACE_ARG);
		ret = rte_eth_pcap_post_process_arguments(&dict, ETH_PCAP_TX_IFACE_ARG,
				&open_tx_iface, &dumpers);
	}

	if (ret < 0)
		return -1;

	if (using_dumpers)
		return rte_eth_from_pcaps_n_dumpers(pcaps.pcaps, pcaps.num_of_rx,
				dumpers.dumpers, dumpers.num_of_tx, numa_node);

	return rte_eth_from_pcaps(pcaps.pcaps, pcaps.num_of_rx, dumpers.pcaps,
			dumpers.num_of_tx, numa_node);

}

