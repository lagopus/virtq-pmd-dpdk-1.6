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

#include "test.h"

#ifdef RTE_LIBRTE_PMD_VIRTQ

#include <stdio.h>
#include <unistd.h>

#include <rte_ethdev.h>
#include <rte_eth_virtq.h>

/*
 * virtq PMD
 * ======
 *
 * There are 2 test groups.
 * Basic tests
 * - Check status of port created by virtq PMD
 * Advanced tests:
 * - Check connectivity between host and guest using qemu-kvm
 *
 * Basic tests are default. Advanced tests are optional.
 * To performe advanced tests, qemu-kvm and ubuntu image(12.04 or higher) are
 * required. See also following steps to setup ubuntu guest.
 * 1. Prepare fixed qemu-kvm, because the virtq PMD works with fixed qemu-kvm
 * 2. Prepare ubuntu guest image.
 * 3. Boot image from qemu-kvm.
 * 4. Copy a DPDK source(1.5 or higher) to the guest and compile it.
 * 5. Start guest image with following network paramerters.
 * 	-device virtio-net-ipc-pci,id=net0,nid=0,socketpath=/tmp/virtq0
 * 	-device virtio-net-ipc-pci,id=net1,nid=1,socketpath=/tmp/virtq0
 * 	-net none
 * 6. Start testpmd application on guest using above 2 ports
 * 7. Uncomment "#define ADVANCED_TESTS" of this file.
 */

/* #define ADVANCED_TESTS */

#define SOCKET		(rte_socket_id())

static struct rte_mempool *mp;
/* will store the port id of the first of our new ports */
static uint8_t start_idx;

#define TX_PORT		(uint8_t)(start_idx + 0)
#define RX_PORT		(uint8_t)(start_idx + 1)

#define MBUF_SIZE	(2048 + sizeof(struct rte_mbuf) + RTE_PKTMBUF_HEADROOM)
#define NB_MBUF		(512)

static int
test_virtq_ethdev_create(void)
{
	int retval;

	printf("Testing virtq pmd create\n");

	retval = rte_pmd_virtq_init("eth_virtq9",
			"mac=00:00:00:00:00:01;lcore_id=0");
	if (retval >= 0) {
		printf("Failure, can create pmd with illegal name\n");
		return -1;
	}

	retval = rte_pmd_virtq_init("eth_virtq0",
			"mac=00:01:02:03:04:05:06;lcore_id=0");
	if (retval >= 0) {
		printf("Failure, can create pmd with illegal eth addr\n");
		return -1;
	}

	retval = rte_pmd_virtq_init("eth_virtq0",
			"mac=00:00:00:00:00:01;lcore_id=0");
	if (retval < 0) {
		printf("Failure, failed to create virtq pmd\n");
		return -1;
	}

	retval = rte_pmd_virtq_init("eth_virtq1",
			"mac=00:00:00:00:00:02;lcore_id=0");
	if (retval < 0) {
		printf("Failure, failed to create virtq pmd\n");
		return -1;
	}

	return 0;
}

static int
test_ethdev_configure(void)
{
	struct rte_eth_conf null_conf;
#ifdef ADVANCED_TESTS
	int limit = 0;
	struct rte_eth_link tx_link, rx_link;
#endif /* ADVANCED_TESTS */

	printf("Testing virtq pmd configure\n");

	memset(&null_conf, 0, sizeof(struct rte_eth_conf));

	if ((TX_PORT >= RTE_MAX_ETHPORTS) || (RX_PORT >= RTE_MAX_ETHPORTS)) {
		printf(" TX/RX port exceed max eth ports\n");
		return -1;
	}

	if (rte_eth_dev_configure(TX_PORT, 1, 1, &null_conf) < 0) {
		printf("Configure failed for TX port\n");
		return -1;
	}
	if (rte_eth_dev_configure(RX_PORT, 1, 1, &null_conf) < 0) {
		printf("Configure failed for RX port\n");
		return -1;
	}

	if (rte_eth_tx_queue_setup(TX_PORT, 0, 0, SOCKET, NULL) < 0) {
		printf("TX queue setup failed\n");
		return -1;
	}
	if (rte_eth_rx_queue_setup(RX_PORT, 0, 0, SOCKET,
			NULL, mp) < 0) {
		printf("RX queue setup failed\n");
		return -1;
	}

	if (rte_eth_dev_start(TX_PORT) < 0) {
		printf("Error starting TX port\n");
		return -1;
	}
	if (rte_eth_dev_start(RX_PORT) < 0) {
		printf("Error starting RX port\n");
		return -1;
	}

#ifdef ADVANCED_TESTS
	do {
		sleep(1);
		rte_eth_link_get(RX_PORT, &rx_link);
		rte_eth_link_get(TX_PORT, &tx_link);
		if (limit == 30)
			return -1;
		limit++;
	} while (!rx_link.link_status || !tx_link.link_status);
#endif /* ADVANCED_TESTS */

	return 0;
}

#ifdef ADVANCED_TESTS
static int
test_send_basic_packets(int pktnum, size_t pktsize)
{
	int i, tx_total, rx_total = 0, limit = 0;
	struct rte_mbuf **tx_bufs, **rx_bufs;
	struct rte_eth_stats stats;
	char *cmpdata;

	if ((pktnum < 0) || (pktsize <= 0))
		return -1;

	rte_eth_stats_reset(RX_PORT);
	rte_eth_stats_reset(TX_PORT);

	/* alloc TX buffer pointer */
	tx_bufs = (struct rte_mbuf **)malloc(sizeof(struct rte_mbuf *) * pktnum);
	if (tx_bufs ==NULL) {
		printf("Error: Allocating TX buffer pointer\n");
		return -1;
	}

	/* init TX buffer */
	for (i = 0; i < pktnum; i++) {
		tx_bufs[i] = rte_pktmbuf_alloc(mp);
		if (tx_bufs[i] == NULL) {
			printf("Error: Allocating packet buffer\n");
			return -1;
		}
		tx_bufs[i]->pkt.data_len = pktsize;
		tx_bufs[i]->pkt.pkt_len = pktsize;
		memset((void*)(tx_bufs[i]->pkt.data), (i & 0xff), pktsize);
	}

	/* send packets */
	tx_total = rte_eth_tx_burst(TX_PORT, 0, tx_bufs, pktnum);
	if (tx_total < pktnum) {
		for (i = tx_total; i < pktnum; i++)
			rte_pktmbuf_free(tx_bufs[i]);
	}

	/* alloc RX buffer pointer */
	rx_bufs = (struct rte_mbuf **)malloc(sizeof(struct rte_mbuf *) * tx_total);
	if (rx_bufs ==NULL) {
		printf("Error: Allocating RX buffer pointer\n");
		return -1;
	}

	/* receive packets */
	do {
		rx_total += rte_eth_rx_burst(RX_PORT, 0, &rx_bufs[rx_total], tx_total);
		if (limit == 100000000) {
			printf("Failed to receive packet burst %d/%d\n", rx_total, tx_total);
			return -1;
		}
		limit++;
	} while (rx_total < tx_total);

	/* check for stats update */
	cmpdata = malloc(pktsize);
	if (cmpdata == NULL) {
		printf("Error: Allocating buffer for comparison\n");
		return -1;
	}

	for (i = 0; i < rx_total; i++) {
		memset(cmpdata, (i & 0xff), pktsize);
		if (memcmp(cmpdata, rx_bufs[i]->pkt.data, pktsize)) {
			printf("Error: received data does not match that transmitted\n");
			return -1;
		}
		rte_pktmbuf_free(rx_bufs[i]);
	}
	free(tx_bufs);
	free(rx_bufs);
	free(cmpdata);

	/* check stats of RX/TX port, should all be zero */
	rte_eth_stats_get(RX_PORT, &stats);
	if (stats.ipackets != (unsigned int )pktnum || stats.opackets != 0 ||
			stats.ibytes != pktnum * pktsize || stats.obytes != 0 ||
			stats.ierrors != 0 || stats.oerrors != 0) {
		printf("Error: RX port stats are not correct\n");
		return -1;
	}
	rte_eth_stats_get(TX_PORT, &stats);
	if (stats.ipackets != 0 || stats.opackets != (unsigned int)pktnum ||
			stats.ibytes != 0 || stats.obytes != pktnum * pktsize ||
			stats.ierrors != 0 || stats.oerrors != 0) {
		printf("Error: TX port stats are not correct\n");
		return -1;
	}

	return 0;
}
#endif /* ADVANCED_TESTS */

static int
test_get_stats(void)
{
#ifdef ADVANCED_TESTS
	int i, pktnum, pktsize;
#endif /* ADVANCED_TESTS */
	struct rte_eth_stats stats;

	printf("Testing virtq PMD stats\n");

	/* check stats of RX/TX port, should all be zero */
	rte_eth_stats_get(RX_PORT, &stats);
	if (stats.ipackets != 0 || stats.opackets != 0 ||
			stats.ibytes != 0 || stats.obytes != 0 ||
			stats.ierrors != 0 || stats.oerrors != 0) {
		printf("Error: RX port stats are not zero\n");
		return -1;
	}
	rte_eth_stats_get(TX_PORT, &stats);
	if (stats.ipackets != 0 || stats.opackets != 0 ||
			stats.ibytes != 0 || stats.obytes != 0 ||
			stats.ierrors != 0 || stats.oerrors != 0) {
		printf("Error: TX port stats are not zero\n");
		return -1;
	}

#ifdef ADVANCED_TESTS
	for (i = 0; i < 1000000; i++) {
		pktnum = i % 32;
		pktsize = (i * 64) % 1536 + 64;
		if (test_send_basic_packets(pktnum, pktsize) < 0)
			return -1;
	}
#endif /* ADVANCED_TESTS */

	rte_eth_stats_reset(RX_PORT);
	rte_eth_stats_reset(TX_PORT);

	/* check stats of RX/TX port, should all be zero */
	rte_eth_stats_get(RX_PORT, &stats);
	if (stats.ipackets != 0 || stats.opackets != 0 ||
			stats.ibytes != 0 || stats.obytes != 0 ||
			stats.ierrors != 0 || stats.oerrors != 0) {
		printf("Error: RX port stats are not zero\n");
		return -1;
	}
	rte_eth_stats_get(TX_PORT, &stats);
	if (stats.ipackets != 0 || stats.opackets != 0 ||
			stats.ibytes != 0 || stats.obytes != 0 ||
			stats.ierrors != 0 || stats.oerrors != 0) {
		printf("Error: TX port stats are not zero\n");
		return -1;
	}
	return 0;
}

int
test_pmd_virtq(void)
{
	int ret = 0;

	mp = rte_mempool_create("virtq_mbuf_pool", NB_MBUF,
			MBUF_SIZE, 32,
			sizeof(struct rte_pktmbuf_pool_private),
			rte_pktmbuf_pool_init, NULL,
			rte_pktmbuf_init, NULL,
			rte_socket_id(), 0);
	if (mp == NULL)
		return -1;

	start_idx = rte_eth_dev_count();

	if (setenv("RTE_PMD_VIRTQ_CONNECTOR", "/tmp/virtq0", 1) < 0)
		return -1;

	if ((TX_PORT >= RTE_MAX_ETHPORTS) || (RX_PORT >= RTE_MAX_ETHPORTS)) {
		printf(" TX/RX port exceed max eth ports\n");
		return -1;
	}

	if (test_virtq_ethdev_create() < 0) {
		ret = -1;
		goto err0;
	}

	if (test_ethdev_configure() < 0) {
		ret = -1;
		goto err1;
	}

	if (test_get_stats() < 0) {
		ret = -1;
		goto err1;
	}

err1:
	rte_eth_dev_stop(RX_PORT);
	rte_eth_dev_stop(TX_PORT);

err0:
	return ret;
}

#else	/* RTE_LIBRTE_PMD_VIRTQ */

int
test_pmd_virtq(void)
{
	return 0;
}

#endif	/* RTE_LIBRTE_PMD_VIRTQ */

