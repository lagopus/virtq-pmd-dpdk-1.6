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

#include <stdint.h>
#include <string.h>
#include <stdio.h>
#include <errno.h>
#include <unistd.h>

#include <rte_ethdev.h>
#include <rte_memcpy.h>
#include <rte_string_fns.h>
#include <rte_memzone.h>
#include <rte_malloc.h>
#include <rte_atomic.h>
#include <rte_branch_prediction.h>
#include <rte_pci.h>
#include <rte_ether.h>
#include <rte_common.h>

#include <rte_memory.h>
#include <rte_eal.h>

#include "virtio_ethdev.h"
#include "virtio_pci.h"
#include "virtio_logs.h"
#include "virtqueue.h"


static int eth_virtio_dev_init(struct eth_driver *eth_drv,
		struct rte_eth_dev *eth_dev);
static int  virtio_dev_configure(struct rte_eth_dev *dev);
static int  virtio_dev_start(struct rte_eth_dev *dev);
static void virtio_dev_stop(struct rte_eth_dev *dev);
static void virtio_dev_info_get(struct rte_eth_dev *dev,
				struct rte_eth_dev_info *dev_info);
static int virtio_dev_link_update(struct rte_eth_dev *dev,
	__rte_unused int wait_to_complete);

static void virtio_set_hwaddr(struct virtio_hw *hw);
static void virtio_get_hwaddr(struct virtio_hw *hw);

static void virtio_dev_rx_queue_release(__rte_unused void *rxq);
static void virtio_dev_tx_queue_release(__rte_unused void *txq);

static void virtio_dev_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats);
static void virtio_dev_stats_reset(struct rte_eth_dev *dev);
static void virtio_dev_free_mbufs(struct rte_eth_dev *dev);

/*
 * The set of PCI devices this driver supports
 */
static struct rte_pci_id pci_id_virtio_map[] = {

#define RTE_PCI_DEV_ID_DECL_VIRTIO(vend, dev) {RTE_PCI_DEVICE(vend, dev)},
#include "rte_pci_dev_ids.h"

{ .vendor_id = 0, /* sentinel */ },
};

int virtio_dev_queue_setup(struct rte_eth_dev *dev,
			int queue_type,
			uint16_t queue_idx,
			uint8_t  vtpci_queue_idx,
			uint16_t nb_desc,
			unsigned int socket_id,
			struct virtqueue **pvq)
{
	char vq_name[VIRTQUEUE_MAX_NAME_SZ];
	const struct rte_memzone *mz;
	uint16_t vq_size;
	int size;
	struct virtio_hw *hw = 
		VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	struct virtqueue  *vq = NULL;

	/* Write the virtqueue index to the Queue Select Field */
	VIRTIO_WRITE_REG_2(hw, VIRTIO_PCI_QUEUE_SEL, vtpci_queue_idx);
	PMD_INIT_LOG(DEBUG, "selecting queue: %d\n", vtpci_queue_idx);

	/*
	 * Read the virtqueue size from the Queue Size field
	 * Always power of 2 and if 0 virtqueue does not exist
	 */
	vq_size = VIRTIO_READ_REG_2(hw, VIRTIO_PCI_QUEUE_NUM);
	PMD_INIT_LOG(DEBUG, "vq_size: %d nb_desc:%d\n", vq_size, nb_desc);
	if (nb_desc == 0)
		nb_desc = vq_size;
	if (vq_size == 0) {
		PMD_INIT_LOG(ERR, "%s: virtqueue does not exist\n", __func__);
		return (-EINVAL);
	} else if (!rte_is_power_of_2(vq_size)) {
		PMD_INIT_LOG(ERR, "%s: virtqueue size is not powerof 2\n", __func__);
		return (-EINVAL);
	} else if (nb_desc != vq_size) {
		PMD_INIT_LOG(ERR, "Warning: nb_desc(%d) is not equal to vq size (%d), fall to vq size\n",
			nb_desc, vq_size);
		nb_desc = vq_size;
	}

	if (queue_type == VTNET_RQ) {
		rte_snprintf(vq_name, sizeof(vq_name), "port%d_rvq%d",
				dev->data->port_id, queue_idx);
		vq = rte_zmalloc(vq_name, sizeof(struct virtqueue) +
			vq_size * sizeof(struct vq_desc_extra), CACHE_LINE_SIZE);
		memcpy(vq->vq_name, vq_name, sizeof(vq->vq_name));
	} else if(queue_type == VTNET_TQ) {
		rte_snprintf(vq_name, sizeof(vq_name), "port%d_tvq%d",
			dev->data->port_id, queue_idx);
		vq = rte_zmalloc(vq_name, sizeof(struct virtqueue) +
			vq_size * sizeof(struct vq_desc_extra), CACHE_LINE_SIZE);
		memcpy(vq->vq_name, vq_name, sizeof(vq->vq_name));
	} else if(queue_type == VTNET_CQ) {
		rte_snprintf(vq_name, sizeof(vq_name), "port%d_cvq",
				dev->data->port_id);
		vq = rte_zmalloc(vq_name, sizeof(struct virtqueue),
			CACHE_LINE_SIZE);
		memcpy(vq->vq_name, vq_name, sizeof(vq->vq_name));
	}
	if (vq == NULL) {
		PMD_INIT_LOG(ERR, "%s: Can not allocate virtqueue\n", __func__);
		return (-ENOMEM); 
	}
	vq->hw = hw;
	vq->port_id = dev->data->port_id;
	vq->queue_id = queue_idx;
	vq->vq_queue_index = vtpci_queue_idx;
	vq->vq_alignment = VIRTIO_PCI_VRING_ALIGN;
	vq->vq_nentries = vq_size;
	vq->vq_free_cnt = vq_size;

	/*
	 * Reserve a memzone for vring elements
	 */
	size = vring_size(vq_size, VIRTIO_PCI_VRING_ALIGN);
	vq->vq_ring_size = RTE_ALIGN_CEIL(size, VIRTIO_PCI_VRING_ALIGN);
	PMD_INIT_LOG(DEBUG, "vring_size: %d, rounded_vring_size: %d\n", size, vq->vq_ring_size);

	mz = rte_memzone_reserve_aligned(vq_name, vq->vq_ring_size,
			socket_id, 0, VIRTIO_PCI_VRING_ALIGN);
	if (mz == NULL) {
		rte_free(vq);
		return (-ENOMEM);
	}
	/*
 	* Virtio PCI device VIRTIO_PCI_QUEUE_PF register is 32bit,
 	* and only accepts 32 bit page frame number. 
 	* Check if the allocated physical memory exceeds 16TB.
 	*/
	if ( (mz->phys_addr + vq->vq_ring_size - 1) >> (VIRTIO_PCI_QUEUE_ADDR_SHIFT + 32) ) {
		PMD_INIT_LOG(ERR, "vring address shouldn't be above 16TB!\n");
		rte_free(vq);
		return (-ENOMEM);
	}
	memset(mz->addr, 0, sizeof(mz->len));
	vq->mz = mz;
	vq->vq_ring_mem = mz->phys_addr;
	vq->vq_ring_virt_mem = mz->addr;
	PMD_INIT_LOG(DEBUG, "vq->vq_ring_mem:      0x%"PRIx64"\n", (uint64_t)mz->phys_addr);
	PMD_INIT_LOG(DEBUG, "vq->vq_ring_virt_mem: 0x%"PRIx64"\n", (uint64_t)mz->addr);
	vq->virtio_net_hdr_mz  = NULL;
	vq->virtio_net_hdr_mem = (void *)NULL;

	if (queue_type == VTNET_TQ) {
		/* 
		* For each xmit packet, allocate a virtio_net_hdr
		*/
		rte_snprintf(vq_name, sizeof(vq_name), "port%d_tvq%d_hdrzone",
			dev->data->port_id, queue_idx);
		vq->virtio_net_hdr_mz = rte_memzone_reserve_aligned(vq_name,
			vq_size * sizeof(struct virtio_net_hdr),
			socket_id, 0, CACHE_LINE_SIZE);
		if (vq->virtio_net_hdr_mz == NULL) {
			rte_free(vq);
			return (-ENOMEM);
		}
		vq->virtio_net_hdr_mem = (void *)(uintptr_t)vq->virtio_net_hdr_mz->phys_addr;
		memset(vq->virtio_net_hdr_mz->addr, 0, vq_size * sizeof(struct virtio_net_hdr));
	} else if (queue_type == VTNET_CQ) {
		/* Allocate a page for control vq command, data and status */
		rte_snprintf(vq_name, sizeof(vq_name), "port%d_cvq_hdrzone",
			dev->data->port_id);
		vq->virtio_net_hdr_mz = rte_memzone_reserve_aligned(vq_name,
			PAGE_SIZE, socket_id, 0, CACHE_LINE_SIZE);
		if (vq->virtio_net_hdr_mz == NULL) {
			rte_free(vq);
			return (-ENOMEM);
		}
		vq->virtio_net_hdr_mem = (void *)(uintptr_t)vq->virtio_net_hdr_mz->phys_addr;
		memset(vq->virtio_net_hdr_mz->addr, 0, PAGE_SIZE);
	}

	/*
	 * Set guest physical address of the virtqueue
	 * in VIRTIO_PCI_QUEUE_PFN config register of device
	 */
	VIRTIO_WRITE_REG_4(hw, VIRTIO_PCI_QUEUE_PFN,
			mz->phys_addr >> VIRTIO_PCI_QUEUE_ADDR_SHIFT);
	*pvq = vq;
	return (0);
}

static int
virtio_dev_cq_queue_setup(struct rte_eth_dev *dev,
		unsigned int socket_id)
{
	struct virtqueue *vq;
	uint16_t nb_desc = 0;
	int ret;
	struct virtio_hw *hw =
		VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	PMD_INIT_FUNC_TRACE();
	ret = virtio_dev_queue_setup(dev, VTNET_CQ, 0, VTNET_SQ_CQ_QUEUE_IDX,
			nb_desc, socket_id, &vq);
	if (ret < 0) {
		PMD_INIT_LOG(ERR, "control vq initialization failed\n");
		return ret;
	}

	hw->cvq = vq;
	return (0);
}

static void
virtio_dev_close(struct rte_eth_dev *dev)
{
	PMD_INIT_LOG(DEBUG, "virtio_dev_close");

	virtio_dev_stop(dev);
}


/*
 * dev_ops for virtio, bare necessities for basic operation
 */
static struct eth_dev_ops virtio_eth_dev_ops = {
	.dev_configure         = virtio_dev_configure,
	.dev_start             = virtio_dev_start,
	.dev_stop              = virtio_dev_stop,
	.dev_close             = virtio_dev_close,

	.dev_infos_get         = virtio_dev_info_get,
	.stats_get             = virtio_dev_stats_get,
	.stats_reset           = virtio_dev_stats_reset,
	.link_update           = virtio_dev_link_update,
	.mac_addr_add          = NULL,
	.mac_addr_remove       = NULL,
	.rx_queue_setup        = virtio_dev_rx_queue_setup,
	.rx_queue_release      = virtio_dev_rx_queue_release,  /* meaningfull only to multiple queue */
	.tx_queue_setup        = virtio_dev_tx_queue_setup,
	.tx_queue_release      = virtio_dev_tx_queue_release /* meaningfull only to multiple queue */
};

static inline int
virtio_dev_atomic_read_link_status(struct rte_eth_dev *dev,
				struct rte_eth_link *link)
{
	struct rte_eth_link *dst = link;
	struct rte_eth_link *src = &(dev->data->dev_link);

	if (rte_atomic64_cmpset((uint64_t *)dst, *(uint64_t *)dst,
			*(uint64_t *)src) == 0)
		return (-1);

	return (0);
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
virtio_dev_atomic_write_link_status(struct rte_eth_dev *dev,
		struct rte_eth_link *link)
{
	struct rte_eth_link *dst = &(dev->data->dev_link);
	struct rte_eth_link *src = link;

	if (rte_atomic64_cmpset((uint64_t *)dst, *(uint64_t *)dst,
					*(uint64_t *)src) == 0)
		return (-1);

	return (0);
}

static void
virtio_dev_stats_get(struct rte_eth_dev *dev, struct rte_eth_stats *stats)
{
	struct virtio_hw *hw =
		VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	if(stats)
		memcpy(stats, &hw->eth_stats, sizeof(*stats));
}

static void
virtio_dev_stats_reset(struct rte_eth_dev *dev)
{
	struct virtio_hw *hw =
		VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	/* Reset software totals */
	memset(&hw->eth_stats, 0, sizeof(hw->eth_stats));
}

static void
virtio_set_hwaddr(struct virtio_hw *hw)
{
	vtpci_write_dev_config(hw,
			offsetof(struct virtio_net_config, mac),
			&hw->mac_addr, ETHER_ADDR_LEN);
}

static void
virtio_get_hwaddr(struct virtio_hw *hw)
{
	if (vtpci_with_feature(hw, VIRTIO_NET_F_MAC)) {
		vtpci_read_dev_config(hw,
			offsetof(struct virtio_net_config, mac),
			&hw->mac_addr, ETHER_ADDR_LEN);
	} else {
		eth_random_addr(&hw->mac_addr[0]);
		virtio_set_hwaddr(hw);
	}
}


static void
virtio_negotiate_features(struct virtio_hw *hw)
{
	uint32_t guest_features, mask;
	mask = VIRTIO_NET_F_CTRL_VQ | VIRTIO_NET_F_CTRL_RX | VIRTIO_NET_F_CTRL_VLAN;
	mask |= VIRTIO_NET_F_CSUM | VIRTIO_NET_F_GUEST_CSUM ;

	/* TSO and LRO are only available when their corresponding
	 * checksum offload feature is also negotiated.
	 */
	mask |= VIRTIO_NET_F_HOST_TSO4 | VIRTIO_NET_F_HOST_TSO6 | VIRTIO_NET_F_HOST_ECN;
	mask |= VIRTIO_NET_F_GUEST_TSO4 | VIRTIO_NET_F_GUEST_TSO6 | VIRTIO_NET_F_GUEST_ECN;
	mask |= VTNET_LRO_FEATURES;

	/* rx_mbuf should not be in multiple merged segments */
	mask |= VIRTIO_NET_F_MRG_RXBUF;

	/* not negotiating INDIRECT descriptor table support */
	mask |= VIRTIO_RING_F_INDIRECT_DESC;

	/* Prepare guest_features: feature that driver wants to support */
	guest_features = VTNET_FEATURES & ~mask;

	/* Read device(host) feature bits */
	hw->host_features = VIRTIO_READ_REG_4(hw, VIRTIO_PCI_HOST_FEATURES);

	/* Negotiate features: Subset of device feature bits are written back (guest feature bits) */
	hw->guest_features = vtpci_negotiate_features(hw, guest_features);
}

/*
 * This function is based on probe() function in virtio_pci.c
 * It returns 0 on success.
 */
static int
eth_virtio_dev_init(__rte_unused struct eth_driver *eth_drv,
		struct rte_eth_dev *eth_dev)
{
	struct rte_pci_device *pci_dev;
	struct virtio_hw *hw =
		VIRTIO_DEV_PRIVATE_TO_HW(eth_dev->data->dev_private);
	if (RTE_PKTMBUF_HEADROOM < sizeof(struct virtio_net_hdr) ) {
		PMD_INIT_LOG(ERR, 
			"MBUF HEADROOM should be enough to hold virtio net hdr\n");
		return (-1); 
	}

	if (! (rte_eal_get_configuration()->flags & EAL_FLG_HIGH_IOPL)) {
		PMD_INIT_LOG(ERR,
			"IOPL call failed in EAL init - cannot use virtio PMD driver\n");
		return (-1);
	}

	eth_dev->dev_ops = &virtio_eth_dev_ops;
	eth_dev->rx_pkt_burst = &virtio_recv_pkts;
	eth_dev->tx_pkt_burst = &virtio_xmit_pkts;

	if(rte_eal_process_type() == RTE_PROC_SECONDARY)
		return 0;

	pci_dev = eth_dev->pci_dev;

	hw->device_id = pci_dev->id.device_id;
	hw->vendor_id = pci_dev->id.vendor_id;
	hw->io_base = (uint32_t)(uintptr_t)pci_dev->mem_resource[0].addr;

	hw->max_rx_queues = VIRTIO_MAX_RX_QUEUES;
	hw->max_tx_queues = VIRTIO_MAX_TX_QUEUES;

	/* Reset the device although not necessary at startup */
	vtpci_reset(hw);

	/* Tell the host we've noticed this device. */
	vtpci_set_status(hw, VIRTIO_CONFIG_STATUS_ACK);

	/* Tell the host we've known how to drive the device. */
	vtpci_set_status(hw, VIRTIO_CONFIG_STATUS_DRIVER);
	virtio_negotiate_features(hw);
	/* Setting up rx_header size for the device */
	if(vtpci_with_feature(hw, VIRTIO_NET_F_MRG_RXBUF))
		hw->vtnet_hdr_size = sizeof(struct virtio_net_hdr_mrg_rxbuf);
	else
		hw->vtnet_hdr_size = sizeof(struct virtio_net_hdr);

	/* Allocate memory for storing MAC addresses */
	eth_dev->data->mac_addrs = rte_zmalloc("virtio", ETHER_ADDR_LEN, 0);
	if (eth_dev->data->mac_addrs == NULL) {
		PMD_INIT_LOG(ERR,
			"Failed to allocate %d bytes needed to store MAC addresses",
			ETHER_ADDR_LEN);
		return (-ENOMEM);
	}
	/* Copy the permanent MAC address to: virtio_hw */
	virtio_get_hwaddr(hw);
	ether_addr_copy((struct ether_addr *) hw->mac_addr,
			&eth_dev->data->mac_addrs[0]);
	PMD_INIT_LOG(DEBUG, "PORT MAC: %02X:%02X:%02X:%02X:%02X:%02X\n", hw->mac_addr[0],
			hw->mac_addr[1],hw->mac_addr[2], hw->mac_addr[3], hw->mac_addr[4], hw->mac_addr[5]);

	if(vtpci_with_feature(hw, VIRTIO_NET_F_CTRL_VQ))
		virtio_dev_cq_queue_setup(eth_dev, SOCKET_ID_ANY);

	PMD_INIT_LOG(DEBUG, "port %d vendorID=0x%x deviceID=0x%x",
			eth_dev->data->port_id, pci_dev->id.vendor_id,
			pci_dev->id.device_id);
	return (0);
}

static struct eth_driver rte_virtio_pmd = {
	{
		.name = "rte_virtio_pmd",
		.id_table = pci_id_virtio_map,
#ifdef RTE_EAL_UNBIND_PORTS
		.drv_flags = RTE_PCI_DRV_NEED_IGB_UIO,
#endif
	},
	.eth_dev_init = eth_virtio_dev_init,
	.dev_private_size = sizeof(struct virtio_adapter),
};

/*
 * Driver initialization routine.
 * Invoked once at EAL init time.
 * Register itself as the [Poll Mode] Driver of PCI virtio devices.
 * Returns 0 on success.
 */
int
rte_virtio_pmd_init(void)
{
	rte_eth_driver_register(&rte_virtio_pmd);
	return (0);
}

/*
 * Only 1 queue is supported, no queue release related operation
 */
static void
virtio_dev_rx_queue_release(__rte_unused void *rxq)
{
}

static void
virtio_dev_tx_queue_release(__rte_unused void *txq)
{
}

/*
 * Configure virtio device
 * It returns 0 on success.
 */
static int
virtio_dev_configure(__rte_unused struct rte_eth_dev *dev)
{
	return (0);
}


static int
virtio_dev_start(struct rte_eth_dev *dev)
{
	uint16_t status;
	struct virtio_hw *hw =
		VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* Tell the host we've noticed this device. */
	vtpci_set_status(hw, VIRTIO_CONFIG_STATUS_ACK);

	/* Tell the host we've known how to drive the device. */
	vtpci_set_status(hw, VIRTIO_CONFIG_STATUS_DRIVER);

	hw->adapter_stopped = 0;

	/* Do final configuration before rx/tx engine starts */
	virtio_dev_rxtx_start(dev);

	/* Check VIRTIO_NET_F_STATUS for link status*/
	if(vtpci_with_feature(hw, VIRTIO_NET_F_STATUS)) {

		vtpci_read_dev_config(hw,
				offsetof(struct virtio_net_config, status),
				&status, sizeof(status));
		if((status & VIRTIO_NET_S_LINK_UP) == 0) {
			PMD_INIT_LOG(ERR,     "Port: %d Link is DOWN\n", dev->data->port_id);
			return (-EIO);
		} else {
			PMD_INIT_LOG(DEBUG, "Port: %d Link is UP\n",  dev->data->port_id);
		}
	}
	vtpci_reinit_complete(hw);

	/*Notify the backend
	 *Otherwise the tap backend might already stop its queue due to fullness.
	 *vhost backend will have no chance to be waked up
	 */
	virtqueue_notify(dev->data->rx_queues[0]);
	PMD_INIT_LOG(DEBUG, "Notified backend at initialization\n");
	return (0);
}

static void virtio_dev_free_mbufs(struct rte_eth_dev *dev)
{
	struct rte_mbuf * buf;
	int i = 0;
	PMD_INIT_LOG(DEBUG, "Before freeing rxq used and unused buf \n");
	VIRTQUEUE_DUMP((struct virtqueue *)dev->data->rx_queues[0]);
	while( (buf =(struct rte_mbuf *)virtqueue_detatch_unused(dev->data->rx_queues[0])) != NULL) {
		rte_pktmbuf_free_seg(buf);
		i++;
	}
	PMD_INIT_LOG(DEBUG, "free %d mbufs\n", i);
	PMD_INIT_LOG(DEBUG, "After freeing rxq used and unused buf\n");
	VIRTQUEUE_DUMP((struct virtqueue *)dev->data->rx_queues[0]);
	PMD_INIT_LOG(DEBUG, "Before freeing txq used and unused bufs\n");
	VIRTQUEUE_DUMP((struct virtqueue *)dev->data->tx_queues[0]);
 	i = 0;
	while( (buf = (struct rte_mbuf *)virtqueue_detatch_unused(dev->data->tx_queues[0])) != NULL) {
		rte_pktmbuf_free_seg(buf);
		i++;
	}
	PMD_INIT_LOG(DEBUG, "free %d mbufs\n", i);
	PMD_INIT_LOG(DEBUG, "After freeing txq used and unused buf\n");
	VIRTQUEUE_DUMP((struct virtqueue *)dev->data->tx_queues[0]);
}

/*
 * Stop device: disable rx and tx functions to allow for reconfiguring.
 */
static void
virtio_dev_stop(struct rte_eth_dev *dev)
{
	struct virtio_hw *hw =
		VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);

	/* reset the NIC */
	vtpci_reset(hw);
	virtio_dev_free_mbufs(dev);
}

static int
virtio_dev_link_update(struct rte_eth_dev *dev, __rte_unused int wait_to_complete)
{
	struct rte_eth_link link, old;
	uint16_t status;
	struct virtio_hw *hw =
		VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	memset(&link, 0, sizeof(link));
	virtio_dev_atomic_read_link_status(dev, &link);
	old = link;
	link.link_duplex = FULL_DUPLEX ;
	link.link_speed  = SPEED_10G ;
	if(vtpci_with_feature(hw, VIRTIO_NET_F_STATUS)) {
		PMD_INIT_LOG(DEBUG, "Get link status from hw\n");
		vtpci_read_dev_config(hw,
				offsetof(struct virtio_net_config, status),
				&status, sizeof(status));
		if((status & VIRTIO_NET_S_LINK_UP) == 0) {
			link.link_status = 0;
			PMD_INIT_LOG(DEBUG, "Port %d is down\n",dev->data->port_id);
		} else {
			link.link_status = 1;
			PMD_INIT_LOG(DEBUG, "Port %d is up\n",dev->data->port_id);
		}
	} else {
		link.link_status = 1;   //Link up
	}
	virtio_dev_atomic_write_link_status(dev, &link);
	if(old.link_status == link.link_status)
		return (-1);
	/*changed*/
	return (0);
}

static void
virtio_dev_info_get(struct rte_eth_dev *dev, struct rte_eth_dev_info *dev_info)
{
	struct virtio_hw *hw = VIRTIO_DEV_PRIVATE_TO_HW(dev->data->dev_private);
	dev_info->driver_name = dev->driver->pci_drv.name;
	dev_info->max_rx_queues = (uint16_t)hw->max_rx_queues;
	dev_info->max_tx_queues = (uint16_t)hw->max_tx_queues;
	dev_info->min_rx_bufsize = VIRTIO_MIN_RX_BUFSIZE;
	dev_info->max_rx_pktlen = VIRTIO_MAX_RX_PKTLEN;
	dev_info->max_mac_addrs = VIRTIO_MAX_MAC_ADDRS;
}
