#   BSD LICENSE
#
#   Copyright (C) 2014 Nippon Telegraph and Telephone Corporation.
#   All rights reserved.
#
#   Redistribution and use in source and binary forms, with or without
#   modification, are permitted provided that the following conditions
#   are met:
#
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in
#       the documentation and/or other materials provided with the
#       distribution.
#
#   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
#   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
#   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
#   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
#   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
#   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
#   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
#   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
#   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
#   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
#   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

include $(RTE_SDK)/mk/rte.vars.mk

#
# library name
#
LIB = librte_pmd_virtq.a

CFLAGS += -O3 -I../librte_pmd_pcap/
CFLAGS += $(WERROR_FLAGS)

#
# all source are stored in SRCS-y
#
SRCS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += rte_eth_virtq.c
SRCS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += rte_eth_virtq_virtio.c
SRCS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += rte_eth_virtq_vmmapi.c
SRCS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += rte_eth_virtq_pci_virtio_net.c
SRCS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += rte_eth_virtq_thread.c
SRCS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += ipc_node.c

CFLAGS_rte_eth_virtq_thread.o := -D_GNU_SOURCE

#
# Export include files
#
SYMLINK-y-include += rte_eth_virtq.h

# this lib depends upon:
DEPDIRS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += lib/librte_eal lib/librte_ether
DEPDIRS-$(CONFIG_RTE_LIBRTE_PMD_VIRTQ) += lib/librte_mempool lib/librte_mbuf

include $(RTE_SDK)/mk/rte.lib.mk
