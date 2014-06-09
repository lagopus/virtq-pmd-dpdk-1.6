/*-
 *   BSD LICENSE
 *
 *   Copyright(c) 2010-2013 Intel Corporation. All rights reserved.
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

#include <rte_log.h>
#include <rte_lcore.h>
#include "rte_eth_virtq_thread.h"

/* set affinity for current thread */
/* This function was imported from lib/librte_eal/linuxapp/eal/eal_thread.c */
int
virtq_thread_set_affinity(unsigned lcore_id)
{
	int s;
	pthread_t thread;

/*
 * According to the section VERSIONS of the CPU_ALLOC man page:
 *
 * The CPU_ZERO(), CPU_SET(), CPU_CLR(), and CPU_ISSET() macros were added
 * in glibc 2.3.3.
 *
 * CPU_COUNT() first appeared in glibc 2.6.
 *
 * CPU_AND(),     CPU_OR(),     CPU_XOR(),    CPU_EQUAL(),    CPU_ALLOC(),
 * CPU_ALLOC_SIZE(), CPU_FREE(), CPU_ZERO_S(),  CPU_SET_S(),  CPU_CLR_S(),
 * CPU_ISSET_S(),  CPU_AND_S(), CPU_OR_S(), CPU_XOR_S(), and CPU_EQUAL_S()
 * first appeared in glibc 2.7.
 */
#if defined(CPU_ALLOC)
	size_t size;
	cpu_set_t *cpusetp;

	cpusetp = CPU_ALLOC(RTE_MAX_LCORE);
	if (cpusetp == NULL) {
		RTE_LOG(ERR, EAL, "CPU_ALLOC failed\n");
		return -1;
	}

	size = CPU_ALLOC_SIZE(RTE_MAX_LCORE);
	CPU_ZERO_S(size, cpusetp);
	CPU_SET_S(lcore_id, size, cpusetp);

	thread = pthread_self();
	s = pthread_setaffinity_np(thread, size, cpusetp);
	if (s != 0) {
		RTE_LOG(ERR, EAL, "pthread_setaffinity_np failed\n");
		CPU_FREE(cpusetp);
		return -1;
	}

	CPU_FREE(cpusetp);
#else /* CPU_ALLOC */
	cpu_set_t cpuset;

	CPU_ZERO(&cpuset);
	CPU_SET(lcore_id, &cpuset);

	thread = pthread_self();
	s = pthread_setaffinity_np(thread, sizeof(cpuset), &cpuset);
	if (s != 0) {
		RTE_LOG(ERR, EAL, "pthread_setaffinity_np failed\n");
		return -1;
	}
#endif
	return 0;
}
