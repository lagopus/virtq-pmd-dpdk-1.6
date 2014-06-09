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

#include <stdint.h>
#include <string.h>
#include <stdlib.h>
#include <sys/stat.h>
#include <assert.h>
#include <sys/mman.h>
#include <errno.h>
#include <unistd.h>

#include "rte_eth_virtq_vmmapi.h"

#define MB	(1024UL * 1024)
#define GB	(1024UL * MB)

struct vmctx {
	int	fd;
	uint32_t lowmem_limit;
	enum vm_mmap_style vms;
	size_t	lowmem;
	size_t	highmem;
	char	*mem_addr;
	char	*name;
};

struct vmctx *
vm_open(char *name, int fd, uint32_t lowmem_limit)
{
	struct vmctx *vm;

	vm = malloc(sizeof(struct vmctx) + strlen(name) + 1);
	if (vm == NULL)
		goto err;

	vm->fd = -1;
	vm->lowmem_limit = lowmem_limit;
	vm->name = (char *)(vm + 1);
	strcpy(vm->name, name);

	vm->fd = fd;

	return vm;
err:
	vm_destroy(vm);
	return NULL;
}

void
vm_destroy(struct vmctx *vm)
{
	assert(vm != NULL);

	munmap(vm->mem_addr, vm->lowmem + vm->highmem);

	if (vm->fd >= 0)
		close(vm->fd);

	free(vm);
}

void *
vm_map_gpa(struct vmctx *ctx, vm_paddr_t gaddr, size_t len)
{

	/* XXX VM_MMAP_SPARSE not implemented yet */
	assert(ctx->vms == VM_MMAP_ALL);

	if (gaddr < ctx->lowmem && gaddr + len <= ctx->lowmem)
		return ((void *)(ctx->mem_addr + gaddr));

	if (gaddr >= 4*GB) {
		gaddr -= 4*GB;
		if (gaddr < ctx->highmem && gaddr + len <= ctx->highmem)
			return ((void *)(ctx->mem_addr
					+ ctx->lowmem_limit + gaddr));
	}

	return NULL;
}

static int
setup_memory_segment(struct vmctx *ctx, vm_paddr_t gpa, size_t len, char **addr)
{
	/*
	 * Create and optionally map 'len' bytes of memory at guest
	 * physical address 'gpa'
	 */
	if (addr != NULL) {
		*addr = mmap(NULL, len, PROT_READ | PROT_WRITE, MAP_SHARED,
				ctx->fd, gpa);
	}

	if (*addr == MAP_FAILED)
		return -1;

	return 0;
}

int
vm_sneak_memory(struct vmctx *ctx, enum vm_mmap_style vms)
{
	char **addr;
	int error;
	size_t memsize;
	struct stat st;

	if (fstat(ctx->fd, &st))
		return -errno;

	memsize = (ssize_t)st.st_size;

	/* XXX VM_MMAP_SPARSE not implemented yet */
	assert(vms == VM_MMAP_NONE || vms == VM_MMAP_ALL);
	ctx->vms = vms;

	/*
	 * If 'memsize' cannot fit entirely in the 'lowmem' segment then
	 * create another 'highmem' segment above 4GB for the remainder.
	 */
	if (memsize > ctx->lowmem_limit) {
		ctx->lowmem = ctx->lowmem_limit;
		ctx->highmem = memsize - ctx->lowmem;
	} else {
		ctx->lowmem = memsize;
		ctx->highmem = 0;
	}

	addr = (vms == VM_MMAP_ALL) ? &ctx->mem_addr : NULL;
	error = setup_memory_segment(ctx, 0, memsize, addr);
	if (error)
		return error;

	return 0;
}

void *
paddr_guest2host(struct vmctx *ctx, uintptr_t gaddr, size_t len)
{
	return vm_map_gpa(ctx, gaddr, len);
}
