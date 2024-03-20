/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * dma_heaps.cpp - Helper class for dma-heap allocations
 */

#include "libcamera/internal/dma_heaps.h"

#include <fcntl.h>
#include <sys/ioctl.h>

#include <linux/dma-buf.h>
#include <linux/dma-heap.h>

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DEFINE_CATEGORY(DmaHeap)

namespace {
constexpr const char *kHeapName = "/dev/dma_heap/system";
constexpr const char *kHeapCmaName = "/dev/dma_heap/scp-isp-cma-region";
} // namespace

DmaHeap::DmaHeap()
{
	valid_ = false;

	int fd = ::open(kHeapName, O_RDWR, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		LOG(DmaHeap, Error) << "Failed to open " << kHeapName << ": "
				    << strerror(errno);
		return;
	}

	UniqueFD systemHeap(fd);

	fd = ::open(kHeapCmaName, O_RDWR, O_RDONLY | O_CLOEXEC);
	if (fd < 0) {
		LOG(DmaHeap, Error) << "Failed to open " << kHeapCmaName << ": "
				    << strerror(errno);
		return;
	}

	dmaHeapCmaHandle_ = UniqueFD(fd);
	dmaHeapHandle_ = std::move(systemHeap);

	valid_ = true;
}

DmaHeap::~DmaHeap() = default;

UniqueFD DmaHeap::alloc(std::size_t size, Type type)
{
	struct dma_heap_allocation_data heap_data {
		.len = size,
		.fd = 0,
		.fd_flags = O_RDWR | O_CLOEXEC,
		.heap_flags = 0,
	};

	int dmaFd = (type == CMA) ? dmaHeapCmaHandle_.get() : dmaHeapHandle_.get();
	int ret;
	do {
		ret = ioctl(dmaFd, DMA_HEAP_IOCTL_ALLOC, &heap_data);
	} while (!ret && (errno == EINTR || errno == EAGAIN));

	if (ret) {
		LOG(DmaHeap, Error) << "Unable to allocate from "
				    << " DMA-BUF heap " << strerror(errno)
				    << " type " << ((type == CMA) ? "CMA" : "System")
				    << " heap_data.fd_flags " << heap_data.fd_flags
				    << " len " << heap_data.len;
		return {};
	}

	return UniqueFD(heap_data.fd);
}

void DmaHeap::sync(int fd, SyncStep step, SyncType type)
{
	uint64_t flags = 0;
	switch (step) {
	case Start:
		flags = DMA_BUF_SYNC_START;
		break;
	case End:
		flags = DMA_BUF_SYNC_END;
		break;
	}

	switch (type) {
	case SyncRead:
		flags = flags | DMA_BUF_SYNC_READ;
		break;
	case SyncWrite:
		flags = flags | DMA_BUF_SYNC_WRITE;
		break;
	case SyncReadWrite:
		flags = flags | DMA_BUF_SYNC_RW;
		break;
	}

	struct dma_buf_sync sync = {
		.flags = flags
	};

	int ret;
	do {
		ret = ioctl(fd, DMA_BUF_IOCTL_SYNC, &sync);
	} while (!ret && (errno == EINTR || errno == EAGAIN));

	if (ret) {
		LOG(DmaHeap, Error) << "Unable to sync dma fd " << fd
				    << " step " << step;
	}
}

} /* namespace libcamera */
