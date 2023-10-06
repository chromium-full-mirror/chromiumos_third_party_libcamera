/*
 * Copyright (C) 2023, Google Inc.
 *
 * capture.h - MTK MtkISP7 Hal 3A Manager
 */

#pragma once

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/task_scheduler.h"

#include "libcamera/base/thread.h"
#include "pipeline/mtkisp7/camsys/camsys.h"
#include "pipeline/mtkisp7/camsys/capture.h"

#include "hal_3a.h"

namespace libcamera {

class AATask;

class Hal3AManager
{
public:
	void configure(DmaHeap *dmaHeap, CamSysDevice *camSys, Hal3A *hal3A);

	void releaseBuffers();

	std::tuple<AATask *>
	make3ATasks(Scheduler *scheduler, Request *request,
		    CaptureFrames &captureFrames,
		    uint32_t internalRequestId, uint32_t camSysMetaRequestId);

	void fetchTuningBuffer(SharedMailBox<InfoFrame> &mailBox)
	{
		tuningPool_.fetch(mailBox);
	}

private:
	void allocateBuffers();

	DmaHeap *dmaHeap_;
	CamSysDevice *camSys_;
	Hal3A *hal3A_;

	InfoFramePool tuningPool_;

	Thread thread3A_;
};

// AE & AWB task.
class AATask : public Task
{
public:
	AATask(Hal3AManager *manager, Scheduler *scheduler, const std::string &id,
	       CaptureFrames &captureFrames, Hal3A *hal3A,
	       uint32_t internalRequestId, uint32_t camSysMetaRequestId)
		: Task(scheduler, id), manager_(manager),
		  captureFrames_(captureFrames), hal3A_(hal3A),
		  internalRequestId_(internalRequestId),
		  camSysMetaRequestId_(camSysMetaRequestId) {}

	void run() override final;

	Hal3AManager *manager_;
	CaptureFrames captureFrames_;

	Hal3A *hal3A_;

	uint32_t internalRequestId_;
	uint32_t camSysMetaRequestId_;
};

} // namespace libcamera
