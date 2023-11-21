/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * aie.h - MtkISP7 AI Engine device
 */

#pragma once

#include <chrono>
#include <cstdint>
#include <memory>

#include <libcamera/base/unique_fd.h>

#include <libcamera/controls.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/pools.h"
#include "libcamera/internal/task_scheduler.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "mtkcam-core/hw/aie/3.1/hardware/v4l2/cam_fdvt_v4l2.h"

namespace libcamera {

class AieDevice
{
public:
	AieDevice();

	class AieTask : public Task
	{
	public:
		AieTask(Scheduler *scheduler, const std::string &id,
			AieDevice *aieDev,
			SharedMailBox<InfoFrame> mailBoxInputImage,
			SharedMailBox<InfoFrame> mailBoxMetadata,
			SharedMailBox<FdDrv_input_struct> mailBoxDriverConfig);

		void run() override;

	private:
		void notifySubTaskDone();

		void requestFdReady();
		void resultMetaReady(FrameBuffer *buffer);
		void sourceVideoReady(FrameBuffer *buffer);

		void requestFdCleanup();
		void resultMetaCleanup();
		void sourceVideoCleanup();

		AieDevice *aieDev_;
		std::unique_ptr<EventNotifier> fdBufferNotifier_;
		SharedMailBox<InfoFrame> mailBoxInputImage_;
		SharedMailBox<InfoFrame> mailBoxMetadata_;
		SharedMailBox<FdDrv_input_struct> mailBoxDriverConfig_;
		std::chrono::steady_clock::time_point timeBeginRun_;

		int requestFd_;

		int pendingSubTaskCount_;
	};

	int configure();
	int init(MediaDevice *media, DmaHeap *dmaHeap);
	int start();
	int stop();

	FdDrv_input_struct createFaceDetectionDriverConfig();
	FdDrv_input_struct createFaceToneClassificationDriverConfig();

private:
	int configureStreams();
	FdDrv_input_struct createDefaultDriverConfig();
	int createRequestFDs(unsigned int count);
	int releaseBuffers();
	int requestBuffers();

	const Size inputSize_;
	const unsigned int bufferNum_;

	std::unique_ptr<V4L2VideoDevice> sourceVideo_;
	std::unique_ptr<V4L2VideoDevice> resultMeta_;

	MediaDevice *media_;
	DmaHeap *dmaHeap_;

	InfoFramePool resultMetadataPool_;
	Pool<int, UniqueFD> requestFDPool_;

	FdDrv_init_struct driverInitConfig_;

	uint32_t initControlId_;
	uint32_t inferenceParamControlId_;
};

} /* namespace libcamera */
