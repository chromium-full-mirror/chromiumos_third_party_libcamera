/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * mfnr.cpp - MtkISP7 ImgSys Device Mutiple Frame Noise Reduction
 */

#include "mfnr.h"

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/request.h>

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "single_device.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

/* todo: hide the NSCam::NSImgStream namespace in the single device interface. */
using namespace NSCam::NSImgStream;

MfnrTasksManager::MfnrTasksManager(
	ImgSysDevice *imgSys, DmaHeap *dmaHeap, OnDeviceTuner *odt)
{
	imgSys_ = imgSys;
	dmaHeap_ = dmaHeap;
	onDeviceTuner_ = odt;
}

int MfnrTasksManager::configure(const Size yuvInputSize, const Size videoOut1Size,
				const Size videoOut2Size)
{
	videoOut1Size_ = videoOut1Size;
	videoOut2Size_ = videoOut2Size;
	yuvInputSize_ = yuvInputSize;

	return 0;
}

int MfnrTasksManager::start()
{
#if !V4L2_STANDARD_MODE
	for (auto &pool : allBufferPools_)
		imgSys_->handleIova(ImgSysDevice::Add, *pool);
#endif
	return 0;
}

int MfnrTasksManager::stop()
{
#if !V4L2_STANDARD_MODE
	for (auto &pool : allBufferPools_)
		imgSys_->handleIova(ImgSysDevice::Delete, *pool);
#endif
	return 0;
}

int MfnrTasksManager::releaseBuffers()
{
	for (auto &pool : allBufferPools_)
		pool->release();

	return 0;
}

} /* namespace libcamera */
