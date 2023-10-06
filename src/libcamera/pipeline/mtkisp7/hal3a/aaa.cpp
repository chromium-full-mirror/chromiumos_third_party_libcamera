/*
 * Copyright (C) 2023, Google Inc.
 *
 * capture.h - MTK MtkISP7 Hal 3A Manager
 */

#include "aaa.h"

#include <libcamera/formats.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "hal_3a.h"

namespace libcamera {

namespace {

static constexpr Size kMetaSize = Size{ Hal3A::kRawMetaSize, 1 };

} // namespace

void Hal3AManager::configure(DmaHeap *dmaHeap, CamSysDevice *camSys,
			     Hal3A *hal3A)
{
	dmaHeap_ = dmaHeap;
	camSys_ = camSys;
	hal3A_ = hal3A;

	releaseBuffers();
	allocateBuffers();
}

void Hal3AManager::allocateBuffers()
{
	tuningPool_.createBuffers(dmaHeap_, formats::MTFP_MTISP, kMetaSize, 8,
				  DmaHeap::CMA);

	thread3A_.start();
}

void Hal3AManager::releaseBuffers()
{
	tuningPool_.release();

	thread3A_.exit();
	thread3A_.wait();
}

std::tuple<AATask *> Hal3AManager::make3ATasks(
	Scheduler *scheduler, Request *request,
	CaptureFrames &captureFrames, uint32_t internalRequestId,
	uint32_t camSysMetaRequestId)
{
	std::string sequence = "padding";
	if (request)
		sequence = std::to_string(request->sequence());

	AATask *aaTask = new AATask(this, scheduler, "3A " + sequence,
				    captureFrames, hal3A_,
				    internalRequestId, camSysMetaRequestId);
	aaTask->moveToThread(&thread3A_);

	return std::make_tuple(aaTask);
}

void AATask::run()
{
	manager_->fetchTuningBuffer(captureFrames_.tuningOutput);

	FrameBuffer *tuningBuffer = captureFrames_.tuningOutput->get().buffer();
	MappedFrameBuffer mappedBuffer(tuningBuffer,
				       MappedFrameBuffer::MapFlag::ReadWrite);
	tuningBuffer->_d()->metadata().planes()[0].bytesused =
		tuningBuffer->planes()[0].length;

	std::pair<uint32_t, uint32_t> exposureAndGain;
	hal3A_->doCalculation(captureFrames_.statistics0->get().buffer(),
			      captureFrames_.timestamp->get(),
			      internalRequestId_, camSysMetaRequestId_,
			      tuningBuffer->planes()[0].fd.get(),
			      mappedBuffer.planes()[0].data(),
			      &exposureAndGain);
	captureFrames_.exposureAndGainOutput->put(
		std::move(exposureAndGain),
		[]([[maybe_unused]] std::pair<uint32_t, uint32_t>
			   &exposureAndGain) {});

	notifyDone();
}

} // namespace libcamera
