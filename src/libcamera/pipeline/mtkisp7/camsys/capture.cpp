/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * capture.cpp - MTK MtkISP7 Camsys Device Capture Tasks
 */

#include "capture.h"

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/request.h"

#include <libcamera/internal/info_frame.h>

#include "mt8188/mtk_cam_metabuf.h"

#include "camsys.h"
#include "mtk_cam_metabuf.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

static constexpr Size kMeSize = Size{ 576, 432 };
static constexpr Size kFdSize = Size{ 640, 480 };
static constexpr Size kMetaSize = Size{ 113664, 1 };
static constexpr Size kStatSize0 = Size{ 1081344, 1 };
static constexpr Size kStatSize1 = Size{ 528384, 1 };

} // namespace

CaptureTasksManager::CaptureTasksManager(OnDeviceTuner *odt)
	: onDeviceTuner_(odt)
{
}

int CaptureTasksManager::configure(DmaHeap *dmaHeap,
				   CamSysDevice *camSys,
				   PipelineHandler *pipe,
				   const Size &rawFrameSize,
				   const Size &yuvFrameSize,
				   Hal3A *hal3A)
{
	dmaHeap_ = dmaHeap;
	camSys_ = camSys;
	pipe_ = pipe;

	rawFrameSize_ = rawFrameSize;
	yuvFrameSize_ = yuvFrameSize;

	hal3A_ = hal3A;

	releaseBuffers();
	allocateBuffers();

	return 0;
}

void CaptureTasksManager::allocateBuffers()
{
	rawPool_.createBuffers(dmaHeap_, camSys_->bayerFormat(), rawFrameSize_, 12);
	yuvo1Pool_.createBuffers(dmaHeap_, formats::NV12_10P_MTISP, yuvFrameSize_, 12);
	yuvo2Pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, yuvFrameSize_ / 2, 12);
	mePool_.createBuffers(dmaHeap_, formats::GREY, kMeSize, 12);
	faceDetectPool_.createBuffers(dmaHeap_, formats::NV12, kFdSize, 12);
	tuningPool_.createBuffers(dmaHeap_, formats::MTFP_MTISP, kMetaSize, 8, DmaHeap::CMA);
	statistics0Pool_.createBuffers(dmaHeap_, formats::MTFA_MTISP, kStatSize0, 8, DmaHeap::CMA);
	statistics1Pool_.createBuffers(dmaHeap_, formats::MTFF_MTISP, kStatSize1, 8, DmaHeap::CMA);
}

void CaptureTasksManager::releaseBuffers()
{
	rawPool_.release();

	yuvo1Pool_.release();
	yuvo2Pool_.release();
	yuvo3Pool_.release();
	yuvo4Pool_.release();

	mePool_.release();
	faceDetectPool_.release();

	tuningPool_.release();
	statistics0Pool_.release();
	statistics1Pool_.release();
}

void CaptureTasksManager::makeCaptureFrames(CaptureFrames &captureFrames)
{
	captureFrames.tuning = makeMailBox<InfoFrame>();

	captureFrames.raw = makeMailBox<InfoFrame>();
	captureFrames.yuvo1 = makeMailBox<InfoFrame>();
	captureFrames.yuvo2 = makeMailBox<InfoFrame>();

	captureFrames.me = makeMailBox<InfoFrame>();
	captureFrames.faceDetection = makeMailBox<InfoFrame>();

	captureFrames.statistics0 = makeMailBox<InfoFrame>();
	captureFrames.statistics1 = makeMailBox<InfoFrame>();
}

std::tuple<QueueTask *, DequeueTask *, SofTask *>
CaptureTasksManager::makeCaptureTasks(Scheduler *scheduler,
				      const std::string &id,
				      Request *request,
				      CaptureFrames &captureFrames)
{
	(void)id;
	auto data = std::make_shared<CaptureData>(captureFrames);

	std::string sequence = "padding";
	if (request)
		sequence = std::to_string(request->sequence());

	SofTask *sofTask = new SofTask(scheduler, "Sof " + sequence, request,
				       camSys_, hal3A_);
	QueueTask *qTask = new QueueTask(this, scheduler, "Queue " + sequence, request, data);
	DequeueTask *dqTask = new DequeueTask(this, scheduler, "Dequeue " + sequence, request, data, hal3A_);

	return std::make_tuple(qTask, dqTask, sofTask);
}

void SofTask::trigger()
{
	// todo: Set exposure and gain accordingly.
	// todo: Set exposure and gain considering the delay of 2.
	auto [exposure, gain] = hal3A_->getExposureAndGain();
	if (exposure != 0) // Assuming it couldn't be zero.
		camSys_->setExposureGain(exposure, gain);
	LOG(MtkISP7, Info) << "exposure: " << exposure << ", gain: " << gain;
	notifyDone();
}

void QueueTask::run()
{
	CamSysDevice *camSys = manager_->camSys_;

	if (request_) {
		const auto testPatternControl =
			request_->controls().get(controls::draft::TestPatternMode);
		if (testPatternControl) {
			camSys->setTestPattern(
				static_cast<controls::draft::TestPatternModeEnum>(*testPatternControl));
		}
		manager_->onDeviceTuner_->loadTuneRequest(request_->sequence());
	}

	auto &frames = data_->frames;
	auto &camSysRequest = data_->request;

	manager_->statistics0Pool_.fetch(frames.statistics0);
	camSysRequest.statistics0 = frames.statistics0->get().buffer();

	manager_->statistics1Pool_.fetch(frames.statistics1);
	camSysRequest.statistics1 = frames.statistics1->get().buffer();

	manager_->tuningPool_.fetch(frames.tuning);
	camSysRequest.tuning = frames.tuning->get().buffer();

	manager_->mePool_.fetch(frames.me);
	camSysRequest.me = frames.me->get().buffer();

	manager_->faceDetectPool_.fetch(frames.faceDetection);
	camSysRequest.faceDetect = frames.faceDetection->get().buffer();

	manager_->rawPool_.fetch(frames.raw);
	camSysRequest.main = frames.raw->get().buffer();

	manager_->yuvo1Pool_.fetch(frames.yuvo1);
	camSysRequest.yuvo1 = frames.yuvo1->get().buffer();

	manager_->yuvo2Pool_.fetch(frames.yuvo2);
	camSysRequest.yuvo2 = frames.yuvo2->get().buffer();

	camSys->queueRequest(&camSysRequest);

	notifyDone();
}

void DequeueTask::run()
{
	CamSysDevice *camSys = manager_->camSys_;

	/* If request is not finished, register call back when it's done. */
	if (camSys->claimCompletedRequest(&data_->request)) {
		camSys->requestCompleted.connect(this, &DequeueTask::requestReady);
		return;
	}

	done();
}

void DequeueTask::requestReady(CamSysDevice::Request *request)
{
	if (request != &data_->request)
		return;

	CamSysDevice *camSys = manager_->camSys_;

	camSys->requestCompleted.disconnect(this, &DequeueTask::requestReady);
	camSys->claimCompletedRequest(&data_->request);

	done();
}

void DequeueTask::done()
{
	if (request_) {
		FrameBuffer *buffer = (data_->request.main) ? data_->request.main : data_->request.yuvo1;

		ControlList metadata;
		metadata.set(controls::SensorTimestamp,
			     buffer->metadata().timestamp);

		manager_->pipe_->completeMetadata(request_, metadata);
		manager_->onDeviceTuner_->tuneCamsys(request_, data_->frames);

		// TODO: Add another 3ATask to DoCalculation.

		hal3A_->doCalculation(data_->request.statistics0, buffer->metadata().timestamp);
	}

	notifyDone();
}

} /* namespace libcamera */
