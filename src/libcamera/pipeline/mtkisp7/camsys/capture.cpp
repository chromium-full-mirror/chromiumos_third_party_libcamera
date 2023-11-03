/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * capture.cpp - MTK MtkISP7 Camsys Device Capture Tasks
 */

#include "capture.h"
#include <cstdint>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>

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
				   const Size &yuvFrameSize)
{
	dmaHeap_ = dmaHeap;
	camSys_ = camSys;
	pipe_ = pipe;

	rawFrameSize_ = rawFrameSize;
	yuvFrameSize_ = yuvFrameSize;

	releaseBuffers();
	allocateBuffers();

	return 0;
}

void CaptureTasksManager::allocateBuffers()
{
	rawPool_.createBuffers(dmaHeap_, camSys_->bayerFormat(), rawFrameSize_, 12);
	yuvo1Pool_.createBuffers(dmaHeap_, formats::NV12_10P_MTISP, yuvFrameSize_, 12);
	yuvo2Pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, yuvFrameSize_ / 2, 12);
	mePool_.createBuffers(dmaHeap_, formats::GREY, kMeSize, 12, DmaHeap::System, 64);
	faceDetectPool_.createBuffers(dmaHeap_, formats::NV12, kFdSize, 12);
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

	statistics0Pool_.release();
	statistics1Pool_.release();
}

void CaptureTasksManager::makeCaptureFrames(CaptureFrames &captureFrames)
{
	captureFrames.tuningOutput = makeMailBox<InfoFrame>();

	captureFrames.raw = makeMailBox<InfoFrame>();
	captureFrames.yuvo1 = makeMailBox<InfoFrame>();
	captureFrames.yuvo2 = makeMailBox<InfoFrame>();

	captureFrames.me = makeMailBox<InfoFrame>();
	captureFrames.faceDetection = makeMailBox<InfoFrame>();

	captureFrames.statistics0 = makeMailBox<InfoFrame>();
	captureFrames.statistics1 = makeMailBox<InfoFrame>();

	captureFrames.timestamp = makeMailBox<uint64_t>();
	captureFrames.exposureAndGainOutput = makeMailBox<std::pair<uint32_t, uint32_t>>();

	captureFrames.aaaIspExchange = makeMailBox<AaaIspExchange>();
}

std::tuple<QueueTask *, DequeueTask *, SofTask *>
CaptureTasksManager::makeCaptureTasks(Scheduler *scheduler,
				      const std::string &id,
				      Request *request,
				      CaptureFrames &captureFrames,
				      uint32_t camSysMetaRequestId)
{
	(void)id;

	std::string sequence = "padding";
	if (request)
		sequence = std::to_string(request->sequence());

	onDeviceTuner_->notifyRequestBegin(camSysMetaRequestId);

	// Create CaptureData after CaptureFrames SharedMailBoxes are set.
	auto data = std::make_shared<CaptureData>(captureFrames);

	SofTask *sofTask = new SofTask(scheduler, "Sof " + sequence, request, data, camSys_);

	QueueTask *qTask = new QueueTask(this, scheduler, "Queue " + sequence, request, data);
	DequeueTask *dqTask = new DequeueTask(this, scheduler, "Dequeue " + sequence, request, data);

	return std::make_tuple(qTask, dqTask, sofTask);
}

void SofTask::run()
{
	run_ = true;
	if (run_ && trigger_) {
		LOG(MtkISP7, Warning) << "Sof Task triggered before run()."
				      << " Some previous tasks may delay the Sof task";
		notifyDone();
	}
}

void SofTask::trigger()
{
	if (run_) { // Avoid race condition of AATask (in another thread) and SofTask.
		if (!data_->frames.exposureAndGain->valid()) {
			LOG(MtkISP7, Fatal) << "No exposureAndGain despite SofTask being run";
		} else {
			auto [exposure, gain] = data_->frames.exposureAndGain->get();
			if (exposure != 0) // Assuming it couldn't be zero.
				camSys_->setExposureGain(exposure, gain);
			LOG(MtkISP7, Info) << "exposure: " << exposure << ", gain: " << gain;
		}
	} else {
		LOG(MtkISP7, Error) << "SharedMailBox exposureAndGain not "
				    << "set yet. Skip setting exposure and gain.";
	}

	trigger_ = true;
	if (run_ && trigger_) {
		notifyDone();
	}
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
	}

	auto &frames = data_->frames;
	auto &camSysRequest = data_->request;

	manager_->statistics0Pool_.fetch(frames.statistics0);
	camSysRequest.statistics0 = frames.statistics0->get().buffer();

	manager_->statistics1Pool_.fetch(frames.statistics1);
	camSysRequest.statistics1 = frames.statistics1->get().buffer();

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
	FrameBuffer *buffer = (data_->request.main) ? data_->request.main : data_->request.yuvo1;
	uint64_t timestamp = buffer->metadata().timestamp;

	data_->frames.timestamp->put(buffer->metadata().timestamp,
				     []([[maybe_unused]] uint64_t &timestamp) {});

	if (request_) {
		ControlList metadata;
		metadata.set(controls::SensorTimestamp, timestamp);

		manager_->pipe_->completeMetadata(request_, metadata);
		manager_->onDeviceTuner_->tuneCamsys(request_, data_->frames);
	}

	notifyDone();
}

} /* namespace libcamera */
