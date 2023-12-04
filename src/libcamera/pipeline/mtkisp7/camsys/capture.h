/*
 * Copyright (C) 2022, Google Inc.
 *
 * capture.h - MTK MtkISP7 Camsys Device Capture Tasks
 */

#pragma once

#include <memory>

#include <libcamera/base/signal.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/hal3a/hal_3a.h"
#include "pipeline/mtkisp7/halisp/hal_isp.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "camsys.h"

namespace libcamera {

class CamSysDevice;
class DequeueTask;
class DmaHeap;
class PipelineHandler;
class QueueTask;
class SofTask;

struct CaptureFrames {
	SharedMailBox<InfoFrame> raw;
	SharedMailBox<InfoFrame> yuvo1;
	SharedMailBox<InfoFrame> yuvo2;
	SharedMailBox<InfoFrame> me;
	SharedMailBox<InfoFrame> faceDetection;
	SharedMailBox<InfoFrame> statistics0;
	SharedMailBox<InfoFrame> statistics1;
	SharedMailBox<InfoFrame> tuning; // Input
	SharedMailBox<InfoFrame> tuningOutput; // Output

	SharedMailBox<uint64_t> timestamp;

	SharedMailBox<std::pair<uint32_t, uint32_t>> exposureAndGain; // input
	SharedMailBox<std::pair<uint32_t, uint32_t>> exposureAndGainOutput; // output

	SharedMailBox<AaaIspExchange> aaaIspExchange;
};

class CaptureData
{
public:
	CaptureData(CaptureFrames &captureFrames)
		: frames(captureFrames) {}

	CamSysDevice::Request request;
	CaptureFrames frames;
};

class CaptureTasksManager
{
public:
	// TODO: Assume (k-2)th 3A task is done when kth Sof task is triggered by hardware.
	static const uint32_t kExposureAndGainDelay = 2;
	// TODO: Currently fix (k-4)th 3A task to prepare for kth request's raw meta.
	static const uint32_t kRawMetaDelay = 4;

	CaptureTasksManager(OnDeviceTuner *odt);
	CaptureTasksManager() = default;
	~CaptureTasksManager() = default;

	int configure(DmaHeap *dmaHeap, CamSysDevice *camSys, PipelineHandler *pipe,
		      const Size &rawFrameSize, const Size &yuvFrameSize);

	void allocateBuffers();
	void releaseBuffers();

	void makeCaptureFrames(CaptureFrames &captureFrames);

	std::tuple<QueueTask *, DequeueTask *, SofTask *>
	makeCaptureTasks(Scheduler *scheduler, const std::string &id,
			 Request *request, CaptureFrames &captureFrames);

private:
	friend QueueTask;
	friend DequeueTask;

	Size rawFrameSize_;
	Size yuvFrameSize_;

	CamSysDevice *camSys_;
	PipelineHandler *pipe_;
	DmaHeap *dmaHeap_;
	OnDeviceTuner *onDeviceTuner_;

	InfoFramePool rawPool_;
	InfoFramePool yuvo1Pool_;
	InfoFramePool yuvo2Pool_;
	InfoFramePool yuvo3Pool_;
	InfoFramePool yuvo4Pool_;
	InfoFramePool mePool_;
	InfoFramePool faceDetectPool_;
	InfoFramePool statistics0Pool_;
	InfoFramePool statistics1Pool_;
};

class SofTask : public Task
{
public:
	SofTask(Scheduler *scheduler, const std::string &id, Request *request,
		std::shared_ptr<CaptureData> &data, CamSysDevice *camSys)
		: Task(scheduler, id), request_(request), data_(data),
		  camSys_(camSys) {}

	virtual void run() override final;
	void trigger();

	Request *request_;
	std::shared_ptr<CaptureData> data_;

	CamSysDevice *camSys_;

	bool run_ = false;
	bool trigger_ = false;
};

class QueueTask : public Task
{
public:
	QueueTask(CaptureTasksManager *manager,
		  Scheduler *scheduler, const std::string &id, Request *request,
		  std::shared_ptr<CaptureData> &data)
		: Task(scheduler, id), request_(request), manager_(manager),
		  data_(data) {}

	void run() override final;

	Request *request_;
	CaptureTasksManager *manager_;
	std::shared_ptr<CaptureData> data_;
};

class DequeueTask : public Task
{
public:
	DequeueTask(CaptureTasksManager *manager,
		    Scheduler *scheduler, const std::string &id, Request *request,
		    std::shared_ptr<CaptureData> &data)
		: Task(scheduler, id), request_(request), manager_(manager),
		  data_(data) {}

	void run() override final;
	void done();
	void requestReady(CamSysDevice::Request *request);

	Request *request_;
	CaptureTasksManager *manager_;

	std::shared_ptr<CaptureData> data_;
};

} /* namespace libcamera */
