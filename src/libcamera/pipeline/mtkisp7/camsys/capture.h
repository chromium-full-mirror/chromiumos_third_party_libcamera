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
	SharedMailBox<InfoFrame> tuning;
};

class CaptureTasksManager
{
public:
	CaptureTasksManager(OnDeviceTuner *odt);
	~CaptureTasksManager() = default;

	int configure(DmaHeap *dmaHeap, CamSysDevice *camSys, PipelineHandler *pipe,
		      const Size &rawFrameSize, const Size &yuvFrameSize, Hal3A *hal3A);

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

	Hal3A *hal3A_;

	InfoFramePool tuningPool_;
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

class CaptureData
{
public:
	CaptureData(CaptureFrames &captureFrames)
		: frames(captureFrames) {}

	CamSysDevice::Request request;
	CaptureFrames frames;
};

class SofTask : public Task
{
public:
	SofTask(Scheduler *scheduler, const std::string &id, Request *request,
		CamSysDevice *camSys, Hal3A *hal3A)
		: Task(scheduler, id), request_(request), camSys_(camSys),
		  hal3A_(hal3A) {}

	virtual void run() override final {}
	void trigger();

	Request *request_;

	CamSysDevice *camSys_;
	Hal3A *hal3A_;
};

class QueueTask : public Task
{
public:
	QueueTask(CaptureTasksManager *manager,
		  Scheduler *scheduler, const std::string &id, Request *request,
		  std::shared_ptr<CaptureData> &data)
		: Task(scheduler, id), request_(request), manager_(manager),
		  data_(data)
	{
	}

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
		    std::shared_ptr<CaptureData> &data, Hal3A *hal3A)
		: Task(scheduler, id), request_(request), manager_(manager),
		  data_(data), hal3A_(hal3A)
	{
	}

	void run() override final;
	void done();
	void requestReady(CamSysDevice::Request *request);

	Request *request_;
	CaptureTasksManager *manager_;

	std::shared_ptr<CaptureData> data_;

	Hal3A *hal3A_;
};

} /* namespace libcamera */
