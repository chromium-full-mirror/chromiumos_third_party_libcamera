/*
 * Copyright (C) 2023, Google Inc.
 *
 * imgsys.h - MtkISP7 ImgSys device
 */

#pragma once

#include <memory>
#include <string>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/task_scheduler.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "single_device.h"

namespace libcamera {

class DmaHeap;
class PipelineHandler;
class ImgSysDevice;

class ImgsysVideoDevice : public V4L2VideoDevice
{
public:
	using V4L2VideoDevice::V4L2VideoDevice;
	int configure(V4L2DeviceFormat *fmt, int resizeRatio, Rectangle crop);

private:
	int resizeRatio_;
	Rectangle crop_;
};

class ImgSysDevice
{
public:
	static Rectangle getCrop(Size inSize, Size outSize);

	enum FdCtrl {
		Add = 0,
		Delete,
	};

	struct Request {
		SingleDeviceRequest *sdRequest;
		size_t stage;
		int buffers_count;
	};

	ImgSysDevice(OnDeviceTuner *odt);

	int init(MediaDevice *media, DmaHeap *dmaHeap);
	int configure();
	int start();
	int stop();

	int queueRequest(Request *request);
	int queueRequestV4L2(Request *request);
	int claimCompletedRequest(Request *request);

	int handleIova(FdCtrl fdHandle, InfoFramePool &pool);
	int handleKva(FdCtrl fdHandle, InfoFramePool &pool);

	Signal<Request *> requestCompleted;

	TokenPool &syncPool() { return syncPool_; }

private:
	friend class ImgSysRequestHelper;

	int startImgSysBackend();
	void bufferReady(std::pair<FrameBuffer *, int> pair);

	V4L2VideoDevice *sigdevNorm_;
	V4L2VideoDevice *ctrlMeta_;
	std::unique_ptr<V4L2Subdevice> mtkIspDip_;
	std::unordered_map<
		NSCam::NSImgStream::IMG_PORT,
		std::unique_ptr<ImgsysVideoDevice>>
		allVideoDevices_;

	InfoFramePool descPool_;
	InfoFramePool ctrlMetaPool_;

	TokenPool syncPool_;

	Pool<int, UniqueFD> mediaRequestPool_;

	struct PendingRequest {
		Request *request;
		int mediaRequest;
		SharedMailBox<InfoFrame> ctrlMeta;
		SharedMailBox<InfoFrame> singleDevNorm;
	};

	std::list<PendingRequest> pendingRequests_;
	std::list<Request *> completedRequests_;

	MediaDevice *media_;
	DmaHeap *dmaHeap_;
	OnDeviceTuner *onDeviceTuner_;

	void *backEndLibrary_;
};

class ImgSysRequestHelper
{
public:
	ImgSysRequestHelper(Task *task, Request *request, ImgSysDevice *imgSys)
		: task_(task), request_(request), imgSys_(imgSys)
	{
	}

	void queueRequest(SingleDeviceRequest &sdRequest);
	void requestReady(ImgSysDevice::Request *request);

	Task *task_;
	Request *request_;
	ImgSysDevice *imgSys_;

	std::list<ImgSysDevice::Request> imgSysRequests_;
	std::chrono::steady_clock::time_point startTime_;
};

} /* namespace libcamera */
