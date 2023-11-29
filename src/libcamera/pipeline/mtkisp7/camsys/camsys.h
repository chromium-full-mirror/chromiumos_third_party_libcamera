/*
 * Copyright (C) 2022, Google Inc.
 *
 * camsys.h - MtkISP7 Camsys device
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/base/signal.h>

#include "libcamera/internal/camera_lens.h"
#include "libcamera/internal/camera_sensor.h"
#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/v4l2_subdevice.h"
#include "libcamera/internal/v4l2_videodevice.h"

namespace libcamera {

class CamSysDevice
{
public:
	static constexpr unsigned int kBufferCount = 8;
	static constexpr unsigned int kMaxBufferQueued = 5;
	static constexpr Size kMinResolution = Size{ 320, 240 };

	struct Request {
		FrameBuffer *main;
		FrameBuffer *yuvo1;
		FrameBuffer *yuvo2;

		FrameBuffer *me;
		FrameBuffer *faceDetect;
		FrameBuffer *tuning;
		FrameBuffer *statistics0;
		FrameBuffer *statistics1;

		int mediaRequest;
	};

	CamSysDevice();

	int init(MediaDevice *media, unsigned int index);

	int start();
	int stop();
	void close();

	int configure(const Size &rawFrameSize, const Size &yuvFrameSize);

	int claimCompletedRequest(Request *request);
	int queueRequest(Request *request);

	int setFrameInterval(uint32_t numerator, uint32_t denominator);
	int setTestPattern(controls::draft::TestPatternModeEnum mode);
	int setExposureGain(uint32_t exposure, uint32_t gain);

	unsigned int getIndex() { return index_; }

	Signal<uint32_t> &frameStart() { return videoHub_->frameStart; }
	Signal<Request *> requestCompleted;

	const ControlList &properties() const { return sensor_->properties(); }

	const PixelFormat bayerFormat() { return bayerFormat_; }
	const std::string &cameraId() { return sensor_->id(); }
	CameraLens *getCameraLens() { return sensor_->focusLens(); }

	const Size rawFrameSize() { return rawFrameSize_; }
	const Size yuvFrameSize() { return yuvFrameSize_; }

private:
	int initSensor(MediaEntity *seninfEntity);

	int setupLinks(bool enable);

	int setFormat(V4L2Subdevice *device, int pad, uint32_t mbus_code, Size size);
	int setFormat(V4L2VideoDevice *device, const PixelFormat &format, Size size);

	int configureSensor();
	int configureMtkCamRaw();
	int configureVideo(V4L2VideoDevice *device, const PixelFormat &format, Size resolution,
			   V4L2Subdevice *rawPipe, int pad, uint32_t mbus);

	int setupResource();

	void bufferReady(std::pair<FrameBuffer *, int> bufferWithRequest);

	Size rawFrameSize_;
	Size yuvFrameSize_;

	unsigned index_;

	unsigned int mbusCode_;
	PixelFormat bayerFormat_;

	MediaDevice *media_;

	std::unique_ptr<V4L2Subdevice> videoHub_;
	std::unique_ptr<V4L2Subdevice> seninf_;
	std::unique_ptr<CameraSensor> sensor_;

	std::unique_ptr<V4L2VideoDevice> metaInput_;
	std::unique_ptr<V4L2VideoDevice> rawi2_;

	std::unique_ptr<V4L2VideoDevice> mainStream_;
	std::unique_ptr<V4L2VideoDevice> yuvo1_;
	std::unique_ptr<V4L2VideoDevice> yuvo2_;
	std::unique_ptr<V4L2VideoDevice> yuvo3_;
	std::unique_ptr<V4L2VideoDevice> yuvo4_;
	std::unique_ptr<V4L2VideoDevice> yuvo5_;

	std::unique_ptr<V4L2VideoDevice> drzs4no1_;
	std::unique_ptr<V4L2VideoDevice> drzs4no2_;
	std::unique_ptr<V4L2VideoDevice> drzs4no3_;
	std::unique_ptr<V4L2VideoDevice> rzh1n2to1_;
	std::unique_ptr<V4L2VideoDevice> rzh1n2to2_;
	std::unique_ptr<V4L2VideoDevice> rzh1n2to3_;
	std::unique_ptr<V4L2VideoDevice> svImgOut1_;
	std::unique_ptr<V4L2VideoDevice> svImgOut2_;

	std::unique_ptr<V4L2VideoDevice> partialMeta0_;
	std::unique_ptr<V4L2VideoDevice> partialMeta1_;
	std::unique_ptr<V4L2VideoDevice> partialMeta2_;
	std::unique_ptr<V4L2VideoDevice> extMeta0_;
	std::unique_ptr<V4L2VideoDevice> extMeta1_;
	std::unique_ptr<V4L2VideoDevice> extMeta2_;

	std::vector<V4L2VideoDevice *> allVideoDevices_;

	Pool<int, UniqueFD> mediaRequestPool_;

	struct PendingRequest {
		Request *request;
		int mediaRequest;
		unsigned int pending;
	};

	std::list<PendingRequest> pendingRequests_;
	std::list<Request *> completedRequests_;
};

} /* namespace libcamera */
