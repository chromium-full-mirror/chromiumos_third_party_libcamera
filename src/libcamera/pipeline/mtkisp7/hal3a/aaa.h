/*
 * Copyright (C) 2023, Google Inc.
 *
 * capture.h - MTK MtkISP7 Hal 3A Manager
 */

#pragma once

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/task_scheduler.h"

#include "libcamera/base/thread.h"
#include "libfdft_lib/faces.h"
#include "pipeline/mtkisp7/camsys/camsys.h"
#include "pipeline/mtkisp7/camsys/capture.h"
#include "pipeline/mtkisp7/face_detect/detector.h"

#include "hal_3a.h"

namespace libcamera {

class AATask;
class AFTask;

class FocusController
{
public:
	void configure(CameraLens *cameraLens);

	VcmFocusInformation getFocusInfo();

	void set(int32_t position, int64_t timestamp);

private:
	void reset();
	bool isFirstRun();

	CameraLens *cameraLens_;
	bool firstRun_ = true;

	int32_t focusPosition_;
	int32_t previousFocusPosition_;
	int64_t movingTimestamp_;
	int64_t previousMovingTimestamp_;
};

class Hal3AManager
{
public:
	void configure(DmaHeap *dmaHeap, CamSysDevice *camSys, Hal3A *hal3A);
	void start();

	void releaseBuffers();

	std::tuple<AATask *, AFTask *>
	make3ATasks(Scheduler *scheduler, Request *request,
		    CaptureFrames &captureFrames,
		    uint32_t internalRequestId, uint32_t camSysMetaRequestId,
		    FaceDetector *faceDetector);

	void fetchTuningBuffer(SharedMailBox<InfoFrame> &mailBox)
	{
		tuningPool_.fetch(mailBox);
	}

	SharedMailBox<InfoFrame> getDummyTuning();

private:
	bool hasAF() const;
	void allocateBuffers();

	DmaHeap *dmaHeap_;
	CamSysDevice *camSys_;
	Hal3A *hal3A_;

	FocusController focusController_;

	InfoFramePool tuningPool_;

	Thread thread3A_;
	Thread threadAF_;

	SharedMailBox<InfoFrame> dummyTuning_;
};

// AE & AWB task.
class AATask : public Task
{
public:
	struct PerFrameControl {
		bool isStillCapture = false;
	};

	MtkCameraFaceMetadata *prevFaceMetadata_ = nullptr;

	AATask(Hal3AManager *manager, Scheduler *scheduler, const std::string &id,
	       CaptureFrames &captureFrames, Hal3A *hal3A,
	       uint32_t internalRequestId, uint32_t camSysMetaRequestId,
	       FaceDetector *faceDetector)
		: Task(scheduler, id), request_(nullptr), manager_(manager),
		  captureFrames_(captureFrames), hal3A_(hal3A),
		  internalRequestId_(internalRequestId),
		  camSysMetaRequestId_(camSysMetaRequestId), faceDetector_(faceDetector) {}

	void setPerFrameControl(PerFrameControl perFrameControl)
	{
		perFrameControl_ = perFrameControl;
	}

	void run() override final;

	void setRequest(Request *request);

	Request *request_;

	Hal3AManager *manager_;
	CaptureFrames captureFrames_;

	Hal3A *hal3A_;

	uint32_t internalRequestId_;
	uint32_t camSysMetaRequestId_;

	FaceDetector *faceDetector_;

	PerFrameControl perFrameControl_;
};

class AFTask : public Task
{
public:
	MtkCameraFaceMetadata *prevFaceMetadata_ = nullptr;

	AFTask(Scheduler *scheduler, const std::string &id,
	       CaptureFrames &captureFrames, Hal3A *hal3A,
	       uint32_t internalRequestId, uint32_t camSysMetaRequestId,
	       FocusController *focusController, FaceDetector *faceDetector)
		: Task(scheduler, id), captureFrames_(captureFrames),
		  hal3A_(hal3A), internalRequestId_(internalRequestId),
		  camSysMetaRequestId_(camSysMetaRequestId),
		  focusController_(focusController), faceDetector_(faceDetector) {}

	void run() override final;

	CaptureFrames captureFrames_;

	Hal3A *hal3A_;

	uint32_t internalRequestId_;
	uint32_t camSysMetaRequestId_;

	FocusController *focusController_;
	FaceDetector *faceDetector_;
};

} // namespace libcamera
