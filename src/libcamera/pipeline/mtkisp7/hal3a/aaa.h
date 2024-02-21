/*
 * Copyright (C) 2023, Google Inc.
 *
 * capture.h - MTK MtkISP7 Hal 3A Manager
 */

#pragma once

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/gyro_sensor.h"
#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/camsys/camsys.h"
#include "pipeline/mtkisp7/camsys/capture.h"
#include "pipeline/mtkisp7/face_detect/detector.h"
#include "pipeline/mtkisp7/fake_ipa/fake_ipa.h"
#include "pipeline/mtkisp7/ipa/ipa_delegate.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

namespace libcamera {

class AATask;
class AFTask;
class MtkISP7CameraData;

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
	void configure(DmaHeap *dmaHeap, CamSysDevice *camSys,
		       GyroSensor *gyroSensor,
		       IPADelegate *ipa);
	int start(int32_t lens_position);

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

	std::pair<uint32_t, SharedMailBox<InfoFrame>> getDummyTuning();

	void setMfnrMode(bool mfnrMode)
	{
		mfnrMode_ = mfnrMode;
	}
	bool getMfnrMode() const
	{
		return mfnrMode_;
	}

private:
	friend MtkISP7CameraData;

	bool hasAF() const;

	DmaHeap *dmaHeap_;
	CamSysDevice *camSys_;
	IPADelegate *ipa_;

	GyroSensor *gyroSensor_;

	FocusController focusController_;

	InfoFramePool tuningPool_;

	uint32_t dummyMetaRequestId_;
	SharedMailBox<InfoFrame> dummyTuning_;

	bool mfnrMode_;
};

// AE & AWB task.
class AATask : public Task
{
public:
	struct PerFrameControl {
		bool isStillCapture = false;
		ControlList controls;
	};

	AATask(Hal3AManager *manager, Scheduler *scheduler, const std::string &id,
	       CaptureFrames &captureFrames, GyroSensor *gyroSensor,
	       IPADelegate *ipa, AFTask *afTask,
	       FocusController *focusController,
	       uint32_t internalRequestId, uint32_t camSysMetaRequestId,
	       FaceDetector *faceDetector)
		: Task(scheduler, id), request_(nullptr), manager_(manager),
		  captureFrames_(captureFrames), gyroSensor_(gyroSensor),
		  ipa_(ipa), afTask_(afTask), focusController_(focusController),
		  internalRequestId_(internalRequestId),
		  camSysMetaRequestId_(camSysMetaRequestId), faceDetector_(faceDetector)
	{
	}

	void setPerFrameControl(PerFrameControl perFrameControl)
	{
		perFrameControl_ = perFrameControl;
	}

	void AAResultReady(ipa::mtkisp7::SensorSetting exposureAndGain);

	void run() override final;

	void setRequest(Request *request);
	void setInternalRequestIdApplied(uint32_t internalRequestIdApplied);
	void setFeatureApplied(Feature feature);

	Request *request_;
	std::optional<uint32_t> internalRequestIdApplied_;
	std::optional<Feature> featureApplied_;

	Hal3AManager *manager_;
	CaptureFrames captureFrames_;

	GyroSensor *gyroSensor_;

	IPADelegate *ipa_;
	AFTask *afTask_;
	FocusController *focusController_;

	uint32_t internalRequestId_;
	uint32_t camSysMetaRequestId_;

	FaceDetector *faceDetector_;

	PerFrameControl perFrameControl_;
};

class AFTask : public Task
{
public:
	constexpr static uint32_t kLensDelay = 3;

	AFTask(Scheduler *scheduler, const std::string &id,
	       CaptureFrames &captureFrames,
	       GyroSensor *gyroSensor, IPADelegate *ipa,
	       uint32_t internalRequestId,
	       FocusController *focusController, FaceDetector *faceDetector)
		: Task(scheduler, id), captureFrames_(captureFrames),
		  gyroSensor_(gyroSensor), ipa_(ipa), internalRequestId_(internalRequestId),
		  focusController_(focusController), faceDetector_(faceDetector)
	{
	}

	void AFResultReady(int32_t position);

	void run() override final;

	CaptureFrames captureFrames_;

	GyroSensor *gyroSensor_;

	IPADelegate *ipa_;

	uint32_t internalRequestId_;

	FocusController *focusController_;
	FaceDetector *faceDetector_;

	bool run_ = false;
	bool executed_ = false;
};

} // namespace libcamera
