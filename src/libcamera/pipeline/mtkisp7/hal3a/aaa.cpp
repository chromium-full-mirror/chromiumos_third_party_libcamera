/*
 * Copyright (C) 2023, Google Inc.
 *
 * capture.h - MTK MtkISP7 Hal 3A Manager
 */

#include "aaa.h"

#include <libcamera/formats.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "libcamera/request.h"
#include "libfdft_lib/faces.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "hal_3a.h"

namespace libcamera {

namespace {

static constexpr Size kMetaSize = Size{ Hal3A::kRawMetaSize, 1 };

} // namespace

LOG_DECLARE_CATEGORY(MtkISP7)

void FocusController::configure(CameraLens *cameraLens)
{
	cameraLens_ = cameraLens;
	reset();
}

void FocusController::reset()
{
	firstRun_ = true;

	focusPosition_ = 0;
	previousFocusPosition_ = 0;
	movingTimestamp_ = 0;
	previousMovingTimestamp_ = 0;
}

VcmFocusInformation FocusController::getFocusInfo()
{
	VcmFocusInformation info;
	info.focus_position = focusPosition_;
	info.previous_focus_position = previousFocusPosition_;
	info.moving_timestamp = movingTimestamp_;
	info.previous_moving_timestamp = previousMovingTimestamp_;
	LOG(MtkISP7, Debug) << "getFocusInfo. focus position: " << focusPosition_
			    << ", previous focus position: " << previousFocusPosition_
			    << ", moving timestamp: " << movingTimestamp_
			    << ", previous moving timestamp: " << previousMovingTimestamp_;

	return info;
}

void FocusController::set(int32_t position, int64_t timestamp)
{
	if (position < 0 || position == focusPosition_)
		return;

	if (isFirstRun()) {
		cameraLens_->setFocusPosition(
			position == 0 ? 1 : position - 1);
	}
	cameraLens_->setFocusPosition(position);

	previousFocusPosition_ = focusPosition_;
	previousMovingTimestamp_ = movingTimestamp_;
	focusPosition_ = position;
	movingTimestamp_ = timestamp;
}

bool FocusController::isFirstRun()
{
	bool firstRun = firstRun_;
	firstRun_ = false;
	return firstRun;
}

void Hal3AManager::configure(DmaHeap *dmaHeap, CamSysDevice *camSys,
			     Hal3A *hal3A, OnDeviceTuner *odt,
			     GyroSensor *gyroSensor)
{
	dmaHeap_ = dmaHeap;
	camSys_ = camSys;
	hal3A_ = hal3A;
	onDeviceTuner_ = odt;
	gyroSensor_ = gyroSensor;

	focusController_.configure(camSys_->getCameraLens());

	if (tuningPool_.size() == 0)
		tuningPool_.createBuffers(dmaHeap_, formats::MTFP_MTISP, kMetaSize, 8,
					  DmaHeap::CMA);

	releaseBuffers();
	allocateBuffers();
}

void Hal3AManager::start()
{
	focusController_.set(hal3A_->r3AResult_.af_result.lens_position, 0);
}

void Hal3AManager::allocateBuffers()
{
	thread3A_.start();
	threadAF_.start();
}

void Hal3AManager::releaseBuffers()
{
	dummyTuning_.reset();

	thread3A_.exit();
	thread3A_.wait();

	threadAF_.exit();
	threadAF_.wait();
}

bool Hal3AManager::hasAF() const
{
	if (!camSys_)
		LOG(MtkISP7, Fatal) << "CamSysDevice hasn't been configured yet.";

	return camSys_->getCameraLens();
}

std::tuple<AATask *, AFTask *> Hal3AManager::make3ATasks(
	Scheduler *scheduler, Request *request,
	CaptureFrames &captureFrames, uint32_t internalRequestId,
	uint32_t camSysMetaRequestId,
	FaceDetector *faceDetector)
{
	std::string sequence = "padding";
	if (request)
		sequence = std::to_string(request->sequence());

	// Update the dummyTuning and the related camSysMetaRequestId
	dummyMetaRequestId_ = camSysMetaRequestId;
	dummyTuning_ = captureFrames.tuning;

	AATask *aaTask = new AATask(this, scheduler, "3A " + sequence,
				    captureFrames, hal3A_,
				    onDeviceTuner_, gyroSensor_,
				    internalRequestId, camSysMetaRequestId,
				    faceDetector);
	aaTask->moveToThread(&thread3A_);

	AFTask *afTask;
	if (hasAF()) {
		afTask = new AFTask(scheduler, "AF " + sequence, captureFrames,
				    hal3A_, gyroSensor_, internalRequestId,
				    &focusController_, faceDetector);
		afTask->moveToThread(&threadAF_);
	}

	return std::make_tuple(aaTask, afTask);
}

std::pair<uint32_t, SharedMailBox<InfoFrame>> Hal3AManager::getDummyTuning()
{
	if (dummyTuning_)
		return std::make_pair(dummyMetaRequestId_, dummyTuning_);

	dummyMetaRequestId_ = 0;

	dummyTuning_ = makeMailBox<InfoFrame>();
	fetchTuningBuffer(dummyTuning_);
	FrameBuffer *tuningBuffer = dummyTuning_->get().buffer();

	MappedFrameBuffer mappedBuffer(tuningBuffer,
				       MappedFrameBuffer::MapFlag::ReadWrite);
	tuningBuffer->_d()->metadata().planes()[0].bytesused = tuningBuffer->planes()[0].length;

	// TODO: replace directly using raw_meta
	// TODO: Check if we need to call getCamSysMetaTuning
	memcpy(mappedBuffer.planes()[0].data(),
	       &hal3A_->r3AResult_.raw_meta, Hal3A::kRawMetaSize);

	return std::make_pair(dummyMetaRequestId_, dummyTuning_);
}

void AATask::run()
{
	manager_->fetchTuningBuffer(captureFrames_.tuningOutput);

	FrameBuffer *tuningBuffer = captureFrames_.tuningOutput->get().buffer();
	MappedFrameBuffer mappedBuffer(tuningBuffer,
				       MappedFrameBuffer::MapFlag::ReadWrite);
	tuningBuffer->_d()->metadata().planes()[0].bytesused =
		tuningBuffer->planes()[0].length;

	std::optional<MtkCameraFaceMetadata> faceMetadata;
	faceDetector_->getLatestOutput(faceMetadata);
	MtkCameraFaceMetadata *faces = (faceMetadata) ? &(*faceMetadata) : nullptr;

	captureFrames_.aaaIspExchange->put({}, nullptr);

	GyroSensor::SensorSample gyroSample;
	if (gyroSensor_) {
		gyroSample = gyroSensor_->getLatestSample();
		if (gyroSample.timestamp == 0)
			LOG(MtkISP7, Error) << "Gyro not found";
	}

	std::pair<uint32_t, uint32_t> exposureAndGain;
	hal3A_->doCalculation(captureFrames_.statistics0->get().buffer(),
			      captureFrames_.timestamp->get(),
			      internalRequestId_, camSysMetaRequestId_,
			      perFrameControl_.isStillCapture,
			      tuningBuffer->planes()[0].fd.get(),
			      mappedBuffer.planes()[0].data(),
			      faces, gyroSample,
			      &exposureAndGain,
			      &captureFrames_.aaaIspExchange->get(),
			      internalRequestIdApplied_);
	captureFrames_.exposureAndGainOutput->put(
		std::move(exposureAndGain),
		[]([[maybe_unused]] std::pair<uint32_t, uint32_t>
			   &exposureAndGain) {});

	if (internalRequestIdApplied_) {
		onDeviceTuner_->tune3AState(
			internalRequestIdApplied_.value(),
			captureFrames_, &hal3A_->r3AResult_);
	}

	manager_->setMfnrMode(captureFrames_.aaaIspExchange->get().mfnrMode);

	notifyDone();
}

/**
 * \brief Set the related application request
 * \param[in] cfg The request coming from application layer
 *
 * For dummy frames, this function will never be called.
 */
void AATask::setRequest(Request *request)
{
	request_ = request;
}

void AATask::setInternalRequestIdApplied(uint32_t internalRequestIdApplied)
{
	ASSERT(!internalRequestIdApplied_);
	internalRequestIdApplied_ = internalRequestIdApplied;
}

void AFTask::run()
{
	int32_t position = -1;

	std::optional<MtkCameraFaceMetadata> faceMetadata;
	faceDetector_->getLatestOutput(faceMetadata);
	MtkCameraFaceMetadata *faces = (faceMetadata) ? &(*faceMetadata) : nullptr;

	GyroSensor::SensorSample gyroSample;
	if (gyroSensor_)
		gyroSample = gyroSensor_->getLatestSample();

	hal3A_->doCalculationAF(captureFrames_.statistics1->get().buffer(),
				captureFrames_.timestamp->get(),
				internalRequestId_,
				internalRequestId_ - kLensDelay,
				focusController_->getFocusInfo(),
				faces, gyroSample, &position);

	focusController_->set(position, captureFrames_.timestamp->get());

	notifyDone();
}

} // namespace libcamera
