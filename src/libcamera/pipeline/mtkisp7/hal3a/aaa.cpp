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
#include "peripheraldriver/lens/vcm_drv.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "hal_3a.h"

namespace libcamera {

namespace {

static constexpr Size kMetaSize = Size{ Hal3A::kRawMetaSize, 1 };

// Todo: Move the funtion to common utils
uint64_t getMonotonicTimestamp()
{
	struct timespec t;
	t.tv_sec = t.tv_nsec = 0;
	clock_gettime(CLOCK_MONOTONIC, &t);
	return (uint64_t)((t.tv_sec) * 1000000000LL + t.tv_nsec);
}

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

	// Do not update moving timestamp if the target position is unchanged
	if (position == focusPosition_)
		return;

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
			     GyroSensor *gyroSensor,
			     IPADelegate *ipa)
{
	dmaHeap_ = dmaHeap;
	camSys_ = camSys;
	gyroSensor_ = gyroSensor;
	ipa_ = ipa;

	focusController_.configure(camSys_->getCameraLens());

	releaseBuffers();

	if (tuningPool_.size() == 0)
		tuningPool_.createBuffers(dmaHeap_, formats::MTFP_MTISP, kMetaSize, 8,
					  DmaHeap::CMA);

	dummyMetaRequestId_ = 0;

	dummyTuning_ = makeMailBox<InfoFrame>();
	fetchTuningBuffer(dummyTuning_);
}

int Hal3AManager::start(int32_t lens_position)
{
	focusController_.set(lens_position, 0);

	return 0;
}

void Hal3AManager::releaseBuffers()
{
	dummyTuning_.reset();
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

	AFTask *afTask = nullptr;
	if (hasAF()) {
		afTask = new AFTask(scheduler, "AF " + sequence, captureFrames,
				    gyroSensor_, ipa_, internalRequestId,
				    &focusController_, faceDetector);
	}

	AATask *aaTask = new AATask(this, scheduler, "3A " + sequence,
				    captureFrames, gyroSensor_,
				    ipa_, afTask, &focusController_,
				    internalRequestId, camSysMetaRequestId,
				    faceDetector);

	return std::make_tuple(aaTask, afTask);
}

std::pair<uint32_t, SharedMailBox<InfoFrame>> Hal3AManager::getDummyTuning()
{
	if (!dummyTuning_)
		LOG(MtkISP7, Fatal) << "Empty dummy tuning buffer";

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

	captureFrames_.aaaIspExchange->put({}, nullptr);

	ipa_->preDoCalculation3A(faceMetadata, &captureFrames_.aaaIspExchange->get());

	ipa::mtkisp7::GyroSampleData gyroSample;
	if (gyroSensor_) {
		GyroSensor::SensorSample sample = gyroSensor_->getLatestSample();
		if (sample.timestamp == 0) {
			LOG(MtkISP7, Error) << "Gyro not found";
		} else {
			gyroSample.x_value = sample.x_value;
			gyroSample.y_value = sample.y_value;
			gyroSample.z_value = sample.z_value;
			gyroSample.timestamp = sample.timestamp;
		}
	}

	ipa::mtkisp7::VcmFocusInformation vcm;

	vcm.focus_position = focusController_->getFocusInfo().focus_position;
	vcm.previous_focus_position = focusController_->getFocusInfo().previous_focus_position;
	vcm.moving_timestamp = focusController_->getFocusInfo().moving_timestamp;
	vcm.previous_moving_timestamp = focusController_->getFocusInfo().previous_moving_timestamp;
	ipa_->doCalculation3A(
		this, afTask_, internalRequestId_,
		captureFrames_.statistics0->get().buffer()->cookie(),
		afTask_ ? captureFrames_.statistics1->get().buffer()->cookie() : 0,
		captureFrames_.timestamp->get(), camSysMetaRequestId_,
		internalRequestId_ - AFTask::kLensDelay,
		perFrameControl_.isStillCapture,
		captureFrames_.tuningOutput->get().buffer()->cookie(),
		gyroSample, internalRequestIdApplied_.value_or(0),
		featureApplied_, vcm,
		perFrameControl_.controls);
}

void AATask::AAResultReady(ipa::mtkisp7::SensorSetting exposureAndGain)
{
	captureFrames_.exposureAndGainOutput->put(exposureAndGain, nullptr);

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
	internalRequestIdApplied_ = internalRequestIdApplied;
}

void AATask::setFeatureApplied(Feature featureApplied)
{
	featureApplied_ = featureApplied;
}

void AFTask::run()
{
	run_ = true;
	if (executed_)
		notifyDone();
}

void AFTask::AFResultReady(int32_t position)
{
	uint64_t timestamp = getMonotonicTimestamp();
	focusController_->set(position, timestamp / 1000);

	executed_ = true;
	if (run_)
		notifyDone();
}

} // namespace libcamera
