/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * detector.cpp - Run face detection.
 */

#include "detector.h"

#include <memory>
#include <utility>

#include <libcamera/control_ids.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"

#include "mtkcam-core/hw/aie/3.1/hardware/v4l2/cam_fdvt_v4l2.h"

/**
 * \file pipeline/mtkisp7/face_detect/detector.h
 * \brief Manages face detection for MTKISP7
 */

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

/**
 * \var FaceDetector::period_
 * \brief How many frames between face detection run.
 *
 * If \a period is 15 then the face detection should run
 * on frame 0, 15, 30, and so on.
 */

/**
 * \var FaceDetector::expectedFrameLatency_
 * \brief Expected duration (in frame count) of one face detection run.
 *
 * If face detection is executed on frame N, then only from frame
 * N + \a kFaceDetectionFrameLatency we should start using the result.
 */

/**
 * \class FaceDetector
 * \brief Delegates face detector hardware/algorithm lifecycle and configuration
 *
 * This \a FaceDetector is using MediaTek AIE device and using MediaTek
 * face detection algorithm library to parse the result of the device.
 */
FaceDetector::FaceDetector(AieDevice *aieDev)
	: aieDev_(aieDev), period_(15),
	  expectedFrameLatency_(2), parser_(std::make_shared<AieParser>())
{
}

bool FaceDetector::canMakeFaceDetectionTask(Request *request)
{
	return request->sequence() % period_ == 0;
}

int FaceDetector::configure(const Size &currentSensorSize)
{
	currentSensorSize_ = currentSensorSize;
	prevOutput_ = makeMailBox<MtkCameraFaceMetadata>();
	latestOutput_ = makeMailBox<MtkCameraFaceMetadata>();
	faceToneConfig_ = makeMailBox<FdDrv_input_struct>();
	// todo(yerlandinata, IPC sandboxing):
	// 	Reset AieParser in the sandbox process every time configure(),
	//	to clear face coordinates cache when switching camera.
	int ret = parser_->initialize();
	if (ret != 0) {
		return ret;
	}
	return aieDev_->configure();
}

/**
 * @brief Tells which face detection task should be the dependency
 * \param[in] requestNum request number
 *
 * If latest face detection was run at frame N with \a expectedFrameLatency_ = k
 * but \a requestNum < N + k, then the request should depend on previous face
 * detection task (executed at frame N - \a period). Otherwise the request
 * should depend on the latest face detection task.
 *
 * \return how far to look back, 0 means current
 */
int FaceDetector::getOutputLookbackStep(uint32_t requestNum)
{
	if (requestNum % period_ < expectedFrameLatency_) {
		return 1;
	} else {
		return 0;
	}
}

/**
 * @brief Returns the mailbox of face detection result
 * \param[in] requestNum request number
 *
 * \return the mailbox, should check FaceDetector::getOutputLookbackStep
 *	   to make sure it's not empty.
 */
SharedMailBox<MtkCameraFaceMetadata> FaceDetector::getOutputMailBox(
	uint32_t requestNum)
{
	if (requestNum < expectedFrameLatency_) {
		// There is no "previous" output at this moment.
		return nullptr;
	} else if (requestNum % period_ < expectedFrameLatency_) {
		return prevOutput_;
	} else {
		return latestOutput_;
	}
}

FaceDetector::FaceDetectionTasks
FaceDetector::makeFaceDetectionTask(
	Scheduler *scheduler, Request *request,
	SharedMailBox<InfoFrame> detectorInput)
{
	prevOutput_ = std::move(latestOutput_);
	latestOutput_ = makeMailBox<MtkCameraFaceMetadata>();

	auto requestNum = std::to_string(request->sequence());
	const std::string fdTaskId = "AieFaceDetectionTask#" + requestNum;
	const std::string faceToneTaskId = "AieFaceToneClassificationTask#" +
					   requestNum;
	const std::string parseTaskId = "AieParseTask#" + requestNum;
	SharedMailBox<InfoFrame> unparsedFaceDetectionMailBox = makeMailBox<InfoFrame>();
	SharedMailBox<InfoFrame> unparsedFaceToneMailBox = makeMailBox<InfoFrame>();
	FdDrv_input_struct faceDetectDriverConfig(
		aieDev_->createFaceDetectionDriverConfig());

	SharedMailBox<FdDrv_input_struct> mailBoxFaceDetectionConfig =
		makeMailBox<FdDrv_input_struct>();
	mailBoxFaceDetectionConfig->put(faceDetectDriverConfig,
					[](FdDrv_input_struct &) {});

	AieDevice::AieTask *fdTask = new AieDevice::AieTask(
		scheduler, fdTaskId, aieDev_, detectorInput,
		unparsedFaceDetectionMailBox, mailBoxFaceDetectionConfig);
	AieDevice::AieTask *faceToneTask = new AieDevice::AieTask(
		scheduler, faceToneTaskId, aieDev_, detectorInput,
		unparsedFaceToneMailBox, std::move(faceToneConfig_));
	faceToneConfig_ = makeMailBox<FdDrv_input_struct>();
	AieParseTask *parseTask =
		new AieParseTask(scheduler, parseTaskId, parser_,
				 detectorInput,
				 unparsedFaceDetectionMailBox,
				 unparsedFaceToneMailBox,
				 faceToneConfig_,
				 latestOutput_,
				 aieDev_->createFaceToneClassificationDriverConfig(),
				 currentSensorSize_);
	return std::make_tuple(fdTask, faceToneTask, parseTask);
}

int FaceDetector::start()
{
	return aieDev_->start();
}

int FaceDetector::stop()
{
	return aieDev_->stop();
}

} /* namespace libcamera */
