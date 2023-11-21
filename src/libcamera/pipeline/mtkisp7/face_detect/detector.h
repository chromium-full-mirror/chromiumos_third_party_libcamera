/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * detector.h - Run face detection.
 */
#pragma once

#include <memory>
#include <tuple>

#include <libcamera/controls.h>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/task_scheduler.h"

#include "libfdft_lib/faces.h"
#include "pipeline/mtkisp7/face_detect/aie.h"
#include "pipeline/mtkisp7/face_detect/parser.h"

namespace libcamera {

class FaceDetector
{
public:
	using FaceDetectionTasks =
		std::tuple<AieDevice::AieTask *,
			   AieDevice::AieTask *,
			   AieParseTask *>;

	FaceDetector(AieDevice *aieDev);

	bool canMakeFaceDetectionTask(Request *request);
	int configure(const Size &currentSensorSize);
	int getOutputLookbackStep(uint32_t requestNum);
	SharedMailBox<MtkCameraFaceMetadata> getOutputMailBox(
		uint32_t requestNum);
	FaceDetectionTasks makeFaceDetectionTask(
		Scheduler *scheduler, Request *request,
		SharedMailBox<InfoFrame> detectorInput);
	int start();
	int stop();

private:
	AieDevice *aieDev_;
	const uint32_t period_;
	const uint32_t expectedFrameLatency_;

	std::shared_ptr<AieParser> parser_;

	SharedMailBox<MtkCameraFaceMetadata> prevOutput_;
	SharedMailBox<MtkCameraFaceMetadata> latestOutput_;

	SharedMailBox<FdDrv_input_struct> faceToneConfig_;

	Size currentSensorSize_;
};

} /* namespace libcamera */
