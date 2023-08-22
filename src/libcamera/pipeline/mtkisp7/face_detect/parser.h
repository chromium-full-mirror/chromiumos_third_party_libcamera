/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * parser.h - MtkISP7 AIE device output parser
 */
#pragma once

#include <memory>

#include <libcamera/base/signal.h>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/task_scheduler.h"

#include "libfdft_lib/MTKDetection.h"
#include "libfdft_lib/faces.h"
#include "mtkcam-core/hw/aie/3.1/hardware/v4l2/cam_fdvt_v4l2.h"

namespace libcamera {

class AieParseTask : public Task
{
public:
	AieParseTask(
		Scheduler *scheduler, const std::string &id,
		std::shared_ptr<MTKDetection> algoInterface,
		SharedMailBox<InfoFrame> mailBoxInputImage,
		SharedMailBox<InfoFrame> mailBoxMetadata,
		SharedMailBox<MtkCameraFaceMetadata> mailBoxOutput,
		const Size &currentSensorSize);

	void run() override;

private:
	FdOptions createBufferOptions() const;
	void init();
	int prepareBuffer();

	int parseAll();
	int parseFaceDetectionOutput();
	void parseFaceLandmark(
		FDRESULT *resultSet, int calibrationIndex, int resultSetIndex);
	void parseFaceRoi(
		FDRESULT *resultSet, int calibrationIndex, int resultSetIndex);

	void transformAllDetectionCoordinates(
		MtkCameraFaceMetadata &faceMetadata) const;
	void transformDetectionCoordinate(int32_t &x, int32_t &y) const;

	std::shared_ptr<MTKDetection> algoInterface_;
	SharedMailBox<InfoFrame> mailBoxInputImage_;
	SharedMailBox<InfoFrame> mailBoxMetadata_;
	SharedMailBox<MtkCameraFaceMetadata> mailBoxOutput_;
	const Size currentSensorSize_;
	std::unique_ptr<MappedFrameBuffer> currentMappedImageBuffer_;
	fd_cal_struct *algoCalibration_;
};

} /* namespace libcamera */
