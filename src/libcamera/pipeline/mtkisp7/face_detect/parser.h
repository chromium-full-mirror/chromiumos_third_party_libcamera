/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * parser.h - MtkISP7 AIE device output parser
 */
#pragma once

#include <memory>

#include <libcamera/base/signal.h>

#include <libcamera/request.h>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/task_scheduler.h"

#include "libfdft_lib/MTKDetection.h"
#include "libfdft_lib/faces.h"
#include "mtkcam-core/hw/aie/3.1/hardware/v4l2/cam_fdvt_v4l2.h"

namespace libcamera {

class FaceDetector;

class AieParser
{
public:
	MUINT8 *getWorkingBuffer();
	bool isValid();
	int initialize();

	std::unique_ptr<MTKDetection> algoInterface;

	int16_t rawFaceToneResult_[MAX_CROP_NUM][MAX_AIE2_ATT_LEN];
	uint16_t parserBuffers_[MAX_CROP_NUM][8];
	uint16_t *parserBufferList_[MAX_CROP_NUM];
	unsigned char parserTaskList_[MAX_AIE_ATTR_TYPE][MAX_CROP_NUM];
	unsigned char parserBufferStatus_[MAX_AIE_ATTR_TYPE][MAX_CROP_NUM];
	int patchSize_[MAX_CROP_NUM];
	int parserAttributeTask_[MAX_AIE_ATTR_TYPE] = { 0 };
	result workingBuffer_[MAX_FACE_NUM];
};

class AieParseTask : public Task
{
public:
	AieParseTask(
		Scheduler *scheduler, const std::string &id,
		std::shared_ptr<AieParser> parser,
		SharedMailBox<InfoFrame> mailBoxInputImage,
		SharedMailBox<InfoFrame> mailBoxFaceDetectionMetadata,
		SharedMailBox<InfoFrame> mailBoxFaceToneClassificationMetadata,
		SharedMailBox<FdDrv_input_struct> mailBoxFaceToneConfig,
		FaceDetector *faceDetector,
		SharedMailBox<MtkCameraFaceMetadata> mailBoxOutput,
		const FdDrv_input_struct &defaultFaceToneConfig,
		const Size &currentSensorSize);

	void run() override;

private:
	FdOptions createBufferOptions() const;
	void init();
	int prepareBuffer();

	void updateFaceToneDriverConfig();

	int parseAll();
	int parseFaceDetectionOutput();
	void parseFaceLandmark(
		FDRESULT *resultSet, int calibrationIndex, int resultSetIndex);
	void parseFaceRoi(
		FDRESULT *resultSet, int calibrationIndex, int resultSetIndex);
	int parseFaceToneClassificationOutput();

	void transformAllDetectionCoordinates(
		MtkCameraFaceMetadata &faceMetadata) const;
	void transformDetectionCoordinate(int32_t &x, int32_t &y) const;

	std::shared_ptr<AieParser> parser_;
	SharedMailBox<InfoFrame> mailBoxInputImage_;
	SharedMailBox<InfoFrame> mailBoxFaceDetectionMetadata_;
	SharedMailBox<InfoFrame> mailBoxFaceToneClassificationMetadata_;
	SharedMailBox<FdDrv_input_struct> mailBoxFaceToneConfig_;
	FaceDetector *faceDetector_;
	SharedMailBox<MtkCameraFaceMetadata> mailBoxOutput_;
	const Size currentSensorSize_;
	std::unique_ptr<MappedFrameBuffer> currentMappedImageBuffer_;
	fd_cal_struct *algoCalibration_;
	const FdDrv_input_struct defaultFaceToneDriverConfig_;
};

} /* namespace libcamera */
