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

struct PrimaryFaceData
{
	uint32_t x1;
	uint32_t y1;
	uint32_t x2;
	uint32_t y2;
	uint32_t padding_left;
	uint32_t padding_right;
	uint32_t padding_up;
	uint32_t padding_down;
};

class AieParser
{
public:
	int initialize();
	void configure();

	int aieParse(FrameBuffer *input,
		     FrameBuffer *faceResult,
		     FrameBuffer *toneResult,
		     Size currentSensorSize,
		     uint32_t camSysMetaRequestId);

	void setLatestOutput(const MtkCameraFaceMetadata &output);
	void getLatestOutput(std::optional<MtkCameraFaceMetadata> &latest);

	Signal<bool, const PrimaryFaceData &, const ControlList &> AieParseResultReady;

private:
	void initParse();

	int doParse(FrameBuffer *input, FrameBuffer *faceResult,
		    FrameBuffer *toneResult, Size currentSensorSize,
		    uint32_t camSysMetaRequestId,
		    PrimaryFaceData &faceToneRoi,
		    ControlList &out);

	int parseAll(FrameBuffer *faceResult, FrameBuffer *toneResult,
		     uint32_t camSysMetaRequestId,
		     PrimaryFaceData &faceToneRoi, ControlList &out);

	int parseFaceDetectionOutput(FrameBuffer *faceResult);
	void parseFaceLandmark(FDRESULT *resultSet, int calibrationIndex, int resultSetIndex);
	void parseFaceRoi(FDRESULT *resultSet, int calibrationIndex, int resultSetIndex);
	int parseFaceToneClassificationOutput(FrameBuffer *toneResult);

	void updateFaceToneDriverConfig(PrimaryFaceData &faceToneRoi);
	void transformDetectionCoordinate(int32_t &x, int32_t &y) const;
	void transformAllDetectionCoordinates(
			MtkCameraFaceMetadata &faceMetadata) const;

	void convertFaceMetadata(MtkCameraFaceMetadata *faceMetadata, ControlList &out);

	FdOptions createBufferOptions() const;

	MUINT8 *getWorkingBuffer();
	bool isValid();

	uint16_t parserBuffers_[MAX_CROP_NUM][8];
	uint16_t *parserBufferList_[MAX_CROP_NUM];
	unsigned char parserTaskList_[MAX_AIE_ATTR_TYPE][MAX_CROP_NUM];
	unsigned char parserBufferStatus_[MAX_AIE_ATTR_TYPE][MAX_CROP_NUM];
	int patchSize_[MAX_CROP_NUM];
	int parserAttributeTask_[MAX_AIE_ATTR_TYPE] = { 0 };

	result workingBuffer_[MAX_FACE_NUM];
	int16_t rawFaceToneResult_[MAX_CROP_NUM][MAX_AIE2_ATT_LEN];

	Size currentSensorSize_;
	fd_cal_struct *algoCalibration_;
	std::unique_ptr<MTKDetection> algoInterface;

	Mutex lock_;
	std::optional<MtkCameraFaceMetadata> latestOutput_;
};

} /* namespace libcamera */
