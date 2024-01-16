/*
 * Copyright (C) 2023, Google Inc.
 *
 * hal_3a.h - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#pragma once

#include "../halisp/hal_isp.h"
#include "../sensor/sensor_info.h"
#include "libcamera/framebuffer.h"
#include "libcamera/geometry.h"
#include "mtkcam-core/aaa/peripheralcontroller/include/IPeripheralController.h"
#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/IHal3A.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

class SensorInfo;

namespace libcamera {

class Hal3A
{
public:
	static const uint32_t kRawMetaSize = 113664;

	Hal3A(const uint32_t sensor_idx, HalIsp *halIsp, OnDeviceTuner *odt);

	void configure(Size camsysYuvSize) { camsysYuvSize_ = camsysYuvSize; }
	void start();

	void doCalculation(FrameBuffer *statistics0, uint64_t timestamp,
			   uint32_t internalRequestId, uint32_t camSysMetaRequestId,
			   bool isStillCapture, int rawMetaFd, unsigned char *rawMetaBuffer,
			   MtkCameraFaceMetadata *metadata,
			   std::pair<uint32_t, uint32_t> *exposureAndGain,
			   AaaIspExchange *aaaIspExchange, Request *request);

	void doCalculationAF(FrameBuffer *statistics1, uint64_t timestamp,
			     uint32_t internalRequestId, uint32_t camSysMetaRequestId,
			     VcmFocusInformation vcmFocusInfo,
			     MtkCameraFaceMetadata *metadata, int32_t *position);

	mtk::hal3a::v1_0::mtk_3a_result r3AResult_ = {};

private:
	void init();
	void getInitialInfo();
	void config();
	void startInternal();

	mtk::hal3a::v1_0::mtk_3a_param get3AParam(uint32_t internalRequestId,
						  MtkCameraFaceMetadata *faceMetadata,
						  bool isAF = false,
						  bool isStillCapture = false);

	void getExposureAndGain(std::pair<uint32_t, uint32_t> *exposureAndGain);

	const uint32_t sensor_idx_;
	int sensor_id_;
	Size camsysYuvSize_;

	mtk::hal3a::IHal3A *m_hal3a_ = nullptr;
	HalIsp *halIsp_ = nullptr;

	mtk::hal3a::v1_0::mtk_hw_initial_setting initialSetting_ = {};

	OnDeviceTuner *onDeviceTuner_;
	std::shared_ptr<SensorInfo> sensor_info_ = nullptr;
};

} /* namespace libcamera */
