/*
 * Copyright (C) 2023, Google Inc.
 *
 * hal_3a.h - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#pragma once

#include <cstddef>

#include <libcamera/ipa/mtkisp7_ipa_interface.h>

#include "libcamera/internal/gyro_sensor.h"

#include "../sensor/sensor_info.h"
#include "../utils/history.h"
#include "libcamera/base/mutex.h"
#include "libcamera/controls.h"
#include "libcamera/framebuffer.h"
#include "libcamera/geometry.h"
#include "mtkcam-core/aaa/peripheralcontroller/include/IPeripheralController.h"
#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/IHal3A.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

class SensorInfo;

namespace libcamera {

class HalIsp;
struct AaaIspExchange;

class Hal3A
{
public:
	static const uint32_t kRawMetaSize = 113664;

	Hal3A(const uint32_t sensor_idx, HalIsp *halIsp, OnDeviceTuner *odt);

	void configure(Size camsysYuvSize, bool isVideo);
	void start(mtk_cam_uapi_meta_raw_stats_cfg *rawMetaBuffer);

	void doCalculation(FrameBuffer *statistics0, uint64_t timestamp,
			   uint32_t internalRequestId, uint32_t camSysMetaRequestId,
			   bool isStillCapture, int rawMetaFd, unsigned char *rawMetaBuffer,
			   MtkCameraFaceMetadata *metadata,
			   GyroSensor::SensorSample gyroSample,
			   ipa::mtkisp7::SensorSetting *exposureAndGain,
			   AaaIspExchange *aaaIspExchange,
			   std::optional<uint32_t> internalRequestIdApplied,
			   std::optional<Feature> featureApplied,
			   ControlList controls);

	void doCalculationAF(FrameBuffer *statistics1, uint64_t timestamp,
			     uint32_t internalRequestId, uint32_t camSysMetaRequestId,
			     VcmFocusInformation vcmFocusInfo,
			     MtkCameraFaceMetadata *metadata,
			     GyroSensor::SensorSample gyroSample, int32_t *position);

	mtk::hal3a::v1_0::mtk_3a_result r3AResult_ = {};

	History<mtk::hal3a::v1_0::mtk_3a_result> resultHistory_;

private:
	void init();
	void getInitialInfo();
	void config();
	void startInternal();

	mtk::hal3a::v1_0::mtk_3a_param get3AParam(uint32_t internalRequestId,
						  MtkCameraFaceMetadata *faceMetadata,
						  GyroSensor::SensorSample gyroSample,
						  bool isStillCapture,
						  std::optional<ControlList> controls_opt);

	void getExposureAndGain(ipa::mtkisp7::SensorSetting *exposureAndGain,
				uint32_t &exposureTimeMs);

	const uint32_t sensor_idx_;
	int sensor_id_;
	Size camsysYuvSize_;
	bool isVideo_ = false;

	mtk::hal3a::IHal3A *m_hal3a_ = nullptr;
	HalIsp *halIsp_ = nullptr;

	mtk::hal3a::v1_0::mtk_hw_initial_setting initialSetting_ = {};

	OnDeviceTuner *onDeviceTuner_;
	std::shared_ptr<SensorInfo> sensor_info_ = nullptr;
};

} /* namespace libcamera */
