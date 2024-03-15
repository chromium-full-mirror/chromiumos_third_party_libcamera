/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * ipa_delegate.h - IPA Delegate to handle signals and callbacks.
 */
#pragma once

#include "libcamera/base/object.h"
#include "pipeline/mtkisp7/fake_ipa/fake_ipa.h"

namespace libcamera {

class AATask;
class AFTask;
class MtkISP7CameraData;

class IPADelegate : public Object
{
public:
	IPADelegate();

	int init(const int32_t sensorIdx);

	int start(const uint32_t rawMetaBufferId);
	void stop();

	int configure(const Size &camsysYuvSize, FaceDetector *faceDetector, bool isVideo);

	void mapBuffers(const std::vector<IPABuffer> &buffers);
	void unmapBuffers(const std::vector<unsigned int> &ids);

	void aieParse(
		const uint32_t inputImageBufferId,
		const uint32_t faceDetectionMetadataBufferId,
		const uint32_t faceToneClassificationMetadataBufferId,
		const Size &currentSensorSize,
		const uint32_t camSysMetaRequestId);

	void doCalculation3A(
		AATask *aaTask, AFTask *afTask,
		const uint32_t frame,
		const uint32_t stat0BufferId, const uint32_t stat1BufferId,
		const uint64_t timestamp, const uint32_t camSysMetaRequestId,
		const uint32_t afCamSysMetaRequestId,
		const bool isStillCapture, const uint32_t rawMetaBufferId,
		const ipa::mtkisp7::GyroSampleData &gyroSample,
		const uint32_t internalRequestIdApplied,
		std::optional<Feature> featureApplied,
		const ipa::mtkisp7::VcmFocusInformation &vcmFocusInfo,
		const ControlList &controls);

	// Workaround
	Hal3A *getHal3A() { return ipa_.hal3A_.get(); }
	void preInit(HalIsp *halIsp, OnDeviceTuner *odt)
	{
		ipa_.preInit(halIsp, odt);
	}

private:
	friend MtkISP7CameraData;

	void AieParseResultReady(
		bool success,
		const ipa::mtkisp7::PrimaryFaceData &primaryFace,
		const ControlList &faceControls);

	void AAResultReady(uint32_t id,
			   const ipa::mtkisp7::SensorSetting &sensorSetting,
			   const ipa::mtkisp7::AaaIspExchange &aaaIspExchange);
	void AFResultReady(uint32_t id, int32_t position);

	ipa::mtkisp7::IPAMtkISP7 ipa_;

	FaceDetector *faceDetector_;

	std::unordered_map<uint32_t, AATask *> aaTasks_;
	std::unordered_map<uint32_t, AFTask *> afTasks_;
};

} // namespace libcamera
