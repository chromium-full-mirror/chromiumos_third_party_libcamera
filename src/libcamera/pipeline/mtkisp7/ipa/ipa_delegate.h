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
class ImgSysTask;

class IPADelegate : public Object
{
public:
	IPADelegate();

	int init(const std::string &model, const int32_t sensorIdx,
		 const std::vector<uint8_t> &eeprom,
		 const std::vector<ipa::mtkisp7::CamSysData> &camSysDataArray);

	int start(const uint32_t rawMetaBufferId);
	void stop();

	int configure(const Size &camsysYuvSize, FaceDetector *faceDetector,
		      const Size &maxVideoSize,
		      const Size &maxStillSize, const std::string &sensorId,
		      const uint32_t camsysIndex, const int32_t sessionTimestamp,
		      bool isVideo,
		      std::vector<uint8_t> *swmeParam,
		      std::vector<uint8_t> *bssParam);

	void mapBuffers(const std::vector<IPABuffer> &buffers);
	void unmapBuffers(const std::vector<unsigned int> &ids);

	void writeStillCaptureDebugMetadata(
		const uint32_t camSysMetaRequestId,
		const Feature feature,
		ControlList *metadata);

	void notifyRequestBegin(const uint32_t frame,
				const bool hasStillCapture);
	void notifyRequestEnd(const uint32_t frame);

	void notifyExportBegin(
		const uint32_t exportBegin,
		const uint32_t exportEnd);
	void notifyImportBegin(
		const uint32_t importBegin,
		const uint32_t importEnd);

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

	void getImgSysMetaTuning(
		ImgSysTask *imgSysTask,
		const uint32_t camSysMetaRequestId,
		const uint32_t frame,
		const bool needCropTNC16x9,
		const Feature feature,
		const std::vector<ipa::mtkisp7::ImgMetaRequestData> &imgMetaRequests,
		const ControlList &controls);

private:
	friend MtkISP7CameraData;

	void AieParseResultReady(
		bool success,
		const ipa::mtkisp7::PrimaryFaceData &primaryFace,
		const ControlList &faceControls);

	void AAResultReady(uint32_t id,
			   const ipa::mtkisp7::SensorSetting &sensorSetting,
			   const ipa::mtkisp7::AaaIspExchange &aaaIspExchange,
			   const ipa::mtkisp7::LensPositionInfo &lensPositionInfo);
	void AFResultReady(uint32_t id, int32_t position);

	void ImgSysMetaTuningDone(uint64_t cookie);

	ipa::mtkisp7::IPAMtkISP7 ipa_;

	FaceDetector *faceDetector_;

	std::unordered_map<uint32_t, AATask *> aaTasks_;
	std::unordered_map<uint32_t, AFTask *> afTasks_;

	uint64_t imgSysCookieCounter_ = 1;
	std::unordered_map<uint64_t, ImgSysTask *> imgSysTasks_;
};

} // namespace libcamera
