/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * ipa_delegate.cpp - IPA Delegate to handle signals and callbacks.
 */

#include "ipa_delegate.h"

#include "../face_detect/detector.h"
#include "../hal3a/aaa.h"
#include "pipeline/mtkisp7/halisp/imgsys_task.h"

#include "mtkisp7_ipa_interface.h"

namespace libcamera {
LOG_DEFINE_CATEGORY(IPADelegateMtkISP7)

// Don't disconnect to avoid issues of BoundMethod.
IPADelegate::IPADelegate()
{
	ipa_.AieParseResultReady.connect(this,
					 &IPADelegate::AieParseResultReady);

	ipa_.AAResultReady.connect(this, &IPADelegate::AAResultReady);
	ipa_.AFResultReady.connect(this, &IPADelegate::AFResultReady);

	ipa_.ImgSysMetaTuningDone.connect(this, &IPADelegate::ImgSysMetaTuningDone);
}

int IPADelegate::init(const std::string &model, const int32_t sensorIdx,
		      const std::vector<uint8_t> &eeprom,
		      const std::vector<ipa::mtkisp7::CamSysData> &camSysDataArray)
{
	return ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::init,
				 ConnectionTypeBlocking, model, sensorIdx,
				 eeprom, camSysDataArray);
}

void IPADelegate::start(const uint32_t rawMetaBufferId,
			ipa::mtkisp7::SensorSetting *sensorSetting,
			int32_t *lens_position)
{
	return ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::start,
				 ConnectionTypeBlocking, rawMetaBufferId,
				 sensorSetting, lens_position);
}

void IPADelegate::stop()
{
	return ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::stop,
				 ConnectionTypeBlocking);
}

int IPADelegate::configure(
	const Size &camsysYuvSize, FaceDetector *faceDetector,
	const Size &maxVideoSize,
	const Size &maxStillSize, const std::string &sensorId,
	const uint32_t camsysIndex, const int32_t sessionTimestamp,
	bool isVideo,
	std::vector<uint8_t> *swmeParam,
	std::vector<uint8_t> *bssParam)
{
	faceDetector_ = faceDetector;

	return ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::configure,
				 ConnectionTypeBlocking,
				 camsysYuvSize, maxVideoSize, maxStillSize,
				 sensorId, camsysIndex, sessionTimestamp,
				 isVideo,
				 swmeParam, bssParam);
}

void IPADelegate::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	return ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::mapBuffers,
				 ConnectionTypeBlocking, buffers);
}

void IPADelegate::unmapBuffers(const std::vector<unsigned int> &ids)
{
	return ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::unmapBuffers,
				 ConnectionTypeBlocking, ids);
}

void IPADelegate::writeStillCaptureDebugMetadata(
	const uint32_t camSysMetaRequestId,
	const Feature feature,
	ControlList *metadata)
{
	ipa_.writeStillCaptureDebugMetadata(
		camSysMetaRequestId, static_cast<uint32_t>(feature), metadata);
}

void IPADelegate::notifyRequestBegin(const uint32_t frame,
				     const bool hasStillCapture)
{
	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::notifyRequestBegin,
			  ConnectionTypeQueued, frame, hasStillCapture);
}

void IPADelegate::notifyRequestEnd(const uint32_t frame)
{
	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::notifyRequestEnd,
			  ConnectionTypeQueued, frame);
}

void IPADelegate::notifyExportBegin(
	const uint32_t exportBegin,
	const uint32_t exportEnd)
{
	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::notifyExportBegin,
			  ConnectionTypeQueued, exportBegin, exportEnd);
}

void IPADelegate::notifyImportBegin(
	const uint32_t importBegin,
	const uint32_t importEnd)
{
	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::notifyImportBegin,
			  ConnectionTypeQueued, importBegin, importEnd);
}

void IPADelegate::aieParse(
	const uint32_t inputImageBufferId,
	const uint32_t faceDetectionMetadataBufferId,
	const uint32_t faceToneClassificationMetadataBufferId,
	const Size &currentSensorSize,
	const uint32_t camSysMetaRequestId)
{
	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::aieParse,
			  ConnectionTypeQueued,
			  inputImageBufferId, faceDetectionMetadataBufferId,
			  faceToneClassificationMetadataBufferId,
			  currentSensorSize, camSysMetaRequestId);
}

void IPADelegate::doCalculation3A(
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
	const ControlList &controls)
{
	aaTasks_.emplace(frame, aaTask);
	if (afTask)
		afTasks_.emplace(frame, afTask);

	int32_t featureEnum = -1;
	if (featureApplied.has_value())
		featureEnum = static_cast<int32_t>(featureApplied.value());

	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::doCalculation3A, ConnectionTypeQueued,
			  frame, stat0BufferId, stat1BufferId,
			  timestamp, camSysMetaRequestId,
			  afCamSysMetaRequestId, isStillCapture,
			  rawMetaBufferId, gyroSample,
			  internalRequestIdApplied, featureEnum, vcmFocusInfo,
			  controls);
}

void IPADelegate::getImgSysMetaTuning(
	ImgSysTask *imgSysTask,
	const uint32_t camSysMetaRequestId,
	const uint32_t frame,
	const bool needCropTNC16x9,
	const Feature feature,
	const std::vector<ipa::mtkisp7::ImgMetaRequestData> &imgMetaRequests,
	const ControlList &controls)
{
	uint64_t cookie = imgSysCookieCounter_++;
	imgSysTasks_.emplace(cookie, imgSysTask);

	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::getImgSysMetaTuning,
			  ConnectionTypeQueued,
			  cookie, camSysMetaRequestId, frame,
			  needCropTNC16x9, static_cast<uint32_t>(feature),
			  imgMetaRequests, controls);
}

void IPADelegate::AieParseResultReady(
	bool success,
	const ipa::mtkisp7::PrimaryFaceData &primaryFace,
	const ControlList &faceControls)
{
	faceDetector_->AieParseResultReady(success, primaryFace, faceControls);
}

void IPADelegate::AAResultReady(uint32_t id,
				const ipa::mtkisp7::SensorSetting &sensorSetting,
				const ipa::mtkisp7::AaaIspExchange &aaaIspExchange,
				const ipa::mtkisp7::LensPositionInfo &lensPositionInfo)
{
	auto it = aaTasks_.find(id);
	if (it == aaTasks_.end()) {
		LOG(IPADelegateMtkISP7, Fatal)
			<< "AAResultReady: couldn't find task with id: " << id;
		return;
	}
	it->second->AAResultReady(sensorSetting, aaaIspExchange, lensPositionInfo);

	aaTasks_.erase(it);
}

void IPADelegate::AFResultReady(uint32_t id, int32_t position)
{
	auto it = afTasks_.find(id);
	if (it == afTasks_.end()) {
		LOG(IPADelegateMtkISP7, Fatal)
			<< "AFResultReady: couldn't find task with id: " << id;
		return;
	}
	it->second->AFResultReady(position);

	afTasks_.erase(it);
}

void IPADelegate::ImgSysMetaTuningDone(uint64_t cookie)
{
	auto it = imgSysTasks_.find(cookie);
	if (it == imgSysTasks_.end()) {
		LOG(IPADelegateMtkISP7, Fatal)
			<< "ImgSysMetaTuningDone: couldn't find task with cookie: "
			<< cookie;
		return;
	}
	it->second->notifyDone();

	imgSysTasks_.erase(it);
}

} // namespace libcamera
