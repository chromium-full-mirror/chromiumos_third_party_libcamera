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

int IPADelegate::init(const int32_t sensorIdx)
{
	return ipa_.init(sensorIdx);
}

int IPADelegate::start(const uint32_t rawMetaBufferId)
{
	return ipa_.start(rawMetaBufferId);
}

void IPADelegate::stop()
{
	ipa_.stop();
}

int IPADelegate::configure(
	const Size &camsysYuvSize, FaceDetector *faceDetector,
	const Size &maxVideoSize,
	const Size &maxStillSize,
	bool isVideo,
	std::vector<uint8_t> *swmeParam,
	std::vector<uint8_t> *bssParam)
{
	faceDetector_ = faceDetector;

	return ipa_.configure(camsysYuvSize, maxVideoSize, maxStillSize,
			      isVideo, swmeParam, bssParam);
}

void IPADelegate::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	ipa_.mapBuffers(buffers);
}

void IPADelegate::unmapBuffers(const std::vector<unsigned int> &ids)
{
	ipa_.unmapBuffers(ids);
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
	const std::vector<ipa::mtkisp7::ImgMetaRequestData> &imgMetaRequests)
{
	uint64_t cookie = imgSysCookieCounter_++;
	imgSysTasks_.emplace(cookie, imgSysTask);

	ipa_.invokeMethod(&ipa::mtkisp7::IPAMtkISP7::getImgSysMetaTuning,
			  ConnectionTypeQueued,
			  cookie, camSysMetaRequestId, frame,
			  needCropTNC16x9, static_cast<uint32_t>(feature),
			  imgMetaRequests);
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
				const ipa::mtkisp7::AaaIspExchange &aaaIspExchange)
{
	auto it = aaTasks_.find(id);
	if (it == aaTasks_.end()) {
		LOG(IPADelegateMtkISP7, Fatal)
			<< "AAResultReady: couldn't find task with id: " << id;
		return;
	}
	it->second->AAResultReady(sensorSetting, aaaIspExchange);

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
