/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * fake_ipa.cpp - Fake IPA implementation for MtkISP7
 */

#include "fake_ipa.h"

#include "libcamera/base/bound_method.h"
#include "libcamera/base/log.h"
#include "pipeline/mtkisp7/halisp/hal_isp.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAMtkISP7)

namespace ipa::mtkisp7 {

IPAMtkISP7::IPAMtkISP7()
{
}

int IPAMtkISP7::init(const int32_t sensorIdx)
{
	sensorIdx_ = sensorIdx;
	halIsp_ = std::make_unique<HalIsp>(onDeviceTuner_);
	hal3A_ = std::make_unique<Hal3A>(sensorIdx, halIsp_.get(), onDeviceTuner_);

	int32_t sensorDev = 0;
	switch (sensorIdx) {
	case 0:
		sensorDev = 1;
		break;
	case 1:
		sensorDev = 2;
		break;

	default:
		LOG(IPAMtkISP7, Fatal) << "Unsupported sensorIdx: " << sensorIdx;
		break;
	}
	halIsp_->init(sensorIdx, sensorDev, hal3A_.get());

	aieParser_ = std::make_unique<AieParser>();

	return 0;
}

/**
 * \brief Perform any processing required before the first frame
 */
int IPAMtkISP7::start(const uint32_t rawMetaBufferId)
{
	IPAMappedBuffer *rawMetaBuffer = getMappedBufferIter(rawMetaBufferId);
	if (!rawMetaBuffer) {
		LOG(IPAMtkISP7, Error) << "Could not find rawMeta buffer!";
		return -1;
	}
	MappedFrameBuffer mappedRawMeta(&rawMetaBuffer->buffer,
					MappedFrameBuffer::MapFlag::ReadWrite);

	hal3A_->start(reinterpret_cast<mtk_cam_uapi_meta_raw_stats_cfg *>(
		rawMetaBuffer->mapped->planes()[0].data()));

	aaThread_.start();
	aaManager_ = std::make_unique<AAManager>(this);
	aaManager_->moveToThread(&aaThread_);

	afThread_.start();
	afManager_ = std::make_unique<AFManager>(this);
	afManager_->moveToThread(&afThread_);

	return hal3A_->r3AResult_.af_result.lens_position;
}

/**
 * \brief Ensure that all processing has completed
 */
void IPAMtkISP7::stop()
{
	aaManager_.reset();
	aaThread_.exit();
	aaThread_.wait();

	afManager_.reset();
	afThread_.exit();
	afThread_.wait();
}

/**
 * \brief Configure the MtkISP7 IPA
 * \param[in] sensorIdx The index of the sensor being used now
 */
int IPAMtkISP7::configure(const Size &camsysYuvSize, const Size &maxVideoSize,
			  const Size &maxStillSize, bool isVideo, std::vector<uint8_t> *swmeParam,
			  std::vector<uint8_t> *bssParam)
{
	hal3A_->configure(camsysYuvSize, isVideo);
	halIsp_->configure(maxVideoSize, maxStillSize, isVideo);

	std::shared_ptr<mtk::isphal::v1::isp_swme_Param> swme = halIsp_->getIspSwmeParam();
	const auto swmeSize = sizeof(mtk::isphal::v1::isp_swme_Param);
	swmeParam->resize(swmeSize);
	memcpy(swmeParam->data(), swme.get(), swmeSize);

	std::shared_ptr<mtk::isphal::v1::isp_bss_Param> bss = halIsp_->getIspBssParam();
	const auto bssSize = sizeof(mtk::isphal::v1::isp_bss_Param);
	bssParam->resize(bssSize);
	memcpy(bssParam->data(), bss.get(), bssSize);

	int ret = aieParser_->initialize();
	if (ret != 0) {
		return ret;
	}
	aieParser_->configure();

	return aieParser_->initialize();
}

/**
 * \brief Map the parameters and stats buffers allocated in the pipeline handler
 * \param[in] buffers The buffers to map
 */
void IPAMtkISP7::mapBuffers(const std::vector<IPABuffer> &buffers)
{
	for (const IPABuffer &buffer : buffers) {
		buffers_.emplace(buffer.id, buffer.planes);
	}
}

/**
 * \brief Unmap the parameters and stats buffers
 * \param[in] ids The IDs of the buffers to unmap
 */
void IPAMtkISP7::unmapBuffers(const std::vector<unsigned int> &ids)
{
	for (unsigned int id : ids) {
		auto it = buffers_.find(id);
		if (it == buffers_.end())
			continue;

		buffers_.erase(it);
	}
}

void IPAMtkISP7::aieParse(
	const uint32_t inputImageBufferId,
	const uint32_t faceDetectionMetadataBufferId,
	const uint32_t faceToneClassificationMetadataBufferId,
	const Size &currentSensorSize,
	const uint32_t camSysMetaRequestId)
{
	auto itInputImage = buffers_.find(inputImageBufferId);
	if (itInputImage == buffers_.end()) {
		LOG(IPAMtkISP7, Error) << "Could not find input image buffer!";
		return;
	}
	FrameBuffer *inputBuffer = &itInputImage->second.buffer;

	auto itFDMetadata = buffers_.find(faceDetectionMetadataBufferId);
	if (itFDMetadata == buffers_.end()) {
		LOG(IPAMtkISP7, Error) << "Could not find FD metadata buffer!";
		return;
	}
	FrameBuffer *faceMetatBuffer = &itFDMetadata->second.buffer;

	FrameBuffer *FTCMetadataFrameBuffer = nullptr;
	if (faceToneClassificationMetadataBufferId != 0) {
		auto itFTCMetadata = buffers_.find(faceToneClassificationMetadataBufferId);
		if (itFTCMetadata == buffers_.end()) {
			LOG(IPAMtkISP7, Error) << "Could not find FTC metadata buffer!";
			return;
		}

		FTCMetadataFrameBuffer = &itFTCMetadata->second.buffer;
	}

	PrimaryFaceData faceData;
	ControlList faceControls;
	aieParser_->doParse(inputBuffer, faceMetatBuffer, FTCMetadataFrameBuffer,
			    currentSensorSize, camSysMetaRequestId,
			    faceData, faceControls);
	aieParser_->getLatestOutput(latestFaceMetadata_);

	bool success = true;
	AieParseResultReady.emit(success, faceData, faceControls);
}

void IPAMtkISP7::doCalculation3A(const uint32_t frame,
				 const uint32_t stat0BufferId, const uint32_t stat1BufferId,
				 const uint64_t timestamp, const uint32_t camSysMetaRequestId,
				 const uint32_t afCamSysMetaRequestId,
				 const bool isStillCapture, const uint32_t rawMetaBufferId,
				 const GyroSampleData &gyroSample,
				 const uint32_t internalRequestIdApplied,
				 const int32_t featureEnum,
				 const VcmFocusInformation &vcmFocusInfo,
				 const ControlList &controls)
{
	auto itStat0 = buffers_.find(stat0BufferId);
	if (itStat0 == buffers_.end()) {
		LOG(IPAMtkISP7, Error) << "Could not find stat0 buffer!";
		return;
	}

	IPAMappedBuffer *rawMetaBuffer = getMappedBufferIter(rawMetaBufferId);
	if (!rawMetaBuffer) {
		LOG(IPAMtkISP7, Error) << "Could not find rawMeta buffer!";
		return;
	}

	GyroSensor::SensorSample sample;
	sample.x_value = gyroSample.x_value;
	sample.y_value = gyroSample.y_value;
	sample.z_value = gyroSample.z_value;
	sample.timestamp = gyroSample.timestamp;

	ControlList aaaMetadata;
	aaManager_->invokeMethod(
		&IPAMtkISP7::AAManager::doCalculation, ConnectionTypeQueued,
		&itStat0->second.buffer, timestamp, frame,
		camSysMetaRequestId, isStillCapture,
		rawMetaBuffer->buffer.planes()[0].fd.get(),
		rawMetaBuffer->mapped->planes()[0].data(),
		latestFaceMetadata_, sample, internalRequestIdApplied,
		controls, featureEnum);

	if (stat1BufferId == 0)
		return;

	// TODO: use another thread.
	::VcmFocusInformation vcm;
	vcm.focus_position = vcmFocusInfo.focus_position;
	vcm.previous_focus_position = vcmFocusInfo.previous_focus_position;
	vcm.moving_timestamp = vcmFocusInfo.moving_timestamp;
	vcm.previous_moving_timestamp = vcmFocusInfo.previous_moving_timestamp;

	auto itStat1 = buffers_.find(stat1BufferId);
	if (itStat1 == buffers_.end()) {
		LOG(IPAMtkISP7, Error) << "Could not find stat1 buffer!";
		return;
	}

	afManager_->invokeMethod(
		&IPAMtkISP7::AFManager::doCalculationAF, ConnectionTypeQueued,
		&itStat1->second.buffer, timestamp, frame,
		afCamSysMetaRequestId, vcm,
		latestFaceMetadata_, sample);
}

IPAMtkISP7::IPAMappedBuffer *
IPAMtkISP7::getMappedBufferIter(unsigned int bufferId)
{
	auto it = buffers_.find(bufferId);
	if (it == buffers_.end())
		return nullptr;

	if (!it->second.mapped) {
		it->second.mapped = std::make_unique<MappedFrameBuffer>(
			&it->second.buffer,
			MappedFrameBuffer::MapFlag::ReadWrite);
	}

	return &it->second;
}

void IPAMtkISP7::getImgSysMetaTuning(
	const uint64_t cookie, const uint32_t camSysMetaRequestId,
	const uint32_t frame, const bool needCropTNC16x9,
	const uint32_t featureEnum,
	const std::vector<ipa::mtkisp7::ImgMetaRequestData> &imgMetaRequests)
{
	for (const auto &requestData : imgMetaRequests) {
		std::vector<DmaSyncer> syncers;

		IPAMappedBuffer *tuningBuffer = getMappedBufferIter(requestData.tuningBufferId);
		if (!tuningBuffer) {
			LOG(IPAMtkISP7, Error) << "Could not find tuning buffer!";
			continue;
		}
		syncers.emplace_back(tuningBuffer->buffer.planes()[0].fd.get());

		IPAMappedBuffer *statBuffer = nullptr;
		if (requestData.statisticsBufferId > 0) {
			statBuffer = getMappedBufferIter(requestData.statisticsBufferId);
			if (!statBuffer) {
				LOG(IPAMtkISP7, Error) << "Could not find stat buffer!";
				continue;
			}

			syncers.emplace_back(statBuffer->buffer.planes()[0].fd.get());
		}

		IPAMappedBuffer *swHistBuffer = nullptr;
		if (requestData.swHistBufferId > 0) {
			swHistBuffer = getMappedBufferIter(requestData.swHistBufferId);
			if (!swHistBuffer) {
				LOG(IPAMtkISP7, Error) << "Could not find swHist buffer!";
				continue;
			}
		}

		ImgMetaRequest request = {
			.isCapture = requestData.isCapture,
			.isMfnr = requestData.isMfnr,
			.stage = static_cast<NSIspTuning::EStage_T>(requestData.stage),
			.tuningBuffer = &tuningBuffer->buffer,
			.mappedTuningBuffer = tuningBuffer->mapped.get(),
			.statisticsBuffer = statBuffer ? &statBuffer->buffer : nullptr,
			.mappedStatisticsBuffer = statBuffer ? statBuffer->mapped.get() : nullptr,
			.swHistBuffer = swHistBuffer ? &swHistBuffer->buffer : nullptr,
			.mappedSwHistBuffer = swHistBuffer ? swHistBuffer->mapped.get() : nullptr,
			.inputSize = requestData.inputSize,
			.outputSize = requestData.outputSize,
			.outputSize2 = requestData.outputSize2,
			.fullDipSize = requestData.fullDipSize,
			.tnr_frameIndex = requestData.tnr_frameIndex,
			.tnr_frameTotal = requestData.tnr_frameTotal,
			.reserved = {}
		};

		for (const auto &[keyInt, bufferId] : requestData.reserved) {
			mtk::isphal::kISPExtBuf key = static_cast<mtk::isphal::kISPExtBuf>(keyInt);
			IPAMappedBuffer *mappedBuffer = getMappedBufferIter(bufferId);
			if (!mappedBuffer) {
				LOG(IPAMtkISP7, Error) << "Could not find reserved buffer: " << bufferId;
				continue;
			}
			request.reserved[key] =
				std::make_pair<FrameBuffer *, MappedFrameBuffer *>(
					&mappedBuffer->buffer,
					mappedBuffer->mapped.get());

			switch (key) {
			case mtk::isphal::kISPExtBif_IN_HWME_STAT_FST_MD0:
			case mtk::isphal::kISPExtBif_IN_HWME_STAT_FST_MD1:
			case mtk::isphal::kISPExtBif_IN_HWME_STAT_FMB_MD0:
			case mtk::isphal::kISPExtBif_OUT_FWMM_MIL:
				syncers.emplace_back(mappedBuffer->buffer.planes()[0].fd.get());
				break;
			default:
				break;
			}
		}

		halIsp_->getImgSysMetaTuning(
			camSysMetaRequestId, request, frame,
			requestData.hasFrameNumber
				? requestData.frameNumber
				: frame,
			needCropTNC16x9, static_cast<Feature>(featureEnum));
	}

	ImgSysMetaTuningDone.emit(cookie);
}

IPAMtkISP7::AAManager::AAManager(IPAMtkISP7 *ipa)
	: ipa_(ipa)
{
}

void IPAMtkISP7::AAManager::doCalculation(FrameBuffer *statistics0, uint64_t timestamp,
					  uint32_t internalRequestId,
					  uint32_t camSysMetaRequestId,
					  bool isStillCapture, int rawMetaFd,
					  unsigned char *rawMetaBuffer,
					  std::optional<MtkCameraFaceMetadata> metadata,
					  GyroSensor::SensorSample gyroSample,
					  uint32_t internalRequestIdApplied,
					  const ControlList &controls,
					  const int32_t featureEnum)
{
	SensorSetting exposureAndGain;
	AaaIspExchange aaaIspExchange;
	std::optional<uint32_t> idApplied = std::nullopt;
	if (internalRequestIdApplied != 0)
		idApplied = internalRequestIdApplied;

	std::optional<Feature> featureApplied = std::nullopt;
	if (featureEnum >= 0)
		featureApplied = static_cast<Feature>(featureEnum);
	ipa_->hal3A_->doCalculation(statistics0, timestamp, internalRequestId,
				    camSysMetaRequestId, isStillCapture,
				    rawMetaFd, rawMetaBuffer,
				    metadata, gyroSample,
				    &exposureAndGain, &aaaIspExchange,
				    idApplied, featureApplied,
				    controls);

	ipa_->AAResultReady.emit(internalRequestId, exposureAndGain, aaaIspExchange);

	if (idApplied && featureApplied) {
		ipa_->onDeviceTuner_->tune3AState(
			internalRequestIdApplied,
			statistics0, &ipa_->hal3A_->r3AResult_,
			featureApplied.value());
	}
}

IPAMtkISP7::AFManager::AFManager(IPAMtkISP7 *ipa)
	: ipa_(ipa)
{
}

void IPAMtkISP7::AFManager::doCalculationAF(FrameBuffer *statistics1, uint64_t timestamp,
					    uint32_t internalRequestId, uint32_t camSysMetaRequestId,
					    ::VcmFocusInformation vcmFocusInfo,
					    std::optional<MtkCameraFaceMetadata> metadata,
					    GyroSensor::SensorSample gyroSample)
{
	int32_t position = -1;
	ipa_->hal3A_->doCalculationAF(statistics1, timestamp, internalRequestId,
				      camSysMetaRequestId, vcmFocusInfo,
				      metadata, gyroSample, &position);
	ipa_->AFResultReady.emit(internalRequestId, position);
}

} // namespace ipa::mtkisp7
} // namespace libcamera
