/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * fake_ipa.cpp - Fake IPA implementation for MtkISP7
 */

#include "fake_ipa.h"

#include "libcamera/base/bound_method.h"
#include "libcamera/base/log.h"
#include "libcamera/controls.h"
#include "pipeline/mtkisp7/halisp/hal_isp.h"
#include "platform/mtkisp7/cam_cal_helper.h"
#include "platform/mtkisp7/platform_utils.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAMtkISP7)

namespace ipa::mtkisp7 {

namespace {
// TODO(chenghaoyang): Set a big core according to models.
// Ciri's big cores are CPU 6 and 7.
static const std::vector<int> k3AThreadCpuAffinity{ 6, 7 };
static const std::vector<int> kIspThreadCpuAffinity{ 6, 7 };
} // namespace

IPAMtkISP7::IPAMtkISP7()
{
}

int IPAMtkISP7::init(const std::string &model, const int32_t sensorIdx,
		     const std::vector<uint8_t> &eeprom,
		     const std::vector<ipa::mtkisp7::CamSysData> &camSysDataArray)
{
	PlatformUtils::setWithModelName(model);
	CamCalHelper::getInstance(sensorIdx)->setEepromData(eeprom);

	if (PlatformUtils::platform_ == PlatformUtils::MtkISP7Platform::NONE)
		LOG(MtkISP7, Fatal) << "Invalid model: " << model;

	LOG(MtkISP7, Debug)
		<< "Running on platform "
		<< PlatformUtils::enumToString(PlatformUtils::platform_)
		<< ", model: " << model;

	std::vector<SensorInfo::CamSysData> dataArray;
	for (const auto &data : camSysDataArray) {
		dataArray.emplace_back(data.has_af, data.mbus_code);
	}
	SensorInfo::add_sensor(dataArray);

	sensorIdx_ = sensorIdx;
	halIsp_ = std::make_unique<HalIsp>(&onDeviceTuner_);
	hal3A_ = std::make_unique<Hal3A>(sensorIdx, halIsp_.get(), &onDeviceTuner_);

	onDeviceTuner_.initialize(true);

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
void IPAMtkISP7::start(const uint32_t rawMetaBufferId,
		       SensorSetting *sensorSetting,
		       int32_t *lens_position)
{
	IPAMappedBuffer *rawMetaBuffer = getMappedBufferIter(rawMetaBufferId);
	if (!rawMetaBuffer) {
		LOG(IPAMtkISP7, Error) << "Could not find rawMeta buffer!";
		*lens_position = -1;
		return;
	}
	MappedFrameBuffer mappedRawMeta(&rawMetaBuffer->buffer,
					MappedFrameBuffer::MapFlag::ReadWrite);

	hal3A_->start(reinterpret_cast<mtk_cam_uapi_meta_raw_stats_cfg *>(
		rawMetaBuffer->mapped->planes()[0].data()));

	aaThread_.start();
	aaThread_.setThreadAffinity(k3AThreadCpuAffinity);
	aaManager_ = std::make_unique<AAManager>(this);
	aaManager_->moveToThread(&aaThread_);

	afManager_ = std::make_unique<AFManager>(this);
	afManager_->moveToThread(&aaThread_);

	ispThread_.start();
	ispThread_.setThreadAffinity(kIspThreadCpuAffinity);
	ispManager_ = std::make_unique<IspManager>(this);
	ispManager_->moveToThread(&ispThread_);

	uint32_t exposureTimeMs;
	hal3A_->getExposureAndGain(sensorSetting, exposureTimeMs);
	*lens_position = hal3A_->r3AResult_.af_result.lens_position;
}

/**
 * \brief Ensure that all processing has completed
 */
void IPAMtkISP7::stop()
{
	aaManager_.reset();
	afManager_.reset();
	aaThread_.exit();
	aaThread_.wait();

	ispManager_.reset();
	ispThread_.exit();
	ispThread_.wait();
}

/**
 * \brief Configure the MtkISP7 IPA
 * \param[in] sensorIdx The index of the sensor being used now
 */
int IPAMtkISP7::configure(const Size &camsysYuvSize, const Size &maxVideoSize,
			  const Size &maxStillSize, const std::string &sensorId,
			  const uint32_t camsysIndex, const int32_t sessionTimestamp,
			  bool isVideo,
			  std::vector<uint8_t> *swmeParam,
			  std::vector<uint8_t> *bssParam)
{
	ImagiqAdapter::sensorIdMap.emplace(
		sensorId, NSCam::TuningUtils::eSensorId(sensorIdx_));

	onDeviceTuner_.configure(sensorId, camsysIndex, sessionTimestamp);

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

void IPAMtkISP7::writeStillCaptureDebugMetadata(
	const uint32_t camSysMetaRequestId,
	const uint32_t featureEnum,
	ControlList *metadata)
{
	*metadata = controls::controls;
	onDeviceTuner_.writeStillCaptureDebugMetadata(*metadata,
						      hal3A_->resultHistory_.query(camSysMetaRequestId),
						      static_cast<Feature>(featureEnum));
}

void IPAMtkISP7::notifyRequestBegin(const uint32_t frame,
				    const bool hasStillCapture)
{
	onDeviceTuner_.notifyRequestBegin(frame);

	if (!hasStillCapture)
		onDeviceTuner_.notifyVideoOnly(frame);
}

void IPAMtkISP7::notifyRequestEnd(const uint32_t frame)
{
	onDeviceTuner_.notifyRequestEnd(frame);
}

void IPAMtkISP7::notifyExportBegin(const uint32_t exportBegin,
				   const uint32_t exportEnd)
{
	onDeviceTuner_.notifyExportBegin(exportBegin, exportEnd);
}

void IPAMtkISP7::notifyImportBegin(const uint32_t importBegin,
				   const uint32_t importEnd)
{
	onDeviceTuner_.notifyImportBegin(importBegin, importEnd);
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
	ControlList faceControls(controls::controls);
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

	if (stat1BufferId) {
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
		// TODO: merge doCalculationAF into aaManager
		afManager_->invokeMethod(
			&IPAMtkISP7::AFManager::doCalculationAF, ConnectionTypeQueued,
			&itStat1->second.buffer, timestamp, frame,
			afCamSysMetaRequestId, vcm,
			latestFaceMetadata_, sample, controls);
	}

	ControlList aaaMetadata;
	aaManager_->invokeMethod(
		&IPAMtkISP7::AAManager::doCalculation, ConnectionTypeQueued,
		&itStat0->second.buffer, timestamp, frame,
		camSysMetaRequestId, isStillCapture,
		rawMetaBuffer->buffer.planes()[0].fd.get(),
		rawMetaBuffer->mapped->planes()[0].data(),
		latestFaceMetadata_, sample, internalRequestIdApplied,
		controls, featureEnum);
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
	const std::vector<ipa::mtkisp7::ImgMetaRequestData> &imgMetaRequests,
	const ControlList &controls)
{
	std::vector<IPAMtkISP7::IspManager::DataMappedBuffers>
		dataMappedBuffersList(imgMetaRequests.size());

	for (uint32_t i = 0; i < imgMetaRequests.size(); ++i) {
		const auto &requestData = imgMetaRequests[i];
		auto &dataMappedBuffers = dataMappedBuffersList[i];
		dataMappedBuffers.valid = true;

		IPAMappedBuffer *tuningBuffer = getMappedBufferIter(
			requestData.tuningBufferId);
		if (!tuningBuffer) {
			LOG(IPAMtkISP7, Error) << "Could not find tuning buffer!";
			dataMappedBuffers.valid = false;
			continue;
		}
		dataMappedBuffers.tuning = tuningBuffer;

		if (requestData.statisticsBufferId > 0) {
			IPAMappedBuffer *statBuffer = getMappedBufferIter(
				requestData.statisticsBufferId);
			if (!statBuffer) {
				LOG(IPAMtkISP7, Error)
					<< "Could not find stat buffer!";
				dataMappedBuffers.valid = false;
				continue;
			}
			dataMappedBuffers.statistics = statBuffer;
		}

		IPAMappedBuffer *swHistBuffer = nullptr;
		if (requestData.swHistBufferId > 0) {
			swHistBuffer = getMappedBufferIter(requestData.swHistBufferId);
			if (!swHistBuffer) {
				LOG(IPAMtkISP7, Error)
					<< "Could not find swHist buffer!";
				dataMappedBuffers.valid = false;
				continue;
			}
			dataMappedBuffers.swHist = swHistBuffer;
		}

		for (const auto &[_, bufferId] : requestData.reserved) {
			IPAMappedBuffer *mappedBuffer = getMappedBufferIter(bufferId);
			if (!mappedBuffer) {
				LOG(IPAMtkISP7, Error)
					<< "Could not find reserved buffer: "
					<< bufferId;
				dataMappedBuffers.valid = false;
				break;
			}
			dataMappedBuffers.reserved.push_back(mappedBuffer);
		}
	}

	ispManager_->invokeMethod(
		&IPAMtkISP7::IspManager::getImgSysMetaTuning,
		ConnectionTypeQueued,
		cookie, camSysMetaRequestId, frame, needCropTNC16x9,
		static_cast<Feature>(featureEnum),
		imgMetaRequests, std::move(dataMappedBuffersList),
		controls);
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
	aaaIspExchange.aaaMetadata = controls::controls;
	LensPositionInfo lensPositionInfo;
	std::optional<uint32_t> idApplied = std::nullopt;
	if (internalRequestIdApplied != 0)
		idApplied = internalRequestIdApplied;

	std::optional<Feature> featureApplied = std::nullopt;
	if (featureEnum >= 0)
		featureApplied = static_cast<Feature>(featureEnum);
	{
		DmaSyncer syncer(rawMetaFd);

		ipa_->hal3A_->doCalculation(statistics0, timestamp, internalRequestId,
					    camSysMetaRequestId, isStillCapture,
					    rawMetaBuffer,
					    metadata, gyroSample,
					    &exposureAndGain, &aaaIspExchange,
					    idApplied, featureApplied,
						&lensPositionInfo, controls);

		ipa_->halIsp_->getCamSysMetaTuning(
			internalRequestId, internalRequestId, rawMetaFd,
			(intptr_t)rawMetaBuffer, 0,
			Hal3A::kRawMetaSize, isStillCapture,
			metadata ? &metadata.value() : nullptr,
			internalRequestIdApplied, featureApplied,
			&aaaIspExchange, controls);
	}

	ipa_->AAResultReady.emit(internalRequestId, exposureAndGain, aaaIspExchange, lensPositionInfo);

	if (idApplied && featureApplied) {
		ipa_->onDeviceTuner_.tune3AState(
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
					    GyroSensor::SensorSample gyroSample,
					    const ControlList &controls)
{
	int32_t position = -1;
	ipa_->hal3A_->doCalculationAF(statistics1, timestamp, internalRequestId,
				      camSysMetaRequestId, vcmFocusInfo,
				      metadata, gyroSample, &position, controls);
	ipa_->AFResultReady.emit(internalRequestId, position);
}

IPAMtkISP7::IspManager::IspManager(IPAMtkISP7 *ipa)
	: ipa_(ipa)
{
}

void IPAMtkISP7::IspManager::getImgSysMetaTuning(
	const uint64_t cookie, const uint32_t camSysMetaRequestId,
	const uint32_t frame, const bool needCropTNC16x9,
	const Feature feature,
	const std::vector<ipa::mtkisp7::ImgMetaRequestData> imgMetaRequests,
	const std::vector<DataMappedBuffers> dataMappedBuffersList,
	const ControlList &controls)
{
	for (uint32_t i = 0; i < imgMetaRequests.size(); ++i) {
		const auto &requestData = imgMetaRequests[i];
		const auto &dataMappedBuffers = dataMappedBuffersList[i];

		if (!dataMappedBuffers.valid)
			continue;

		std::vector<DmaSyncer> syncers;

		IPAMappedBuffer *tuningBuffer = dataMappedBuffers.tuning;
		if (!tuningBuffer) {
			LOG(IPAMtkISP7, Fatal) << "Could not find tuning buffer!";
			continue;
		}
		syncers.emplace_back(tuningBuffer->buffer.planes()[0].fd.get());

		IPAMappedBuffer *statBuffer = nullptr;
		if (requestData.statisticsBufferId > 0) {
			statBuffer = dataMappedBuffers.statistics;
			if (!statBuffer) {
				LOG(IPAMtkISP7, Fatal) << "Could not find stat buffer!";
				continue;
			}

			syncers.emplace_back(statBuffer->buffer.planes()[0].fd.get());
		}

		IPAMappedBuffer *swHistBuffer = nullptr;
		if (requestData.swHistBufferId > 0) {
			swHistBuffer = dataMappedBuffers.swHist;
			if (!swHistBuffer) {
				LOG(IPAMtkISP7, Fatal) << "Could not find swHist buffer!";
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

		uint32_t j = 0;
		for (const auto &[keyInt, bufferId] : requestData.reserved) {
			if (j >= dataMappedBuffers.reserved.size()) {
				LOG(IPAMtkISP7, Fatal)
					<< "Invalid reserved mapped buffer index: "
					<< j << ", size: "
					<< dataMappedBuffers.reserved.size();
			}

			IPAMappedBuffer *mappedBuffer = dataMappedBuffers.reserved[j++];
			if (!mappedBuffer) {
				LOG(IPAMtkISP7, Fatal)
					<< "Invalid reserved mapped buffer: "
					<< j << ", size: "
					<< dataMappedBuffers.reserved.size();
				continue;
			}

			mtk::isphal::kISPExtBuf key =
				static_cast<mtk::isphal::kISPExtBuf>(keyInt);
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

		ipa_->halIsp_->getImgSysMetaTuning(
			camSysMetaRequestId, request, frame,
			requestData.hasFrameNumber
				? requestData.frameNumber
				: frame,
			needCropTNC16x9, feature, controls);
	}

	ipa_->ImgSysMetaTuningDone.emit(cookie);
}

} // namespace ipa::mtkisp7
} // namespace libcamera
