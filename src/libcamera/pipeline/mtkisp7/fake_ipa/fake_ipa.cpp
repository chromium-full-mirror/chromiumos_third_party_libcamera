/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * fake_ipa.cpp - Fake IPA implementation for MtkISP7
 */

#include "fake_ipa.h"

#include "libcamera/base/bound_method.h"
#include "libcamera/base/log.h"

namespace libcamera {

LOG_DEFINE_CATEGORY(IPAMtkISP7)

namespace ipa::mtkisp7 {

IPAMtkISP7::IPAMtkISP7()
{
}

int IPAMtkISP7::init(const int32_t sensorIdx)
{
	sensorIdx_ = sensorIdx;
	hal3A_ = std::make_unique<Hal3A>(sensorIdx, halIsp_, onDeviceTuner_);
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

	return hal3A_->r3AResult_.af_result.lens_position;
}

/**
 * \brief Ensure that all processing has completed
 */
void IPAMtkISP7::stop()
{
}

/**
 * \brief Configure the MtkISP7 IPA
 * \param[in] sensorIdx The index of the sensor being used now
 */
int IPAMtkISP7::configure(const Size &camsysYuvSize, bool isVideo)
{
	hal3A_->configure(camsysYuvSize, isVideo);

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

	std::optional<uint32_t> idApplied = std::nullopt;
	if (internalRequestIdApplied != 0)
		idApplied = internalRequestIdApplied;

	MtkCameraFaceMetadata *metadata = latestFaceMetadata_.has_value()
						  ? &latestFaceMetadata_.value()
						  : nullptr;

	SensorSetting exposureAndGain;
	std::optional<Feature> featureApplied = std::nullopt;
	if (featureEnum >= 0)
		featureApplied = static_cast<Feature>(featureEnum);

	hal3A_->doCalculation(&itStat0->second.buffer, timestamp, frame,
			      camSysMetaRequestId, isStillCapture,
			      rawMetaBuffer->buffer.planes()[0].fd.get(),
			      rawMetaBuffer->mapped->planes()[0].data(),
			      metadata, sample,
			      &exposureAndGain, aaaIspExchange_,
			      idApplied, featureApplied, controls);

	AAResultReady.emit(frame, exposureAndGain);

	if (internalRequestIdApplied != 0 && featureApplied.has_value()) {
		onDeviceTuner_->tune3AState(
			internalRequestIdApplied,
			&itStat0->second.buffer, &hal3A_->r3AResult_,
			featureApplied.value());
	}

	if (stat1BufferId == 0)
		return;

	// TODO: use another thread.
	::VcmFocusInformation vcm;
	vcm.focus_position = vcmFocusInfo.focus_position;
	vcm.previous_focus_position = vcmFocusInfo.previous_focus_position;
	vcm.moving_timestamp = vcmFocusInfo.moving_timestamp;
	vcm.previous_moving_timestamp = vcmFocusInfo.previous_moving_timestamp;

	int32_t position = -1;
	auto itStat1 = buffers_.find(stat1BufferId);
	if (itStat1 == buffers_.end()) {
		LOG(IPAMtkISP7, Error) << "Could not find stat1 buffer!";
		return;
	}

	hal3A_->doCalculationAF(&itStat1->second.buffer, timestamp, frame,
				afCamSysMetaRequestId, vcm,
				metadata, sample, &position);
	AFResultReady.emit(frame, position);
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

} // namespace ipa::mtkisp7
} // namespace libcamera
