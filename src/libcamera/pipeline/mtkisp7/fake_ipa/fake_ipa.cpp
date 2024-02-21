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
	return 0;
}

/**
 * \brief Perform any processing required before the first frame
 */
int IPAMtkISP7::start(const uint32_t rawMetaBufferId)
{
	auto itRawMeta = buffers_.find(rawMetaBufferId);
	if (itRawMeta == buffers_.end()) {
		LOG(IPAMtkISP7, Error) << "Could not find rawMeta buffer!";
		return -1;
	}
	MappedFrameBuffer mappedRawMeta(&itRawMeta->second,
					MappedFrameBuffer::MapFlag::ReadWrite);

	hal3A_->start(reinterpret_cast<mtk_cam_uapi_meta_raw_stats_cfg *>(
		mappedRawMeta.planes()[0].data()));

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

	return 0;
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

	auto itRawMeta = buffers_.find(rawMetaBufferId);
	if (itRawMeta == buffers_.end()) {
		LOG(IPAMtkISP7, Error) << "Could not find rawMeta buffer!";
		return;
	}
	MappedFrameBuffer mappedRawMeta(&itRawMeta->second, MappedFrameBuffer::MapFlag::ReadWrite);

	MtkCameraFaceMetadata *faces = (metadata_) ? &(*metadata_) : nullptr;

	GyroSensor::SensorSample sample;
	sample.x_value = gyroSample.x_value;
	sample.y_value = gyroSample.y_value;
	sample.z_value = gyroSample.z_value;
	sample.timestamp = gyroSample.timestamp;

	std::optional<uint32_t> idApplied = std::nullopt;
	if (internalRequestIdApplied != 0)
		idApplied = internalRequestIdApplied;

	SensorSetting exposureAndGain;
	std::optional<Feature> featureApplied = std::nullopt;
	if (featureEnum >= 0)
		featureApplied = static_cast<Feature>(featureEnum);

	hal3A_->doCalculation(&itStat0->second, timestamp, frame,
			      camSysMetaRequestId, isStillCapture,
			      itRawMeta->second.planes()[0].fd.get(),
			      mappedRawMeta.planes()[0].data(),
			      faces, sample,
			      &exposureAndGain, aaaIspExchange_,
			      idApplied, featureApplied, controls);

	AAResultReady.emit(frame, exposureAndGain);

	if (internalRequestIdApplied != 0 && featureApplied.has_value()) {
		onDeviceTuner_->tune3AState(
			internalRequestIdApplied,
			&itStat0->second, &hal3A_->r3AResult_,
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

	hal3A_->doCalculationAF(&itStat1->second, timestamp, frame,
				afCamSysMetaRequestId, vcm,
				faces, sample, &position);
	AFResultReady.emit(frame, position);
}

} // namespace ipa::mtkisp7
} // namespace libcamera
