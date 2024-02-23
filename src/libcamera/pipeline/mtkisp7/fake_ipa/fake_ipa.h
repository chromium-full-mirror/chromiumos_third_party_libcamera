/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * fake_ipa.h - Fake IPA implementation for MtkISP7
 */
#pragma once

#include <libcamera/ipa/mtkisp7_ipa_interface.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "../hal3a/hal_3a.h"
#include "libfdft_lib/faces.h"

// Workarounds
#include "../halisp/hal_isp.h"
#include "pipeline/mtkisp7/camsys/capture.h"
#include "pipeline/mtkisp7/face_detect/parser.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

namespace libcamera {

using namespace std::literals::chrono_literals;

class IPADelegate;
class MtkISP7CameraData;

namespace ipa::mtkisp7 {
class IPAMtkISP7 : public IPAMtkISP7Interface, public Object
{
public:
	IPAMtkISP7();

	// Workaround functions
	void preInit(HalIsp *halIsp, OnDeviceTuner *odt)
	{
		halIsp_ = halIsp;
		onDeviceTuner_ = odt;
	}
	void preDoCalculation3A(AaaIspExchange *aaaIspExchange)
	{
		aaaIspExchange_ = aaaIspExchange;
	}

	int init(const int32_t sensorIdx) override;

	int start(const uint32_t rawMetaBufferId) override;
	void stop() override;

	int configure(const Size &camsysYuvSize, bool isVideo) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

	void aieParse(
		const uint32_t inputImageBufferId,
		const uint32_t faceDetectionMetadataBufferId,
		const uint32_t faceToneClassificationMetadataBufferId,
		const Size &currentSensorSize,
		const uint32_t camSysMetaRequestId) override;

	void doCalculation3A(
		const uint32_t frame,
		const uint32_t stat0BufferId, const uint32_t stat1BufferId,
		const uint64_t timestamp, const uint32_t camSysMetaRequestId,
		const uint32_t afCamSysMetaRequestId,
		const bool isStillCapture, const uint32_t rawMetaBufferId,
		const GyroSampleData &gyroSample,
		const uint32_t internalRequestIdApplied,
		const int32_t featureEnum,
		const VcmFocusInformation &vcmFocusInfo,
		const ControlList &controls) override;

private:
	friend IPADelegate;
	friend MtkISP7CameraData;

	struct IPAMappedBuffer {
		IPAMappedBuffer(const std::vector<FrameBuffer::Plane> &planes)
			: buffer(planes) {}

		FrameBuffer buffer;
		std::unique_ptr<MappedFrameBuffer> mapped;
	};

	IPAMappedBuffer *getMappedBufferIter(unsigned int bufferId);

	ControlList convertFaceMetadata();

	// Workarounds
	HalIsp *halIsp_ = nullptr;
	OnDeviceTuner *onDeviceTuner_;
	AaaIspExchange *aaaIspExchange_;

	std::map<unsigned int, IPAMappedBuffer> buffers_;

	std::optional<MtkCameraFaceMetadata> latestFaceMetadata_;

	// TODO: Check if we create a different instance for each CameraData.
	std::unique_ptr<Hal3A> hal3A_;
	std::unique_ptr<AieParser> aieParser_;

	// The sensor being configured.
	int32_t sensorIdx_;
};

} // namespace ipa::mtkisp7
} // namespace libcamera
