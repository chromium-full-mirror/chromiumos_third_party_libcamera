/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2024, Google Inc.
 *
 * fake_ipa.h - Fake IPA implementation for MtkISP7
 */
#pragma once

#include <libcamera/ipa/mtkisp7_ipa_interface.h>

#include "../hal3a/hal_3a.h"

// Workarounds
#include "../halisp/hal_isp.h"
#include "pipeline/mtkisp7/camsys/capture.h"
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
	void preDoCalculation3A(std::optional<MtkCameraFaceMetadata> metadata,
				AaaIspExchange *aaaIspExchange)
	{
		metadata_ = metadata;
		aaaIspExchange_ = aaaIspExchange;
	}

	int init(const int32_t sensorIdx) override;

	int start(const uint32_t rawMetaBufferId) override;
	void stop() override;

	int configure(const Size &camsysYuvSize, bool isVideo) override;

	void mapBuffers(const std::vector<IPABuffer> &buffers) override;
	void unmapBuffers(const std::vector<unsigned int> &ids) override;

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

	// Workarounds
	HalIsp *halIsp_ = nullptr;
	OnDeviceTuner *onDeviceTuner_;
	std::optional<MtkCameraFaceMetadata> metadata_;
	AaaIspExchange *aaaIspExchange_;

	std::map<unsigned int, FrameBuffer> buffers_;

	// TODO: Check if we create a different instance for each CameraData.
	std::unique_ptr<Hal3A> hal3A_;

	// The sensor being configured.
	int32_t sensorIdx_;
};

} // namespace ipa::mtkisp7
} // namespace libcamera
