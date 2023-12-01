/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * hal_isp.h - Delegate of MtkISP7 HalIsp
 */

#pragma once

#include <memory>
#include <optional>
#include "stdint.h"

#include <libcamera/geometry.h>

#include "platform/mtkisp7/halisp/IHalIsp.h"
#include "platform/mtkisp7/halisp/ITuningDataProvider.h"

#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/IHal3A.h"

namespace libcamera {

/* Struct to exchange information between 3A and HalIsp tasks */
struct AaaIspExchange {
	mtk::isphal::v1_0::IspPerframeControl cam_info;
	mtk::isphal::v1_0::IspReadOnlyControl cam_info_3a;
	mtk::hal3a::v1_0::mtk_3a_result aaaResult;
};

class HalIsp
{
public:
	HalIsp();

	int init(int32_t sensorIdx, int32_t sensorDev);

	int getCamSysMetaTuning(uint64_t frmId, uint64_t aaaFrmId,
				int fd, intptr_t va, size_t offset,
				size_t bufSize, AaaIspExchange *aaaIspExchange);

	uint32_t getLpnrIsoThreshold(AaaIspExchange *aaaIspExchange);

private:
	int32_t sensorIdx_;
	int32_t sensorDev_;

	Rectangle activeArray_;

	std::shared_ptr<mtk::isphal::v1::IHalIsp> m_pHalisp;
	mtk::isphal::v1_0::IspPerframeControl m_P1CamInfo;
	mtk::isphal::v1_0::IspReadOnlyControl m_P1CamInfo_3a;
	mtk::isphal::v1_0::IspPerframeControl m_BackupCamInfo;     // for p2
	mtk::isphal::v1_0::IspReadOnlyControl m_BackupCamInfo_3a;  // for p2

	std::shared_ptr<mtk::isphal::v1::ITuningDataProvider> provider_;
	std::optional<uint32_t> lpnrThredshold_;
};

} // namespace libcamera
