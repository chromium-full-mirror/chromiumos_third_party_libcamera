/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * hal_isp.h - Delegate of MtkISP7 HalIsp
 */

#pragma once

#include <memory.h>
#include <stdint.h>

#include <libcamera/geometry.h>

#include "platform/mtkisp7/halisp/IHalIsp.h"

namespace libcamera {

class HalIsp
{
public:
	HalIsp();

	int init(int32_t sensorIdx, int32_t sensorDev);

	int getCamSysMetaTuning(uint64_t frmId, uint64_t aaaFrmId,
				int fd, intptr_t va, size_t offset,
				size_t bufSize);

private:
	int32_t sensorIdx_;
	int32_t sensorDev_;

	Rectangle activeArray_;

	std::shared_ptr<mtk::isphal::v1::IHalIsp> m_pHalisp;
	mtk::isphal::v1_0::IspPerframeControl m_P1CamInfo;
	mtk::isphal::v1_0::IspReadOnlyControl m_P1CamInfo_3a;
	mtk::isphal::v1_0::IspPerframeControl m_BackupCamInfo; // for p2
	mtk::isphal::v1_0::IspReadOnlyControl m_BackupCamInfo_3a; // for p2
};

} // namespace libcamera
