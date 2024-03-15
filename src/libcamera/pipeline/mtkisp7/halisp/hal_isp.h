/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * hal_isp.h - Delegate of MtkISP7 HalIsp
 */

#pragma once

#include <cstdint>
#include <memory>
#include <optional>

#include <libcamera/geometry.h>

#include <libcamera/internal/info_frame.h>

#include "../utils/history.h"
#include "halisp/utils/Size.h"
#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/IHal3A.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"
#include "platform/mtkisp7/halisp/IHalIsp.h"
#include "platform/mtkisp7/halisp/ITuningDataProvider.h"

#include "stdint.h"

namespace libcamera {
class Hal3A;

/* Struct to exchange information between 3A and HalIsp tasks */
struct AaaIspExchange {
	bool highIsoMode = false;
	uint32_t aaaRequestId = 0;
	ControlList aaaMetadata;
	bool mfnrMode;
};

struct ImgMetaRequest {
	bool isCapture;
	bool isMfnr = false;
	NSIspTuning::EStage_T stage;
	InfoFrame tuningBuffer;
	InfoFrame statisticsBuffer;
	InfoFrame swHistBuffer;
	Size inputSize;
	Size outputSize;
	Size outputSize2;
	Size fullDipSize;
	int tnr_frameIndex = 0;
	int tnr_frameTotal = 1;

	std::unordered_map<mtk::isphal::kISPExtBuf, InfoFrame> reserved;
};

class HalIsp
{
public:
	struct CamInfo {
		mtk::isphal::v1_0::IspPerframeControl cam_info;
		mtk::isphal::v1_0::IspReadOnlyControl cam_info_3a;
	};

	HalIsp(OnDeviceTuner *odt);

	int init(int32_t sensorIdx, int32_t sensorDev, Hal3A *hal3A);

	void configure(const Size &maxVideoSize, const Size &maxStillSize);

	int getCamSysMetaTuning(uint64_t frmId, uint64_t aaaFrmId,
				int fd, intptr_t va, size_t offset,
				size_t bufSize, bool isCapture,
				MtkCameraFaceMetadata *faces,
				AaaIspExchange *aaaIspExchange,
				std::optional<uint32_t> internalRequestIdApplied);

	int getImgSysMetaTuning(AaaIspExchange *aaaIspExchange,
				ImgMetaRequest &imgMetaRequest,
				uint32_t internalRequestId,
				uint32_t frameNumber,
				bool needCropTNC16x9);

	int getImgSysMetaTuning(AaaIspExchange *aaaIspExchange,
				ImgMetaRequest &imgMetaRequest,
				uint32_t internalRequestId,
				bool needCropTNC16x9);

	std::shared_ptr<mtk::isphal::v1::isp_swme_Param> getIspSwmeParam();
	std::shared_ptr<mtk::isphal::v1::isp_bss_Param> getIspBssParam();
	std::shared_ptr<mtk::isphal::v1::isp_mfnrthres_Param> getIspMfnrThresParam();

private:
	uint32_t getLpnrIsoThreshold(mtk::isphal::v1_0::IspPerframeControl &cam_info);

	void fillCamInfoFaceData(MtkCameraFaceMetadata *faces,
				 mtk::isphal::CAMERA_TUNING_FD_INFO_T &fdInfo);
	mtk::isphal::Size getTargetSize(bool isCapture);

	void addHistory(uint32_t internalRequestId,
			mtk::isphal::v1_0::IspPerframeControl &cam_info,
			mtk::isphal::v1_0::IspReadOnlyControl &cam_info_3a);

	CamInfo *queryHistory(uint32_t internalRequestId);

	int32_t sensorIdx_;
	int32_t sensorDev_;
	int32_t sensorId_;

	Rectangle activeArray_;

	Size maxVideoStreamSize_;
	Size maxStillStreamSize_;

	std::shared_ptr<mtk::isphal::v1::IHalIsp> m_pHalisp;
	mtk::isphal::v1_0::IspPerframeControl m_P1CamInfo;
	mtk::isphal::v1_0::IspReadOnlyControl m_P1CamInfo_3a;
	mtk::isphal::v1_0::IspPerframeControl m_BackupCamInfo; // for p2
	mtk::isphal::v1_0::IspReadOnlyControl m_BackupCamInfo_3a; // for p2

	std::shared_ptr<mtk::isphal::v1::ITuningDataProvider> provider_;
	std::optional<uint32_t> lpnrThredshold_;
	OnDeviceTuner *onDeviceTuner_;

	Hal3A *hal3A_;

	History<CamInfo> camInfoHistory_;

public:
	std::shared_ptr<mtk::isphal::v1::isp_swme_Param> isp_swme_Param_ = nullptr;
	std::shared_ptr<mtk::isphal::v1::isp_bss_Param> isp_bss_Param_ = nullptr;
	std::shared_ptr<mtk::isphal::v1::isp_mfnrthres_Param> isp_mfnrthres_Param_ = nullptr;
};

} // namespace libcamera
