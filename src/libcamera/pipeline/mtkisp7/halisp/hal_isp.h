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
#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/IHal3A.h"
#include "halisp/utils/Size.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"
#include "platform/mtkisp7/halisp/IHalIsp.h"
#include "platform/mtkisp7/halisp/ITuningDataProvider.h"
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/camera_db/include/BasicModule/auto/isp/isp_swme_Param.h"

#include "stdint.h"

namespace libcamera {
class Hal3A;

/* Struct to exchange information between 3A and HalIsp tasks */
struct AaaIspExchange {
	bool highIsoMode = false;
	uint32_t aaaRequestId = 0;
	ControlList aaaMetadata;
};

struct ImgMetaRequest {
	bool isCapture;
	NSIspTuning::EStage_T stage;
	InfoFrame tuningBuffer;
	InfoFrame statisticsBuffer;
	InfoFrame swHistBuffer;
	Size inputSize;
	Size outputSize;
	Size outputSize2;
	Size fullDipSize;

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
				bool needCropTNC16x9);

	std::shared_ptr<isp_swme_Param> querySwmeParam(const CAM_IDX_QRY_COMB_WITH_SYSTEM_INFO &qry, MBOOL force);
	std::shared_ptr<isp_swme_Param> getIspSwmeParam();

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
	class Data
	{
	public:
		Data() {}
		virtual ~Data() {}
	};
	template<typename T>
	class TData : public Data
	{
	public:
		TData() { data_ = std::make_shared<T>(); }
		~TData()
		{
			//MY_LOGD("~TData, size = %u", sizeof(T));
		}
		std::shared_ptr<T> get() { return data_; }

	private:
		std::shared_ptr<T> data_;
	};

	std::map<NSIspTuning::EModuleDB_T, std::shared_ptr<Data>> data_;

	std::mutex lk_;
};

} // namespace libcamera
