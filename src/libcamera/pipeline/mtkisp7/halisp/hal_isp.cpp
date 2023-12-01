/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023; Google Inc.
 *
 * hal_isp.cpp - Delegate of MtkISP7 HalIsp
 */

#include "hal_isp.h"

#include <sys/mman.h>

#include <libcamera/base/log.h>

#include "platform/mtkisp7/mtkcam-core/aaa/include/nvbuf_util.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

HalIsp::HalIsp()
{
}

int HalIsp::init(int32_t sensorIdx, int32_t sensorDev)
{
	sensorIdx_ = sensorIdx;
	sensorDev_ = sensorDev;

	// TODO: implement a proper init() for m_P1CamInfo to avoid vtable pointer overwritten.
	memset(&m_P1CamInfo, 0, sizeof(m_P1CamInfo));

	NVRAM_SENSOR_IDX_INFO _sensorIdxInfo;
	if (sensorIdx_ == 0) {
		_sensorIdxInfo.sensorDev = 1;
		_sensorIdxInfo.sensorId = 4921;
		_sensorIdxInfo.facing = 0;
		_sensorIdxInfo.moduleId = 0;
		_sensorIdxInfo.sensorName = "HI1339_MIPI_RAW";
	} else {
		_sensorIdxInfo.sensorDev = 2;
		_sensorIdxInfo.sensorId = 2211;
		_sensorIdxInfo.facing = 1;
		_sensorIdxInfo.moduleId = 0;
		_sensorIdxInfo.sensorName = "GC08A3_MIPI_RAW";
	}

	m_P1CamInfo.i4_sensor_id = _sensorIdxInfo.sensorId;
	m_P1CamInfo.app_iso_value = 100;
	m_P1CamInfo.i4ZoomRatio_x100 = 100;
	m_P1CamInfo.fgFDEnable = 1;
	m_P1CamInfo.rFdInfo.FD_source = 1;

	NvBufUtil::initSensorInfo(sensorIdx_, _sensorIdxInfo);
	m_pHalisp = mtk::isphal::v1::IHalIsp::createInstance(sensorDev, sensorIdx, 0);

	mtk_isp_buf_info bufferInfo;
	m_pHalisp->queryISPBufferInfo(&bufferInfo);

	printf("lceso_size: %zu\n"
	       "lcesho_size: %zu\n"
	       "dceso_size: %zu\n"
	       "camsys_stat_size: %zu\n"
	       "camsys_meta_size: %zu\n"
	       "camsys_meta_version: %zu\n"
	       "imgsys_stat_size: %zu\n"
	       "imgsys_hist_buf_size: %zu\n"
	       "imgsys_meta_size: %zu\n"
	       "imgsys_meta_version: %zu\n"
	       "hwme_stat_fst_size: %zu\n"
	       "hwme_stat_fmb_size: %zu\n"
	       "hwme_stat_lmi_size: %zu\n"
	       "fwme_fst_meta_size: %zu\n"
	       "fwmm_mmg_fbfst_meta_size: %zu\n"
	       "fwmm_mmg_rst_meta_size: %zu\n"
	       "fwmm_mil_meta_size: %zu\n",
	       bufferInfo.lceso_size, bufferInfo.lcesho_size, bufferInfo.dceso_size,
	       bufferInfo.camsys_stat_size, bufferInfo.camsys_meta_size, bufferInfo.camsys_meta_version,
	       bufferInfo.imgsys_stat_size, bufferInfo.imgsys_hist_buf_size, bufferInfo.imgsys_meta_size,
	       bufferInfo.imgsys_meta_version, bufferInfo.hwme_stat_fst_size,
	       bufferInfo.hwme_stat_fmb_size, bufferInfo.hwme_stat_lmi_size, bufferInfo.fwme_fst_meta_size,
	       bufferInfo.fwmm_mmg_fbfst_meta_size, bufferInfo.fwmm_mmg_rst_meta_size,
	       bufferInfo.fwmm_mil_meta_size);

	m_P1CamInfo.rMapping_Info.eSensorMode = ESensorMode_Preview;

	if (sensorIdx_ == 1) {
		m_P1CamInfo.rCropRzInfo.sTGout = mtk::isphal::Size{ 3264, 2448 };
		activeArray_ = Rectangle{ 0, 0, 3264, 2448 };
	} else { // sensor_idx_ == 0
		m_P1CamInfo.rCropRzInfo.sTGout = mtk::isphal::Size{ 4208, 3120 };
		activeArray_ = Rectangle{ 0, 0, 4208, 3120 };
	}

	m_P1CamInfo.rMapping_Info.eFeature = NSIspTuning::EFeature_Preview;
	m_P1CamInfo.rMapping_Info.eStage = NSIspTuning::EStage_P1;
	m_P1CamInfo.rMapping_Info.eCustomFeature = NSIspTuning::ECustomFeature_OFF;

	m_P1CamInfo.p1_yuv_port = 0x0111; // rConfigInfo.direct_yuv_path;
	m_P1CamInfo.bYUV_after_rrz = 0;

	m_P1CamInfo.rMapping_Info.eSensorFeature = NSIspTuning::ESensorFeature_OFF;

	m_P1CamInfo.hwhdr_info.i4fus_num = 0;
	m_P1CamInfo.hwhdr_info.hdr_type = mtk::isphal::v1_0::EISP_HWHDRType_None;

	m_P1CamInfo.yuvo_ds_mode_info.yuvo_r2_ds = 2;
	m_P1CamInfo.yuvo_ds_mode_info.yuvo_r4_ds = 0;

	return 0;
}

int HalIsp::getCamSysMetaTuning(uint64_t frmId, uint64_t aaaFrmId,
				int fd, intptr_t va, size_t offset,
				size_t bufSize)
{
	mtk::isphal::IspTuningBufferP1 tuning_data = {};

	mtk::isphal::Buffer regBuf1((intptr_t)va, fd, offset, bufSize);
	tuning_data.p1_meta_buffer = regBuf1;

	m_P1CamInfo.mock_camsys = false;

	// Target structure
	mtk::isphal::v1_0::TuningParamP1 tuning_param_p1 = {};
	mtk::isphal::v1_0::ReturnParamP1 result_p1 = {};

	// setup input/out data
	result_p1.tuning_data = &tuning_data;

	tuning_param_p1.cam_info = &m_P1CamInfo;
	tuning_param_p1.cam_info_3a = &m_P1CamInfo_3a;
	tuning_param_p1.tuning_stat = nullptr;

	tuning_param_p1.cam_info->hwhdr_info.i4fus_num = 0;
	tuning_param_p1.cam_info->hwhdr_info.hdr_type = mtk::isphal::v1_0::EISP_HWHDRType_None;

	tuning_param_p1.is_need_exif = false;

	tuning_param_p1.cam_info->rMapping_Info.eFeature = NSIspTuning::EFeature_Preview;
	tuning_param_p1.cam_info->rMapping_Info.eStage = NSIspTuning::EStage_P1;
	tuning_param_p1.cam_info->rMapping_Info.eSensorFeature = NSIspTuning::ESensorFeature_OFF;

	tuning_param_p1.cam_info->qrm_enable = 0;
	tuning_param_p1.cam_info->qbpc_enable = 0;

	tuning_param_p1.cam_info->i4ZoomRatio_x100 = 100;

	tuning_param_p1.cam_info->rCropRzInfo.targetSize =
		mtk::isphal::Size{ activeArray_.width, activeArray_.height };

	tuning_param_p1.cam_info->control_mode = mtk::isphal::v1_0::kControlModeOn;
	tuning_param_p1.capture_mode = mtk::isphal::v1_0::kCaptureModeNone;
	tuning_param_p1.cam_info->color_correction_mode = mtk::isphal::v1_0::kColorCorrectionModeAuto;
	tuning_param_p1.cam_info->sensor_test_pattern_mode = mtk::isphal::v1_0::kSensorTestPatternModeOff;

	float mat[9] = { 1.0f, 0.0f, 0.0f,
			 0.0f, 1.0f, 0.0f,
			 0.0f, 0.0f, 1.0f };
	memcpy(tuning_param_p1.cam_info->color_correction_transform.mat, mat, sizeof(mat));
	tuning_param_p1.cam_info->hdr10_enable = false;

	tuning_param_p1.cam_info->rMapping_Info.eCustomFeature = NSIspTuning::ECustomFeature_OFF;
	tuning_param_p1.cam_info->frame_rate = 30;

	tuning_param_p1.cam_info->rP1SyncInfo.bSync2AMode = false;
	tuning_param_p1.cam_info->rP1SyncInfo.bSlave_P1 = false;

	tuning_param_p1.pModulesCtrl = NULL;
	tuning_param_p1.shading_table = NULL;

	tuning_param_p1.cam_info->sr_para.enable = 0;
	tuning_param_p1.cam_info->ggm_info.is_ai3a_en = false;

	tuning_param_p1.cam_info->u8Id = frmId; // update frame number
	tuning_param_p1.cam_info->aaaId = aaaFrmId; // update frame number

	tuning_param_p1.magic_num = frmId;
	tuning_param_p1.aaa_magic_num = aaaFrmId;
	tuning_param_p1.subsample_count = 1;

	m_pHalisp->getCamSysMetaTuning(&tuning_param_p1, &result_p1);

	return 0;
}

} // namespace libcamera
