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

	provider_ = mtk::isphal::v1_0::TuningDataProvider::createInstance(
		sensorIdx_, sensorDev_, 0);

	return 0;
}

uint32_t HalIsp::getLpnrIsoThreshold(AaaIspExchange *aaaIspExchange)
{
	if (lpnrThredshold_)
		return lpnrThredshold_.value();

	mtk::isphal::v1::isp_lpnrthres_Param param;
	CAM_IDX_QRY_COMB_WITH_SYSTEM_INFO qry =
		aaaIspExchange->cam_info.rMapping_Info_with_sys_info;

	qry.mapping_info.eFeature = EFeature_Capture_lpnr;
	qry.mapping_info.eStage = EStage_TR_R2Y;
	qry.mapping_info.eAction = EAction_Capture;

	provider_->readDataForFeature(&param, sizeof(param), EModuleDB_LPNR_THRES, qry);
	lpnrThredshold_ = static_cast<uint32_t>(param.LPNR_ISO_HIGH_TH);

	return lpnrThredshold_.value();
}

int HalIsp::getCamSysMetaTuning(uint64_t frmId, uint64_t aaaFrmId,
				int fd, intptr_t va, size_t offset,
				size_t bufSize, AaaIspExchange *aaaIspExchange)
{
	ASSERT(aaaIspExchange);

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

	aaaIspExchange->cam_info = *tuning_param_p1.cam_info;
	aaaIspExchange->cam_info_3a = *tuning_param_p1.cam_info_3a;

	return 0;
}

void fillPqInfo(NSIspTuning::EStage_T stage, Size inputSize,
		Size outputSize, Size outputSize2,
		mtk::isphal::IspTuningBufferP2 &tuning_data)
{
	(void) inputSize;
	mtk::isphal::PQInfo pqInfo = {};
	mtk::isphal::WPEInfo wpeInfo = {};

	switch (stage) {
	case EStage_P2_Y2Y_PQ_DIP:
	case EStage_P2_MS_F0_PQ_DIP:
		pqInfo.CropSize = { inputSize.width, inputSize.height };
		pqInfo.OutSize = { outputSize.width, outputSize.height };
		pqInfo.serial_id = 1;
		tuning_data.pq_info.push_back(pqInfo);

		pqInfo.serial_id = 2;
		tuning_data.pq_info.push_back(pqInfo);
		break;
	case EStage_WPE_LTR_Y2Y_F1:
		wpeInfo.buf_id = mtk::isphal::kWPE_LITE;
		wpeInfo.is_motion = 1;
		tuning_data.wpe_info.push_back(wpeInfo);
		break;
	case EStage_WPE_WghtMap:
		wpeInfo.is_motion = 1;
		wpeInfo.buf_id = mtk::isphal::kWPE_TNR;
		tuning_data.wpe_info.push_back(wpeInfo);
		wpeInfo.buf_id = mtk::isphal::kWPE_LITE;
		tuning_data.wpe_info.push_back(wpeInfo);
		break;
	case EStage_WPE_P2_PQDIP_MS_F0:
		pqInfo.active_tcc = 1;
		pqInfo.ctrl = mtk::isphal::kIspPQControlModeAuto;

		pqInfo.serial_id = 1;
		pqInfo.CropSize = { outputSize.width, outputSize.height };
		pqInfo.OutSize = { outputSize.width, outputSize.height };
		tuning_data.pq_info.push_back(pqInfo);

		pqInfo.serial_id = 2;
		pqInfo.CropSize = { outputSize2.width, outputSize2.height };
		pqInfo.OutSize = { outputSize2.width, outputSize2.height };
		tuning_data.pq_info.push_back(pqInfo);

		wpeInfo.is_motion = 1;
		wpeInfo.buf_id = mtk::isphal::kWPE_TNR;
		tuning_data.wpe_info.push_back(wpeInfo);
		break;
	case EStage_TR_Y2Y_F1:
	case EStage_TR_R2Y:
	case EStage_TR_Y2Y_F4:
	case EStage_LTR_VBI:
	case EStage_LTR_Y2Y_F4:
	case EStage_P2_MS_F0_H:
	case EStage_ME_3PASS_MODE0:
	case EStage_ME_3PASS_MM:
	case EStage_ME_3PASS_MODE1:
	case EStage_LTR_ME_L1:
	case EStage_P2_IDI:
	case EStage_P2_MS_F_SMALL:
	case EStage_P2_MS_F4:
	case EStage_P2_MS_F3:
	case EStage_P2_MS_F2:
	case EStage_P2_MS_F1:
		break;
	default:
		ASSERT(false);
	}
}

void fillIndex(NSIspTuning::EStage_T stage, bool isCapture,
	       mtk::isphal::v1_0::IspImgSysControl& imgsys_info)
{
	imgsys_info.tnr_fw_config.frameIndex = 0;
	imgsys_info.tnr_fw_config.scaleIndex = 0;
	imgsys_info.tnr_fw_config.totalScaleNo = 0;
	imgsys_info.tnr_fw_config.bInkMode = 0;
	imgsys_info.tnr_fw_config.frameTotal = 0;
	imgsys_info.ds_mode = 0;
	imgsys_info.total_frame_num = 0;

	switch (stage) {
	case EStage_TR_Y2Y_F1:
	case EStage_TR_Y2Y_F4:
	case EStage_LTR_VBI:
	case EStage_LTR_Y2Y_F4:
	case EStage_WPE_LTR_Y2Y_F1:
	case EStage_WPE_WghtMap:
	case EStage_TR_R2Y:
	case EStage_ME_3PASS_MODE0:
	case EStage_ME_3PASS_MM:
	case EStage_ME_3PASS_MODE1:
	case EStage_LTR_ME_L1:
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_P2_Y2Y_PQ_DIP:
	case EStage_P2_MS_F0_PQ_DIP:
		imgsys_info.ds_mode = 1;
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_P2_MS_F0_H:
		imgsys_info.ds_mode = 1;
		imgsys_info.total_frame_num = 1;
		imgsys_info.tnr_fw_config.scaleIndex = 0;
		break;
	case EStage_P2_IDI:
		imgsys_info.tnr_fw_config.frameIndex = 255;
		imgsys_info.tnr_fw_config.scaleIndex = 6;
		imgsys_info.tnr_fw_config.totalScaleNo = 7;
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_P2_MS_F_SMALL:
		imgsys_info.tnr_fw_config.frameIndex = 255;
		imgsys_info.tnr_fw_config.scaleIndex = 5;
		imgsys_info.tnr_fw_config.totalScaleNo = 7;
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_P2_MS_F4:
		imgsys_info.tnr_fw_config.frameIndex = 255;
		imgsys_info.tnr_fw_config.scaleIndex = 4;
		imgsys_info.tnr_fw_config.totalScaleNo = 7;
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_P2_MS_F3:
		imgsys_info.tnr_fw_config.frameIndex = 255;
		imgsys_info.tnr_fw_config.scaleIndex = 3;
		if (isCapture)
			imgsys_info.tnr_fw_config.totalScaleNo = 4;
		else
			imgsys_info.tnr_fw_config.totalScaleNo = 7;
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_P2_MS_F2:
		imgsys_info.tnr_fw_config.frameIndex = 255;
		imgsys_info.tnr_fw_config.scaleIndex = 2;
		if (isCapture)
			imgsys_info.tnr_fw_config.totalScaleNo = 4;
		else
			imgsys_info.tnr_fw_config.totalScaleNo = 7;
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_P2_MS_F1:
		imgsys_info.tnr_fw_config.frameIndex = 255;
		imgsys_info.tnr_fw_config.scaleIndex = 1;
		if (isCapture)
			imgsys_info.tnr_fw_config.totalScaleNo = 4;
		else
			imgsys_info.tnr_fw_config.totalScaleNo = 7;
		imgsys_info.total_frame_num = 1;
		break;
	case EStage_WPE_P2_PQDIP_MS_F0:
		imgsys_info.tnr_fw_config.frameIndex = 255;
		imgsys_info.tnr_fw_config.scaleIndex = 0;
		imgsys_info.tnr_fw_config.totalScaleNo = 7;
		imgsys_info.total_frame_num = 1;
		break;
	default:
		ASSERT(false);
	}
}

void fillTncInfo(NSIspTuning::EStage_T stage, Size inputSize, Size outputSize, Size fullDipSize,
		 mtk::isphal::v1_0::IspImgSysControl& imgsys_info)
{
	(void) stage;
	(void) outputSize;
	imgsys_info.rCropRzInfo.rBefore_Warp_Crop = {};
	imgsys_info.rCropRzInfo.rBefore_Warp_Size = {};
	imgsys_info.tncs_info = {};
	imgsys_info.tnc_roi = {};

	switch (stage) {
	case EStage_TR_Y2Y_F1:
	case EStage_TR_R2Y:
		imgsys_info.rCropRzInfo.rBefore_Warp_Crop =
			mtk::isphal::Rectangle { 0, 0, inputSize.width, inputSize.width };
		imgsys_info.rCropRzInfo.rBefore_Warp_Size = mtk::isphal::Size {
			inputSize.width, inputSize.height };
		break;
	case EStage_P2_Y2Y_PQ_DIP:
	case EStage_P2_MS_F0_PQ_DIP:
	case EStage_WPE_P2_PQDIP_MS_F0:
	case EStage_P2_IDI:
		imgsys_info.rCropRzInfo.rBefore_Warp_Size = mtk::isphal::Size {
			inputSize.width, inputSize.height };
		break;
	case EStage_P2_MS_F_SMALL:
	case EStage_P2_MS_F4:
	case EStage_P2_MS_F3:
	case EStage_P2_MS_F2:
	case EStage_P2_MS_F1:
	case EStage_TR_Y2Y_F4:
	case EStage_LTR_VBI:
	case EStage_LTR_Y2Y_F4:
	case EStage_WPE_LTR_Y2Y_F1:
	case EStage_WPE_WghtMap:
	case EStage_P2_MS_F0_H:
	case EStage_ME_3PASS_MODE0:
	case EStage_ME_3PASS_MM:
	case EStage_ME_3PASS_MODE1:
	case EStage_LTR_ME_L1:
		break;
	default:
		ASSERT(false);
	}

	switch (stage) {
	case EStage_TR_Y2Y_F1:
	case EStage_TR_R2Y:
		imgsys_info.tncs_info.bValid = 1;
		imgsys_info.tncs_info.tncs_in_cropinfo = mtk::isphal::Rectangle {
			0, 0, inputSize.width, inputSize.height };;
		imgsys_info.tncs_info.target_tnc_size = mtk::isphal::Size {
			fullDipSize.width, fullDipSize.height };
		break;
	case EStage_P2_Y2Y_PQ_DIP:
	case EStage_P2_MS_F0_PQ_DIP:
	case EStage_P2_IDI:
	case EStage_WPE_P2_PQDIP_MS_F0:
		imgsys_info.tnc_roi.bValid = 1;
		imgsys_info.tnc_roi.tnc_in_cropinfo = mtk::isphal::Rectangle {
			0, 0, inputSize.width, inputSize.height };
		break;
	case EStage_P2_MS_F_SMALL:
	case EStage_P2_MS_F4:
	case EStage_P2_MS_F3:
	case EStage_P2_MS_F2:
	case EStage_P2_MS_F1:
	case EStage_TR_Y2Y_F4:
	case EStage_LTR_VBI:
	case EStage_LTR_Y2Y_F4:
	case EStage_WPE_LTR_Y2Y_F1:
	case EStage_WPE_WghtMap:
	case EStage_P2_MS_F0_H:
	case EStage_ME_3PASS_MODE0:
	case EStage_ME_3PASS_MM:
	case EStage_ME_3PASS_MODE1:
	case EStage_LTR_ME_L1:
		break;
	default:
		ASSERT(false);
	}
}

int HalIsp::getImgSysMetaTuning(AaaIspExchange *aaaIspExchange,
				ImgMetaRequest &request)
{
	bool is_capture = request.isCapture;
	Size inputSize = request.inputSize;
	Size outputSize = request.outputSize;
	Size outputSize2 = request.outputSize2;
	Size fullDipSize = request.fullDipSize;

	InfoFrame &tuningFrame = request.tuningBuffer;
	InfoFrame &statisFrame = request.statisticsBuffer;
	InfoFrame &swHistBuffer = request.swHistBuffer;

	mtk::isphal::IspTuningControl tuning_control = {};
	mtk::isphal::IspTuningStatisticsP2 tuning_statistics = {};
	mtk::isphal::IspTuningBufferP2 tuning_data = {};

	tuning_control.stage = request.stage;
	tuning_control.mock = false;
	tuning_control.update_mode = mtk::isphal::kIspUpdateModeAuto;

	if (is_capture)
		tuning_control.action = NSIspTuning::EAction_Capture;
	else
		tuning_control.action = NSIspTuning::EAction_Preview;

	mtk::isphal::Buffer metaBuf(
		(intptr_t)tuningFrame.address(0),
		tuningFrame.buffer()->planes()[0].fd.get(),
		tuningFrame.buffer()->planes()[0].offset,
		tuningFrame.buffer()->planes()[0].length);

	tuning_data.p2_meta_buffer = metaBuf;
	tuning_data.in_image = {};

	fillPqInfo(request.stage, inputSize, outputSize, outputSize2, tuning_data);

	if (statisFrame.buffer()) {
		mtk::isphal::Buffer statsBuf(
			(intptr_t)statisFrame.address(0),
			statisFrame.buffer()->planes()[0].fd.get(),
			statisFrame.buffer()->planes()[0].offset,
			statisFrame.buffer()->planes()[0].length);
		tuning_statistics.imgsys_statistics = statsBuf;
	}

	if (swHistBuffer.buffer()) {
		mtk::isphal::Buffer swBuf(
			(intptr_t)swHistBuffer.address(0),
			swHistBuffer.buffer()->planes()[0].fd.get(),
			swHistBuffer.buffer()->planes()[0].offset,
			swHistBuffer.buffer()->planes()[0].length);
		tuning_statistics.imgsys_hist_buffer = swBuf;
	}

	for (auto &[key, frame] : request.reserved) {
		mtk::isphal::Buffer reserveBuf(
			(intptr_t)frame.address(0),
			frame.buffer()->planes()[0].fd.get(),
			frame.buffer()->planes()[0].offset,
			frame.buffer()->planes()[0].length);
		tuning_statistics.reserved[key] = reserveBuf;
	}

	mtk::isphal::v1_0::TuningParamDip tuning_param_p2 = {};
	mtk::isphal::v1_0::ReturnParamDip result_p2 = {};

	tuning_param_p2.tuning_stat.push_back(&tuning_statistics);

	tuning_param_p2.imgsys_info.emplace_back();
	mtk::isphal::v1_0::IspImgSysControl& imgsys_info = tuning_param_p2.imgsys_info.back();

	result_p2.tuning_data.push_back(&tuning_data);

	imgsys_info.mock_imgsys = tuning_control.mock;

	int camsysFrmId = 0;

	/* parsePipelineMetadata */
	{
		mtk::isphal::v1_0::IspPerframeControl *pCaminfoBuf = NULL;
		mtk::isphal::v1_0::IspReadOnlyControl *pCaminfoBuf_3a = NULL;
		const uint8_t* pModuleBuf = NULL;

		pCaminfoBuf = &aaaIspExchange->cam_info;
		pCaminfoBuf_3a = &aaaIspExchange->cam_info_3a;

		tuning_param_p2.is_need_exif = 0;

		tuning_param_p2.cam_info = *pCaminfoBuf;
		tuning_param_p2.cam_info_3a = pCaminfoBuf_3a;
		tuning_param_p2.pModulesCtrl = pModuleBuf;

		tuning_param_p2.cam_info.drzs8t_crop_info.is_valid = true;
		tuning_param_p2.cam_info.drzs8t_crop_info.crop_region =
			NSCam::MRect(activeArray_.width, activeArray_.height);
		tuning_param_p2.cam_info.drzs8t_crop_info.dst_size =
			NSCam::MSize(fullDipSize.width, fullDipSize.height);

		// Update u8Id to pipe frame no to align NDD, internalId?
		camsysFrmId = tuning_param_p2.cam_info.u8Id;
		tuning_param_p2.cam_info.ISP_3A_result_id = camsysFrmId;
		//tuning_param_p2.cam_info.u8Id = camsysFrmId - 4;

		auto &shading = aaaIspExchange->aaaResult.shading_result;
		int32_t lsc_data_size = shading.lsc_data.size();

		if (lsc_data_size > 0) {
			tuning_param_p2.shading_table = shading.lsc_data.data();
			tuning_param_p2.shading_table_size = shading.lsc_data.size();
			tuning_param_p2.cam_info.shd_info.data_valid = true;
		}

		tuning_param_p2.cam_info.multi_frame_bss_index = 0;
		tuning_param_p2.cam_info.user_id = 0;

		if (is_capture)
			tuning_param_p2.cam_info.rMapping_Info.eFeature = NSIspTuning::EFeature_Capture_lpnr;
		else
			tuning_param_p2.cam_info.rMapping_Info.eFeature = NSIspTuning::EFeature_Video;

		tuning_param_p2.cam_info.rMapping_Info.eCustomFeature = NSIspTuning::ECustomFeature_OFF;

		tuning_param_p2.cam_info.tone_map_mode = mtk::isphal::v1_0::kToneMapModeAuto;
		tuning_param_p2.cam_info.edge_mode = mtk::isphal::v1_0::kEdgeModeOn;

		if (is_capture) {
			tuning_param_p2.camsys_history.tone_map_mode = MTK_TONEMAP_MODE_HIGH_QUALITY;
			tuning_param_p2.camsys_history.edge_mode = MTK_EDGE_MODE_HIGH_QUALITY;
			tuning_param_p2.camsys_history.nr_mode = MTK_NOISE_REDUCTION_MODE_HIGH_QUALITY;
		} else {
			tuning_param_p2.camsys_history.tone_map_mode = MTK_TONEMAP_MODE_FAST;
			tuning_param_p2.camsys_history.edge_mode = MTK_EDGE_MODE_FAST;
			tuning_param_p2.camsys_history.nr_mode = MTK_NOISE_REDUCTION_MODE_FAST;
		}

		if (is_capture)
			tuning_param_p2.capture_mode = mtk::isphal::v1_0::kCaptureModeNormal;
		else
			tuning_param_p2.capture_mode = mtk::isphal::v1_0::kCaptureModeNone;

		mtk::isphal::Size tSize(inputSize.width, inputSize.height);
		tuning_param_p2.cam_info.rCropRzInfo.targetSize = tSize;
	}

	// parseImgSysMetadata
	{
		fillIndex(request.stage, is_capture, imgsys_info);

		imgsys_info.sequence_num = camsysFrmId;
		imgsys_info.is_need_dump_exif = 0;

		mtk::isphal::Size mel0Out(576, 432);
		//mtk::isphal::Size gyroOut(32, 24);
		imgsys_info.rCropRzInfo.sMEL0out = mel0Out;
		imgsys_info.rCropRzInfo.sGyroMv = {};

		fillTncInfo(request.stage, inputSize, outputSize, fullDipSize, imgsys_info);

		imgsys_info.rWrappingInfo = {};
		imgsys_info.bypass_nr = 0;
	}

	// parseImgSysBriefPart
	{
		uint8_t u1P2TuningUpdate = tuning_control.update_mode;
		imgsys_info.tuing_update_mode =
			static_cast<mtk::isphal::v1_0::TuningUpdateMode>(u1P2TuningUpdate);

		imgsys_info.stage = static_cast<NSIspTuning::EStage_T>(tuning_control.stage);

		mtk::isphal::Size imgsys_in_size(inputSize.width, inputSize.height);
		imgsys_info.rCropRzInfo.imgsys_in_size = imgsys_in_size;

		imgsys_info.is_capture = is_capture;
		imgsys_info.action = tuning_control.action;
	}

	mtk::isphal::v1_0::IspPerframeControl& cam_info = tuning_param_p2.cam_info;
	{
		// Check the following values
		imgsys_info.srcimg_descriptor.p2_in_img_fmg = 0;
		imgsys_info.srcimg_descriptor.format = mtk::isphal::kImageFormatMtkRawBayer;

		imgsys_info.nr_mode = mtk::isphal::v1_0::kNoiseReductionModeOn;

		// copy caminfo
		imgsys_info.rMapping_Info = cam_info.rMapping_Info;
		imgsys_info.rMapping_Info_with_sys_info =
		    cam_info.rMapping_Info_with_sys_info;
		imgsys_info.rNdd_info = cam_info.rNdd_info;
		imgsys_info.sr_para = cam_info.sr_para;

		imgsys_info.rFdInfo_afterWarp = cam_info.rFdInfo;

		// replace with correct stage
		imgsys_info.rMapping_Info.eStage =
		    static_cast<NSIspTuning::EStage_T>(imgsys_info.stage);
		imgsys_info.rMapping_Info.eAction =
		    static_cast<NSIspTuning::EAction_T>(imgsys_info.action);

	}
	m_pHalisp->getImgSysMetaTuning(&tuning_param_p2, &result_p2);

	return 0;
}

} // namespace libcamera
