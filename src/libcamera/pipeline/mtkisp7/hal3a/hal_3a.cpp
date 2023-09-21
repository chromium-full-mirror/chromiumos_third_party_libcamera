/*
 * Copyright (C) 2023; Google Inc.
 *
 * hal_3a.cpp - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#include "hal_3a.h"

#include <sys/mman.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "mtkcam-core/aaa/include/nvbuf_util.h"

namespace libcamera {

namespace {
constexpr unsigned int kMetaSize = 113664;
} // namespace

LOG_DECLARE_CATEGORY(MtkISP7)

Hal3A::Hal3A(const uint32_t sensor_idx, DmaHeap *dmaHeap)
	: sensor_idx_(sensor_idx), dmaHeap_(dmaHeap)
{
	fd_ = dmaHeap_->alloc(kMetaSize, libcamera::DmaHeap::Type::CMA);
	meta_addr_ = reinterpret_cast<mtk_cam_uapi_meta_raw_stats_cfg *>(mmap(nullptr, kMetaSize, PROT_READ | PROT_WRITE, MAP_SHARED, fd_.get(), 0));
}

void Hal3A::start()
{
	init();
	getInitialInfo();
	config();
	startInternal();
}

void Hal3A::init()
{
	NSCam::IHalSensorList *const pHalSensorList = NSCam::IHalSensorList::get();
	pHalSensorList->searchSensors();
	peripheralController_ = mtk::hal3a::IPeripheralController::GetInstance(sensor_idx_);
	peripheralController_->notifyPowerOn();

	NVRAM_SENSOR_IDX_INFO _sensorIdxInfo;
	// TODO(chenghaoyang): Abstract sensors' information to support different sensor modules.
	if (sensor_idx_ == 0) { // back camera
		_sensorIdxInfo.sensorDev = 1;
		_sensorIdxInfo.sensorId = 4921;
		_sensorIdxInfo.facing = 0;
		_sensorIdxInfo.moduleId = 0;
		_sensorIdxInfo.sensorName = "HI1339_MIPI_RAW";
	} else { // front camera
		_sensorIdxInfo.sensorDev = 2;
		_sensorIdxInfo.sensorId = 2211;
		_sensorIdxInfo.facing = 1;
		_sensorIdxInfo.moduleId = 0;
		_sensorIdxInfo.sensorName = "GC08A3_MIPI_RAW";
	}
	NvBufUtil::initSensorInfo(sensor_idx_, _sensorIdxInfo);

	m_hal3a_ = mtk::hal3a::IHal3A::GetInstance(sensor_idx_);
	m_isp_hal_ = mtk::ispcf::IHalISPAdapter::createInstance(sensor_idx_);
	mtk::hal3a::v1_0::mtk_3a_init init = {};

	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetSensorStaticInfo,
					   (intptr_t)(&init.sensor_static_info_array), 0,
					   0, 0);

	// Get sensor init dynamic info
	peripheralController_->NotifyEvent(
		mtk::hal3a::IPeripheralController::kGetSensorInitialDynamicInfo,
		(intptr_t)(&init.sensor_init_dynamic_info), 0, 0, 0);
	// get cam calibration data
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetCalData,
					   CAMERA_CAM_CAL_DATA_MODULE_VERSION,
					   (intptr_t)(&init.cal_data), 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetCalData,
					   CAMERA_CAM_CAL_DATA_3A_GAIN,
					   (intptr_t)(&init.cal_aa), 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetCalData,
					   CAMERA_CAM_CAL_DATA_SHADING_TABLE,
					   (intptr_t)(&init.cal_lsc), 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetCalData,
					   CAMERA_CAM_CAL_DATA_PDAF,
					   (intptr_t)(&init.cal_pdaf), 0, 0);

	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kIsAfSupported,
					   (intptr_t)(&init.is_vcm_support),
					   (intptr_t)(&init.is_ozoom_support), 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kIsIrcutSupported,
					   (intptr_t)(&init.is_ircut_support), 0, 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kIsIrisSupported,
					   (intptr_t)(&init.is_iris_support), 0, 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetFlashCapability,
					   (intptr_t)(&init.flash_capability), 0, 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kFlashSupport,
					   (intptr_t)(&init.flash_hw_support), 0, 0, 0);

	// Stereo Feature: init
	// m_Sync3AFlowCtrl->Init(sensor_idx_);

	m_hal3a_->Init(init);
}

void Hal3A::getInitialInfo()
{
	mtk::hal3a::v1_0::mtk_3a_config config = {};

	// `config.static_meta` is not used in the proprietary library.

	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetIrisData,
					   (intptr_t)(&config.iris_info), 0, 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetSensorStaticInfo,
					   (intptr_t)(&config.sensor_static_info_array),
					   0, 0, 0);
	peripheralController_->NotifyEvent(
		mtk::hal3a::IPeripheralController::kGetSensorInitialDynamicInfo,
		(intptr_t)(&config.sensor_init_dynamic_info), 0, 0, 0);

	// Replace m_meta_helper.convertToConfigRequest
	config.ae_target_mode = 0;
	config.isp_fus_num = 0;
	config.ae_sensor_min_fps = 0;
	config.ae_sensor_max_fps = 0;
	config.multiexp_hdr_mode = 0;
	config.ae_valid_exp = 0;
	config.tuning_feature = 0;
	config.tuning_feature_cap = 0;
	config.target_size_w = 0;
	config.target_size_h = 0;
	config.capture_feature = 5316725;
	config.ae_min_fps = 5000;
	config.ae_max_fps = 30000;
	config.zoom_ratio = 100;
	config.capture_intent = 1;
	config.aov_enable = 0;
	config.custom_feature = 0;
	config.custom_feature_cap = 0;
	config.is_subsample_mode = 0;

	config.sensor_idx = sensor_idx_;

	config.control_config.subsample_count = 1;
	config.control_config.request_count = 1;
	config.control_config.sensor_mode = 0;
	config.control_config.sensor_id = 0;
	config.control_config.bit_mode = 1;

	config.fno = 1.790000;
	config.focal_length = 4.710000;
	config.sensor_mode = 0;

	NSCam::IHalSensorList *const pHalSensorList = NSCam::IHalSensorList::get();
	if (!pHalSensorList) {
		LOG(MtkISP7, Fatal) << "Couldn't get IHalSensorList";
		return;
	}
	config.control_config.sensor_dev = pHalSensorList->querySensorDevIdx(sensor_idx_);
	if (sensor_idx_ == 0) { // back camera
		config.control_config.sensor_tg_width = 4208;
		config.control_config.sensor_tg_height = 3120;

		config.orientation.sensor_orientation = 0;
		config.orientation.facing = 1;
		config.tg_width = 4208;
		config.tg_height = 3120;
	} else { // front camera
		config.control_config.sensor_tg_width = 3264;
		config.control_config.sensor_tg_height = 2448;

		config.orientation.sensor_orientation = 270;
		config.orientation.facing = 0;
		config.tg_width = 3264;
		config.tg_height = 2448;
	}

	m_hal3a_->GetHwInitialSetting(config, initialSetting_);
	m_hal3a_->GetResultForceUpdate(r3AResult_);
	m_hal3a_->Set2aDataToLastPool();
}

void Hal3A::config()
{
	mtk::hal3a::v1_0::mtk_3a_config config = {};

	// `config.static_meta` is not used in the proprietary library.

	peripheralController_->NotifyEvent(
		mtk::hal3a::IPeripheralController::kGetSensorConfigDynamicInfo, 0 /* According to dump */,
		(intptr_t)(&config.sensor_config_dynamic_info), 0, 0);

	// Get sensor perframe dynamic info
	peripheralController_->NotifyEvent(
		mtk::hal3a::IPeripheralController::kGetSensorPerframeDynamicInfo,
		(intptr_t)(&config.sensor_perframe_dynamic_info), 0, 0, 0);
	peripheralController_->NotifyEvent(mtk::hal3a::IPeripheralController::kGetIrisData,
					   (intptr_t)(&config.iris_info), 0, 0, 0);

	// Replace m_meta_helper.convertToConfigRequest
	config.ae_target_mode = 0;
	config.isp_fus_num = 0;
	config.ae_sensor_min_fps = 0;
	config.ae_sensor_max_fps = 0;
	config.multiexp_hdr_mode = 0;
	config.ae_valid_exp = 0;
	config.tuning_feature = 0;
	config.tuning_feature_cap = 0;
	config.target_size_w = 0;
	config.target_size_h = 0;
	config.capture_feature = 5316725;
	config.ae_min_fps = 5000;
	config.ae_max_fps = 30000;
	config.zoom_ratio = 100;
	config.capture_intent = 1;
	config.aov_enable = 0;
	config.custom_feature = 0;
	config.custom_feature_cap = 0;
	config.is_subsample_mode = 0;

	// TODO: remove if it's always 0.
	if (config.aov_enable) {
		peripheralController_->NotifyEvent(
			mtk::hal3a::IPeripheralController::kDisableSensorProvider, 0, 0, 0, 0);
	}

	config.control_config.subsample_count = 1;
	config.control_config.request_count = 1;
	config.control_config.sensor_mode = 0;
	config.control_config.sensor_id = 0;
	config.control_config.bit_mode = 1;

	NSCam::IHalSensorList *const pHalSensorList = NSCam::IHalSensorList::get();
	if (!pHalSensorList) {
		LOG(MtkISP7, Fatal) << "Couldn't get IHalSensorList";
		return;
	}

	config.control_config.sensor_dev = pHalSensorList->querySensorDevIdx(sensor_idx_);
	if (sensor_idx_ == 0) { // back camera
		config.control_config.sensor_tg_width = 4208;
		config.control_config.sensor_tg_height = 3120;
	} else { // front camera
		config.control_config.sensor_tg_width = 3264;
		config.control_config.sensor_tg_height = 2448;
	}

	config.sensor_idx = sensor_idx_;
	if (sensor_idx_ == 0) { // back camera
		config.sub_flash_enable = 0;
		config.orientation.sensor_orientation = 0;
		config.orientation.facing = 1;
		config.tg_width = 4208;
		config.tg_height = 3120;
		config.fno = 1.790000;
		config.focal_length = 4.710000;
		config.feature_mode = 0;
		config.sensor_mode = 0;
	} else { // front camera
		config.sub_flash_enable = 1;
		config.orientation.sensor_orientation = 270;
		config.orientation.facing = 0;
		config.tg_width = 3264;
		config.tg_height = 2448;
		config.fno = 1.790000;
		config.focal_length = 4.710000;
		config.feature_mode = 0;
		config.sensor_mode = 0;
	}

	m_hal3a_->Config(config);

	m_hal3a_->GetResult(r3AResult_);

	m_isp_hal_->setSensorMode(NSIspTuning::ESensorMode_Preview);

	mtk_isp_config configInfo; // TODO: fill in _appMeta & _halMeta. sensor_dev & sensor_idx are not filled by mtk's hal as well.
	if (sensor_idx_ == 0) { // back camera
		configInfo.tg_width = 4208;
		configInfo.tg_height = 3120;
		configInfo.sub_sample_count = 1;
		configInfo.direct_yuv_path = 0;
		configInfo.yuv_after_rrz = 0;
	} else { // front camera
		configInfo.tg_width = 3264;
		configInfo.tg_height = 2448;
		configInfo.sub_sample_count = 1;
		configInfo.direct_yuv_path = 0;
		configInfo.yuv_after_rrz = 0;
	}

	m_isp_hal_->config4Camsys(configInfo);
}

void Hal3A::startInternal()
{
	peripheralController_->NotifyEvent(
		mtk::hal3a::IPeripheralController::kFocusMove,
		r3AResult_.af_result.lens_position, 0 /* arg1: ozoom_param is not used */, 0, 0);
	peripheralController_->NotifyEvent(
		mtk::hal3a::IPeripheralController::kFlashInitialDuty,
		(intptr_t)r3AResult_.flash_result.duty_setting, 0, 0, 0);
	peripheralController_->NotifyEvent(
		mtk::hal3a::IPeripheralController::kNotifyOisStreamOn, 0, 0, 0, 0);

	mtk::hal3a::v1_0::mtk_3a_start r_3a_start;
	m_hal3a_->Start(r_3a_start);
}

void Hal3A::doCalculation(FrameBuffer *statistics0, uint64_t timestamp)
{
	mtk::hal3a::v1_0::mtk_3a_param r_3a_param;
	// TODO: get request_id
	static uint64_t request_id = 0;
	const auto current_request_id = request_id++;

	// TODO: get parameters for SetParam properly
	r_3a_param.request_id = current_request_id;
	r_3a_param.active_items = 59;
	r_3a_param.updated = true;
	r_3a_param.is_dummy_request = false;
	r_3a_param.control_mode = 1;
	r_3a_param.scene_mode = 0;
	r_3a_param.capture_intent = 1;
	r_3a_param.inflight_capture = 0;
	r_3a_param.ae_lock = 0;
	r_3a_param.ae_mode = 1;
	r_3a_param.ae_precap_trigger = 0;
	r_3a_param.ae_anti_banding_mode = 3;
	r_3a_param.sensor_frame_duration = 33333333;
	r_3a_param.sensor_exposure = 10000000;
	r_3a_param.sensor_sensitivity = 100;
	r_3a_param.ae_exp_index = 0;
	r_3a_param.ae_exp_step = 0.500000;
	r_3a_param.ae_min_fps = 5000;
	r_3a_param.ae_max_fps = 30000;
	r_3a_param.ae_region.count = 1;
	r_3a_param.black_level_lock = 0;
	r_3a_param.set_converge = 0;
	r_3a_param.awb_lock = 0;
	r_3a_param.awb_mode = 1;
	r_3a_param.color_correct_mode = 1;
	for (unsigned int i = 0; i < kMaxColorGainsCount; i++) {
		r_3a_param.color_correct_gain[i] = 1.0;
	}
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			if (i != j)
				continue;

			r_3a_param.color_correct_mat[i * 3 + j] = 1.0;
		}
	}
	r_3a_param.awb_default_pregain1 = 0;
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.af_mode = 4;
	} else { // front camera
		r_3a_param.af_mode = 0;
	}
	r_3a_param.af_trigger = 0;
	r_3a_param.af_focus_distance = 0.000000;
	r_3a_param.af_zoom_ratio = 0;
	r_3a_param.af_zoom_stop = 0;
	r_3a_param.af_region.count = 0;
	r_3a_param.lens_ois_mode = 0;
	r_3a_param.af_notify_timeout = 0;
	r_3a_param.strobe_mode = 0;
	r_3a_param.flash_type = mtk::hal3a::kNoFlash;
	r_3a_param.shading_mode = 1;
	r_3a_param.shadingmap_mode = 0;
	r_3a_param.tonemap_mode = 1;
	r_3a_param.lock_ratio = 0;
	r_3a_param.lock_shading = 0;
	r_3a_param.sensor_test_patten_mode = 0;
	// r_3a_param.sensor_test_patten_data = { valid = 1, Channel_R = 0, Channel_Gr = 0, Channel_Gb = 0, Channel_B = 0};
	r_3a_param.sensor_test_patten_data.valid = 1;
	r_3a_param.prolong_frame_length = 0;
	r_3a_param.face_detect_mode = 1;
	r_3a_param.face_detect_force = 1;
	r_3a_param.ae_custom_pline_mode = 0;
	r_3a_param.ae_manual_pline_idx = 0;
	r_3a_param.ae_iso_speed_mode = 0;
	r_3a_param.ae_meter_mode = 0;
	r_3a_param.ae_convergence_speed = 0;
	r_3a_param.ae_custom_metering_table_mode = 0;
	r_3a_param.ae_clusive_roi_mode = 0;
	// Empty
	// r_3a_param.ae_custom_metering_table =
	// r_3a_param.ae_pline_anchor =
	// r_3a_param.ae_clusive_roi =
	// r_3a_param.ae_manualarea_roi =
	r_3a_param.flash_cali_en = 0;
	r_3a_param.awb_convergence_speed = 6;
	r_3a_param.awb_warmstart_enable = 1;
	//  common tag
	r_3a_param.repeat_tag = 0;
	r_3a_param.get_exif = 0;
	r_3a_param.is_center_region = 0;
	r_3a_param.imgo_type = 1;
	r_3a_param.app_mode = 0;
	r_3a_param.zoom_ratio = 100;
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.target_size_w = 1280;
		r_3a_param.target_size_h = 960;
	} else { // front camera
		r_3a_param.target_size_w = 1440;
		r_3a_param.target_size_h = 1080;
	}
	// r_3a_param.prv_crop_region = { left = 0, top = 0, right = 3264, bottom = 2448, weight = 0 };
	// r_3a_param.prv_crop_normalize_region = { left = 0, top = 0, right = 3264, bottom = 2448, weight = 0 };
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.prv_crop_region.right = 4208;
		r_3a_param.prv_crop_region.bottom = 3120;

		r_3a_param.prv_crop_normalize_region.right = 4208;
		r_3a_param.prv_crop_normalize_region.bottom = 3120;
	} else { // front camera
		r_3a_param.prv_crop_region.right = 3264;
		r_3a_param.prv_crop_region.bottom = 2448;

		r_3a_param.prv_crop_normalize_region.right = 3264;
		r_3a_param.prv_crop_normalize_region.bottom = 2448;
	}
	r_3a_param.low_fps = 0;
	r_3a_param.remosaic_enable = 0;
	// ae tag
	r_3a_param.ae_target_mode = 0;
	// Empty
	// r_3a_param.ae_exp_level =   custom_param:
	r_3a_param.custom_param.id = 0;
	r_3a_param.custom_param.gain_align = 0;
	r_3a_param.custom_param.target_gain_x1024 = 0;
	r_3a_param.custom_param.ev0_isp_gain_x1024 = 0;
	r_3a_param.custom_param.ev0_sensor_gain_x1024 = 0;
	r_3a_param.custom_param.ev0_exptime_us = 0;
	r_3a_param.custom_param.exptime_limit_us = 0;
	r_3a_param.custom_param.gain_limit_x1024 = 0;
	r_3a_param.custom_param.frame_index = 0;
	r_3a_param.custom_param.end_frame_index = 0;
	r_3a_param.ae_valid_exp = 0;
	r_3a_param.denoise_mode = 0;
	r_3a_param.ae_sensor_min_fps = 5000;
	r_3a_param.ae_sensor_max_fps = 30000;
	r_3a_param.ae_hal_exp_index = 0;
	r_3a_param.manual_ev_ctrl.frame_count = 0;
	r_3a_param.manual_ev_ctrl.frame_index = 0;
	r_3a_param.manual_ev_ctrl.ev_step = 0.000000;
	r_3a_param.manual_ev_ctrl.ev_begin = 0.000000;
	r_3a_param.manual_ev_ctrl.ae_lock = 0;
	r_3a_param.mwb_cct = 0;
	r_3a_param.pause_af = 0;
	r_3a_param.gyro_valid = 0;
	r_3a_param.acce_valid = 0;
	r_3a_param.light_valid = 0;
	r_3a_param.als_all_valid[0] = 0;
	r_3a_param.als_all_valid[1] = 0;
	r_3a_param.flicker_valid[0] = 0;
	r_3a_param.flicker_valid[1] = 0;
	r_3a_param.color_valid[0] = 0;
	r_3a_param.color_valid[1] = 0;
	r_3a_param.is_stagger = 0;
	r_3a_param.is_mstream = 0;
	r_3a_param.flash_info.isAvailable = 0;
	r_3a_param.flash_info.isOn = 0;
	r_3a_param.flash_info.inCharge = 0;
	r_3a_param.flash_info.battVol = 0;
	r_3a_param.flash_info.isLowPower = 0;
	r_3a_param.flash_info.chargerStatus = 0;
	r_3a_param.flash_info.driverFault = 0;
	r_3a_param.flash_info.timeInfo.mfStartTime = 0;
	r_3a_param.flash_info.timeInfo.mfEndTime = 0;
	r_3a_param.flash_info.timeInfo.mfTimeout = 0;
	r_3a_param.flash_info.timeInfo.mfTimeoutLt = 0;
	r_3a_param.flash_info.timeInfo.mfIsTimeout = 0;
	r_3a_param.is_flash_force_off = 0;
	r_3a_param.flash_full_cali_en = 0;
	r_3a_param.flash_fast_cali_en = 0;
	r_3a_param.isp_fus_num = 0;
	r_3a_param.subsample_sync_info = 0;
	r_3a_param.multiexp_hdr_mode = 0;
	r_3a_param.hdr_mode = 0;
	r_3a_param.fast_switch_param.sensor_mode = 0;
	// r_3a_param.fast_switch_param.tg_size = { w = 3264, h = 2448 };
	// r_3a_param.fast_switch_param.full_tg_size = { w = 3264, h = 2448 };
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.fast_switch_param.tg_size.w = 4208;
		r_3a_param.fast_switch_param.tg_size.h = 3120;

		r_3a_param.fast_switch_param.full_tg_size.w = 4208;
		r_3a_param.fast_switch_param.full_tg_size.h = 3120;
	} else { // front camera
		r_3a_param.fast_switch_param.tg_size.w = 3264;
		r_3a_param.fast_switch_param.tg_size.h = 2448;

		r_3a_param.fast_switch_param.full_tg_size.w = 3264;
		r_3a_param.fast_switch_param.full_tg_size.h = 2448;
	}
	r_3a_param.fast_switch_param.ae_target_mode_next = 0;
	r_3a_param.fast_switch_param.ae_valid_exp_next = 0;
	r_3a_param.fast_switch_param.ae_sensor_mode_next = 0;
	r_3a_param.fast_switch_param.is_seamless = 0;
	r_3a_param.fast_switch_param.seam_policy = 0;
	// TODO: Track the data flow from MtkCameraFaceMetadata
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.face_num = 0;
		r_3a_param.is_fd_ready = 0;
	} else { // front camera
		r_3a_param.face_num = 1;
		r_3a_param.is_fd_ready = 1;
	}
	r_3a_param.is_fd_enable = 1;
	r_3a_param.ot_info.is_valid = 0;
	r_3a_param.gf_info.is_valid = 0;
	r_3a_param.tuning_feature = 0;
	r_3a_param.sensor_feature = 0;
	r_3a_param.custom_feature = 0;
	r_3a_param.tuning_feature_cap = 0;
	r_3a_param.custom_feature_cap = 0;
	r_3a_param.custom_00 = 0;
	r_3a_param.sync2a_mode = 0;
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.master_idx = 0;
		r_3a_param.awb_master_idx = 0;
	} else { // front camera
		r_3a_param.master_idx = 1;
		r_3a_param.awb_master_idx = 1;
	}
	r_3a_param.iris_info.iris_status = 0;
	r_3a_param.iris_info.iris_position = 0;
	r_3a_param.iris_info.previous_iris_position = 0;
	r_3a_param.iris_info.moving_timestamp = 0;
	r_3a_param.iris_info.previous_moving_timestamp = 0;
	r_3a_param.iris_info.fn_cali = 0.000000;

	m_hal3a_->SetParam(r_3a_param);

	mtk::hal3a::v1_0::mtk_3a_request r_3a_request;
	if (statistics0->planes().empty()) {
		LOG(MtkISP7, Fatal) << "Empty statistics0";
		return;
	}

	r_3a_request.scenario = mtk::hal3a::Mtk3AScenario::kPreview;
	r_3a_request.buf_info.request_id = current_request_id;
	r_3a_request.buf_info.sof_timestamp = timestamp;

	r_3a_request.stt_buf.fd = statistics0->planes()[0].fd.get();

	MappedFrameBuffer mappedFrameBuffer(statistics0, MappedFrameBuffer::MapFlag::Read);
	r_3a_request.stt_buf.buf = reinterpret_cast<const mtk_cam_uapi_meta_raw_stats_0 *>(
		mappedFrameBuffer.planes()[0].data());

	m_hal3a_->DoCalculation(r_3a_request);

	mtk::hal3a::v1_0::mtk_3a_result r_3a_result;
	m_hal3a_->GetResult(r_3a_result);
	r3AResult_ = r_3a_result;

	mtk::hal3a::v1_0::mtk_hal3a_metaset metaSet; // TODO: skip_exposure_setting, appMeta, halMeta
	std::vector<mtk::hal3a::v1_0::mtk_hal3a_metaset *> requestQ;
	requestQ.push_back(&metaSet);

	mtk::isphal::IspTuningCamsysControl ctrl;
	ctrl.fgForce = false; // TODO: check if it's a dummy frame.
	ctrl.fgDue = false; // TODO: check if 3a finishes calculation on time.

	mtk::isphal::IspTuningBufferP1 tuning_data;
	*meta_addr_ = r_3a_result.raw_meta;

	mtk::isphal::Buffer regBuf1((intptr_t)meta_addr_, fd_.get(), 0, kMetaSize);
	tuning_data.p1_meta_buffer = regBuf1;
	m_isp_hal_->getCamSysMetaTuning(request_id, r3AResult_.request_id, requestQ, ctrl, tuning_data);

	r3AResult_.raw_meta = *meta_addr_;
}

std::pair<uint32_t, uint32_t> Hal3A::getExposureAndGain()
{
	// TODO: consider delay
	ae_exposure_setting_table ae_table = r3AResult_.ae_result.ae_exp_table;
	if (ae_table.cnt == 0)
		return std::make_pair(0, 0);

	auto *const pSensorList = NSCam::IHalSensorList::get();

	if (!pSensorList)
		return std::make_pair(0, 0);

	auto dev_idx = pSensorList->querySensorDevIdx(sensor_idx_);
	auto *const pHalSensor = pSensorList->createSensor("pipemgrPerframeSet", sensor_idx_);

	if (!pHalSensor)
		return std::make_pair(0, 0);

	for (int exp = 0; exp < AE_EXP_MODE_MAX_T; ++exp) {
		if (ae_table.table[exp].mode <= 0)
			continue;

		// exp should be 4: AE_EXP_MODE_NE_T

		uint32_t gain = pHalSensor->convert_gain(dev_idx, ae_table.table[exp].afe_gain);
		uint32_t ex = ae_table.table[exp].exposure_line;

		pHalSensor->destroyInstance("pipemgrPerframeSet");

		return std::make_pair(ex, gain);
	}

	pHalSensor->destroyInstance("pipemgrPerframeSet");
	return std::make_pair(0, 0);
}

} /* namespace libcamera */
