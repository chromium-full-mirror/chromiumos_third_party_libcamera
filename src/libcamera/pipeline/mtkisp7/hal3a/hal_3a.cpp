/*
 * Copyright (C) 2023; Google Inc.
 *
 * hal_3a.cpp - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#include "hal_3a.h"

#include <libcamera/base/log.h>

#include "mtkcam-core/aaa/include/nvbuf_util.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

Hal3A::Hal3A(const uint32_t sensor_idx)
	: sensor_idx_(sensor_idx)
{
}

void Hal3A::start()
{
	init();
	getInitialInfo();
	config();
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
}

} /* namespace libcamera */
