/*
 * Copyright (C) 2023; Google Inc.
 *
 * hal_3a.cpp - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#include "hal_3a.h"

#include <libcamera/base/log.h>

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

Hal3A::Hal3A(const uint32_t sensor_idx)
	: sensor_idx_(sensor_idx)
{
}

void Hal3A::start()
{
	init();
}

void Hal3A::init()
{
	NSCam::IHalSensorList *const pHalSensorList = NSCam::IHalSensorList::get();
	pHalSensorList->searchSensors();
	peripheralController_ = mtk::hal3a::IPeripheralController::GetInstance(sensor_idx_);
	peripheralController_->notifyPowerOn();

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

} /* namespace libcamera */
