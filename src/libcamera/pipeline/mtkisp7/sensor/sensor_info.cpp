// Copyright 2023 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "sensor_info.h"

#include <algorithm>
#include <cstdint>
#include <fstream>
#include <memory>
#include <regex>
#include <string>

#include <libcamera/base/log.h>

#include "platform/mtkisp7/cam_cal_helper.h"
#include "platform/mtkisp7/halsensor_helper.h"
#include "platform/mtkisp7/imgsensor_info_helper.h"
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/imgsensor_src/imgsensor_info_custom.h"
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/inc/camera_custom_imgsensor_cfg.h"
#include "platform/mtkisp7/mtkcam-interfaces/include/mtkcam-interfaces/hw/sensor/imgsensor_info.h"
#include "platform/mtkisp7/platform_utils.h"
namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

std::shared_ptr<SensorInfo> SensorInfo::sensor_info_[MAX_SENSOR_INFO_COUNT] = {
	nullptr
};
std::vector<std::shared_ptr<NSCam::SensorStaticInfo>>
	SensorInfo::nscam_sensor_static_info_;
std::vector<SensorInfo::CamSysData> SensorInfo::camSysDataArray_;

/*
map senidx to sensnorId
0 -> GC08A3_SENSOR_ID
1 -> HI1339_SENSOR_ID
2 -> GC05A2_SENSOR_ID
*/

std::map<int, int> sensorId_idx_map_geralt = { { 0, 1 }, { 1, 0 } };
std::map<int, int> sensorId_idx_map_ciri = { { 0, 0 }, { 1, 2 } };
SensorInfo::SensorInfo(int sensor_idx)
	: m_sensor_index(sensor_idx),
	  m_sensor_dev(0),
	  m_sensor_id(0),
	  m_module_id(0)
{
}

void SensorInfo::init(int sensor_dev, int sensor_id, int module_id)
{
	m_sensor_dev = sensor_dev;
	m_sensor_id = sensor_id;
	m_module_id = module_id;
}

std::shared_ptr<SensorInfo> SensorInfo::getInstance(int sensor_idx)
{
	if (!sensor_info_[sensor_idx]) {
		sensor_info_[sensor_idx].reset(new SensorInfo(sensor_idx));
	}
	return sensor_info_[sensor_idx];
}

void SensorInfo::add_sensor(const std::vector<CamSysData> &camSysDataArray)
{
	if (!camSysDataArray_.empty())
		return;

	camSysDataArray_ = camSysDataArray;
	for (unsigned i = 0; i < camSysDataArray_.size(); i++) {
		std::shared_ptr<NSCam::SensorStaticInfo> s =
			std::shared_ptr<NSCam::SensorStaticInfo>(new NSCam::SensorStaticInfo);
		nscam_sensor_static_info_.push_back(s);
	}

	for (int i = 0; i < (int)nscam_sensor_static_info_.size(); ++i) {
		std::shared_ptr<NSCam::SensorStaticInfo> s = nscam_sensor_static_info_[i];
		construct_sensor_static_info(i, s);
	}
}

void SensorInfo::get_sensor_static_info(
	std::array<mtk::hal3a::SensorStaticInfo, kMaxSensorCnt> *
		nscam_sensor_static_info_array)
{
	*nscam_sensor_static_info_array = {};
	for (int i = 0; i < (int)kMaxSensorCnt; i++) {
		if (i >= (int)camSysDataArray_.size()) {
			break;
		}
		nscam_sensor_static_info_array->at(i).index = i;
		nscam_sensor_static_info_array->at(i).dev_id = 1 << i;
		nscam_sensor_static_info_array->at(i).sensor_id =
			nscam_sensor_static_info_[i]->sensorDevID;
		// TODO, Query module id from EEProm
		nscam_sensor_static_info_array->at(i).module_id = 0;
		nscam_sensor_static_info_array->at(i).orientation =
			nscam_sensor_static_info_[i]->facingDirection;
		nscam_sensor_static_info_array->at(i).info = *nscam_sensor_static_info_[i];
		LOG(MtkISP7, Info)
			<< "index:" << nscam_sensor_static_info_array->at(i).index
			<< ", dev_id:" << nscam_sensor_static_info_array->at(i).dev_id
			<< ", sensor_id:" << nscam_sensor_static_info_array->at(i).sensor_id
			<< ", module_id:" << nscam_sensor_static_info_array->at(i).module_id
			<< ", orientation:"
			<< nscam_sensor_static_info_array->at(i).orientation;
	}
}
void SensorInfo::get_sensor_initial_dynamic_info(
	mtk::hal3a::SensorInitialDynamicInfo *sensor_dynamic_info)
{
	std::shared_ptr<HalSensorHelper> hal_sensor_helper =
		HalSensorHelper::getInstance();
	hal_sensor_helper->get_sensor_initial_dynamic_info(
		m_sensor_index, m_sensor_dev, sensor_dynamic_info);
}

void SensorInfo::get_sensor_perframe_dynamic_info(
	mtk::hal3a::SensorPerframeDynamicInfo *sensor_perframe_dynamic_info)
{
	std::shared_ptr<HalSensorHelper> hal_sensor_helper =
		HalSensorHelper::getInstance();
	hal_sensor_helper->get_sensor_perframe_dynamic_info(
		m_sensor_index, m_sensor_dev, sensor_perframe_dynamic_info);
}

int SensorInfo::get_cal_data(ENUM_CAMERA_CAM_CAL_TYPE_ENUM cal_enum,
			     void *a_pCamCalData)
{
	return CamCalHelper::getInstance()->get_cal_data(cal_enum, m_sensor_id, m_sensor_dev, a_pCamCalData);
}

bool SensorInfo::is_af_support()
{
	if (m_sensor_index >= camSysDataArray_.size()) {
		LOG(MtkISP7, Error) << "Invalid m_sensor_idx => " << m_sensor_index;
		return false;
	} else {
		return camSysDataArray_[m_sensor_index].has_af;
	}
}

void SensorInfo::construct_sensor_static_info(
	int index, std::shared_ptr<NSCam::SensorStaticInfo> pSensorStaticInfo)
{
	int sensorId_idx = 0;
	switch (PlatformUtils::platform_) {
	case PlatformUtils::MtkISP7Platform::NONE:
		LOG(MtkISP7, Fatal) << "Platform unconfigured";
		break;

	case PlatformUtils::MtkISP7Platform::GOOGLE:
		sensorId_idx = sensorId_idx_map_geralt[index];
		break;

	case PlatformUtils::MtkISP7Platform::LENOVO:
		sensorId_idx = sensorId_idx_map_ciri[index];
		break;
	}

	IMGSENSOR_SENSOR_IDX sensorIdx = (IMGSENSOR_SENSOR_IDX)index;
	struct imgsensor_info_struct *imgsensor_info;
	if (index >
	    int(sizeof(gImgsensor_info) / sizeof(struct imgsensor_info_struct))) {
		imgsensor_info = nullptr;
		LOG(MtkISP7, Error) << "Invalid index!!: " << index;
	} else {
		imgsensor_info = &gImgsensor_info[sensorId_idx];
	}
	querySensorInfo(sensorId_idx, sensorIdx, imgsensor_info, pSensorStaticInfo);
	pSensorStaticInfo->sensorMBusCode = camSysDataArray_[index].mbus_code;
}

} // namespace libcamera
