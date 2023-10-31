// Copyright 2023 The ChromiumOS Authors
// Use of this source code is governed by a BSD-style license that can be
// found in the LICENSE file.

#include "sensor_info.h"

#include <algorithm>
#include <cstdint>
#include <memory>
#include <regex>
#include <string>

#include <libcamera/base/log.h>

#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/imgsensor_src/imgsensor_info_custom.h"
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/inc/camera_custom_imgsensor_cfg.h"
#include "platform/mtkisp7/mtkcam-interfaces/include/mtkcam-interfaces/hw/sensor/imgsensor_info.h"
#include "platform/mtkisp7/halsensor_helper.h"
#include "platform/mtkisp7/imgsensor_info_helper.h"
namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

std::shared_ptr<SensorInfo> SensorInfo::sensor_info_[MAX_SENSOR_INFO_COUNT] = {
	nullptr
};
std::map<int, CamSysDevice *> SensorInfo::idx_camsys_map;
std::vector<std::shared_ptr<NSCam::SensorStaticInfo>>
	SensorInfo::nscam_sensor_static_info_;
std::vector<CamSysDevice *> SensorInfo::camSysDevices_;
SensorInfo::SensorInfo(int sensor_idx)
	: m_sensor_index(sensor_idx),
	  m_sensor_dev(0),
	  m_sensor_id(0),
	  m_module_id(0),
	  m_cal_drv(CamCalDrvBase::createInstance())
{
}

void SensorInfo::init(int sensor_dev, int sensor_id, int module_id)
{
	m_sensor_dev = sensor_dev;
	m_sensor_id = sensor_id;
	m_module_id = module_id;
}
std::map<int, int> sensorId_idx_map = { { 0, 1 }, { 1, 0 } };

std::shared_ptr<SensorInfo> SensorInfo::getInstance(int sensor_idx)
{
	if (!sensor_info_[sensor_idx]) {
		sensor_info_[sensor_idx].reset(new SensorInfo(sensor_idx));
	}
	return sensor_info_[sensor_idx];
}

void SensorInfo::add_sensor(CamSysDevice *camSysDevice, int size)
{
	if (camSysDevices_.size()) {
		return;
	}
	for (int i = 0; i < size; i++) {
		camSysDevices_.push_back(camSysDevice + i);
		std::shared_ptr<NSCam::SensorStaticInfo> s =
			std::shared_ptr<NSCam::SensorStaticInfo>(new NSCam::SensorStaticInfo);
		nscam_sensor_static_info_.push_back(s);
	}

	auto getId = [](CamSysDevice *A) {
		std::string reg_str(R"(.*sensor([\d])@[\d])");
		std::regex reg(reg_str);
		// Example:
		//  /base/soc/i2c@11ec0000/sensor1@1 -> id = 1
		//  /base/soc/i2c@11ec1000/sensor0@1 -> id = 0
		std::smatch m;
		LOG(MtkISP7, Info) << "Camera ID = " << A->cameraId();
		if (std::regex_match(A->cameraId(), m, reg)) {
			LOG(MtkISP7, Info) << "Match regex: " << reg_str << "id = " << m[1].str();
			return std::stoi(m[1].str());
		}
		return 0;
	};
	std::sort(camSysDevices_.begin(), camSysDevices_.end(),
		  [getId](CamSysDevice *A, CamSysDevice *B) {
			  return getId(A) < getId(B);
		  });
	for (int i = 0; i < (int)camSysDevices_.size(); ++i) {
		idx_camsys_map[getId(camSysDevices_[i])] = camSysDevices_[i];
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
		if (i >= (int)camSysDevices_.size()) {
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
			     void *pCamCalData)
{
	unsigned int size = 0;
	switch (cal_enum) {
	case CAMERA_CAM_CAL_DATA_MODULE_VERSION:
		size = sizeof(CAM_CAL_MODULE_VERSION_STRUCT);
		break;
	case CAMERA_CAM_CAL_DATA_PART_NUMBER:
		size = sizeof(CAM_CAL_PART_NUM_STRUCT);
		break;
	case CAMERA_CAM_CAL_DATA_SHADING_TABLE:
		size = sizeof(CAM_CAL_LSC_DATA_STRUCT);
		break;
	case CAMERA_CAM_CAL_DATA_3A_GAIN:
		size = sizeof(CAM_CAL_2A_DATA_STRUCT);
		break;
	case CAMERA_CAM_CAL_DATA_STEREO_DATA:
		size = sizeof(CAM_CAL_STEREO_DATA_STRUCT);
		break;
	case CAMERA_CAM_CAL_DATA_PDAF:
		size = sizeof(CAM_CAL_PDAF_DATA_STRUCT);
		break;
	case CAMERA_CAM_CAL_DATA_DUMP:
		size = sizeof(CAM_CAL_DATA_STRUCT);
		break;
	case CAMERA_CAM_CAL_DATA_LENS_ID:
		size = sizeof(CAM_CAL_LENS_ID_STRUCT);
		break;
	default:
		LOG(MtkISP7, Error) << "Size of type(" << cal_enum << ") is not defined";
		break;
	}
	int result = 0;
	if (m_cal_drv && pCamCalData) {
		result = m_cal_drv->GetCamCalCalDataV2(
			m_sensor_dev, cal_enum, reinterpret_cast<void *>(pCamCalData), size);
	}
	LOG(MtkISP7, Info) << "result: " << result;
	return result;
}

bool SensorInfo::is_af_support()
{
	if (idx_camsys_map.find(m_sensor_index) != idx_camsys_map.end()) {
		bool hasAF = idx_camsys_map[m_sensor_index]->getCameraLens();
		return hasAF;
	}
	return false;
}

void SensorInfo::construct_sensor_static_info(
	int index, std::shared_ptr<NSCam::SensorStaticInfo> pSensorStaticInfo)
{
	int sensorId_idx = sensorId_idx_map[index];
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
	pSensorStaticInfo->sensorMBusCode = camSysDevices_[index]->mbusCode();
}

} // namespace libcamera