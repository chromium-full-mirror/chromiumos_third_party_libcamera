/*
 * Copyright (C) 2023 MediaTek Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "platform/mtkisp7/halsensor_helper.h"

#include "platform/mtkisp7/mtkcam-core/aaa/peripheralcontroller/include/PeripheralInfoDef.h"
#include "platform/mtkisp7/mtkcam-interfaces/include/mtkcam-interfaces/hw/sensor/IHalSensor.h"

#define LOG_ERR(fmt, ...) printf((fmt "\n"), ##__VA_ARGS__)
#define LOG_WRN(fmt, ...)
#define LOG_ADBDBG(...)
#define LOG_INF(...)
#define LOG_VRB(...)
#define LOG_DBG(...)

// kGetSensorInitialDynamicInfo
void HalSensorHelper::get_sensor_initial_dynamic_info(
	int m_sensor_index, int m_sensor_dev,
	mtk::hal3a::SensorInitialDynamicInfo *sensor_dynamic_info)
{
	NSCam::IHalSensor *p_halsensor = NSCam::IHalSensorList::get()->createSensor(
		"PeripheralCtrlInitGet", m_sensor_index);
	if (p_halsensor && sensor_dynamic_info) {
		p_halsensor->sendCommand(m_sensor_dev,
					 NSCam::SENSOR_CMD_GET_BASE_GAIN_ISO_AND_STEP,
					 (intptr_t)&sensor_dynamic_info->gain_iSO,
					 (intptr_t)&sensor_dynamic_info->gain_step_unit,
					 (intptr_t)&sensor_dynamic_info->gain_type);
		// SENSOR_CMD_GET_ANA_GAIN_TABLE
		{
			// Get table size first
			uint32_t *ptr_t = nullptr;
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_ANA_GAIN_TABLE,
				(intptr_t)&sensor_dynamic_info->real_table_size, (intptr_t)&ptr_t, 0);
			// Check table size and query gain table
			if (sensor_dynamic_info->real_table_size <=
			    sensor_dynamic_info->gain_table.size() * sizeof(uint32_t)) {
				uint32_t *ptr = sensor_dynamic_info->gain_table.data();
				p_halsensor->sendCommand(
					m_sensor_dev, NSCam::SENSOR_CMD_GET_ANA_GAIN_TABLE,
					(intptr_t)&sensor_dynamic_info->real_table_size, (intptr_t)&ptr, 0);
			} else {
				LOG_ERR("Real table size(%d) exceed definition(%d)!\n",
					sensor_dynamic_info->real_table_size,
					(int32_t)sensor_dynamic_info->gain_table.size());
			}
		}
		// Iterate all sensor mode and query info
		for (uint32_t i = 0; i < SENSOR_SCENARIO_ID_MAX; i++) {
			uint32_t sensor_mode = i;
			uint32_t min_line = 0, line_time = 0;
			auto &info = sensor_dynamic_info->by_sensor_mode_info[sensor_mode];
			p_halsensor->sendCommand(m_sensor_dev,
						 NSCam::SENSOR_CMD_GET_SENSOR_ROLLING_SHUTTER,
						 (intptr_t)&info.rolling_shutter,
						 (MUINTPTR) nullptr, (intptr_t)&sensor_mode);

			NSCam::IHalSensor::ConfigParam sensor_cfg;
			sensor_cfg.framerate = 0;
			sensor_cfg.scenarioId = sensor_mode;
			sensor_cfg.isBypassScenario = 0;
			sensor_cfg.isContinuous = 1;
			sensor_cfg.HDRMode = MFALSE;
			p_halsensor->sendCommand(m_sensor_dev, NSCam::SENSOR_CMD_GET_BINNING_TYPE,
						 (intptr_t)&info.bin_sum_ratio,
						 (intptr_t)&sensor_cfg, 0);
			p_halsensor->sendCommand(m_sensor_dev,
						 NSCam::SENSOR_CMD_GET_GAIN_RANGE_BY_SCENARIO,
						 (intptr_t)&sensor_mode, (intptr_t)&info.min_gain,
						 (intptr_t)&info.max_gain);
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_SENSOR_PDAF_CAPACITY,
				(intptr_t)&sensor_mode, (intptr_t)&info.is_sensor_mode_support_pd, 0);
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_SENSOR_PDAF_INFO,
				(intptr_t)&sensor_mode, (intptr_t)&info.pd_blk_info, 0);
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_SENSOR_VC_INFO2,
				(intptr_t)&info.vc_info_2, (intptr_t)&sensor_mode, 0);
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_SENSOR_CROP_WIN_INFO,
				(intptr_t)&sensor_mode, (intptr_t)&info.crop_info, 0);
			// Compute min_time_us and line_time
			p_halsensor->sendCommand(m_sensor_dev,
						 NSCam::SENSOR_CMD_GET_MIN_SHUTTER_BY_SCENARIO,
						 (intptr_t)&sensor_mode, (intptr_t)&min_line,
						 (intptr_t)&info.shutter_step);
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_PIXEL_CLOCK_FREQ_BY_SCENARIO,
				(intptr_t)&sensor_mode, (intptr_t)&info.pixel_clock, 0);
			p_halsensor->sendCommand(
				m_sensor_dev,
				NSCam::SENSOR_CMD_GET_FRAME_SYNC_PIXEL_LINE_NUM_BY_SCENARIO,
				(intptr_t)&sensor_mode, (intptr_t)&info.line_length, 0);
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_FINE_INTEGRATION_LINE,
				(intptr_t)&sensor_mode, (intptr_t)&info.fine_integ_line, 0);
			{
				if (info.shutter_step < 1) {
					info.shutter_step = 1;
				}
				if (info.pixel_clock != 0) {
					line_time = (1000000 * ((uint64_t)(info.line_length & 0xFFFF)) +
						     (info.pixel_clock / 1000 - 1)) /
						    (info.pixel_clock / 1000);
				}
				info.min_time_ns =
					(line_time * min_line) +
					((info.fine_integ_line * line_time) + 1000 - 1) / 1000;
				info.line_time = line_time;
			}
			//
			p_halsensor->sendCommand(
				m_sensor_dev, NSCam::SENSOR_CMD_GET_EXPOSURE_MARGIN_BY_SCENARIO,
				(intptr_t)&sensor_mode, (intptr_t)&info.exp_margin, 0);

			MUINT32 exp_type = VC_STAGGER_ME;
			p_halsensor->sendCommand(m_sensor_dev,
						 NSCam::SENSOR_CMD_GET_STAGGER_MAX_EXP_TIME,
						 (intptr_t)&sensor_mode, (intptr_t)&exp_type,
						 (intptr_t)&info.max_me_time_us);
			exp_type = VC_STAGGER_SE;
			p_halsensor->sendCommand(m_sensor_dev,
						 NSCam::SENSOR_CMD_GET_STAGGER_MAX_EXP_TIME,
						 (intptr_t)&sensor_mode, (intptr_t)&exp_type,
						 (intptr_t)&info.max_se_time_us);
			LOG_DBG(
				"idx/dev/sid/mid(%d/%d/%d/%d), Sensor mode:%d "
				"u4MinTime_ns:%d "
				"u4ShutterStep:%d lineTime:%d minLine:%d u4PixelClock:%d "
				"u4FrameLength:%d u4LineLength:%d u4FineIntegLine:%d\n",
				m_sensor_index, m_sensor_dev, m_sensor_id, m_module_id, sensor_mode,
				info.min_time_ns, info.shutter_step, info.line_time, min_line,
				info.pixel_clock, info.line_length >> 16, info.line_length & 0xFFFF,
				info.fine_integ_line);
		}
		p_halsensor->destroyInstance("PeripheralCtrlInitGet");
	} else {
		LOG_ERR("hal_sensor(%p) or arg0(%p) is null !\n", p_halsensor,
			(void *)sensor_dynamic_info);
	}
}
// kGetSensorPerframeDynamicInfo
void HalSensorHelper::get_sensor_perframe_dynamic_info(
	int m_sensor_index, int m_sensor_dev,
	mtk::hal3a::SensorPerframeDynamicInfo *sensor_perframe_dynamic_info)
{
	NSCam::IHalSensor *p_halsensor = NSCam::IHalSensorList::get()->createSensor(
		"PeripheralCtrlPerframeGet", m_sensor_index);
	if (p_halsensor) {
		// Query TG info
		p_halsensor->querySensorDynamicInfo(m_sensor_dev,
						    &sensor_perframe_dynamic_info->info);
		//
		p_halsensor->sendCommand(
			m_sensor_dev, NSCam::SENSOR_CMD_GET_FRAME_SYNC_PIXEL_LINE_NUM,
			(intptr_t)&sensor_perframe_dynamic_info->period, 0, 0);
		p_halsensor->sendCommand(
			m_sensor_dev, NSCam::SENSOR_CMD_GET_FRAME_SYNC_PIXEL_LINE_NUM,
			(intptr_t)&sensor_perframe_dynamic_info->pixels_in_line, 0, 0);
		p_halsensor->sendCommand(
			m_sensor_dev, NSCam::SENSOR_CMD_GET_TEMPERATURE_VALUE,
			(intptr_t)&sensor_perframe_dynamic_info->temperature_value, 0, 0);
		//
		p_halsensor->destroyInstance("PeripheralCtrlPerframeGet");
	} else {
		LOG_ERR("hal_sensor(%p) or arg0(%p) is null !\n", p_halsensor,
			(void *)sensor_perframe_dynamic_info);
	}
}
