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
#include <memory>

#include "platform/mtkisp7/mtkcam-core/aaa/peripheralcontroller/include/PeripheralInfoDef.h"

class HalSensorHelper
{
public:
	static std::shared_ptr<HalSensorHelper> getInstance()
	{
		static std::shared_ptr<HalSensorHelper> instance =
			std::make_shared<HalSensorHelper>();
		return instance;
	}
	void get_sensor_initial_dynamic_info(
		int m_sensor_index, int m_sensor_dev,
		mtk::hal3a::SensorInitialDynamicInfo *sensor_dynamic_info);
	void get_sensor_perframe_dynamic_info(
		int m_sensor_index, int m_sensor_dev,
		mtk::hal3a::SensorPerframeDynamicInfo *sensor_perframe_dynamic_info);
};
