/*
 * Copyright (C) 2025 Google Inc.
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

#include "platform_utils.h"

namespace libcamera {

// static
PlatformUtils::MtkISP7Platform PlatformUtils::platform_ = PlatformUtils::MtkISP7Platform::NONE;
std::string PlatformUtils::model_ = "";

// static
void PlatformUtils::setWithModelName(const std::string &model)
{
	model_ = model;
	if (!model_.compare("geralt")) {
		platform_ = MtkISP7Platform::GOOGLE;
	} else if (!model_.compare("ciri")) {
		platform_ = MtkISP7Platform::LENOVO;
	}
}

// static
std::string PlatformUtils::enumToString(MtkISP7Platform platform)
{
	switch (platform) {
	case MtkISP7Platform::NONE:
		return "None";

	case MtkISP7Platform::GOOGLE:
		return "Google";

	case MtkISP7Platform::LENOVO:
		return "Lenovo";
	}
}

} // namespace libcamera
