/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * imagiq_adapter.h - MtkISP7 OnDeviceTuner Imagiq Adapter
 */

#pragma once

#include <filesystem>

#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"

namespace libcamera {

class ImagiqAdapter {
public:
    using SensorIdMap = std::map<std::string, NSCam::TuningUtils::eSensorId>;

    static int loadConfig(
            std::map<Dump::Id, Dump::Config> &config,
            const std::filesystem::path &configPath);

    static int exportDump(const Dump& dump);
private:
    static std::filesystem::path getDumpFileName(
            const Dump &dump, const NSCam::TuningUtils::NddData &ndd,
            const std::string &suffix="");
    static std::string getFileExtension(const PixelFormat &pixelFormat);
    static NSCam::TuningUtils::NddData
    parseNdd(const Dump &dumpInfo);

    static const SensorIdMap kSensorIdMap;
};

} // namespace libcamera
