/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * imagiq_adapter.h - MtkISP7 OnDeviceTuner Imagiq Adapter
 */

#pragma once

#include <filesystem>

#include "libcamera/internal/mapped_framebuffer.h"

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
    static int exportDumpMergePlanes(
            const Dump &dumpInfo, const MappedFrameBuffer &mappedBuffer,
            const NSCam::TuningUtils::NddData &ndd,
            const std::string &fileSuffix="");

    static int exportDumpSplitPlanes(
            const Dump &dumpInfo, const MappedFrameBuffer &mappedBuffer,
            const NSCam::TuningUtils::NddData &ndd,
            const PixelFormat &pixelFormat,
            const std::string &fileSuffix="");

    static std::string formatPlaneName(int planeNumber,
                                       const PixelFormat &pixelFormat);

    static std::filesystem::path getDumpFileNameSplitPlanes(
            const Dump &dumpInfo, const NSCam::TuningUtils::NddData &ndd,
            int planeNumber, const std::optional<PixelFormat> pixelFormat,
            const std::string &suffix="");

    static std::filesystem::path getDumpFileNameSingleFile(
            const Dump &dumpInfo, const NSCam::TuningUtils::NddData &ndd,
            const std::string &suffix="");

    static std::string getFileExtension(const PixelFormat &pixelFormat);

    static NSCam::TuningUtils::NddData
    parseNdd(const Dump &dumpInfo);

    static bool shouldSplitExport(const PixelFormat &pixelFormat);

    static const SensorIdMap kSensorIdMap;
    static const std::array<std::string, 2> kYcPlaneNames;
    static const std::array<std::string, 3> kYuvPlaneNames;
    static const std::array<std::string, 2> kWarpPlaneNames;
};

} // namespace libcamera
