/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * imagiq_adapter.h - MtkISP7 OnDeviceTuner Imagiq Adapter
 */

#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"

#include <filesystem>
#include <fstream>

#include <libcamera/formats.h>
#include <libcamera/pixel_format.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "pipeline/mtkisp7/imgsys/single-device/single_device_helper.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/mtk_headers/ndd_autogen_def.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/dump_metadata.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/static_strings.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

const ImagiqAdapter::SensorIdMap kGeraltSensorMap
{
    {
        "/base/soc/i2c@11ec1000/sensor0@1",
        NSCam::TuningUtils::eSensorId::kMAIN
    },
    {
        "/base/soc/i2c@11ec0000/sensor1@1",
        NSCam::TuningUtils::eSensorId::kSUB
    }
};

const std::filesystem::path kDumpConfigPath = "dump.cfg";

} // namespace

const ImagiqAdapter::SensorIdMap ImagiqAdapter::kSensorIdMap(kGeraltSensorMap);

const std::array<std::string, 2> ImagiqAdapter::kYcPlaneNames{
        "-yplane", "-cplane"};

const std::array<std::string, 3> ImagiqAdapter::kYuvPlaneNames{
        "-yplane", "-uplane", "-vplane"};

const std::array<std::string, 2> ImagiqAdapter::kWarpPlaneNames{
        "-mapx", "-mapy"};


int ImagiqAdapter::exportDump(const Dump &dump)
{
    const NSCam::TuningUtils::NddData ndd(parseNdd(dump));
    const MappedFrameBuffer mappedBuffer(
            dump.frame->buffer(), MappedFrameBuffer::MapFlag::Read);

    const PixelFormat &pixelFormat = dump.frame->format();
    const std::string fileSuffix = getFileExtension(pixelFormat);
    if (shouldSplitExport(pixelFormat)) {
        return exportDumpSplitPlanes(
                dump, mappedBuffer, ndd, pixelFormat, fileSuffix);
    } else {
        return exportDumpMergePlanes(
                dump, mappedBuffer, ndd, fileSuffix);
    }
}

int ImagiqAdapter::exportDumpMergePlanes(
        const Dump &dump, const MappedFrameBuffer &mappedBuffer,
        const NSCam::TuningUtils::NddData &ndd,
        const std::string &fileSuffix)
{
    const std::filesystem::path exportPath =
            getDumpFileNameSingleFile(dump, ndd, fileSuffix);
    std::ofstream exportFile(exportPath, std::ios::binary);
    if (!exportFile) {
        LOG(MtkISP7, Error) << "Failed to open export file: "
                            << exportPath;
        return -EIO;
    }
    const PixelFormatInfo formatInfo =
            PixelFormatInfo::info(dump.frame->format());
    for (unsigned int i = 0; i < formatInfo.numPlanes(); i++) {
        exportFile.write(
                reinterpret_cast<char*>(mappedBuffer.planes()[i].data()),
                mappedBuffer.planes()[i].size());
        LOG(MtkISP7, Info) << "Dump id: " << static_cast<int>(dump.id)
                << "; plane: " << i << "; plane size: " << mappedBuffer.planes()[i].size()
                << "; write file size: " << exportFile.tellp()
                << "; File name: " << exportPath;
        if (!exportFile.good()) {
            LOG(MtkISP7, Error) << "Error writing plane to file: " << exportPath;
            return -EIO;
        }
    }
    return 0;
}

int ImagiqAdapter::exportDumpSplitPlanes(
        const Dump &dump, const MappedFrameBuffer &mappedBuffer,
        const NSCam::TuningUtils::NddData &ndd, const PixelFormat &pixelFormat,
        const std::string &fileSuffix)
{
    const PixelFormatInfo formatInfo =
            PixelFormatInfo::info(dump.frame->format());
    for (size_t i = 0; i < formatInfo.numPlanes(); i++) {
        const std::filesystem::path planeExportPath =
                getDumpFileNameSplitPlanes(
                        dump, ndd, i, pixelFormat, fileSuffix);
        std::ofstream outFile(planeExportPath, std::ios::binary);
        if (!outFile) {
            LOG(MtkISP7, Error) << "Failed to open dump dump file: "
                                << planeExportPath;
            return -EIO;
        }
        outFile.write(
                reinterpret_cast<char*>(mappedBuffer.planes()[i].data()),
                mappedBuffer.planes()[i].size());
        if (!outFile.good()) {
            LOG(MtkISP7, Error) << "Error writing plane to file: " << planeExportPath;
            return -EIO;
        }
        LOG(MtkISP7, Info) << "Dump id: " << static_cast<int>(dump.id)
                << "; plane: " << i << "; plane size: " << mappedBuffer.planes()[i].size()
                << "; File name: " << planeExportPath;
    }
    return 0;
}

std::string ImagiqAdapter::formatPlaneName(int planeNumber, const PixelFormat &pixelFormat)
{
    if (pixelFormat == formats::WARP2P_MTISP) {
        return kWarpPlaneNames[planeNumber];
    }

    int planeCount = PixelFormatInfo::info(pixelFormat).numPlanes();
    if (planeCount == 2) {
        return kYcPlaneNames[planeNumber];
    } else if (planeCount == 3) {
        return kYuvPlaneNames[planeNumber];
    }
    return  "";
}

std::string ImagiqAdapter::getDumpFileName(const Dump &dump)
{
    return getDumpFileNameSingleFile(dump, parseNdd(dump));
}

std::filesystem::path ImagiqAdapter::getDumpFileNameSingleFile(
        const Dump &dump, const NSCam::TuningUtils::NddData &ndd,
        const std::string &suffix)
{
    return getDumpFileNameSplitPlanes(dump, ndd, 1, std::nullopt, suffix);
}

std::filesystem::path ImagiqAdapter::getDumpFileNameSplitPlanes(
        const Dump &dump, const NSCam::TuningUtils::NddData &ndd,
        int planeNumber, const std::optional<PixelFormat> pixelFormat,
        const std::string &suffix)
{
    std::string fileName = "";
    for (const auto& format: dump.config.dumpFileNameFormat) {
        if (format == "Format") {
            fileName += suffix;
            continue;
        }
        if (!StaticStrings::formatKeyExists(format)) {
            fileName += format;
            continue;
        }
        if (format == "Padding" && pixelFormat.has_value()) {
            fileName += formatPlaneName(planeNumber, *pixelFormat);
        }
        fileName += StaticStrings::format(format, ndd);
    }
    std::filesystem::path filePath(fileName);
    return dump.workPath / filePath;
}

std::string ImagiqAdapter::getFileExtension(const PixelFormat &pixelFormat)
{
    switch (pixelFormat) {
    case formats::NV12: return "nv12";
    case formats::NV21: return "nv21";
    case formats::GREY: return "y";
        return "y";
    }
    return "packed_word";
}

int ImagiqAdapter::loadConfig(
        std::map<Dump::Id, Dump::Config> &config,
        const std::filesystem::path &configPath)
{
    std::ifstream dumpCfgFile(configPath / kDumpConfigPath);
    if (!dumpCfgFile.is_open()) {
        LOG(MtkISP7, Error) << "Failed to open config file";
        return -EIO;
    }

    std::string line;
    while (std::getline(dumpCfgFile, line)) {
        std::stringstream ss(line);
        std::string token;
        std::vector<std::string> parsed;
        while (std::getline(ss, token, ',')) {
            parsed.push_back(token);
        }
        if (parsed.size() != 7) {
            LOG(MtkISP7, Error) << "Dump config file error, expected 7 tokens, "
                    << "but got " << parsed.size() << ": " << line;
            return -EINVAL;
        }
        std::string featureStr = parsed[0];
        std::string stageStr = parsed[1];
        std::string categoryStr = parsed[3];
        std::string moduleStr = parsed[4];

        // We (will) only register ~200 dump IDs,
        // while MTK have 20K, so most of the 20K not be mapped.

        if (kStrFeatureMap.count(featureStr) == 0) {
            LOG(MtkISP7, Debug) << "Feature not found: " << featureStr
                                << "Config: " << line;
            continue;
        }
        Feature feature = kStrFeatureMap.at(featureStr);

        if (kStrStageMap.count(stageStr) == 0) {
            LOG(MtkISP7, Debug) << "Stage not found: " << stageStr
                                << "Config: " << line;
            continue;
        }
        Stage stage = kStrStageMap.at(stageStr);

        if (StaticStrings::kStrCategoryMap.count(categoryStr) == 0) {
            LOG(MtkISP7, Debug) << "Category not found: " << categoryStr
                                << "Config: " << line;
            continue;
        }
        NSCam::TuningUtils::eCategory category = StaticStrings::kStrCategoryMap.at(categoryStr);

        if (StaticStrings::kStrModuleMap.count(moduleStr) == 0) {
            LOG(MtkISP7, Debug) << "Module not found: " << moduleStr
                                << "Config: " << line;
            continue;
        }
        NSCam::TuningUtils::eModule module = StaticStrings::kStrModuleMap.at(moduleStr);

        // Multiple dump ids will match because of different layers share
        // config from the .cfg.
        std::vector<Dump::Id> dumpIds;

        // todo(yerlandinata): ~20K config lines (don't know negotiable) and
        // next loop is ~200 steps (length of kDumpMetadata).
        // const map kDumpMetadata may grow in the future development,
        // but won't reach 1000, probably.
        for (const auto &[id, metadata]: kDumpMetadata) {
            if (metadata.featureId == feature && metadata.stage == stage &&
                metadata.category == category && metadata.moduleId == module) {
                dumpIds.push_back(id);
            }
        }

        if (dumpIds.empty()) {
            LOG(MtkISP7, Debug) << "(" << featureStr << ","
                                << stageStr << ","
                                << categoryStr << ","
                                << moduleStr << ") not found";
            continue;
        }

        std::vector<std::string> fileNameFormat;
        // Parsing the filename format keys.
        bool bracketOpen = false;
        std::string current = "";
        for (const auto &c: parsed[2]) {
            switch (c) {
            case '[':
                if (bracketOpen) {
                    LOG(MtkISP7, Error) << "Config file error, "
                            << "invalid dump filename format: " << line;
                    return -EINVAL;
                }
                bracketOpen = true;
                if (!current.empty()) {
                    fileNameFormat.push_back(current);
                    current = "";
                }
                break;
            case ']':
                if (!bracketOpen || current.empty()) {
                    LOG(MtkISP7, Error) << "Config file error, "
                            << "invalid dump filename format: " << line;
                    return -EINVAL;
                }
                bracketOpen = false;
                // Bracketed format must be valid!
                if (!StaticStrings::formatKeyExists(current)) {
                    LOG(MtkISP7, Error) << "Config file error, "
                            << "invalid dump filename format: " << line;
                    return -EINVAL;
                }
                fileNameFormat.push_back(current);
                current = "";
                break;
            default:
                current += c;
                break;
            }
        }
        if (bracketOpen) {
            LOG(MtkISP7, Error) << "Config file error, "
                    << "invalid dump filename format: " << line;
            return -EINVAL;
        }
        if (current.size() != 0) {
            fileNameFormat.push_back(current);
        }
        for (Dump::Id id: dumpIds) {
            config[id] = {
                .dumpFileNameFormat = fileNameFormat,
                .enableExport = parsed[5] == "1",
                .enableImport = parsed[6].length() > 0 ?
                        parsed[6][0] == '1' : false};
        }
    }

    LOG(MtkISP7, Info) << "Loaded dump config for " << config.size() << " different dumps.";
    return 0;
}


NSCam::TuningUtils::NddData ImagiqAdapter::parseNdd(const Dump &dump)
{
    NSCam::TuningUtils::NddData ndd;
    ndd.requestNo = dump.requestNumber;
    ndd.frameNo = dump.requestNumber;
    ndd.timestamp = std::chrono::system_clock::to_time_t(
            std::chrono::system_clock::now());
    ndd.sensorId = kSensorIdMap.at(dump.sensorId);

    ndd.feature = static_cast<int>(dump.metadata.featureId);
    ndd.stage = static_cast<int>(dump.metadata.stage);
    ndd.action = dump.metadata.action.has_value() ?
                 static_cast<int>(dump.metadata.action.value()) : -1;
    ndd.layer = dump.metadata.layer;
    ndd.platform = 8188;

    if (!dump.frame.has_value()) {
        return ndd;
    }

    const PixelFormatInfo pixelFormatInfo =
            PixelFormatInfo::info(dump.frame->format());
    ndd.bitResultion = pixelFormatInfo.planes[0].bytesPerGroup * 8 /
                                  pixelFormatInfo.pixelsPerGroup;

    ndd.byteWidth = pixelFormatInfo.stride(dump.frame->size().width, 0);
    ndd.pixelWidth = dump.frame->size().width;
    ndd.pixelHeight = dump.frame->size().height;

    switch (dump.frame->format()) {
    case formats::SBGGR10_MTISP:
        ndd.bayerOrder = SENSOR_FORMAT_ORDER_RAW_B;
        ndd.signedness = -1;
        break;
    case formats::SGBRG10_MTISP:
        ndd.bayerOrder = SENSOR_FORMAT_ORDER_RAW_Gb;
        ndd.signedness = -1;
        break;
    case formats::SGRBG10_MTISP:
        ndd.bayerOrder = SENSOR_FORMAT_ORDER_RAW_Gr;
        ndd.signedness = -1;
        break;
    case formats::SRGGB10_MTISP:
        ndd.bayerOrder = SENSOR_FORMAT_ORDER_RAW_R;
        ndd.signedness = -1;
        break;
    default:
        ndd.bayerOrder = -1;
        ndd.signedness = 0;
        break;
    }

    ndd.width = ndd.pixelWidth;
    ndd.height = ndd.pixelHeight;

    return ndd;
}

bool ImagiqAdapter::shouldSplitExport(const PixelFormat &pixelFormat)
{
    switch (pixelFormat) {
    case formats::NV21:
    case formats::NV12:
        return false;
    default: return PixelFormatInfo::info(pixelFormat).numPlanes() > 1;
    }
}

} // namespace libcamera
