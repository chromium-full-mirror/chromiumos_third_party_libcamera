/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.cpp - MtkISP7 On Device Tuner module.
 */

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include <cstdint>
#include <fstream>

#include <libcamera/base/log.h>

#include "linux/mtkisp7/drv/7.1/ctrl_meta.h"
#include "pipeline/mtkisp7/camsys/capture.h"

#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/dump_metadata.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

constexpr const char *kEnableTuningPath = "/run/camera/enable_tuning";
constexpr const char *kExportRequestPath = "/run/camera/export_dump";
// Dump key must be exactly 9 digits.
constexpr int kMinDumpKey = 1e8;
constexpr int kMaxDumpKey = 1e9 - 1;
constexpr const char *kWorkDir =  "/tmp/vendor/camera_dump";

} // namespace

void OnDeviceTuner::batchExport(
        const std::vector<Dump> &dumps)
{
    for (auto dump: dumps) {
        ImagiqAdapter::exportDump(dump);
    }
}

void OnDeviceTuner::configure(const std::string &sensorId)
{
    enabled_ = false;

    if (!std::filesystem::exists(kEnableTuningPath)) {
        return;
    }

    int ret = ImagiqAdapter::loadConfig(dumpConfig_, kWorkDir);
    if (ret) {
        LOG(MtkISP7, Error) << "Attempted to enable pipeline tuning, but failed"
                            << " to load the config, error code: " << ret;
        return;
    }

    LOG(MtkISP7, Warning) << "Pipeline tuning enabled";
    enabled_ = true;
    exportBegin_ = 0;
    exportEnd_ = 0;
    sensorId_ = sensorId;
}

void OnDeviceTuner::loadTuneRequest(int requestNumber)
{
    if (!enabled_) {
        return;
    }
    int exportRequestCount = 0;
    std::ifstream exportRequestFile(kExportRequestPath);
    if (exportRequestFile.good()) {
        exportRequestFile >> exportRequestCount;
        LOG(MtkISP7, Info) << "Loaded dump export request from file: "
                           << exportRequestCount << " frames";
        exportRequestFile.close();
        std::filesystem::path path(kExportRequestPath);
        std::filesystem::remove(path);
        exportBegin_ = requestNumber;
        exportEnd_ = requestNumber + exportRequestCount;
        prepareNewExportDirectory();
    }
}

int OnDeviceTuner::prepareNewExportDirectory()
{
    std::filesystem::path newPath;
    std::filesystem::path workPath(kWorkDir);
    int dumpKey;
    for (dumpKey = kMinDumpKey; dumpKey <= kMaxDumpKey; dumpKey++) {
        std::filesystem::path exportFolderName = "UKey" + std::to_string(dumpKey);
        newPath.assign(workPath / exportFolderName);
        if (!std::filesystem::exists(newPath)) {
            break;
        }
    }
    if (dumpKey > kMaxDumpKey) {
        LOG(MtkISP7, Error) << "Failed to create dump directory: "
                            << "too many dumps already.";
        return -EEXIST;
    }
    auto cmd = "mkdir -p " + newPath.string();
    int ret = system(cmd.c_str());
    if (ret != 0) {
        LOG(MtkISP7, Error) << "Failed to prepare dump directory, error code: "
                            << ret;
        return ret;
    }
    LOG(MtkISP7, Info) << "Current dump directory: " << newPath.string();
    currentExportPath_ = newPath;
    return ret;
}

bool OnDeviceTuner::shouldExportDumpNow(uint32_t requestNumber)
{
    return enabled_ && exportBegin_ <= requestNumber &&
           requestNumber < exportEnd_;
}


void OnDeviceTuner::tune(
        uint32_t requestNumber,
        std::vector<NamedFrame> namedFrames,
        bool forceDump)
{
    if (!enabled_ && !forceDump && !shouldExportDumpNow(requestNumber)) {
        return;
    }
    std::vector<Dump> dumps;
    for (auto namedFrame: namedFrames) {
        Dump::Metadata metadata = kDumpMetadata.at(namedFrame.id);
        Dump::Config config = dumpConfig_[namedFrame.id];
        dumps.push_back({
            .id = namedFrame.id,
            .requestNumber = requestNumber,
            .sensorId = sensorId_,
            .workPath = currentExportPath_,
            .frame = namedFrame.frame,
            .metadata = metadata,
            .config = config});
    }
    if (forceDump || shouldExportDumpNow(requestNumber)) {
        batchExport(dumps);
        // todo next CL: prepare reload exported
    }
    // todo next CL: import dumps
}

void OnDeviceTuner::tuneCamsys(Request *request, CaptureFrames &frames)
{
    uint32_t requestNumber = request->sequence();
    if (shouldExportDumpNow(requestNumber)) {
        tune(
            request->sequence(), {
            {Dump::Id::P1_IMGO, frames.raw->get()},
            {Dump::Id::P1_YUVO_R1, frames.yuvo1->get()},
            {Dump::Id::P1_YUVO_R2, frames.yuvo2->get()},
            {Dump::Id::P1_DRZS4NO_R3, frames.me->get()}});
    }
    // todo next CL: reload exported
    // todo next CL: import dump
}

void OnDeviceTuner::tuneImgsysMetadata(
        SingleDeviceRequest *sdRequest,
        InfoFrame &metaFrame)
{
    if (!enabled_) {
        return;
    }
    ctrl_meta_t *imgSysMetadata =
            reinterpret_cast<ctrl_meta_t*>(metaFrame.address(0));
    const auto stageEnums = sdRequest->getStageEnums();
    for (size_t i = 0; i < stageEnums.size(); i++) {
        if (kPeuStageDumpIdMap.count(stageEnums[i]) == 0) {
            LOG(MtkISP7, Error) << "Unrecognized stageEnum: "
                                << stageEnums[i];
            // maybe LPNR
            // todo next CL: LPNR
            continue;
        }
        
        if (!shouldExportDumpNow(sdRequest->sequence())) {
            continue;
        }
        Dump::Id id = kPeuStageDumpIdMap.at(stageEnums[i]);
        Dump::Metadata dumpMetadata = kDumpMetadata.at(id);
        Dump::Config config = dumpConfig_[id];
        imgSysMetadata[i].common.needDump = true;
        auto dumpFileName = ImagiqAdapter::getDumpFileName({
            .id=id,
            .requestNumber=sdRequest->sequence(),
            .sensorId=sensorId_,
            .workPath=currentExportPath_,
            .frame=std::nullopt,
            .metadata=dumpMetadata,
            .config=config,
        });
        strncpy(imgSysMetadata[i].common.nddfp, dumpFileName.c_str(),
                dumpFileName.size());
        LOG(MtkISP7, Info) << "Requested imgsys driver to dump register --"
                        << " request number: " << sdRequest->sequence()
                        << " stage: " << stageEnums[i]
                        << " dump file prefix: " << dumpFileName;
    }
}


} // namespace libcamera
