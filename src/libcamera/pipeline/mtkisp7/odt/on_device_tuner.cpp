/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.cpp - MtkISP7 On Device Tuner module.
 */

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include <cstdint>

#include <libcamera/base/log.h>

#include "pipeline/mtkisp7/camsys/capture.h"

#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

constexpr const char *kEnableTuningPath = "/run/camera/enable_tuning";
constexpr const char *kWorkDir =  "/tmp/vendor/camera_dump";

}

void OnDeviceTuner::configure(const std::string &sensorId)
{
    sensorId_ = sensorId;

    if (!std::filesystem::exists(kEnableTuningPath)) {
        enabled_ = false;
        return;
    }

    LOG(MtkISP7, Warning) << "Pipeline tuning enabled";
    enabled_ = true;

    // todo next CL: load config from imagiq adapter
}

bool OnDeviceTuner::shouldExportDumpNow(uint32_t requestNumber)
{
    // todo next CL: load export request i.e. when to export dumps
    return enabled_ && requestNumber == 5;
}

void OnDeviceTuner::tuneCamsys(Request *request, CaptureFrames &frames)
{
    uint32_t requestNumber = request->sequence();
    if (shouldExportDumpNow(requestNumber)) {
        ImagiqAdapter::exportDump({
            .requestNumber=requestNumber,
            .sensorId=sensorId_,
            .workPath=kWorkDir,
            .frame=frames.raw->get(),
        });
    }
    // todo next CL: reload exported
    // todo next CL: import dump
}

} // namespace libcamera
