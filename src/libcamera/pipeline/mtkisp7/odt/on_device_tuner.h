/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.h - MtkISP7 On Device Tuner module.
 */

#pragma once

#include <filesystem>
#include <string>

#include <libcamera/request.h>

#include "pipeline/mtkisp7/imgsys/single-device/single_device.h"

#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"

namespace libcamera {

struct CaptureFrames;

class OnDeviceTuner {
public:
    void configure(const std::string &sensorId);

    void loadTuneRequest(int requestNumber);

    // P1 Camsys
    void tuneCamsys(Request *request, CaptureFrames &frames);

    // P2 Imgsys driver
    void tuneImgsysMetadata(
            SingleDeviceRequest *sdRequest,
            InfoFrame &metaFrame);

private:
    struct NamedFrame {
        Dump::Id id;
        InfoFrame &frame;
    };

    void batchExport(const std::vector<Dump> &dumps);
    int prepareNewExportDirectory();
    bool shouldExportDumpNow(uint32_t requestNumber);
    void tune(uint32_t requestNumber,
              std::vector<NamedFrame> namedFrames,
              bool forceDump=false);

    bool enabled_;
    std::string sensorId_;

    uint32_t exportBegin_;
    uint32_t exportEnd_;
    std::filesystem::path currentExportPath_;

    std::map<Dump::Id, Dump::Config> dumpConfig_;
};

} // namespace libcamera
