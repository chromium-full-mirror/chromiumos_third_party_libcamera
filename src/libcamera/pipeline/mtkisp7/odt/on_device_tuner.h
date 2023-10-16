/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.h - MtkISP7 On Device Tuner module.
 */

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include <libcamera/request.h>

#include "pipeline/mtkisp7/imgsys/single_device.h"

#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"

namespace libcamera {

struct CaptureFrames;
struct MeFrames;
struct TrFrames;
struct Dip1Frames;
struct Dip2Frames;

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

    // MCNR
    void tuneMe(Request *request, MeFrames &frames);
    void tuneTr(Request *request, TrFrames &frames);
    void tuneDip1(Request *request, Dip1Frames &frames);
    void tuneDip2(
            Request *request, Dip2Frames &frames,
            FrameBuffer *videoOut1, FrameBuffer *videoOut2);
private:
    struct NamedFrame {
        Dump::Id id;
        InfoFrame &frame;
    };

    std::vector<ImagiqAdapter::ExportResult> batchExport(
            const std::vector<Dump> &dumps);
    void batchImport(const std::vector<Dump> &dumps);
    void batchPrepareReimport(
            const std::vector<ImagiqAdapter::ExportResult> &exportResults);
    InfoFrame getFrameInfoFromRequest(
            Request *request, FrameBuffer *buffer);
    bool isImgsysCaptureStage(PEU_Stage stage);
    int prepareNewExportDirectory();
    bool shouldExportDumpNow(uint32_t requestNumber);
    bool shouldImportDumpNow(uint32_t requestNumber);
    void tune(uint32_t requestNumber,
              std::vector<NamedFrame> namedFrames,
              bool forceDump=false);

    bool enabled_;
    std::string sensorId_;

    uint32_t exportBegin_;
    uint32_t exportEnd_;
    uint32_t importBegin_;
    uint32_t importEnd_;
    std::filesystem::path currentExportPath_;

    std::map<Dump::Id, Dump::Config> dumpConfig_;
};

} // namespace libcamera
