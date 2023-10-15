/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.h - MtkISP7 On Device Tuner module.
 */

#pragma once

#include <string>

#include <libcamera/request.h>

namespace libcamera {

struct CaptureFrames;

class OnDeviceTuner {
public:
    void configure(const std::string &sensorId);

    // P1 Camsys
    void tuneCamsys(Request *request, CaptureFrames &frames);

private:
    bool shouldExportDumpNow(uint32_t requestNumber);

    bool enabled_;
    std::string sensorId_;
};

} // namespace libcamera
