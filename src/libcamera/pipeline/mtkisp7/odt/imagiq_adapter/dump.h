/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * dump.h - MtkISP7 on device tuner dump information.
 */

#pragma once

#include <cstdint>
#include <filesystem>
#include <string>

#include "libcamera/internal/info_frame.h"

namespace libcamera {

struct Dump {
    uint32_t requestNumber;
    std::string sensorId;
    std::filesystem::path workPath;
    InfoFrame frame;
};

} // namespace libcamera
