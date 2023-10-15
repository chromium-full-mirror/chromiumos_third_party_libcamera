/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * imagiq_adapter.h - MtkISP7 OnDeviceTuner Imagiq Adapter
 */

#pragma once

#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"

namespace libcamera {

class ImagiqAdapter {
public:
    static int exportDump(const Dump& dump);
};

} // namespace libcamera
