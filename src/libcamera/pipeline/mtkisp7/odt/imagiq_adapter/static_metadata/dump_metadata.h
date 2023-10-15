/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * dump_metadata.h - MtkISP7 OnDeviceTuner dump static metadata.
 */

#pragma once

#include <map>

#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"

#include "pipeline/mtkisp7/odt/imagiq_adapter/mtk_headers/ndd_autogen_def.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/feature.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/stage.h"

namespace libcamera {
const std::map<Dump::Id, const Dump::Metadata> kDumpMetadata
{
    // P1
    {
        Dump::Id::P1_IMGO,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P1,
            .moduleId=NSCam::TuningUtils::eModule::kIMGO,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=std::nullopt,
        }
    },
    {
        Dump::Id::P1_DRZS4NO_R3,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P1,
            .moduleId=NSCam::TuningUtils::eModule::kDRZS4NO_R3,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=std::nullopt,
        }
    },
};
} // namespace libcamera
