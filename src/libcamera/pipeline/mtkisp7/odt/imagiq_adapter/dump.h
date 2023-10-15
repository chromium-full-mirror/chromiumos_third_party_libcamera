/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * dump.h - MtkISP7 on device tuner dump information.
 */

#pragma once

#include <cstdint>
#include <filesystem>
#include <optional>
#include <string>

#include "libcamera/internal/info_frame.h"

#include "pipeline/mtkisp7/odt/imagiq_adapter/mtk_headers/ndd_autogen_def.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/action.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/feature.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/stage.h"

namespace libcamera {

struct Dump {

    struct Config {
        std::vector<std::string> dumpFileNameFormat = {};
        bool enableExport = false;
        bool enableImport = false;
    };

    // Static Metadata
    struct Metadata {
        Feature featureId;
        Stage stage;
        NSCam::TuningUtils::eModule moduleId;
        NSCam::TuningUtils::eCategory category;
        std::optional<Action> action = std::nullopt; // nullopt = multi-purpose
        int layer = -1;
    };

    // [Stage]_[BufferName]_[Pipeline (optional)]
    enum class Id {
        // P1
        P1_IMGO = 0,
        P1_YUVO_R1,
        P1_YUVO_R2,
        P1_DRZS4NO_R3,

        // IMGSYS MCNR registers
        LTR_ME_L1_IMGSYS_DRVREG,
        ME_3PASS_MODE0_IMGSYS_DRVREG,
        ME_3PASS_MODE1_IMGSYS_DRVREG,
        TR_DSMAP_IMGSYS_DRVREG,
        TR_Y2Y_F1_IMGSYS_DRVREG,
        TR_Y2Y_F4_IMGSYS_DRVREG,
        WPE_LTR_Y2Y_F1_IMGSYS_DRVREG,
        LTR_VBI_IMGSYS_DRVREG,
        LTR_Y2Y_F4_IMGSYS_DRVREG,
        TR_Y2Y_Conf_IMGSYS_DRVREG,
        WPE_WghtMap_F5_IMGSYS_DRVREG,
        WPE_WghtMap_F4_IMGSYS_DRVREG,
        WPE_WghtMap_F3_IMGSYS_DRVREG,
        WPE_WghtMap_F2_IMGSYS_DRVREG,
        WPE_WghtMap_F1_IMGSYS_DRVREG,
        WPE_WghtMap_F0_IMGSYS_DRVREG,
        P2_IDI_IMGSYS_DRVREG,
        P2_MS_F_SMALL_IMGSYS_DRVREG,
        WPE_P2_PQDIP_MS_F0_IMGSYS_DRVREG,
        P2_MS_F4_IMGSYS_DRVREG,
        P2_MS_F3_IMGSYS_DRVREG_MCNR,
        P2_MS_F2_IMGSYS_DRVREG_MCNR,
        P2_MS_F1_IMGSYS_DRVREG_MCNR,
    };

    Id id;
    uint32_t requestNumber;
    std::string sensorId;
    std::filesystem::path workPath;
    std::optional<InfoFrame> frame;
    Metadata metadata;
    Config config;
};

} // namespace libcamera
