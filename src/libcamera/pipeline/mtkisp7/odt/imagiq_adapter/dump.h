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
    // Map frame number to planes.
    using SavedPath = std::map<int, std::vector<std::filesystem::path>>;

    struct Config {
        std::vector<std::string> dumpFileNameFormat = {};
        bool enableExport = false;
        bool enableImport = false;
        SavedPath savedDumps = {};
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

        // ME: LTR
        LTR_ME_L1_IMGI_T1,
        LTR_ME_L1_YUVO_T2,
        ME_3PASS_MODE0_MEI_L0,
        // ME: 3PASS_MODE0
        ME_3PASS_MODE0_MEI_L0_P,
        ME_3PASS_MODE0_MEI_L1,
        ME_3PASS_MODE0_MEI_L1_P,
        ME_3PASS_MODE0_MV_L1_M0_P,
        ME_3PASS_MODE0_MV_L0_M1_P,
        ME_3PASS_MODE0_CONF_MAP,
        ME_3PASS_MODE0_MV_L0,
        ME_3PASS_MODE0_MV_L1,
        ME_3PASS_MODE0_FMB_L0,
        ME_3PASS_MODE0_FMB_L1,
        ME_3PASS_MODE0_FST,
        // ME: 3PASS_MODE1
        ME_3PASS_MODE1_MEI_L0,
        ME_3PASS_MODE1_MEI_L0_P,
        ME_3PASS_MODE1_MEI_L1_P,
        ME_3PASS_MODE1_MV_L0_M0,
        ME_3PASS_MODE1_MV_L0,
        ME_3PASS_MODE1_MIL,
        ME_3PASS_MODE1_MMAP,
        ME_3PASS_MODE1_CONF_MAP,
        ME_3PASS_MODE1_FMB_L1_M0,
        ME_3PASS_MODE1_FMB_L0,
        ME_3PASS_MODE1_LMI,
        ME_3PASS_MODE1_FST,

        // TR: DSMAP
        TR_DSMAP_MMAP,
        TR_DSMAP_MMAP_DS0,
        TR_DSMAP_MMAP_DS1,
        TR_DSMAP_MMAP_DS2,
        // TR: Y2Y
        TR_Y2Y_F1_IMGI_T1,
        TR_Y2Y_F1_YUVO_T2,
        TR_Y2Y_F1_YUVO_T3,
        TR_Y2Y_F1_YUVO_T4,
        TR_Y2Y_F4_IMGI_T1,
        TR_Y2Y_F4_YUVO_T2,
        TR_Y2Y_F4_YUVO_T3,
        TR_Y2Y_F4_YUVO_T4,
        // TR: Conf
        TR_Y2Y_Conf_IMGI_T1,
        TR_Y2Y_Conf_YUVO_T5,

        // Dip1: LTR_Y2Y
        WPE_LTR_Y2Y_F1_WPEI,
        WPE_LTR_Y2Y_F1_WPE_MAP,
        WPE_LTR_Y2Y_F1_WPEO,
        WPE_LTR_Y2Y_F1_YUVO_T2,
        WPE_LTR_Y2Y_F1_YUVO_T3,
        WPE_LTR_Y2Y_F1_YUVO_T4,
        WPE_LTR_Y2Y_F1_YUVO_T5,
        // Dip1: LTR VBI
        LTR_VBI_IMGI_T1,
        LTR_VBI_YUVO_T2,
        LTR_VBI_YUVO_T3,
        LTR_VBI_YUVO_T4,
        // Dip1: LTR Y2Y
        LTR_Y2Y_F4_IMGI_T1,
        LTR_Y2Y_F4_YUVO_T2,
        LTR_Y2Y_F4_YUVO_T3,
        // Dip1: WPE WeightMap
        WPE_WghtMap_WPEI_F0,
        WPE_WghtMap_WPE_MAP_F0,
        WPE_WghtMap_WPEO_F0,
        WPE_WghtMap_WPEI_F1,
        WPE_WghtMap_WPE_MAP_F1,
        WPE_WghtMap_WPEO_F1,
        WPE_WghtMap_WPEI_F2,
        WPE_WghtMap_WPE_MAP_F2,
        WPE_WghtMap_WPEO_F2,
        WPE_WghtMap_WPEI_F3,
        WPE_WghtMap_WPE_MAP_F3,
        WPE_WghtMap_WPEO_F3,
        WPE_WghtMap_WPEI_F4,
        WPE_WghtMap_WPE_MAP_F4,
        WPE_WghtMap_WPEO_F4,
        WPE_WghtMap_WPEI_F5,
        WPE_WghtMap_WPE_MAP_F5,
        WPE_WghtMap_WPEO_F5,
        // Dip1: DIP
        P2_IDI_IMGI_D1,
        P2_IDI_VIPI,
        P2_IDI_TNRSI,
        P2_IDI_TNRSO,
        P2_IDI_IMG3O,
        P2_MS_F_SMALL_IMGI_D1,
        P2_MS_F_SMALL_VIPI,
        P2_MS_F_SMALL_TNRSI,
        P2_MS_F_SMALL_TNRWI,
        P2_MS_F_SMALL_TNRCI,
        P2_MS_F_SMALL_TNRLI,
        P2_MS_F_SMALL_TNRVBI,
        P2_MS_F_SMALL_TNRMO,
        P2_MS_F_SMALL_TNRSO,
        P2_MS_F_SMALL_TNRWO,
        P2_MS_F_SMALL_RECI_D1,
        P2_MS_F_SMALL_IMG3O,
        P2_MS_F4_IMGI_D1,
        P2_MS_F4_VIPI,
        P2_MS_F4_TNRSI,
        P2_MS_F4_TNRWI,
        P2_MS_F4_TNRMI,
        P2_MS_F4_TNRCI,
        P2_MS_F4_TNRLI,
        P2_MS_F4_TNRVBI,
        P2_MS_F4_TNRMO,
        P2_MS_F4_TNRSO,
        P2_MS_F4_TNRWO,
        P2_MS_F4_RECI_D1,
        P2_MS_F4_IMG3O,
        P2_MS_F3_IMGI_D1_MCNR,
        P2_MS_F3_VIPI,
        P2_MS_F3_TNRSI,
        P2_MS_F3_TNRWI,
        P2_MS_F3_TNRMI,
        P2_MS_F3_TNRCI,
        P2_MS_F3_TNRLI,
        P2_MS_F3_TNRVBI,
        P2_MS_F3_TNRMO,
        P2_MS_F3_TNRSO,
        P2_MS_F3_TNRWO,
        P2_MS_F3_RECI_D1,
        P2_MS_F3_IMG3O_MCNR,
        P2_MS_F2_IMGI_D1_MCNR,
        P2_MS_F2_VIPI,
        P2_MS_F2_TNRSI,
        P2_MS_F2_TNRWI,
        P2_MS_F2_TNRMI,
        P2_MS_F2_TNRCI,
        P2_MS_F2_TNRLI,
        P2_MS_F2_TNRVBI,
        P2_MS_F2_TNRMO,
        P2_MS_F2_TNRSO,
        P2_MS_F2_TNRWO,
        P2_MS_F2_RECI_D1_MCNR,
        P2_MS_F2_IMG3O_MCNR,
        P2_MS_F1_IMGI_D1_MCNR,
        P2_MS_F1_VIPI,
        P2_MS_F1_TNRSI,
        P2_MS_F1_TNRWI,
        P2_MS_F1_TNRMI,
        P2_MS_F1_TNRCI,
        P2_MS_F1_TNRLI,
        P2_MS_F1_TNRVBI,
        P2_MS_F1_TNRMO,
        P2_MS_F1_TNRSO,
        P2_MS_F1_TNRWO,
        P2_MS_F1_RECI_D1_MCNR,
        P2_MS_F1_IMG3O_MCNR,
        P2_MS_F1_IMG4O,

        // Dip2
        WPE_P2_PQDIP_MS_F0_WPETI,
        WPE_P2_PQDIP_MS_F0_WPET_MAP,
        WPE_P2_PQDIP_MS_F0_WPETO,
        WPE_P2_PQDIP_MS_F0_TNRSI,
        WPE_P2_PQDIP_MS_F0_TNRWI,
        WPE_P2_PQDIP_MS_F0_TNRMI,
        WPE_P2_PQDIP_MS_F0_TNRCI,
        WPE_P2_PQDIP_MS_F0_TNRLI,
        WPE_P2_PQDIP_MS_F0_TNRSO,
        WPE_P2_PQDIP_MS_F0_TNRWO,
        WPE_P2_PQDIP_MS_F0_RECI_D1,
        WPE_P2_PQDIP_MS_F0_IMG3O,
        WPE_P2_PQDIP_MS_F0_IMG4O,
        WPE_P2_PQDIP_MS_F0_IMGI_D1,
        WPE_P2_PQDIP_MS_F0_WROTO,
        WPE_P2_PQDIP_MS_F0_WDMAO,
    };

    const static std::array<Dump::Id, 6> kWpeInputImageDumpIds;
    const static std::array<Dump::Id, 6> kWpeWeightMapDumpIds;
    const static std::array<Dump::Id, 6> kWpeOutputImageDumpIds;
    const static std::array<Dump::Id, 6> kDip1ImgiDumpIds;
    const static std::array<Dump::Id, 6> kDip1VipiDumpIds;
    const static std::array<Dump::Id, 6> kDip1TnrsiDumpIds;
    const static std::array<Dump::Id, 6> kDip1TnrsoDumpIds;
    const static std::array<Dump::Id, 6> kDip1Img3oDumpIds;
    const static std::array<Dump::Id, 5> kDip1TnrwiDumpIds;
    const static std::array<Dump::Id, 5> kDip1TnrciDumpIds;
    const static std::array<Dump::Id, 5> kDip1TnrliDumpIds;
    const static std::array<Dump::Id, 5> kDip1TnrvbiDumpIds;
    const static std::array<Dump::Id, 5> kDip1TnrmoDumpIds;
    const static std::array<Dump::Id, 5> kDip1TnrwoDumpIds;
    const static std::array<Dump::Id, 5> kDip1ReciDumpIds;
    const static std::array<Dump::Id, 4> kDip1TnrmiDumpIds;

    Id id;
    uint32_t requestNumber;
    std::string sensorId;
    std::filesystem::path workPath;
    std::optional<InfoFrame> frame;
    Metadata metadata;
    Config config;
};

} // namespace libcamera
