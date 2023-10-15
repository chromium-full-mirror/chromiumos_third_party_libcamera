/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * dump_metadata.h - MtkISP7 OnDeviceTuner dump static metadata.
 */

#pragma once

#include <map>

#include "platform/mtkisp7/single_device_helper.h"

#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/mtk_headers/ndd_autogen_def.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/feature.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/stage.h"

namespace libcamera {

const std::map<PEU_Stage, Dump::Id> kPeuStageDumpIdMap
{
    // ? = need double check, ?? = really need double check
    // MCNR
    {PEU_Stage::HW_LTR_ME_L1, Dump::Id::LTR_ME_L1_IMGSYS_DRVREG},
    {PEU_Stage::HW_ME_3PASS_MODE_0, Dump::Id::ME_3PASS_MODE0_IMGSYS_DRVREG},
    {PEU_Stage::HW_ME_3PASS_MODE_1, Dump::Id::ME_3PASS_MODE1_IMGSYS_DRVREG},
    {PEU_Stage::HW_TR_F1, Dump::Id::TR_Y2Y_F1_IMGSYS_DRVREG},
    {PEU_Stage::HW_TR_F4, Dump::Id::TR_Y2Y_F4_IMGSYS_DRVREG},
    {PEU_Stage::HW_TR_HWMVP, Dump::Id::TR_DSMAP_IMGSYS_DRVREG}, // ?
    {PEU_Stage::HW_TR_CONF4, Dump::Id::TR_Y2Y_Conf_IMGSYS_DRVREG},
    {PEU_Stage::HW_TR_CONF5, Dump::Id::TR_Y2Y_Conf_IMGSYS_DRVREG},
    {PEU_Stage::HW_LTR_F1, Dump::Id::WPE_LTR_Y2Y_F1_IMGSYS_DRVREG}, // ??
    {PEU_Stage::HW_LTR_F4, Dump::Id::LTR_Y2Y_F4_IMGSYS_DRVREG},
    {PEU_Stage::HW_LTR_VBI, Dump::Id::LTR_VBI_IMGSYS_DRVREG},
    {PEU_Stage::HW_WPE_W_F1, Dump::Id::WPE_WghtMap_F1_IMGSYS_DRVREG},
    {PEU_Stage::HW_WPE_W_F2, Dump::Id::WPE_WghtMap_F2_IMGSYS_DRVREG},
    {PEU_Stage::HW_WPE_W_F3, Dump::Id::WPE_WghtMap_F3_IMGSYS_DRVREG},
    {PEU_Stage::HW_WPE_W_F4, Dump::Id::WPE_WghtMap_F4_IMGSYS_DRVREG},
    {PEU_Stage::HW_WPE_W_F5, Dump::Id::WPE_WghtMap_F5_IMGSYS_DRVREG},
    {PEU_Stage::HW_WPE_W_F0, Dump::Id::WPE_WghtMap_F0_IMGSYS_DRVREG},
    {PEU_Stage::HW_DIP_IDI, Dump::Id::P2_IDI_IMGSYS_DRVREG},
    {PEU_Stage::HW_DIP_IDI2, Dump::Id::P2_MS_F_SMALL_IMGSYS_DRVREG}, // ??
    {PEU_Stage::HW_DIP_F4, Dump::Id::P2_MS_F4_IMGSYS_DRVREG},
    {PEU_Stage::HW_DIP_F3, Dump::Id::P2_MS_F3_IMGSYS_DRVREG_MCNR},
    {PEU_Stage::HW_DIP_F2, Dump::Id::P2_MS_F2_IMGSYS_DRVREG_MCNR},
    {PEU_Stage::HW_DIP_F1, Dump::Id::P2_MS_F1_IMGSYS_DRVREG_MCNR},
    {PEU_Stage::HW_DIP_F0, Dump::Id::WPE_P2_PQDIP_MS_F0_IMGSYS_DRVREG}, // ??
};

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
        Dump::Id::P1_YUVO_R1,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P1,
            .moduleId=NSCam::TuningUtils::eModule::kYUVO_R1,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=std::nullopt,
        }
    },
    {
        Dump::Id::P1_YUVO_R2,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P1,
            .moduleId=NSCam::TuningUtils::eModule::kYUVO_R2,
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
    // IMGSYS metadata
    {
        Dump::Id::LTR_ME_L1_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::LTR_ME_L1,
            .moduleId=NSCam::TuningUtils::eModule::kREG_LTRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::ME_3PASS_MODE0_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::ME_3PASS_MODE0,
            .moduleId=NSCam::TuningUtils::eModule::kREG_ME,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::ME_3PASS_MODE1_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::ME_3PASS_MODE1,
            .moduleId=NSCam::TuningUtils::eModule::kREG_ME,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::TR_DSMAP_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::TR_DSMAP,
            .moduleId=NSCam::TuningUtils::eModule::kREG_TRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::TR_Y2Y_F1_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::TR_Y2Y_F1,
            .moduleId=NSCam::TuningUtils::eModule::kREG_TRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::TR_Y2Y_F4_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::TR_Y2Y_F4,
            .moduleId=NSCam::TuningUtils::eModule::kREG_TRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::WPE_LTR_Y2Y_F1_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_LTR_Y2Y_F1,
            .moduleId=NSCam::TuningUtils::eModule::kREG_LTRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::LTR_VBI_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::LTR_VBI,
            .moduleId=NSCam::TuningUtils::eModule::kREG_LTRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::LTR_Y2Y_F4_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::LTR_Y2Y_F4,
            .moduleId=NSCam::TuningUtils::eModule::kREG_LTRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::TR_Y2Y_Conf_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::TR_Y2Y_Conf,
            .moduleId=NSCam::TuningUtils::eModule::kREG_TRAW,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::WPE_WghtMap_F5_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_WghtMap,
            .moduleId=NSCam::TuningUtils::eModule::kREG_WPE,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=5,
        }
    },
    {
        Dump::Id::WPE_WghtMap_F4_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_WghtMap,
            .moduleId=NSCam::TuningUtils::eModule::kREG_WPE,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=4,
        }
    },
    {
        Dump::Id::WPE_WghtMap_F3_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_WghtMap,
            .moduleId=NSCam::TuningUtils::eModule::kREG_WPE,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=3,
        }
    },
    {
        Dump::Id::WPE_WghtMap_F2_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_WghtMap,
            .moduleId=NSCam::TuningUtils::eModule::kREG_WPE,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=2,
        }
    },
    {
        Dump::Id::WPE_WghtMap_F1_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_WghtMap,
            .moduleId=NSCam::TuningUtils::eModule::kREG_WPE,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=1,
        }
    },
    {
        Dump::Id::WPE_WghtMap_F0_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_WghtMap,
            .moduleId=NSCam::TuningUtils::eModule::kREG_WPE,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=0,
        }
    },
    {
        Dump::Id::P2_IDI_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P2_IDI,
            .moduleId=NSCam::TuningUtils::eModule::kREG_DIP,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::P2_MS_F_SMALL_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P2_MS_F_SMALL,
            .moduleId=NSCam::TuningUtils::eModule::kREG_DIP,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::P2_MS_F4_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P2_MS_F4,
            .moduleId=NSCam::TuningUtils::eModule::kREG_DIP,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
        }
    },
    {
        Dump::Id::P2_MS_F3_IMGSYS_DRVREG_MCNR,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P2_MS_F3,
            .moduleId=NSCam::TuningUtils::eModule::kREG_DIP,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=3,
        }
    },
    {
        Dump::Id::P2_MS_F2_IMGSYS_DRVREG_MCNR,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P2_MS_F2,
            .moduleId=NSCam::TuningUtils::eModule::kREG_DIP,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=2,
        }
    },
    {
        Dump::Id::P2_MS_F1_IMGSYS_DRVREG_MCNR,
        {
            .featureId=Feature::Preview,
            .stage=Stage::P2_MS_F1,
            .moduleId=NSCam::TuningUtils::eModule::kREG_DIP,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=1,
        }
    },
    {
        Dump::Id::WPE_P2_PQDIP_MS_F0_IMGSYS_DRVREG,
        {
            .featureId=Feature::Preview,
            .stage=Stage::WPE_P2_PQDIP_MS_F0,
            .moduleId=NSCam::TuningUtils::eModule::kREG_DIP,
            .category=NSCam::TuningUtils::eCategory::kSTREAMING,
            .action=Action::Preview,
            .layer=0,
        }
    },
};
} // namespace libcamera
