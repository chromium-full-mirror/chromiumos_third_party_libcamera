/*
 * Copyright (C) 2024, Google Inc.
 *
 * const.h - Constants of ImgSys.
 */

#pragma once

#include <cstring>

#include "libmfnr/MTKBssType.h"
#include "mtkcam-core/libcamera_ext/lib/libMfbllWrapper/MTKMfbllHeader/IMTKMfbll.h"
#include "mtkcam-core/libcamera_ext/lib/libMfbllWrapper/MTKMfbllHeader/include/EMfbll.h"

#include "MediaTypes.h"

namespace libcamera {

constexpr int kInputRawCount = 4;

struct IMFBLL_SET_PROC_INFO_STRUCT {
	MUINT8 *workbuf_addr;
	MUINT32 buf_size;
	MUINT8 *Proc1_base;
	MUINT8 *Proc1_ref;
	MUINT32 Proc1_width;
	MUINT32 Proc1_height;
	MUINT32 Proc1_ME_top_mode;
	MUINT32 Proc1_ME_force_mode;
	//0x30
	MUINT32 Proc1_HG_info_en;

	MUINT32 Proc1_me_wpe_image_width;
	MUINT32 Proc1_me_wpe_image_height;
	MUINT32 Proc1_me_wpe_np1_mode;
	//0x40
	MUINT32 Proc1_me_wpe_stride;
	MUINT32 Proc1_FD_image_width;
	MUINT32 Proc1_FD_image_height;

	MUINT32 Proc1_Bst_FDROI_X0;
	//0x50
	MUINT32 Proc1_Bst_FDROI_Y0;
	MUINT32 Proc1_Bst_FDROI_X1;
	MUINT32 Proc1_Bst_FDROI_Y1;
	MUINT32 Proc1_Ref_FDROI_X0;
	//0x60
	MUINT32 Proc1_Ref_FDROI_Y0;
	MUINT32 Proc1_Ref_FDROI_X1;
	MUINT32 Proc1_Ref_FDROI_Y1;

	MUINT32 Proc1_Bst_LEYE_X0;
	//0x70
	MUINT32 Proc1_Bst_LEYE_X1;
	MUINT32 Proc1_Bst_LEYE_Y0;
	MUINT32 Proc1_Bst_LEYE_Y1;
	MUINT32 Proc1_Bst_LEYE_UX;
	//0x80
	MUINT32 Proc1_Bst_LEYE_UY;
	MUINT32 Proc1_Bst_LEYE_DX;
	MUINT32 Proc1_Bst_LEYE_DY;
	MUINT32 Proc1_Bst_REYE_X0;
	//0x90
	MUINT32 Proc1_Bst_REYE_X1;
	MUINT32 Proc1_Bst_REYE_Y0;
	MUINT32 Proc1_Bst_REYE_Y1;
	MUINT32 Proc1_Bst_REYE_UX;
	//0xA0
	MUINT32 Proc1_Bst_REYE_UY;
	MUINT32 Proc1_Bst_REYE_DX;
	MUINT32 Proc1_Bst_REYE_DY;

	MUINT32 Proc1_Ref_LEYE_X0;
	//0xB0
	MUINT32 Proc1_Ref_LEYE_X1;
	MUINT32 Proc1_Ref_LEYE_Y0;
	MUINT32 Proc1_Ref_LEYE_Y1;
	MUINT32 Proc1_Ref_LEYE_UX;
	//0xC0
	MUINT32 Proc1_Ref_LEYE_UY;
	MUINT32 Proc1_Ref_LEYE_DX;
	MUINT32 Proc1_Ref_LEYE_DY;
	MUINT32 Proc1_Ref_REYE_X0;
	//0xD0
	MUINT32 Proc1_Ref_REYE_X1;
	MUINT32 Proc1_Ref_REYE_Y0;
	MUINT32 Proc1_Ref_REYE_Y1;
	MUINT32 Proc1_Ref_REYE_UX;
	//0xE0
	MUINT32 Proc1_Ref_REYE_UY;
	MUINT32 Proc1_Ref_REYE_DX;
	MUINT32 Proc1_Ref_REYE_DY;

	MUINT32 Proc1_base_ae_dgn_gain;
	//0xF0
	MUINT32 Proc1_base_ae_exposure_time;
	MUINT32 Proc1_base_ae_isp_gain;
	MUINT32 Proc1_base_ae_sensor_gain;
	MUINT32 Proc1_ref_ae_dgn_gain;
	//0x100
	MUINT32 Proc1_ref_ae_exposure_time;
	MUINT32 Proc1_ref_ae_isp_gain;
	MUINT32 Proc1_ref_ae_sensor_gain;

	IPROC_IMAGE_FORMAT Proc1_ImgFmt;
	//0x110
	MUINT32 iBssOrgScore_base;
	MUINT32 iBssOrgScore_ref;

	MINT8 Proc_idx; //debug

	void *pSWMENvram;

	IPass_HG_Info HGInfo;
	IPass_GYRO_MV_INFO GyroMVInfo;

#ifdef FOR_SIM

	MINT32 i4ForceMapMode;
	MINT32 i4ForceMvMode;
	MINT32 i4ForceMvXVal;
	MINT32 i4ForceMvYVal;
	MINT32 i4ForceMvBkXSt;
	MINT32 i4ForceMvBkXEd;
	MINT32 i4ForceMvBkYSt;
	MINT32 i4ForceMvBkYEd;

	MINT32 i4MVPTestModeA;
	MINT32 i4MVPTestModeB;
	MINT32 i4MVPTestModeC;
	MINT32 i4MVPTestModeD;
	MINT32 i4MVPTestModeE;

	MINT32 i4MEBlkLogPosEn;
	MINT32 i4MEBlkLogPosPx;
	MINT32 i4MEBlkLogPosPy;
	MINT32 i4MEBlkLogPosLR;
	MINT32 i4MEBlkLogPosUD;

#endif

	// constructor
	IMFBLL_SET_PROC_INFO_STRUCT()
	{
		memset(this, 0x0, sizeof(struct IMFBLL_SET_PROC_INFO_STRUCT));
	}
};

typedef struct
{
	MUINT32 Ext_mem_size;
	MUINT32 CofMap_width;
	MUINT32 CofMap_height;
	MUINT32 MV_width;
	MUINT32 MV_height;
	MUINT32 WpeMap_width;
	MUINT32 WpeMap_height;

} IMFBLL_GET_PROC_INFO_STRUCT;

struct IMFBLL_VerInfo {
	char rMainVer[5];
	char rSubVer[5];
	char rPatchVer[5];
};

typedef struct IMFBLL_PROC1_OUT_STRUCT {
	MUINT8 *pu1ConfMap;
	MUINT32 u4MapSize;
	MUINT8 *pu1MV;
	MINT32 *pi4WpeMapX;
	MINT32 *pi4WpeMapY;
	MUINT32 u4WpeMapSize;
	MUINT32 u4MVSize;
} IMFBLL_PROC1_OUT_STRUCT;

struct IMFBLL_INIT_PARAM_STRUCT {
	MUINT16 Proc1_imgW;
	MUINT16 Proc1_imgH;
	MUINT32 core_num;
	MBOOL Proc1_DSUS_mode; // 0: 1/4, 1: 1/2
#ifdef FOR_SIM
	char achDmpPath[512];
#endif

	// constructor
	IMFBLL_INIT_PARAM_STRUCT()
		: core_num(4)
	{
	}
};

struct IMFBLL_SET_PROC_INFO_STRUCT_IPC : public IMFBLL_SET_PROC_INFO_STRUCT {
	void *workbuf_ptr;
	void *Proc1_base_ptr;
	void *Proc1_ref_ptr;

	// constructor
	IMFBLL_SET_PROC_INFO_STRUCT_IPC()
	{
		memset(this, 0x0, sizeof(struct IMFBLL_SET_PROC_INFO_STRUCT_IPC));
	}
};

typedef struct IMFBLL_PROC1_OUT_STRUCT_IPC : IMFBLL_PROC1_OUT_STRUCT {
	void *pConfMapPtr;
	void *pMVPtr;
	void *pWpeMapPtr;
} IMFBLL_PROC1_OUT_STRUCT_IPC, *P_IMFBLL_PROC1_OUT_STRUCT_IPC;

} /* namespace libcamera */
