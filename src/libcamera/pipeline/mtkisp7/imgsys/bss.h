/*
 * Copyright (C) 2024, Google Inc.
 *
 * bss.h - MtkISP7 ImgSys Device BSS
 */
#pragma once

#include <memory>
#include <stdio.h>
#include <string.h>
#include <vector>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mapped_framebuffer.h"

#include "mtkcam-interfaces/utils/hw/faces.h"
#include "mtkcam-interfaces/utils/odt/IOnDeviceTuning.h"
#include "platform/mtkisp7/halisp/ITuningDataProvider.h"
#include "platform/mtkisp7/mtkcam-core/libcamera/mt8188/include/libmfnr/MTKBss.h"
#include "platform/mtkisp7/mtkcam-core/libcamera_ext/lib/libBssWrapper/MTKBssHeader/EMTKBss.h"
#include "platform/mtkisp7/mtkcam-core/libcamera_ext/lib/libBssWrapper/MTKBssHeader/IMTKBss.h"

namespace libcamera {

struct ZipOutData {
	MINT32 imgWidth;
	MINT32 imgHeight;
	MINT32 imgFormat;
	MINT32 imgPlaneCount;
	MINT32 imgStride[3];
	MINT32 imgSize[3];
	MINT32 frameIndex; // MTK_CONTROL_CAPTURE_HINT_FOR_ISP_FRAME_INDEX
	ZipOutData()
		: imgWidth{},
		  imgHeight{},
		  imgFormat{},
		  imgPlaneCount{},
		  imgStride{},
		  imgSize{},
		  frameIndex{} {}
};

typedef enum IBSS_PROC_ENUM {
	IBSS_PROC1 = 0,
	IBSS_PROC2,
	IBSS_PROC3,
	IBSS_UNKNOWN_PROC,
} IBSS_PROC_ENUM;

struct IBSS_WB_STRUCT {
	IBSS_PROC_ENUM rProcId; // [User SET]process mode
	MUINT32 u4Width; // [User SET] frame width
	MUINT32 u4Height; // [User SET] frame height
	MUINT32 u4FrameNum; // [User SET] frame number
	MUINT32 u4WKSize; // [return value] working buffer require size
	MUINT8 *pu1BW; // [User SET] frame number

	// constructor
	IBSS_WB_STRUCT() { ::memset(this, 0x0, sizeof(struct IBSS_WB_STRUCT)); }
};

struct IBSS_PARAM_STRUCT {
	MUINT8 BSS_ON;
	MUINT32 BSS_ROI_WIDTH;
	MUINT32 BSS_ROI_HEIGHT;
	MUINT32 BSS_ROI_X0;
	MUINT32 BSS_ROI_Y0;
	void *pBSSNvram;
	MUINT32 BSS_FRAME_NUM;
	MUINT32 BSS_GAIN_TH0;
	MUINT32 BSS_GAIN_TH1;
	MUINT32 BSS_MIN_ISP_GAIN;
	MUINT32 BSS_LCSO_SIZE;
	MUINT8 BSS_FD_TH0;
	MUINT8 BSS_FD_TH1;
	MUINT8 BSS_AEVC_EN;
	MUINT16 BSS_AEVC_DCNT;

	// constructor
	IBSS_PARAM_STRUCT()
		: BSS_ON(1),
		  BSS_FRAME_NUM(4),
		  BSS_GAIN_TH0(853),
		  BSS_GAIN_TH1(1229),
		  BSS_MIN_ISP_GAIN(546),
		  BSS_LCSO_SIZE(147456),
		  BSS_FD_TH0(5),
		  BSS_FD_TH1(40),
		  BSS_AEVC_EN(0),
		  BSS_AEVC_DCNT(512) {}
};

struct IBSS_CONFIG_ZIP_IN_STRUCT {
	MUINT32 u4Width; // [User SET] frame width
	MUINT32 u4Height; // [User SET] frame height
	IBSS_PROC_TYPE eType;
	MUINT32 u4ROI_X0;
	MUINT32 u4ROI_Y0;
	MUINT32 u4ROI_WIDTH;
	MUINT32 u4ROI_HEIGHT;
	MUINT32 u4SCALE_FACTOR;
	void *pBSSNvram;

	// constructor
	IBSS_CONFIG_ZIP_IN_STRUCT()
	{
		::memset(this, 0x0, sizeof(struct IBSS_CONFIG_ZIP_IN_STRUCT));
	}
};

struct IBSS_PROC_ZIP_STRUCT {
	MUINT8 *pu8InImg;
	MUINT32 u4ImgInStride;
	MUINT32 u4ImgInSize;
	MUINT32 u4ImgInBayerOrder; // for raw input
	MUINT8 *pu8OutImg;
	MUINT32 u4ImgOutSize;

	MINT32 i4GmvX;
	MINT32 i4GmvY;

	// constructor
	IBSS_PROC_ZIP_STRUCT()
	{
		::memset(this, 0x0, sizeof(struct IBSS_PROC_ZIP_STRUCT));
	}
};

struct IBSS_CONVERT_STRUCT {
	MUINT8 *pu1Y;
	MUINT8 *pu1U;
	MUINT8 *pu1V;
	MUINT8 *pu1Out;
	MINT32 u4Width;
	MINT32 u4Height;
	MINT32 u4Core;

	// constructor
	IBSS_CONVERT_STRUCT()
	{
		::memset(this, 0x0, sizeof(struct IBSS_CONVERT_STRUCT));
	}
};

struct IBSS_CONFIG_ZIP_OUT_STRUCT {
	MUINT32 u4ZipBufferSize; // [return value] zip out buffer require size

	// constructor
	IBSS_CONFIG_ZIP_OUT_STRUCT()
	{
		::memset(this, 0x0, sizeof(struct IBSS_CONFIG_ZIP_OUT_STRUCT));
	}
};

struct IBSS_INPUT_DATA_G {
	MUINT32 Bitnum;
	MUINT32 BayerOrder;
	MUINT32 Stride;

	MUINT32 inWidth;
	MUINT32 inHeight;
	MUINT32 fdWidth;
	MUINT32 fdHeight;

	IBSS_PROC_TYPE eType;
	MBOOL InputZip;

#ifdef D_BSS_FORCE_BEST_IDX
	MUINT32 i4BSFEN; // force enable
	MUINT32 i4BSFBI; // force best index
#endif

	MUINT8 *apbyBssInImg[MAX_FRAME_NUM];

	IGmv gmv[MAX_FRAME_NUM];
	IBssFaceMetadata *Face[MAX_FRAME_NUM];
	MUINT32 u4AGain[MAX_FRAME_NUM];
	MUINT32 u4DGain[MAX_FRAME_NUM];
	MUINT32 u4ExpT[MAX_FRAME_NUM];
	MUINT16 *u2Lcso[MAX_FRAME_NUM];
	MUINT32 *act_hist_buf[MAX_FRAME_NUM];

	IBSS_GYRO_INFO *prGyroInfo;
	MUINT32 u4GyroNum;
	MUINT32 u4GyroIntervalMS;
	MINT32 bss_iso;

	// constructor
	IBSS_INPUT_DATA_G() { ::memset(this, 0x0, sizeof(struct IBSS_INPUT_DATA_G)); }
};

struct IBSS_INPUT_DATA_G_IPC : public IBSS_INPUT_DATA_G {
	void *apbyBssInImgBufPtr[MAX_FRAME_NUM];
	// constructor
	IBSS_INPUT_DATA_G_IPC()
	{
		::memset(this, 0x0, sizeof(struct IBSS_INPUT_DATA_G_IPC));
	}
};

struct IBSS_OUTPUT_DATA {
	MUINT32 originalOrder[MAX_FRAME_NUM];
	IGmv gmv[MAX_FRAME_NUM];
	long long SharpScore[MAX_FRAME_NUM];
	long long adj1_score[MAX_FRAME_NUM];
	long long adj2_score[MAX_FRAME_NUM];
	long long adj3_score[MAX_FRAME_NUM];
	long long final_score[MAX_FRAME_NUM];
	long long ACTSScore[MAX_FRAME_NUM];
	long long BlendingScore[MAX_FRAME_NUM];
	MINT32 AvgPxLvl[MAX_FRAME_NUM];
	MUINT32 i4SkipFrmCnt;
	MUINT32 u4DGain[MAX_FRAME_NUM];
	MUINT16 *u2Lcso[MAX_FRAME_NUM];

	// constructor
	IBSS_OUTPUT_DATA() { ::memset(this, 0x0, sizeof(struct IBSS_OUTPUT_DATA)); }
};

struct MTKFDContainerInfo {
	MtkCameraFaceMetadata facedata;
	MINT32 sensorId = -1;
	MTKFDContainerInfo() { memset(&facedata, 0, sizeof(MtkCameraFaceMetadata)); }
	~MTKFDContainerInfo() {}

	void clone(MTKFDContainerInfo *cloneInfo)
	{
		if (cloneInfo)
			*cloneInfo = *this;
	}
};
#define FD_DATATYPE MTKFDContainerInfo

struct MTKBSSFDInfo {
	IBssFaceMetadata facedata;
	IBssFace faces[15];
	IBssFaceInfo posInfo[15];
	MTKBSSFDInfo()
		: facedata{}, faces{}, posInfo{} {}
	~MTKBSSFDInfo() {}
};

struct BssFrames {
	struct {
		SharedMailBox<InfoFrame> bssParamInfo;
		SharedMailBox<InfoFrame> bssDataGInfo;
		SharedMailBox<InfoFrame> bssVerInfo;
		SharedMailBox<std::shared_ptr<mtk::isphal::v1::isp_bss_Param>> db_param;
		SharedMailBox<InfoFrame> bssTuningInfo;

		std::vector<SharedMailBox<InfoFrame>> bssFdMainInfo;
		std::vector<SharedMailBox<InfoFrame>> imgi;
		std::vector<SharedMailBox<InfoFrame>> bssFdInfo;
		std::vector<SharedMailBox<InfoFrame>> bssFaceInfo;
		std::vector<SharedMailBox<InfoFrame>> bssPosInfo;
	} in;
	struct {
		SharedMailBox<InfoFrame> bssOutDataInfo;
		SharedMailBox<std::vector<int>> bss_order;
	} out;
};

class BssWrapper
{
public:
	BssWrapper(int sensorIndex);
	~BssWrapper();
	MRESULT bssInit();
	MRESULT bssMain(IBSS_PROC_ENUM ProcId, void *pParaIn, void *pParaOut);
	MRESULT bssFeatureCtrl(IBSS_FTCTRL_ENUM FcId, void *pParaIn, void *pParaOut);
	void *Parser_MfbllIn(void *pParaIn);
	void *Parser_MfbllOut(void *pParaOut);
	// void Parser_MfbllIn_Done(void *pParaIn, void *pParaParseIn);
	void Parser_MfbllOut_Done(void *pParaOut, void *pParaParseOut);
	void *Parser_ParaIn(IBSS_FTCTRL_ENUM FcId, void *pParaIn);
	void *Parser_ParaOut(IBSS_FTCTRL_ENUM FcId, void *pParaOut);
	void Parser_ParaIn_Done(IBSS_FTCTRL_ENUM FcId,
				void *pParaIn,
				void *pParaParseIn);
	void Parser_ParaOut_Done(IBSS_FTCTRL_ENUM FcId,
				 void *pParaOut,
				 void *pParaParseOut);
	void *Parser_BSSIn(void *pParaIn);
	void *Parser_BSSOut(void *pParaOut);
	void Parser_BSSIn_Done(void *pParaIn, void *pParaParseIn);
	void Parser_BSSOut_Done(void *pParaOut, void *pParaParseOut);

	MBOOL getForceBss(void *param_addr, size_t param_size);
	void updateBssProcInfo(IBSS_PARAM_STRUCT *bss_param,
			       MINT32 frameNum,
			       Size srcSize,
			       std::shared_ptr<mtk::isphal::v1::isp_bss_Param> dbParam);
	MBOOL appendBSSInput(std::vector<MappedFrameBuffer> &p1YuvMappedFrameBuffer,

			     IBSS_INPUT_DATA_G_IPC &bss_input);
	MVOID updateBssIOInfo(IBSS_INPUT_DATA_G_IPC &bss_input);

	MVOID collectPreBSSExifData(IBSS_PARAM_STRUCT *param);
	MVOID collectPostBSSExifData(std::vector<MINT32> &vNewIndex,
				     IBSS_OUTPUT_DATA &bss_output);
	void doBss(int frameNum, BssFrames &bssFrame);

private:
	void *m_pBssDrv;
	std::shared_ptr<NSCam::TuningUtils::IOdtUtils> mOdtUtils;
	std::shared_ptr<mtk::isphal::v1::isp_bss_Param> mDbParam;

	ZipOutData mZipData;
	uint32_t sensorIndex_;
	bool mEnableBSSOrdering = true;

	IPass_BSS_WB_STRUCT m_BSSWBParse;
	IPass_BSS_PARAM_STRUCT m_BSSParaParse;
	IPass_BSS_CONFIG_ZIP_IN_STRUCT m_BSSZipInParse;
	IPass_BSS_PROC_ZIP_STRUCT m_BSSZipProcParse;
	IPass_BSS_CONVERT_STRUCT m_BSSCovertParaParse;
	IPass_BSS_CONFIG_ZIP_OUT_STRUCT m_BSSZipOutParse;
	IPass_BSS_INPUT_DATA_G m_BSSInputParse;
	IPass_BSS_OUTPUT_DATA m_BSSOutParse;

	std::map<MINT32, MINT32> mExifData;
};

} // namespace libcamera
