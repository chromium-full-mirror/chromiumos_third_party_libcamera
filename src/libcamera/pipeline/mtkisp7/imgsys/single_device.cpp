/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * single_device.cpp - MtkISP7 ImgSys single device wrapper
 */

#include "single_device.h"

#include <cstring>
#include <vector>

#include <libcamera/formats.h>

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/formats.h"
#include "libcamera/internal/framebuffer.h"

#include "kernel-headers/mtk_header_desc.h"
#include "mtkcam-halif/def/ImageFormat.h"
#include "platform/mtkisp7/ImgPortDef.h"

using namespace NSCam;
using namespace NSCam::NSImgStream;

NSCam::NSImgStream::BufferProperty toBufferPropery(const libcamera::InfoFrame &info)
{
	NSCam::NSImgStream::BufferProperty property;

	switch (info.format()) {
	case libcamera::formats::SBGGR10_MTISP:
	case libcamera::formats::SGBRG10_MTISP:
	case libcamera::formats::SGRBG10_MTISP:
	case libcamera::formats::SRGGB10_MTISP:
		property.format = eImgFmt_BAYER10;
		break;
	case libcamera::formats::NV12_10P_MTISP:
		property.format = eImgFmt_MTK_YUV_P010;
		break;
	case libcamera::formats::NV12_12P_MTISP:
		property.format = eImgFmt_MTK_YUV_P012;
		break;
	case libcamera::formats::NV12:
		property.format = eImgFmt_NV12;
		break;
	case libcamera::formats::NV21:
		property.format = eImgFmt_NV21;
		break;
	case libcamera::formats::GREY:
		property.format = eImgFmt_Y8;
		break;
	case libcamera::formats::Y8_MTISP:
		property.format = eImgFmt_STA_BYTE;
		break;
	case libcamera::formats::Y16_MTISP:
		property.format = eImgFmt_STA_2BYTE;
		break;
	case libcamera::formats::Y32_MTISP:
		property.format = eImgFmt_STA_4BYTE;
		break;
	case libcamera::formats::WARP2P_MTISP:
		property.format = eImgFmt_WARP_2PLANE;
		break;
	case libcamera::formats::MTFD_MTISP:
		property.format = eImgFmt_ISP_TUNING;
		break;
	default:
		printf("Unsupported format\n");
		std::abort();
	}

	switch (info.format()) {
	case libcamera::formats::SBGGR10_MTISP:
		property.ColorArrangeMent = SENSOR_FORMAT_ORDER_RAW_B;
		break;
	case libcamera::formats::SGBRG10_MTISP:
		property.ColorArrangeMent = SENSOR_FORMAT_ORDER_RAW_Gb;
		break;
	case libcamera::formats::SGRBG10_MTISP:
		property.ColorArrangeMent = SENSOR_FORMAT_ORDER_RAW_Gr;
		break;
	case libcamera::formats::SRGGB10_MTISP:
		property.ColorArrangeMent = SENSOR_FORMAT_ORDER_RAW_R;
		break;
	default:
		property.ColorArrangeMent = -1;
	}

	property.colorSpace = NSCam::eImgColorSpace_BT601_FULL;

	property.width = info.size().width;
	property.height = info.size().height;

	property.numPlanes = info.numPlanes();

	const libcamera::PixelFormatInfo &formatInfo =
		libcamera::PixelFormatInfo::info(info.format());

	for (unsigned int i = 0; i < info.numPlanes(); i++) {
		property.planes[i].fd = info.buffer()->planes()[i].fd.get();
		property.planes[i].offset = info.buffer()->planes()[i].offset;
		property.planes[i].size = info.buffer()->planes()[i].length;
		property.planes[i].stride = formatInfo.stride(info.size().width, i, info.strideAlign());

		unsigned int planeSize = formatInfo.planeSize(
				info.size(), i, info.strideAlign(), info.scanAlign());

		property.planes[i].scanline = planeSize / property.planes[i].stride;
		property.planes[i].va = reinterpret_cast<MINTPTR>(info.address(i));
	}

	return property;
}

namespace libcamera {

void translatePortEx(std::vector<PortInfoEx> &portInfoExs, std::vector<PortInfo> &portInfos)
{
	for (auto &inEx : portInfoExs) {
		portInfos.emplace_back();
		auto &info = portInfos.back();
		info.mPortIdx = inEx.portIdx;
		info.mResizeInfo.mResizeRatio = (IMG_RESIZE_RATIO)inEx.mResizeRatio;
		info.mSrcCrop.CropX = inEx.CropX;
		info.mSrcCrop.CropY = inEx.CropY;
		info.mSrcCrop.CropW = inEx.CropW;
		info.mSrcCrop.CropH = inEx.CropH;
		info.mSrcCrop.CropFloatX = inEx.CropFloatX;
		info.mSrcCrop.CropFloatY = inEx.CropFloatY;
		info.mSrcCrop.CropFloatW = inEx.CropFloatW;
		info.mSrcCrop.CropFloatH = inEx.CropFloatH;
		info.mSecureTag = (IMG_SECURE_ENUM)0;
		info.mTransform = 0;

		info.mBuffer = &inEx.img;
	}
}

bool syncCache(NSCam::NSImgStream::CacheCtrl const ctrl, int fd)
{
	/* todo: Collect fds from each planes and sync once */
	if (ctrl == NSCam::NSImgStream::eCACHECTRL_INVALID)
		libcamera::DmaHeap::sync(
			fd,
			libcamera::DmaHeap::Start,
			libcamera::DmaHeap::SyncReadWrite);
	else
		libcamera::DmaHeap::sync(
			fd,
			libcamera::DmaHeap::End,
			libcamera::DmaHeap::SyncReadWrite);
	return true;
}

void StageEx::input(const InfoFrame &info, uint32_t idx, int ratio, const Rectangle &crop)
{
	inputs_.emplace_back();
	inputs_.back().set(info, idx, ratio, crop);
}

void StageEx::output(const InfoFrame &info, uint32_t idx, int ratio, const Rectangle &crop)
{
	outputs_.emplace_back();
	outputs_.back().set(info, idx, ratio, crop);
}

void StageEx::setWpeInfo(NSCam::NSImgStream::IMG_EXTRA_PARAM_ID id, Size crop, NSCam::NSImgStream::WPE_MODE mode, unsigned int featureIndex)
{
	using WPE_MODE = NSCam::NSImgStream::WPE_MODE;
	using PSP_TABLE_SEL = NSCam::NSImgStream::PSP_TABLE_SEL;
	using RGB_MODE = NSCam::NSImgStream::RGB_MODE;

	using WPE_CrpInfo = NSCam::NSImgStream::WPE_CrpInfo;
	using WPE_CrpOfstInfo = NSCam::NSImgStream::WPE_CrpOfstInfo;

	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = id;
	auto crpInfo = WPE_CrpInfo{ .x_start_point = 0, .x_end_point = crop.width - 1, .y_start_point = 0, .y_end_point = crop.height - 1 };

	auto crpOfstInfo = WPE_CrpOfstInfo{ .x_start = 0, .hr_int_ofst = 0, .hr_sub_ofst = 0, .y_start = 0, .vt_int_ofst = 0, .vt_sub_ofst = 0, .wd = 0, .ht = 0 };

	param.mData.mWPEInfo = WPEInfo{
		.wpe_mode = (WPE_MODE)mode,
		.vgen_out = crpInfo,
		.tbl_sel_v = (PSP_TABLE_SEL)1,
		.tbl_sel_h = (PSP_TABLE_SEL)1,
		.extra_feature_index = featureIndex,
		.rgb_mode = (RGB_MODE)0,
		.vgen_in = crpOfstInfo,
		.psp_border_color_y = 0,
		.psp_border_color_u = 0,
		.psp_border_color_v = 0
	};
}

void StageEx::setMvFrame(Size f0, Size me)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_MVFRAME_INFO;
	param.mData.mMVFrameInfo = MVFrameInfo{
		.mF0Width = f0.width, .mF0Height = f0.height, .mME0Width = me.width, .mME0Height = me.height, .mConfScaleRatio = 4
	};
}

void StageEx::setMeInfo(NSCam::NSImgStream::ME_MODE mode)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_ME_INFO;
	param.mData.mMEInfo = MEInfo{ .me_mode = mode };
}

void StageEx::setAplInfo()
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_APL_INFO;
	param.mData.mAPLInfo = APLInfo{ .mAplEnable = 1 };
}

void StageEx::setMultiScale(NSCam::NSImgStream::IMG_MULTI_SCALE_RATIO ratio,
			    uint32_t index, uint32_t total)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_DIP_MULTISCALE_INFO;
	param.mData.mMutiScaleInfo = MultiScaleInfo{
		.mScaleRatio = ratio, .mScaleIdx = index, .mScaleTotal = total
	};
}

void StageEx::setPqInfo()
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_PQ_PORT_INFO;
	param.mData.mPQPortInfo = PQPortInfo{
		.mWdmaoPQIdx = 1, .mWdmaoUserString = 0, .mWdmaoBypassCrop = 0, .mWrotoPQIdx = 2, .mWrotoUserString = 0, .mWrotoBypassCrop = 0
	};
}

void StageEx::setImg4oCrop(const Rectangle &crop)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_P_IMG4O_CROP_INFO;
	param.mData.mPImg4oCropInfo = PImg4oCropInfo{
		.p_img4o_crop_x = (uint32_t)crop.x,
		.p_img4o_crop_y = (uint32_t)crop.y,
		.p_img4o_crop_w = (uint32_t)crop.width,
		.p_img4o_crop_h = (uint32_t)crop.height,
		.tnrwo_scale_ratio = 8
	};
}

void StageEx::setCostLevel()
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_COST_LEVEL_INFO;
	param.mData.mCostLevel = WPECostLevel{ .costlevel = (COST_LEVEL)0 };
}

void StageEx::addNotify(uint32_t sync)
{
	mSyncTokenNotifyList.push_back(sync);
}

void StageEx::addWait(uint32_t sync)
{
	mSyncTokenWaitList.push_back(sync);
}

void SingleDeviceRequest::fillFrameParams(
	NSCam::NSImgStream::FrameParams &frameParams, StageEx &stage)
{
	frameParams.mTimestamp = timestamp();
	frameParams.mStage = stage.stageEnum_;

	frameParams.mSecureFra = 0;
	frameParams.mScenPath = 0;

	frameParams.mSyncPrevFrameParam = false;
	frameParams.mSyncNextFrameParam = false;
	frameParams.mSyncTokenNotify = 0;
	frameParams.mSyncTokenWait = 0;
	frameParams.mSyncTokenNotifyList.clear();
	frameParams.mSyncTokenWaitList.clear();
	frameParams.mFrameOwner = EIGHTCC();

	frameParams.mSyncTokenNotifyList = stage.mSyncTokenNotifyList;
	frameParams.mSyncTokenWaitList = stage.mSyncTokenWaitList;

	translatePortEx(stage.inputs_, frameParams.mvIn);
	translatePortEx(stage.outputs_, frameParams.mvOut);
	frameParams.mvExtraParam = stage.extra_;
}

void SingleDeviceRequest::fillRequestBufferForStage(InfoFrame &infoCtrl,
						    InfoFrame &infoDesc,
						    int requestFd, size_t stage)
{
	std::vector<FrameParams> mvFrameParams;
	auto &frameParams = mvFrameParams.emplace_back();
	fillFrameParams(frameParams, stages_[stage]);

	std::shared_ptr<ImgParams> pParams = std::make_shared<ImgParams>();
	pParams->mHWSharing = 0;
	pParams->mFps = 30;
	pParams->mSyncID = -1;
	pParams->mRequestNo = sequence();
	pParams->mFrameNo = sequence();
	pParams->mNumBatchRun = 1;
	pParams->mvFrameParams = std::vector<FrameParams>({ mvFrameParams });

	NSCam::NSImgStream::IImageBuffer imageCM(toBufferPropery(infoCtrl));
	CtrlMetaBuf CMBuf{
		.mFd = imageCM.getPlaneFD(0),
		.mOffset = (MUINT32)imageCM.getPlaneOffsetInBytes(0),
		.mBufSize = (MINT32)imageCM.getBufSizeInBytes(0),
		.mpBufVa = (MINTPTR)infoCtrl.address(0),
		.mpBufPa = 0
	};

	MediaRequest fr(requestFd);

	/* TODO: Set the corresponding userid for each request */
	EIGHTCC userid = EIGHTCC("S_ME-A");

	RequestInfo reqInfo;
	reqInfo.mpRequest = &fr;
	reqInfo.mImgStreamOwner = userid;
	reqInfo.mMemMode = MEMORY_MODE_NORMAL;
	reqInfo.mpCMBuf = &CMBuf;
	reqInfo.pParams = pParams;

	gettimeofday(&reqInfo.enque_time, NULL);

	ImgInitParam initParam;
	initParam.mMaxFps = 30;
	initParam.mPriority = IMG_PRIORITY_PREVIEW;
	initParam.mLowLatency = 0;

	/* Pipeline handler does not need to populate singlenode_desc_norm.
	 * the driver will do that according to the queued buffers.
	 * we still need createSingleDevBuffer() to fill the control meta
	 * buffers, tuning buffers, etc.
	 * Pass in a dummy buffer as singlenode_desc_norm here.
	 */
	static uint8_t dummy[sizeof(struct singlenode_desc_norm)];
	NSCam::NSImgStream::IImageBuffer imageDesc(toBufferPropery(infoDesc));
	VNDescBuf descBuf{
		.mFd = imageDesc.getPlaneFD(0),
		.mBufSize = (MINT32)imageDesc.getBufSizeInBytes(0),
		.mOffset = (MUINT32)imageDesc.getPlaneOffsetInBytes(0),
		.mpDescBufVa = (MINTPTR)&dummy,
		.mbUsed = false,
	};

	syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID, infoCtrl.buffer()->planes()[0].fd.get());

	for (auto &frameParam : mvFrameParams) {
		for (auto &input : frameParam.mvIn)
			if (input.mPortIdx == NSCam::NSImgStream::IMG_PORT_METAI)
				syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID,
					  input.mBuffer->getPlaneFD(0));
	}

	createSingleDevBuffer(&reqInfo, &initParam, userid, V4L2_MODE_SIGNLE_DEVICE, &descBuf);

	for (auto &frameParam : mvFrameParams) {
		for (auto &input : frameParam.mvIn)
			if (input.mPortIdx == NSCam::NSImgStream::IMG_PORT_METAI)
				syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH,
					  input.mBuffer->getPlaneFD(0));
	}

	syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH, infoCtrl.buffer()->planes()[0].fd.get());

	infoDesc.buffer()->_d()->metadata().planes()[0].bytesused = infoDesc.buffer()->planes()[0].length;
}

void SingleDeviceRequest::fillRequestBuffer(InfoFrame &infoCtrl,
					    InfoFrame &infoDesc,
					    int requestFd)
{
	std::vector<FrameParams> mvFrameParams;

	for (auto &stage : stages_) {
		auto &frameParams = mvFrameParams.emplace_back();
		fillFrameParams(frameParams, stage);
	}

	std::shared_ptr<ImgParams> pParams = std::make_shared<ImgParams>();
	pParams->mHWSharing = 0;
	pParams->mFps = 30;
	pParams->mSyncID = -1;
	pParams->mRequestNo = sequence();
	pParams->mFrameNo = sequence();
	pParams->mNumBatchRun = 1;
	pParams->mvFrameParams.swap(mvFrameParams);

	NSCam::NSImgStream::IImageBuffer imageCM(toBufferPropery(infoCtrl));
	CtrlMetaBuf CMBuf{
		.mFd = imageCM.getPlaneFD(0),
		.mOffset = (MUINT32)imageCM.getPlaneOffsetInBytes(0),
		.mBufSize = (MINT32)imageCM.getBufSizeInBytes(0),
		.mpBufVa = (MINTPTR)infoCtrl.address(0),
		.mpBufPa = 0
	};

	MediaRequest fr(requestFd);

	/* TODO: Set the corresponding userid for each request */
	EIGHTCC userid = EIGHTCC("S_ME-A");

	RequestInfo reqInfo;
	reqInfo.mpRequest = &fr;
	reqInfo.mImgStreamOwner = userid;
	reqInfo.mMemMode = MEMORY_MODE_NORMAL;
	reqInfo.mpCMBuf = &CMBuf;
	reqInfo.pParams = pParams;

	gettimeofday(&reqInfo.enque_time, NULL);

	ImgInitParam initParam;
	initParam.mMaxFps = 30;
	initParam.mPriority = IMG_PRIORITY_PREVIEW;
	initParam.mLowLatency = 0;

	NSCam::NSImgStream::IImageBuffer imageDesc(toBufferPropery(infoDesc));
	VNDescBuf descBuf{
		.mFd = imageDesc.getPlaneFD(0),
		.mBufSize = (MINT32)imageDesc.getBufSizeInBytes(0),
		.mOffset = (MUINT32)imageDesc.getPlaneOffsetInBytes(0),
		.mpDescBufVa = (MINTPTR)infoDesc.address(0),
		.mbUsed = false,
	};

	syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID, infoCtrl.buffer()->planes()[0].fd.get());
	syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID, infoDesc.buffer()->planes()[0].fd.get());

	for (auto &frameParam : mvFrameParams) {
		for (auto &input : frameParam.mvIn)
			if (input.mPortIdx == NSCam::NSImgStream::IMG_PORT_METAI)
				syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID,
					  input.mBuffer->getPlaneFD(0));
	}

	createSingleDevBuffer(&reqInfo, &initParam, userid, V4L2_MODE_SIGNLE_DEVICE, &descBuf);

	for (auto &frameParam : mvFrameParams) {
		for (auto &input : frameParam.mvIn)
			if (input.mPortIdx == NSCam::NSImgStream::IMG_PORT_METAI)
				syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH,
					  input.mBuffer->getPlaneFD(0));
	}

	syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH, infoCtrl.buffer()->planes()[0].fd.get());
	syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH, infoDesc.buffer()->planes()[0].fd.get());

	infoDesc.buffer()->_d()->metadata().planes()[0].bytesused = infoDesc.buffer()->planes()[0].length;
}

std::vector<PEU_Stage> SingleDeviceRequest::getStageEnums() const
{
	std::vector<PEU_Stage> stageEnums;
	for (const auto &stage : stages_) {
		stageEnums.push_back(stage.getStageEnum());
	}
	return stageEnums;
}

} // namespace libcamera
