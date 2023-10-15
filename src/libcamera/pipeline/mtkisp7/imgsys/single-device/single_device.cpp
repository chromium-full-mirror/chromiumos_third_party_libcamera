/*
 * Copyright (C) 2022 MediaTek Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "single_device.h"

#include <cstring>
#include <vector>

#include <libcamera/formats.h>

#include "libcamera/internal/dma_heaps.h"
#include <libcamera/internal/formats.h>
#include "libcamera/internal/framebuffer.h"

#include "ImageFormat.h"
#include "ImgPortDef.h"

namespace NSCam {
namespace NSImgStream {

MINT IImageBuffer::getImgFormat() const
{
	switch(info.format()) {
	case libcamera::formats::SBGGR10_MTISP:
	case libcamera::formats::SGBRG10_MTISP:
	case libcamera::formats::SGRBG10_MTISP:
	case libcamera::formats::SRGGB10_MTISP:
		return eImgFmt_BAYER10;
	case libcamera::formats::NV12_10P_MTISP:
		return eImgFmt_MTK_YUV_P010;
	case libcamera::formats::NV12_12P_MTISP:
		return eImgFmt_MTK_YUV_P012;
	case libcamera::formats::NV12:
		return eImgFmt_NV12;
	case libcamera::formats::NV21:
		return eImgFmt_NV21;
	case libcamera::formats::GREY:
		return eImgFmt_Y8;
	case libcamera::formats::Y16_MTISP:
		return eImgFmt_STA_2BYTE;
	case libcamera::formats::Y32_MTISP:
		return eImgFmt_STA_4BYTE;
	case libcamera::formats::WARP2P_MTISP:
		return eImgFmt_WARP_2PLANE;
	case libcamera::formats::MTFD_MTISP:
		return eImgFmt_ISP_TUNING;
	default:
		std::abort();
	}
}

MSize const IImageBuffer::getImgSize() const
{
	return MSize(info.size().width, info.size().height);
}

size_t IImageBuffer::getPlaneCount() const
{
	return info.numPlanes();
}

MINT32 IImageBuffer::getColorArrangement() const
{
	switch(info.format()) {
	case libcamera::formats::SBGGR10_MTISP:
		return SENSOR_FORMAT_ORDER_RAW_B;
	case libcamera::formats::SGBRG10_MTISP:
		return SENSOR_FORMAT_ORDER_RAW_Gb;
	case libcamera::formats::SGRBG10_MTISP:
		return SENSOR_FORMAT_ORDER_RAW_Gr;
	case libcamera::formats::SRGGB10_MTISP:
		return SENSOR_FORMAT_ORDER_RAW_R;
	default:
		return -1;
	}
}

MINT32 IImageBuffer::getColorSpace() const
{
	return NSCam::eImgColorSpace_BT601_FULL;
}

MINT32 IImageBuffer::getPlaneFD(size_t index) const
{
	return info.buffer()->planes()[index].fd.get();
}

size_t IImageBuffer::getPlaneOffsetInBytes(size_t index) const
{
	return info.buffer()->planes()[index].offset;
}

MINTPTR IImageBuffer::getBufVA(size_t index) const
{
	return reinterpret_cast<MINTPTR>(info.address(index));
}

size_t IImageBuffer::getBufSizeInBytes(size_t index) const
{
	return info.buffer()->planes()[index].length;
}

size_t IImageBuffer::getBufStridesInBytes(size_t index) const
{
	const libcamera::PixelFormatInfo &formatInfo =
		libcamera::PixelFormatInfo::info(info.format());
	return formatInfo.stride(info.size().width, index);
}

size_t IImageBuffer::getBufScanlines(size_t index) const
{
	const libcamera::PixelFormatInfo &formatInfo =
		libcamera::PixelFormatInfo::info(info.format());

	unsigned int planeSize = formatInfo.planeSize(info.size(), index);
	return (planeSize / formatInfo.stride(info.size().width, index));
}

MBOOL IImageBuffer::syncCache(CacheCtrl const ctrl)
{
	/* todo: Collect fds from each planes and sync once */
	if (ctrl == eCACHECTRL_INVALID)
		libcamera::DmaHeap::sync(
				getPlaneFD(0),
				libcamera::DmaHeap::Start,
				libcamera::DmaHeap::SyncReadWrite);
	else
		libcamera::DmaHeap::sync(
				getPlaneFD(0),
				libcamera::DmaHeap::End,
				libcamera::DmaHeap::SyncReadWrite);
	return true;
}

SecType IImageBuffer::getSecType() const
{
	return (SecType)0;
}

bool syncCache(NSCam::NSImgStream::CacheCtrl const ctrl, libcamera::FrameBuffer *buffer)
{
	/* todo: Collect fds from each planes and sync once */
	if (ctrl == NSCam::NSImgStream::eCACHECTRL_INVALID)
		libcamera::DmaHeap::sync(
				buffer->planes()[0].fd.get(),
				libcamera::DmaHeap::Start,
				libcamera::DmaHeap::SyncReadWrite);
	else
		libcamera::DmaHeap::sync(
				buffer->planes()[0].fd.get(),
				libcamera::DmaHeap::End,
				libcamera::DmaHeap::SyncReadWrite);
	return true;
}

} // NSImgStream
} // NSCam

namespace libcamera {

using namespace NSCam::NSImgStream;

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
	auto crpInfo = WPE_CrpInfo{.x_start_point=0, .x_end_point=crop.width - 1,
				   .y_start_point=0, .y_end_point=crop.height - 1};

	auto crpOfstInfo = WPE_CrpOfstInfo{.x_start=0, .hr_int_ofst=0,
				      .hr_sub_ofst=0, .y_start=0,
				      .vt_int_ofst=0, .vt_sub_ofst=0,
				      .wd=0, .ht=0};

	param.mData.mWPEInfo = WPEInfo{
		.wpe_mode=(WPE_MODE)mode,
		.vgen_out=crpInfo,
		.tbl_sel_v=(PSP_TABLE_SEL)1, .tbl_sel_h=(PSP_TABLE_SEL)1,
		.extra_feature_index=featureIndex, .rgb_mode=(RGB_MODE)0,
		.vgen_in=crpOfstInfo, .psp_border_color_y=0, .psp_border_color_u=0,
		.psp_border_color_v=0};
}

void StageEx::setMvFrame(Size f0, Size me)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_MVFRAME_INFO;
	param.mData.mMVFrameInfo = MVFrameInfo{
		.mF0Width=f0.width, .mF0Height=f0.height,
		.mME0Width=me.width, .mME0Height=me.height,
		.mConfScaleRatio=4};
}

void StageEx::setMeInfo(NSCam::NSImgStream::ME_MODE mode)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_ME_INFO;
	param.mData.mMEInfo = MEInfo{.me_mode=mode};
}

void StageEx::setAplInfo()
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_APL_INFO;
	param.mData.mAPLInfo = APLInfo{.mAplEnable=1};
}

void StageEx::setMultiScale(NSCam::NSImgStream::IMG_MULTI_SCALE_RATIO ratio,
			    uint32_t index, uint32_t total)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_DIP_MULTISCALE_INFO;
	param.mData.mMutiScaleInfo = MultiScaleInfo{
		.mScaleRatio=ratio, .mScaleIdx=index, .mScaleTotal=total};
}

void StageEx::setPqInfo()
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_PQ_PORT_INFO;
	param.mData.mPQPortInfo = PQPortInfo{
	    .mWdmaoPQIdx=1, .mWdmaoUserString=0, .mWdmaoBypassCrop=0,
	    .mWrotoPQIdx=2, .mWrotoUserString=0, .mWrotoBypassCrop=0};
}

void StageEx::setImg4oCrop(const Rectangle &crop)
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_P_IMG4O_CROP_INFO;
	param.mData.mPImg4oCropInfo = PImg4oCropInfo{
		.p_img4o_crop_x=(uint32_t)crop.x,
		.p_img4o_crop_y=(uint32_t)crop.y,
		.p_img4o_crop_w=(uint32_t)crop.width,
		.p_img4o_crop_h=(uint32_t)crop.height,
		.tnrwo_scale_ratio=8};
}

void StageEx::setCostLevel()
{
	extra_.emplace_back();
	auto &param = extra_.back();

	param.mID = IMG_EXTRA_PARAM_ID_COST_LEVEL_INFO;
	param.mData.mCostLevel = WPECostLevel{.costlevel=(COST_LEVEL)0};
}

void StageEx::addNotify(uint32_t sync)
{
	mSyncTokenNotifyList.push_back(sync);
}

void StageEx::addWait(uint32_t sync)
{
	mSyncTokenWaitList.push_back(sync);
}

void SingleDeviceRequest::fillFrameParams(std::vector<NSCam::NSImgStream::FrameParams> &mvFrameParams)
{
	for (auto &stage : stages_) {
		auto &frameParams = mvFrameParams.emplace_back();

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
}

void SingleDeviceRequest::fillRequestBuffer(InfoFrame &infoCtrl,
					    InfoFrame &infoDesc,
					    int requestFd)
{
	std::vector<FrameParams> mvFrameParams;
	fillFrameParams(mvFrameParams);

	std::shared_ptr<ImgParams> pParams = std::make_shared<ImgParams>();
	pParams->mHWSharing = 0;
	pParams->mFps = 30;
	pParams->mSyncID = -1;
	pParams->mRequestNo = sequence();
	pParams->mFrameNo = sequence();
	pParams->mNumBatchRun = 1;
	pParams->mvFrameParams.swap(mvFrameParams);


	NSCam::NSImgStream::IImageBuffer imageCM(infoCtrl);
	CtrlMetaBuf CMBuf {
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

	NSCam::NSImgStream::IImageBuffer imageDesc(infoDesc);
	VNDescBuf descBuf{
		.mFd = imageDesc.getPlaneFD(0),
		.mBufSize = (MINT32)imageDesc.getBufSizeInBytes(0),
		.mOffset = (MUINT32)imageDesc.getPlaneOffsetInBytes(0),
		.mpDescBufVa = (MINTPTR)infoDesc.address(0),
		.mbUsed = false,
	};

	syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID, infoCtrl.buffer());
	syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID, infoDesc.buffer());

	for (auto &frameParam : mvFrameParams) {
		for (auto &input : frameParam.mvIn)
			if (input.mPortIdx == NSCam::NSImgStream::IMG_PORT_METAI)
				syncCache(NSCam::NSImgStream::eCACHECTRL_INVALID,
						  input.mBuffer->info.buffer());
	}

	createSingleDevBuffer(&reqInfo, &initParam, userid, V4L2_MODE_SIGNLE_DEVICE, &descBuf);

	for (auto &frameParam : mvFrameParams) {
		for (auto &input : frameParam.mvIn)
			if (input.mPortIdx == NSCam::NSImgStream::IMG_PORT_METAI)
				syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH,
						  input.mBuffer->info.buffer());
	}

	syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH, infoCtrl.buffer());
	syncCache(NSCam::NSImgStream::eCACHECTRL_FLUSH, infoDesc.buffer());

	infoDesc.buffer()->_d()->metadata().planes()[0].bytesused = infoDesc.buffer()->planes()[0].length;
}

std::vector<PEU_Stage> SingleDeviceRequest::getStageEnums() const
{
	std::vector<PEU_Stage> stageEnums;
	for (const auto &stage: stages_) {
		stageEnums.push_back(stage.getStageEnum());
	}
	return stageEnums;
}

} // namespace libcamera


