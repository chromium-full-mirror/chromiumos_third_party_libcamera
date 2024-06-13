/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * mfnr.cpp - MtkISP7 ImgSys Device Mutiple Frame Noise Reduction
 */

#include "mfnr.h"

#include <cstddef>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <string>
#include <unistd.h>
#include <vector>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/request.h>

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/imgsys/bss.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "ImgPortDef.h"
#include "single_device.h"
#include "single_device_helper.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

constexpr const char *kEnforceMfnr = "/run/camera/enforce_mfnr";
constexpr const char *kMfnrPrecheck = "/run/camera/mfnr_precheck";

constexpr Size kP2sttoSize{ 738624, 1 };
constexpr Size kTnrsoSize{ 40, 1 };
constexpr Size kBssGmDataMSize{ 5, 1 };
constexpr Size kWrotoSize{ 192, 144 };
using namespace NSCam::NSImgStream;

static void zeroImage(SharedMailBox<InfoFrame> &mailBox)
{
	const InfoFrame &info = mailBox->get();

	void *dest = info.address(0);
	size_t length = info.buffer()->planes()[0].length;

	assert(dest);
	assert(mailBox->valid());

	{
		DmaSyncer syncer(info.buffer()->planes()[0].fd.get());
		memset(dest, 0, length);
	}
}

MfnrTasksManager::MfnrTasksManager(
	ImgSysDevice *imgSys, DmaHeap *dmaHeap, OnDeviceTuner *odt)
{
	imgSys_ = imgSys;
	dmaHeap_ = dmaHeap;
	onDeviceTuner_ = odt;

	allBufferPools_.emplace_back(&bssParamPool_);
	allBufferPools_.emplace_back(&bssDataGPool_);
	allBufferPools_.emplace_back(&bssVerPool_);
	allBufferPools_.emplace_back(&bssFdMainPool_);
	allBufferPools_.emplace_back(&bssTuningPool_);
	allBufferPools_.emplace_back(&bssOutDataPool_);
	allBufferPools_.emplace_back(&bssFdMainPool_);
	allBufferPools_.emplace_back(&bssFdPool_);
	allBufferPools_.emplace_back(&bssFacePool_);
	allBufferPools_.emplace_back(&bssPosPool_);

	allBufferPools_.emplace_back(&swmeParamPool_);
	allBufferPools_.emplace_back(&swmeOutPool_);
	allBufferPools_.emplace_back(&swmeTuningPool_);

	allBufferPools_.emplace_back(&tunbufiPool_);
	allBufferPools_.emplace_back(&p2sttoPool_);
	allBufferPools_.emplace_back(&tnrciPool_);
	allBufferPools_.emplace_back(&wrap2pPool_);
	allBufferPools_.emplace_back(&yuvp010_1_1_pool_);
	allBufferPools_.emplace_back(&yuvp010_1_4_pool_);
	allBufferPools_.emplace_back(&yuvp010_1_4_pool_aligned16_);
	allBufferPools_.emplace_back(&yuvp012_1_1_pool_);
	allBufferPools_.emplace_back(&yuvp012_1_2_pool_);
	allBufferPools_.emplace_back(&yuvp012_1_4_pool_);
	allBufferPools_.emplace_back(&yuvp012_1_8_pool_);
	allBufferPools_.emplace_back(&yuvp012_1_16_pool_);
	allBufferPools_.emplace_back(&yuvp012_1_32_pool_);
	allBufferPools_.emplace_back(&yuvp012_1_64_pool_);
	allBufferPools_.emplace_back(&y8_1_1_pool_);
	allBufferPools_.emplace_back(&y8_1_2_pool_);
	allBufferPools_.emplace_back(&y8_1_4_pool_);
	allBufferPools_.emplace_back(&y8_1_4_pool_aligned16_);
	allBufferPools_.emplace_back(&y8_1_8_pool_);
	allBufferPools_.emplace_back(&y8_1_16_pool_);
	allBufferPools_.emplace_back(&y8_1_32_pool_);

	allBufferPools_.emplace_back(&fourBytes_pool_);
	allBufferPools_.emplace_back(&fourBytes_1_16_pool_);
	allBufferPools_.emplace_back(&nv21_1_64_pool_);
	allBufferPools_.emplace_back(&nv21_1_1_pool_);
	allBufferPools_.emplace_back(&nv12_wroto_pool_);
	allBufferPools_.emplace_back(&memc_workbuf_pool_);

	poolsWritenByCpu_.emplace_back(&bssParamPool_);
	poolsWritenByCpu_.emplace_back(&bssDataGPool_);
	poolsWritenByCpu_.emplace_back(&bssVerPool_);
	poolsWritenByCpu_.emplace_back(&bssFdMainPool_);
	poolsWritenByCpu_.emplace_back(&bssTuningPool_);
	poolsWritenByCpu_.emplace_back(&bssOutDataPool_);
	poolsWritenByCpu_.emplace_back(&bssFdMainPool_);
	poolsWritenByCpu_.emplace_back(&bssFdPool_);
	poolsWritenByCpu_.emplace_back(&bssFacePool_);
	poolsWritenByCpu_.emplace_back(&bssPosPool_);

	poolsWritenByCpu_.emplace_back(&swmeParamPool_);
	poolsWritenByCpu_.emplace_back(&swmeOutPool_);
	poolsWritenByCpu_.emplace_back(&swmeTuningPool_);

	poolsWritenByCpu_.emplace_back(&tunbufiPool_);
	poolsWritenByCpu_.emplace_back(&p2sttoPool_);
	poolsWritenByCpu_.emplace_back(&tnrciPool_);
	poolsWritenByCpu_.emplace_back(&wrap2pPool_);
	poolsWritenByCpu_.emplace_back(&yuvp010_1_1_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp010_1_4_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp010_1_4_pool_aligned16_);
	poolsWritenByCpu_.emplace_back(&yuvp012_1_1_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp012_1_2_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp012_1_4_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp012_1_8_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp012_1_16_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp012_1_32_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp012_1_64_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_1_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_2_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_4_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_4_pool_aligned16_);
	poolsWritenByCpu_.emplace_back(&y8_1_8_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_16_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_32_pool_);
	poolsWritenByCpu_.emplace_back(&fourBytes_pool_);
	poolsWritenByCpu_.emplace_back(&fourBytes_1_16_pool_);
	poolsWritenByCpu_.emplace_back(&nv21_1_64_pool_);
	poolsWritenByCpu_.emplace_back(&nv21_1_1_pool_);
	poolsWritenByCpu_.emplace_back(&nv12_wroto_pool_);
	poolsWritenByCpu_.emplace_back(&memc_workbuf_pool_);
}

int MfnrTasksManager::configure(const Size &bayerInputSize,
				const Size &yuvOutputSize1, const Size &yuvOutputSize2,
				const Size &videoOutputSize1, const Size &videoOutputSize2,
				int sensor_idx)
{
	yuvOutputSize1_ = yuvOutputSize1;
	yuvOutputSize2_ = yuvOutputSize2;
	videoOutputSize1_ = videoOutputSize1;
	videoOutputSize2_ = videoOutputSize2;
	bayerInputSize_ = bayerInputSize;
	sensor_idx_ = sensor_idx;

	mfnrSizes_.resize(7);
	mfnrSizes_aligned16_.resize(3);
	Size size = bayerInputSize_;

	/* Assign the size to 1/2 of the previous level.
	 * Align to 2 for hardware's requirement */
	for (size_t i = 0; i < mfnrSizes_.size(); i++) {
		LOG(MtkISP7, Info) << "mfnrSizes_[" << i << "] = " << size;
		mfnrSizes_[i] = size;
		size.width = (size.width + 1) / 2;
		size.height = (size.height + 1) / 2;
		size.alignUpTo(2, 2);
	}
	size = bayerInputSize_;
	for (size_t i = 0; i < mfnrSizes_aligned16_.size(); i++) {
		LOG(MtkISP7, Info) << "mfnrSizes_aligned16_[" << i << "] = " << size;
		mfnrSizes_aligned16_[i] = size;
		size.width = (size.width + 1) / 2;
		size.height = (size.height + 1) / 2;
		size.alignDownTo(16, 16);
	}

	for (auto i = 0; i < kInputRawCount - 1; i++) {
		std::shared_ptr<SwmeWrapper> swmewrapper = std::make_shared<SwmeWrapper>();
		Size swme_in = mfnrSizes_aligned16_[2];
		swmewrapper->setMotionEstimationResolution(swme_in.width, swme_in.height);
		swmewrapper->init();
		swmeWrapper_.push_back(swmewrapper);
	}
	swmeWorkingBufSize_ = swmeWrapper_[0]->getAlgorithmWorkBufferSize();
	wrappingMapSize_ = swmeWrapper_[0]->getWarppingMapSize();
	confMapSize_ = swmeWrapper_[0]->getConfMapSize();
	bssWrapper_ = std::make_shared<BssWrapper>(sensor_idx_);
	bssWrapper_->bssInit();

	configureBuffers();
	for (auto &pool : poolsWritenByCpu_)
		pool->mmap();
	return 0;
}

int MfnrTasksManager::configureBuffers()
{
	bssParamPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IBSS_PARAM_STRUCT), 1), 1);
	bssDataGPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IBSS_INPUT_DATA_G), 1), 1);
	bssVerPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kBssGmDataMSize, 1);
	bssTuningPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(mtk::isphal::v1::isp_bss_Param), 1), 1);
	bssFdMainPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(FD_DATATYPE), 1), kInputRawCount);
	bssFdPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IBssFaceMetadata), 1), kInputRawCount);
	bssFacePool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IBssFace) * 15, 1), kInputRawCount);
	bssPosPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IBssFaceInfo) * 15, 1), kInputRawCount);

	swmeOutPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IMFBLL_PROC1_OUT_STRUCT), 1), kInputRawCount - 1);
	swmeParamPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IMFBLL_SET_PROC_INFO_STRUCT), 1), kInputRawCount - 1);
	swmeTuningPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(mtk::isphal::v1::isp_swme_Param), 1), kInputRawCount - 1);

	bssOutDataPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size(sizeof(IBSS_OUTPUT_DATA), 1), 1);
	p2sttoPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kP2sttoSize, 5, DmaHeap::CMA);
	wrap2pPool_.createBuffers(dmaHeap_, formats::WARP2P_MTISP, wrappingMapSize_, 4, DmaHeap::System, 1, 1);
	tnrciPool_.createBuffers(dmaHeap_, formats::Y8_MTISP, confMapSize_, 3, DmaHeap::System);

	yuvp010_1_1_pool_.createBuffers(dmaHeap_, formats::NV12_10P_MTISP, mfnrSizes_[0], 9);
	yuvp010_1_4_pool_.createBuffers(dmaHeap_, formats::NV12_10P_MTISP, mfnrSizes_[2], 10, DmaHeap::System, 16);
	yuvp010_1_4_pool_aligned16_.createBuffers(dmaHeap_, formats::NV12_10P_MTISP, mfnrSizes_aligned16_[2], 10, DmaHeap::System, 16);

	yuvp012_1_1_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[0], 12, DmaHeap::System, 16, 16);
	yuvp012_1_2_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[1], 12, DmaHeap::System, 16, 16);
	yuvp012_1_4_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[2], 12, DmaHeap::System, 16, 16);
	y8_1_4_pool_aligned16_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_aligned16_[2], 12, DmaHeap::System, 8, 8);

	yuvp012_1_8_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[3], 12, DmaHeap::System, 16, 16);
	yuvp012_1_16_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[4], 12, DmaHeap::System, 16, 16);
	yuvp012_1_32_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[5], 12, DmaHeap::System, 16, 16);
	yuvp012_1_64_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[6], 12, DmaHeap::System, 16, 16);

	y8_1_1_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[0], 10, DmaHeap::System, 16, 16);
	y8_1_2_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[1], 10, DmaHeap::System, 16, 16);
	y8_1_4_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[2], 14, DmaHeap::System, 16, 16);
	y8_1_8_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[3], 10, DmaHeap::System, 16, 16);
	y8_1_16_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[4], 13, DmaHeap::System, 16, 16);
	y8_1_32_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[5], 13, DmaHeap::System, 16, 16);

	fourBytes_pool_.createBuffers(dmaHeap_, formats::Y32_MTISP, kTnrsoSize, 28);
	fourBytes_1_16_pool_.createBuffers(dmaHeap_, formats::Y32_MTISP, mfnrSizes_[4], 3);
	nv21_1_1_pool_.createBuffers(dmaHeap_, formats::NV21, mfnrSizes_[0], 7);
	nv21_1_64_pool_.createBuffers(dmaHeap_, formats::NV21, mfnrSizes_[6], 9);
	nv12_wroto_pool_.createBuffers(dmaHeap_, formats::NV12, kWrotoSize, 7);

	memc_workbuf_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, swmeWorkingBufSize_, 5);

	return 0;
}

int MfnrTasksManager::start()
{
#if !V4L2_STANDARD_MODE
	for (auto &pool : allBufferPools_)
		imgSys_->handleIova(ImgSysDevice::Add, *pool);
#endif
	return 0;
}

int MfnrTasksManager::stop()
{
#if !V4L2_STANDARD_MODE
	for (auto &pool : allBufferPools_)
		imgSys_->handleIova(ImgSysDevice::Delete, *pool);
#endif
	return 0;
}

int MfnrTasksManager::releaseBuffers()
{
	for (auto &pool : allBufferPools_)
		pool->release();

	return 0;
}

bool MfnrTasksManager::forceMfnr()
{
	if (std::filesystem::exists(kEnforceMfnr)) {
		return true;
	}
	return false;
}

bool MfnrTasksManager::mfnrPrecheck()
{
	if (std::filesystem::exists(kMfnrPrecheck)) {
		return true;
	}
	return false;
}

void MfnrTasksManager::makeMFNRFrames(
	MFNRFrames &mfnr,
	std::array<SharedMailBox<InfoFrame>, MFNR_QUEUE_SIZE> &captureRawQueue,
	std::array<SharedMailBox<InfoFrame>, MFNR_QUEUE_SIZE> &previewQueue,
	int captureRawQueue_idx,
	FrameBuffer *output1Frame,
	FrameBuffer *output2Frame)
{
	mfnr.still1Output = output1Frame;
	mfnr.still2Output = output2Frame;

	/* Should be created and generated by IPA, once it's ready */
	SharedMailBox<std::vector<int>> bssOrder = makeMailBox<std::vector<int>>();

	std::vector<SharedMailBox<InfoFrame>> bssFdMain = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bssFd = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bssFace = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bssPos = makeMailBoxVector<InfoFrame>(kInputRawCount);

	std::vector<SharedMailBox<InfoFrame>> bfbldTun = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bfbldP2stto = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bfbldImg2o = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bfbldImg3o = makeMailBoxVector<InfoFrame>(kInputRawCount);

	std::vector<SharedMailBox<InfoFrame>> bfmeTun = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bfmeImg2o = makeMailBoxVector<InfoFrame>(kInputRawCount);

	std::vector<SharedMailBox<InfoFrame>> swmeTun = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> swmeWorkBuf = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> swmeConfMapBuf = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> swmeWrappingBuf = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> swmeMcmvBuf = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> swmeParamInBuf = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> swmeParamOutBuf = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<std::shared_ptr<mtk::isphal::v1::isp_swme_Param>>> swmeDbParam =
		makeMailBoxVector<std::shared_ptr<mtk::isphal::v1::isp_swme_Param>>(kInputRawCount - 1);

	std::vector<SharedMailBox<InfoFrame>> mcdsF1Tun = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);

	std::vector<SharedMailBox<InfoFrame>> mcdsWpeWpeo = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> mcdsF1Ltyuv2o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> mcdsF1Ltyuv3o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> mcdsF1Ltyuv4o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> mcdsF1Ltyuv5o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);

	std::vector<SharedMailBox<InfoFrame>> dsTun = makeMailBoxVector<InfoFrame>(kInputRawCount + 1);
	std::vector<SharedMailBox<InfoFrame>> dsYuv2o = makeMailBoxVector<InfoFrame>(kInputRawCount + 1);
	std::vector<SharedMailBox<InfoFrame>> dsYuv3o = makeMailBoxVector<InfoFrame>(kInputRawCount + 1);
	std::vector<SharedMailBox<InfoFrame>> dsYuv4o = makeMailBoxVector<InfoFrame>(kInputRawCount + 1);

	std::vector<SharedMailBox<InfoFrame>> dsVbiV2Tun = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> dsVbiV2Tyuv2o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> dsVbiV2Tyuv4o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> dsVbiV2Tyuv3o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);

	std::vector<SharedMailBox<InfoFrame>> dsVbiV5Tun = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> dsVbiV5Tyuv2o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> dsVbiV5Tyuv4o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);
	std::vector<SharedMailBox<InfoFrame>> dsVbiV5Tyuv3o = makeMailBoxVector<InfoFrame>(kInputRawCount - 1);

	std::vector<SharedMailBox<InfoFrame>> msbldFx_Tnrwi = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tun = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrsi = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrci = makeMailBoxVector<InfoFrame>(1);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Img4o = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrwo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrmo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrso = makeMailBoxVector<InfoFrame>(7);

	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Tun = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Tnrci = makeMailBoxVector<InfoFrame>(1);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Img4o = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Tnrwo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Tnrmo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Tnrso = makeMailBoxVector<InfoFrame>(7);

	std::vector<SharedMailBox<InfoFrame>> afbldFx_Tun = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Wroto = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Wdmao = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Img3o = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Img4o = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Tnrwo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Tnrmo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Tnrso = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Tnrci = makeMailBoxVector<InfoFrame>(1);

	/* Frames used by Bsstask */
	BssFrames &bssFrames = mfnr.bssFrames;
	bssFrames.in.bssParamInfo = makeMailBox<InfoFrame>();
	bssFrames.in.bssDataGInfo = makeMailBox<InfoFrame>();
	bssFrames.in.db_param = makeMailBox<std::shared_ptr<mtk::isphal::v1::isp_bss_Param>>();
	bssFrames.in.bssTuningInfo = makeMailBox<InfoFrame>();
	bssFrames.in.bssVerInfo = makeMailBox<InfoFrame>();
	bssFrames.out.bssOutDataInfo = makeMailBox<InfoFrame>();

	bssParamPool_.fetch(bssFrames.in.bssParamInfo);
	bssDataGPool_.fetch(bssFrames.in.bssDataGInfo);
	bssTuningPool_.fetch(bssFrames.in.bssTuningInfo);
	bssVerPool_.fetch(bssFrames.in.bssVerInfo);
	bssOutDataPool_.fetch(bssFrames.out.bssOutDataInfo);

	bssFrames.in.imgi.resize(kInputRawCount);
	bssFrames.in.bssFdMainInfo.resize(kInputRawCount);
	bssFrames.in.bssFdInfo.resize(kInputRawCount);
	bssFrames.in.bssFaceInfo.resize(kInputRawCount);
	bssFrames.in.bssPosInfo.resize(kInputRawCount);
	mfnr.bss_order = bssOrder;
	bssFrames.out.bss_order = bssOrder;
	for (auto i = 0; i < kInputRawCount; i++) {
		int idx = (captureRawQueue_idx - (kInputRawCount - 1 - i) + MFNR_QUEUE_SIZE) % MFNR_QUEUE_SIZE;
		bssFrames.in.imgi[i] = previewQueue[idx];
		bssFrames.in.bssFdMainInfo[i] = bssFdMain[i];
		bssFrames.in.bssFdInfo[i] = bssFd[i];
		bssFrames.in.bssFaceInfo[i] = bssFace[i];
		bssFrames.in.bssPosInfo[i] = bssPos[i];
		bssFdMainPool_.fetch(bssFrames.in.bssFdMainInfo[i]);
		bssFdPool_.fetch(bssFrames.in.bssFdInfo[i]);
		bssFacePool_.fetch(bssFrames.in.bssFaceInfo[i]);
		bssPosPool_.fetch(bssFrames.in.bssPosInfo[i]);
	}
	/* Frames used by BfbldTask */
	BfbldFrames &bfbldFrames = mfnr.bfbldFrames;
	bfbldFrames.capturedRaws.resize(kInputRawCount);
	bfbldFrames.in.timgi.resize(kInputRawCount);
	for (auto i = 0; i < kInputRawCount; i++) {
		int idx = (captureRawQueue_idx - (kInputRawCount - 1 - i) + MFNR_QUEUE_SIZE) % MFNR_QUEUE_SIZE;
		bfbldFrames.capturedRaws[i] = captureRawQueue[idx];
		bfbldFrames.in.tunbufi.push_back(bfbldTun[i]);
		bfbldFrames.out.p2stto.push_back(bfbldP2stto[i]);
		bfbldFrames.out.img2o.push_back(bfbldImg2o[i]);
		bfbldFrames.out.img3o.push_back(bfbldImg3o[i]);
	}
	/* Frames used by BfmeTask */
	BfmeFrames &bfmeFrames = mfnr.bfmeFrames;
	bfmeFrames.tncso = bfbldFrames.out.p2stto[0];
	for (auto i = 0; i < kInputRawCount; i++) {
		bfmeFrames.in.imgi.push_back(bfbldFrames.out.img2o[i]);
		bfmeFrames.in.tunbufi.push_back(bfmeTun[i]);
		bfmeFrames.out.img2o.push_back(bfmeImg2o[i]);
	}

	/* Frames used by BfmeTask */
	SwmeFrames &swmeFrame = mfnr.swmeFrames;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		memc_workbuf_pool_.fetch(swmeWorkBuf[i]);
		swmeFrame.in.bss_buf.push_back(bssFrames.out.bssOutDataInfo);
		swmeFrame.in.workbuf.push_back(swmeWorkBuf[i]);
		swmeFrame.in.base_buf.push_back(bfmeFrames.out.img2o[0]);
		swmeFrame.in.ref_buf.push_back(bfmeFrames.out.img2o[i + 1]);
		swmeFrame.in.db_param.push_back(swmeDbParam[i]);
		swmeFrame.in.tuningInfo.push_back(swmeTun[i]);
		swmeFrame.in.paramInInfo.push_back(swmeParamInBuf[i]);
		swmeFrame.out.warpping_map.push_back(swmeWrappingBuf[i]);
		swmeFrame.out.conf_map.push_back(swmeConfMapBuf[i]);
		swmeFrame.out.mcmv.push_back(swmeMcmvBuf[i]);
		swmeFrame.out.paramOutInfo.push_back(swmeParamOutBuf[i]);
	}

	/* Frames used by MCDS_F1 Task */
	//mcdsWpeVeci.resize(3);
	McdsF1Frames &mcdsF1Frames = mfnr.mcdsF1Frames;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		mcdsF1Frames.in.wpe_wpei.push_back(bfbldFrames.out.img3o[i + 1]);
		mcdsF1Frames.in.tunbufi.push_back(mcdsF1Tun[i]);
		mcdsF1Frames.in.wpe_veci.push_back(swmeFrame.out.warpping_map[i]);
		mcdsF1Frames.out.wpe_wpeo.push_back(mcdsWpeWpeo[i]);
		mcdsF1Frames.out.ltyuv2o.push_back(mcdsF1Ltyuv2o[i]);
		mcdsF1Frames.out.ltyuv3o.push_back(mcdsF1Ltyuv3o[i]);
		mcdsF1Frames.out.ltyuv4o.push_back(mcdsF1Ltyuv4o[i]);
		mcdsF1Frames.out.ltyuv5o.push_back(mcdsF1Ltyuv5o[i]);
	}

	/* Frames used by DS Task */
	DsFrames &dsFrames = mfnr.dsFrames;
	dsFrames.in.ltimgi.push_back(bfbldFrames.out.img3o[0]);
	dsFrames.in.tunbufi.push_back(dsTun[0]);
	dsFrames.out.ltyuv2o.push_back(dsYuv2o[0]);
	dsFrames.out.ltyuv3o.push_back(dsYuv3o[0]);
	dsFrames.out.ltyuv4o.push_back(dsYuv4o[0]);
	for (auto i = 1; i < kInputRawCount + 1; i++) {
		if (i == 1) {
			dsFrames.in.ltimgi.push_back(dsFrames.out.ltyuv4o[0]);
		} else {
			dsFrames.in.ltimgi.push_back(mcdsF1Frames.out.ltyuv4o[i - 2]);
		}
		//tunbufiPool_.fetch(dsTun[i]);
		dsFrames.in.tunbufi.push_back(dsTun[i]);
		dsFrames.out.ltyuv2o.push_back(dsYuv2o[i]);
		dsFrames.out.ltyuv3o.push_back(dsYuv3o[i]);
		dsFrames.out.ltyuv4o.push_back(dsYuv4o[i]);
	}

	/* Frames used by DS_VBI Task */
	DsVbiFrames &dsVbiFramesV2 = mfnr.dsVbiFramesV2;
	DsVbiFrames &dsVbiFramesV5 = mfnr.dsVbiFramesV5;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		dsVbiFramesV2.in.timgi.push_back(mcdsF1Frames.out.ltyuv5o[i]);
		//tunbufiPool_.fetch(dsVbiV2Tun[i]);
		dsVbiFramesV2.in.tunbufi.push_back(dsVbiV2Tun[i]);
		dsVbiFramesV2.out.tyuv2o.push_back(dsVbiV2Tyuv2o[i]);
		dsVbiFramesV2.out.tyuv3o.push_back(dsVbiV2Tyuv3o[i]);
		dsVbiFramesV2.out.tyuv4o.push_back(dsVbiV2Tyuv4o[i]);

		dsVbiFramesV5.in.timgi.push_back(dsVbiFramesV2.out.tyuv4o[i]);
		//tunbufiPool_.fetch(dsVbiV5Tun[i]);
		dsVbiFramesV5.in.tunbufi.push_back(dsVbiV5Tun[i]);
		dsVbiFramesV5.out.tyuv2o.push_back(dsVbiV5Tyuv2o[i]);
		dsVbiFramesV5.out.tyuv3o.push_back(dsVbiV5Tyuv3o[i]);
		dsVbiFramesV5.out.tyuv4o.push_back(dsVbiV5Tyuv4o[i]);
	}

	auto constructMsbldMailBox =
		[](MsbldFrames &msbld, int idx,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tun,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Img4o,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tnrmo,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tnrso,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tnrwo,
		   SharedMailBox<InfoFrame> &msbldFx_Tnrci) {
			msbld.in.tunbufi.push_back(msbldFx_Tun[idx]);
			msbld.in.tnrci.push_back(msbldFx_Tnrci);
			msbld.out.img4o.push_back(msbldFx_Img4o[idx]);
			msbld.out.tnrmo.push_back(msbldFx_Tnrmo[idx]);
			msbld.out.tnrso.push_back(msbldFx_Tnrso[idx]);
			msbld.out.tnrwo.push_back(msbldFx_Tnrwo[idx]);
		};

	auto constructAfbldMailBox =
		[](AfbldFrames &afbld, int idx,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Tun,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Wroto,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Wdmao,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Img3o,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Img4o,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Tnrwo,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Tnrmo,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Tnrso,
		   SharedMailBox<InfoFrame> &afbldFx_Tnrci) {
			afbld.in.tunbufi.push_back(afbldFx_Tun[idx]);
			afbld.in.tnrci.push_back(afbldFx_Tnrci);
			afbld.out.img4o.push_back(afbldFx_Img4o[idx]);
			afbld.out.img3o.push_back(afbldFx_Img3o[idx]);
			afbld.out.wdmao.push_back(afbldFx_Wdmao[idx]);
			afbld.out.wroto.push_back(afbldFx_Wroto[idx]);
			afbld.out.tnrwo.push_back(afbldFx_Tnrwo[idx]);
			afbld.out.tnrmo.push_back(afbldFx_Tnrmo[idx]);
			afbld.out.tnrso.push_back(afbldFx_Tnrso[idx]);
		};

	/* Frames used by MSBLD*/
	MsbldFrames &msbldF6 = mfnr.msbldF6;
	MsbldFrames &msbldF5 = mfnr.msbldF5;
	MsbldFrames &msbldF4 = mfnr.msbldF4;
	MsbldFrames &msbldF3 = mfnr.msbldF3;
	MsbldFrames &msbldF2 = mfnr.msbldF2;
	MsbldFrames &msbldF1 = mfnr.msbldF1;
	MsbldFrames &msbldF0 = mfnr.msbldF0;

	AfbldFrames &afbldF6 = mfnr.afbldF6;
	AfbldFrames &afbldF5 = mfnr.afbldF5;
	AfbldFrames &afbldF4 = mfnr.afbldF4;
	AfbldFrames &afbldF3 = mfnr.afbldF3;
	AfbldFrames &afbldF2 = mfnr.afbldF2;
	AfbldFrames &afbldF1 = mfnr.afbldF1;
	AfbldFrames &afbldF0 = mfnr.afbldF0;

	msbldFx_0Tnrci[0] = swmeFrame.out.conf_map[0];
	msbldFx_1Tnrci[0] = swmeFrame.out.conf_map[1];
	afbldFx_Tnrci[0] = swmeFrame.out.conf_map[2];

	mfnr.msbld_tnrso = makeMailBox<InfoFrame>();
	fourBytes_pool_.fetch(mfnr.msbld_tnrso);
	constructMsbldMailBox(msbldF6, 6, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, msbldFx_0Tnrso, msbldFx_0Tnrwo, swmeFrame.out.conf_map[0]);
	constructMsbldMailBox(msbldF5, 5, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, msbldFx_0Tnrso, msbldFx_0Tnrwo, swmeFrame.out.conf_map[0]);
	constructMsbldMailBox(msbldF4, 4, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, msbldFx_0Tnrso, msbldFx_0Tnrwo, swmeFrame.out.conf_map[0]);
	constructMsbldMailBox(msbldF3, 3, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, msbldFx_0Tnrso, msbldFx_0Tnrwo, swmeFrame.out.conf_map[0]);
	constructMsbldMailBox(msbldF2, 2, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, msbldFx_0Tnrso, msbldFx_0Tnrwo, swmeFrame.out.conf_map[0]);
	constructMsbldMailBox(msbldF1, 1, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, msbldFx_0Tnrso, msbldFx_0Tnrwo, swmeFrame.out.conf_map[0]);
	constructMsbldMailBox(msbldF0, 0, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, msbldFx_0Tnrso, msbldFx_0Tnrwo, swmeFrame.out.conf_map[0]);

	constructMsbldMailBox(msbldF6, 6, msbldFx_1Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, msbldFx_1Tnrso, msbldFx_1Tnrwo, swmeFrame.out.conf_map[1]);
	constructMsbldMailBox(msbldF5, 5, msbldFx_1Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, msbldFx_1Tnrso, msbldFx_1Tnrwo, swmeFrame.out.conf_map[1]);
	constructMsbldMailBox(msbldF4, 4, msbldFx_1Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, msbldFx_1Tnrso, msbldFx_1Tnrwo, swmeFrame.out.conf_map[1]);
	constructMsbldMailBox(msbldF3, 3, msbldFx_1Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, msbldFx_1Tnrso, msbldFx_1Tnrwo, swmeFrame.out.conf_map[1]);
	constructMsbldMailBox(msbldF2, 2, msbldFx_1Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, msbldFx_1Tnrso, msbldFx_1Tnrwo, swmeFrame.out.conf_map[1]);
	constructMsbldMailBox(msbldF1, 1, msbldFx_1Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, msbldFx_1Tnrso, msbldFx_1Tnrwo, swmeFrame.out.conf_map[1]);
	constructMsbldMailBox(msbldF0, 0, msbldFx_1Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, msbldFx_1Tnrso, msbldFx_1Tnrwo, swmeFrame.out.conf_map[1]);

	constructAfbldMailBox(afbldF6, 6, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, afbldFx_Tnrso, swmeFrame.out.conf_map[2]);
	constructAfbldMailBox(afbldF5, 5, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, afbldFx_Tnrso, swmeFrame.out.conf_map[2]);
	constructAfbldMailBox(afbldF4, 4, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, afbldFx_Tnrso, swmeFrame.out.conf_map[2]);
	constructAfbldMailBox(afbldF3, 3, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, afbldFx_Tnrso, swmeFrame.out.conf_map[2]);
	constructAfbldMailBox(afbldF2, 2, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, afbldFx_Tnrso, swmeFrame.out.conf_map[2]);
	constructAfbldMailBox(afbldF1, 1, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, afbldFx_Tnrso, swmeFrame.out.conf_map[2]);
	constructAfbldMailBox(afbldF0, 0, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, afbldFx_Tnrso, swmeFrame.out.conf_map[2]);

	//tnrciPool_.fetch(msbldFx_0Tnrci[0]);
	//tnrciPool_.fetch(msbldFx_1Tnrci[0]);
	//tnrciPool_.fetch(afbldFx_Tnrci[0]);
	//testImage(msbldFx_0Tnrci[0]);
	//testImage(msbldFx_1Tnrci[0]);
	//testImage(afbldFx_Tnrci[0]);

	y8_1_32_pool_.fetch(msbldFx_Tnrwi[5]);
	y8_1_16_pool_.fetch(msbldFx_Tnrwi[4]);
	y8_1_8_pool_.fetch(msbldFx_Tnrwi[3]);
	y8_1_4_pool_.fetch(msbldFx_Tnrwi[2]);
	y8_1_2_pool_.fetch(msbldFx_Tnrwi[1]);
	y8_1_1_pool_.fetch(msbldFx_Tnrwi[0]);

	msbldF6.in.tnrsi.push_back(mfnr.msbld_tnrso); // 4BYTE:40x1
	afbldF0.tncso = bfbldFrames.out.p2stto[0];
	zeroImage(msbldFx_Tnrwi[5]);
	zeroImage(msbldFx_Tnrwi[4]);
	zeroImage(msbldFx_Tnrwi[3]);
	zeroImage(msbldFx_Tnrwi[2]);
	zeroImage(msbldFx_Tnrwi[1]);
	zeroImage(msbldFx_Tnrwi[0]);

	msbldF5.in.tnrwi.push_back(msbldFx_Tnrwi[5]);
	msbldF4.in.tnrwi.push_back(msbldFx_Tnrwi[4]);
	msbldF3.in.tnrwi.push_back(msbldFx_Tnrwi[3]);
	msbldF2.in.tnrwi.push_back(msbldFx_Tnrwi[2]);
	msbldF1.in.tnrwi.push_back(msbldFx_Tnrwi[1]);
	msbldF0.in.tnrwi.push_back(msbldFx_Tnrwi[0]);

	//MSBLD_F6(0)
	msbldF6.in.vipi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	msbldF6.in.imgi.push_back(dsFrames.out.ltyuv4o[2]); //MTK_YUV_P012:52x40

	//MSBLD_F5(0)
	msbldF5.in.vipi.push_back(dsFrames.out.ltyuv3o[1]); //MTK_YUV_P012:102x78
	msbldF5.in.imgi.push_back(dsFrames.out.ltyuv3o[2]); //MTK_YUV_P012:102x78
	msbldF5.in.tnrsi.push_back(mfnr.msbld_tnrso); //4BYTE:40x1
	msbldF5.in.rec_dsi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	msbldF5.in.tnrvbi.push_back(dsVbiFramesV5.out.tyuv2o[0]); //Y8:102x78
	msbldF5.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40

	//MSBLD_F4(0)
	msbldF4.in.vipi.push_back(dsFrames.out.ltyuv2o[1]); //MTK_YUV_P012:204x154
	msbldF4.in.imgi.push_back(dsFrames.out.ltyuv2o[2]); //MTK_YUV_P012:204x154
	msbldF4.in.tnrsi.push_back(mfnr.msbld_tnrso); //4BYTE:40x1
	msbldF4.in.rec_dsi.push_back(msbldF5.out.img4o[0]); //MTK_YUV_P012:102x78
	msbldF4.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv4o[0]); //Y8:204x154
	msbldF4.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF4.in.tnrmi.push_back(msbldF5.out.tnrmo[0]); //Y8:102x78

	//MSBLD_F3(0)
	msbldF3.in.vipi.push_back(dsFrames.out.ltyuv4o[0]); //MTK_YUV_P012:408x306
	msbldF3.in.imgi.push_back(mcdsF1Frames.out.ltyuv4o[0]); //MTK_YUV_P012:408x306
	msbldF3.in.tnrsi.push_back(mfnr.msbld_tnrso); //4BYTE:40x1
	msbldF3.in.rec_dsi.push_back(msbldF4.out.img4o[0]); //MTK_YUV_P012:204x154
	msbldF3.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv3o[0]); //Y8:408x306
	msbldF3.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF3.in.tnrmi.push_back(msbldF4.out.tnrmo[0]); //Y8:204x154

	//MSBLD_F2(0)
	msbldF2.in.vipi.push_back(dsFrames.out.ltyuv3o[0]); //MTK_YUV_P012:816x612
	msbldF2.in.imgi.push_back(mcdsF1Frames.out.ltyuv3o[0]); //MTK_YUV_P012:816x612
	msbldF2.in.tnrsi.push_back(mfnr.msbld_tnrso); //4BYTE:40x1
	msbldF2.in.rec_dsi.push_back(msbldF3.out.img4o[0]); //MTK_YUV_P012:408x306
	msbldF2.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv2o[0]); //Y8:816x612
	msbldF2.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF2.in.tnrmi.push_back(msbldF3.out.tnrmo[0]); //Y8:408x306

	//MSBLD_F1(0)
	msbldF1.in.vipi.push_back(dsFrames.out.ltyuv2o[0]); //MTK_YUV_P012:1632x1224
	msbldF1.in.imgi.push_back(mcdsF1Frames.out.ltyuv2o[0]); //MTK_YUV_P012:1632x1224
	msbldF1.in.tnrsi.push_back(mfnr.msbld_tnrso); //4BYTE:40x1
	msbldF1.in.rec_dsi.push_back(msbldF2.out.img4o[0]); //MTK_YUV_P012:816x612
	msbldF1.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[0]); //Y8:1632x1224
	msbldF1.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF1.in.tnrmi.push_back(msbldF2.out.tnrmo[0]); //Y8:816x612

	//MSBLD_F0(0)
	msbldF0.in.vipi.push_back(bfbldFrames.out.img3o[0]); //MTK_YUV_P010:3264x2448
	msbldF0.in.imgi.push_back(mcdsF1Frames.out.wpe_wpeo[0]); //MTK_YUV_P010:3264x2448
	msbldF0.in.tnrsi.push_back(mfnr.msbld_tnrso); //4BYTE:40x1
	msbldF0.in.rec_dsi.push_back(msbldF1.out.img4o[0]); //MTK_YUV_P012:1632x1224
	msbldF0.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[0]); //Y8:1632x1224
	msbldF0.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF0.in.tnrmi.push_back(msbldF1.out.tnrmo[0]); //Y8:1632x1224

	//MSBLD_F6(1)
	msbldF6.in.vipi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	msbldF6.in.imgi.push_back(dsFrames.out.ltyuv4o[3]); //MTK_YUV_P012:52x40
	msbldF6.in.tnrsi.push_back(msbldF6.out.tnrso[0]); //4BYTE:40x1

	//MSBLD_F5(1)
	msbldF5.in.vipi.push_back(msbldF5.out.img4o[0]); //MTK_YUV_P012:102x78
	msbldF5.in.imgi.push_back(dsFrames.out.ltyuv3o[3]); //MTK_YUV_P012:102x78
	msbldF5.in.tnrsi.push_back(msbldF5.out.tnrso[0]); //4BYTE:40x1
	msbldF5.in.rec_dsi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	msbldF5.in.tnrwi.push_back(msbldF5.out.tnrwo[0]); //Y8:102x78
	msbldF5.in.tnrvbi.push_back(dsVbiFramesV5.out.tyuv2o[1]); //Y8:102x78
	msbldF5.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40

	//MSBLD_F4(1)
	msbldF4.in.vipi.push_back(msbldF4.out.img4o[0]); //MTK_YUV_P012:204x154
	msbldF4.in.imgi.push_back(dsFrames.out.ltyuv2o[3]); //MTK_YUV_P012:204x154
	msbldF4.in.tnrsi.push_back(msbldF4.out.tnrso[0]); //4BYTE:40x1
	msbldF4.in.rec_dsi.push_back(msbldF5.out.img4o[1]); //MTK_YUV_P012:102x78
	msbldF4.in.tnrwi.push_back(msbldF4.out.tnrwo[0]); //Y8:204x154
	msbldF4.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv4o[1]); //Y8:204x154
	msbldF4.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF4.in.tnrmi.push_back(msbldF5.out.tnrmo[1]); //Y8:102x78

	//MSBLD_F3(1)
	msbldF3.in.vipi.push_back(msbldF3.out.img4o[0]); //MTK_YUV_P012:408x306
	msbldF3.in.imgi.push_back(mcdsF1Frames.out.ltyuv4o[1]); //MTK_YUV_P012:408x306
	msbldF3.in.tnrsi.push_back(msbldF3.out.tnrso[0]); //4BYTE:40x1
	msbldF3.in.rec_dsi.push_back(msbldF4.out.img4o[1]); //MTK_YUV_P012:204x154
	msbldF3.in.tnrwi.push_back(msbldF3.out.tnrwo[0]); //Y8:408x306
	msbldF3.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv3o[1]); //Y8:408x306
	msbldF3.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF3.in.tnrmi.push_back(msbldF4.out.tnrmo[1]); //Y8:204x154

	//MSBLD_F2(1)
	msbldF2.in.vipi.push_back(msbldF2.out.img4o[0]); //MTK_YUV_P012:816x612
	msbldF2.in.imgi.push_back(mcdsF1Frames.out.ltyuv3o[1]); //MTK_YUV_P012:816x612
	msbldF2.in.tnrsi.push_back(msbldF2.out.tnrso[0]); //4BYTE:40x1
	msbldF2.in.rec_dsi.push_back(msbldF3.out.img4o[1]); //MTK_YUV_P012:408x306
	msbldF2.in.tnrwi.push_back(msbldF2.out.tnrwo[0]); //Y8:816x612
	msbldF2.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv2o[1]); //Y8:816x612
	msbldF2.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF2.in.tnrmi.push_back(msbldF3.out.tnrmo[1]); //Y8:408x306

	//MSBLD_F1(1)
	msbldF1.in.vipi.push_back(msbldF1.out.img4o[0]); //MTK_YUV_P012:1632x1224
	msbldF1.in.imgi.push_back(mcdsF1Frames.out.ltyuv2o[1]); //MTK_YUV_P012:1632x1224
	msbldF1.in.tnrsi.push_back(msbldF1.out.tnrso[0]); //4BYTE:40x1
	msbldF1.in.rec_dsi.push_back(msbldF2.out.img4o[1]); //MTK_YUV_P012:816x612
	msbldF1.in.tnrwi.push_back(msbldF1.out.tnrwo[0]); //Y8:1632x1224
	msbldF1.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[1]); //Y8:1632x1224
	msbldF1.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF1.in.tnrmi.push_back(msbldF2.out.tnrmo[1]); //Y8:816x612

	//MSBLD_F0(1)
	msbldF0.in.vipi.push_back(msbldF0.out.img4o[0]); //MTK_YUV_P010:3264x2448
	msbldF0.in.imgi.push_back(mcdsF1Frames.out.wpe_wpeo[1]); //MTK_YUV_P010:3264x2448
	msbldF0.in.tnrsi.push_back(msbldF0.out.tnrso[0]); //4BYTE:40x1
	msbldF0.in.rec_dsi.push_back(msbldF1.out.img4o[1]); //MTK_YUV_P012:1632x1224
	msbldF0.in.tnrwi.push_back(msbldF0.out.tnrwo[0]); //Y8:3264x2448
	msbldF0.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[1]); //Y8:1632x1224
	msbldF0.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF0.in.tnrmi.push_back(msbldF1.out.tnrmo[1]); //Y8:1632x1224

	//AFBLD_F6(0)
	afbldF6.in.vipi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	afbldF6.in.imgi.push_back(dsFrames.out.ltyuv4o[4]); //MTK_YUV_P012:52x40
	afbldF6.in.tnrsi.push_back(msbldF6.out.tnrso[1]); //4BYTE:40x1

	//AFBLD_F5(0)
	afbldF5.in.vipi.push_back(msbldF5.out.img4o[1]); //MTK_YUV_P012:102x78
	afbldF5.in.imgi.push_back(dsFrames.out.ltyuv3o[4]); //MTK_YUV_P012:102x78
	afbldF5.in.tnrsi.push_back(msbldF5.out.tnrso[1]); //4BYTE:40x1
	afbldF5.in.rec_dsi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	afbldF5.in.tnrwi.push_back(msbldF5.out.tnrwo[1]); //Y8:102x78
	afbldF5.in.tnrvbi.push_back(dsVbiFramesV5.out.tyuv2o[2]); //Y8:102x78
	afbldF5.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40

	//AFBLD_F4(0)
	afbldF4.in.vipi.push_back(msbldF4.out.img4o[1]); //MTK_YUV_P012:204x154
	afbldF4.in.imgi.push_back(dsFrames.out.ltyuv2o[4]); //MTK_YUV_P012:204x154
	afbldF4.in.tnrsi.push_back(msbldF4.out.tnrso[1]); //4BYTE:40x1
	afbldF4.in.rec_dsi.push_back(afbldF5.out.img3o[0]); //MTK_YUV_P012:102x78
	afbldF4.in.tnrwi.push_back(msbldF4.out.tnrwo[1]); //Y8:204x154
	afbldF4.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv4o[2]); //Y8:204x154
	afbldF4.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF4.in.tnrmi.push_back(afbldF5.out.tnrmo[0]); //Y8:102x78

	//AFBLD_F3(0)
	afbldF3.in.vipi.push_back(msbldF3.out.img4o[1]); //MTK_YUV_P012:408x306
	afbldF3.in.imgi.push_back(mcdsF1Frames.out.ltyuv4o[2]); //MTK_YUV_P012:408x306
	afbldF3.in.tnrsi.push_back(msbldF3.out.tnrso[1]); //4BYTE:40x1
	afbldF3.in.rec_dsi.push_back(afbldF4.out.img3o[0]); //MTK_YUV_P012:204x154
	afbldF3.in.tnrwi.push_back(msbldF3.out.tnrwo[1]); //Y8:408x306
	afbldF3.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv3o[2]); //Y8:408x306
	afbldF3.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF3.in.tnrmi.push_back(afbldF4.out.tnrmo[0]); //Y8:204x154

	//AFBLD_F2(0)
	afbldF2.in.vipi.push_back(msbldF2.out.img4o[1]); //MTK_YUV_P012:816x612
	afbldF2.in.imgi.push_back(mcdsF1Frames.out.ltyuv3o[2]); //MTK_YUV_P012:816x612
	afbldF2.in.tnrsi.push_back(msbldF2.out.tnrso[1]); //4BYTE:40x1
	afbldF2.in.rec_dsi.push_back(afbldF3.out.img3o[0]); //MTK_YUV_P012:408x306
	afbldF2.in.tnrwi.push_back(msbldF2.out.tnrwo[1]); //Y8:816x612
	afbldF2.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv2o[2]); //Y8:816x612
	afbldF2.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF2.in.tnrmi.push_back(afbldF3.out.tnrmo[0]); //Y8:408x306

	//AFBLD_F1(0)
	afbldF1.in.vipi.push_back(msbldF1.out.img4o[1]); //MTK_YUV_P012:1632x1224
	afbldF1.in.imgi.push_back(mcdsF1Frames.out.ltyuv2o[2]); //MTK_YUV_P012:1632x1224
	afbldF1.in.tnrsi.push_back(msbldF1.out.tnrso[1]); //4BYTE:40x1
	afbldF1.in.rec_dsi.push_back(afbldF2.out.img3o[0]); //MTK_YUV_P012:816x612
	afbldF1.in.tnrwi.push_back(msbldF1.out.tnrwo[1]); //Y8:1632x1224
	afbldF1.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[2]); //Y8:1632x1224
	afbldF1.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF1.in.tnrmi.push_back(afbldF2.out.tnrmo[0]); //Y8:816x612

	//AFBLD_F0(0)
	afbldF0.in.vipi.push_back(msbldF0.out.img4o[1]); //MTK_YUV_P010:3264x2448
	afbldF0.in.imgi.push_back(mcdsF1Frames.out.wpe_wpeo[2]); //MTK_YUV_P010:3264x2448
	afbldF0.in.tnrsi.push_back(msbldF1.out.tnrso[1]); //4BYTE:40x1
	afbldF0.in.rec_dsi.push_back(afbldF1.out.img3o[0]); //MTK_YUV_P012:1632x1224
	afbldF0.in.tnrwi.push_back(msbldF0.out.tnrwo[1]); //Y8:3264x2448
	afbldF0.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[2]); //Y8:1632x1224
	afbldF0.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF0.in.tnrmi.push_back(afbldF1.out.tnrmo[0]); //Y8:1632x1224
}

std::tuple<BssTask *, BfbldTask *, BfmeTask *, SwmeTask *, McdsF1Task *, DsTask *, DsVbiTask *, MsbldTask *, AfbldTask *>
MfnrTasksManager::makeMfnrTasks(MFNRFrames &mfnr, Scheduler *scheduler,
				const std::string &id, Request *request,
				uint32_t internalRequestId, ImgSysDevice *imgSys)
{
	BssTask *bssTask = new BssTask(scheduler, id + " (BSS)", request, internalRequestId, imgSys, mfnr, this);
	BfbldTask *bfbldTask = new BfbldTask(scheduler, id + " (BFBLD)", request, internalRequestId, imgSys, mfnr, this);
	BfmeTask *bfmeTask = new BfmeTask(scheduler, id + " (BFME)", request, internalRequestId, imgSys, mfnr, this);
	SwmeTask *swmeTask = new SwmeTask(scheduler, id + " (SWME)", request, internalRequestId, imgSys, mfnr, this);
	McdsF1Task *mcdsF1Task = new McdsF1Task(scheduler, id + " (MCDSF1)", request, internalRequestId, imgSys, mfnr, this);
	DsTask *dsTask = new DsTask(scheduler, id + " (DS)", request, internalRequestId, imgSys, mfnr, this);
	DsVbiTask *dsVbiTask = new DsVbiTask(scheduler, id + " (DSVBI)", request, internalRequestId, imgSys, mfnr, this);
	MsbldTask *msbldTask = new MsbldTask(scheduler, id + " (MSBLD)", request, internalRequestId, imgSys, mfnr, this);
	AfbldTask *afbldTask = new AfbldTask(scheduler, id + " (AFBLD)", request, internalRequestId, imgSys, mfnr, this);

	return std::make_tuple(bssTask, bfbldTask, bfmeTask, swmeTask, mcdsF1Task, dsTask, dsVbiTask, msbldTask, afbldTask);
}

BssTask::BssTask(Scheduler *scheduler, const std::string &id, [[maybe_unused]] Request *request, uint32_t internalRequestId,
		 ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	frames_ = mfnr.bssFrames;
	bssWrapper_ = manager->bssWrapper_;
	mfnr_ = mfnr;
}

void BssTask::allocateOutputBuffers()
{
	[[maybe_unused]] auto &out = frames_.out;
	for (auto i = 0; i < kInputRawCount; i++) {
	}
}

void BssTask::run()
{
	allocateOutputBuffers();

	[[maybe_unused]] auto &mfnrSizes_ = manager_->mfnrSizes_;
	[[maybe_unused]] auto &in = frames_.in;
	[[maybe_unused]] auto &out = frames_.out;
	MappedFrameBuffer mappedBssParamBuffers =
		MappedFrameBuffer(in.bssParamInfo->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
	DmaSyncer syncer_bssParam(in.bssParamInfo->get().buffer()->planes()[0].fd.get());

	MappedFrameBuffer mappedBssDataGBuffers =
		MappedFrameBuffer(in.bssDataGInfo->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
	DmaSyncer syncer_dataG(in.bssDataGInfo->get().buffer()->planes()[0].fd.get());

	std::vector<MappedFrameBuffer> mappedBssFdMain;
	std::vector<MappedFrameBuffer> mappedBssFd;
	std::vector<MappedFrameBuffer> mappedFace;
	std::vector<MappedFrameBuffer> mappedPos;

	MappedFrameBuffer mappedBssTuningBuffers =
		MappedFrameBuffer(in.bssTuningInfo->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
	DmaSyncer syncer_bssTuningInfo(in.bssTuningInfo->get().buffer()->planes()[0].fd.get());

	MappedFrameBuffer mappedBssVerInfoBuffers =
		MappedFrameBuffer(in.bssVerInfo->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
	DmaSyncer syncer_bssVerInfo(in.bssVerInfo->get().buffer()->planes()[0].fd.get());

	MappedFrameBuffer mappedBssOutBuffers =
		MappedFrameBuffer(out.bssOutDataInfo->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
	DmaSyncer syncer_bssOut(out.bssOutDataInfo->get().buffer()->planes()[0].fd.get());

	bssWrapper_->doBss(kInputRawCount, frames_);

	for (int i = 0; i < kInputRawCount; i++) {
		mappedBssFdMain.push_back(MappedFrameBuffer(in.bssFdMainInfo[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite));
		DmaSyncer syncer_bssFdMainInfo(in.bssFdMainInfo[i]->get().buffer()->planes()[0].fd.get());
		mappedBssFd.push_back(MappedFrameBuffer(in.bssFdInfo[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite));
		DmaSyncer syncer_bssFdInfo(in.bssFdInfo[i]->get().buffer()->planes()[0].fd.get());
		mappedFace.push_back(MappedFrameBuffer(in.bssFaceInfo[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite));
		DmaSyncer syncer_bssFaceInfo(in.bssFaceInfo[i]->get().buffer()->planes()[0].fd.get());
		mappedPos.push_back(MappedFrameBuffer(in.bssPosInfo[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite));
		DmaSyncer syncer_bssPosInfo(in.bssPosInfo[i]->get().buffer()->planes()[0].fd.get());
	}

	memcpy(reinterpret_cast<void *>(in.bssTuningInfo->get().address(0)), in.db_param->get().get(), sizeof(mtk::isphal::v1::isp_bss_Param));

	manager_->onDeviceTuner_->tuneBss(internalRequestId_, frames_, kInputRawCount);
	Task::notifyDone();
}

BfbldTask::BfbldTask(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
		     ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	frames_ = mfnr.bfbldFrames;
	mfnr_ = mfnr;
}

void BfbldTask::allocateOutputBuffers()
{
	auto &out = frames_.out;
	for (auto i = 0; i < kInputRawCount; i++) {
		manager_->p2sttoPool_.fetch(out.p2stto[i]);
		manager_->yuvp010_1_4_pool_aligned16_.fetch(out.img2o[i]);
		manager_->yuvp010_1_1_pool_.fetch(out.img3o[i]);
	}
}

void BfbldTask::notifyDone()
{
	auto bssOrder = mfnr_.bss_order->get();
	manager_->onDeviceTuner_->tuneBfbld(internalRequestId_, frames_, bssOrder);
	//BFBLD Precheck
	if (MfnrTasksManager::mfnrPrecheck()) {
		for (auto i = 0; i < kInputRawCount; i++) {
			if (i == 0) {
				LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_BFBLD_BASE:1";
			} else {
				LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_BFBLD_REF:1";
			}
		}
	}
	Task::notifyDone();
}

void BfbldTask::run()
{
	allocateOutputBuffers();
	auto &mfnrSizes_ = manager_->mfnrSizes_;

	MUINT32 timestampMili = request_->metadata().get(controls::SensorTimestamp).value_or(0);
	auto &in = frames_.in;
	auto &out = frames_.out;
	auto &capturedRaws = frames_.capturedRaws;
	auto bssOrder = mfnr_.bss_order->get();
	for (auto i = 0; i < (int)bssOrder.size(); i++) {
		in.timgi[i] = capturedRaws[bssOrder[i]];
	}
	SingleDeviceRequest sdRequest;
	sdRequest.init(internalRequestId_, timestampMili, "BfbldTask");
	for (auto i = 0; i < kInputRawCount; i++) {
		if (i == 0) {
			StageEx &BFBLD_BASE = sdRequest.emplaceStage(PEU_Stage::BFBLD_BASE, internalRequestId_ + bssOrder[i]);
			BFBLD_BASE.input(in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
			BFBLD_BASE.input(in.timgi[0]->get(), IMG_PORT_TIMGI, 0, Size{ 0, 0 });
			BFBLD_BASE.output(out.img3o[0]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[0]);
			BFBLD_BASE.output(out.img2o[0]->get(), IMG_PORT_IMG2O, 0, mfnrSizes_[0]);
			BFBLD_BASE.output(out.p2stto[0]->get(), IMG_PORT_IMGSTATO, 0, mfnrSizes_[0]);
		} else {
			StageEx &BFBLD_REF = sdRequest.emplaceStage(PEU_Stage::BFBLD_REF, internalRequestId_ + bssOrder[i]);
			BFBLD_REF.input(in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
			BFBLD_REF.input(in.timgi[i]->get(), IMG_PORT_TIMGI, 0, Size{ 0, 0 });
			BFBLD_REF.output(out.img3o[i]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[0]);
			BFBLD_REF.output(out.img2o[i]->get(), IMG_PORT_IMG2O, 0, mfnrSizes_[0]);
			BFBLD_REF.output(out.p2stto[i]->get(), IMG_PORT_IMGSTATO, 0, mfnrSizes_[0]);
		}
	}
	requestHelper_.queueRequest(UserIdMfnr, sdRequest);
}

McdsF1Task::McdsF1Task(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
		       ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	frames_ = mfnr.mcdsF1Frames;
	mfnr_ = mfnr;
}

void McdsF1Task::allocateOutputBuffers()
{
	auto &out = frames_.out;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		manager_->yuvp010_1_1_pool_.fetch(out.wpe_wpeo[i]);
		manager_->yuvp012_1_2_pool_.fetch(out.ltyuv2o[i]);
		manager_->yuvp012_1_4_pool_.fetch(out.ltyuv3o[i]);
		manager_->yuvp012_1_8_pool_.fetch(out.ltyuv4o[i]);
		manager_->y8_1_2_pool_.fetch(out.ltyuv5o[i]);
	}
}

void McdsF1Task::notifyDone()
{
	auto bssOrder = mfnr_.bss_order->get();
	manager_->onDeviceTuner_->tuneMcdsF1(internalRequestId_, frames_, bssOrder);
	//MCDS_F1 Precheck
	if (MfnrTasksManager::mfnrPrecheck()) {
		for (auto i = 0; i < kInputRawCount; i++) {
			LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MCDS_F1:1";
		}
	}
	Task::notifyDone();
}

void McdsF1Task::run()
{
	allocateOutputBuffers();

	auto &mfnrSizes_ = manager_->mfnrSizes_;
	auto bssOrder = mfnr_.bss_order->get();

	MUINT32 timestampMili = request_->metadata().get(controls::SensorTimestamp).value_or(0);
	SingleDeviceRequest sdRequest;

	sdRequest.init(internalRequestId_, timestampMili, "McdsF1Task");

	auto &in = frames_.in;
	auto &out = frames_.out;

	for (auto i = 0; i < kInputRawCount - 1; i++) {
		StageEx &MCDS_F1 = sdRequest.emplaceStage(PEU_Stage::MCDS_F1, internalRequestId_ + bssOrder[i + 1]);
		MCDS_F1.input(in.wpe_wpei[i]->get(), IMG_PORT_WPE_WPEI, 0, Size{ 0, 0 });
		MCDS_F1.input(in.wpe_veci[i]->get(), IMG_PORT_WPE_VECI, 0, Size{ 0, 0 });
		MCDS_F1.input(in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

		MCDS_F1.output(out.wpe_wpeo[i]->get(), IMG_PORT_WPE_WPEO, 0, Size{ 0, 0 });
		MCDS_F1.output(out.ltyuv2o[i]->get(), IMG_PORT_LTYUV2O, 2, Size{ 0, 0 });
		MCDS_F1.output(out.ltyuv3o[i]->get(), IMG_PORT_LTYUV3O, 2, Size{ 0, 0 });
		MCDS_F1.output(out.ltyuv4o[i]->get(), IMG_PORT_LTYUV4O, 2, Size{ 0, 0 });
		MCDS_F1.output(out.ltyuv5o[i]->get(), IMG_PORT_LTYUV5O, 0, Size{ 0, 0 });
		MCDS_F1.setMcdsF1WpeInfo(IMG_EXTRA_PARAM_ID_WPE_INFO, mfnrSizes_[0], EWPE_HW_LITE);
	}
	requestHelper_.queueRequest(UserIdMfnr, sdRequest);
}

BfmeTask::BfmeTask(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
		   ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	frames_ = mfnr.bfmeFrames;
	mfnr_ = mfnr;
}

void BfmeTask::allocateOutputBuffers()
{
	auto &out = frames_.out;

	for (auto i = 0; i < kInputRawCount; i++) {
		manager_->y8_1_4_pool_aligned16_.fetch(out.img2o[i]);
	}
}

void BfmeTask::notifyDone()
{
	auto bssOrder = mfnr_.bss_order->get();
	manager_->onDeviceTuner_->tuneBfme(internalRequestId_, frames_, bssOrder);
	//BFME Precheck
	if (MfnrTasksManager::mfnrPrecheck()) {
		for (auto i = 0; i < kInputRawCount; i++) {
			LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_BFME:1";
		}
	}
	Task::notifyDone();
}

void BfmeTask::run()
{
	allocateOutputBuffers();

	auto &mfnrSizes_aligned16 = manager_->mfnrSizes_aligned16_;

	MUINT32 timestampMili = request_->metadata().get(controls::SensorTimestamp).value_or(0);
	SingleDeviceRequest sdRequest;

	sdRequest.init(internalRequestId_, timestampMili, "BfmeTask");

	auto &in = frames_.in;
	auto &out = frames_.out;
	auto bssOrder = mfnr_.bss_order->get();
	for (auto i = 0; i < kInputRawCount; i++) {
		StageEx &BFME = sdRequest.emplaceStage(PEU_Stage::BFME, internalRequestId_ + bssOrder[i]);
		BFME.input(in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		BFME.input(in.tunbufi[1]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
		BFME.output(out.img2o[i]->get(), IMG_PORT_IMG2O, 0, mfnrSizes_aligned16[2]);
		BFME.setMultiScale(IMG_MULTI_SCALE_DOWN4, 1, 0);
	}
	requestHelper_.queueRequest(UserIdMfnr, sdRequest);
}

SwmeTask::SwmeTask(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
		   ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	frames_ = mfnr.swmeFrames;
	swmeWrapper_ = manager->swmeWrapper_;
	mfnr_ = mfnr;
}

void SwmeTask::allocateOutputBuffers()
{
	auto &in = frames_.in;
	auto &out = frames_.out;

	for (auto i = 0; i < kInputRawCount - 1; i++) {
		manager_->tnrciPool_.fetch(out.conf_map[i]);
		manager_->wrap2pPool_.fetch(out.warpping_map[i]);
		manager_->fourBytes_1_16_pool_.fetch(out.mcmv[i]);
		manager_->swmeOutPool_.fetch(out.paramOutInfo[i]);

		manager_->swmeParamPool_.fetch(in.paramInInfo[i]);
		manager_->swmeTuningPool_.fetch(in.tuningInfo[i]);
	}
}

void SwmeTask::run()
{
	allocateOutputBuffers();

	auto &mfnrSizes_ = manager_->mfnrSizes_;
	auto &mfnrSizes_aligned16_ = manager_->mfnrSizes_aligned16_;
	auto &in = frames_.in;
	auto &out = frames_.out;
	auto bssOrder = mfnr_.bss_order->get();
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		std::shared_ptr<SwmeWrapper> swmewrapper = swmeWrapper_[i];
		IMFBLL_SET_PROC_INFO_STRUCT_IPC paramIn;
		MappedFrameBuffer mappedWarppingMapBuffers =
			MappedFrameBuffer(out.warpping_map[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
		DmaSyncer syncer_WarppingMapX(out.warpping_map[i]->get().buffer()->planes()[0].fd.get());
		DmaSyncer syncer_WarppingMapY(out.warpping_map[i]->get().buffer()->planes()[1].fd.get());

		MappedFrameBuffer mappedConfMapBuffers =
			MappedFrameBuffer(out.conf_map[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
		DmaSyncer syncer_ConfMap(out.conf_map[i]->get().buffer()->planes()[0].fd.get());

		MappedFrameBuffer mappedMcmvMapBuffers =
			MappedFrameBuffer(out.mcmv[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
		DmaSyncer syncer_McmvMap(out.mcmv[i]->get().buffer()->planes()[0].fd.get());

		SwmeWrapper::prepareParam(
			paramIn,
			in.workbuf[i],
			in.base_buf[i],
			in.ref_buf[i],
			in.bss_buf[i],
			out.warpping_map[i],
			in.db_param[i]->get(),
			mfnrSizes_[0],
			mfnrSizes_aligned16_[2],
			i);
		swmewrapper->featureCtrl(IMFBLL_FTCTRL_SET_PROC_INFO, &paramIn, NULL);
		IMFBLL_PROC1_OUT_STRUCT_IPC paramOut;
		SwmeWrapper::prepareOutParam(
			&paramOut,
			out.conf_map[i],
			out.warpping_map[i],
			out.mcmv[i]);

		MRESULT ErrCode = swmewrapper->swmeMain(IMFBLL_PROC1, NULL, &paramOut);
		if (ErrCode)
			LOG(MtkISP7, Error) << "Some error with in swmeMain, ErrCode = " << ErrCode;

		MappedFrameBuffer mappedSwmeParamInBuffer =
			MappedFrameBuffer(in.paramInInfo[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
		DmaSyncer syncer_swmeParamIn(in.paramInInfo[i]->get().buffer()->planes()[0].fd.get());

		memcpy(reinterpret_cast<void *>(in.paramInInfo[i]->get().address(0)),
		       reinterpret_cast<void *>(&paramIn),
		       sizeof(IMFBLL_SET_PROC_INFO_STRUCT));

		MappedFrameBuffer mappedSwmeTuningBuffer =
			MappedFrameBuffer(in.tuningInfo[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
		DmaSyncer syncer_swmeTuning(in.tuningInfo[i]->get().buffer()->planes()[0].fd.get());

		memcpy(reinterpret_cast<void *>(in.tuningInfo[i]->get().address(0)),
		       reinterpret_cast<void *>(in.db_param[i]->get().get()),
		       sizeof(mtk::isphal::v1::isp_swme_Param));

		MappedFrameBuffer mappedSwmeParamOutBuffer =
			MappedFrameBuffer(out.paramOutInfo[i]->get().buffer(), MappedFrameBuffer::MapFlag::ReadWrite);
		DmaSyncer syncer_swmeParamOut(out.paramOutInfo[i]->get().buffer()->planes()[0].fd.get());

		memcpy(reinterpret_cast<void *>(out.paramOutInfo[i]->get().address(0)),
		       reinterpret_cast<void *>(&paramOut),
		       sizeof(IMFBLL_PROC1_OUT_STRUCT));

		//SWME Precheck
		if (MfnrTasksManager::mfnrPrecheck()) {
			bool hasVal = false;
			MINT32 *px = static_cast<MINT32 *>(reinterpret_cast<void *>(out.warpping_map[i]->get().address(0)));
			MINT32 *py = static_cast<MINT32 *>(reinterpret_cast<void *>(out.warpping_map[i]->get().address(1)));
			Size warppingMapSize = out.warpping_map[i]->get().size();
			int stride = out.warpping_map[i]->get().buffer()->planes()[0].stride;
			for (int h = 0; h < (int)warppingMapSize.height && !hasVal; h++) {
				for (int w = 0; w < (int)warppingMapSize.width; w++) {
					MUINT8 *x = reinterpret_cast<MUINT8 *>(px + w + h * stride);
					MUINT8 *y = reinterpret_cast<MUINT8 *>(py + w + h * stride);
					if (*x != 0 || *y != 0) {
						hasVal = true;
						break;
					}
				}
			}
			LOG(MtkISP7, Info) << "[CAT][MFNR] swme_out:" << hasVal;
		}
	}

	manager_->onDeviceTuner_->tuneSwme(internalRequestId_, frames_, bssOrder);
	Task::notifyDone();
}

DsTask::DsTask(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
	       ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	frames_ = mfnr.dsFrames;
	mfnr_ = mfnr;
}

void DsTask::allocateOutputBuffers()
{
	auto &out = frames_.out;
	manager_->yuvp012_1_2_pool_.fetch(out.ltyuv2o[0]);
	manager_->yuvp012_1_4_pool_.fetch(out.ltyuv3o[0]);
	manager_->yuvp012_1_8_pool_.fetch(out.ltyuv4o[0]);
	for (auto i = 1; i < kInputRawCount + 1; i++) {
		manager_->yuvp012_1_16_pool_.fetch(out.ltyuv2o[i]);
		manager_->yuvp012_1_32_pool_.fetch(out.ltyuv3o[i]);
		manager_->yuvp012_1_64_pool_.fetch(out.ltyuv4o[i]);
	}
}

void DsTask::notifyDone()
{
	auto bssOrder = mfnr_.bss_order->get();
	manager_->onDeviceTuner_->tuneDs(internalRequestId_, frames_, bssOrder);
	//DS Precheck
	if (MfnrTasksManager::mfnrPrecheck()) {
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_DS:1";
	}

	Task::notifyDone();
}

void DsTask::run()
{
	allocateOutputBuffers();

	//auto &mfnrSizes_ = manager_->mfnrSizes_;

	MUINT32 timestampMili = request_->metadata().get(controls::SensorTimestamp).value_or(0);
	SingleDeviceRequest sdRequest;

	sdRequest.init(internalRequestId_, timestampMili, "DS");

	auto &in = frames_.in;
	auto &out = frames_.out;
	auto bssOrder = mfnr_.bss_order->get();
	for (auto i = 0; i < kInputRawCount + 1; i++) {
		if (i == 0 || i == 1) {
			int frameNumber = internalRequestId_ + bssOrder[0];
			StageEx &DS = sdRequest.emplaceStage(PEU_Stage::DS, frameNumber, i);
			DS.input(in.ltimgi[i]->get(), IMG_PORT_LTIMGI, 0, Size{ 0, 0 });
			DS.input(in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
			DS.output(out.ltyuv2o[i]->get(), IMG_PORT_LTYUV2O, 2, Size{ 0, 0 });
			DS.output(out.ltyuv3o[i]->get(), IMG_PORT_LTYUV3O, 2, Size{ 0, 0 });
			DS.output(out.ltyuv4o[i]->get(), IMG_PORT_LTYUV4O, 2, Size{ 0, 0 });
		} else {
			int frameNumber = internalRequestId_ + bssOrder[i - 1];
			StageEx &DS = sdRequest.emplaceStage(PEU_Stage::DS, frameNumber);
			DS.input(in.ltimgi[i]->get(), IMG_PORT_LTIMGI, 0, Size{ 0, 0 });
			DS.input(in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
			DS.output(out.ltyuv2o[i]->get(), IMG_PORT_LTYUV2O, 2, Size{ 0, 0 });
			DS.output(out.ltyuv3o[i]->get(), IMG_PORT_LTYUV3O, 2, Size{ 0, 0 });
			DS.output(out.ltyuv4o[i]->get(), IMG_PORT_LTYUV4O, 2, Size{ 0, 0 });
		}
	}
	requestHelper_.queueRequest(UserIdMfnr, sdRequest);
}

DsVbiTask::DsVbiTask(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
		     ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	dsVbiFramesV2_ = mfnr.dsVbiFramesV2;
	dsVbiFramesV5_ = mfnr.dsVbiFramesV5;
	mfnr_ = mfnr;
}

void DsVbiTask::allocateOutputBuffers()
{
	auto &outV2 = dsVbiFramesV2_.out;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		manager_->y8_1_4_pool_.fetch(outV2.tyuv2o[i]);
		manager_->y8_1_8_pool_.fetch(outV2.tyuv3o[i]);
		manager_->y8_1_16_pool_.fetch(outV2.tyuv4o[i]);
	}
	auto &outV5 = dsVbiFramesV5_.out;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		manager_->y8_1_32_pool_.fetch(outV5.tyuv2o[i]);
	}
}

void DsVbiTask::notifyDone()
{
	auto bssOrder = mfnr_.bss_order->get();
	manager_->onDeviceTuner_->tuneDsVbi(internalRequestId_, dsVbiFramesV2_, dsVbiFramesV5_, bssOrder);
	//DS_VBI Precheck
	if (MfnrTasksManager::mfnrPrecheck()) {
		for (auto i = 0; i < kInputRawCount - 1; i++) {
			LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_DS_VBI_V2:1";
			LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_DS_VBI_V5:1";
		}
	}
	Task::notifyDone();
}

void DsVbiTask::run()
{
	allocateOutputBuffers();

	//auto &mfnrSizes_ = manager_->mfnrSizes_;

	MUINT32 timestampMili = request_->metadata().get(controls::SensorTimestamp).value_or(0);
	SingleDeviceRequest sdRequest;

	sdRequest.init(internalRequestId_, timestampMili, "DS_VBI");

	auto &inV2 = dsVbiFramesV2_.in;
	auto &outV2 = dsVbiFramesV2_.out;
	auto bssOrder = mfnr_.bss_order->get();
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		StageEx &DS_VBI_V2 = sdRequest.emplaceStage(PEU_Stage::DS_VBI_V2, internalRequestId_ + bssOrder[i + 1]);
		DS_VBI_V2.input(inV2.timgi[i]->get(), IMG_PORT_TIMGI, 0, Size{ 0, 0 });
		DS_VBI_V2.input(inV2.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

		DS_VBI_V2.output(outV2.tyuv2o[i]->get(), IMG_PORT_TYUV2O, 2, Size{ 0, 0 });
		DS_VBI_V2.output(outV2.tyuv3o[i]->get(), IMG_PORT_TYUV3O, 2, Size{ 0, 0 });
		DS_VBI_V2.output(outV2.tyuv4o[i]->get(), IMG_PORT_TYUV4O, 2, Size{ 0, 0 });
	}
	auto &inV5 = dsVbiFramesV5_.in;
	auto &outV5 = dsVbiFramesV5_.out;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		StageEx &DS_VBI_V5 = sdRequest.emplaceStage(PEU_Stage::DS_VBI_V5, internalRequestId_ + bssOrder[i + 1]);

		DS_VBI_V5.input(inV5.timgi[i]->get(), IMG_PORT_TIMGI, 0, Size{ 0, 0 });
		DS_VBI_V5.input(inV5.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

		DS_VBI_V5.output(outV5.tyuv2o[i]->get(), IMG_PORT_TYUV2O, 2, Size{ 0, 0 });
	}
	requestHelper_.queueRequest(UserIdMfnr, sdRequest);
}

MsbldTask::MsbldTask(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
		     ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	msbldF6_ = mfnr.msbldF6;
	msbldF5_ = mfnr.msbldF5;
	msbldF4_ = mfnr.msbldF4;
	msbldF3_ = mfnr.msbldF3;
	msbldF2_ = mfnr.msbldF2;
	msbldF1_ = mfnr.msbldF1;
	msbldF0_ = mfnr.msbldF0;
	tnrso_ = mfnr.msbld_tnrso;
	mfnr_ = mfnr;
}

void MsbldTask::allocateOutputBuffers()
{
	for (auto i = 0; i < 2; i++) {
		auto &msbldF6_out = msbldF6_.out;
		manager_->nv21_1_64_pool_.fetch(msbldF6_out.img4o[i]);
		manager_->fourBytes_pool_.fetch(msbldF6_out.tnrso[i]);
		auto &msbldF5_out = msbldF5_.out;
		manager_->yuvp012_1_32_pool_.fetch(msbldF5_out.img4o[i]);
		manager_->y8_1_32_pool_.fetch(msbldF5_out.tnrwo[i]);
		manager_->y8_1_32_pool_.fetch(msbldF5_out.tnrmo[i]);
		manager_->fourBytes_pool_.fetch(msbldF5_out.tnrso[i]);
		auto &msbldF4_out = msbldF4_.out;
		manager_->yuvp012_1_16_pool_.fetch(msbldF4_out.img4o[i]);
		manager_->y8_1_16_pool_.fetch(msbldF4_out.tnrwo[i]);
		manager_->y8_1_16_pool_.fetch(msbldF4_out.tnrmo[i]);
		manager_->fourBytes_pool_.fetch(msbldF4_out.tnrso[i]);
		auto &msbldF3_out = msbldF3_.out;
		manager_->yuvp012_1_8_pool_.fetch(msbldF3_out.img4o[i]);
		manager_->y8_1_8_pool_.fetch(msbldF3_out.tnrwo[i]);
		manager_->y8_1_8_pool_.fetch(msbldF3_out.tnrmo[i]);
		manager_->fourBytes_pool_.fetch(msbldF3_out.tnrso[i]);
		auto &msbldF2_out = msbldF2_.out;
		manager_->yuvp012_1_4_pool_.fetch(msbldF2_out.img4o[i]);
		manager_->y8_1_4_pool_.fetch(msbldF2_out.tnrwo[i]);
		manager_->y8_1_4_pool_.fetch(msbldF2_out.tnrmo[i]);
		manager_->fourBytes_pool_.fetch(msbldF2_out.tnrso[i]);
		auto &msbldF1_out = msbldF1_.out;
		manager_->yuvp012_1_2_pool_.fetch(msbldF1_out.img4o[i]);
		manager_->y8_1_2_pool_.fetch(msbldF1_out.tnrwo[i]);
		manager_->y8_1_2_pool_.fetch(msbldF1_out.tnrmo[i]);
		manager_->fourBytes_pool_.fetch(msbldF1_out.tnrso[i]);
		auto &msbldF0_out = msbldF0_.out;
		manager_->yuvp010_1_1_pool_.fetch(msbldF0_out.img4o[i]);
		manager_->y8_1_1_pool_.fetch(msbldF0_out.tnrwo[i]);
		manager_->fourBytes_pool_.fetch(msbldF0_out.tnrso[i]);
	}
}

void MsbldTask::notifyDone()
{
	auto bssOrder = mfnr_.bss_order->get();
	manager_->onDeviceTuner_->tuneMsbld(
		internalRequestId_, msbldF0_, msbldF1_, msbldF2_,
		msbldF3_, msbldF4_, msbldF5_, msbldF6_, bssOrder);
	//MSBLD Precheck
	if (MfnrTasksManager::mfnrPrecheck()) {
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MSBLD_F0:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MSBLD_F1:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MSBLD_F2:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MSBLD_F3:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MSBLD_F4:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MSBLD_F5:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_MSBLD_F6:1";
	}
	Task::notifyDone();
}

void MsbldTask::run()
{
	allocateOutputBuffers();

	auto &mfnrSizes_ = manager_->mfnrSizes_;

	MUINT32 timestampMili = request_->metadata().get(controls::SensorTimestamp).value_or(0);
	SingleDeviceRequest sdRequest;

	sdRequest.init(internalRequestId_, timestampMili, "MSBLD");

	for (auto i = 0; i < 2; i++) {
		auto bssOrder = mfnr_.bss_order->get();

		int frameNumber = internalRequestId_ + bssOrder[i + 1];
		StageEx &MSBLD_F6 = sdRequest.emplaceStage(PEU_Stage::MSBLD_F6, frameNumber);
		auto &msbldF6_in = msbldF6_.in;
		auto &msbldF6_out = msbldF6_.out;
		MSBLD_F6.input(msbldF6_in.vipi[i]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
		MSBLD_F6.input(msbldF6_in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		MSBLD_F6.input(msbldF6_in.tnrsi[i]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
		MSBLD_F6.input(msbldF6_in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
		MSBLD_F6.output(msbldF6_out.tnrso[i]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
		MSBLD_F6.output(msbldF6_out.img4o[i]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[6]);
		MSBLD_F6.setMultiScale(IMG_MULTI_SCALE_DOWN2, 6, 7);
		MSBLD_F6.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

		StageEx &MSBLD_F5 = sdRequest.emplaceStage(PEU_Stage::MSBLD_F5, frameNumber);
		auto &msbldF5_in = msbldF5_.in;
		auto &msbldF5_out = msbldF5_.out;
		MSBLD_F5.input(msbldF5_in.vipi[i]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
		MSBLD_F5.input(msbldF5_in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		MSBLD_F5.input(msbldF5_in.tnrsi[i]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
		MSBLD_F5.input(msbldF5_in.rec_dsi[i]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[6]);
		MSBLD_F5.input(msbldF5_in.tnrci[i]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
		MSBLD_F5.input(msbldF5_in.tnrwi[i]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[5]);
		MSBLD_F5.input(msbldF5_in.tnrvbi[i]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[5]);
		MSBLD_F5.input(msbldF5_in.tnrlfdi[i]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
		MSBLD_F5.input(msbldF5_in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

		MSBLD_F5.output(msbldF5_out.img4o[i]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[5]);
		MSBLD_F5.output(msbldF5_out.tnrwo[i]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[5]);
		MSBLD_F5.output(msbldF5_out.tnrmo[i]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[5]);
		MSBLD_F5.output(msbldF5_out.tnrso[i]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
		MSBLD_F5.setMultiScale(IMG_MULTI_SCALE_DOWN2, 5, 7);
		MSBLD_F5.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

		StageEx &MSBLD_F4 = sdRequest.emplaceStage(PEU_Stage::MSBLD_F4, frameNumber);
		auto &msbldF4_in = msbldF4_.in;
		auto &msbldF4_out = msbldF4_.out;
		MSBLD_F4.input(msbldF4_in.vipi[i]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
		MSBLD_F4.input(msbldF4_in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		MSBLD_F4.input(msbldF4_in.tnrsi[i]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
		MSBLD_F4.input(msbldF4_in.rec_dsi[i]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[5]);
		MSBLD_F4.input(msbldF4_in.tnrci[i]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
		MSBLD_F4.input(msbldF4_in.tnrwi[i]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[4]);
		MSBLD_F4.input(msbldF4_in.tnrvbi[i]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[4]);
		MSBLD_F4.input(msbldF4_in.tnrlfdi[i]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
		MSBLD_F4.input(msbldF4_in.tnrmi[i]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[5]);
		MSBLD_F4.input(msbldF4_in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

		MSBLD_F4.output(msbldF4_out.img4o[i]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[4]);
		MSBLD_F4.output(msbldF4_out.tnrwo[i]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[4]);
		MSBLD_F4.output(msbldF4_out.tnrmo[i]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[4]);
		MSBLD_F4.output(msbldF4_out.tnrso[i]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
		MSBLD_F4.setMultiScale(IMG_MULTI_SCALE_DOWN2, 4, 7);
		MSBLD_F4.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);
		StageEx &MSBLD_F3 = sdRequest.emplaceStage(PEU_Stage::MSBLD_F3, frameNumber);
		auto &msbldF3_in = msbldF3_.in;
		auto &msbldF3_out = msbldF3_.out;
		MSBLD_F3.input(msbldF3_in.vipi[i]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
		MSBLD_F3.input(msbldF3_in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		MSBLD_F3.input(msbldF3_in.tnrsi[i]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
		MSBLD_F3.input(msbldF3_in.rec_dsi[i]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[4]);
		MSBLD_F3.input(msbldF3_in.tnrci[i]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
		MSBLD_F3.input(msbldF3_in.tnrwi[i]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[3]);
		MSBLD_F3.input(msbldF3_in.tnrvbi[i]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[3]);
		MSBLD_F3.input(msbldF3_in.tnrlfdi[i]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
		MSBLD_F3.input(msbldF3_in.tnrmi[i]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[4]);
		MSBLD_F3.input(msbldF3_in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

		MSBLD_F3.output(msbldF3_out.img4o[i]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[3]);
		MSBLD_F3.output(msbldF3_out.tnrwo[i]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[3]);
		MSBLD_F3.output(msbldF3_out.tnrmo[i]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[3]);
		MSBLD_F3.output(msbldF3_out.tnrso[i]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
		MSBLD_F3.setMultiScale(IMG_MULTI_SCALE_DOWN2, 3, 7);
		MSBLD_F3.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);
		StageEx &MSBLD_F2 = sdRequest.emplaceStage(PEU_Stage::MSBLD_F2, frameNumber);
		auto &msbldF2_in = msbldF2_.in;
		auto &msbldF2_out = msbldF2_.out;

		MSBLD_F2.input(msbldF2_in.vipi[i]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
		MSBLD_F2.input(msbldF2_in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		MSBLD_F2.input(msbldF2_in.tnrsi[i]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
		MSBLD_F2.input(msbldF2_in.rec_dsi[i]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[3]);
		MSBLD_F2.input(msbldF2_in.tnrci[i]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
		MSBLD_F2.input(msbldF2_in.tnrwi[i]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[2]);
		MSBLD_F2.input(msbldF2_in.tnrvbi[i]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[2]);
		MSBLD_F2.input(msbldF2_in.tnrlfdi[i]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
		MSBLD_F2.input(msbldF2_in.tnrmi[i]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[3]);
		MSBLD_F2.input(msbldF2_in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

		MSBLD_F2.output(msbldF2_out.img4o[i]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[2]);
		MSBLD_F2.output(msbldF2_out.tnrwo[i]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[2]);
		MSBLD_F2.output(msbldF2_out.tnrmo[i]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[2]);
		MSBLD_F2.output(msbldF2_out.tnrso[i]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
		MSBLD_F2.setMultiScale(IMG_MULTI_SCALE_DOWN2, 2, 7);
		MSBLD_F2.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

		StageEx &MSBLD_F1 = sdRequest.emplaceStage(PEU_Stage::MSBLD_F1, frameNumber);
		auto &msbldF1_in = msbldF1_.in;
		auto &msbldF1_out = msbldF1_.out;
		MSBLD_F1.input(msbldF1_in.vipi[i]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
		MSBLD_F1.input(msbldF1_in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		MSBLD_F1.input(msbldF1_in.tnrsi[i]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
		MSBLD_F1.input(msbldF1_in.rec_dsi[i]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[2]);
		MSBLD_F1.input(msbldF1_in.tnrci[i]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
		MSBLD_F1.input(msbldF1_in.tnrwi[i]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[1]);
		MSBLD_F1.input(msbldF1_in.tnrvbi[i]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[1]);
		MSBLD_F1.input(msbldF1_in.tnrlfdi[i]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
		MSBLD_F1.input(msbldF1_in.tnrmi[i]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[2]);
		MSBLD_F1.input(msbldF1_in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
		MSBLD_F1.output(msbldF1_out.img4o[i]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[1]);
		MSBLD_F1.output(msbldF1_out.tnrwo[i]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[1]);
		MSBLD_F1.output(msbldF1_out.tnrmo[i]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[1]);
		MSBLD_F1.output(msbldF1_out.tnrso[i]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
		MSBLD_F1.setMultiScale(IMG_MULTI_SCALE_DOWN2, 1, 7);
		MSBLD_F1.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

		StageEx &MSBLD_F0 = sdRequest.emplaceStage(PEU_Stage::MSBLD_F0, frameNumber);
		auto &msbldF0_in = msbldF0_.in;
		auto &msbldF0_out = msbldF0_.out;

		MSBLD_F0.input(msbldF0_in.vipi[i]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
		MSBLD_F0.input(msbldF0_in.imgi[i]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
		MSBLD_F0.input(msbldF0_in.tnrsi[i]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
		MSBLD_F0.input(msbldF0_in.rec_dsi[0]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[1]);
		MSBLD_F0.input(msbldF0_in.tnrci[i]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
		MSBLD_F0.input(msbldF0_in.tnrwi[i]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[0]);
		MSBLD_F0.input(msbldF0_in.tnrvbi[i]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[1]);
		MSBLD_F0.input(msbldF0_in.tnrlfdi[i]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
		MSBLD_F0.input(msbldF0_in.tnrmi[i]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[1]);
		MSBLD_F0.input(msbldF0_in.tunbufi[i]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
		MSBLD_F0.output(msbldF0_out.img4o[i]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[0]);
		MSBLD_F0.output(msbldF0_out.tnrwo[i]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[0]);
		MSBLD_F0.output(msbldF0_out.tnrso[i]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
		MSBLD_F0.setMultiScale(IMG_MULTI_SCALE_DOWN2, 0, 7);
		MSBLD_F0.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);
	}
	requestHelper_.queueRequest(UserIdMfnr, sdRequest);
}

AfbldTask::AfbldTask(Scheduler *scheduler, const std::string &id, Request *request, uint32_t internalRequestId,
		     ImgSysDevice *imgSys, MFNRFrames &mfnr, MfnrTasksManager *manager)
	: Task(scheduler, id), requestHelper_(this, request, imgSys),
	  request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	afbldF0_ = mfnr.afbldF0;
	afbldF1_ = mfnr.afbldF1;
	afbldF2_ = mfnr.afbldF2;
	afbldF3_ = mfnr.afbldF3;
	afbldF4_ = mfnr.afbldF4;
	afbldF5_ = mfnr.afbldF5;
	afbldF6_ = mfnr.afbldF6;
	tnrso_ = mfnr.msbld_tnrso;
	stillOutput1_ = mfnr.still1Output;
	stillOutput2_ = mfnr.still2Output;
	mfnr_ = mfnr;
}

void AfbldTask::allocateOutputBuffers()
{
	auto &afbldF6_out = afbldF6_.out;
	manager_->nv21_1_64_pool_.fetch(afbldF6_out.img4o[0]);
	manager_->fourBytes_pool_.fetch(afbldF6_out.tnrso[0]);
	auto &afbldF5_out = afbldF5_.out;

	manager_->yuvp012_1_32_pool_.fetch(afbldF5_out.img3o[0]);
	manager_->y8_1_32_pool_.fetch(afbldF5_out.tnrwo[0]);
	manager_->y8_1_32_pool_.fetch(afbldF5_out.tnrmo[0]);
	manager_->fourBytes_pool_.fetch(afbldF5_out.tnrso[0]);

	auto &afbldF4_out = afbldF4_.out;
	manager_->yuvp012_1_16_pool_.fetch(afbldF4_out.img3o[0]);
	manager_->y8_1_16_pool_.fetch(afbldF4_out.tnrwo[0]);
	manager_->y8_1_16_pool_.fetch(afbldF4_out.tnrmo[0]);
	manager_->fourBytes_pool_.fetch(afbldF4_out.tnrso[0]);

	auto &afbldF3_out = afbldF3_.out;
	manager_->yuvp012_1_8_pool_.fetch(afbldF3_out.img3o[0]);
	manager_->y8_1_8_pool_.fetch(afbldF3_out.tnrwo[0]);
	manager_->y8_1_8_pool_.fetch(afbldF3_out.tnrmo[0]);
	manager_->fourBytes_pool_.fetch(afbldF3_out.tnrso[0]);

	auto &afbldF2_out = afbldF2_.out;
	manager_->yuvp012_1_4_pool_.fetch(afbldF2_out.img3o[0]);
	manager_->y8_1_4_pool_.fetch(afbldF2_out.tnrwo[0]);
	manager_->y8_1_4_pool_.fetch(afbldF2_out.tnrmo[0]);
	manager_->fourBytes_pool_.fetch(afbldF2_out.tnrso[0]);

	auto &afbldF1_out = afbldF1_.out;
	manager_->yuvp012_1_2_pool_.fetch(afbldF1_out.img3o[0]);
	manager_->y8_1_2_pool_.fetch(afbldF1_out.tnrwo[0]);
	manager_->y8_1_2_pool_.fetch(afbldF1_out.tnrmo[0]);
	manager_->fourBytes_pool_.fetch(afbldF1_out.tnrso[0]);

	auto &afbldF0_out = afbldF0_.out;
	//manager_->yuvp012_1_1_pool_.fetch(afbldF0_out.wdmao[0]);
	manager_->yuvp012_1_1_pool_.fetch(afbldF0_out.img3o[0]);
	manager_->y8_1_1_pool_.fetch(afbldF0_out.tnrwo[0]);
	manager_->nv12_wroto_pool_.fetch(afbldF0_out.wroto[0]);
	manager_->fourBytes_pool_.fetch(afbldF0_out.tnrso[0]);
}

void AfbldTask::notifyDone()
{
	auto bssOrder = mfnr_.bss_order->get();
	manager_->onDeviceTuner_->tuneAfbld(
		request_, internalRequestId_, afbldF0_, afbldF1_, afbldF2_,
		afbldF3_, afbldF4_, afbldF5_, afbldF6_, bssOrder, stillOutput1_, stillOutput2_);
	//MSBLD Precheck
	if (MfnrTasksManager::mfnrPrecheck()) {
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_AFBLD_F0:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_AFBLD_F1:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_AFBLD_F2:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_AFBLD_F3:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_AFBLD_F4:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_AFBLD_F5:1";
		LOG(MtkISP7, Info) << "[CAT][MFNR] EStage_AFBLD_F6:1";
	}
	Task::notifyDone();
}

void AfbldTask::run()
{
	allocateOutputBuffers();

	auto &mfnrSizes_ = manager_->mfnrSizes_;

	MUINT32 timestampMili = request_->metadata().get(controls::SensorTimestamp).value_or(0);
	SingleDeviceRequest sdRequest;
	auto bssOrder = mfnr_.bss_order->get();
	int frameNumber = internalRequestId_ + bssOrder[bssOrder.size() - 1];
	sdRequest.init(internalRequestId_, timestampMili, "AFBLD");

	StageEx &AFBLD_F6 = sdRequest.emplaceStage(PEU_Stage::AFBLD_F6, frameNumber);
	auto &afbldF6_in = afbldF6_.in;
	auto &afbldF6_out = afbldF6_.out;
	AFBLD_F6.input(afbldF6_in.vipi[0]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
	AFBLD_F6.input(afbldF6_in.imgi[0]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
	AFBLD_F6.input(afbldF6_in.tnrsi[0]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
	AFBLD_F6.input(afbldF6_in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
	AFBLD_F6.output(afbldF6_out.tnrso[0]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
	AFBLD_F6.output(afbldF6_out.img4o[0]->get(), IMG_PORT_IMG4O, 0, mfnrSizes_[6]);
	AFBLD_F6.setMultiScale(IMG_MULTI_SCALE_DOWN2, 6, 7);
	AFBLD_F6.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

	StageEx &AFBLD_F5 = sdRequest.emplaceStage(PEU_Stage::AFBLD_F5, frameNumber);
	auto &afbldF5_in = afbldF5_.in;
	auto &afbldF5_out = afbldF5_.out;
	AFBLD_F5.input(afbldF5_in.vipi[0]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
	AFBLD_F5.input(afbldF5_in.imgi[0]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
	AFBLD_F5.input(afbldF5_in.tnrsi[0]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
	AFBLD_F5.input(afbldF5_in.rec_dsi[0]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[6]);
	AFBLD_F5.input(afbldF5_in.tnrci[0]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
	AFBLD_F5.input(afbldF5_in.tnrwi[0]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[5]);
	AFBLD_F5.input(afbldF5_in.tnrvbi[0]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[5]);
	AFBLD_F5.input(afbldF5_in.tnrlfdi[0]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
	AFBLD_F5.input(afbldF5_in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
	AFBLD_F5.output(afbldF5_out.img3o[0]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[5]);
	AFBLD_F5.output(afbldF5_out.tnrwo[0]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[5]);
	AFBLD_F5.output(afbldF5_out.tnrmo[0]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[5]);
	AFBLD_F5.output(afbldF5_out.tnrso[0]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
	AFBLD_F5.setMultiScale(IMG_MULTI_SCALE_DOWN2, 5, 7);
	AFBLD_F5.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

	StageEx &AFBLD_F4 = sdRequest.emplaceStage(PEU_Stage::AFBLD_F4, frameNumber);
	auto &afbldF4_in = afbldF4_.in;
	auto &afbldF4_out = afbldF4_.out;
	AFBLD_F4.input(afbldF4_in.vipi[0]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
	AFBLD_F4.input(afbldF4_in.imgi[0]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
	AFBLD_F4.input(afbldF4_in.tnrsi[0]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
	AFBLD_F4.input(afbldF4_in.rec_dsi[0]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[5]);
	AFBLD_F4.input(afbldF4_in.tnrci[0]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
	AFBLD_F4.input(afbldF4_in.tnrwi[0]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[4]);
	AFBLD_F4.input(afbldF4_in.tnrvbi[0]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[4]);
	AFBLD_F4.input(afbldF4_in.tnrlfdi[0]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
	AFBLD_F4.input(afbldF4_in.tnrmi[0]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[5]);
	AFBLD_F4.input(afbldF4_in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
	AFBLD_F4.output(afbldF4_out.img3o[0]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[4]);
	AFBLD_F4.output(afbldF4_out.tnrwo[0]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[4]);
	AFBLD_F4.output(afbldF4_out.tnrmo[0]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[4]);
	AFBLD_F4.output(afbldF4_out.tnrso[0]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
	AFBLD_F4.setMultiScale(IMG_MULTI_SCALE_DOWN2, 4, 7);
	AFBLD_F4.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

	StageEx &AFBLD_F3 = sdRequest.emplaceStage(PEU_Stage::AFBLD_F3, frameNumber);
	auto &afbldF3_in = afbldF3_.in;
	auto &afbldF3_out = afbldF3_.out;
	AFBLD_F3.input(afbldF3_in.vipi[0]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
	AFBLD_F3.input(afbldF3_in.imgi[0]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
	AFBLD_F3.input(afbldF3_in.tnrsi[0]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
	AFBLD_F3.input(afbldF3_in.rec_dsi[0]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[4]);
	AFBLD_F3.input(afbldF3_in.tnrci[0]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
	AFBLD_F3.input(afbldF3_in.tnrwi[0]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[3]);
	AFBLD_F3.input(afbldF3_in.tnrvbi[0]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[3]);
	AFBLD_F3.input(afbldF3_in.tnrlfdi[0]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
	AFBLD_F3.input(afbldF3_in.tnrmi[0]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[4]);
	AFBLD_F3.input(afbldF3_in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
	AFBLD_F3.output(afbldF3_out.img3o[0]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[3]);
	AFBLD_F3.output(afbldF3_out.tnrwo[0]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[3]);
	AFBLD_F3.output(afbldF3_out.tnrmo[0]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[3]);
	AFBLD_F3.output(afbldF3_out.tnrso[0]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
	AFBLD_F3.setMultiScale(IMG_MULTI_SCALE_DOWN2, 3, 7);
	AFBLD_F3.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

	StageEx &AFBLD_F2 = sdRequest.emplaceStage(PEU_Stage::AFBLD_F2, frameNumber);
	auto &afbldF2_in = afbldF2_.in;
	auto &afbldF2_out = afbldF2_.out;
	AFBLD_F2.input(afbldF2_in.vipi[0]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
	AFBLD_F2.input(afbldF2_in.imgi[0]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
	AFBLD_F2.input(afbldF2_in.tnrsi[0]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
	AFBLD_F2.input(afbldF2_in.rec_dsi[0]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[3]);
	AFBLD_F2.input(afbldF2_in.tnrci[0]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
	AFBLD_F2.input(afbldF2_in.tnrwi[0]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[2]);
	AFBLD_F2.input(afbldF2_in.tnrvbi[0]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[2]);
	AFBLD_F2.input(afbldF2_in.tnrlfdi[0]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
	AFBLD_F2.input(afbldF2_in.tnrmi[0]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[3]);
	AFBLD_F2.input(afbldF2_in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
	AFBLD_F2.output(afbldF2_out.img3o[0]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[2]);
	AFBLD_F2.output(afbldF2_out.tnrwo[0]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[2]);
	AFBLD_F2.output(afbldF2_out.tnrmo[0]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[2]);
	AFBLD_F2.output(afbldF2_out.tnrso[0]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
	AFBLD_F2.setMultiScale(IMG_MULTI_SCALE_DOWN2, 2, 7);
	AFBLD_F2.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

	StageEx &AFBLD_F1 = sdRequest.emplaceStage(PEU_Stage::AFBLD_F1, frameNumber);
	auto &afbldF1_in = afbldF1_.in;
	auto &afbldF1_out = afbldF1_.out;
	AFBLD_F1.input(afbldF1_in.vipi[0]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
	AFBLD_F1.input(afbldF1_in.imgi[0]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
	AFBLD_F1.input(afbldF1_in.tnrsi[0]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
	AFBLD_F1.input(afbldF1_in.rec_dsi[0]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[2]);
	AFBLD_F1.input(afbldF1_in.tnrci[0]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
	AFBLD_F1.input(afbldF1_in.tnrwi[0]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[1]);
	AFBLD_F1.input(afbldF1_in.tnrvbi[0]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[1]);
	AFBLD_F1.input(afbldF1_in.tnrlfdi[0]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
	AFBLD_F1.input(afbldF1_in.tnrmi[0]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[2]);
	AFBLD_F1.input(afbldF1_in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });
	AFBLD_F1.output(afbldF1_out.img3o[0]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[1]);
	AFBLD_F1.output(afbldF1_out.tnrwo[0]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[1]);
	AFBLD_F1.output(afbldF1_out.tnrmo[0]->get(), IMG_PORT_TNRMO, 0, mfnrSizes_[1]);
	AFBLD_F1.output(afbldF1_out.tnrso[0]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });
	AFBLD_F1.setMultiScale(IMG_MULTI_SCALE_DOWN2, 1, 7);
	AFBLD_F1.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

	StageEx &AFBLD_F0 = sdRequest.emplaceStage(PEU_Stage::AFBLD_F0, frameNumber);
	auto &afbldF0_in = afbldF0_.in;
	auto &afbldF0_out = afbldF0_.out;
	AFBLD_F0.input(afbldF0_in.vipi[0]->get(), IMG_PORT_VIPI, 0, Size{ 0, 0 });
	AFBLD_F0.input(afbldF0_in.imgi[0]->get(), IMG_PORT_IMGI, 0, Size{ 0, 0 });
	AFBLD_F0.input(afbldF0_in.tnrsi[0]->get(), IMG_PORT_TNRSI, 0, Size{ 0, 0 });
	AFBLD_F0.input(afbldF0_in.rec_dsi[0]->get(), IMG_PORT_REC_DSI, 2, mfnrSizes_[1]);
	AFBLD_F0.input(afbldF0_in.tnrci[0]->get(), IMG_PORT_TNRCI, 2, manager_->confMapSize_);
	AFBLD_F0.input(afbldF0_in.tnrwi[0]->get(), IMG_PORT_TNRWI, 2, mfnrSizes_[0]);
	AFBLD_F0.input(afbldF0_in.tnrvbi[0]->get(), IMG_PORT_TNRVBI, 2, mfnrSizes_[1]);
	AFBLD_F0.input(afbldF0_in.tnrlfdi[0]->get(), IMG_PORT_TNRLFDI, 2, mfnrSizes_[6]);
	AFBLD_F0.input(afbldF0_in.tnrmi[0]->get(), IMG_PORT_TNRMI, 2, mfnrSizes_[1]);
	AFBLD_F0.input(afbldF0_in.tunbufi[0]->get(), IMG_PORT_METAI, 0, Size{ 0, 0 });

	AFBLD_F0.output(afbldF0_out.img3o[0]->get(), IMG_PORT_IMG3O, 0, mfnrSizes_[0]);
	AFBLD_F0.output(afbldF0_out.tnrwo[0]->get(), IMG_PORT_TNRWO, 0, mfnrSizes_[0]);
	AFBLD_F0.output(afbldF0_out.tnrso[0]->get(), IMG_PORT_TNRSO, 0, Size{ 0, 0 });

	AFBLD_F0.setMultiScale(IMG_MULTI_SCALE_DOWN2, 0, 7);
	AFBLD_F0.setMvFrame(mfnrSizes_[0], mfnrSizes_[2], 8);

	if (stillOutput1_) {
		InfoFrame info(formats::NV12, manager_->yuvOutputSize1_, stillOutput1_, 64);
		Rectangle crop = ImgSysDevice::getCrop(mfnrSizes_[0], info.size());
		AFBLD_F0.output(info, IMG_PORT_WDMAO, 0, crop);
	}

	if (stillOutput2_) {
		InfoFrame info(formats::NV12, manager_->yuvOutputSize2_, stillOutput2_, 64);
		Rectangle crop = ImgSysDevice::getCrop(Size{ 192, 144 }, info.size());
		AFBLD_F0.output(info, IMG_PORT_WROTO, 0, crop);
	}

	AFBLD_F0.setPqInfo();
	requestHelper_.queueRequest(UserIdMfnr, sdRequest);
}

} /* namespace libcamera */
