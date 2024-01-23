/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * mfnr.cpp - MtkISP7 ImgSys Device Mutiple Frame Noise Reduction
 */

#include "mfnr.h"

#include <cstdint>
#include <memory>
#include <stdio.h>
#include <stdlib.h>
#include <sys/resource.h>
#include <sys/sysinfo.h>
#include <unistd.h>

#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/request.h>

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "ImgPortDef.h"
#include "single_device.h"
#include "single_device_helper.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

constexpr int kInputRawCount = 4;

constexpr Size kP2sttoSize{ 738624, 1 };
constexpr Size kWrap2pSize{ 409, 305 };
constexpr Size kTnrsoSize{ 40, 1 };
constexpr Size kWrotoSize{ 192, 144 };
constexpr Size kTnrciSize{ 102, 76 };
/* todo: hide the NSCam::NSImgStream namespace in the single device interface. */
using namespace NSCam::NSImgStream;

static void zeroImage(SharedMailBox<InfoFrame> &mailBox)
{
	InfoFrame &info = mailBox->get();

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

	allBufferPools_.emplace_back(&tunbufiPool_);
	allBufferPools_.emplace_back(&p2sttoPool_);
	allBufferPools_.emplace_back(&tnrciPool_);
	allBufferPools_.emplace_back(&wrap2pPool_);
	allBufferPools_.emplace_back(&yuvp010_1_1_pool_);
	allBufferPools_.emplace_back(&yuvp010_1_4_pool_);
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
	allBufferPools_.emplace_back(&y8_1_8_pool_);
	allBufferPools_.emplace_back(&y8_1_16_pool_);
	allBufferPools_.emplace_back(&y8_1_32_pool_);
	allBufferPools_.emplace_back(&fourBytes_pool_);
	allBufferPools_.emplace_back(&nv12_1_64_pool_);
	allBufferPools_.emplace_back(&nv21_1_1_pool_);
	allBufferPools_.emplace_back(&nv12_wroto_pool_);

	poolsWritenByCpu_.emplace_back(&tunbufiPool_);
	poolsWritenByCpu_.emplace_back(&p2sttoPool_);
	poolsWritenByCpu_.emplace_back(&tnrciPool_);
	poolsWritenByCpu_.emplace_back(&wrap2pPool_);
	poolsWritenByCpu_.emplace_back(&yuvp010_1_1_pool_);
	poolsWritenByCpu_.emplace_back(&yuvp010_1_4_pool_);
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
	poolsWritenByCpu_.emplace_back(&y8_1_8_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_16_pool_);
	poolsWritenByCpu_.emplace_back(&y8_1_32_pool_);
	poolsWritenByCpu_.emplace_back(&fourBytes_pool_);
	poolsWritenByCpu_.emplace_back(&nv12_1_64_pool_);
	poolsWritenByCpu_.emplace_back(&nv21_1_1_pool_);
	poolsWritenByCpu_.emplace_back(&nv12_wroto_pool_);
}

int MfnrTasksManager::configure(const Size &bayerInputSize,
				const Size &yuvOutputSize1, const Size &yuvOutputSize2)
{
	yuvOutputSize1_ = yuvOutputSize1;
	yuvOutputSize2_ = yuvOutputSize2;
	bayerInputSize_ = bayerInputSize;

	mfnrSizes_.resize(7);
	Size size = bayerInputSize_;

	/* Assign the size to 1/2 of the previous level.
	 * Align to 2 for hardware's requirement */
	for (size_t i = 0; i < mfnrSizes_.size(); i++) {
		mfnrSizes_[i] = size;
		size.width = (size.width + 1) / 2;
		size.height = (size.height + 1) / 2;
		size.alignUpTo(2, 2);
	}

	configureBuffers();
	for (auto &pool : poolsWritenByCpu_)
		pool->mmap();
	return 0;
}

int MfnrTasksManager::configureBuffers()
{
	struct sysinfo info;

	if (sysinfo(&info) != 0) {
		perror("sysinfo");
		exit(EXIT_FAILURE);
	}

	LOG(MtkISP7, Info) << "Total file descriptors: " << info.procs;

	struct rlimit rlim;

	if (getrlimit(RLIMIT_NOFILE, &rlim) != 0) {
		perror("getrlimit");
		exit(EXIT_FAILURE);
	}

	if (rlim.rlim_cur == RLIM_INFINITY) {
		LOG(MtkISP7, Info) << "Current file descriptor limit: unlimited ";
	} else {
		LOG(MtkISP7, Info) << "Current file descriptor limit:" << rlim.rlim_cur;
	}

	if (rlim.rlim_max == RLIM_INFINITY) {
		LOG(MtkISP7, Info) << "Maximum file descriptor limit: unlimited:";
	} else {
		LOG(MtkISP7, Info) << "Maximum file descriptor limit: " << rlim.rlim_max;
	}

	// Increase the soft limit (current limit)
	rlim.rlim_cur = 2048; // Set your desired limit

	// Set the new limits
	if (setrlimit(RLIMIT_NOFILE, &rlim) != 0) {
		perror("setrlimit");
		exit(EXIT_FAILURE);
	}

	// Get and print the updated limits
	if (getrlimit(RLIMIT_NOFILE, &rlim) != 0) {
		perror("getrlimit");
		exit(EXIT_FAILURE);
	}

	LOG(MtkISP7, Error) << "New file descriptor limit:" << rlim.rlim_cur;

	//tunbufiPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 50);
	p2sttoPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kP2sttoSize, 4);
	wrap2pPool_.createBuffers(dmaHeap_, formats::WARP2P_MTISP, kWrap2pSize, 4);
	tnrciPool_.createBuffers(dmaHeap_, formats::Y8_MTISP, kTnrciSize, 3);

	yuvp010_1_1_pool_.createBuffers(dmaHeap_, formats::NV12_10P_MTISP, mfnrSizes_[0], 9);
	yuvp010_1_4_pool_.createBuffers(dmaHeap_, formats::NV12_10P_MTISP, mfnrSizes_[2], 18);

	yuvp012_1_1_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[0], 12, DmaHeap::System, 1, 64);
	yuvp012_1_2_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[1], 12, DmaHeap::System, 1, 64);
	yuvp012_1_4_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[2], 12, DmaHeap::System, 1, 64);
	yuvp012_1_8_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[3], 12, DmaHeap::System, 1, 64);
	yuvp012_1_16_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[4], 12, DmaHeap::System, 1, 64);
	yuvp012_1_32_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[5], 12, DmaHeap::System, 1, 64);
	yuvp012_1_64_pool_.createBuffers(dmaHeap_, formats::NV12_12P_MTISP, mfnrSizes_[6], 12, DmaHeap::System, 1, 64);

	y8_1_1_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[0], 10, DmaHeap::System, 16);
	y8_1_2_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[1], 10, DmaHeap::System, 16);
	y8_1_4_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[2], 14, DmaHeap::System, 16);
	y8_1_8_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[3], 10, DmaHeap::System, 16);
	y8_1_16_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[4], 10, DmaHeap::System, 16);
	y8_1_32_pool_.createBuffers(dmaHeap_, formats::Y8_MTISP, mfnrSizes_[5], 10, DmaHeap::System, 16);
	fourBytes_pool_.createBuffers(dmaHeap_, formats::Y32_MTISP, kTnrsoSize, 80);
	nv21_1_1_pool_.createBuffers(dmaHeap_, formats::NV21, mfnrSizes_[0], 7);
	nv12_1_64_pool_.createBuffers(dmaHeap_, formats::NV12, mfnrSizes_[6], 9);
	nv12_wroto_pool_.createBuffers(dmaHeap_, formats::NV12, kWrotoSize, 7);
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

void MfnrTasksManager::makeMFNRFrames(MFNRFrames &mfnr,
				      SharedMailBox<InfoFrame> &p1Raw,
				      FrameBuffer *output1Frame,
				      FrameBuffer *output2Frame)
{
	mfnr.still1Output = output1Frame;
	mfnr.still2Output = output2Frame;

	/* Should be created and generated by IPA, once it's ready */
	std::vector<SharedMailBox<InfoFrame>> bfbldTun = makeMailBoxVector<InfoFrame>(kInputRawCount);

	std::vector<SharedMailBox<InfoFrame>> bfbldP2stto = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bfbldImg2o = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bfbldImg3o = makeMailBoxVector<InfoFrame>(kInputRawCount);

	std::vector<SharedMailBox<InfoFrame>> bfmeTun = makeMailBoxVector<InfoFrame>(kInputRawCount);
	std::vector<SharedMailBox<InfoFrame>> bfmeImg2o = makeMailBoxVector<InfoFrame>(kInputRawCount);

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
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrci = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Img4o = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrwo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrmo = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_0Tnrso = makeMailBoxVector<InfoFrame>(7);

	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Tun = makeMailBoxVector<InfoFrame>(7);
	std::vector<SharedMailBox<InfoFrame>> msbldFx_1Tnrci = makeMailBoxVector<InfoFrame>(7);
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
	std::vector<SharedMailBox<InfoFrame>> afbldFx_Tnrci = makeMailBoxVector<InfoFrame>(7);

	/* Frames used by BfbldTask */
	BfbldFrames &bfbldFrames = mfnr.bfbldFrames;
	for (auto i = 0; i < kInputRawCount; i++) {
		bfbldFrames.in.timgi.push_back(p1Raw);
		//tunbufiPool_.fetch(bfbldTun[i]);
		if (i == 0) {
			bfbldFrames.in.tunbufi.push_back(bfbldTun[0]);
		} else {
			bfbldFrames.in.tunbufi.push_back(bfbldTun[1]);
		}
		bfbldFrames.out.p2stto.push_back(bfbldP2stto[i]);
		bfbldFrames.out.img2o.push_back(bfbldImg2o[i]);
		bfbldFrames.out.img3o.push_back(bfbldImg3o[i]);
	}
	/* Frames used by BfmeTask */
	BfmeFrames &bfmeFrames = mfnr.bfmeFrames;
	for (auto i = 0; i < kInputRawCount; i++) {
		bfmeFrames.in.imgi.push_back(bfbldFrames.out.img2o[i]);
		//tunbufiPool_.fetch(bfmeTun);
		bfmeFrames.in.tunbufi.push_back(bfmeTun[0]);
		bfmeFrames.out.img2o.push_back(bfmeImg2o[i]);
	}
	/* Frames used by MCDS_F1 Task */
	mcdsWpeVeci.resize(3);
	McdsF1Frames &mcdsF1Frames = mfnr.mcdsF1Frames;
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		mcdsF1Frames.in.wpe_wpei.push_back(bfbldFrames.out.img3o[i + 1]);
		//tunbufiPool_.fetch(mcdsF1Tun[i]);
		mcdsF1Frames.in.tunbufi.push_back(mcdsF1Tun[0]);
		mcdsWpeVeci[i] = makeMailBox<InfoFrame>();
		wrap2pPool_.fetch(mcdsWpeVeci[i]);
		zeroImage(mcdsWpeVeci[i]);
		mcdsF1Frames.in.wpe_veci.push_back(mcdsWpeVeci[i]);
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
		dsFrames.in.tunbufi.push_back(dsTun[0]);
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
		dsVbiFramesV2.in.tunbufi.push_back(dsVbiV2Tun[0]);
		dsVbiFramesV2.out.tyuv2o.push_back(dsVbiV2Tyuv2o[i]);
		dsVbiFramesV2.out.tyuv3o.push_back(dsVbiV2Tyuv3o[i]);
		dsVbiFramesV2.out.tyuv4o.push_back(dsVbiV2Tyuv4o[i]);

		dsVbiFramesV5.in.timgi.push_back(dsVbiFramesV2.out.tyuv4o[i]);
		//tunbufiPool_.fetch(dsVbiV5Tun[i]);
		dsVbiFramesV5.in.tunbufi.push_back(dsVbiV5Tun[0]);
		dsVbiFramesV5.out.tyuv2o.push_back(dsVbiV5Tyuv2o[i]);
		dsVbiFramesV5.out.tyuv3o.push_back(dsVbiV5Tyuv3o[i]);
		dsVbiFramesV5.out.tyuv4o.push_back(dsVbiV5Tyuv4o[i]);
	}

	auto constructMsbldMailBox =
		[](MsbldFrames &msbld, int idx,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tun,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Img4o,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tnrmo,
		   SharedMailBox<InfoFrame> &msbld_tnrso,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tnrwo,
		   std::vector<SharedMailBox<InfoFrame>> &msbldFx_Tnrci) {
			msbld.in.tunbufi.push_back(msbldFx_Tun[idx]);
			msbld.in.tnrci.push_back(msbldFx_Tnrci[0]);
			msbld.out.img4o.push_back(msbldFx_Img4o[idx]);
			msbld.out.tnrmo.push_back(msbldFx_Tnrmo[idx]);
			msbld.out.tnrso.push_back(msbld_tnrso);
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
		   SharedMailBox<InfoFrame> &msbld_tnrso,
		   std::vector<SharedMailBox<InfoFrame>> &afbldFx_Tnrci) {
			afbld.in.tunbufi.push_back(afbldFx_Tun[idx]);
			afbld.in.tnrci.push_back(afbldFx_Tnrci[0]);
			afbld.out.img4o.push_back(afbldFx_Img4o[idx]);
			afbld.out.img3o.push_back(afbldFx_Img3o[idx]);
			afbld.out.wdmao.push_back(afbldFx_Wdmao[idx]);
			afbld.out.wroto.push_back(afbldFx_Wroto[idx]);
			afbld.out.tnrwo.push_back(afbldFx_Tnrwo[idx]);
			afbld.out.tnrmo.push_back(afbldFx_Tnrmo[idx]);
			afbld.out.tnrso.push_back(msbld_tnrso);
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

	mfnr.msbld_tnrso = makeMailBox<InfoFrame>();
	fourBytes_pool_.fetch(mfnr.msbld_tnrso);
	constructMsbldMailBox(msbldF6, 6, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, mfnr.msbld_tnrso, msbldFx_0Tnrwo, msbldFx_0Tnrci);
	constructMsbldMailBox(msbldF5, 5, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, mfnr.msbld_tnrso, msbldFx_0Tnrwo, msbldFx_0Tnrci);
	constructMsbldMailBox(msbldF4, 4, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, mfnr.msbld_tnrso, msbldFx_0Tnrwo, msbldFx_0Tnrci);
	constructMsbldMailBox(msbldF3, 3, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, mfnr.msbld_tnrso, msbldFx_0Tnrwo, msbldFx_0Tnrci);
	constructMsbldMailBox(msbldF2, 2, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, mfnr.msbld_tnrso, msbldFx_0Tnrwo, msbldFx_0Tnrci);
	constructMsbldMailBox(msbldF1, 1, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, mfnr.msbld_tnrso, msbldFx_0Tnrwo, msbldFx_0Tnrci);
	constructMsbldMailBox(msbldF0, 0, msbldFx_0Tun, msbldFx_0Img4o, msbldFx_0Tnrmo, mfnr.msbld_tnrso, msbldFx_0Tnrwo, msbldFx_0Tnrci);

	constructMsbldMailBox(msbldF6, 6, msbldFx_0Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, mfnr.msbld_tnrso, msbldFx_1Tnrwo, msbldFx_1Tnrci);
	constructMsbldMailBox(msbldF5, 5, msbldFx_0Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, mfnr.msbld_tnrso, msbldFx_1Tnrwo, msbldFx_1Tnrci);
	constructMsbldMailBox(msbldF4, 4, msbldFx_0Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, mfnr.msbld_tnrso, msbldFx_1Tnrwo, msbldFx_1Tnrci);
	constructMsbldMailBox(msbldF3, 3, msbldFx_0Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, mfnr.msbld_tnrso, msbldFx_1Tnrwo, msbldFx_1Tnrci);
	constructMsbldMailBox(msbldF2, 2, msbldFx_0Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, mfnr.msbld_tnrso, msbldFx_1Tnrwo, msbldFx_1Tnrci);
	constructMsbldMailBox(msbldF1, 1, msbldFx_0Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, mfnr.msbld_tnrso, msbldFx_1Tnrwo, msbldFx_1Tnrci);
	constructMsbldMailBox(msbldF0, 0, msbldFx_0Tun, msbldFx_1Img4o, msbldFx_1Tnrmo, mfnr.msbld_tnrso, msbldFx_1Tnrwo, msbldFx_1Tnrci);

	constructAfbldMailBox(afbldF6, 6, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, mfnr.msbld_tnrso, afbldFx_Tnrci);
	constructAfbldMailBox(afbldF5, 5, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, mfnr.msbld_tnrso, afbldFx_Tnrci);
	constructAfbldMailBox(afbldF4, 4, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, mfnr.msbld_tnrso, afbldFx_Tnrci);
	constructAfbldMailBox(afbldF3, 3, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, mfnr.msbld_tnrso, afbldFx_Tnrci);
	constructAfbldMailBox(afbldF2, 2, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, mfnr.msbld_tnrso, afbldFx_Tnrci);
	constructAfbldMailBox(afbldF1, 1, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, mfnr.msbld_tnrso, afbldFx_Tnrci);
	constructAfbldMailBox(afbldF0, 0, afbldFx_Tun, afbldFx_Wroto, afbldFx_Wdmao, afbldFx_Img3o, afbldFx_Img4o, afbldFx_Tnrwo, afbldFx_Tnrmo, mfnr.msbld_tnrso, afbldFx_Tnrci);

	tnrciPool_.fetch(msbldFx_0Tnrci[0]);
	tnrciPool_.fetch(msbldFx_1Tnrci[0]);
	tnrciPool_.fetch(afbldFx_Tnrci[0]);
	zeroImage(msbldFx_0Tnrci[0]);
	zeroImage(msbldFx_1Tnrci[0]);
	zeroImage(afbldFx_Tnrci[0]);

	y8_1_32_pool_.fetch(msbldFx_Tnrwi[5]);
	y8_1_16_pool_.fetch(msbldFx_Tnrwi[4]);
	y8_1_8_pool_.fetch(msbldFx_Tnrwi[3]);
	y8_1_4_pool_.fetch(msbldFx_Tnrwi[2]);
	y8_1_2_pool_.fetch(msbldFx_Tnrwi[1]);
	y8_1_1_pool_.fetch(msbldFx_Tnrwi[0]);

	msbldF6.in.tnrsi.push_back(mfnr.msbld_tnrso); //4BYTE:40x1

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
	msbldF5.in.tnrsi.push_back(msbldF6.out.tnrso[0]); //4BYTE:40x1
	msbldF5.in.rec_dsi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	msbldF5.in.tnrvbi.push_back(dsVbiFramesV5.out.tyuv2o[0]); //Y8:102x78
	msbldF5.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40

	//MSBLD_F4(0)
	msbldF4.in.vipi.push_back(dsFrames.out.ltyuv2o[1]); //MTK_YUV_P012:204x154
	msbldF4.in.imgi.push_back(dsFrames.out.ltyuv2o[2]); //MTK_YUV_P012:204x154
	msbldF4.in.tnrsi.push_back(msbldF5.out.tnrso[0]); //4BYTE:40x1
	msbldF4.in.rec_dsi.push_back(msbldF5.out.img4o[0]); //MTK_YUV_P012:102x78
	msbldF4.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv4o[0]); //Y8:204x154
	msbldF4.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF4.in.tnrmi.push_back(msbldF5.out.tnrmo[0]); //Y8:102x78

	//MSBLD_F3(0)
	msbldF3.in.vipi.push_back(dsFrames.out.ltyuv4o[0]); //MTK_YUV_P012:408x306
	msbldF3.in.imgi.push_back(mcdsF1Frames.out.ltyuv4o[0]); //MTK_YUV_P012:408x306
	msbldF3.in.tnrsi.push_back(msbldF4.out.tnrso[0]); //4BYTE:40x1
	msbldF3.in.rec_dsi.push_back(msbldF4.out.img4o[0]); //MTK_YUV_P012:204x154
	msbldF3.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv3o[0]); //Y8:408x306
	msbldF3.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF3.in.tnrmi.push_back(msbldF4.out.tnrmo[0]); //Y8:204x154

	//MSBLD_F2(0)
	msbldF2.in.vipi.push_back(dsFrames.out.ltyuv3o[0]); //MTK_YUV_P012:816x612
	msbldF2.in.imgi.push_back(mcdsF1Frames.out.ltyuv3o[0]); //MTK_YUV_P012:816x612
	msbldF2.in.tnrsi.push_back(msbldF3.out.tnrso[0]); //4BYTE:40x1
	msbldF2.in.rec_dsi.push_back(msbldF3.out.img4o[0]); //MTK_YUV_P012:408x306
	msbldF2.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv2o[0]); //Y8:816x612
	msbldF2.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF2.in.tnrmi.push_back(msbldF3.out.tnrmo[0]); //Y8:408x306

	//MSBLD_F1(0)
	msbldF1.in.vipi.push_back(dsFrames.out.ltyuv2o[0]); //MTK_YUV_P012:1632x1224
	msbldF1.in.imgi.push_back(mcdsF1Frames.out.ltyuv2o[0]); //MTK_YUV_P012:1632x1224
	msbldF1.in.tnrsi.push_back(msbldF2.out.tnrso[0]); //4BYTE:40x1
	msbldF1.in.rec_dsi.push_back(msbldF2.out.img4o[0]); //MTK_YUV_P012:816x612
	msbldF1.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[0]); //Y8:1632x1224
	msbldF1.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF1.in.tnrmi.push_back(msbldF2.out.tnrmo[0]); //Y8:816x612

	//MSBLD_F0(0)
	msbldF0.in.vipi.push_back(bfbldFrames.out.img3o[0]); //MTK_YUV_P010:3264x2448
	msbldF0.in.imgi.push_back(mcdsF1Frames.out.wpe_wpeo[0]); //MTK_YUV_P010:3264x2448
	msbldF0.in.tnrsi.push_back(msbldF1.out.tnrso[0]); //4BYTE:40x1
	msbldF0.in.rec_dsi.push_back(msbldF1.out.img4o[0]); //MTK_YUV_P012:1632x1224
	msbldF0.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[0]); //Y8:1632x1224
	msbldF0.in.tnrlfdi.push_back(msbldF6.out.img4o[0]); //NV21:52x40
	msbldF0.in.tnrmi.push_back(msbldF1.out.tnrmo[0]); //Y8:1632x1224

	//MSBLD_F6(1)
	msbldF6.in.vipi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	msbldF6.in.imgi.push_back(dsFrames.out.ltyuv4o[3]); //MTK_YUV_P012:52x40
	msbldF6.in.tnrsi.push_back(msbldF0.out.tnrso[0]); //4BYTE:40x1

	//MSBLD_F5(1)
	msbldF5.in.vipi.push_back(msbldF5.out.img4o[0]); //MTK_YUV_P012:102x78
	msbldF5.in.imgi.push_back(dsFrames.out.ltyuv3o[3]); //MTK_YUV_P012:102x78
	msbldF5.in.tnrsi.push_back(msbldF6.out.tnrso[1]); //4BYTE:40x1
	msbldF5.in.rec_dsi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	msbldF5.in.tnrwi.push_back(msbldF5.out.tnrwo[0]); //Y8:102x78
	msbldF5.in.tnrvbi.push_back(dsVbiFramesV5.out.tyuv2o[1]); //Y8:102x78
	msbldF5.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40

	//MSBLD_F4(1)
	msbldF4.in.vipi.push_back(msbldF4.out.img4o[0]); //MTK_YUV_P012:204x154
	msbldF4.in.imgi.push_back(dsFrames.out.ltyuv2o[3]); //MTK_YUV_P012:204x154
	msbldF4.in.tnrsi.push_back(msbldF5.out.tnrso[1]); //4BYTE:40x1
	msbldF4.in.rec_dsi.push_back(msbldF5.out.img4o[1]); //MTK_YUV_P012:102x78
	msbldF4.in.tnrwi.push_back(msbldF4.out.tnrwo[0]); //Y8:204x154
	msbldF4.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv4o[1]); //Y8:204x154
	msbldF4.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF4.in.tnrmi.push_back(msbldF5.out.tnrmo[1]); //Y8:102x78

	//MSBLD_F3(1)
	msbldF3.in.vipi.push_back(msbldF3.out.img4o[0]); //MTK_YUV_P012:408x306
	msbldF3.in.imgi.push_back(mcdsF1Frames.out.ltyuv4o[1]); //MTK_YUV_P012:408x306
	msbldF3.in.tnrsi.push_back(msbldF4.out.tnrso[1]); //4BYTE:40x1
	msbldF3.in.rec_dsi.push_back(msbldF4.out.img4o[1]); //MTK_YUV_P012:204x154
	msbldF3.in.tnrwi.push_back(msbldF3.out.tnrwo[0]); //Y8:408x306
	msbldF3.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv3o[1]); //Y8:408x306
	msbldF3.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF3.in.tnrmi.push_back(msbldF4.out.tnrmo[1]); //Y8:204x154

	//MSBLD_F2(1)
	msbldF2.in.vipi.push_back(msbldF2.out.img4o[0]); //MTK_YUV_P012:816x612
	msbldF2.in.imgi.push_back(mcdsF1Frames.out.ltyuv3o[1]); //MTK_YUV_P012:816x612
	msbldF2.in.tnrsi.push_back(msbldF3.out.tnrso[1]); //4BYTE:40x1
	msbldF2.in.rec_dsi.push_back(msbldF3.out.img4o[1]); //MTK_YUV_P012:408x306
	msbldF2.in.tnrwi.push_back(msbldF2.out.tnrwo[0]); //Y8:816x612
	msbldF2.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv2o[1]); //Y8:816x612
	msbldF2.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF2.in.tnrmi.push_back(msbldF3.out.tnrmo[1]); //Y8:408x306

	//MSBLD_F1(1)
	msbldF1.in.vipi.push_back(msbldF1.out.img4o[0]); //MTK_YUV_P012:1632x1224
	msbldF1.in.imgi.push_back(mcdsF1Frames.out.ltyuv2o[1]); //MTK_YUV_P012:1632x1224
	msbldF1.in.tnrsi.push_back(msbldF2.out.tnrso[1]); //4BYTE:40x1
	msbldF1.in.rec_dsi.push_back(msbldF2.out.img4o[1]); //MTK_YUV_P012:816x612
	msbldF1.in.tnrwi.push_back(msbldF1.out.tnrwo[0]); //Y8:1632x1224
	msbldF1.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[1]); //Y8:1632x1224
	msbldF1.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF1.in.tnrmi.push_back(msbldF2.out.tnrmo[1]); //Y8:816x612

	//MSBLD_F0(1)
	msbldF0.in.vipi.push_back(msbldF0.out.img4o[0]); //MTK_YUV_P010:3264x2448
	msbldF0.in.imgi.push_back(mcdsF1Frames.out.wpe_wpeo[1]); //MTK_YUV_P010:3264x2448
	msbldF0.in.tnrsi.push_back(msbldF1.out.tnrso[1]); //4BYTE:40x1
	msbldF0.in.rec_dsi.push_back(msbldF1.out.img4o[1]); //MTK_YUV_P012:1632x1224
	msbldF0.in.tnrwi.push_back(msbldF0.out.tnrwo[0]); //Y8:3264x2448
	msbldF0.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[1]); //Y8:1632x1224
	msbldF0.in.tnrlfdi.push_back(msbldF6.out.img4o[1]); //NV21:52x40
	msbldF0.in.tnrmi.push_back(msbldF1.out.tnrmo[1]); //Y8:1632x1224

	//98(24)

	//AFBLD_F6(0)
	afbldF6.in.vipi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	afbldF6.in.imgi.push_back(dsFrames.out.ltyuv4o[4]); //MTK_YUV_P012:52x40
	afbldF6.in.tnrsi.push_back(msbldF0.out.tnrso[1]); //4BYTE:40x1

	//AFBLD_F5(0)
	afbldF5.in.vipi.push_back(msbldF5.out.img4o[1]); //MTK_YUV_P012:102x78
	afbldF5.in.imgi.push_back(dsFrames.out.ltyuv3o[4]); //MTK_YUV_P012:102x78
	afbldF5.in.tnrsi.push_back(afbldF6.out.tnrso[0]); //4BYTE:40x1
	afbldF5.in.rec_dsi.push_back(dsFrames.out.ltyuv4o[1]); //MTK_YUV_P012:52x40
	afbldF5.in.tnrwi.push_back(msbldF5.out.tnrwo[1]); //Y8:102x78
	afbldF5.in.tnrvbi.push_back(dsVbiFramesV5.out.tyuv2o[2]); //Y8:102x78
	afbldF5.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40

	//AFBLD_F4(0)
	afbldF4.in.vipi.push_back(msbldF4.out.img4o[1]); //MTK_YUV_P012:204x154
	afbldF4.in.imgi.push_back(dsFrames.out.ltyuv2o[4]); //MTK_YUV_P012:204x154
	afbldF4.in.tnrsi.push_back(afbldF5.out.tnrso[0]); //4BYTE:40x1
	afbldF4.in.rec_dsi.push_back(afbldF5.out.img3o[0]); //MTK_YUV_P012:102x78
	afbldF4.in.tnrwi.push_back(msbldF4.out.tnrwo[1]); //Y8:204x154
	afbldF4.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv4o[2]); //Y8:204x154
	afbldF4.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF4.in.tnrmi.push_back(afbldF5.out.tnrmo[0]); //Y8:102x78

	//AFBLD_F3(0)
	afbldF3.in.vipi.push_back(msbldF3.out.img4o[1]); //MTK_YUV_P012:408x306
	afbldF3.in.imgi.push_back(mcdsF1Frames.out.ltyuv4o[2]); //MTK_YUV_P012:408x306
	afbldF3.in.tnrsi.push_back(afbldF4.out.tnrso[0]); //4BYTE:40x1
	afbldF3.in.rec_dsi.push_back(afbldF4.out.img3o[0]); //MTK_YUV_P012:204x154
	afbldF3.in.tnrwi.push_back(msbldF3.out.tnrwo[1]); //Y8:408x306
	afbldF3.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv3o[2]); //Y8:408x306
	afbldF3.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF3.in.tnrmi.push_back(afbldF4.out.tnrmo[0]); //Y8:204x154

	//AFBLD_F2(0)
	afbldF2.in.vipi.push_back(msbldF2.out.img4o[1]); //MTK_YUV_P012:816x612
	afbldF2.in.imgi.push_back(mcdsF1Frames.out.ltyuv3o[2]); //MTK_YUV_P012:816x612
	afbldF2.in.tnrsi.push_back(afbldF3.out.tnrso[0]); //4BYTE:40x1
	afbldF2.in.rec_dsi.push_back(afbldF3.out.img3o[0]); //MTK_YUV_P012:408x306
	afbldF2.in.tnrwi.push_back(msbldF2.out.tnrwo[1]); //Y8:816x612
	afbldF2.in.tnrvbi.push_back(dsVbiFramesV2.out.tyuv2o[2]); //Y8:816x612
	afbldF2.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF2.in.tnrmi.push_back(afbldF3.out.tnrmo[0]); //Y8:408x306

	//AFBLD_F1(0)
	afbldF1.in.vipi.push_back(msbldF1.out.img4o[1]); //MTK_YUV_P012:1632x1224
	afbldF1.in.imgi.push_back(mcdsF1Frames.out.ltyuv2o[2]); //MTK_YUV_P012:1632x1224
	afbldF1.in.tnrsi.push_back(afbldF2.out.tnrso[0]); //4BYTE:40x1
	afbldF1.in.rec_dsi.push_back(afbldF2.out.img3o[0]); //MTK_YUV_P012:816x612
	afbldF1.in.tnrwi.push_back(msbldF1.out.tnrwo[1]); //Y8:1632x1224
	afbldF1.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[2]); //Y8:1632x1224
	afbldF1.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF1.in.tnrmi.push_back(afbldF2.out.tnrmo[0]); //Y8:816x612

	//AFBLD_F0(0)
	afbldF0.in.vipi.push_back(msbldF0.out.img4o[1]); //MTK_YUV_P010:3264x2448
	afbldF0.in.imgi.push_back(mcdsF1Frames.out.wpe_wpeo[2]); //MTK_YUV_P010:3264x2448
	afbldF0.in.tnrsi.push_back(afbldF1.out.tnrso[0]); //4BYTE:40x1
	afbldF0.in.rec_dsi.push_back(afbldF1.out.img3o[0]); //MTK_YUV_P012:1632x1224
	afbldF0.in.tnrwi.push_back(msbldF0.out.tnrwo[1]); //Y8:3264x2448
	afbldF0.in.tnrvbi.push_back(mcdsF1Frames.out.ltyuv5o[2]); //Y8:1632x1224
	afbldF0.in.tnrlfdi.push_back(afbldF6.out.img4o[0]); //NV21:52x40
	afbldF0.in.tnrmi.push_back(afbldF1.out.tnrmo[0]); //Y8:1632x1224

	LOG(MtkISP7, Error) << "makeMFNRFrames done";
}

} /* namespace libcamera */
