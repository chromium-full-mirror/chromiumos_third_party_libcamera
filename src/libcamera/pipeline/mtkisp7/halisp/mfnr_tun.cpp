/*
 * Copyright (C) 2024, Google Inc.
 *
 * mfnr_tun.cpp - MTK MtkISP7 mfnr tuning generator
 */

#include "mfnr_tun.h"

#include <functional>
#include <memory>

#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/imgsys/mfnr.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/feature.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"
#include "tuning_mapping/cam_idx_struct_ext_pub.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

static constexpr Size kTunSize{ 219348, 1 };

} //namespace

[[maybe_unused]] static void fillTuning(SharedMailBox<InfoFrame> &mailBox, uint8_t *tuning)
{
	assert(tuning);

	const InfoFrame &info = mailBox->get();

	void *dest = info.address(0);
	size_t length = info.buffer()->planes()[0].length;

	assert(dest);
	assert(mailBox->valid());

	{
		DmaSyncer syncer(info.buffer()->planes()[0].fd.get());
		memcpy(dest, tuning, length);
	}
}

MfnrTunManager::MfnrTunManager(
	DmaHeap *dmaHeap, IPADelegate *ipa, OnDeviceTuner *odt)
{
	dmaHeap_ = dmaHeap;
	ipa_ = ipa;
	onDeviceTuner_ = odt;
}

void MfnrTunManager::allocateBuffers()
{
	mfnrTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 45, DmaHeap::CMA);
	mfnrTun_.mmap();
}

void MfnrTunManager::releaseBuffers()
{
	mfnrTun_.unmap();
	mfnrTun_.release();
}

int MfnrTunManager::configure(const Size &bayerInputSize,
			      const Size &yuvOutput1Size, const Size &yuvOutput2Size,
			      std::shared_ptr<mtk::isphal::v1::isp_swme_Param> swme,
			      std::shared_ptr<mtk::isphal::v1::isp_bss_Param> bss)
{
	yuvOutput1Size_ = yuvOutput1Size;
	yuvOutput2Size_ = yuvOutput2Size;

	bayerInputSize_ = bayerInputSize;

	swme_ = swme;
	bss_ = bss;

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
	needCropTNC16x9_ = false;
	if ((yuvOutput1Size_.width * 9 == yuvOutput1Size_.height * 16) &&
	    (yuvOutput2Size_.width * 9 == yuvOutput2Size_.height * 16))
		needCropTNC16x9_ = true;
	allocateBuffers();

	return 0;
}

std::tuple<MfnrTunBssTask *, MfnrTunBfbldTask *, MfnrTunBfmeTask *,
	   MfnrTunSwmeTask *, MfnrTunDsTask *, MfnrTunDsVbiTask *, MfnrTunMcdsF1Task *,
	   MfnrTunMsbldTask *, MfnrTunAfbldTask *>
MfnrTunManager::makeMfnrTunTasks(MFNRFrames &mfnr,
				 uint32_t camSysMetaRequestId,
				 Scheduler *scheduler,
				 const std::string &id, Request *request,
				 uint32_t internalRequestId)
{
	MfnrTunBssTask *mfnrTunBssTask = new MfnrTunBssTask(
		mfnr, scheduler, id, bss_);

	MfnrTunBfbldTask *mfnrTunBfbldTask = new MfnrTunBfbldTask(
		mfnr, camSysMetaRequestId, scheduler, id, request, this, internalRequestId);

	MfnrTunBfmeTask *mfnrTunBfmeTask = new MfnrTunBfmeTask(
		mfnr, camSysMetaRequestId, scheduler, id, request, this, internalRequestId);

	MfnrTunSwmeTask *mfnrTunSwmeTask = new MfnrTunSwmeTask(
		mfnr, scheduler, id, swme_);

	MfnrTunDsTask *mfnrTunDsTask = new MfnrTunDsTask(
		mfnr, camSysMetaRequestId, scheduler, id, request, this, internalRequestId);

	MfnrTunDsVbiTask *mfnrTunDsVbiTask = new MfnrTunDsVbiTask(
		mfnr, camSysMetaRequestId, scheduler, id, request, this, internalRequestId);

	MfnrTunMcdsF1Task *mfnrTunMcdsF1Task = new MfnrTunMcdsF1Task(
		mfnr, camSysMetaRequestId, scheduler, id, request, this, internalRequestId);

	MfnrTunMsbldTask *mfnrTunMsbldTask = new MfnrTunMsbldTask(
		mfnr, camSysMetaRequestId, scheduler, id, request, this, internalRequestId);

	MfnrTunAfbldTask *mfnrTunAfbldTask = new MfnrTunAfbldTask(
		mfnr, camSysMetaRequestId, scheduler, id, request, this, internalRequestId);

	return std::make_tuple(mfnrTunBssTask, mfnrTunBfbldTask, mfnrTunBfmeTask,
			       mfnrTunSwmeTask, mfnrTunDsTask, mfnrTunDsVbiTask, mfnrTunMcdsF1Task,
			       mfnrTunMsbldTask, mfnrTunAfbldTask);
}

MfnrTunBssTask::MfnrTunBssTask(MFNRFrames &mfnr,
			       Scheduler *scheduler,
			       const std::string &id,
			       std::shared_ptr<mtk::isphal::v1::isp_bss_Param> bss)
	: Task(scheduler, id), bss_(bss)
{
	bssFrames_ = mfnr.bssFrames;
}

void MfnrTunBssTask::run()
{
	auto &in = bssFrames_.in;
	in.db_param->put(bss_, nullptr);

	notifyDone();
}

MfnrTunBfbldTask::MfnrTunBfbldTask(MFNRFrames &mfnr,
				   uint32_t camSysMetaRequestId,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: ImgSysTask(scheduler, id, camSysMetaRequestId, internalRequestId,
		     Feature::Capture_mfnr, manager->ipa_, request->controls()),
	  request_(request), manager_(manager)
{
	bfbldTun_ = mfnr.bfbldFrames.in.tunbufi;
	bssOrder_ = mfnr.bss_order;
}

void MfnrTunBfbldTask::run()
{
	ipa::mtkisp7::ImgMetaRequestData request;
	std::vector<ipa::mtkisp7::ImgMetaRequestData> requests;

	for (auto i = 0; i < kInputRawCount; i++) {
		manager_->mfnrTun_.fetch(bfbldTun_[i]);
	}
	auto &bssOrder = bssOrder_->get();

	//fillTuning(bfbldRefTun_, tuningBuffers.capture_BFBLD_REF_tunbufi);
	for (auto i = 0; i < kInputRawCount; i++) {
		auto frameNumber = internalRequestId_ + bssOrder[i];
		request = ipa::mtkisp7::ImgMetaRequestData(
			true,
			(i == 0) ? NSIspTuning::EStage_BFBLD_BASE
				 : NSIspTuning::EStage_BFBLD_REF,
			bfbldTun_[i]->get().buffer()->cookie(),
			0, 0, manager_->mfnrSizes_[0],
			manager_->yuvOutput1Size_, manager_->yuvOutput2Size_,
			manager_->mfnrSizes_[0], {}, true, 0, 1, true, frameNumber);
		requests.push_back(std::move(request));
	}

	getImgSysMetaTuning(manager_->needCropTNC16x9_, requests);
}

MfnrTunBfmeTask::MfnrTunBfmeTask(MFNRFrames &mfnr,
				 uint32_t camSysMetaRequestId,
				 Scheduler *scheduler,
				 const std::string &id, Request *request, MfnrTunManager *manager,
				 uint32_t internalRequestId)
	: ImgSysTask(scheduler, id, camSysMetaRequestId, internalRequestId,
		     Feature::Capture_mfnr, manager->ipa_, request->controls()),
	  request_(request), manager_(manager)
{
	bfmeTun_ = mfnr.bfmeFrames.in.tunbufi;
	bssOrder_ = mfnr.bss_order;
	tncso_ = mfnr.bfmeFrames.tncso;
}

void MfnrTunBfmeTask::run()
{
	ipa::mtkisp7::ImgMetaRequestData request;
	std::vector<ipa::mtkisp7::ImgMetaRequestData> requests;

	for (auto i = 0; i < kInputRawCount; i++) {
		manager_->mfnrTun_.fetch(bfmeTun_[i]);
	}
	//fillTuning(bfmeTun_, tuningBuffers.capture_BFME_tunbufi);

	auto &bssOrder = bssOrder_->get();
	for (auto i = 0; i < kInputRawCount; i++) {
		auto frameNumber = internalRequestId_ + bssOrder[i];
		request = ipa::mtkisp7::ImgMetaRequestData(
			true, NSIspTuning::EStage_BFME,
			bfmeTun_[i]->get().buffer()->cookie(),
			tncso_->get().buffer()->cookie(), 0,
			manager_->mfnrSizes_[2],
			manager_->yuvOutput1Size_, manager_->yuvOutput2Size_,
			manager_->mfnrSizes_[0], {}, true, 0, 1, true, frameNumber);
		requests.push_back(std::move(request));
	}

	getImgSysMetaTuning(manager_->needCropTNC16x9_, requests);
}

MfnrTunSwmeTask::MfnrTunSwmeTask(MFNRFrames &mfnr,
				 Scheduler *scheduler,
				 const std::string &id,
				 std::shared_ptr<mtk::isphal::v1::isp_swme_Param> swme)
	: Task(scheduler, id), swme_(swme)
{
	swmeFrames_ = mfnr.swmeFrames;
}

void MfnrTunSwmeTask::run()
{
	auto &in = swmeFrames_.in;
	for (auto i = 0; i < (int)in.db_param.size(); i++) {
		in.db_param[i]->put(swme_, nullptr);
	}

	notifyDone();
}

MfnrTunDsTask::MfnrTunDsTask(MFNRFrames &mfnr,
			     uint32_t camSysMetaRequestId,
			     Scheduler *scheduler,
			     const std::string &id, Request *request, MfnrTunManager *manager,
			     uint32_t internalRequestId)
	: ImgSysTask(scheduler, id, camSysMetaRequestId, internalRequestId,
		     Feature::Capture_mfnr, manager->ipa_, request->controls()),
	  request_(request), manager_(manager)
{
	dsTun = mfnr.dsFrames.in.tunbufi;
	bssOrder_ = mfnr.bss_order;
}

void MfnrTunDsTask::run()
{
	ipa::mtkisp7::ImgMetaRequestData request;
	std::vector<ipa::mtkisp7::ImgMetaRequestData> requests;

	auto &bssOrder = bssOrder_->get();

	for (auto i = 0; i < (int)bssOrder.size() + 1; i++) {
		manager_->mfnrTun_.fetch(dsTun[i]);
	}

	for (auto i = 0; i < (int)bssOrder.size() + 1; i++) {
		int frameNumber = (i == 0 || i == 1) ? internalRequestId_ + bssOrder[0] : internalRequestId_ + bssOrder[i - 1];
		request = ipa::mtkisp7::ImgMetaRequestData(
			true, NSIspTuning::EStage_DS,
			dsTun[i]->get().buffer()->cookie(),
			0, 0, manager_->mfnrSizes_[0],
			manager_->yuvOutput1Size_, manager_->yuvOutput2Size_,
			manager_->mfnrSizes_[0], {}, true, 0, 1, true, frameNumber);
		requests.push_back(std::move(request));
	}

	getImgSysMetaTuning(manager_->needCropTNC16x9_, requests);
}

MfnrTunMcdsF1Task::MfnrTunMcdsF1Task(MFNRFrames &mfnr,
				     uint32_t camSysMetaRequestId,
				     Scheduler *scheduler,
				     const std::string &id, Request *request, MfnrTunManager *manager,
				     uint32_t internalRequestId)
	: ImgSysTask(scheduler, id, camSysMetaRequestId, internalRequestId,
		     Feature::Capture_mfnr, manager->ipa_, request->controls()),
	  request_(request), manager_(manager)
{
	mcdsF1Tun_.resize(kInputRawCount - 1);
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		mcdsF1Tun_[i] = mfnr.mcdsF1Frames.in.tunbufi[i];
	}
	bssOrder_ = mfnr.bss_order;
}

void MfnrTunMcdsF1Task::run()
{
	ipa::mtkisp7::ImgMetaRequestData request;
	std::vector<ipa::mtkisp7::ImgMetaRequestData> requests;

	for (auto i = 0; i < kInputRawCount - 1; i++) {
		manager_->mfnrTun_.fetch(mcdsF1Tun_[i]);
		//fillTuning(mcdsF1Tun_[i], tuningBuffers.capture_MCDS_F1_tunbufi);
	}
	auto &bssOrder = bssOrder_->get();
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		int frameNumber = internalRequestId_ + bssOrder[i + 1];
		request = ipa::mtkisp7::ImgMetaRequestData(
			true, NSIspTuning::EStage_MCDS_F1,
			mcdsF1Tun_[i]->get().buffer()->cookie(),
			0, 0, manager_->mfnrSizes_[0],
			manager_->yuvOutput1Size_, manager_->yuvOutput2Size_,
			manager_->mfnrSizes_[0], {}, true, 0, 1, true, frameNumber);
		requests.push_back(std::move(request));
	}

	getImgSysMetaTuning(manager_->needCropTNC16x9_, requests);
}

MfnrTunDsVbiTask::MfnrTunDsVbiTask(MFNRFrames &mfnr,
				   uint32_t camSysMetaRequestId,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: ImgSysTask(scheduler, id, camSysMetaRequestId, internalRequestId,
		     Feature::Capture_mfnr, manager->ipa_, request->controls()),
	  request_(request), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	dsVbiV2Tun_ = mfnr.dsVbiFramesV2.in.tunbufi;
	dsVbiV5Tun_ = mfnr.dsVbiFramesV5.in.tunbufi;
	bssOrder_ = mfnr.bss_order;
}

void MfnrTunDsVbiTask::run()
{
	ipa::mtkisp7::ImgMetaRequestData request;
	std::vector<ipa::mtkisp7::ImgMetaRequestData> requests;

	auto bssOrder = bssOrder_->get();
	for (auto i = 0; i < (int)bssOrder.size() - 1; i++) {
		manager_->mfnrTun_.fetch(dsVbiV2Tun_[i]);
		manager_->mfnrTun_.fetch(dsVbiV5Tun_[i]);
	}

	//fillTuning(dsVbiV2Tun_, tuningBuffers.capture_DS_VBI_V2_tunbufi);
	//fillTuning(dsVbiV5Tun_, tuningBuffers.capture_DS_VBI_V5_tunbufi);
	for (auto i = 0; i < (int)bssOrder.size() - 1; i++) {
		int frameNumber = internalRequestId_ + bssOrder[i + 1];
		request = ipa::mtkisp7::ImgMetaRequestData(
			true, NSIspTuning::EStage_DS_VBI_V2,
			dsVbiV2Tun_[i]->get().buffer()->cookie(),
			0, 0, manager_->mfnrSizes_[1],
			manager_->yuvOutput1Size_, manager_->yuvOutput2Size_,
			manager_->mfnrSizes_[0], {}, true, 0, 1, true, frameNumber);
		requests.push_back(std::move(request));

		request = ipa::mtkisp7::ImgMetaRequestData(
			true, NSIspTuning::EStage_DS_VBI_V5,
			dsVbiV5Tun_[i]->get().buffer()->cookie(),
			0, 0, manager_->mfnrSizes_[4],
			manager_->yuvOutput1Size_, manager_->yuvOutput2Size_,
			manager_->mfnrSizes_[0], {}, true, 0, 1, true, frameNumber);
		requests.push_back(std::move(request));
	}

	getImgSysMetaTuning(manager_->needCropTNC16x9_, requests);
}

MfnrTunMsbldTask::MfnrTunMsbldTask(MFNRFrames &mfnr,
				   uint32_t camSysMetaRequestId,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: ImgSysTask(scheduler, id, camSysMetaRequestId, internalRequestId,
		     Feature::Capture_mfnr, manager->ipa_, request->controls()),
	  request_(request), manager_(manager)
{
	msbldF0Tun_.resize(kInputRawCount - 2);
	msbldF1Tun_.resize(kInputRawCount - 2);
	msbldF2Tun_.resize(kInputRawCount - 2);
	msbldF3Tun_.resize(kInputRawCount - 2);
	msbldF4Tun_.resize(kInputRawCount - 2);
	msbldF5Tun_.resize(kInputRawCount - 2);
	msbldF6Tun_.resize(kInputRawCount - 2);
	for (auto i = 0; i < kInputRawCount - 2; i++) {
		msbldF0Tun_[i] = mfnr.msbldF0.in.tunbufi[i];
		msbldF1Tun_[i] = mfnr.msbldF1.in.tunbufi[i];
		msbldF2Tun_[i] = mfnr.msbldF2.in.tunbufi[i];
		msbldF3Tun_[i] = mfnr.msbldF3.in.tunbufi[i];
		msbldF4Tun_[i] = mfnr.msbldF4.in.tunbufi[i];
		msbldF5Tun_[i] = mfnr.msbldF5.in.tunbufi[i];
		msbldF6Tun_[i] = mfnr.msbldF6.in.tunbufi[i];
	}
	bssOrder_ = mfnr.bss_order;
}

void MfnrTunMsbldTask::run()
{
	ipa::mtkisp7::ImgMetaRequestData request;
	std::vector<ipa::mtkisp7::ImgMetaRequestData> requests;

	for (auto i = 0; i < kInputRawCount - 2; i++) {
		manager_->mfnrTun_.fetch(msbldF0Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF1Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF2Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF3Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF4Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF5Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF6Tun_[i]);
	}

	std::map<EStage_T, std::vector<SharedMailBox<InfoFrame>>, std::greater<EStage_T>> stageToTuningMap = {
		{ NSIspTuning::EStage_MSBLD_F0, msbldF0Tun_ },
		{ NSIspTuning::EStage_MSBLD_F1, msbldF1Tun_ },
		{ NSIspTuning::EStage_MSBLD_F2, msbldF2Tun_ },
		{ NSIspTuning::EStage_MSBLD_F3, msbldF3Tun_ },
		{ NSIspTuning::EStage_MSBLD_F4, msbldF4Tun_ },
		{ NSIspTuning::EStage_MSBLD_F5, msbldF5Tun_ },
		{ NSIspTuning::EStage_MSBLD_F6, msbldF6Tun_ }
	};

	auto bssOrder = bssOrder_->get();
	for (auto i = 0; i < kInputRawCount - 2; i++) {
		for (auto it = stageToTuningMap.begin(); it != stageToTuningMap.end(); it++) {
			int size_idx = it->first - EStage_MSBLD_F0;
			int frameNumber = internalRequestId_ + bssOrder[i + 1];
			request = ipa::mtkisp7::ImgMetaRequestData(
				true, it->first,
				it->second[i]->get().buffer()->cookie(),
				0, 0, manager_->mfnrSizes_[size_idx],
				manager_->yuvOutput1Size_, manager_->yuvOutput2Size_,
				manager_->mfnrSizes_[0], {}, true, i,
				kInputRawCount, true, frameNumber);
			requests.push_back(std::move(request));
		}
	}

	getImgSysMetaTuning(manager_->needCropTNC16x9_, requests);
}

MfnrTunAfbldTask::MfnrTunAfbldTask(MFNRFrames &mfnr,
				   uint32_t camSysMetaRequestId,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: ImgSysTask(scheduler, id, camSysMetaRequestId, internalRequestId,
		     Feature::Capture_mfnr, manager->ipa_, request->controls()),
	  request_(request), manager_(manager)
{
	afbldF0Tun_.resize(1);
	afbldF1Tun_.resize(1);
	afbldF2Tun_.resize(1);
	afbldF3Tun_.resize(1);
	afbldF4Tun_.resize(1);
	afbldF5Tun_.resize(1);
	afbldF6Tun_.resize(1);
	afbldF0Tun_[0] = mfnr.afbldF0.in.tunbufi[0];
	afbldF1Tun_[0] = mfnr.afbldF1.in.tunbufi[0];
	afbldF2Tun_[0] = mfnr.afbldF2.in.tunbufi[0];
	afbldF3Tun_[0] = mfnr.afbldF3.in.tunbufi[0];
	afbldF4Tun_[0] = mfnr.afbldF4.in.tunbufi[0];
	afbldF5Tun_[0] = mfnr.afbldF5.in.tunbufi[0];
	afbldF6Tun_[0] = mfnr.afbldF6.in.tunbufi[0];
	bssOrder_ = mfnr.bss_order;
	tncso_ = mfnr.afbldF0.tncso;
}

void MfnrTunAfbldTask::run()
{
	ipa::mtkisp7::ImgMetaRequestData request;
	std::vector<ipa::mtkisp7::ImgMetaRequestData> requests;

	manager_->mfnrTun_.fetch(afbldF0Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF1Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF2Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF3Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF4Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF5Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF6Tun_[0]);

	std::map<EStage_T, std::vector<SharedMailBox<InfoFrame>>, std::greater<EStage_T>> stageToTuningMap = {
		{ NSIspTuning::EStage_AFBLD_F6, afbldF6Tun_ },
		{ NSIspTuning::EStage_AFBLD_F5, afbldF5Tun_ },
		{ NSIspTuning::EStage_AFBLD_F4, afbldF4Tun_ },
		{ NSIspTuning::EStage_AFBLD_F3, afbldF3Tun_ },
		{ NSIspTuning::EStage_AFBLD_F2, afbldF2Tun_ },
		{ NSIspTuning::EStage_AFBLD_F1, afbldF1Tun_ },
		{ NSIspTuning::EStage_AFBLD_F0, afbldF0Tun_ }
	};
	auto bssOrder = bssOrder_->get();
	for (auto it = stageToTuningMap.begin(); it != stageToTuningMap.end(); it++) {
		int size_idx = it->first - EStage_AFBLD_F0;
		int frameNumber = internalRequestId_ + bssOrder[bssOrder.size() - 1];

		if (it->first == NSIspTuning::EStage_AFBLD_F0) {
			request = ipa::mtkisp7::ImgMetaRequestData(
				true, it->first,
				it->second[0]->get().buffer()->cookie(),
				tncso_->get().buffer()->cookie(), 0,
				manager_->mfnrSizes_[size_idx],
				manager_->yuvOutput1Size_,
				manager_->yuvOutput2Size_,
				manager_->mfnrSizes_[0], {}, true,
				kInputRawCount - 2, kInputRawCount, true,
				frameNumber);
		} else {
			request = ipa::mtkisp7::ImgMetaRequestData(
				true, it->first,
				it->second[0]->get().buffer()->cookie(),
				0, 0, manager_->mfnrSizes_[size_idx],
				manager_->yuvOutput1Size_,
				manager_->yuvOutput2Size_,
				manager_->mfnrSizes_[0], {}, true,
				kInputRawCount - 2, kInputRawCount, true,
				frameNumber);
		}

		requests.push_back(std::move(request));
	}

	getImgSysMetaTuning(manager_->needCropTNC16x9_, requests);
}

} // namespace libcamera
/* namespace libcamera */
