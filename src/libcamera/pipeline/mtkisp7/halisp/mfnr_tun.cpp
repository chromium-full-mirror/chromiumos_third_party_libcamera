/*
 * Copyright (C) 2024, Google Inc.
 *
 * mfnr_tun.cpp - MTK MtkISP7 mfnr tuning generator
 */

#include "mfnr_tun.h"

#include <functional>
#include <memory>
#include <unordered_map>

#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/imgsys/mfnr.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"
#include "tuning_mapping/cam_idx_struct_ext_pub.h"

#include "hal_isp.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

static constexpr Size kTunSize{ 219348, 1 };

/* Reserve the tuning Buffers for debug usage */
class TuningBuffers
{
public:
	TuningBuffers();
	void readBuffer(uint8_t *dest, size_t length, const char *file);
	void readAll();

	uint8_t capture_AFBLD_F0_tunbufi[219348];
	uint8_t capture_AFBLD_F1_tunbufi[219348];
	uint8_t capture_AFBLD_F2_tunbufi[219348];
	uint8_t capture_AFBLD_F3_tunbufi[219348];
	uint8_t capture_AFBLD_F4_tunbufi[219348];
	uint8_t capture_AFBLD_F5_tunbufi[219348];
	uint8_t capture_AFBLD_F6_tunbufi[219348];
	uint8_t capture_BFBLD_BASE_tunbufi[219348];
	uint8_t capture_BFBLD_REF_tunbufi[219348];
	uint8_t capture_BFME_tunbufi[219348];
	uint8_t capture_DS_tunbufi[219348];
	uint8_t capture_DS_VBI_V2_tunbufi[219348];
	uint8_t capture_DS_VBI_V5_tunbufi[219348];
	uint8_t capture_MCDS_F1_tunbufi[219348];
	uint8_t capture_MSBLD_F0_tunbufi[219348];
	uint8_t capture_MSBLD_F1_tunbufi[219348];
	uint8_t capture_MSBLD_F2_tunbufi[219348];
	uint8_t capture_MSBLD_F3_tunbufi[219348];
	uint8_t capture_MSBLD_F4_tunbufi[219348];
	uint8_t capture_MSBLD_F5_tunbufi[219348];
	uint8_t capture_MSBLD_F6_tunbufi[219348];
};

TuningBuffers::TuningBuffers()
{
	//readAll();
}

void TuningBuffers::readBuffer(uint8_t *dest, size_t length, const char *filename)
{
	FILE *file = nullptr;
	std::string filePath = std::string("/etc/camera/back_settings/") + filename;
	file = fopen(filePath.c_str(), "rb");

	if (!file)
		LOG(MtkISP7, Error) << "Fail to open file " << filePath;

	size_t size = fread(dest, length, 1, file);
	LOG(MtkISP7, Info) << "Read" << filename << " with size " << size;
	fclose(file);
}

void TuningBuffers::readAll()
{
	readBuffer(capture_AFBLD_F0_tunbufi, 219348, "capture_AFBLD_F0_tunbufi.bin");
	readBuffer(capture_AFBLD_F1_tunbufi, 219348, "capture_AFBLD_F1_tunbufi.bin");
	readBuffer(capture_AFBLD_F2_tunbufi, 219348, "capture_AFBLD_F2_tunbufi.bin");
	readBuffer(capture_AFBLD_F3_tunbufi, 219348, "capture_AFBLD_F3_tunbufi.bin");
	readBuffer(capture_AFBLD_F4_tunbufi, 219348, "capture_AFBLD_F4_tunbufi.bin");
	readBuffer(capture_AFBLD_F5_tunbufi, 219348, "capture_AFBLD_F5_tunbufi.bin");
	readBuffer(capture_AFBLD_F6_tunbufi, 219348, "capture_AFBLD_F6_tunbufi.bin");
	readBuffer(capture_BFBLD_BASE_tunbufi, 219348, "capture_BFBLD_BASE_tunbufi.bin");
	readBuffer(capture_BFBLD_REF_tunbufi, 219348, "capture_BFBLD_REF_tunbufi.bin");
	readBuffer(capture_BFME_tunbufi, 219348, "capture_BFME_tunbufi.bin");
	readBuffer(capture_DS_VBI_V2_tunbufi, 219348, "capture_DS_VBI_V2_tunbufi.bin");
	readBuffer(capture_DS_VBI_V5_tunbufi, 219348, "capture_DS_VBI_V5_tunbufi.bin");
	readBuffer(capture_DS_tunbufi, 219348, "capture_DS_tunbufi.bin");
	readBuffer(capture_MCDS_F1_tunbufi, 219348, "capture_MCDS_F1_tunbufi.bin");
	readBuffer(capture_MSBLD_F0_tunbufi, 219348, "capture_MSBLD_F0_tunbufi.bin");
	readBuffer(capture_MSBLD_F1_tunbufi, 219348, "capture_MSBLD_F1_tunbufi.bin");
	readBuffer(capture_MSBLD_F2_tunbufi, 219348, "capture_MSBLD_F2_tunbufi.bin");
	readBuffer(capture_MSBLD_F3_tunbufi, 219348, "capture_MSBLD_F3_tunbufi.bin");
	readBuffer(capture_MSBLD_F4_tunbufi, 219348, "capture_MSBLD_F4_tunbufi.bin");
	readBuffer(capture_MSBLD_F5_tunbufi, 219348, "capture_MSBLD_F5_tunbufi.bin");
	readBuffer(capture_MSBLD_F6_tunbufi, 219348, "capture_MSBLD_F6_tunbufi.bin");
}

static TuningBuffers tuningBuffers;

} //namespace

[[maybe_unused]] static void fillTuning(SharedMailBox<InfoFrame> &mailBox, uint8_t *tuning)
{
	assert(tuning);

	InfoFrame &info = mailBox->get();

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
	DmaHeap *dmaHeap, HalIsp *halIsp, OnDeviceTuner *odt)
{
	dmaHeap_ = dmaHeap;
	halIsp_ = halIsp;
	onDeviceTuner_ = odt;
}

void MfnrTunManager::allocateBuffers()
{
	mfnrTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 100);
	mfnrTun_.mmap();
}

void MfnrTunManager::releaseBuffers()
{
	mfnrTun_.unmap();
	mfnrTun_.release();
}

int MfnrTunManager::configure(const Size &bayerInputSize,
			      const Size &yuvOutput1Size, const Size &yuvOutput2Size)
{
	yuvOutput1Size_ = yuvOutput1Size;
	yuvOutput2Size_ = yuvOutput2Size;

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
				 SharedMailBox<AaaIspExchange> &aaaIspExchange,
				 Scheduler *scheduler,
				 const std::string &id, Request *request,
				 uint32_t internalRequestId)
{
	MfnrTunBssTask *mfnrTunBssTask = new MfnrTunBssTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunBfbldTask *mfnrTunBfbldTask = new MfnrTunBfbldTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunBfmeTask *mfnrTunBfmeTask = new MfnrTunBfmeTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunSwmeTask *mfnrTunSwmeTask = new MfnrTunSwmeTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunDsTask *mfnrTunDsTask = new MfnrTunDsTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunDsVbiTask *mfnrTunDsVbiTask = new MfnrTunDsVbiTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunMcdsF1Task *mfnrTunMcdsF1Task = new MfnrTunMcdsF1Task(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunMsbldTask *mfnrTunMsbldTask = new MfnrTunMsbldTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	MfnrTunAfbldTask *mfnrTunAfbldTask = new MfnrTunAfbldTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalRequestId);

	return std::make_tuple(mfnrTunBssTask, mfnrTunBfbldTask, mfnrTunBfmeTask,
			       mfnrTunSwmeTask, mfnrTunDsTask, mfnrTunDsVbiTask, mfnrTunMcdsF1Task,
			       mfnrTunMsbldTask, mfnrTunAfbldTask);
}

MfnrTunBssTask::MfnrTunBssTask([[maybe_unused]] MFNRFrames &mfnr,
			       SharedMailBox<AaaIspExchange> &aaaIspExchange,
			       Scheduler *scheduler,
			       const std::string &id, Request *request, MfnrTunManager *manager,
			       uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	bssFrames_ = mfnr.bssFrames;
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunBssTask::run()
{
	auto &in = bssFrames_.in;
	std::shared_ptr<isp_bss_Param> dbParam = manager_->halIsp_->getIspBssParam();
	in.db_param->put(dbParam, nullptr);

	notifyDone();
}

MfnrTunBfbldTask::MfnrTunBfbldTask(MFNRFrames &mfnr,
				   SharedMailBox<AaaIspExchange> &aaaIspExchange,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	bfbldTun_ = mfnr.bfbldFrames.in.tunbufi;
	bssOrder_ = mfnr.bss_order;
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunBfbldTask::run()
{
	for (auto i = 0; i < kInputRawCount; i++) {
		manager_->mfnrTun_.fetch(bfbldTun_[i]);
	}
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	auto &bssOrder = bssOrder_->get();

	//fillTuning(bfbldRefTun_, tuningBuffers.capture_BFBLD_REF_tunbufi);
	for (auto i = 0; i < kInputRawCount; i++) {
		ImgMetaRequest request = {};
		auto frameNumber = internalRequestId_ + bssOrder[i];
		request = ImgMetaRequest{
			.isCapture = true,
			.isMfnr = true,
			.stage = (i == 0) ? NSIspTuning::EStage_BFBLD_BASE : NSIspTuning::EStage_BFBLD_REF,
			.tuningBuffer = bfbldTun_[i]->get(),
			.statisticsBuffer = {},
			.swHistBuffer = {},
			.inputSize = manager_->mfnrSizes_[0],
			.outputSize = manager_->yuvOutput1Size_,
			.outputSize2 = manager_->yuvOutput2Size_,
			.fullDipSize = manager_->mfnrSizes_[0],
			.reserved = {}
		};

		{
			DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
			manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, frameNumber, manager_->needCropTNC16x9_);
		}
	}

	notifyDone();
}

MfnrTunBfmeTask::MfnrTunBfmeTask(MFNRFrames &mfnr,
				 SharedMailBox<AaaIspExchange> &aaaIspExchange,
				 Scheduler *scheduler,
				 const std::string &id, Request *request, MfnrTunManager *manager,
				 uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	bfmeTun_ = mfnr.bfmeFrames.in.tunbufi[0];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunBfmeTask::run()
{
	manager_->mfnrTun_.fetch(bfmeTun_);

	//fillTuning(bfmeTun_, tuningBuffers.capture_BFME_tunbufi);

	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	ImgMetaRequest request = {};
	request = ImgMetaRequest{
		.isCapture = true,
		.isMfnr = true,
		.stage = NSIspTuning::EStage_BFME,
		.tuningBuffer = bfmeTun_->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->mfnrSizes_[2],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->mfnrSizes_[0],
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, manager_->needCropTNC16x9_);
	}

	notifyDone();
}

MfnrTunSwmeTask::MfnrTunSwmeTask([[maybe_unused]] MFNRFrames &mfnr,
				 SharedMailBox<AaaIspExchange> &aaaIspExchange,
				 Scheduler *scheduler,
				 const std::string &id, Request *request, MfnrTunManager *manager,
				 uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	aaaIspExchange_ = aaaIspExchange;
	swmeFrames_ = mfnr.swmeFrames;
}

void MfnrTunSwmeTask::run()
{
	auto &out = swmeFrames_.out;
	for (auto i = 0; i < (int)out.db_param.size(); i++) {
		std::shared_ptr<isp_swme_Param> dbParam = manager_->halIsp_->getIspSwmeParam();
		out.db_param[i]->put(dbParam, nullptr);
	}

	notifyDone();
}

MfnrTunDsTask::MfnrTunDsTask(MFNRFrames &mfnr,
			     SharedMailBox<AaaIspExchange> &aaaIspExchange,
			     Scheduler *scheduler,
			     const std::string &id, Request *request, MfnrTunManager *manager,
			     uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	dsTun_0 = mfnr.dsFrames.in.tunbufi[0];
	dsTun_1 = mfnr.dsFrames.in.tunbufi[1];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunDsTask::run()
{
	manager_->mfnrTun_.fetch(dsTun_0);
	manager_->mfnrTun_.fetch(dsTun_1);
	//fillTuning(dsTun_0, tuningBuffers.capture_DS_tunbufi);
	//fillTuning(dsTun_1, tuningBuffers.capture_DS_tunbufi);

	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	ImgMetaRequest request = {};
	request = ImgMetaRequest{
		.isCapture = true,
		.isMfnr = true,
		.stage = NSIspTuning::EStage_DS,
		.tuningBuffer = dsTun_0->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->mfnrSizes_[0],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->mfnrSizes_[0],
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, manager_->needCropTNC16x9_);
	}

	request = ImgMetaRequest{
		.isCapture = true,
		.isMfnr = true,
		.stage = NSIspTuning::EStage_DS,
		.tuningBuffer = dsTun_1->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->mfnrSizes_[3],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->mfnrSizes_[0],
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, manager_->needCropTNC16x9_);
	}

	notifyDone();
}

MfnrTunMcdsF1Task::MfnrTunMcdsF1Task(MFNRFrames &mfnr,
				     SharedMailBox<AaaIspExchange> &aaaIspExchange,
				     Scheduler *scheduler,
				     const std::string &id, Request *request, MfnrTunManager *manager,
				     uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	mcdsF1Tun_.resize(kInputRawCount - 1);
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		mcdsF1Tun_[i] = mfnr.mcdsF1Frames.in.tunbufi[i];
	}
	bssOrder_ = mfnr.bss_order;
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunMcdsF1Task::run()
{
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		manager_->mfnrTun_.fetch(mcdsF1Tun_[i]);
		//fillTuning(mcdsF1Tun_[i], tuningBuffers.capture_MCDS_F1_tunbufi);
	}
	auto &bssOrder = bssOrder_->get();
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	for (auto i = 0; i < kInputRawCount - 1; i++) {
		ImgMetaRequest request = {};
		request = ImgMetaRequest{
			.isCapture = true,
			.isMfnr = true,
			.stage = NSIspTuning::EStage_MCDS_F1,
			.tuningBuffer = mcdsF1Tun_[i]->get(),
			.statisticsBuffer = {},
			.swHistBuffer = {},
			.inputSize = manager_->mfnrSizes_[0],
			.outputSize = manager_->yuvOutput1Size_,
			.outputSize2 = manager_->yuvOutput2Size_,
			.fullDipSize = manager_->mfnrSizes_[0],
			.reserved = {}
		};

		{
			int frameNumber = internalRequestId_ + bssOrder[i + 1];
			DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
			manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, frameNumber, manager_->needCropTNC16x9_);
		}
	}
	notifyDone();
}

MfnrTunDsVbiTask::MfnrTunDsVbiTask(MFNRFrames &mfnr,
				   SharedMailBox<AaaIspExchange> &aaaIspExchange,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	dsVbiV2Tun_ = mfnr.dsVbiFramesV2.in.tunbufi[0];
	dsVbiV5Tun_ = mfnr.dsVbiFramesV5.in.tunbufi[0];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunDsVbiTask::run()
{
	manager_->mfnrTun_.fetch(dsVbiV2Tun_);
	manager_->mfnrTun_.fetch(dsVbiV5Tun_);

	fillTuning(dsVbiV2Tun_, tuningBuffers.capture_DS_VBI_V2_tunbufi);
	fillTuning(dsVbiV5Tun_, tuningBuffers.capture_DS_VBI_V5_tunbufi);

	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	ImgMetaRequest request = {};
	request = ImgMetaRequest{
		.isCapture = true,
		.isMfnr = true,
		.stage = NSIspTuning::EStage_DS_VBI_V2,
		.tuningBuffer = dsVbiV2Tun_->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->mfnrSizes_[1],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->mfnrSizes_[0],
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, manager_->needCropTNC16x9_);
	}

	request = ImgMetaRequest{
		.isCapture = true,
		.isMfnr = true,
		.stage = NSIspTuning::EStage_DS_VBI_V5,
		.tuningBuffer = dsVbiV5Tun_->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->mfnrSizes_[4],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->mfnrSizes_[0],
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, manager_->needCropTNC16x9_);
	}

	notifyDone();
}

MfnrTunMsbldTask::MfnrTunMsbldTask(MFNRFrames &mfnr,
				   SharedMailBox<AaaIspExchange> &aaaIspExchange,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
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

	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunMsbldTask::run()
{
	for (auto i = 0; i < kInputRawCount - 2; i++) {
		manager_->mfnrTun_.fetch(msbldF0Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF1Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF2Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF3Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF4Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF5Tun_[i]);
		manager_->mfnrTun_.fetch(msbldF6Tun_[i]);
		//fillTuning(msbldF0Tun_[i], tuningBuffers.capture_MSBLD_F0_tunbufi);
		//fillTuning(msbldF1Tun_[i], tuningBuffers.capture_MSBLD_F1_tunbufi);
		//fillTuning(msbldF2Tun_[i], tuningBuffers.capture_MSBLD_F2_tunbufi);
		//fillTuning(msbldF3Tun_[i], tuningBuffers.capture_MSBLD_F3_tunbufi);
		//fillTuning(msbldF4Tun_[i], tuningBuffers.capture_MSBLD_F4_tunbufi);
		//fillTuning(msbldF5Tun_[i], tuningBuffers.capture_MSBLD_F5_tunbufi);
		//fillTuning(msbldF6Tun_[i], tuningBuffers.capture_MSBLD_F6_tunbufi);
	}

	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	std::map<EStage_T, std::vector<SharedMailBox<InfoFrame>>, std::greater<EStage_T>> stageToTuningMap = {
		{ NSIspTuning::EStage_MSBLD_F0, msbldF0Tun_ },
		{ NSIspTuning::EStage_MSBLD_F1, msbldF1Tun_ },
		{ NSIspTuning::EStage_MSBLD_F2, msbldF2Tun_ },
		{ NSIspTuning::EStage_MSBLD_F3, msbldF3Tun_ },
		{ NSIspTuning::EStage_MSBLD_F4, msbldF4Tun_ },
		{ NSIspTuning::EStage_MSBLD_F5, msbldF5Tun_ },
		{ NSIspTuning::EStage_MSBLD_F6, msbldF6Tun_ }
	};
	for (auto i = 0; i < kInputRawCount - 2; i++) {
		for (auto it = stageToTuningMap.begin(); it != stageToTuningMap.end(); it++) {
			int size_idx = it->first - EStage_MSBLD_F0;
			ImgMetaRequest request = {};
			request = ImgMetaRequest{
				.isCapture = true,
				.isMfnr = true,
				.stage = it->first,
				.tuningBuffer = it->second[i]->get(),
				.statisticsBuffer = {},
				.swHistBuffer = {},
				.inputSize = manager_->mfnrSizes_[size_idx],
				.outputSize = manager_->yuvOutput1Size_,
				.outputSize2 = manager_->yuvOutput2Size_,
				.fullDipSize = manager_->mfnrSizes_[0],
				.tnr_frameIndex = i,
				.tnr_frameTotal = kInputRawCount,
				.reserved = {},
			};
			{
				DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
				manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, manager_->needCropTNC16x9_);
			}
		}
	}

	notifyDone();
}

MfnrTunAfbldTask::MfnrTunAfbldTask(MFNRFrames &mfnr,
				   SharedMailBox<AaaIspExchange> &aaaIspExchange,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalRequestId)
	: Task(scheduler, id), request_(request), internalRequestId_(internalRequestId), manager_(manager)
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

	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunAfbldTask::run()
{
	manager_->mfnrTun_.fetch(afbldF0Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF1Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF2Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF3Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF4Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF5Tun_[0]);
	manager_->mfnrTun_.fetch(afbldF6Tun_[0]);

	//fillTuning(afbldF0Tun_[0], tuningBuffers.capture_AFBLD_F0_tunbufi);
	//fillTuning(afbldF1Tun_[0], tuningBuffers.capture_AFBLD_F1_tunbufi);
	//fillTuning(afbldF2Tun_[0], tuningBuffers.capture_AFBLD_F2_tunbufi);
	//fillTuning(afbldF3Tun_[0], tuningBuffers.capture_AFBLD_F3_tunbufi);
	//fillTuning(afbldF4Tun_[0], tuningBuffers.capture_AFBLD_F4_tunbufi);
	//fillTuning(afbldF5Tun_[0], tuningBuffers.capture_AFBLD_F5_tunbufi);
	//fillTuning(afbldF6Tun_[0], tuningBuffers.capture_AFBLD_F6_tunbufi);
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	std::map<EStage_T, std::vector<SharedMailBox<InfoFrame>>, std::greater<EStage_T>> stageToTuningMap = {
		{ NSIspTuning::EStage_AFBLD_F6, afbldF6Tun_ },
		{ NSIspTuning::EStage_AFBLD_F5, afbldF5Tun_ },
		{ NSIspTuning::EStage_AFBLD_F4, afbldF4Tun_ },
		{ NSIspTuning::EStage_AFBLD_F3, afbldF3Tun_ },
		{ NSIspTuning::EStage_AFBLD_F2, afbldF2Tun_ },
		{ NSIspTuning::EStage_AFBLD_F1, afbldF1Tun_ },
		{ NSIspTuning::EStage_AFBLD_F0, afbldF0Tun_ }
	};

	for (auto it = stageToTuningMap.begin(); it != stageToTuningMap.end(); it++) {
		int size_idx = it->first - EStage_AFBLD_F0;
		ImgMetaRequest request = {};
		request = ImgMetaRequest{
			.isCapture = true,
			.isMfnr = true,
			.stage = it->first,
			.tuningBuffer = it->second[0]->get(),
			.statisticsBuffer = {},
			.swHistBuffer = {},
			.inputSize = manager_->mfnrSizes_[size_idx],
			.outputSize = manager_->yuvOutput1Size_,
			.outputSize2 = manager_->yuvOutput2Size_,
			.fullDipSize = manager_->mfnrSizes_[0],
			.tnr_frameIndex = kInputRawCount - 2,
			.tnr_frameTotal = kInputRawCount,
			.reserved = {},
		};
		{
			DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
			manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, internalRequestId_, manager_->needCropTNC16x9_);
		}
	}

	notifyDone();
}

} // namespace libcamera
/* namespace libcamera */
