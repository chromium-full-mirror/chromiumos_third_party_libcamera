/*
 * Copyright (C) 2024, Google Inc.
 *
 * mfnr_tun.cpp - MTK MtkISP7 mfnr tuning generator
 */

#include "mfnr_tun.h"

#include <memory>

#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/imgsys/mfnr.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

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
	readAll();
}

void TuningBuffers::readBuffer(uint8_t *dest, size_t length, const char *filename)
{
	FILE *file = nullptr;
	std::string filePath = std::string("/etc/camera/back_settings/") + filename;
	file = fopen(filePath.c_str(), "rb");

	if (!file)
		LOG(MtkISP7, Error) << "Fail to open file " << filePath;

	size_t size = fread(dest, length, 1, file);
	LOG(MtkISP7, Error) << "Read" << filename << " with size " << size;
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

	mfnrSizes.resize(4);
	Size size = bayerInputSize_;

	/* Assign the size to 1/4 of the previous level.
	 * Align to 2 for hardware's requirement */
	for (size_t i = 0; i < mfnrSizes.size(); i++) {
		mfnrSizes[i] = size;
		size.width = (size.width + 3) / 4;
		size.height = (size.height + 3) / 4;
		size.alignUpTo(2, 2);
	}

	allocateBuffers();

	return 0;
}

std::tuple<MfnrTunBfbldBaseTask *, MfnrTunBfbldRefTask *, MfnrTunBfmeTask *, MfnrTunSwmeTask *, MfnrTunDsTask *, MfnrTunDsVbiTask *, MfnrTunMcdsF1Task *, MfnrTunMsbldTask *, MfnrTunAfbldTask *>
MfnrTunManager::makeMfnrTunTasks(MFNRFrames &mfnr,
				 SharedMailBox<AaaIspExchange> &aaaIspExchange,
				 Scheduler *scheduler,
				 const std::string &id, Request *request,
				 uint32_t internalId)
{
	MfnrTunBfbldBaseTask *mfnrTunBfbldBaseTask = new MfnrTunBfbldBaseTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunBfbldRefTask *mfnrTunBfbldRefTask = new MfnrTunBfbldRefTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunBfmeTask *mfnrTunBfmeTask = new MfnrTunBfmeTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunSwmeTask *mfnrTunSwmeTask = new MfnrTunSwmeTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunDsTask *mfnrTunDsTask = new MfnrTunDsTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunDsVbiTask *mfnrTunDsVbiTask = new MfnrTunDsVbiTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunMcdsF1Task *mfnrTunMcdsF1Task = new MfnrTunMcdsF1Task(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunMsbldTask *mfnrTunMsbldTask = new MfnrTunMsbldTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	MfnrTunAfbldTask *mfnrTunAfbldTask = new MfnrTunAfbldTask(
		mfnr, aaaIspExchange, scheduler, id, request, this, internalId);

	return std::make_tuple(mfnrTunBfbldBaseTask, mfnrTunBfbldRefTask, mfnrTunBfmeTask, mfnrTunSwmeTask, mfnrTunDsTask, mfnrTunDsVbiTask, mfnrTunMcdsF1Task, mfnrTunMsbldTask, mfnrTunAfbldTask);
}

MfnrTunBfbldBaseTask::MfnrTunBfbldBaseTask(MFNRFrames &mfnr,
					   SharedMailBox<AaaIspExchange> &aaaIspExchange,
					   Scheduler *scheduler,
					   const std::string &id, Request *request, MfnrTunManager *manager,
					   uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	bfbldBaseTun_ = mfnr.bfbldFrames.in.tunbufi[0];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunBfbldBaseTask::run()
{
	manager_->mfnrTun_.fetch(bfbldBaseTun_);

	fillTuning(bfbldBaseTun_, tuningBuffers.capture_BFBLD_BASE_tunbufi);

	notifyDone();
}

MfnrTunBfbldRefTask::MfnrTunBfbldRefTask(MFNRFrames &mfnr,
					 SharedMailBox<AaaIspExchange> &aaaIspExchange,
					 Scheduler *scheduler,
					 const std::string &id, Request *request, MfnrTunManager *manager,
					 uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	bfbldRefTun_ = mfnr.bfbldFrames.in.tunbufi[1];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunBfbldRefTask::run()
{
	manager_->mfnrTun_.fetch(bfbldRefTun_);

	fillTuning(bfbldRefTun_, tuningBuffers.capture_BFBLD_REF_tunbufi);

	notifyDone();
}

MfnrTunBfmeTask::MfnrTunBfmeTask(MFNRFrames &mfnr,
				 SharedMailBox<AaaIspExchange> &aaaIspExchange,
				 Scheduler *scheduler,
				 const std::string &id, Request *request, MfnrTunManager *manager,
				 uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	bfmeTun_ = mfnr.bfmeFrames.in.tunbufi[0];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunBfmeTask::run()
{
	manager_->mfnrTun_.fetch(bfmeTun_);

	fillTuning(bfmeTun_, tuningBuffers.capture_BFME_tunbufi);

	notifyDone();
}

MfnrTunSwmeTask::MfnrTunSwmeTask([[maybe_unused]] MFNRFrames &mfnr,
				 SharedMailBox<AaaIspExchange> &aaaIspExchange,
				 Scheduler *scheduler,
				 const std::string &id, Request *request, MfnrTunManager *manager,
				 uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	aaaIspExchange_ = aaaIspExchange;
	LOG(MtkISP7, Error) << "mfnr addr = " << static_cast<void *>(&mfnr);
	swmeFrames_ = mfnr.swmeFrames;
}

void MfnrTunSwmeTask::run()
{
	LOG(MtkISP7, Error) << "MfnrTunSwmeTask run";
	auto &out = swmeFrames_.out;
	for (auto i = 0; i < (int)out.db_param.size(); i++) {
		LOG(MtkISP7, Error) << "swmeDbParam_ " << i << " " << out.db_param.size();
		std::shared_ptr<isp_swme_Param> dbParam = manager_->halIsp_->getIspSwmeParam();
		out.db_param[i]->put(dbParam, nullptr);
		LOG(MtkISP7, Error) << "mfnr_.dbParam addr = " << static_cast<void *>(dbParam.get());
	}
	LOG(MtkISP7, Error) << "MfnrTunSwmeTask run debug2";
	notifyDone();
}

MfnrTunDsTask::MfnrTunDsTask(MFNRFrames &mfnr,
			     SharedMailBox<AaaIspExchange> &aaaIspExchange,
			     Scheduler *scheduler,
			     const std::string &id, Request *request, MfnrTunManager *manager,
			     uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	dsTun_ = mfnr.dsFrames.in.tunbufi[0];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunDsTask::run()
{
	manager_->mfnrTun_.fetch(dsTun_);

	fillTuning(dsTun_, tuningBuffers.capture_DS_tunbufi);

	notifyDone();
}

MfnrTunMcdsF1Task::MfnrTunMcdsF1Task(MFNRFrames &mfnr,
				     SharedMailBox<AaaIspExchange> &aaaIspExchange,
				     Scheduler *scheduler,
				     const std::string &id, Request *request, MfnrTunManager *manager,
				     uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	// 0 for BFBLD_BASE Task
	mcdsF1Tun_ = mfnr.mcdsF1Frames.in.tunbufi[0];
	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunMcdsF1Task::run()
{
	manager_->mfnrTun_.fetch(mcdsF1Tun_);

	fillTuning(mcdsF1Tun_, tuningBuffers.capture_MCDS_F1_tunbufi);

	notifyDone();
}

MfnrTunDsVbiTask::MfnrTunDsVbiTask(MFNRFrames &mfnr,
				   SharedMailBox<AaaIspExchange> &aaaIspExchange,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
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

	notifyDone();
}

MfnrTunMsbldTask::MfnrTunMsbldTask(MFNRFrames &mfnr,
				   SharedMailBox<AaaIspExchange> &aaaIspExchange,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	msbldF0Tun_ = mfnr.msbldF0.in.tunbufi[0];
	msbldF1Tun_ = mfnr.msbldF1.in.tunbufi[0];
	msbldF2Tun_ = mfnr.msbldF2.in.tunbufi[0];
	msbldF3Tun_ = mfnr.msbldF3.in.tunbufi[0];
	msbldF4Tun_ = mfnr.msbldF4.in.tunbufi[0];
	msbldF5Tun_ = mfnr.msbldF5.in.tunbufi[0];
	msbldF6Tun_ = mfnr.msbldF6.in.tunbufi[0];

	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunMsbldTask::run()
{
	manager_->mfnrTun_.fetch(msbldF0Tun_);
	manager_->mfnrTun_.fetch(msbldF1Tun_);
	manager_->mfnrTun_.fetch(msbldF2Tun_);
	manager_->mfnrTun_.fetch(msbldF3Tun_);
	manager_->mfnrTun_.fetch(msbldF4Tun_);
	manager_->mfnrTun_.fetch(msbldF5Tun_);
	manager_->mfnrTun_.fetch(msbldF6Tun_);

	fillTuning(msbldF0Tun_, tuningBuffers.capture_MSBLD_F0_tunbufi);
	fillTuning(msbldF1Tun_, tuningBuffers.capture_MSBLD_F1_tunbufi);
	fillTuning(msbldF2Tun_, tuningBuffers.capture_MSBLD_F2_tunbufi);
	fillTuning(msbldF3Tun_, tuningBuffers.capture_MSBLD_F3_tunbufi);
	fillTuning(msbldF4Tun_, tuningBuffers.capture_MSBLD_F4_tunbufi);
	fillTuning(msbldF5Tun_, tuningBuffers.capture_MSBLD_F5_tunbufi);
	fillTuning(msbldF6Tun_, tuningBuffers.capture_MSBLD_F6_tunbufi);

	notifyDone();
}

MfnrTunAfbldTask::MfnrTunAfbldTask(MFNRFrames &mfnr,
				   SharedMailBox<AaaIspExchange> &aaaIspExchange,
				   Scheduler *scheduler,
				   const std::string &id, Request *request, MfnrTunManager *manager,
				   uint32_t internalId)
	: Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	afbldF0Tun_ = mfnr.afbldF0.in.tunbufi[0];
	afbldF1Tun_ = mfnr.afbldF1.in.tunbufi[0];
	afbldF2Tun_ = mfnr.afbldF2.in.tunbufi[0];
	afbldF3Tun_ = mfnr.afbldF3.in.tunbufi[0];
	afbldF4Tun_ = mfnr.afbldF4.in.tunbufi[0];
	afbldF5Tun_ = mfnr.afbldF5.in.tunbufi[0];
	afbldF6Tun_ = mfnr.afbldF6.in.tunbufi[0];

	aaaIspExchange_ = aaaIspExchange;
}

void MfnrTunAfbldTask::run()
{
	manager_->mfnrTun_.fetch(afbldF0Tun_);
	manager_->mfnrTun_.fetch(afbldF1Tun_);
	manager_->mfnrTun_.fetch(afbldF2Tun_);
	manager_->mfnrTun_.fetch(afbldF3Tun_);
	manager_->mfnrTun_.fetch(afbldF4Tun_);
	manager_->mfnrTun_.fetch(afbldF5Tun_);
	manager_->mfnrTun_.fetch(afbldF6Tun_);

	fillTuning(afbldF0Tun_, tuningBuffers.capture_AFBLD_F0_tunbufi);
	fillTuning(afbldF1Tun_, tuningBuffers.capture_AFBLD_F1_tunbufi);
	fillTuning(afbldF2Tun_, tuningBuffers.capture_AFBLD_F2_tunbufi);
	fillTuning(afbldF3Tun_, tuningBuffers.capture_AFBLD_F3_tunbufi);
	fillTuning(afbldF4Tun_, tuningBuffers.capture_AFBLD_F4_tunbufi);
	fillTuning(afbldF5Tun_, tuningBuffers.capture_AFBLD_F5_tunbufi);
	fillTuning(afbldF6Tun_, tuningBuffers.capture_AFBLD_F6_tunbufi);

	notifyDone();
}

} // namespace libcamera
/* namespace libcamera */
