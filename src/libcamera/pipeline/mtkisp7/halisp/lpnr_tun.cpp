/*
 * Copyright (C) 2023, Google Inc.
 *
 * lpnr_tun.cpp - MTK MtkISP7 LPNR tuning generator
 */

#include "lpnr_tun.h"

#include <memory>

#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/task_scheduler.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "hal_isp.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

static constexpr Size kTunSize{219348, 1};

/* Reserve the tuning Buffers for debug usage */
class TuningBuffers {
public:
	TuningBuffers();
	void readBuffer(uint8_t *dest, size_t length, const char* file);
	void readAll();

	uint8_t capture_TR_R2Y_tunbufi[219348];
	uint8_t capture_P2_MS_F3_tunbufi[219348];
	uint8_t capture_P2_MS_F2_tunbufi[219348];
	uint8_t capture_P2_MS_F1_tunbufi[219348];
	uint8_t capture_P2_MS_F0_H_tunbufi[219348];
	uint8_t capture_P2_Y2Y_PQ_DIP_tunbufi[219348];
	uint8_t capture_P2_MS_F0_PQ_DIP_tunbufi[219348];
};

TuningBuffers::TuningBuffers()
{
	readAll();
}

void TuningBuffers::readBuffer(uint8_t *dest, size_t length, const char* filename)
{
	FILE *file = nullptr;
	std::string filePath = std::string("/etc/camera/back_settings/") + filename;
	file = fopen(filePath.c_str(), "rb");

	if (!file)
		LOG(MtkISP7, Error) << "Fail to open file " << filePath;

	size_t size = fread(dest, length , 1, file);
	LOG(MtkISP7, Error) << "Read" << filename << " with size " << size;
	fclose(file);
}

void TuningBuffers::readAll()
{
	readBuffer(capture_TR_R2Y_tunbufi, 219348, "capture_TR_R2Y_tunbufi.bin");
	readBuffer(capture_P2_MS_F3_tunbufi, 219348, "capture_P2_MS_F3_tunbufi.bin");
	readBuffer(capture_P2_MS_F2_tunbufi, 219348, "capture_P2_MS_F2_tunbufi.bin");
	readBuffer(capture_P2_MS_F1_tunbufi, 219348, "capture_P2_MS_F1_tunbufi.bin");
	readBuffer(capture_P2_MS_F0_H_tunbufi, 219348, "capture_P2_MS_F0_H_tunbufi.bin");
	readBuffer(capture_P2_Y2Y_PQ_DIP_tunbufi, 219348, "capture_P2_Y2Y_PQ_DIP_tunbufi.bin");
	readBuffer(capture_P2_MS_F0_PQ_DIP_tunbufi, 219348, "capture_P2_MS_F0_PQ_DIP_tunbufi.bin");
}

static TuningBuffers tuningBuffers;

} //namespace

[[maybe_unused]]static void fillTuning(SharedMailBox<InfoFrame> &mailBox, uint8_t* tuning)
{
	assert(tuning);

	InfoFrame &info = mailBox->get();

	void *dest = info.address(0);
	size_t length = info.buffer()->planes()[0].length;

	assert(dest);
	assert(mailBox->valid());

	libcamera::DmaHeap::sync(
			info.buffer()->planes()[0].fd.get(),
			libcamera::DmaHeap::Start,
			libcamera::DmaHeap::SyncReadWrite);

	memcpy(dest, tuning, length);

	libcamera::DmaHeap::sync(
			info.buffer()->planes()[0].fd.get(),
			libcamera::DmaHeap::End,
			libcamera::DmaHeap::SyncReadWrite);
}

LpnrTunTasksManager::LpnrTunTasksManager(
	DmaHeap *dmaHeap, HalIsp *halIsp, OnDeviceTuner *odt)
{
	dmaHeap_ = dmaHeap;
	halIsp_ = halIsp;
	onDeviceTuner_ = odt;
}

void LpnrTunTasksManager::allocateBuffers()
{
	lpnrTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 12);
	lpnrTun_.mmap();
}

void LpnrTunTasksManager::releaseBuffers()
{
	lpnrTun_.unmap();
	lpnrTun_.release();
}

int LpnrTunTasksManager::configure(const Size &bayerInputSize,
				   const Size &yuvOutput1Size, const Size &yuvOutput2Size)
{
	yuvOutput1Size_ = yuvOutput1Size;
	yuvOutput2Size_ = yuvOutput2Size;

	bayerInputSize_ = bayerInputSize;

	lpnrSizes.resize(4);
	Size size = bayerInputSize_;

	/* Assign the size to 1/4 of the previous level.
	 * Align to 2 for hardware's requirement */
	for (size_t i = 0; i < lpnrSizes.size(); i++) {
		lpnrSizes[i] = size;
		size.width = (size.width + 3) / 4;
		size.height = (size.height + 3) / 4;
		size.alignUpTo(2, 2);
	}

	allocateBuffers();

	return 0;
}

std::tuple<LpnrTunXtrTask *, LpnrTunDipTask *>
LpnrTunTasksManager::makeLpnrTunTasks(LPNRFrames &lpnr,
				      SharedMailBox<AaaIspExchange> &aaaIspExchange,
				      Scheduler *scheduler,
				      const std::string &id, Request *request,
				      uint32_t internalId)
{
	LpnrTunXtrTask *lpnrTunXtrTask =  new LpnrTunXtrTask(
			lpnr, aaaIspExchange, scheduler, id, request, this, internalId);

	LpnrTunDipTask *lpnrTunDipTask =  new LpnrTunDipTask(
			lpnr, aaaIspExchange, scheduler, id, request, this, internalId);

	return std::make_tuple(lpnrTunXtrTask, lpnrTunDipTask);
}

LpnrTunXtrTask::LpnrTunXtrTask(LPNRFrames &lpnr,
			 SharedMailBox<AaaIspExchange> &aaaIspExchange,
			 Scheduler *scheduler,
			 const std::string &id, Request *request, LpnrTunTasksManager *manager,
			 uint32_t internalId)
	:Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	xtrTun_ = lpnr.xtrFrames.in.xtrTun;
	aaaIspExchange_ = aaaIspExchange;
}

void LpnrTunXtrTask::run()
{
	manager_->lpnrTun_.fetch(xtrTun_);
	InfoFrame &frame = xtrTun_->get();

	ImgMetaRequest request = {
		.isCapture = true,
		.stage = EStage_TR_R2Y,
		.tuningBuffer = frame,
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->bayerInputSize_,
		.outputSize = manager_->bayerInputSize_,
		.outputSize2 = {},
		.fullDipSize = manager_->lpnrSizes[0],
		.reserved = {}
	};

	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);

	notifyDone();
}

LpnrTunDipTask::LpnrTunDipTask(LPNRFrames &lpnr,
			   SharedMailBox<AaaIspExchange> &aaaIspExchange,
			   Scheduler *scheduler,
			   const std::string &id, Request *request, LpnrTunTasksManager *manager,
			 uint32_t internalId)
	:Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	highIsoMode_ = lpnr.lpnrDipFrames.in.highIsoMode;
	xtrStt_ = lpnr.xtrFrames.out.xtrStt;
	dipTunPq_ = lpnr.lpnrDipFrames.in.dipTunPq;
	dipTunY2YPq_ = lpnr.lpnrDipFrames.in.dipTunY2YPq;
	dipTun_ = lpnr.lpnrDipFrames.in.dipTun;

	aaaIspExchange_ = aaaIspExchange;
}

void LpnrTunDipTask::run()
{
	ImgMetaRequest request = {};
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();

	// TODO: Read the threshold from Tuning Provider
	int32_t threshold = manager_->halIsp_->getLpnrIsoThreshold(aaaIspExchange);
	int32_t sensorSensitivity = aaaIspExchange->aaaResult.ae_result.sensor_sensitivity;

	bool highIsoMode = (sensorSensitivity > threshold) ? true : false;
	if (manager_->onDeviceTuner_->isLowIsoLpnrEnforced()) {
		highIsoMode = false;
	}

	highIsoMode_->put(highIsoMode, nullptr);

	manager_->lpnrTun_.fetch(dipTun_[3]);

	request = ImgMetaRequest{
		.isCapture = true,
		.stage = EStage_P2_MS_F3,
		.tuningBuffer = dipTun_[3]->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->lpnrSizes[3],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->lpnrSizes[0],
		.reserved = {}
	};

	manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);

	manager_->lpnrTun_.fetch(dipTun_[2]);

	request = ImgMetaRequest{
		.isCapture = true,
		.stage = EStage_P2_MS_F2,
		.tuningBuffer = dipTun_[2]->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->lpnrSizes[2],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->lpnrSizes[0],
		.reserved = {}
	};

	manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);

	manager_->lpnrTun_.fetch(dipTun_[1]);

	request = ImgMetaRequest{
		.isCapture = true,
		.stage = EStage_P2_MS_F1,
		.tuningBuffer = dipTun_[1]->get(),
		.statisticsBuffer = {},
		.swHistBuffer = {},
		.inputSize = manager_->lpnrSizes[1],
		.outputSize = manager_->yuvOutput1Size_,
		.outputSize2 = manager_->yuvOutput2Size_,
		.fullDipSize = manager_->lpnrSizes[0],
		.reserved = {}
	};

	manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);

	if (highIsoMode) {
		manager_->lpnrTun_.fetch(dipTun_[0]);

		request = ImgMetaRequest{
			.isCapture = true,
			.stage = EStage_P2_MS_F0_H,
			.tuningBuffer = dipTun_[0]->get(),
			.statisticsBuffer = {},
			.swHistBuffer = {},
			.inputSize = manager_->lpnrSizes[0],
			.outputSize = manager_->yuvOutput1Size_,
			.outputSize2 = manager_->yuvOutput2Size_,
			.fullDipSize = manager_->lpnrSizes[0],
			.reserved = {}
		};

		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);

		manager_->lpnrTun_.fetch(dipTunY2YPq_);

		request = ImgMetaRequest{
			.isCapture = true,
			.stage = EStage_P2_Y2Y_PQ_DIP,
			.tuningBuffer = dipTunY2YPq_->get(),
			.statisticsBuffer = xtrStt_->get(),
			.swHistBuffer = {},
			.inputSize = manager_->lpnrSizes[0],
			.outputSize = manager_->yuvOutput1Size_,
			.outputSize2 = manager_->yuvOutput2Size_,
			.fullDipSize = manager_->lpnrSizes[0],
			.reserved = {}
		};

		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	} else {
		manager_->lpnrTun_.fetch(dipTunPq_);

		request = ImgMetaRequest{
			.isCapture = true,
			.stage = EStage_P2_MS_F0_PQ_DIP,
			.tuningBuffer = dipTunPq_->get(),
			.statisticsBuffer = xtrStt_->get(),
			.swHistBuffer = {},
			.inputSize = manager_->lpnrSizes[0],
			.outputSize = manager_->yuvOutput1Size_,
			.outputSize2 = manager_->yuvOutput2Size_,
			.fullDipSize = manager_->lpnrSizes[0],
			.reserved = {}
		};

		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	notifyDone();
}

} /* namespace libcamera */
