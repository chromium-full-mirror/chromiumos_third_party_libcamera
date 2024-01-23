/*
 * Copyright (C) 2023, Google Inc.
 *
 * mcnr_tun.cpp - MTK MtkISP7 MCNR tuning generator
 */

#include "mcnr_tun.h"

#include <memory>

#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/task_scheduler.h"
#include "mtkcam-interfaces/isphal/IspTuningMeta.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "hal_isp.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

constexpr Size kMeL1Size{144, 108};
constexpr Size kTunSize{219348, 1};
constexpr Size kHistSize{11776, 1};

/* Reserve the tuning Buffers for debug usage */
class TuningBuffers {
public:
	TuningBuffers();
	void readBuffer(uint8_t *dest, size_t length, const char* file);
	void readAll();

	uint8_t HW_DIP_F0_tunbufi[219348];
	uint8_t HW_DIP_F1_tunbufi[219348];
	uint8_t HW_DIP_F2_tunbufi[219348];
	uint8_t HW_DIP_F3_tunbufi[219348];
	uint8_t HW_DIP_F4_tunbufi[219348];
	uint8_t HW_DIP_IDI2_tunbufi[219348];
	uint8_t HW_DIP_IDI_tunbufi[219348];
	uint8_t HW_LTR_F1_tunbufi[219348];
	uint8_t HW_LTR_F4_tunbufi[219348];
	uint8_t HW_LTR_VBI_tunbufi[219348];
	uint8_t HW_WPE_W_F0_tunbufi[219348];
	uint8_t HW_ME_3PASS_MODE_0_tunbufi[219348];
	uint8_t HW_ME_3PASS_MODE_1_tunbufi[219348];
	uint8_t HW_ME_3PASS_MODE_1_me_mili[15552];
	uint8_t HW_TR_F1_tunbufi[219348];
	uint8_t HW_TR_F4_tunbufi[219348];
	uint8_t HW_LTR_ME_L1_tunbufi[219348];
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
	readBuffer(HW_DIP_F0_tunbufi, 219348, "HW_DIP_F0_tunbufi.bin");
	readBuffer(HW_DIP_F1_tunbufi, 219348, "HW_DIP_F1_tunbufi.bin");
	readBuffer(HW_DIP_F2_tunbufi, 219348, "HW_DIP_F2_tunbufi.bin");
	readBuffer(HW_DIP_F3_tunbufi, 219348, "HW_DIP_F3_tunbufi.bin");
	readBuffer(HW_DIP_F4_tunbufi, 219348, "HW_DIP_F4_tunbufi.bin");
	readBuffer(HW_DIP_IDI2_tunbufi, 219348, "HW_DIP_IDI2_tunbufi.bin");
	readBuffer(HW_DIP_IDI_tunbufi, 219348, "HW_DIP_IDI_tunbufi.bin");
	readBuffer(HW_LTR_F1_tunbufi, 219348, "HW_LTR_F1_tunbufi.bin");
	readBuffer(HW_LTR_F4_tunbufi, 219348, "HW_LTR_F4_tunbufi.bin");
	readBuffer(HW_LTR_VBI_tunbufi, 219348, "HW_LTR_VBI_tunbufi.bin");
	readBuffer(HW_WPE_W_F0_tunbufi, 219348, "HW_WPE_W_F0_tunbufi.bin");
	readBuffer(HW_ME_3PASS_MODE_0_tunbufi, 219348, "HW_ME_3PASS_MODE_0_tunbufi.bin");
	readBuffer(HW_ME_3PASS_MODE_1_me_mili, 15552, "HW_ME_3PASS_MODE_1_me_mili.bin");
	readBuffer(HW_ME_3PASS_MODE_1_tunbufi, 219348, "HW_ME_3PASS_MODE_1_tunbufi.bin");
	readBuffer(HW_TR_F1_tunbufi, 219348, "HW_TR_F1_tunbufi.bin");
	readBuffer(HW_TR_F4_tunbufi, 219348, "HW_TR_F4_tunbufi.bin");
	readBuffer(HW_LTR_ME_L1_tunbufi, 219348, "HW_LTR_ME_L1_tunbufi.bin");
}

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

	{
		DmaSyncer syncer(info.buffer()->planes()[0].fd.get());
		memcpy(dest, tuning, length);
	}
}

McnrTunManager::McnrTunManager(
	DmaHeap *dmaHeap, HalIsp *halIsp, OnDeviceTuner *odt)
{
	poolsWritenByCpu_.emplace_back(&fwmeFst_);
	poolsWritenByCpu_.emplace_back(&fwmmFst_);
	poolsWritenByCpu_.emplace_back(&fwmmRst_);
	poolsWritenByCpu_.emplace_back(&fwmmMil_);
	poolsWritenByCpu_.emplace_back(&fwmmGyro_);
	poolsWritenByCpu_.emplace_back(&swHist_);

	poolsWritenByCpu_.emplace_back(&meTun_);
	poolsWritenByCpu_.emplace_back(&wpeTun_);
	poolsWritenByCpu_.emplace_back(&dipTun_);
	poolsWritenByCpu_.emplace_back(&trawTun_);
	poolsWritenByCpu_.emplace_back(&pqdipTun_);

	dmaHeap_ = dmaHeap;
	halIsp_ = halIsp;
	onDeviceTuner_ = odt;

	threadHalIsp_.start();
}

McnrTunManager::~McnrTunManager()
{
	threadHalIsp_.exit();
	threadHalIsp_.wait();
}

void McnrTunManager::allocateBuffers()
{
	fwmeFst_.createBuffers(dmaHeap_, formats::Y8_MTISP, Size{400, 1}, 8);
	fwmmFst_.createBuffers(dmaHeap_, formats::Y8_MTISP, Size{80, 1}, 8);
	fwmmRst_.createBuffers(dmaHeap_, formats::Y8_MTISP, Size{132, 1}, 8);
	fwmmGyro_.createBuffers(dmaHeap_, formats::Y32_MTISP, Size{32, 24}, 8);
	fwmmMil_.createBuffers(dmaHeap_, formats::Y8_MTISP, kMeL1Size, 8);
	dipTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 56);
	pqdipTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 8);
	meTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 6);
	trawTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 56);
	wpeTun_.createBuffers(dmaHeap_, formats::MTFD_MTISP, kTunSize, 6);
	swHist_.createBuffers(dmaHeap_, formats::Y8_MTISP, kHistSize, 12);

	for (auto &pool : poolsWritenByCpu_)
		pool->mmap();
}

void McnrTunManager::releaseBuffers()
{
	for (auto &pool : poolsWritenByCpu_) {
		pool->unmap();
		pool->release();
	}
}

int McnrTunManager::configure(const Size &yuvInputSize, const Size &yuvOutputSize1,
			      const Size &yuvOutputSize2)
{
	yuvOutputSize1_ = yuvOutputSize1;
	yuvOutputSize2_ = yuvOutputSize2;
	yuvInputSize_ = yuvInputSize;
	yuvInputSize2_ = yuvInputSize / 2;

	mcnrSizes.resize(7);
	Size size = yuvInputSize_;

	/* Assign the size to 1/2 of the previous level.
	 * Align to 2 for hardware's requirement */
	for (size_t i = 0; i < mcnrSizes.size(); i++) {
		mcnrSizes[i] = size;
		size.width = (size.width + 1) / 2;
		size.height = (size.height + 1) / 2;
		size.alignUpTo(2, 2);
	}

	allocateBuffers();

	return 0;
}

std::tuple<McnrMeATask *, McnrMeBTask *, McnrTrTask *, McnrDipTask *>
McnrTunManager::makeMcnrTunTasks(
		MCNRFrames &mcnr,
		SharedMailBox<AaaIspExchange> &aaaIspExchange,
		Scheduler *scheduler,
		const std::string &id, Request *request,
		uint32_t internalId)
{
	McnrMeATask *meATunTask =  new McnrMeATask(
			mcnr, aaaIspExchange, scheduler, id, request, this, internalId);

	McnrMeBTask *meBTunTask =  new McnrMeBTask(
			mcnr, aaaIspExchange, scheduler, id, request, this, internalId);

	McnrTrTask *trTunTask =  new McnrTrTask(
			mcnr, aaaIspExchange, scheduler, id, request, this, internalId);

	McnrDipTask *dipTunTask =  new McnrDipTask(
			mcnr, aaaIspExchange, scheduler, id, request, this, internalId);

	meATunTask->moveToThread(&threadHalIsp_);
	meBTunTask->moveToThread(&threadHalIsp_);
	trTunTask->moveToThread(&threadHalIsp_);
	dipTunTask->moveToThread(&threadHalIsp_);

	return std::make_tuple(meATunTask, meBTunTask, trTunTask, dipTunTask);
}

McnrMeATask::McnrMeATask(MCNRFrames &mcnr,
			 SharedMailBox<AaaIspExchange> &aaaIspExchange,
			 Scheduler *scheduler,
			 const std::string &id, Request *request, McnrTunManager *manager,
			 uint32_t internalId)
	:Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	(void) mcnr;

	trMeTun = mcnr.meFrames.in.trMeTun;
	meATun = mcnr.meFrames.in.meATun;

	prevFwMeFst = mcnr.meFrames.in.prevFwMeFst;
	prevFwMmFst = mcnr.meFrames.in.prevFwMmFst;
	prevPrevMeAFst = mcnr.meFrames.in.prevPrevMeAFst;
	prevPrevMeBFst = mcnr.meFrames.in.prevPrevMeBFst;

	fwMeFst = mcnr.meFrames.in.fwMeFst;

	swHist = mcnr.dip1Frames.out.swHist;

	aaaIspExchange_ = aaaIspExchange;
}

void McnrMeATask::run()
{
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();

	manager_->swHist_.fetch(swHist);
	zeroImage(swHist);

	manager_->trawTun_.fetch(trMeTun);

	ImgMetaRequest request = {};
	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_LTR_ME_L1,
		.tuningBuffer = trMeTun->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = Size{ 576, 432 },
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	manager_->meTun_.fetch(meATun);
	manager_->fwmeFst_.fetch(fwMeFst);

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_ME_3PASS_MODE0,
		.tuningBuffer = meATun->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->yuvInputSize_,
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = prevFwMeFst->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_FWMM_MMG_FBFST] = prevFwMmFst->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_HWME_STAT_FST_MD0] = prevPrevMeAFst->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_HWME_STAT_FST_MD1] = prevPrevMeBFst->get();
	request.reserved[mtk::isphal::kISPExtBif_OUT_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		DmaSyncer syncerMeAFst(prevPrevMeAFst->get().buffer()->planes()[0].fd.get());
		DmaSyncer syncerMeBFst(prevPrevMeBFst->get().buffer()->planes()[0].fd.get());

		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	notifyDone();
}

McnrMeBTask::McnrMeBTask(MCNRFrames &mcnr,
			   SharedMailBox<AaaIspExchange> &aaaIspExchange,
			   Scheduler *scheduler,
			   const std::string &id, Request *request, McnrTunManager *manager,
			 uint32_t internalId)
	:Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	meBTun = mcnr.meFrames.in.meBTun;
	meMil = mcnr.meFrames.in.meMil;

	meATun = mcnr.meFrames.in.meATun;
	fwMeFst = mcnr.meFrames.in.fwMeFst;
	fwMmFst = mcnr.meFrames.in.fwMmFst;
	fwMmRst = mcnr.meFrames.in.fwMmRst;
	fwMmGryo = makeMailBox<InfoFrame>();

	meAFst = mcnr.meFrames.out.meAFst;
	meAFmb0 = mcnr.meFrames.out.meAFmb0;

	swHist = mcnr.dip1Frames.out.swHist;

	aaaIspExchange_ = aaaIspExchange;
}

void McnrMeBTask::run()
{
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();

	manager_->meTun_.fetch(meBTun);
	manager_->fwmmFst_.fetch(fwMmFst);
	manager_->fwmmRst_.fetch(fwMmRst);
	manager_->fwmmMil_.fetch(meMil);
	manager_->fwmmGyro_.fetch(fwMmGryo);

	/* TODO: Read the Gyro data from gyro sensor */
	zeroImage(fwMmGryo);

	ImgMetaRequest request = {};
	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_ME_3PASS_MM,
		.tuningBuffer = meBTun->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->yuvInputSize_,
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_HWME_STAT_FST_MD0] = meAFst->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_HWME_STAT_FMB_MD0] = meAFmb0->get();
	request.reserved[mtk::isphal::kISPExtBif_OUT_FWMM_MMG_FBFST] = fwMmFst->get();
	request.reserved[mtk::isphal::kISPExtBif_OUT_FWMM_MMG_RST] = fwMmRst->get();
	request.reserved[mtk::isphal::kISPExtBif_OUT_FWMM_MIL] = meMil->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_GYRO_MV] = fwMmGryo->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		DmaSyncer syncerMil(meMil->get().buffer()->planes()[0].fd.get());
		DmaSyncer syncerMeAFst(meAFst->get().buffer()->planes()[0].fd.get());
		DmaSyncer syncerMeAFmb0(meAFmb0->get().buffer()->planes()[0].fd.get());

		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
		manager_->onDeviceTuner_->tuneMeMM(request_, meBTun);
	}

	request = ImgMetaRequest {
		.isCapture = false,
		.stage = EStage_ME_3PASS_MODE1,
		.tuningBuffer = meBTun->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->yuvInputSize_,
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_HWME_MODE_0_TUN_BUF] = meATun->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_HWME_STAT_FST_MD0] = meAFst->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_FWMM_MMG_RST] = fwMmRst->get();
	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		DmaSyncer syncerMeAFst(meAFst->get().buffer()->planes()[0].fd.get());

		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	notifyDone();
}

McnrTrTask::McnrTrTask(MCNRFrames &mcnr,
			   SharedMailBox<AaaIspExchange> &aaaIspExchange,
			   Scheduler *scheduler,
			   const std::string &id, Request *request, McnrTunManager *manager,
			 uint32_t internalId)
	:Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{
	trTunF1 = mcnr.trFrames.in.trTunF1;
	trTunF4 = mcnr.trFrames.in.trTunF4;

	swHist = mcnr.dip1Frames.out.swHist;

	aaaIspExchange_ = aaaIspExchange;
}

void McnrTrTask::run()
{
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	ImgMetaRequest request = {};

	manager_->trawTun_.fetch(trTunF1);

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_TR_Y2Y_F1,
		.tuningBuffer = trTunF1->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[1],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	manager_->trawTun_.fetch(trTunF4);

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_TR_Y2Y_F4,
		.tuningBuffer = trTunF4->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[4],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	notifyDone();
}

McnrDipTask::McnrDipTask(MCNRFrames &mcnr,
			   SharedMailBox<AaaIspExchange> &aaaIspExchange,
			   Scheduler *scheduler,
			   const std::string &id, Request *request, McnrTunManager *manager,
			 uint32_t internalId)
	:Task(scheduler, id), request_(request), internalId_(internalId), manager_(manager)
{

	fwMeFst = mcnr.meFrames.in.fwMeFst;
	trawStt = mcnr.trFrames.out.trawStt;

	ltrTunF1 = mcnr.dip1Frames.in.ltrTunF1;
	ltrTunF4 = mcnr.dip1Frames.in.ltrTunF4;
	ltrTunVbi = mcnr.dip1Frames.in.ltrTunVbi;
	wpeTun = mcnr.dip1Frames.in.wpeTun;
	dipTun = mcnr.dip1Frames.in.dipTun;

	swHist = mcnr.dip1Frames.out.swHist;

	aaaIspExchange_ = aaaIspExchange;
}

void McnrDipTask::run()
{
	AaaIspExchange *aaaIspExchange = &aaaIspExchange_->get();
	ImgMetaRequest request = {};

	manager_->trawTun_.fetch(ltrTunF1);

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_WPE_LTR_Y2Y_F1,
		.tuningBuffer = ltrTunF1->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[1],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	manager_->trawTun_.fetch(ltrTunF4);

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_LTR_Y2Y_F4,
		.tuningBuffer = ltrTunF4->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[4],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	manager_->trawTun_.fetch(ltrTunVbi);

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_LTR_VBI,
		.tuningBuffer = ltrTunVbi->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[3],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	manager_->trawTun_.fetch(wpeTun);

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_WPE_WghtMap,
		.tuningBuffer = wpeTun->get(),
		.statisticsBuffer = {},
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->yuvInputSize_,
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	// Sync here since all the following DIP stages will access it
	DmaSyncer syncerStt(trawStt->get().buffer()->planes()[0].fd.get());

	for (size_t i = 0; i < dipTun.size(); i++) {
		manager_->dipTun_.fetch(dipTun[i]);
	}

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_P2_IDI,
		.tuningBuffer = dipTun[6]->get(),
		.statisticsBuffer = trawStt->get(),
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[6],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_P2_MS_F_SMALL,
		.tuningBuffer = dipTun[5]->get(),
		.statisticsBuffer = trawStt->get(),
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[5],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_P2_MS_F4,
		.tuningBuffer = dipTun[4]->get(),
		.statisticsBuffer = trawStt->get(),
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[4],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_P2_MS_F3,
		.tuningBuffer = dipTun[3]->get(),
		.statisticsBuffer = trawStt->get(),
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[3],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_P2_MS_F2,
		.tuningBuffer = dipTun[2]->get(),
		.statisticsBuffer = trawStt->get(),
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[2],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_P2_MS_F1,
		.tuningBuffer = dipTun[1]->get(),
		.statisticsBuffer = trawStt->get(),
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[1],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	request = ImgMetaRequest{
		.isCapture = false,
		.stage = EStage_WPE_P2_PQDIP_MS_F0,
		.tuningBuffer = dipTun[0]->get(),
		.statisticsBuffer = trawStt->get(),
		.swHistBuffer = swHist->get(),
		.inputSize = manager_->mcnrSizes[0],
		.outputSize = manager_->yuvOutputSize1_,
		.outputSize2 = manager_->yuvOutputSize2_,
		.fullDipSize = manager_->yuvInputSize_,
		.reserved = {}
	};

	request.reserved[mtk::isphal::kISPExtBif_IN_FWME_FST] = fwMeFst->get();

	{
		DmaSyncer syncer(request.tuningBuffer.buffer()->planes()[0].fd.get());
		manager_->halIsp_->getImgSysMetaTuning(aaaIspExchange, request, request_);
	}

	notifyDone();
}

} /* namespace libcamera */
