/*
 * Copyright (C) 2023, Google Inc.
 *
 * lpnr_tun.h - MTK MtkISP7 LPNR tuning generator
 */

#pragma once

#include <memory>

#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/imgsys/lpnr.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "hal_isp.h"

namespace libcamera {

class DmaHeap;
class PipelineHandler;
class LpnrTunXtrTask;
class LpnrTunDipTask;

class LpnrTunTasksManager {
public:
	LpnrTunTasksManager(
		DmaHeap *dmaHeap, HalIsp *halIsp, OnDeviceTuner *odt);

	int configure(const Size &bayerInputSize,
		      const Size &yuvOutput1Size, const Size &yuvOutput2Size);

	void allocateBuffers();
	void releaseBuffers();


	std::tuple<LpnrTunXtrTask *, LpnrTunDipTask *>
	makeLpnrTunTasks(
			LPNRFrames &lpnr,
			SharedMailBox<AaaIspExchange> &aaaIspExchange,
			Scheduler *scheduler,
			const std::string &id, Request *request,
			uint32_t internalId);

private:
	friend LpnrTunXtrTask;
	friend LpnrTunDipTask;

	Size yuvOutput1Size_;
	Size yuvOutput2Size_;

	Size bayerInputSize_;

	std::vector<Size> lpnrSizes;

	DmaHeap *dmaHeap_;
	InfoFramePool lpnrTun_;

	HalIsp *halIsp_;

	OnDeviceTuner *onDeviceTuner_;
};

class LpnrTunXtrTask : public Task
{
public:
	LpnrTunXtrTask(LPNRFrames &lpnr,
		    SharedMailBox<AaaIspExchange> &aaaIspExchange,
		    Scheduler *scheduler, const std::string &id,
		    Request *request, LpnrTunTasksManager *manager,
		    uint32_t internalId);

	virtual void run() override final;

	SharedMailBox<InfoFrame> xtrTun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;

	Request* request_;
	uint32_t internalId_;

	LpnrTunTasksManager *manager_;
};

class LpnrTunDipTask : public Task
{
public:
	LpnrTunDipTask(LPNRFrames &lpnr,
		       SharedMailBox<AaaIspExchange> &aaaIspExchange,
		       Scheduler *scheduler, const std::string &id,
		       Request *request, LpnrTunTasksManager *manager,
		       uint32_t internalId);

	virtual void run() override final;

	SharedMailBox<bool> highIsoMode_;
	SharedMailBox<InfoFrame> xtrStt_;
	SharedMailBox<InfoFrame> dipTunPq_;
	SharedMailBox<InfoFrame> dipTunY2YPq_;
	std::vector<SharedMailBox<InfoFrame>> dipTun_;

	SharedMailBox<AaaIspExchange> aaaIspExchange_;

	Request* request_;
	uint32_t internalId_;

	LpnrTunTasksManager *manager_;
};

} /* namespace libcamera */
