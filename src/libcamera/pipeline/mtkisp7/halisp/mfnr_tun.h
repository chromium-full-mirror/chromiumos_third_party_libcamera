/*
 * Copyright (C) 2024, Google Inc.
 *
 * mfnr_tun.h - MTK MtkISP7 MFNR tuning generator
 */

#pragma once

#include <memory>
#include <vector>

#include <libcamera/base/signal.h>
#include <libcamera/base/thread.h>

#include <libcamera/geometry.h>

#include "libcamera/internal/task_scheduler.h"

#include "pipeline/mtkisp7/imgsys/mfnr.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "hal_isp.h"

namespace libcamera {

class DmaHeap;
class PipelineHandler;

class MfnrTunBssTask;
class MfnrTunBfbldTask;
class MfnrTunBfmeTask;
class MfnrTunSwmeTask;
class MfnrTunDsTask;
class MfnrTunDsVbiTask;
class MfnrTunMcdsF1Task;
class MfnrTunMsbldTask;
class MfnrTunAfbldTask;

class MfnrTunManager
{
	friend class MfnrTunBssTask;
	friend class MfnrTunBfbldTask;
	friend class MfnrTunBfmeTask;
	friend class MfnrTunSwmeTask;
	friend class MfnrTunDsTask;
	friend class MfnrTunDsVbiTask;
	friend class MfnrTunMcdsF1Task;
	friend class MfnrTunMsbldTask;
	friend class MfnrTunAfbldTask;

public:
	MfnrTunManager(
		DmaHeap *dmaHeap, HalIsp *halIsp, OnDeviceTuner *odt);

	int configure(const Size &bayerInputSize,
		      const Size &yuvOutput1Size, const Size &yuvOutput2Size);

	void allocateBuffers();
	void releaseBuffers();

	std::tuple<MfnrTunBssTask *, MfnrTunBfbldTask *, MfnrTunBfmeTask *,
		   MfnrTunSwmeTask *, MfnrTunDsTask *, MfnrTunDsVbiTask *,
		   MfnrTunMcdsF1Task *, MfnrTunMsbldTask *, MfnrTunAfbldTask *>
	makeMfnrTunTasks(
		MFNRFrames &mfnr,
		SharedMailBox<AaaIspExchange> &aaaIspExchange,
		Scheduler *scheduler,
		const std::string &id, Request *request,
		uint32_t internalRequestId);

private:
	Size yuvOutput1Size_;
	Size yuvOutput2Size_;

	Size bayerInputSize_;

	bool needCropTNC16x9_;

	std::vector<Size> mfnrSizes_;

	DmaHeap *dmaHeap_;
	InfoFramePool mfnrTun_;

	HalIsp *halIsp_;

	OnDeviceTuner *onDeviceTuner_;
};

class MfnrTunBssTask : public Task
{
public:
	MfnrTunBssTask(MFNRFrames &mfnr,
		       SharedMailBox<AaaIspExchange> &aaaIspExchange,
		       Scheduler *scheduler, const std::string &id,
		       Request *request, MfnrTunManager *manager,
		       uint32_t internalRequestId);

	virtual void run() override final;

	SharedMailBox<InfoFrame> bfbldBaseTun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
	BssFrames bssFrames_;
};

class MfnrTunBfbldTask : public Task
{
public:
	MfnrTunBfbldTask(MFNRFrames &mfnr,
			 SharedMailBox<AaaIspExchange> &aaaIspExchange,
			 Scheduler *scheduler, const std::string &id,
			 Request *request, MfnrTunManager *manager,
			 uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> bfbldTun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
};

class MfnrTunBfmeTask : public Task
{
public:
	MfnrTunBfmeTask(MFNRFrames &mfnr,
			SharedMailBox<AaaIspExchange> &aaaIspExchange,
			Scheduler *scheduler, const std::string &id,
			Request *request, MfnrTunManager *manager,
			uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> bfmeTun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;
	SharedMailBox<std::vector<int>> bssOrder_;
	SharedMailBox<InfoFrame> tncso_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
};

class MfnrTunSwmeTask : public Task
{
public:
	MfnrTunSwmeTask(MFNRFrames &mfnr,
			SharedMailBox<AaaIspExchange> &aaaIspExchange,
			Scheduler *scheduler, const std::string &id,
			Request *request, MfnrTunManager *manager,
			uint32_t internalRequestId);

	virtual void run() override final;

	SharedMailBox<InfoFrame> bfmeTun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
	SwmeFrames swmeFrames_;
};

class MfnrTunDsTask : public Task
{
public:
	MfnrTunDsTask(MFNRFrames &mfnr,
		      SharedMailBox<AaaIspExchange> &aaaIspExchange,
		      Scheduler *scheduler, const std::string &id,
		      Request *request, MfnrTunManager *manager,
		      uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> dsTun;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
};

class MfnrTunDsVbiTask : public Task
{
public:
	MfnrTunDsVbiTask(MFNRFrames &mfnr,
			 SharedMailBox<AaaIspExchange> &aaaIspExchange,
			 Scheduler *scheduler, const std::string &id,
			 Request *request, MfnrTunManager *manager,
			 uint32_t internalRequestId);

	virtual void run() override final;

	SharedMailBox<InfoFrame> dsVbiV2Tun_;
	SharedMailBox<InfoFrame> dsVbiV5Tun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
};

class MfnrTunMcdsF1Task : public Task
{
public:
	MfnrTunMcdsF1Task(MFNRFrames &mfnr,
			  SharedMailBox<AaaIspExchange> &aaaIspExchange,
			  Scheduler *scheduler, const std::string &id,
			  Request *request, MfnrTunManager *manager,
			  uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> mcdsF1Tun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
};

class MfnrTunMsbldTask : public Task
{
public:
	MfnrTunMsbldTask(MFNRFrames &mfnr,
			 SharedMailBox<AaaIspExchange> &aaaIspExchange,
			 Scheduler *scheduler, const std::string &id,
			 Request *request, MfnrTunManager *manager,
			 uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> msbldF0Tun_;
	std::vector<SharedMailBox<InfoFrame>> msbldF1Tun_;
	std::vector<SharedMailBox<InfoFrame>> msbldF2Tun_;
	std::vector<SharedMailBox<InfoFrame>> msbldF3Tun_;
	std::vector<SharedMailBox<InfoFrame>> msbldF4Tun_;
	std::vector<SharedMailBox<InfoFrame>> msbldF5Tun_;
	std::vector<SharedMailBox<InfoFrame>> msbldF6Tun_;

	SharedMailBox<AaaIspExchange> aaaIspExchange_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
};

class MfnrTunAfbldTask : public Task
{
public:
	MfnrTunAfbldTask(MFNRFrames &mfnr,
			 SharedMailBox<AaaIspExchange> &aaaIspExchange,
			 Scheduler *scheduler, const std::string &id,
			 Request *request, MfnrTunManager *manager,
			 uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> afbldF0Tun_;
	std::vector<SharedMailBox<InfoFrame>> afbldF1Tun_;
	std::vector<SharedMailBox<InfoFrame>> afbldF2Tun_;
	std::vector<SharedMailBox<InfoFrame>> afbldF3Tun_;
	std::vector<SharedMailBox<InfoFrame>> afbldF4Tun_;
	std::vector<SharedMailBox<InfoFrame>> afbldF5Tun_;
	std::vector<SharedMailBox<InfoFrame>> afbldF6Tun_;

	SharedMailBox<InfoFrame> xtrTun_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;

	Request *request_;
	uint32_t internalRequestId_;

	MfnrTunManager *manager_;
};

} /* namespace libcamera */
