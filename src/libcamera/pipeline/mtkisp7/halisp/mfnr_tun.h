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
#include "pipeline/mtkisp7/ipa/ipa_delegate.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include "imgsys_task.h"

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
class MtkISP7CameraData;

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
		DmaHeap *dmaHeap, IPADelegate *ipa, OnDeviceTuner *odt);

	int configure(const Size &bayerInputSize,
		      const Size &yuvOutput1Size, const Size &yuvOutput2Size,
		      std::shared_ptr<mtk::isphal::v1::isp_swme_Param> swme,
		      std::shared_ptr<mtk::isphal::v1::isp_bss_Param> bss);

	void allocateBuffers();
	void releaseBuffers();

	std::tuple<MfnrTunBssTask *, MfnrTunBfbldTask *, MfnrTunBfmeTask *,
		   MfnrTunSwmeTask *, MfnrTunDsTask *, MfnrTunDsVbiTask *,
		   MfnrTunMcdsF1Task *, MfnrTunMsbldTask *, MfnrTunAfbldTask *>
	makeMfnrTunTasks(
		MFNRFrames &mfnr,
		uint32_t camSysMetaRequestId,
		Scheduler *scheduler,
		const std::string &id, Request *request,
		uint32_t internalRequestId);

private:
	friend MtkISP7CameraData;

	Size yuvOutput1Size_;
	Size yuvOutput2Size_;

	Size bayerInputSize_;

	std::shared_ptr<mtk::isphal::v1::isp_swme_Param> swme_;
	std::shared_ptr<mtk::isphal::v1::isp_bss_Param> bss_;

	bool needCropTNC16x9_;

	std::vector<Size> mfnrSizes_;

	DmaHeap *dmaHeap_;
	InfoFramePool mfnrTun_;

	IPADelegate *ipa_;

	OnDeviceTuner *onDeviceTuner_;
};

class MfnrTunBssTask : public Task
{
public:
	MfnrTunBssTask(MFNRFrames &mfnr,
		       Scheduler *scheduler, const std::string &id,
		       std::shared_ptr<mtk::isphal::v1::isp_bss_Param> bss);

	virtual void run() override final;

	BssFrames bssFrames_;

	std::shared_ptr<mtk::isphal::v1::isp_bss_Param> bss_;
};

class MfnrTunBfbldTask : public ImgSysTask
{
public:
	MfnrTunBfbldTask(MFNRFrames &mfnr,
			 uint32_t camSysMetaRequestId,
			 Scheduler *scheduler, const std::string &id,
			 Request *request, MfnrTunManager *manager,
			 uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> bfbldTun_;
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;

	MfnrTunManager *manager_;
};

class MfnrTunBfmeTask : public ImgSysTask
{
public:
	MfnrTunBfmeTask(MFNRFrames &mfnr,
			uint32_t camSysMetaRequestId,
			Scheduler *scheduler, const std::string &id,
			Request *request, MfnrTunManager *manager,
			uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> bfmeTun_;
	SharedMailBox<std::vector<int>> bssOrder_;
	SharedMailBox<InfoFrame> tncso_;

	Request *request_;

	MfnrTunManager *manager_;
};

class MfnrTunSwmeTask : public Task
{
public:
	MfnrTunSwmeTask(MFNRFrames &mfnr,
			Scheduler *scheduler, const std::string &id,
			std::shared_ptr<mtk::isphal::v1::isp_swme_Param> swme);

	virtual void run() override final;

	SwmeFrames swmeFrames_;

	std::shared_ptr<mtk::isphal::v1::isp_swme_Param> swme_;
};

class MfnrTunDsTask : public ImgSysTask
{
public:
	MfnrTunDsTask(MFNRFrames &mfnr,
		      uint32_t camSysMetaRequestId,
		      Scheduler *scheduler, const std::string &id,
		      Request *request, MfnrTunManager *manager,
		      uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> dsTun;
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;

	MfnrTunManager *manager_;
};

class MfnrTunDsVbiTask : public ImgSysTask
{
public:
	MfnrTunDsVbiTask(MFNRFrames &mfnr,
			 uint32_t camSysMetaRequestId,
			 Scheduler *scheduler, const std::string &id,
			 Request *request, MfnrTunManager *manager,
			 uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> dsVbiV2Tun_;
	std::vector<SharedMailBox<InfoFrame>> dsVbiV5Tun_;
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;

	MfnrTunManager *manager_;
};

class MfnrTunMcdsF1Task : public ImgSysTask
{
public:
	MfnrTunMcdsF1Task(MFNRFrames &mfnr,
			  uint32_t camSysMetaRequestId,
			  Scheduler *scheduler, const std::string &id,
			  Request *request, MfnrTunManager *manager,
			  uint32_t internalRequestId);

	virtual void run() override final;

	std::vector<SharedMailBox<InfoFrame>> mcdsF1Tun_;
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;

	MfnrTunManager *manager_;
};

class MfnrTunMsbldTask : public ImgSysTask
{
public:
	MfnrTunMsbldTask(MFNRFrames &mfnr,
			 uint32_t camSysMetaRequestId,
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
	SharedMailBox<std::vector<int>> bssOrder_;

	Request *request_;

	MfnrTunManager *manager_;
};

class MfnrTunAfbldTask : public ImgSysTask
{
public:
	MfnrTunAfbldTask(MFNRFrames &mfnr,
			 uint32_t camSysMetaRequestId,
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
	SharedMailBox<std::vector<int>> bssOrder_;

	SharedMailBox<InfoFrame> tncso_;

	Request *request_;

	MfnrTunManager *manager_;
};

} /* namespace libcamera */
