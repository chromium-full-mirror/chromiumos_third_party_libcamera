/*
 * Copyright (C) 2023, Google Inc.
 *
 * lpnr.h - MtkISP7 ImgSys Low Pass Noise Reduction Tasks
 */

#pragma once

#include <memory>
#include <string>

#include "libcamera/internal/info_frame.h"
#include "libcamera/internal/task_scheduler.h"

#include "imgsys.h"

namespace libcamera {

class XTRTask;
class LpnrDipTask;

struct XtrFrames {
	struct {
		SharedMailBox<InfoFrame> p1Raw;
		SharedMailBox<InfoFrame> xtrTun;
	} in;
	struct {
		SharedMailBox<InfoFrame> xtrStt;
		std::vector<SharedMailBox<InfoFrame>> dipImgi;
	} out;
};

struct LpnrDipFrames {
	struct {
		std::vector<SharedMailBox<InfoFrame>> dipTun;
		std::vector<SharedMailBox<InfoFrame>> dipImgi;
	} in;
};

struct LPNRFrames {
	XtrFrames xtrFrames;
	LpnrDipFrames lpnrDipFrames;

	FrameBuffer *stillOutput;
};

class LpnrTasksManager {
public:
	LpnrTasksManager(ImgSysDevice *imgSys, DmaHeap *dmaHeap);

	int configure(const Size &bayerInputSize, const Size &yuvOutputSize);

	int start();
	int stop();

	int releaseBuffers();

	void makeLPNRFrames(LPNRFrames &lpnr,
			    SharedMailBox<InfoFrame> &p1Raw,
			    FrameBuffer* outputFrame);

	std::tuple<XTRTask *, LpnrDipTask *>
	makeLpnrTasks(LPNRFrames &lpnr, Scheduler* scheduler, const std::string& id,
		      Request* request, ImgSysDevice* imgSys);

private:
	friend class XTRTask;
	friend class LpnrDipTask;

	Size yuvOutputSize_;
	Size bayerInputSize_;

	std::vector<Size> lpnrSizes;

	InfoFramePool lpnrStt_;
	InfoFramePool lpnrTun_;
	std::array<InfoFramePool, 4> lpnr_;

	/* Weak ptr for above pools for easier control */
	std::vector<InfoFramePool *> allBufferPools_;
	std::vector<InfoFramePool *> poolsWritenByCpu_;

	ImgSysDevice *imgSys_;
	DmaHeap *dmaHeap_;
};

class XTRTask : public Task
{
public:
	XTRTask(Scheduler* scheduler, const std::string& id, Request* request,
		ImgSysDevice* imgSys, LPNRFrames &lpnr, LpnrTasksManager* manager);

	void run() override;

private:
	void allocateOutputBuffers();

	XtrFrames frames_;
	ImgSysRequestHelper requestHelper_;

	Request *request_;
	LpnrTasksManager *manager_;
};

class LpnrDipTask : public Task
{
public:
	LpnrDipTask(Scheduler* scheduler, const std::string& id, Request* request,
		 ImgSysDevice* imgSys, LPNRFrames &lpnr, LpnrTasksManager *manager);

	void run() override;

private:
	void allocateOutputBuffers();

	/* Intermediate Frames */
	std::vector<SharedMailBox<InfoFrame>> dipImg3o;
	std::vector<SharedMailBox<InfoFrame>> reci;

	LpnrDipFrames frames_;
	FrameBuffer *stillOutput_;

	ImgSysRequestHelper requestHelper_;
	Request *request_;
	LpnrTasksManager *manager_;
};

} /* namespace libcamera */
