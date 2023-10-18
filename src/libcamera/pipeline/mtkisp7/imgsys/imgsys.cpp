/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * imgsys.cpp - MTK MtkISP7 ImgSys device
 */

#include "imgsys.h"

#include <numeric>

#include <dlfcn.h>
#include <sys/ioctl.h>

#include "linux/mtkisp7/mtk_imgsys.h"

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>

#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pools.h"

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

Rectangle ImgSysDevice::getCrop(Size inSize, Size outSize)
{
	/* 4:3 */
	if (outSize.width * 3 == outSize.height * 4)
		return {0, 0, inSize.width, inSize.height};

	/* 16:9 */
	unsigned int height = inSize.width * 9 / 16;
	int y = (inSize.height - height) / 2;

	return {0, y, inSize.width, height};
}

ImgSysDevice::ImgSysDevice(OnDeviceTuner *odt) :
	onDeviceTuner_(odt), backEndLibrary_(nullptr)
{}

int ImgSysDevice::init(MediaDevice *media, DmaHeap *dmaHeap)
{
	media_ = media;
	dmaHeap_ = dmaHeap;

	const std::string hubName = "MTK-ISP-DIP-V4L2";
	mtkIspDip_ = V4L2Subdevice::fromEntityName(media_, hubName);

	if (!mtkIspDip_ || mtkIspDip_->open())
		return -ENODEV;

	/* The two entities shouldn't be enabled */
	MediaEntity *feo = media_->getEntityByName(hubName + " FEO Output");
	MediaEntity *metai = media_->getEntityByName(hubName + " METAI Input");

	/* The four entities would be configured differently */
	MediaEntity *sigdevNorm = media_->getEntityByName(hubName + " SIGDEVN");
	MediaEntity *tuningMeta = media_->getEntityByName(hubName + " Tuning");
	MediaEntity *ctrlMeta = media_->getEntityByName(hubName + " CtrlMeta");
	MediaEntity *sigdev = media_->getEntityByName(hubName + " Single Device");

	media_->disableLinks();

	/* Helper function to configure video nodes */
	auto configureVideo = [](V4L2VideoDevice *device,
				 const PixelFormat &pixelFormat, Size size) {
		V4L2DeviceFormat format;
		format.size = size;
		format.fourcc = device->toV4L2PixelFormat(pixelFormat);

		device->setFormat(&format);
	};

	/* Find video devices, configure and save them in allVideoDevices_*/
	for (const auto &entity : media_->entities()) {
		if (entity->type() != MediaEntity::Type::V4L2VideoDevice ||
		    entity == metai || entity == feo)
			continue;

		// Enable the only link of video devices to/from hub
		MediaLink *link = entity->pads()[0]->links()[0];
		link->setEnabled(true);

		std::unique_ptr<V4L2VideoDevice> videoDev =
			std::make_unique<V4L2VideoDevice>(entity);

		if (videoDev->open())
			return -ENODEV;

		if (entity == sigdevNorm) {
			// Weak ptr for sigdevNorm for easier queuing requests
			sigdevNorm_ = videoDev.get();
			configureVideo(videoDev.get(), formats::MTSR_MTISP, { 640, 480 });
		}
		else if (entity == sigdev)
			configureVideo(videoDev.get(), formats::MTFS_MTISP, { 640, 480 });
		else if (entity == ctrlMeta || entity == tuningMeta)
			configureVideo(videoDev.get(), formats::MTFD_MTISP, { 38408, 1 });
		else
			configureVideo(videoDev.get(), formats::MTFD_MTISP, { 640, 480 });

		// All video devices for easier streamOn/Off
		allVideoDevices_.emplace_back(std::move(videoDev));
	}

	std::vector<UniqueFD> requests;
	media_->allocateRequests(16, requests);
	mediaRequestPool_.setData(requests);

	/* Sync token starts from 1 */
	std::vector<BasicContainer<uint32_t>> syncs;
	for (uint32_t i = 1; i < 200; i++)
		syncs.emplace_back(i);

	syncPool_.setData(syncs);

	sigdevNorm_->requestBufferReady.connect(this, &ImgSysDevice::bufferReady);

	// todo: Do streamOn/streamOff in start/stop when the backend library
	// is moved to scp in driver
	if (startImgSysBackend()) {
		LOG(MtkISP7, Error) << "Fail to start ImgSys backend";
		return -EBUSY;
	}

	for (auto &videoDev : allVideoDevices_) {
		videoDev->importBuffers(16);
		videoDev->streamOn();
	}

	return 0;
}

int ImgSysDevice::queueRequest(Request *request)
{
	SharedMailBox<InfoFrame> ctrlMeta = makeMailBox<InfoFrame>();
	ctrlMetaPool_.fetch(ctrlMeta);
	InfoFrame &infoCtrl = ctrlMeta->get();

	SharedMailBox<InfoFrame> singleDevNorm = makeMailBox<InfoFrame>();
	descPool_.fetch(singleDevNorm);
	InfoFrame &infoDesc = singleDevNorm->get();

	FrameBuffer *singleDev = infoDesc.buffer();
	int mediaRequest = mediaRequestPool_.get();

	request->sdRequest->fillRequestBuffer(infoCtrl, infoDesc, mediaRequest);
	onDeviceTuner_->tuneImgsysMetadata(request->sdRequest, infoCtrl);

	int ret = sigdevNorm_->queueBuffer(singleDev, mediaRequest);
	ret |= media_->queueRequest(mediaRequest);

	if (ret) {
		LOG(MtkISP7, Error) << "Fail to queue request";
		return ret;
	}

	pendingRequests_.push_back({request, mediaRequest, ctrlMeta, singleDevNorm});
	return 0;
}

int ImgSysDevice::claimCompletedRequest(Request *request)
{
	for (auto iter = completedRequests_.begin();
	     iter != completedRequests_.end(); ++iter) {
		if (*iter == request) {
			completedRequests_.erase(iter);
			return 0;
		}
	}

	return -EINVAL;
}

void ImgSysDevice::bufferReady(std::pair<FrameBuffer *, int> pair)
{
	auto[buffer, mediaRequest] = pair;
	ASSERT(buffer);

	bool foundRequest = false;
	for (auto iter = pendingRequests_.begin();
	     iter != pendingRequests_.end(); iter++) {

		PendingRequest &request = *iter;
		if (request.mediaRequest != mediaRequest)
			continue;

		foundRequest = true;

		/* Mark request as completed */
		completedRequests_.emplace_back(request.request);
		requestCompleted.emit(request.request);

		/* Re-init media request. Buffers will be recycled on the
		 * destructor of PendingRequest */
		media_->reInitRequest(request.mediaRequest);
		mediaRequestPool_.put(request.mediaRequest);

		pendingRequests_.erase(iter);
		break;
	}

	ASSERT(foundRequest == true);
}

int ImgSysDevice::configure()
{
	handleKva(Delete, descPool_);
	handleIova(Delete, ctrlMetaPool_);

	descPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size{238544, 1}, 6);
	ctrlMetaPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size{24704, 1}, 6);

	handleKva(Add, descPool_);
	handleIova(Add, ctrlMetaPool_);

	descPool_.mmap();
	ctrlMetaPool_.mmap();

	return 0;
}

int ImgSysDevice::start()
{
	ASSERT(pendingRequests_.empty());
	ASSERT(completedRequests_.empty());
	return 0;
}

int ImgSysDevice::stop()
{
	ASSERT(pendingRequests_.empty());
	ASSERT(completedRequests_.empty());
	return 0;
}

int ImgSysDevice::handleKva(FdCtrl fdHandle, InfoFramePool &pool)
{
	std::vector<int> data = pool.collectFds();
	if (data.empty())
		return -EINVAL;

	const unsigned long int ctrl =
		(fdHandle == Add) ? MTKDIP_IOC_ADD_KVA : MTKDIP_IOC_DEL_KVA;

	struct fd_info fds;
	fds.fd_num = data.size();
	std::copy(data.begin(), data.end(), fds.fds);

	if (mtkIspDip_->ioctl(ctrl, &fds)) {
		LOG(MtkISP7, Error) << "Fail to handle kva";
		return -EINVAL;
	}

	return 0;
}

int ImgSysDevice::handleIova(FdCtrl fdHandle, InfoFramePool &pool)
{
	std::vector<int> data = pool.collectFds();
	if (data.empty())
		return -EINVAL;

	const unsigned long int ctrl =
		(fdHandle == Add) ? MTKDIP_IOC_ADD_IOVA : MTKDIP_IOC_DEL_IOVA;

	struct fd_tbl fds;
	fds.fd_num = data.size();
	fds.fds = reinterpret_cast<uint64_t>(data.data());

	if (mtkIspDip_->ioctl(ctrl, &fds)) {
		LOG(MtkISP7, Error) << "Fail to handle iova";
		return -EINVAL;
	}

	return 0;
}

/* The function should be removed once the backend library is moved to scp */
int ImgSysDevice::startImgSysBackend()
{
	if (!backEndLibrary_) {
		backEndLibrary_ = dlopen("libimgsys_daemon.so", RTLD_NOW);
		if (!backEndLibrary_) {
			LOG(MtkISP7, Error) << "Fail to load backend library: "
					    << dlerror();
			return -EINVAL;
		}
	}

	auto startRED = reinterpret_cast<int (*)()>(dlsym(backEndLibrary_, "startRED"));
	if (!startRED) {
		LOG(MtkISP7, Error) << "Fail to load backend symbol startRED: "
				    << dlerror();
		return -EINVAL;
	}

	return (*startRED)();
}

using namespace NSCam::NSImgStream;

void ImgSysRequestHelper::queueRequest(SingleDeviceRequest &sdRequest)
{
	imgSysRequest_.sdRequest = &sdRequest;
	imgSys_->queueRequest(&imgSysRequest_);

	imgSys_->requestCompleted.connect(this, &ImgSysRequestHelper::requestReady);
	startTime_ = std::chrono::steady_clock::now();
}

void ImgSysRequestHelper::requestReady(ImgSysDevice::Request *request)
{
	if (request != &imgSysRequest_)
		return;

	imgSys_->requestCompleted.disconnect(this, &ImgSysRequestHelper::requestReady);
	imgSys_->claimCompletedRequest(request);

	/* Sample running time of the task */
	if (request_->sequence() % 30 == 0) {
		std::chrono::steady_clock::time_point finish =
			std::chrono::steady_clock::now();
		std::chrono::steady_clock::duration d = finish - startTime_;
		std::chrono::milliseconds milliseconds =
			std::chrono::duration_cast<std::chrono::milliseconds>(d);
		LOG(MtkISP7, Debug) << task_->id()
				    << " runs " << milliseconds.count() << "ms";
	}

	task_->notifyDone();
}

} /* namespace libcamera */
