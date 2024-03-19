/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * imgsys.cpp - MTK MtkISP7 ImgSys device
 */

#include "imgsys.h"

#include <dlfcn.h>
#include <numeric>
#include <sys/ioctl.h>

#include <libcamera/formats.h>
#include <libcamera/geometry.h>
#include <libcamera/request.h>

#include "libcamera/internal/dma_heaps.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pools.h"

#include "kernel-headers/mtk_header_desc.h"
#include "kernel-headers/mtk_imgsys.h"
#include "pipeline/mtkisp7/imgsys/single_device.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"
#include "platform/mtkisp7/ImgPortDef.h"
#include "platform/mtkisp7/single_device_helper.h"
#include "platform/mtkisp7/topology.h"

namespace libcamera {

using namespace NSCam::NSImgStream;

LOG_DECLARE_CATEGORY(MtkISP7)

int ImgsysVideoDevice::configure(V4L2DeviceFormat *fmt, int resizeRatio,
				 Rectangle crop)
{
	int ret;

	if (*fmt != format_) {
		ret = setFormat(fmt);
		if (ret)
			return ret;

		resizeRatio_ = 0;
		crop_ = Rectangle();
	}

	if (resizeRatio != resizeRatio_) {
		struct v4l2_ext_control ext_ctrl;
		ext_ctrl.id = V4L2_CID_MTK_RESIZE_RATIO;
		ext_ctrl.size = sizeof(int);
		ext_ctrl.value = resizeRatio;
		ret = setExtControl(&ext_ctrl, -1);
		if (ret)
			return ret;

		resizeRatio_ = resizeRatio;
	}

	if (crop != crop_) {
		ret = setSelection(V4L2_SEL_TGT_CROP, &crop);
		if (ret)
			return ret;

		crop_ = crop;
	}

	return 0;
}

Rectangle ImgSysDevice::getCrop(Size inSize, Size outSize)
{
	/* 4:3 */
	if (outSize.width * 3 == outSize.height * 4)
		return { 0, 0, inSize.width, inSize.height };

	/* 16:9 */
	unsigned int height = inSize.width * 9 / 16;
	int y = (inSize.height - height) / 2;

	return { 0, y, inSize.width, height };
}

ImgSysDevice::ImgSysDevice(OnDeviceTuner *odt) : onDeviceTuner_(odt)
{
}

int ImgSysDevice::init(MediaDevice *media, DmaHeap *dmaHeap)
{
	media_ = media;
	dmaHeap_ = dmaHeap;

	const std::string hubName = "MTK-ISP-DIP-V4L2";
	mtkIspDip_ = V4L2Subdevice::fromEntityName(media_, hubName);

	if (!mtkIspDip_ || mtkIspDip_->open())
		return -ENODEV;

	/* The four entities would be configured differently */
	MediaEntity *sigdevNorm = media_->getEntityByName(hubName + " SIGDEVN");
	MediaEntity *tuningMeta = media_->getEntityByName(hubName + " Tuning");
	MediaEntity *ctrlMeta = media_->getEntityByName(hubName + " CtrlMeta");

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
	for (auto &port : ports) {
		MediaEntity *entity = media_->getEntityByName(port.device_name);
		if (entity->type() != MediaEntity::Type::V4L2VideoDevice)
			continue;

		// Enable the only link of video devices to/from hub
		MediaLink *link = entity->pads()[0]->links()[0];
		link->setEnabled(true);

		std::unique_ptr<ImgsysVideoDevice> videoDev =
			std::make_unique<ImgsysVideoDevice>(entity);

		if (videoDev->open())
			return -ENODEV;

		if (entity == sigdevNorm) {
			// Weak ptr for sigdevNorm for easier queuing requests
			sigdevNorm_ = videoDev.get();
			configureVideo(videoDev.get(), formats::MTSR_MTISP, { sizeof(struct singlenode_desc_norm), 1 });
		} else if (entity == ctrlMeta) {
			ctrlMeta_ = videoDev.get();
			configureVideo(videoDev.get(), formats::MTFP_MTISP, { 28672, 1 });
		} else if (entity == tuningMeta)
			configureVideo(videoDev.get(), formats::MTFD_MTISP, { 219348, 1 });

		// All video devices for easier streamOn/Off
		allVideoDevices_[IMG_PORT(port.port_index)] = std::move(videoDev);
	}

	std::vector<UniqueFD> requests;
	media_->allocateRequests(32, requests);
	mediaRequestPool_.setData(requests);

	/* Sync token starts from 1 */
	std::vector<BasicContainer<uint32_t>> syncs;
	for (uint32_t i = 1; i < 200; i++)
		syncs.emplace_back(i);

	syncPool_.setData(syncs);

	for (auto &[portIdx, device] : allVideoDevices_)
		device->requestBufferReady.connect(this, &ImgSysDevice::bufferReady);

	return 0;
}

void reconfigureVideoNode(ImgsysVideoDevice &device, const PortInfoEx &info)
{
	if (info.portIdx == IMG_PORT_METAI ||
	    info.portIdx == IMG_PORT_DRV_CTRLMETAI ||
	    info.portIdx == IMG_PORT_DRV_SIGDEV_NORMI ||
	    info.portIdx == IMG_PORT_IMGSTATO)
		return;

	V4L2DeviceFormat format;
	uint32_t fourcc =
		getV4L2Fmt(info.img.getImgFormat(),
			   info.img.getColorArrangement());

	format.size = { static_cast<unsigned int>(info.img.getImgSize().w),
			static_cast<unsigned int>(info.img.getImgSize().h) };
	format.fourcc = V4L2PixelFormat(fourcc);
	format.planesCount = info.img.getPlaneCount();
	for (unsigned int i = 0; i < format.planesCount; ++i) {
		format.planes[i] = { static_cast<uint32_t>(info.img.getBufSizeInBytes(i)),
				     static_cast<uint32_t>(info.img.getBufStridesInBytes(i)) };
	}

	Rectangle crop =
		Rectangle(info.CropX, info.CropY,
			  { static_cast<unsigned int>(info.CropW),
			    static_cast<unsigned int>(info.CropH) });

	device.configure(&format, info.mResizeRatio, crop);
}

static IMG_PORT getDevicePort(uint32_t portIdx)
{
	switch (portIdx) {
	case IMG_PORT_LTIMGI:
		return IMG_PORT_TIMGI;
	case IMG_PORT_LTYUV2O:
		return IMG_PORT_TYUV2O;
	case IMG_PORT_LTYUV3O:
		return IMG_PORT_TYUV3O;
	case IMG_PORT_LTYUV4O:
		return IMG_PORT_TYUV4O;
	case IMG_PORT_LTYUV5O:
	case IMG_PORT_FEO:
		return IMG_PORT_TYUV5O;
	default:
		return IMG_PORT(portIdx);
	}
}

int ImgSysDevice::queueRequestV4L2(Request *request)
{
	SharedMailBox<InfoFrame> ctrlMeta = makeMailBox<InfoFrame>();
	ctrlMetaPool_.fetch(ctrlMeta);
	InfoFrame &infoCtrl = ctrlMeta->get();

	SharedMailBox<InfoFrame> singleDevNorm = makeMailBox<InfoFrame>();
	int mediaRequest = mediaRequestPool_.get();

	std::vector<PEU_Stage> stages{
		request->sdRequest->Stages()[request->stage].getStageEnum() };
	{
		DmaSyncer syncerCtrl(infoCtrl.buffer()->planes()[0].fd.get(), DmaHeap::SyncWrite);

		request->sdRequest->fillRequestBufferForStage(
			infoCtrl, mediaRequest, request->stage);
		onDeviceTuner_->tuneImgsysMetadata(
			request->sdRequest->sequence(),
			request->sdRequest->sequence(),
			stages,
			infoCtrl, mediaRequest);
	}

	StageEx &stage = request->sdRequest->Stages()[request->stage];
	int ret = 0;

	for (const PortInfoEx &port : stage.getInputs()) {
		ImgsysVideoDevice &device = *allVideoDevices_[getDevicePort(port.portIdx)];
		reconfigureVideoNode(device, port);
		ret |= device.queueBuffer(port.frameBuffer, mediaRequest);
		request->buffers_count++;
	}

	for (const PortInfoEx &port : stage.getOutputs()) {
		ImgsysVideoDevice &device = *allVideoDevices_[getDevicePort(port.portIdx)];
		reconfigureVideoNode(device, port);
		ret |= device.queueBuffer(port.frameBuffer, mediaRequest);
		request->buffers_count++;
	}

	ret |= ctrlMeta_->queueBuffer(infoCtrl.buffer(), mediaRequest);
	request->buffers_count ++;
	ret |= media_->queueRequest(mediaRequest);

	if (ret) {
		LOG(MtkISP7, Error) << "Fail to queue request";
		return ret;
	}

	pendingRequests_.push_back({ request, mediaRequest,
				     request->sdRequest->sequence(),
				     1, ctrlMeta,
				     singleDevNorm });

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

	{
		DmaSyncer syncerCtrl(infoCtrl.buffer()->planes()[0].fd.get(), DmaHeap::SyncWrite);
		DmaSyncer syncerDesc(infoDesc.buffer()->planes()[0].fd.get(), DmaHeap::SyncWrite);

		request->sdRequest->fillRequestBuffer(infoCtrl, infoDesc, mediaRequest);
		onDeviceTuner_->tuneImgsysMetadata(
			request->sdRequest->sequence(),
			request->sdRequest->sequence(),
			request->sdRequest->getStageEnums(),
			infoCtrl, mediaRequest);
	}

	int ret = sigdevNorm_->queueBuffer(singleDev, mediaRequest);
	request->buffers_count++;
	ret |= media_->queueRequest(mediaRequest);

	if (ret) {
		LOG(MtkISP7, Error) << "Fail to queue request";
		return ret;
	}

	pendingRequests_.push_back({ request, mediaRequest,
				     request->sdRequest->sequence(),
				     request->sdRequest->Stages().size(),
				     ctrlMeta, singleDevNorm });
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
	auto [buffer, mediaRequest] = pair;
	ASSERT(buffer);

	bool foundRequest = false;
	for (auto iter = pendingRequests_.begin();
	     iter != pendingRequests_.end(); iter++) {
		PendingRequest &request = *iter;
		if (request.mediaRequest != mediaRequest)
			continue;

		if (request.request->buffers_count <= 0) {
			LOG(MtkISP7, Error)
				<< "Buffer count for media request: "
				<< mediaRequest << "doesn't match.";
			return;
		}

		foundRequest = true;

		request.request->buffers_count--;

		if (request.request->buffers_count)
			return;

		/* Mark request as completed */
		completedRequests_.emplace_back(request.request);
		requestCompleted.emit(request.request);

		/* Use the media request to tune the driver */
		onDeviceTuner_->tuneImgsysDriver(request.internalRequestId,
						 request.mediaRequest,
						 request.stageCount);

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
	#if !V4L2_STANDARD_MODE
		handleIova(Delete, ctrlMetaPool_);
		descPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size{ 266960, 1 }, 32, DmaHeap::CMA);
	#endif

	ctrlMetaPool_.createBuffers(dmaHeap_, formats::MTFD_MTISP, Size{ 28672, 1 }, 32, DmaHeap::CMA);

	#if !V4L2_STANDARD_MODE
		handleKva(Add, descPool_);
		handleIova(Add, ctrlMetaPool_);

		descPool_.mmap();
	#endif
	ctrlMetaPool_.mmap();

	int ret;
	for (auto &[portIdx, device] : allVideoDevices_) {
		/*
		 * Force kernel to release the previously queued buffers,
		 * otherwise, CMA will run out of memory.
		 *
		 * TODO: The buffers from CMA (and other buffers whose size
		 * doesn't depend on resolution) can be allocated in init(),
		 * instead of allocate each time in configure().
		 * Remove this after buffer allocation is moved into init().
		 */
		device->releaseBuffers();

		/*
		 * Some video nodes require more buffers than others to avoid
		 * cache misses.
		 *
		 * TODO: If a video node will take buffers with different format
		 * or size during stream, call setFormat and CREATEBUFS with
		 * the correct formats.
		 * The number of buffers created should match the total number
		 * of buffers needed for a video node.
		 */
		switch (portIdx) {
#if V4L2_STANDARD_MODE
		case IMG_PORT_TIMGI:
		case IMG_PORT_IMGI:
			ret = device->importBuffers(128);
			break;
		case IMG_PORT_METAI:
			ret = device->importBuffers(256);
			break;
#else
		case IMG_PORT_TIMGI:
		case IMG_PORT_METAI:
		case IMG_PORT_IMGI:
#endif
		case IMG_PORT_WPE_VECI:
		case IMG_PORT_VIPI:
		case IMG_PORT_TYUV2O:
		case IMG_PORT_TYUV3O:
		case IMG_PORT_TYUV5O:
		case IMG_PORT_TNRCI:
		case IMG_PORT_REC_DSI:
		case IMG_PORT_IMG3O:
			ret = device->importBuffers(64);
			break;
		case IMG_PORT_DRV_CTRLMETAI:
		case IMG_PORT_DRV_SIGDEV_NORMI:
			ret = device->importBuffers(32);
			break;
		default:
			ret = device->importBuffers(24);
			break;
		}

		if (ret)
			return ret;
	}

	return 0;
}

int ImgSysDevice::start()
{
	for (auto &[portIdx, device] : allVideoDevices_) {
		int ret = device->streamOn();
		if (ret) {
			LOG(MtkISP7, Error) << "Fail to start "
					    << device->devicePath();
			return ret;
		}
	}

	ASSERT(pendingRequests_.empty());
	ASSERT(completedRequests_.empty());

	return 0;
}

int ImgSysDevice::stop()
{
	for (auto &[portIdx, device] : allVideoDevices_) {
		int ret = device->streamOff();
		if (ret) {
			LOG(MtkISP7, Error) << "Fail to streamOff "
					    << device->devicePath();
			return ret;
		}
	}

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

void ImgSysRequestHelper::queueRequest(SingleDeviceRequest &sdRequest)
{
#if V4L2_STANDARD_MODE
	for (size_t stage = 0; stage < sdRequest.Stages().size(); ++stage) {
		imgSysRequests_.push_back({ &sdRequest, stage, 0 });
		imgSys_->queueRequestV4L2(&imgSysRequests_.back());
	}
#else
	imgSysRequests_.push_back({ &sdRequest, 0, 0 });
	imgSys_->queueRequest(&imgSysRequests_.back());
#endif

	imgSys_->requestCompleted.connect(this, &ImgSysRequestHelper::requestReady);
	startTime_ = std::chrono::steady_clock::now();
}

void ImgSysRequestHelper::requestReady(ImgSysDevice::Request *request)
{
	for (auto iter = imgSysRequests_.begin(); iter != imgSysRequests_.end(); iter++) {
		if (request != &*iter)
			continue;

		imgSys_->claimCompletedRequest(request);
		imgSysRequests_.erase(iter);
		break;
	}

	if (!imgSysRequests_.empty())
		return;

	imgSys_->requestCompleted.disconnect(this, &ImgSysRequestHelper::requestReady);

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
