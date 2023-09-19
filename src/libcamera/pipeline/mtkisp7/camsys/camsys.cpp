/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * camsys.cpp - MTK MtkISP7 Camsys device
 */

#include "camsys.h"

#include <libcamera/formats.h>
#include <libcamera/framebuffer.h>
#include <libcamera/geometry.h>

#include "libcamera/internal/camera_sensor_properties.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/mapped_framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/request.h"

#include "kernel-headers/imgsensor-user.h"
#include "linux/v4l2-controls.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

constexpr unsigned int PAD_SENSOR_OUT = 0;
constexpr unsigned int PAD_SENINF_OUT = 1;
constexpr unsigned int PAD_SENINF_IN = 0;
constexpr unsigned int PAD_RAW_IN = 0;
constexpr unsigned int PAD_MAIN = 5;
constexpr unsigned int PAD_YUV1 = 6;
constexpr unsigned int PAD_YUV2 = 7;
constexpr unsigned int PAD_DRZS4NO3 = 13;
constexpr unsigned int PAD_RZH1N2TO1 = 14;

constexpr Size kMeSize = Size{ 576, 432 };
constexpr Size kFdSize = Size{ 640, 480 };

const std::string kRawPrefix = "mtk-cam raw-";
const std::string kSeninfPrefix = "seninf-";

constexpr unsigned int kRequestCount = 24;

} /* namespace */

CamSysDevice::CamSysDevice() = default;

int CamSysDevice::init(MediaDevice *media, unsigned int index, Hal3A *hal3A)
{
	index_ = index;
	media_ = media;
	hal3A_ = hal3A;

	MediaEntity *videoHubEntity =
		media_->getEntityByName(kRawPrefix + std::to_string(index_));

	MediaEntity *seninfEntity =
		media_->getEntityByName(kSeninfPrefix + std::to_string(index_));

	if (initSensor(seninfEntity)) {
		LOG(MtkISP7, Info) << "No sensor attached to CamSys " << index;
		return -ENODEV;
	}

	seninf_ = std::make_unique<V4L2Subdevice>(seninfEntity);
	videoHub_ = std::make_unique<V4L2Subdevice>(videoHubEntity);

	if (videoHub_->open()) {
		LOG(MtkISP7, Error) << "Fail to open "
				    << videoHub_->entity()->id();
		close();
		return -ENODEV;
	}

	if (seninf_->open()) {
		LOG(MtkISP7, Error) << "Fail to open "
				    << seninf_->entity()->id();
		close();
		return -ENODEV;
	}

	/* Helper function to create video nodes and collect them into
	 * allVideoDevices_ for easier StreamOn/Off. */
	auto getVideoDevice = [this](const std::string &name,
				     std::unique_ptr<V4L2VideoDevice> &videoDevice) {
		MediaEntity *entity = media_->getEntityByName(name);
		videoDevice = std::make_unique<V4L2VideoDevice>(entity);

		allVideoDevices_.emplace_back(videoDevice.get());
	};

	const std::string &hubName = videoHubEntity->name();
	getVideoDevice(hubName + " meta-input", metaInput_);
	getVideoDevice(hubName + " rawi-2", rawi2_);
	getVideoDevice(hubName + " main-stream", mainStream_);
	getVideoDevice(hubName + " yuvo-1", yuvo1_);
	getVideoDevice(hubName + " yuvo-2", yuvo2_);
	getVideoDevice(hubName + " yuvo-3", yuvo3_);
	getVideoDevice(hubName + " yuvo-4", yuvo4_);
	getVideoDevice(hubName + " yuvo-5", yuvo5_);
	getVideoDevice(hubName + " drzs4no-1", drzs4no1_);
	getVideoDevice(hubName + " drzs4no-2", drzs4no2_);
	getVideoDevice(hubName + " drzs4no-3", drzs4no3_);
	getVideoDevice(hubName + " rzh1n2to-1", rzh1n2to1_);
	getVideoDevice(hubName + " rzh1n2to-2", rzh1n2to2_);
	getVideoDevice(hubName + " rzh1n2to-3", rzh1n2to3_);
	getVideoDevice(hubName + " sv-imgo-1", svImgOut1_);
	getVideoDevice(hubName + " sv-imgo-2", svImgOut2_);
	getVideoDevice(hubName + " partial-meta-0", partialMeta0_);
	getVideoDevice(hubName + " partial-meta-1", partialMeta1_);
	getVideoDevice(hubName + " partial-meta-2", partialMeta2_);
	getVideoDevice(hubName + " ext-meta-0", extMeta0_);
	getVideoDevice(hubName + " ext-meta-1", extMeta1_);
	getVideoDevice(hubName + " ext-meta-2", extMeta2_);

	for (V4L2VideoDevice *device : allVideoDevices_) {
		int ret = device->open();
		// todo: Fix rawi2 failing to enum formats properly from driver.
		// For now, ignore the open error for rawi2.
		if (ret && device != rawi2_.get()) {
			LOG(MtkISP7, Error) << "Fail to open "
					    << device->devicePath();
			close();
			return ret;
		}
	}

	std::vector<UniqueFD> requests;
	media_->allocateRequests(kRequestCount, requests);
	mediaRequestPool_.setData(requests);

	yuvo1_->bufferReady.connect(this, &CamSysDevice::bufferReady);
	yuvo2_->bufferReady.connect(this, &CamSysDevice::bufferReady);
	drzs4no3_->bufferReady.connect(this, &CamSysDevice::bufferReady);
	rzh1n2to1_->bufferReady.connect(this, &CamSysDevice::bufferReady);
	metaInput_->bufferReady.connect(this, &CamSysDevice::bufferReady);
	mainStream_->bufferReady.connect(this, &CamSysDevice::bufferReady);
	partialMeta0_->bufferReady.connect(this, &CamSysDevice::bufferReady);
	partialMeta1_->bufferReady.connect(this, &CamSysDevice::bufferReady);

	return 0;
}

int CamSysDevice::start()
{
	/* Initial settings before start */
	int ret = setExposureGain(1540, 1024);
	if (ret) {
		LOG(MtkISP7, Warning) << "Fail to set initila exposure";
		return ret;
	}

	ret = setTestPattern(controls::draft::TestPatternModeOff);
	if (ret)
		LOG(MtkISP7, Warning) << "Fail to reset test pattern";

	ret = setupSeninf(true);
	if (ret) {
		LOG(MtkISP7, Error) << "Fail to setup Seninf";
		return ret;
	}

	for (V4L2VideoDevice *device : allVideoDevices_) {
		ret = device->streamOn();
		if (ret) {
			LOG(MtkISP7, Error) << "Fail to streamOn "
					    << device->devicePath();
			return ret;
		}
	}

	ret = videoHub_->setFrameStartEnabled(true);
	if (ret) {
		LOG(MtkISP7, Error) << "Fatal due to cannot enable frame start "
				    << strerror(-ret);
		return ret;
	}

	ASSERT(pendingRequests_.empty());
	ASSERT(completedRequests_.empty());

	return 0;
}

int CamSysDevice::stop()
{
	int ret;
	for (V4L2VideoDevice *device : allVideoDevices_) {
		ret = device->streamOff();
		if (ret) {
			LOG(MtkISP7, Error) << "Fail to streamOff "
					    << device->devicePath();
			return ret;
		}
	}

	for (V4L2VideoDevice *device : allVideoDevices_) {
		ret = device->releaseBuffers();
		if (ret) {
			LOG(MtkISP7, Error) << "Fail to release buffers "
					    << device->devicePath();
			return ret;
		}
	}

	ret = setupSeninf(false);
	if (ret) {
		LOG(MtkISP7, Error) << "Fail to setup Seninf";
		return ret;
	}

	ret = videoHub_->setFrameStartEnabled(false);
	if (ret)
		LOG(MtkISP7, Error) << "Fatal due to cannot disable frame start " << strerror(-ret);

	ASSERT(pendingRequests_.empty());
	ASSERT(completedRequests_.empty());

	return ret;
}

void CamSysDevice::close()
{
	videoHub_.reset();
	seninf_.reset();
	sensor_.reset();

	metaInput_.reset();
	rawi2_.reset();

	mainStream_.reset();
	yuvo1_.reset();
	yuvo2_.reset();
	yuvo3_.reset();
	yuvo4_.reset();
	yuvo5_.reset();

	drzs4no1_.reset();
	drzs4no2_.reset();
	drzs4no3_.reset();
	rzh1n2to1_.reset();
	rzh1n2to2_.reset();
	rzh1n2to3_.reset();
	svImgOut1_.reset();
	svImgOut2_.reset();

	partialMeta0_.reset();
	partialMeta1_.reset();
	partialMeta2_.reset();
	extMeta0_.reset();
	extMeta1_.reset();
	extMeta2_.reset();

	allVideoDevices_.clear();
}

int CamSysDevice::configure(const Size &rawFrameSize, const Size &yuvFrameSize)
{
	rawFrameSize_ = rawFrameSize;
	yuvFrameSize_ = yuvFrameSize;

	setupLinks(false);

	int ret = configureSensor();
	ret |= configureMtkCamRaw();

	if (ret)
		LOG(MtkISP7, Error) << "Fail to configure CamSys";

	return ret;
}

void CamSysDevice::fillTuningBuffer(FrameBuffer *buffer)
{
	MappedFrameBuffer mappedBuffer(buffer, MappedFrameBuffer::MapFlag::ReadWrite);

	buffer->_d()->metadata().planes()[0].bytesused = buffer->planes()[0].length;
	memcpy(mappedBuffer.planes()[0].data(), &hal3A_->r3AResult_.raw_meta, 113664);
}

int CamSysDevice::queueRequest(Request *request)
{
	fillTuningBuffer(request->tuning);

	int mediaRequest = mediaRequestPool_.get();

	int ret = metaInput_->queueBuffer(request->tuning, mediaRequest);
	ret |= partialMeta0_->queueBuffer(request->statistics0, mediaRequest);
	ret |= partialMeta1_->queueBuffer(request->statistics1, mediaRequest);
	ret |= mainStream_->queueBuffer(request->main, mediaRequest);
	ret |= yuvo1_->queueBuffer(request->yuvo1, mediaRequest);
	ret |= yuvo2_->queueBuffer(request->yuvo2, mediaRequest);
	ret |= drzs4no3_->queueBuffer(request->me, mediaRequest);
	ret |= rzh1n2to1_->queueBuffer(request->faceDetect, mediaRequest);

	ret |= media_->queueRequest(mediaRequest);
	if (ret) {
		LOG(MtkISP7, Error) << "Fail to queue request";
		return ret;
	}

	// todo: Set pending number by the framebuffers queued, instead of
	// hard coding as a magic number 8.
	pendingRequests_.emplace_back(PendingRequest{ request, mediaRequest, 8 });

	return 0;
}

int CamSysDevice::claimCompletedRequest(Request *request)
{
	auto iter = completedRequests_.begin();
	while (iter != completedRequests_.end()) {
		if (*iter == request) {
			completedRequests_.erase(iter);
			return 0;
		}
	}
	return -EINVAL;
}

int CamSysDevice::setExposureGain(uint32_t exposure, uint32_t gain)
{
	ControlList ctrl(sensor_->controls());
	ctrl.set(V4L2_CID_EXPOSURE, (int32_t)exposure);
	ctrl.set(V4L2_CID_ANALOGUE_GAIN, (int32_t)gain);
	ctrl.set(V4L2_CID_DIGITAL_GAIN, (int32_t)gain);

	return sensor_->device()->setControls(&ctrl);
}

int CamSysDevice::setTestPattern(controls::draft::TestPatternModeEnum mode)
{
	const CameraSensorProperties *properties =
		CameraSensorProperties::get(sensor_->model());

	auto iter = properties->testPatternModes.find(mode);
	if (iter == properties->testPatternModes.end()) {
		LOG(MtkISP7, Debug) << "Invalid pattern mode: " << mode;
		return -EINVAL;
	}

	ControlList ctrl(sensor_->controls());
	ctrl.set(V4L2_CID_TEST_PATTERN, iter->second);

	return sensor_->device()->setControls(&ctrl);
}

int CamSysDevice::setFrameInterval(uint32_t numerator, uint32_t denominator)
{
	int ret = sensor_->device()->setFrameInterval(PAD_SENSOR_OUT, numerator,
						      denominator);
	if (ret)
		LOG(MtkISP7, Error) << "Fail to set frame interval "
				    << " numerator " << numerator
				    << " denominator " << denominator;

	return ret;
}

int CamSysDevice::setupResource()
{
	V4L2SubdeviceFormat format = {};

	format.mbus_code = mbusCode_;
	format.size = rawFrameSize_;

	int ret = videoHub_->setFormat(PAD_RAW_IN, &format);
	if (ret) {
		LOG(MtkISP7, Error) << "Fail to set format for " << videoHub_->entity();
		return -EINVAL;
	}

	struct mtk_cam_resource camsysResource;
	camsysResource.sink_fmt = (__u64)&format.subdevFmt.format;

	auto &sensorResource = camsysResource.sensor_res;

	IPACameraSensorInfo sensorInfo;
	sensor_->sensorInfo(&sensorInfo);
	auto ctrls = sensor_->getControls({ V4L2_CID_HBLANK, V4L2_CID_VBLANK });

	const ControlInfo hblank = ctrls.infoMap()->at(V4L2_CID_HBLANK);
	const ControlInfo vblank = ctrls.infoMap()->at(V4L2_CID_VBLANK);

	sensorResource.interval = { 1, 30 };
	sensorResource.hblank = hblank.min().get<int32_t>();
	sensorResource.vblank = vblank.max().get<int32_t>();
	sensorResource.pixel_rate = sensorInfo.pixelRate;
	sensorResource.cust_pixel_rate = sensorInfo.pixelRate;

	auto &rawResource(camsysResource.raw_res);
	rawResource.feature = 0;
	rawResource.strategy = 3;
	rawResource.raw_max = (uint8_t)MTK_CAM_RESOURCE_DEFAULT;
	rawResource.raw_min = (uint8_t)MTK_CAM_RESOURCE_DEFAULT;
	rawResource.raw_used = 0;
	rawResource.bin = 0;
	rawResource.path_sel = (uint8_t)MTK_CAM_RESOURCE_DEFAULT;
	rawResource.pixel_mode = 0;
	rawResource.throughput = 0;

	struct v4l2_ext_control ext_ctrl {
		.id = V4L2_CID_MTK_CAM_RAW_RESOURCE_CALC,
		.size = sizeof(camsysResource),
		.reserved2 = {},
		.ptr = &camsysResource
	};

	ret = videoHub_->setExtControl(&ext_ctrl);
	if (ret) {
		LOG(MtkISP7, Error) << "Fail to calculate resources for "
				    << videoHub_->entity()->name();
		return ret;
	}

	return ret;
}

int CamSysDevice::setFormat(V4L2Subdevice *device, int pad,
			    uint32_t mbus_code, Size size)
{
	V4L2SubdeviceFormat format = { mbus_code, size, {}, {} };
	int ret = device->setFormat(pad, &format);
	if (ret)
		LOG(MtkISP7, Error) << "Fail to set format to "
				    << device->entity()->id()
				    << " pad " << pad << " format " << format;

	return ret;
}

int CamSysDevice::setFormat(V4L2VideoDevice *device, const PixelFormat &format,
			    Size size)
{
	const PixelFormatInfo &info = PixelFormatInfo::info(format);

	V4L2DeviceFormat outputFormat = {
		.fourcc = device->toV4L2PixelFormat(format),
		.size = size,
		.colorSpace = std::nullopt,
		.planes = { { { 0, info.stride(size.width, 0) } } },
		.planesCount = 1,
	};

	return device->setFormat(&outputFormat);
}

int CamSysDevice::initSensor(MediaEntity *seninfEntity)
{
	const std::vector<MediaPad *> &pads = seninfEntity->pads();
	if (pads.empty())
		return -ENODEV;

	/* seninf have a single sink pad from sensor at index 0. */
	MediaPad *sink = pads[0];

	const std::vector<MediaLink *> &links = sink->links();
	if (links.empty())
		return -ENODEV;

	MediaLink *link = links[0];
	MediaEntity *entity = link->source()->entity();

	auto sensor = std::make_unique<CameraSensor>(entity);
	if (sensor->init())
		return -ENODEV;

	/* Supported mbus codes */
	static std::map<unsigned int, PixelFormat> mbusCodes = {
		{ MEDIA_BUS_FMT_SBGGR10_1X10, formats::SBGGR10_MTISP },
		{ MEDIA_BUS_FMT_SGBRG10_1X10, formats::SGBRG10_MTISP },
		{ MEDIA_BUS_FMT_SGRBG10_1X10, formats::SGRBG10_MTISP },
		{ MEDIA_BUS_FMT_SRGGB10_1X10, formats::SRGGB10_MTISP },
	};

	/* Find one mbus supported by both the sensor and Camsys */
	mbusCode_ = 0;
	for (auto code : sensor->mbusCodes()) {
		const auto &iter = mbusCodes.find(code);
		if (iter != mbusCodes.end()) {
			mbusCode_ = code;
			bayerFormat_ = iter->second;
		}
	}
	if (!mbusCode_)
		return -ENODEV;

	sensor_ = std::move(sensor);
	return 0;
}

int CamSysDevice::configureSensor()
{
	V4L2SubdeviceFormat format;
	format.mbus_code = mbusCode_;
	format.size = rawFrameSize_;

	if (sensor_->device()->setFormat(PAD_SENSOR_OUT, &format)) {
		LOG(MtkISP7, Error) << "Fail to set format to sensor: "
				    << sensor_->entity()->name();
		return -EINVAL;
	}

	if (seninf_->setFormat(PAD_SENINF_IN, &format)) {
		LOG(MtkISP7, Error) << "Fail to set format to seninf in: "
				    << seninf_->entity()->name();
		return -EINVAL;
	}

	if (seninf_->setFormat(PAD_SENINF_OUT, &format)) {
		LOG(MtkISP7, Error) << "Fail to set format to seninf out: "
				    << seninf_->entity()->name();
		return -EINVAL;
	}

	if (setFrameInterval(1, 30)) {
		LOG(MtkISP7, Error) << "Fail to set initial frame interval";
		return -EINVAL;
	}

	return 0;
}

int CamSysDevice::configureMtkCamRaw()
{
	int ret = setupLinks(true);
	if (ret) {
		LOG(MtkISP7, Error) << "Fail to setup links";
		return ret;
	}

	ret = setupResource();
	if (ret) {
		LOG(MtkISP7, Error) << "Fail to calculate resources";
		return ret;
	}

	Size halfYuvSize = yuvFrameSize_ / 2;

	ret = configureVideo(mainStream_.get(), bayerFormat_, rawFrameSize_,
			     videoHub_.get(), PAD_MAIN,
			     mbusCode_);
	ret |= configureVideo(yuvo1_.get(), formats::NV12_10P_MTISP, yuvFrameSize_,
			      videoHub_.get(), PAD_YUV1,
			      MEDIA_BUS_FMT_SRGGB10_1X10);
	ret |= configureVideo(yuvo2_.get(), formats::NV12_12P_MTISP, halfYuvSize,
			      videoHub_.get(), PAD_YUV2,
			      MEDIA_BUS_FMT_SRGGB10_1X10);
	ret |= configureVideo(drzs4no3_.get(), formats::GREY, kMeSize,
			      videoHub_.get(), PAD_DRZS4NO3,
			      MEDIA_BUS_FMT_SRGGB10_1X10);
	ret |= configureVideo(rzh1n2to1_.get(), formats::NV12, kFdSize,
			      videoHub_.get(), PAD_RZH1N2TO1,
			      MEDIA_BUS_FMT_SRGGB10_1X10);

	if (ret) {
		LOG(MtkISP7, Error) << "Fail to configure video nodes";
		return ret;
	}

	Rectangle fullRect(rawFrameSize_);

	ret |= yuvo1_->setSelection(V4L2_SEL_TGT_CROP, &fullRect);
	ret |= yuvo2_->setSelection(V4L2_SEL_TGT_CROP, &fullRect);
	ret |= drzs4no3_->setSelection(V4L2_SEL_TGT_CROP, &fullRect);
	ret |= rzh1n2to1_->setSelection(V4L2_SEL_TGT_CROP, &fullRect);

	if (ret) {
		LOG(MtkISP7, Error) << "Fail to set selection for video nodes";
		return ret;
	}

	ret = 0;
	for (V4L2VideoDevice *device : allVideoDevices_) {
		ret |= device->importBuffers(16);
	}

	if (ret) {
		LOG(MtkISP7, Error) << "Fail to import buffers";
		return ret;
	}

	return ret;
}

int CamSysDevice::configureVideo(V4L2VideoDevice *device, const PixelFormat &format,
				 Size resolution, V4L2Subdevice *rawPipe,
				 int pad, uint32_t mbus)
{
	int ret = setFormat(rawPipe, pad, mbus, resolution);
	ret |= setFormat(device, format, resolution);
	return ret;
}

int CamSysDevice::setupSeninf(bool enable)
{
	ControlList ctrl(seninf_->controls());
	ctrl.set(V4L2_CID_MTK_SENINF_S_STREAM, (int32_t)enable);

	return seninf_->setControls(&ctrl);
}

int CamSysDevice::setupLinks(bool enable)
{
	MediaLink *link = nullptr;

	link = media_->link(sensor_->entity(), PAD_SENSOR_OUT,
			    seninf_->entity(), PAD_SENINF_IN);
	int ret = link->setEnabled(enable);

	link = media_->link(seninf_->entity(), PAD_SENINF_OUT,
			    videoHub_->entity(), PAD_RAW_IN);
	ret |= link->setEnabled(enable);

	return ret;
}

void CamSysDevice::bufferReady(FrameBuffer *buffer)
{
	bool foundRequest = false;
	for (auto iter = pendingRequests_.begin();
	     iter != pendingRequests_.end(); iter++) {
		Request *request = iter->request;
		if (request->main != buffer &&
		    request->yuvo1 != buffer &&
		    request->yuvo2 != buffer &&
		    request->me != buffer &&
		    request->faceDetect != buffer &&
		    request->tuning != buffer &&
		    request->statistics0 != buffer &&
		    request->statistics1 != buffer) {
			continue;
		}

		foundRequest = true;
		if (--iter->pending > 0)
			continue;

		media_->reInitRequest(iter->mediaRequest);
		mediaRequestPool_.put(iter->mediaRequest);

		completedRequests_.emplace_back(request);
		requestCompleted.emit(request);

		pendingRequests_.erase(iter);
		break;
	}

	ASSERT(foundRequest == true);
}

} /* namespace libcamera */
