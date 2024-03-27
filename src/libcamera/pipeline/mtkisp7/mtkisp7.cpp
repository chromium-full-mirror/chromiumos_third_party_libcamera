/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * mtkisp7.cpp - Pipeline handler for Mediatek MtkISP7
 */

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>
#include <sys/resource.h>
#include <sys/sysinfo.h>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/controls.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/gyro_sensor.h"
#include "libcamera/internal/mailbox.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/task_scheduler.h"

#include "camsys/camsys.h"
#include "camsys/capture.h"
#include "hal3a/aaa.h"
#include "hal3a/hal_3a.h"
#include "halisp/ITuningDataProvider.h"
#include "halisp/hal_isp.h"
#include "halisp/lpnr_tun.h"
#include "halisp/mcnr_tun.h"
#include "halisp/mfnr_tun.h"
#include "imgsys/imgsys.h"
#include "imgsys/lpnr.h"
#include "imgsys/mcnr.h"
#include "imgsys/mfnr.h"
#include "libfdft_lib/faces.h"
#include "pipeline/mtkisp7/face_detect/detector.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"
#include "sensor/sensor_info.h"
#include "utils/history.h"

namespace libcamera {
LOG_DEFINE_CATEGORY(MtkISP7)

static const ControlInfoMap::Map MtkISP7Controls = {
	{ &controls::draft::PipelineDepth, ControlInfo(8, 8, 8) },
	{ &controls::draft::NoiseReductionMode, ControlInfo(controls::draft::NoiseReductionModeValues) },
};

static const std::vector<int> kMainThreadCpuAffinity{ 6, 7 };

//TODO implement strategy to choose between mfnr and lpnr
static const bool useMfnr = false;
static const bool useLpnr = true;

enum MtkISP7TaskGroup {
	SofGroup = 0,
	CaptureQueueGroup,
	CaptureDequeueGroup,
	AAGroup,
	AFGroup,
	MeAGroup,
	MeBGroup,
	MeATunGroup,
	MeBTunGroup,
	TrGroup,
	TrTunGroup,
	XtrGroup,
	Dip1Group,
	Dip2Group,
	DipTunGroup,
	LpnrDipGroup,
	LpnrTunXtrTaskGroup,
	LpnrTunDipTaskGroup,
	AieFaceDetectionGroup,
	AieFaceToneClassificationGroup,
	AieParseGroup,
	BssTaskGroup,
	BfbldTaskGroup,
	McdsF1Group,
	BfmeGroup,
	SwmeGroup,
	DsGroup,
	DsVbiGroup,
	MsbldGroup,
	AfbldGroup,
	BssTunTaskGroup,
	BfbldTunTaskGroup,
	McdsF1TunGroup,
	BfmeTunGroup,
	SwmeTunGroup,
	DsTunGroup,
	DsVbiTunGroup,
	MsbldTunGroup,
	AfbldTunGroup,
	CompleteGroup,
};

static const std::map<MtkISP7TaskGroup, std::string> kGroupName{
	{ SofGroup, "SofGroup" },
	{ CaptureQueueGroup, "CaptureQueueGroup" },
	{ CaptureDequeueGroup, "CaptureDequeueGroup" },
	{ AAGroup, "AAGroup" },
	{ AFGroup, "AFGroup" },
	{ MeAGroup, "MeAGroup" },
	{ MeBGroup, "MeBGroup" },
	{ MeATunGroup, "MeATunGroup" },
	{ MeBTunGroup, "MeBTunGroup" },
	{ TrTunGroup, "TrTunGroup" },
	{ TrGroup, "TrGroup" },
	{ XtrGroup, "XtrGroup" },
	{ DipTunGroup, "DipTunGroup" },
	{ Dip1Group, "Dip1Group" },
	{ Dip2Group, "Dip2Group" },
	{ LpnrTunXtrTaskGroup, "LpnrTunXtrTaskGroup" },
	{ LpnrTunDipTaskGroup, "LpnrTunDipTaskGroup" },
	{ AieFaceDetectionGroup, "AieFaceDetectionGroup" },
	{ AieFaceToneClassificationGroup, "AieFaceToneClassificationGroup" },
	{ BssTaskGroup, "BssTaskGroup" },
	{ BfbldTaskGroup, "BfbldTaskGroup" },
	{ McdsF1Group, "McdsF1Group" },
	{ BfmeGroup, "BfmeGroup" },
	{ SwmeGroup, "SwmeGroup" },
	{ DsGroup, "DsGroup" },
	{ DsVbiGroup, "DsVbiGroup" },
	{ MsbldGroup, "MsbldGroup" },
	{ AfbldGroup, "AfbldGroup" },
	{ BssTunTaskGroup, "BssTunTaskGroup" },
	{ BfbldTunTaskGroup, "BfbldTunTaskGroup" },
	{ McdsF1TunGroup, "McdsF1TunGroup" },
	{ BfmeTunGroup, "BfmeTunGroup" },
	{ SwmeTunGroup, "SwmeTunGroup" },
	{ DsTunGroup, "DsTunGroup" },
	{ DsVbiTunGroup, "DsVbiTunGroup" },
	{ MsbldTunGroup, "MsbldTunGroup" },
	{ AfbldTunGroup, "AfbldTunGroup" },
	{ CompleteGroup, "CompleteGroup" },
};

class CompleteRequestTask : public Task
{
public:
	CompleteRequestTask(Scheduler *scheduler, const std::string &id,
			    Request *request, uint32_t internalRequestId,
			    uint32_t camSysMetaRequestId, PipelineHandler *pipe,
			    OnDeviceTuner *odt, FaceDetector *faceDetector,
			    SharedMailBox<AaaIspExchange> aaaIspExchange,
			    Hal3A *hal3A, Feature feature);

	virtual void run() override final;

private:
	PipelineHandler *pipe_;
	Request *request_;
	uint32_t internalRequestId_;
	uint32_t camSysMetaRequestId_;
	FaceDetector *faceDetector_;
	OnDeviceTuner *onDeviceTuner_;
	SharedMailBox<AaaIspExchange> aaaIspExchange_;
	Hal3A *hal3A_;
	Feature feature_;
};

CompleteRequestTask::CompleteRequestTask(
	Scheduler *scheduler,
	const std::string &id,
	Request *request,
	uint32_t internalRequestId,
	uint32_t camSysMetaRequestId,
	PipelineHandler *pipe,
	OnDeviceTuner *odt,
	FaceDetector *faceDetector,
	SharedMailBox<AaaIspExchange> aaaIspExchange,
	Hal3A *hal3A, Feature feature)
	: Task(scheduler, id), pipe_(pipe), request_(request),
	  internalRequestId_(internalRequestId),
	  camSysMetaRequestId_(camSysMetaRequestId),
	  faceDetector_(faceDetector), onDeviceTuner_(odt),
	  aaaIspExchange_(aaaIspExchange), hal3A_(hal3A), feature_(feature)
{
}

struct CaptureResult {
	SharedMailBox<InfoFrame> tuningOutput;
	SharedMailBox<std::pair<uint32_t, uint32_t>> exposureAndGainOutput;
	SharedMailBox<AaaIspExchange> aaaIspExchange;
};

class MtkISP7CameraData : public Camera::Private
{
public:
	MtkISP7CameraData(PipelineHandler *pipe, CamSysDevice *camSysDev,
			  ImgSysDevice *imgSysDev, GyroSensor *gyroSensor, OnDeviceTuner *odt,
			  FaceDetector *faceDetector, DmaHeap *dmaHeap, Hal3A *hal3A, HalIsp *halIsp,
			  int sensor_idx)
		: Camera::Private(pipe), camSysDev_(camSysDev),
		  imgSysDev_(imgSysDev), gyroSensor_(gyroSensor),
		  captureManager(odt), mcnrManager(imgSysDev, dmaHeap, odt),
		  lpnrManager(imgSysDev, dmaHeap, odt),
		  mfnrManager(imgSysDev, dmaHeap, odt),
		  lpnrTunManager(dmaHeap, halIsp, odt),
		  mcnrTunManager(dmaHeap, halIsp, odt),
		  mfnrTunManager(dmaHeap, halIsp, odt),
		  onDeviceTuner_(odt),
		  faceDetector_(faceDetector), dmaHeap_(dmaHeap), hal3A_(hal3A),
		  halIsp_(halIsp), captureResult_(5), sensor_idx_(sensor_idx),
		  control_cache_(nullptr)
	{
	}

	int configure(CameraConfiguration *c);
	int queueRequest(Request *request);

	int start(const ControlList *controls);
	void stopDevice();
	void releaseDevice();

	void frameStart(uint32_t sequence);

	bool is3aControlChanged(std::shared_ptr<ControlList> controls_cur, std::shared_ptr<ControlList> controls_cache);

	std::tuple<QueueTask *, DequeueTask *, SofTask *,
		   AATask *, AFTask *, uint32_t>
	makeTasks(const std::string &id, Request *request,
		  CaptureFrames &captureFrames, uint32_t internalRequestId);
	void setTasksDependencies(QueueTask *taskQBuf, DequeueTask *taskDQBuf,
				  SofTask *sofTask, AATask *aaTask,
				  AFTask *afTask);

	Stream video1Stream_;
	Stream video2Stream_;
	Stream still1Stream_;
	Stream still2Stream_;

	uint32_t frameSequence_ = 0;
	std::list<SofTask *> pendingSofTasks_;

	Size sensorFullSize_;
	CamSysDevice *camSysDev_;
	ImgSysDevice *imgSysDev_;
	GyroSensor *gyroSensor_;

	CaptureTasksManager captureManager;
	Hal3AManager hal3AManager_;

	MCNRPrevOutput mcnrPrev;
	McnrTasksManager mcnrManager;
	LpnrTasksManager lpnrManager;
	MfnrTasksManager mfnrManager;

	LpnrTunTasksManager lpnrTunManager;
	McnrTunManager mcnrTunManager;
	MfnrTunManager mfnrTunManager;

	OnDeviceTuner *onDeviceTuner_;
	FaceDetector *faceDetector_;
	DmaHeap *dmaHeap_;

	Hal3A *hal3A_;
	HalIsp *halIsp_;

	History<CaptureResult> captureResult_;

	uint32_t requestCount_ = 0;

	std::array<SharedMailBox<InfoFrame>, 8> captureRawQueue;
	std::array<SharedMailBox<InfoFrame>, 8> previewQueue;
	int captureRawQueue_idx = -1;

	int getSensorIdx() { return sensor_idx_; }

private:
	int sensor_idx_;
	std::shared_ptr<ControlList> control_cache_;
};

class MtkISP7CameraConfiguration : public CameraConfiguration
{
public:
	MtkISP7CameraConfiguration(MtkISP7CameraData *data);
	Status validate() override;

private:
	/*
	 * The MtkISP7CameraData instance is guaranteed to be valid as long as the
	 * corresponding Camera instance is valid. In order to borrow a
	 * reference to the camera data, store a new reference to the camera.
	 */
	const MtkISP7CameraData *data_;
};

class PipelineHandlerMtkISP7 : public PipelineHandler
{
public:
	PipelineHandlerMtkISP7(CameraManager *manager);

	std::unique_ptr<CameraConfiguration> generateConfiguration(Camera *camera,
								   Span<const StreamRole> roles) override;
	int configure(Camera *camera, CameraConfiguration *config) override;

	int exportFrameBuffers(Camera *camera, Stream *stream,
			       std::vector<std::unique_ptr<FrameBuffer>> *buffers) override;

	int start(Camera *camera, const ControlList *controls) override;
	void stopDevice(Camera *camera) override;
	void releaseDevice(Camera *camera) override;

	int queueRequestDevice(Camera *camera, Request *request) override;

	bool match(DeviceEnumerator *enumerator) override;

	void adjustRLimit();

	std::unique_ptr<CategorizedScheduler<MtkISP7TaskGroup>> scheduler_;
	std::unique_ptr<DmaHeap> dmaHeap_;

	OnDeviceTuner onDeviceTuner_;

	MediaDevice *camSysMedia_;
	CamSysDevice camSysDev_[2];
	GyroSensor gyroSensor_;

	std::unique_ptr<Hal3A> hal3A_[2];
	HalIsp halIsp_[2];

	MediaDevice *imgSysMedia_;
	ImgSysDevice imgSysDev_;

	MediaDevice *aieMedia_;
	AieDevice aieDev_;

	FaceDetector faceDetector_;

private:
	MtkISP7CameraData *cameraData(Camera *camera)
	{
		return static_cast<MtkISP7CameraData *>(camera->_d());
	}
};

void CompleteRequestTask::run()
{
	ControlList metadata;
	const auto &scalerCrop = request_->controls().get(controls::ScalerCrop);
	if (scalerCrop) {
		metadata.set(controls::ScalerCrop, *scalerCrop);
	}

	metadata.set(controls::draft::PipelineDepth, 8);

	int32_t testPatternMode = controls::draft::TestPatternModeOff;
	const auto &testPatternControl = request_->controls().get(controls::draft::TestPatternMode);
	if (testPatternControl)
		testPatternMode = *testPatternControl;

	metadata.set(controls::draft::TestPatternMode, testPatternMode);

	// todo(yerlandinata, before CTS): check if face metadata is requested
	ControlList faceControls;
	faceDetector_->getLatestFaceControls(faceControls);

	metadata.merge(faceControls);

	if (aaaIspExchange_->valid()) {
		AaaIspExchange aaaIspExchange = aaaIspExchange_->get();
		metadata.merge(aaaIspExchange.aaaMetadata);
		onDeviceTuner_->writeStillCaptureDebugMetadata(
			metadata,
			hal3A_->resultHistory_.query(camSysMetaRequestId_),
			feature_);
	}

	pipe_->completeMetadata(request_, metadata);

	for (auto it : request_->buffers()) {
		FrameBuffer *buffer = it.second;
		pipe_->completeBuffer(request_, buffer);
	}

	onDeviceTuner_->notifyRequestEnd(internalRequestId_);
	pipe_->completeRequest(request_);
	Task::notifyDone();
}

MtkISP7CameraConfiguration::MtkISP7CameraConfiguration(MtkISP7CameraData *data)
	: CameraConfiguration()
{
	data_ = data;
}

CameraConfiguration::Status MtkISP7CameraConfiguration::validate()
{
	static const std::vector<Size> resolutions = {
		{ 320, 240 },
		{ 640, 360 },
		{ 640, 480 },
		{ 1280, 720 },
		{ 1280, 960 },
		{ 1440, 1080 },
		{ 1920, 1080 },
		{ 1920, 1440 },
		{ 2560, 1440 },
		{ 2560, 1920 },
	};

	const Stream *vidStreams[2]{
		&data_->video1Stream_,
		&data_->video2Stream_
	};

	const Stream *stillStreams[2]{
		&data_->still1Stream_,
		&data_->still2Stream_
	};

	int videoCnt = 0;
	int stillCnt = 0;
	for (StreamConfiguration &cfg : config_) {
		/* Allows the predefined resolutions plus the sensor size */
		if (!std::count(resolutions.begin(), resolutions.end(), cfg.size) &&
		    cfg.size != data_->sensorFullSize_)
			return Invalid;

		cfg.pixelFormat = formats::NV12;
		cfg.bufferCount = CamSysDevice::kBufferCount;

		const PixelFormatInfo &info = PixelFormatInfo::info(cfg.pixelFormat);
		cfg.stride = info.stride(cfg.size.width, 0, 64);

		switch (cfg.role) {
		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording:
			if (videoCnt >= 2) {
				LOG(MtkISP7, Error)
					<< "Support only 2 Preview/Video streams";
				return Invalid;
			}
			cfg.setStream(const_cast<Stream *>(vidStreams[videoCnt++]));
			break;
		case StreamRole::StillCapture:
			if (stillCnt >= 2) {
				LOG(MtkISP7, Error)
					<< "Support only 2 StillCapture streams";
				return Invalid;
			}
			cfg.setStream(const_cast<Stream *>(stillStreams[stillCnt++]));
			break;
		default:
			LOG(MtkISP7, Error) << "Invalid StreamRole " << cfg.role;
			return Invalid;
		}
	}

	return Valid;
}

PipelineHandlerMtkISP7::PipelineHandlerMtkISP7(CameraManager *manager)
	: PipelineHandler(manager),
	  camSysDev_{ { &onDeviceTuner_ }, { &onDeviceTuner_ } },
	  halIsp_{ { &onDeviceTuner_ }, { &onDeviceTuner_ } },
	  imgSysDev_(&onDeviceTuner_), faceDetector_(&aieDev_)
{
	scheduler_ = std::make_unique<CategorizedScheduler<MtkISP7TaskGroup>>(kGroupName);
	dmaHeap_ = std::make_unique<DmaHeap>();

	adjustRLimit();

	thread()->setThreadAffinity(kMainThreadCpuAffinity);
}

std::unique_ptr<CameraConfiguration>
PipelineHandlerMtkISP7::generateConfiguration(Camera *camera, Span<const StreamRole> roles)
{
	MtkISP7CameraData *data = cameraData(camera);
	std::unique_ptr<MtkISP7CameraConfiguration> config =
		std::make_unique<MtkISP7CameraConfiguration>(data);

	if (roles.empty())
		return config;

	Size maxSize = data->sensorFullSize_;
	PixelFormat pixelFormat = formats::NV12;

	std::map<PixelFormat, std::vector<SizeRange>> streamFormats = { { pixelFormat, { { CamSysDevice::kMinResolution, maxSize } } } };
	StreamFormats formats(streamFormats);

	Stream *streams[2]{
		&data->video1Stream_,
		&data->video2Stream_
	};

	int videoCnt = 0;
	for (const StreamRole role : roles) {
		StreamConfiguration cfg(formats);
		cfg.size = maxSize;
		cfg.pixelFormat = pixelFormat;
		cfg.bufferCount = CamSysDevice::kBufferCount;

		switch (role) {
		case StreamRole::StillCapture:
			cfg.setStream(&data->still1Stream_);
			cfg.role = StreamRole::StillCapture;
			break;

		case StreamRole::Viewfinder:
		case StreamRole::VideoRecording:
			if (videoCnt >= 2) {
				LOG(MtkISP7, Error)
					<< "Support only 2 Preview/Video streams";
				return nullptr;
			}
			cfg.setStream(streams[videoCnt++]);
			cfg.role = role;
			break;

		case StreamRole::Raw:
		default:
			LOG(MtkISP7, Error)
				<< "Requested stream role not supported: " << role;
			return nullptr;
		}

		config->addConfiguration(cfg);
	}

	if (config->validate() == CameraConfiguration::Invalid)
		return {};

	return config;
}

int PipelineHandlerMtkISP7::exportFrameBuffers([[maybe_unused]] Camera *camera,
					       [[maybe_unused]] Stream *stream,
					       [[maybe_unused]] std::vector<std::unique_ptr<FrameBuffer>> *buffers)
{
	/* todo: Generate frame buffers by DMA heap */
	return -EINVAL;
}

void PipelineHandlerMtkISP7::adjustRLimit()
{
	struct sysinfo info;

	if (sysinfo(&info) != 0) {
		perror("sysinfo");
		exit(EXIT_FAILURE);
	}

	struct rlimit rlim;

	if (getrlimit(RLIMIT_NOFILE, &rlim) != 0) {
		perror("getrlimit");
		exit(EXIT_FAILURE);
	}

	if (rlim.rlim_cur == RLIM_INFINITY)
		LOG(MtkISP7, Info) << "Current file descriptor limit: unlimited ";
	else
		LOG(MtkISP7, Info) << "Current file descriptor limit:" << rlim.rlim_cur;

	if (rlim.rlim_max == RLIM_INFINITY)
		LOG(MtkISP7, Info) << "Maximum file descriptor limit: unlimited:";
	else
		LOG(MtkISP7, Info) << "Maximum file descriptor limit: " << rlim.rlim_max;

	// Increase the soft limit to 2048
	rlim.rlim_cur = 2048;

	if (setrlimit(RLIMIT_NOFILE, &rlim) != 0) {
		perror("setrlimit");
		exit(EXIT_FAILURE);
	}

	if (getrlimit(RLIMIT_NOFILE, &rlim) != 0) {
		perror("getrlimit");
		exit(EXIT_FAILURE);
	}
}

int PipelineHandlerMtkISP7::configure(Camera *camera, CameraConfiguration *c)
{
	return cameraData(camera)->configure(c);
}

int PipelineHandlerMtkISP7::start(Camera *camera, [[maybe_unused]] const ControlList *controls)
{
	return cameraData(camera)->start(controls);
}

void PipelineHandlerMtkISP7::stopDevice(Camera *camera)
{
	cameraData(camera)->stopDevice();
}

void PipelineHandlerMtkISP7::releaseDevice(Camera *camera)
{
	cameraData(camera)->releaseDevice();
}

int PipelineHandlerMtkISP7::queueRequestDevice(Camera *camera, Request *request)
{
	return cameraData(camera)->queueRequest(request);
}

bool PipelineHandlerMtkISP7::match(DeviceEnumerator *enumerator)
{
	onDeviceTuner_.initialize();

	DeviceMatch camSysDM("mtk-cam");
	camSysDM.add("mtk-cam raw-0");
	camSysDM.add("mtk-cam raw-1");

	camSysMedia_ = acquireMediaDevice(enumerator, camSysDM);

	if (!camSysMedia_)
		return false;

	if (camSysMedia_->disableLinks())
		return false;

	int errGyro = gyroSensor_.init(GyroSensor::Location::kLid);
	if (errGyro) {
		LOG(MtkISP7, Warning) << "No gyroscope available";
	}

	DeviceMatch imgSysDM("camera-dip");
	imgSysDM.add("MTK-ISP-DIP-V4L2");

	imgSysMedia_ = acquireMediaDevice(enumerator, imgSysDM);

	if (!imgSysMedia_)
		return false;

	if (imgSysMedia_->disableLinks())
		return false;

	DeviceMatch aieDM("mtk-aie-5.3");
	aieMedia_ = acquireMediaDevice(enumerator, aieDM);
	if (!aieMedia_) {
		LOG(MtkISP7, Error) << "Failed to match AIE Media";
		return false;
	}
	if (faceDetector_.init(aieMedia_, dmaHeap_.get()) != 0) {
		LOG(MtkISP7, Error) << "Failed to init AIE device";
		return false;
	}
	hal3A_[0] = std::make_unique<Hal3A>(0, &halIsp_[0], &onDeviceTuner_);
	hal3A_[1] = std::make_unique<Hal3A>(1, &halIsp_[1], &onDeviceTuner_);

	halIsp_[0].init(0, 1, hal3A_[0].get());
	halIsp_[1].init(1, 2, hal3A_[1].get());

	uint32_t sensorCnt = 0;
	for (unsigned int i = 0; i < 2; i++) {
		if (camSysDev_[i].init(camSysMedia_, i))
			continue;

		ControlList properties = camSysDev_[i].properties();

		// Fill ControlInfoMap
		ControlInfoMap::Map controls = MtkISP7Controls;

		// Increase the pipeline depth when ODT is enabled to ensure
		// the FPS is consistent with CCA, since it needs every
		// request to preserve enough pending 3A tasks according to
		// CaptureTasksManager::kRawMetaDelay.
		if (onDeviceTuner_.isEnabled())
			controls[&controls::draft::PipelineDepth] = ControlInfo(10, 10, 10);

		// todo: Fix the frame duration to 30fps for now. It should be
		// updated on stream configuration
		controls[&controls::FrameDurationLimits] = ControlInfo((int64_t)33'333,
								       (int64_t)66'666,
								       (int64_t)33'333);

		// todo: Assign correct crop range
		const Size &pixelArraySize = properties.get(properties::PixelArraySize).value_or(Size{});
		Rectangle maxCrop = Rectangle{ pixelArraySize };
		controls[&controls::ScalerCrop] = ControlInfo(maxCrop, maxCrop, maxCrop);

		// todo: Assigned the test pattern unconditionally due to the
		// sensor driver is not ready for test patterns. Use the test
		// patterns reported by the sensor when the driver is ready.
		std::vector<ControlValue> patterns;
		patterns.emplace_back(static_cast<int32_t>(controls::draft::TestPatternModeOff));
		patterns.emplace_back(static_cast<int32_t>(controls::draft::TestPatternModeColorBars));
		controls[&controls::draft::TestPatternMode] = ControlInfo(patterns);

		std::vector<ControlValue> supportedFaceDetectModes{
			static_cast<uint8_t>(controls::FaceDetectModeOff),
			static_cast<uint8_t>(controls::FaceDetectModeSimple)
		};
		controls[&controls::FaceDetectMode] = ControlInfo(supportedFaceDetectModes);
		controls[&controls::AeMode] = ControlInfo(controls::AeModeValues);
		controls[&controls::AeLocked] = ControlInfo(true, false);

		controls[&controls::AwbMode] = ControlInfo(controls::AwbModeValues);
		controls[&controls::AwbEnable] = ControlInfo(true, false);
		controls[&controls::AwbLocked] = ControlInfo(true, false);
		controls[&controls::draft::AePrecaptureTrigger] = ControlInfo(controls::draft::AePrecaptureTriggerValues);

		controls[&controls::FrameDuration] = ControlInfo(
			static_cast<int64_t>(33'333'333),
			static_cast<int64_t>(66'333'333),
			static_cast<int64_t>(33'333'333));

		// For now these two controls are ignored.
		// However, because MTK 3A algo is configured to prioritize
		// human face, ignoring any combination of these two controls
		// does not violate Android Camera API specs.
		std::vector<ControlValue> supported3AModes{
			static_cast<uint8_t>(controls::Mode3AAuto),
			static_cast<uint8_t>(controls::Mode3AUseSceneMode),
		};
		controls[&controls::Mode3A] = ControlInfo(supported3AModes);

		std::vector<ControlValue> supportedSceneModes{
			static_cast<uint8_t>(controls::SceneModeDisabled),
			static_cast<uint8_t>(controls::SceneModeFacePriority),
		};
		controls[&controls::SceneMode] = ControlInfo(supportedSceneModes);

		// Create CameraData
		std::unique_ptr<MtkISP7CameraData> data =
			std::make_unique<MtkISP7CameraData>(
				this, &camSysDev_[i], &imgSysDev_,
				errGyro ? nullptr : &gyroSensor_,
				&onDeviceTuner_, &faceDetector_,
				dmaHeap_.get(), hal3A_[i].get(), &halIsp_[i], i);

		std::set<Stream *> streams = { &data->video1Stream_,
					       &data->video2Stream_,
					       &data->still1Stream_ };

		data->sensorFullSize_ = pixelArraySize;
		data->properties_ = properties;
		data->controlInfo_ = ControlInfoMap(std::move(controls), controls::controls);

		// Create and register the Camera
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), camSysDev_[i].cameraId(), streams);
		registerCamera(std::move(camera));

		ImagiqAdapter::sensorIdMap.emplace(
			camSysDev_[i].cameraId(),
			NSCam::TuningUtils::eSensorId(i));

		sensorCnt++;
		LOG(MtkISP7, Info) << "Registered Camera[" << camSysDev_[i].cameraId() << "]";
	}

	if (sensorCnt < 2)
		return false;

	// TODO: Only init imgsys when there is sensor detected.
	// A temporary hack for factory testing. Find a more proper way to
	// handle this case.
	imgSysDev_.init(imgSysMedia_, dmaHeap_.get());
	SensorInfo::add_sensor(camSysDev_, 2);

	return true;
}

int MtkISP7CameraData::start([[maybe_unused]] const ControlList *controls)
{
	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());
	auto *scheduler = pipeline->scheduler_.get();
	control_cache_.reset();
	camSysDev_->frameStart().disconnect(this);
	camSysDev_->frameStart().connect(this, &MtkISP7CameraData::frameStart);

	camSysDev_->start();
	imgSysDev_->start();

	hal3A_->start();
	// Needs to be called after |hal3A_->start()|, as it uses AF result.
	// Needs to be called after |camSysDev_->start()|, as it uses lens.
	hal3AManager_.start();

	mcnrManager.start();
	lpnrManager.start();
	faceDetector_->start();

	if (useMfnr)
		mfnrManager.start();

	if (gyroSensor_)
		gyroSensor_->startReading(30); // Assume FPS == 30

	scheduler->schedule();
	return 0;
}

std::tuple<QueueTask *, DequeueTask *, SofTask *, AATask *, AFTask *, uint32_t>
MtkISP7CameraData::makeTasks(const std::string &id, Request *request,
			     CaptureFrames &captureFrames,
			     uint32_t internalRequestId)
{
	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());
	auto *scheduler = pipeline->scheduler_.get();

	captureManager.makeCaptureFrames(captureFrames);

	if (internalRequestId >= CaptureTasksManager::kExposureAndGainDelay) {
		uint32_t aaRequestId = internalRequestId - CaptureTasksManager::kExposureAndGainDelay;
		CaptureResult *aaCaptureResult = captureResult_.query(aaRequestId);
		captureFrames.exposureAndGain = aaCaptureResult->exposureAndGainOutput;
	} else {
		captureFrames.exposureAndGain = makeMailBox<std::pair<uint32_t, uint32_t>>();
		captureFrames.exposureAndGain->put(std::make_pair(0, 0), nullptr);
	}

	uint32_t camSysMetaRequestId = 0;
	if (internalRequestId >= CaptureTasksManager::kRawMetaDelay) {
		camSysMetaRequestId = internalRequestId - CaptureTasksManager::kRawMetaDelay;
		CaptureResult *aaCaptureResult = captureResult_.query(camSysMetaRequestId);
		captureFrames.tuning = aaCaptureResult->tuningOutput;
	} else {
		auto [dummyId, dummyTuning] = hal3AManager_.getDummyTuning();
		camSysMetaRequestId = dummyId;
		captureFrames.tuning = dummyTuning;
	}

	CaptureResult captureResult;
	captureResult.tuningOutput = captureFrames.tuningOutput;
	captureResult.exposureAndGainOutput = captureFrames.exposureAndGainOutput;
	captureResult.aaaIspExchange = captureFrames.aaaIspExchange;

	captureResult_.add(internalRequestId, captureResult);

	auto [taskQBuf, taskDQBuf, sofTask] = captureManager.makeCaptureTasks(
		scheduler, id, request, captureFrames, internalRequestId);

	auto [aaTask, afTask] = hal3AManager_.make3ATasks(
		scheduler, request, captureFrames, internalRequestId,
		camSysMetaRequestId, faceDetector_);

	setTasksDependencies(taskQBuf, taskDQBuf, sofTask, aaTask, afTask);

	return std::make_tuple(taskQBuf, taskDQBuf, sofTask,
			       aaTask, afTask, camSysMetaRequestId);
}

void MtkISP7CameraData::setTasksDependencies(
	QueueTask *taskQBuf, DequeueTask *taskDQBuf, SofTask *sofTask,
	AATask *aaTask, AFTask *afTask)
{
	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());
	auto *scheduler = pipeline->scheduler_.get();

	Scheduler::precede(sofTask, taskDQBuf);
	Scheduler::precede(taskQBuf, taskDQBuf);
	Scheduler::precede(taskDQBuf, aaTask);
	if (afTask) {
		Scheduler::precede(taskDQBuf, afTask);
	}

	scheduler->succeedPrevTaskByStep(CaptureQueueGroup, 0, taskQBuf);
	scheduler->succeedPrevTaskByStep(CaptureDequeueGroup, 0, taskDQBuf);
	scheduler->succeedPrevTaskByStep(AAGroup, 0, aaTask);
	if (afTask) {
		scheduler->succeedPrevTaskByStep(AFGroup, 0, afTask);
	}

	scheduler->succeedPrevTaskByStep(AAGroup, CaptureTasksManager::kExposureAndGainDelay - 1, sofTask);
	scheduler->succeedPrevTaskByStep(AAGroup, CaptureTasksManager::kRawMetaDelay - 1, taskQBuf);

	/* At most 5 request can be queued into CamSys */
	scheduler->succeedPrevTaskByStep(CaptureDequeueGroup, 4, taskQBuf);

	scheduler->queueTask(sofTask, SofGroup);
	scheduler->queueTask(taskQBuf, CaptureQueueGroup);
	scheduler->queueTask(taskDQBuf, CaptureDequeueGroup);
	scheduler->queueTask(aaTask, AAGroup);
	if (afTask) {
		scheduler->queueTask(afTask, AFGroup);
	}

	pendingSofTasks_.push_back(sofTask);
}

void MtkISP7CameraData::stopDevice()
{
	camSysDev_->frameStart().disconnect(this);

	captureRawQueue_idx = -1;
	captureRawQueue.fill(makeMailBox<InfoFrame>());
	previewQueue.fill(makeMailBox<InfoFrame>());

	camSysDev_->stop();
	imgSysDev_->stop();

	/* Release the transient frames for MCNR before stopping mcnrManger */
	mcnrPrev = {};

	mcnrManager.stop();
	lpnrManager.stop();

	if (useMfnr)
		mfnrManager.stop();

	faceDetector_->stop();

	if (gyroSensor_)
		gyroSensor_->stopReading();
	frameSequence_ = 0;
}

void MtkISP7CameraData::releaseDevice()
{
	/* Release the transient frames of MCNR */
	mcnrPrev = {};

	captureManager.releaseBuffers();
	hal3AManager_.releaseBuffers();
	mcnrManager.releaseBuffers();
	lpnrManager.releaseBuffers();

	lpnrTunManager.releaseBuffers();
	mcnrTunManager.releaseBuffers();

	if (useMfnr) {
		mfnrManager.releaseBuffers();
		mfnrTunManager.releaseBuffers();
	}
}

/*
 * \brief Handle the start of frame exposure signal
 * \param[in] sequence The sequence number of frame
 */
void MtkISP7CameraData::frameStart(uint32_t sequence)
{
	if (sequence <= frameSequence_) {
		LOG(MtkISP7, Debug) << "Underrun dropping " << sequence;
		return;
	}

	while (frameSequence_ < sequence) {
		++frameSequence_;

		if (pendingSofTasks_.empty()) {
			LOG(MtkISP7, Error) << "Frame Start No Sof tasks when camsys writes a frame " << frameSequence_;
			continue;
		}

		SofTask *task = pendingSofTasks_.front();
		pendingSofTasks_.pop_front();

		task->trigger();
	}
}

int MtkISP7CameraData::configure(CameraConfiguration *c)
{
	Size camsysYuvSize;
	Size video1 = Size{ 0, 0 };
	Size video2 = Size{ 0, 0 };
	Size still1 = Size{ 0, 0 };
	Size still2 = Size{ 0, 0 };

	/* Only cover the video resolution */
	for (auto &cfg : *c) {
		if (cfg.stream() == &video1Stream_)
			video1 = cfg.size;
		else if (cfg.stream() == &video2Stream_)
			video2 = cfg.size;
		else if (cfg.stream() == &still1Stream_)
			still1 = cfg.size;
		else if (cfg.stream() == &still2Stream_)
			still2 = cfg.size;
		else
			return -EINVAL;
	}

	camsysYuvSize.expandTo(video1);
	camsysYuvSize.expandTo(video2);

	/* Only support 4:3 resolution as output of CamSys */
	std::vector<Size> supportedSize = {
		{ 1280, 960 },
		{ 1440, 1080 },
		{ 1600, 1200 },
		{ 1920, 1440 },
		{ 2560, 1920 },
	};

	/* Support sensor full size. */
	supportedSize.push_back(sensorFullSize_);

	/* Find the smallest supported size which covers video streams */
	bool found = false;
	for (auto &size : supportedSize) {
		if (size >= camsysYuvSize) {
			camsysYuvSize = size;
			found = true;
			break;
		}
	}

	if (!found) {
		LOG(MtkISP7, Error)
			<< "Video/Preview resolution is larger than the limit "
			<< supportedSize.back();
		return -EINVAL;
	}

	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());

	onDeviceTuner_->configure(camSysDev_->cameraId(), camSysDev_->getIndex());
	camSysDev_->configure(sensorFullSize_, camsysYuvSize);
	hal3A_->configure(camsysYuvSize);
	halIsp_->configure(video1 > video2 ? video1 : video2,
			   still1 > still2 ? still1 : still2);
	captureManager.configure(dmaHeap_, camSysDev_, pipeline, sensorFullSize_, camsysYuvSize);
	faceDetector_->configure(sensorFullSize_);
	hal3AManager_.configure(dmaHeap_, camSysDev_, hal3A_, onDeviceTuner_, gyroSensor_);

	imgSysDev_->configure();
	mcnrManager.configure(camsysYuvSize, video1, video2);
	lpnrManager.configure(sensorFullSize_, still1, still2);
	lpnrTunManager.configure(sensorFullSize_, still1, still2);
	mcnrTunManager.configure(camsysYuvSize, video1, video2);

	if (useMfnr) {
		mfnrManager.configure(sensorFullSize_,
				      still1, still2,
				      video1, video2,
				      faceDetector_,
				      sensor_idx_);
		mfnrTunManager.configure(sensorFullSize_, still1, still2);
	}

	return 0;
}

int MtkISP7CameraData::queueRequest(Request *request)
{
	FrameBuffer *video1Buffer = request->findBuffer(&video1Stream_);
	FrameBuffer *video2Buffer = request->findBuffer(&video2Stream_);
	FrameBuffer *still1Buffer = request->findBuffer(&still1Stream_);
	FrameBuffer *still2Buffer = request->findBuffer(&still2Stream_);

	if (!video1Buffer && !video2Buffer && !still1Buffer && !still2Buffer)
		return -EINVAL;

	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());
	auto *scheduler = pipeline->scheduler_.get();

	std::string sequence = std::to_string(request->sequence());

	// TODO: Implement the padding condition for per-frame control
	std::shared_ptr<ControlList> controls_cur = std::make_shared<ControlList>(request->controls());

	bool aaControlChanged = is3aControlChanged(controls_cur, control_cache_);

	control_cache_ = controls_cur;

	bool nddEnabled = onDeviceTuner_->isEnabled();

	if (requestCount_ == 0 || aaControlChanged || nddEnabled) {
		std::list<Task *> &capture3ATasks = scheduler->groupTasks(AAGroup);
		size_t numberOfPending3ATasks = 0;
		for (auto *task : capture3ATasks)
			if (!task->isRunning())
				numberOfPending3ATasks++;

		size_t needed = 0;
		if (CaptureTasksManager::kRawMetaDelay > numberOfPending3ATasks)
			needed = CaptureTasksManager::kRawMetaDelay - numberOfPending3ATasks;

		for (size_t i = 0; i < needed; ++i) {
			CaptureFrames captureFrames;
			makeTasks("Padding capture", nullptr, captureFrames, requestCount_++);
		}
	}

	uint32_t internalRequestId = requestCount_++;
	bool isStillCapture = (still1Buffer || still2Buffer);
	// todo(yerlandinata): set feature for MFNR.
	// todo(yerlandinata): check whether we need Feature::video or not.
	Feature feature = isStillCapture ? Feature::Capture_lpnr : Feature::Preview;

	if (aaControlChanged || nddEnabled) {
		std::list<Task *> &capture3ATasks = scheduler->groupTasks(AAGroup);
		if (capture3ATasks.size() >= CaptureTasksManager::kRawMetaDelay) {
			auto iter = capture3ATasks.rbegin();
			for (uint32_t shift = 0; shift < CaptureTasksManager::kRawMetaDelay; ++shift) {
				auto *prevAATask = static_cast<AATask *>(*iter);
				prevAATask->setRequest(request);
				prevAATask->setInternalRequestIdApplied(internalRequestId);
				prevAATask->setFeatureApplied(feature);
				prevAATask->setPerFrameControl(
					AATask::PerFrameControl{
						.isStillCapture = isStillCapture,
						.controls = request->controls() });
				iter++;
			}
		}
	}

	CaptureFrames captureFrames;

	auto [taskQBuf, taskDQBuf, sofTask, aaTask, afTask, camSysMetaRequestId] = makeTasks(
		"Capture " + sequence, request, captureFrames, internalRequestId);
	aaTask->setPerFrameControl(
		AATask::PerFrameControl{
			.isStillCapture = isStillCapture,
			.controls = request->controls() });

	Task *taskTr = nullptr;
	Task *taskDip2 = nullptr;
	bool hasVideo = video1Buffer || video2Buffer;

	CaptureResult *aaCaptureResult = captureResult_.query(camSysMetaRequestId);
	SharedMailBox<AaaIspExchange> aaaIspExchange = aaCaptureResult->aaaIspExchange;

	CompleteRequestTask *completeTask = new CompleteRequestTask(
		scheduler, "Complete " + sequence, request, internalRequestId,
		camSysMetaRequestId, pipeline, onDeviceTuner_, faceDetector_,
		aaaIspExchange, hal3A_, feature);

	if (afTask)
		Scheduler::precede(afTask, completeTask);

	captureRawQueue_idx += 1;
	captureRawQueue_idx = captureRawQueue_idx % 8;
	captureRawQueue[captureRawQueue_idx] = captureFrames.raw;
	previewQueue[captureRawQueue_idx] = captureFrames.yuvo1;

	/* Face Detection Task */
	Task *faceDetectTask = faceDetector_->makeFaceDetectionTask(
		scheduler, request, captureFrames.faceDetection,
		aaTask->camSysMetaRequestId_);

	Scheduler::precede(taskDQBuf, faceDetectTask);
	Scheduler::precede(faceDetectTask, completeTask);
	scheduler->succeedPrevTaskByStep(AieFaceDetectionGroup, 0, faceDetectTask);

	scheduler->queueTask(faceDetectTask, AieFaceDetectionGroup);

	if (hasVideo) {
		MCNRFrames mcnr;
		mcnrManager.makeMCNRFrames(mcnr, mcnrPrev,
					   captureFrames.me,
					   captureFrames.yuvo1,
					   captureFrames.yuvo2,
					   video1Buffer,
					   video2Buffer);

		auto [meATunTask, meBTunTask, trTunTask, dipTunTask] =
			mcnrTunManager.makeMcnrTunTasks(mcnr, camSysMetaRequestId, scheduler,
							"MCNR " + sequence, request, internalRequestId);

		auto [taskMeA, taskMeB, tempTaskTr, taskDip1, tempTaskDip2] =
			mcnrManager.makeMcnrTasks(mcnr, scheduler, "MCNR " + sequence,
						  request, internalRequestId, imgSysDev_);

		taskTr = tempTaskTr;
		taskDip2 = tempTaskDip2;

		// Limit the interval from a producer task of tuning buffers
		// to its corresponding consumer task as 2, and we only need
		// to allocate 3 set of the tuning buffers instead of 8.
		// The reason for the regulation is that the V4L2 will cause
		// cache miss, it there are too many associated FDs for one
		// video node, and all of the tuning buffers are using the
		// video node for ImgSys.
		scheduler->succeedPrevTaskByStep(MeAGroup, 2, meATunTask);
		scheduler->succeedPrevTaskByStep(MeBGroup, 2, meBTunTask);
		scheduler->succeedPrevTaskByStep(TrGroup, 2, trTunTask);
		scheduler->succeedPrevTaskByStep(Dip2Group, 2, dipTunTask);

		Scheduler::precede(taskDQBuf, meATunTask);
		scheduler->succeedPrevTaskByStep(MeATunGroup, 0, meATunTask);
		scheduler->succeedPrevTaskByStep(MeBTunGroup, 0, meATunTask);
		scheduler->queueTask(meATunTask, MeATunGroup);

		Scheduler::precede(meATunTask, taskMeA);
		Scheduler::precede(taskDQBuf, taskMeA);
		scheduler->succeedPrevTaskByStep(MeAGroup, 0, taskMeA);
		scheduler->queueTask(taskMeA, MeAGroup);

		Scheduler::precede(taskMeA, meBTunTask);
		scheduler->succeedPrevTaskByStep(MeBTunGroup, 0, meBTunTask);
		scheduler->queueTask(meBTunTask, MeBTunGroup);

		Scheduler::precede(meBTunTask, taskMeB);
		Scheduler::precede(taskMeA, taskMeB);
		scheduler->succeedPrevTaskByStep(MeBGroup, 0, taskMeB);
		scheduler->queueTask(taskMeB, MeBGroup);

		Scheduler::precede(taskMeB, trTunTask);
		scheduler->succeedPrevTaskByStep(TrTunGroup, 0, trTunTask);
		scheduler->queueTask(trTunTask, TrTunGroup);

		Scheduler::precede(trTunTask, taskTr);
		Scheduler::precede(taskMeB, taskTr);
		scheduler->succeedPrevTaskByStep(TrGroup, 0, taskTr);
		scheduler->queueTask(taskTr, TrGroup);

		Scheduler::precede(taskTr, dipTunTask);
		scheduler->succeedPrevTaskByStep(DipTunGroup, 0, dipTunTask);
		scheduler->queueTask(dipTunTask, DipTunGroup);

		Scheduler::precede(dipTunTask, taskDip1);
		Scheduler::precede(taskTr, taskDip1);
		scheduler->succeedPrevTaskByStep(Dip1Group, 0, taskDip1);
		scheduler->queueTask(taskDip1, Dip1Group);

		Scheduler::precede(taskDip1, taskDip2);
		scheduler->succeedPrevTaskByStep(Dip2Group, 0, taskDip2);
		scheduler->queueTask(taskDip2, Dip2Group);

		Scheduler::precede(taskDip2, completeTask);
	}

	bool hasStillCapture = still1Buffer || still2Buffer;

	if (!hasStillCapture) {
		onDeviceTuner_->notifyVideoOnly(internalRequestId);
	} else {
		if (useMfnr && !useLpnr) {
			onDeviceTuner_->notifyStillCapture(internalRequestId);
			MFNRFrames mfnr;
			mfnrManager.makeMFNRFrames(mfnr, captureRawQueue, previewQueue, captureRawQueue_idx, still1Buffer, still2Buffer);

			auto [mfnrTunBssTask, mfnrTunBfbldTask, mfnrTunBfmeTask,
			      mfnrTunSwmeTask, mfnrTunDsTask, mfnrTunDsVbiTask, mfnrTunMcdsF1Task,
			      mfnrTunMsbldTask, mfnrTunAfbldTask] =
				mfnrTunManager.makeMfnrTunTasks(mfnr, camSysMetaRequestId, scheduler, "MfnrTun " + sequence, request, internalRequestId);

			auto [mfnrBssTask, mfnrBfbldTask, mfnrBfmeTask, mfnrSwmeTask,
			      mfnrMcdsF1Task, mfnrDsTask, mfnrDsVbiTask, mfnrMsbldTask,
			      mfnrAfbldTask] =
				mfnrManager.makeMfnrTasks(mfnr, scheduler, "Mfnr " + sequence, request, internalRequestId, imgSysDev_);

			if (hasVideo) {
				Scheduler::precede(taskTr, mfnrBfbldTask);
				Scheduler::precede(taskDip2, mfnrBfbldTask);
			}

			Scheduler::precede(mfnrTunBssTask, mfnrBssTask);
			scheduler->succeedPrevTaskByStep(BssTunTaskGroup, 0, mfnrTunBssTask);
			scheduler->queueTask(mfnrTunBssTask, BssTunTaskGroup);

			scheduler->succeedPrevTaskByStep(BssTaskGroup, 0, mfnrBssTask);
			scheduler->queueTask(mfnrBssTask, BssTaskGroup);

			Scheduler::precede(mfnrBssTask, mfnrTunBfbldTask);
			Scheduler::precede(mfnrTunBfbldTask, mfnrBfbldTask);
			scheduler->succeedPrevTaskByStep(BfbldTunTaskGroup, 0, mfnrTunBfbldTask);
			scheduler->queueTask(mfnrTunBfbldTask, BfbldTunTaskGroup);

			Scheduler::precede(mfnrBssTask, mfnrBfbldTask);
			scheduler->succeedPrevTaskByStep(BfbldTaskGroup, 0, mfnrBfbldTask);
			scheduler->queueTask(mfnrBfbldTask, BfbldTaskGroup);

			Scheduler::precede(mfnrBssTask, mfnrTunBfmeTask);
			Scheduler::precede(mfnrBfbldTask, mfnrTunBfmeTask);
			Scheduler::precede(mfnrTunBfmeTask, mfnrBfmeTask);
			scheduler->succeedPrevTaskByStep(BfmeTunGroup, 0, mfnrTunBfmeTask);
			scheduler->queueTask(mfnrTunBfmeTask, BfmeTunGroup);

			Scheduler::precede(mfnrBfbldTask, mfnrBfmeTask);
			scheduler->succeedPrevTaskByStep(BfmeGroup, 0, mfnrBfmeTask);
			scheduler->queueTask(mfnrBfmeTask, BfmeGroup);

			Scheduler::precede(mfnrTunBfmeTask, mfnrTunSwmeTask);
			Scheduler::precede(mfnrTunSwmeTask, mfnrSwmeTask);
			scheduler->succeedPrevTaskByStep(SwmeTunGroup, 0, mfnrSwmeTask);
			scheduler->queueTask(mfnrTunSwmeTask, SwmeTunGroup);

			Scheduler::precede(mfnrBfbldTask, mfnrSwmeTask);
			scheduler->succeedPrevTaskByStep(SwmeGroup, 0, mfnrSwmeTask);
			scheduler->queueTask(mfnrSwmeTask, SwmeGroup);

			Scheduler::precede(mfnrTunSwmeTask, mfnrTunMcdsF1Task);
			Scheduler::precede(mfnrTunMcdsF1Task, mfnrMcdsF1Task);
			scheduler->succeedPrevTaskByStep(McdsF1TunGroup, 0, mfnrTunMcdsF1Task);
			scheduler->queueTask(mfnrTunMcdsF1Task, McdsF1TunGroup);

			Scheduler::precede(mfnrBfbldTask, mfnrMcdsF1Task);
			scheduler->succeedPrevTaskByStep(McdsF1Group, 0, mfnrMcdsF1Task);
			scheduler->queueTask(mfnrMcdsF1Task, McdsF1Group);

			Scheduler::precede(mfnrBssTask, mfnrTunDsTask);
			Scheduler::precede(mfnrTunMcdsF1Task, mfnrTunDsTask);
			Scheduler::precede(mfnrTunDsTask, mfnrDsTask);
			scheduler->succeedPrevTaskByStep(DsTunGroup, 0, mfnrTunDsTask);
			scheduler->queueTask(mfnrTunDsTask, DsTunGroup);

			Scheduler::precede(mfnrMcdsF1Task, mfnrDsTask);
			scheduler->succeedPrevTaskByStep(DsGroup, 0, mfnrDsTask);
			scheduler->queueTask(mfnrDsTask, DsGroup);

			Scheduler::precede(mfnrTunDsTask, mfnrTunDsVbiTask);
			Scheduler::precede(mfnrTunDsVbiTask, mfnrDsVbiTask);
			scheduler->succeedPrevTaskByStep(DsVbiTunGroup, 0, mfnrTunDsVbiTask);
			scheduler->queueTask(mfnrTunDsVbiTask, DsVbiTunGroup);

			Scheduler::precede(mfnrMcdsF1Task, mfnrDsVbiTask);
			scheduler->succeedPrevTaskByStep(DsVbiGroup, 0, mfnrDsVbiTask);
			scheduler->queueTask(mfnrDsVbiTask, DsVbiGroup);

			Scheduler::precede(mfnrBssTask, mfnrTunMsbldTask);
			Scheduler::precede(mfnrTunDsVbiTask, mfnrTunMsbldTask);
			Scheduler::precede(mfnrTunMsbldTask, mfnrMsbldTask);
			scheduler->succeedPrevTaskByStep(MsbldTunGroup, 0, mfnrTunMsbldTask);
			scheduler->queueTask(mfnrTunMsbldTask, MsbldTunGroup);

			Scheduler::precede(mfnrDsVbiTask, mfnrMsbldTask);
			Scheduler::precede(mfnrDsTask, mfnrMsbldTask);
			Scheduler::precede(mfnrMcdsF1Task, mfnrMsbldTask);
			scheduler->succeedPrevTaskByStep(MsbldGroup, 0, mfnrMsbldTask);
			scheduler->queueTask(mfnrMsbldTask, MsbldGroup);

			Scheduler::precede(mfnrTunMsbldTask, mfnrTunAfbldTask);
			Scheduler::precede(mfnrBfbldTask, mfnrTunAfbldTask);
			Scheduler::precede(mfnrTunAfbldTask, mfnrAfbldTask);
			scheduler->succeedPrevTaskByStep(AfbldTunGroup, 0, mfnrTunAfbldTask);
			scheduler->queueTask(mfnrTunAfbldTask, AfbldTunGroup);

			Scheduler::precede(mfnrMsbldTask, mfnrAfbldTask);
			scheduler->succeedPrevTaskByStep(AfbldGroup, 0, mfnrAfbldTask);
			scheduler->queueTask(mfnrAfbldTask, AfbldGroup);

			Scheduler::precede(mfnrAfbldTask, completeTask);
		} else {
			onDeviceTuner_->notifyStillCapture(internalRequestId);
			LPNRFrames lpnr;
			lpnrManager.makeLPNRFrames(lpnr, captureFrames.raw, still1Buffer, still2Buffer);

			auto [lpnrTunXtrTask, lpnrTunDipTask] = lpnrTunManager.makeLpnrTunTasks(
				lpnr, aaaIspExchange, camSysMetaRequestId, scheduler, "Lpnr " + sequence, request, internalRequestId);

			auto [taskXtr, taskLpnrDip] = lpnrManager.makeLpnrTasks(
				lpnr, scheduler, "Lpnr " + sequence, request,
				internalRequestId, imgSysDev_);

			if (hasVideo) {
				Scheduler::precede(taskTr, taskXtr);
				Scheduler::precede(taskDip2, taskLpnrDip);
			}

			// Limit the interval from a producer task of tuning buffers
			// to its corresponding consumer task as 2.
			scheduler->succeedPrevTaskByStep(XtrGroup, 2, lpnrTunXtrTask);
			scheduler->succeedPrevTaskByStep(LpnrDipGroup, 2, lpnrTunDipTask);

			scheduler->succeedPrevTaskByStep(
				AAGroup, CaptureTasksManager::kRawMetaDelay,
				lpnrTunXtrTask);

			Scheduler::precede(lpnrTunXtrTask, taskXtr);
			scheduler->succeedPrevTaskByStep(LpnrTunXtrTaskGroup, 0, lpnrTunXtrTask);
			scheduler->queueTask(lpnrTunXtrTask, LpnrTunXtrTaskGroup);

			Scheduler::precede(taskXtr, lpnrTunDipTask);
			Scheduler::precede(lpnrTunDipTask, taskLpnrDip);
			scheduler->succeedPrevTaskByStep(LpnrTunDipTaskGroup, 0, lpnrTunDipTask);
			scheduler->queueTask(lpnrTunDipTask, LpnrTunDipTaskGroup);

			Scheduler::precede(taskDQBuf, taskXtr);
			scheduler->succeedPrevTaskByStep(XtrGroup, 0, taskXtr);
			scheduler->queueTask(taskXtr, XtrGroup);

			Scheduler::precede(taskXtr, taskLpnrDip);
			scheduler->succeedPrevTaskByStep(LpnrDipGroup, 0, taskLpnrDip);
			scheduler->queueTask(taskLpnrDip, LpnrDipGroup);

			Scheduler::precede(taskLpnrDip, completeTask);
		}
	}

	scheduler->succeedPrevTaskByStep(CompleteGroup, 0, completeTask);
	scheduler->queueTask(completeTask, CompleteGroup);

	scheduler->schedule();
	return 0;
}

bool MtkISP7CameraData::is3aControlChanged(std::shared_ptr<ControlList> controls_cur, std::shared_ptr<ControlList> controls_cache)
{
	if (!(controls_cache.get())) {
		return true;
	}
	std::vector<int32_t> checkList{
		controls::AE_MODE,
		controls::AE_LOCKED,
		controls::EXPOSURE_TIME,
		controls::AE_PRECAPTURE_TRIGGER,
		controls::AWB_MODE,
		controls::AWB_ENABLE,
		controls::AWB_LOCKED,
	};

	for (auto id : checkList) {
		auto control = controls::controls.at(id);
		auto type = control->type();
		switch (type) {
		case ControlTypeBool: {
			auto bool_control = static_cast<Control<bool> *>(const_cast<ControlId *>(control));
			bool value = static_cast<bool>(
				controls_cur->get(*bool_control).value_or(false));
			bool value_cache = static_cast<bool>(
				controls_cache->get(*bool_control).value_or(false));
			if (value != value_cache) {
				LOG(MtkISP7, Debug) << "id:" << control->id() << " value changed!! " << value << ":" << value_cache;
				return true;
			}
			break;
		}
		case ControlTypeByte:
			break;
		case ControlTypeUnsigned16:
			break;
		case ControlTypeUnsigned32:
			break;
		case ControlTypeInteger32: {
			auto int32_control = static_cast<Control<int32_t> *>(const_cast<ControlId *>(control));
			int32_t value = static_cast<int32_t>(
				controls_cur->get(*int32_control).value_or(0));
			int32_t value_cache = static_cast<int32_t>(
				controls_cache->get(*int32_control).value_or(0));
			if (value != value_cache) {
				LOG(MtkISP7, Debug) << "id:" << int32_control->id() << " value changed!! " << value << ":" << value_cache;
				return true;
			}
			break;
		}
		case ControlTypeInteger64:
			break;
		case ControlTypeFloat:
			break;
		case ControlTypeRectangle:
			break;
		case ControlTypeSize:
			break;
		case ControlTypePoint:
			break;
		case ControlTypeNone:
			break;
		case ControlTypeString:
			break;
			break;
		default:
			break;
		}
	}

	return false;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMtkISP7)

} /* namespace libcamera */
