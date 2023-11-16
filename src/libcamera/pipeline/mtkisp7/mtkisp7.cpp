/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2022, Google Inc.
 *
 * mtkisp7.cpp - Pipeline handler for Mediatek MtkISP7
 */

#include <memory>
#include <string>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/camera.h>
#include <libcamera/control_ids.h>
#include <libcamera/formats.h>
#include <libcamera/property_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "libcamera/internal/camera.h"
#include "libcamera/internal/device_enumerator.h"
#include "libcamera/internal/framebuffer.h"
#include "libcamera/internal/media_device.h"
#include "libcamera/internal/pipeline_handler.h"
#include "libcamera/internal/task_scheduler.h"

#include "camsys/camsys.h"
#include "camsys/capture.h"
#include "hal3a/aaa.h"
#include "hal3a/hal_3a.h"
#include "halisp/hal_isp.h"
#include "imgsys/imgsys.h"
#include "imgsys/lpnr.h"
#include "imgsys/mcnr.h"
#include "libfdft_lib/faces.h"
#include "pipeline/mtkisp7/face_detect/detector.h"
#include "pipeline/mtkisp7/odt/on_device_tuner.h"

namespace libcamera {
LOG_DEFINE_CATEGORY(MtkISP7)

static const ControlInfoMap::Map MtkISP7Controls = {
	{ &controls::draft::PipelineDepth, ControlInfo(8, 8, 8) },
};

enum MtkISP7TaskGroup {
	SofGroup = 0,
	CaptureQueueGroup,
	CaptureDequeueGroup,
	AAGroup,
	AFGroup,
	MeGroup,
	TrGroup,
	XtrGroup,
	Dip1Group,
	Dip2Group,
	LpnrDipGroup,
	AieFaceDetectionGroup,
	AieFaceToneClassificationGroup,
	AieParseGroup,
	CompleteGroup,
};

static const std::map<MtkISP7TaskGroup, std::string> kGroupName{
	{ SofGroup, "SofGroup" },
	{ CaptureQueueGroup, "CaptureQueueGroup" },
	{ CaptureDequeueGroup, "CaptureDequeueGroup" },
	{ AAGroup, "AAGroup" },
	{ AFGroup, "AFGroup" },
	{ MeGroup, "MeGroup" },
	{ TrGroup, "TrGroup" },
	{ XtrGroup, "XtrGroup" },
	{ Dip1Group, "Dip1Group" },
	{ Dip2Group, "Dip2Group" },
	{ AieFaceDetectionGroup, "AieFaceDetectionGroup" },
	{ AieFaceToneClassificationGroup, "AieFaceToneClassificationGroup" },
	{ AieParseGroup, "AieParseGroup" },
	{ CompleteGroup, "CompleteGroup" },
};

class CompleteRequestTask : public Task
{
public:
	CompleteRequestTask(Scheduler *scheduler,
			    const std::string &id,
			    Request *request,
			    PipelineHandler *pipe,
			    SharedMailBox<MtkCameraFaceMetadata> faceMetadata);

	virtual void run() override final;

private:
	void convertFaceMetadata(ControlList &out);

	PipelineHandler *pipe_;
	Request *request_;
	SharedMailBox<MtkCameraFaceMetadata> faceMetadata_;
};

CompleteRequestTask::CompleteRequestTask(Scheduler *scheduler,
					 const std::string &id,
					 Request *request,
					 PipelineHandler *pipe,
					 SharedMailBox<MtkCameraFaceMetadata>
						 faceMetadata)
	: Task(scheduler, id), pipe_(pipe), request_(request),
	  faceMetadata_(std::move(faceMetadata))
{
}

class MtkISP7CameraData : public Camera::Private
{
public:
	MtkISP7CameraData(PipelineHandler *pipe, CamSysDevice *camSysDev,
			  ImgSysDevice *imgSysDev, OnDeviceTuner *odt,
			  FaceDetector *faceDetector, DmaHeap *dmaHeap, Hal3A *hal3A)
		: Camera::Private(pipe), camSysDev_(camSysDev), imgSysDev_(imgSysDev),
		  captureManager(odt), mcnrManager(imgSysDev, dmaHeap, odt),
		  lpnrManager(imgSysDev, dmaHeap, odt), onDeviceTuner_(odt),
		  faceDetector_(faceDetector), dmaHeap_(dmaHeap), hal3A_(hal3A)
	{
	}

	int configure(CameraConfiguration *c);
	int queueRequest(Request *request);

	int start(const ControlList *controls);
	void stopDevice();
	void releaseDevice();

	void frameStart(uint32_t sequence);

	std::tuple<QueueTask *, DequeueTask *, SofTask *, AATask *, AFTask *>
	makeTasks(const std::string &id, Request *request,
		  CaptureFrames &captureFrames, AATask::PerFrameControl perFrameControl);
	void setTasksDependencies(QueueTask *taskQBuf, DequeueTask *taskDQBuf,
				  SofTask *sofTask, AATask *aaTask,
				  AFTask *afTask);

	Stream video1Stream_;
	Stream video2Stream_;
	Stream stillStream_;

	uint32_t frameSequence_ = 0;
	std::list<SofTask *> pendingSofTasks_;

	Size sensorFullSize_;
	CamSysDevice *camSysDev_;
	ImgSysDevice *imgSysDev_;

	CaptureTasksManager captureManager;
	Hal3AManager hal3AManager_;

	MCNRPrevOutput mcnrPrev;
	McnrTasksManager mcnrManager;
	LpnrTasksManager lpnrManager;

	OnDeviceTuner *onDeviceTuner_;
	FaceDetector *faceDetector_;
	DmaHeap *dmaHeap_;

	Hal3A *hal3A_;
	HalIsp *halIsp_;

	uint32_t requestCount_ = 0;
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

	std::unique_ptr<CategorizedScheduler<MtkISP7TaskGroup>> scheduler_;
	std::unique_ptr<DmaHeap> dmaHeap_;

	OnDeviceTuner onDeviceTuner_;

	MediaDevice *camSysMedia_;
	CamSysDevice camSysDev_[2];

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

void CompleteRequestTask::convertFaceMetadata(
	ControlList &out)
{
	std::vector<uint8_t> faceScores;
	std::vector<Rectangle> faceRectangles;
	std::vector<Point> faceLandmarks;

	if (faceMetadata_ == nullptr || !faceMetadata_->valid()) {
		out.set(controls::FaceDetectFaceScores, faceScores);
		out.set(controls::FaceDetectFaceRectangles, faceRectangles);
		out.set(controls::FaceDetectFaceLandmark, faceLandmarks);
		return;
	}
	MtkCameraFaceMetadata faceMetadata = faceMetadata_->get();
	faceScores.reserve(faceMetadata.number_of_faces);
	faceRectangles.reserve(faceMetadata.number_of_faces);
	faceLandmarks.reserve(3 * faceMetadata.number_of_faces);
	for (int i = 0; i < faceMetadata.number_of_faces; i++) {
		faceScores.push_back(faceMetadata.faces[i].score);
		Point faceTopLeft = Point{
			faceMetadata.faces[i].rect[0],
			faceMetadata.faces[i].rect[1]
		};
		Point faceBottomRight = Point{
			faceMetadata.faces[i].rect[2],
			faceMetadata.faces[i].rect[3]
		};
		faceRectangles.emplace_back(faceTopLeft, faceBottomRight);
		Point leftEye = Point{
			(faceMetadata.leyex0[i] + faceMetadata.leyex1[i]) / 2,
			(faceMetadata.leyey0[i] + faceMetadata.leyey1[i]) / 2
		};
		faceLandmarks.push_back(leftEye);

		Point rightEye = Point{
			(faceMetadata.reyex0[i] + faceMetadata.reyex1[i]) / 2,
			(faceMetadata.reyey0[i] + faceMetadata.reyey1[i]) / 2
		};
		faceLandmarks.push_back(rightEye);

		Point mouth = Point{
			(faceMetadata.mouthx0[i] + faceMetadata.mouthx1[i]) / 2,
			(faceMetadata.mouthy0[i] + faceMetadata.mouthy1[i]) / 2
		};
		faceLandmarks.push_back(mouth);
	}
	out.set(controls::FaceDetectFaceScores, faceScores);
	out.set(controls::FaceDetectFaceRectangles, faceRectangles);
	out.set(controls::FaceDetectFaceLandmark, faceLandmarks);
}

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
	convertFaceMetadata(metadata);

	pipe_->completeMetadata(request_, metadata);

	for (auto it : request_->buffers()) {
		FrameBuffer *buffer = it.second;
		pipe_->completeBuffer(request_, buffer);
	}

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
		{ 640, 480 },
		{ 1280, 720 },
		{ 1280, 960 },
		{ 1600, 1200 },
		{ 1920, 1080 },
		{ 1920, 1440 },
		{ 2560, 1440 },
		{ 2560, 1920 },
	};

	const Stream *streams[2]{
		&data_->video1Stream_,
		&data_->video2Stream_
	};

	int videoCnt = 0;
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
			cfg.setStream(const_cast<Stream *>(streams[videoCnt++]));
			break;
		case StreamRole::StillCapture:
			cfg.setStream(const_cast<Stream *>(&data_->stillStream_));
			break;
		default:
			LOG(MtkISP7, Error) << "Invalid StreamRole " << cfg.role;
			return Invalid;
		}
	}

	return Valid;
}

PipelineHandlerMtkISP7::PipelineHandlerMtkISP7(CameraManager *manager)
	: PipelineHandler(manager), imgSysDev_(&onDeviceTuner_), faceDetector_(&aieDev_)
{
	scheduler_ = std::make_unique<CategorizedScheduler<MtkISP7TaskGroup>>(kGroupName);
	dmaHeap_ = std::make_unique<DmaHeap>();
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
			cfg.setStream(&data->stillStream_);
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
	DeviceMatch camSysDM("mtk-cam");
	camSysDM.add("mtk-cam raw-0");
	camSysDM.add("mtk-cam raw-1");

	camSysMedia_ = acquireMediaDevice(enumerator, camSysDM);

	if (!camSysMedia_)
		return false;

	if (camSysMedia_->disableLinks())
		return false;

	DeviceMatch imgSysDM("camera-dip");
	imgSysDM.add("MTK-ISP-DIP-V4L2");

	imgSysMedia_ = acquireMediaDevice(enumerator, imgSysDM);

	if (!imgSysMedia_)
		return false;

	if (imgSysMedia_->disableLinks())
		return false;

	imgSysDev_.init(imgSysMedia_, dmaHeap_.get());

	DeviceMatch aieDM("mtk-aie-5.3");
	aieMedia_ = acquireMediaDevice(enumerator, aieDM);
	if (!aieMedia_) {
		LOG(MtkISP7, Error) << "Failed to match AIE Media";
		return false;
	}
	if (aieDev_.init(aieMedia_, dmaHeap_.get()) != 0) {
		LOG(MtkISP7, Error) << "Failed to init AIE device";
		return false;
	}
	hal3A_[0] = std::make_unique<Hal3A>(0, &halIsp_[0]);
	hal3A_[1] = std::make_unique<Hal3A>(1, &halIsp_[1]);

	halIsp_[0].init(0, 1);
	halIsp_[1].init(1, 2);

	for (unsigned int i = 0; i < 2; i++) {
		if (camSysDev_[i].init(camSysMedia_, i))
			continue;

		ControlList properties = camSysDev_[i].properties();

		// Fill ControlInfoMap
		ControlInfoMap::Map controls = MtkISP7Controls;

		// todo: Fix the frame duration to 30fps for now. It should be
		// updated on stream configuration
		controls[&controls::FrameDurationLimits] = ControlInfo((int64_t)33'333,
								       (int64_t)33'333,
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

		// Create CameraData
		std::unique_ptr<MtkISP7CameraData> data =
			std::make_unique<MtkISP7CameraData>(
				this, &camSysDev_[i], &imgSysDev_,
				&onDeviceTuner_, &faceDetector_,
				dmaHeap_.get(), hal3A_[i].get());

		std::set<Stream *> streams = { &data->video1Stream_,
					       &data->video2Stream_,
					       &data->stillStream_ };

		data->sensorFullSize_ = pixelArraySize;
		data->properties_ = properties;
		data->controlInfo_ = ControlInfoMap(std::move(controls), controls::controls);

		// Create and register the Camera
		std::shared_ptr<Camera> camera =
			Camera::create(std::move(data), camSysDev_[i].cameraId(), streams);
		registerCamera(std::move(camera));

		LOG(MtkISP7, Info) << "Registered Camera[" << camSysDev_[i].cameraId() << "]";
	}

	return true;
}

int MtkISP7CameraData::start([[maybe_unused]] const ControlList *controls)
{
	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());
	auto *scheduler = pipeline->scheduler_.get();

	camSysDev_->frameStart().disconnect(this);
	camSysDev_->frameStart().connect(this, &MtkISP7CameraData::frameStart);

	hal3A_->start();
	// Needs to be called after |hal3A_->start()|, as it uses AF result.
	hal3AManager_.start();

	camSysDev_->start();

	imgSysDev_->start();
	mcnrManager.start();
	lpnrManager.start();
	faceDetector_->start();

	scheduler->schedule();
	return 0;
}

std::tuple<QueueTask *, DequeueTask *, SofTask *, AATask *, AFTask *>
MtkISP7CameraData::makeTasks(const std::string &id, Request *request,
			     CaptureFrames &captureFrames,
			     AATask::PerFrameControl perFrameControl)
{
	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());
	auto *scheduler = pipeline->scheduler_.get();

	captureManager.makeCaptureFrames(captureFrames);

	uint32_t internalRequestId = requestCount_++;
	uint32_t camSysMetaRequestId = 0;

	std::list<Task *> &capture3ATasks = scheduler->groupTasks(AAGroup);

	if (capture3ATasks.size() >= CaptureTasksManager::kExposureAndGainDelay) {
		auto iter = capture3ATasks.rbegin();
		for (uint32_t i = 0; i < CaptureTasksManager::kExposureAndGainDelay - 1; ++i)
			++iter;
		captureFrames.exposureAndGain = static_cast<AATask *>(*iter)->captureFrames_.exposureAndGainOutput;
	} else {
		captureFrames.exposureAndGain = makeMailBox<std::pair<uint32_t, uint32_t>>();
		captureFrames.exposureAndGain->put(std::make_pair(0, 0), []([[maybe_unused]] std::pair<uint32_t, uint32_t> &ex_and_gain) {});
	}

	if (capture3ATasks.size() >= CaptureTasksManager::kRawMetaDelay) {
		camSysMetaRequestId = internalRequestId - CaptureTasksManager::kRawMetaDelay;
		auto iter = capture3ATasks.rbegin();
		for (uint32_t i = 0; i < CaptureTasksManager::kRawMetaDelay - 1; ++i)
			++iter;

		auto *prevAATask = static_cast<AATask *>(*iter);
		captureFrames.tuning = prevAATask->captureFrames_.tuningOutput;

		prevAATask->setPerFrameControl(perFrameControl);
	} else {
		captureFrames.tuning = hal3AManager_.getDummyTuning();
	}

	auto [taskQBuf, taskDQBuf, sofTask] = captureManager.makeCaptureTasks(
		scheduler, id, request, captureFrames);

	auto [aaTask, afTask] = hal3AManager_.make3ATasks(
		scheduler, request, captureFrames, internalRequestId,
		camSysMetaRequestId);

	setTasksDependencies(taskQBuf, taskDQBuf, sofTask, aaTask, afTask);

	return std::make_tuple(taskQBuf, taskDQBuf, sofTask, aaTask, afTask);
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

	camSysDev_->stop();
	imgSysDev_->stop();

	/* Release the transient frames for MCNR before stopping mcnrManger */
	mcnrPrev = {};

	mcnrManager.stop();
	lpnrManager.stop();

	faceDetector_->stop();

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
	Size still = Size{ 0, 0 };

	/* Only cover the video resolution */
	for (auto &cfg : *c) {
		if (cfg.stream() == &video1Stream_)
			video1 = cfg.size;
		else if (cfg.stream() == &video2Stream_)
			video2 = cfg.size;
		else if (cfg.stream() == &stillStream_)
			still = cfg.size;
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

	camSysDev_->configure(sensorFullSize_, camsysYuvSize);
	hal3A_->configure(camsysYuvSize);
	captureManager.configure(dmaHeap_, camSysDev_, pipeline, sensorFullSize_, camsysYuvSize);
	faceDetector_->configure(sensorFullSize_);
	hal3AManager_.configure(dmaHeap_, camSysDev_, hal3A_);

	imgSysDev_->configure();
	onDeviceTuner_->configure(camSysDev_->cameraId(), camSysDev_->getIndex());
	mcnrManager.configure(camsysYuvSize, video1, video2);
	lpnrManager.configure(sensorFullSize_, still);

	return 0;
}

int MtkISP7CameraData::queueRequest(Request *request)
{
	FrameBuffer *video1Buffer = request->findBuffer(&video1Stream_);
	FrameBuffer *video2Buffer = request->findBuffer(&video2Stream_);
	FrameBuffer *stillBuffer = request->findBuffer(&stillStream_);

	if (!video1Buffer && !video2Buffer && !stillBuffer)
		return -EINVAL;

	auto *pipeline = static_cast<PipelineHandlerMtkISP7 *>(pipe());
	auto *scheduler = pipeline->scheduler_.get();

	std::string sequence = std::to_string(request->sequence());

	for (size_t i = scheduler->groupTasks(AAGroup).size();
	     i < CaptureTasksManager::kRawMetaDelay; ++i) {
		CaptureFrames captureFrames;

		makeTasks("Padding capture", nullptr, captureFrames,
			  AATask::PerFrameControl{ .isStillCapture = false });
	}

	CaptureFrames captureFrames;

	auto [taskQBuf, taskDQBuf, sofTask, aaTask, afTask] = makeTasks(
		"Capture " + sequence, request, captureFrames,
		AATask::PerFrameControl{ .isStillCapture = (bool)stillBuffer });

	if (faceDetector_->canMakeFaceDetectionTask(request)) {
		auto [faceDetectionTask, faceToneTask, parseTask] =
			faceDetector_->makeFaceDetectionTask(
				scheduler, request, captureFrames.faceDetection);

		// Current face tone task depends on previous parse task
		scheduler->succeedPrevTaskByStep(AieParseGroup,
						 0, faceToneTask);

		scheduler->succeedPrevTaskByStep(AieFaceDetectionGroup,
						 0, faceDetectionTask);
		scheduler->succeedPrevTaskByStep(AieFaceToneClassificationGroup,
						 0, faceToneTask);
		scheduler->succeedPrevTaskByStep(AieParseGroup,
						 0, parseTask);
		Scheduler::precede(taskDQBuf, faceDetectionTask);
		Scheduler::precede(faceDetectionTask, faceToneTask);
		Scheduler::precede(faceToneTask, parseTask);
		scheduler->queueTask(faceDetectionTask, AieFaceDetectionGroup);
		scheduler->queueTask(faceToneTask, AieFaceToneClassificationGroup);
		scheduler->queueTask(parseTask, AieParseGroup);
	}

	CompleteRequestTask *completeTask = new CompleteRequestTask(
		scheduler, "Complete " + sequence, request, pipeline,
		faceDetector_->getOutputMailBox(request->sequence()));

	Task *taskTr = nullptr;
	Task *taskDip2 = nullptr;
	bool hasVideo = video1Buffer || video2Buffer;

	if (hasVideo) {
		MCNRFrames mcnr;
		mcnrManager.makeMCNRFrames(mcnr, mcnrPrev,
					   captureFrames.me,
					   captureFrames.yuvo1,
					   captureFrames.yuvo2,
					   video1Buffer,
					   video2Buffer);

		auto [taskME, tempTaskTr, taskDip1, tempTaskDip2] =
			mcnrManager.makeMcnrTasks(mcnr, scheduler, "MCNR " + sequence,
						  request, imgSysDev_);

		taskTr = tempTaskTr;
		taskDip2 = tempTaskDip2;

		Scheduler::precede(taskDQBuf, taskME);
		scheduler->succeedPrevTaskByStep(MeGroup, 0, taskME);
		scheduler->queueTask(taskME, MeGroup);

		Scheduler::precede(taskME, taskTr);
		scheduler->succeedPrevTaskByStep(TrGroup, 0, taskTr);
		scheduler->queueTask(taskTr, TrGroup);

		Scheduler::precede(taskTr, taskDip1);
		scheduler->succeedPrevTaskByStep(Dip1Group, 0, taskDip1);
		scheduler->queueTask(taskDip1, Dip1Group);

		Scheduler::precede(taskDip1, taskDip2);
		scheduler->succeedPrevTaskByStep(Dip2Group, 0, taskDip2);
		scheduler->queueTask(taskDip2, Dip2Group);

		Scheduler::precede(taskDip2, completeTask);
	}

	if (stillBuffer) {
		LPNRFrames lpnr;
		lpnrManager.makeLPNRFrames(lpnr, captureFrames.raw, stillBuffer);

		auto [taskXtr, taskLpnrDip] = lpnrManager.makeLpnrTasks(
			lpnr, scheduler, "Lpnr " + sequence, request, imgSysDev_);

		if (hasVideo) {
			Scheduler::precede(taskTr, taskXtr);
			Scheduler::precede(taskDip2, taskLpnrDip);
		}

		Scheduler::precede(taskDQBuf, taskXtr);
		scheduler->succeedPrevTaskByStep(XtrGroup, 0, taskXtr);
		scheduler->queueTask(taskXtr, XtrGroup);

		Scheduler::precede(taskXtr, taskLpnrDip);
		scheduler->succeedPrevTaskByStep(LpnrDipGroup, 0, taskLpnrDip);
		scheduler->queueTask(taskLpnrDip, LpnrDipGroup);

		Scheduler::precede(taskLpnrDip, completeTask);
	}

	scheduler->succeedPrevTaskByStep(CompleteGroup, 0, completeTask);
	scheduler->succeedPrevTaskByStep(AieParseGroup,
					 faceDetector_->getOutputLookbackStep(request->sequence()),
					 completeTask);
	scheduler->queueTask(completeTask, CompleteGroup);

	scheduler->schedule();

	return 0;
}

REGISTER_PIPELINE_HANDLER(PipelineHandlerMtkISP7)

} /* namespace libcamera */
