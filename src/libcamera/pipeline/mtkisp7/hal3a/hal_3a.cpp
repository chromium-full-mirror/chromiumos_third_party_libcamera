/*
 * Copyright (C) 2023; Google Inc.
 *
 * hal_3a.cpp - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#include "hal_3a.h"

#include <cstdint>
#include <cstring>
#include <fstream>
#include <string>
#include <sys/mman.h>

#include <libcamera/base/log.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include "../halisp/hal_isp.h"
#include "mtkcam-core/aaa/include/nvbuf_util.h"
#include "platform/mtkisp7/mtkcam-interfaces/include/kernel-headers/kd_imgsensor.h"

#include "control_ids.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

Hal3A::Hal3A(const uint32_t sensor_idx, HalIsp *halIsp, OnDeviceTuner *odt)
	: sensor_idx_(sensor_idx), onDeviceTuner_(odt)
{
	ASSERT(halIsp);
	halIsp_ = halIsp;
}

void Hal3A::configure(Size camsysYuvSize)
{
	camsysYuvSize_ = camsysYuvSize;
}

void Hal3A::start()
{
	init();
	getInitialInfo();
	config();
	startInternal();
}

void Hal3A::init()
{
	NSCam::IHalSensorList *const pHalSensorList = NSCam::IHalSensorList::get();
	pHalSensorList->searchSensors();

	NVRAM_SENSOR_IDX_INFO _sensorIdxInfo;

	int platformIdx = 1;
	std::string model_name_path = "/run/chromeos-config/v1/name";
	std::fstream model_name_file;
	model_name_file.open(model_name_path, std::ios::in);
	std::string model;
	if (model_name_file.is_open()) {
		getline(model_name_file, model);
		model_name_file.close();
	} else {
		LOG(MtkISP7, Error) << "Unable to open file " << model_name_path;
	}

	if (!model.compare("geralt")) {
		platformIdx = 0;
	} else if (!model.compare("ciri")) {
		platformIdx = 1;
	} else {
		LOG(MtkISP7, Error) << "Undefined model name: " << model;
	}

	if (platformIdx == 0)
		LOG(MtkISP7, Info) << "----google platform----";
	else if (platformIdx == 1)
		LOG(MtkISP7, Info) << "----lenovo platform----";
	else
		LOG(MtkISP7, Info) << "----check platform!!----";

	if (platformIdx == 0) {
		// TODO(chenghaoyang): Abstract sensors' information to support different sensor modules.
		if (sensor_idx_ == 0) { // back camera
			_sensorIdxInfo.sensorDev = 1;
			_sensorIdxInfo.sensorId = 4921;
			_sensorIdxInfo.facing = 0;
			_sensorIdxInfo.moduleId = 0;
			_sensorIdxInfo.sensorName = "HI1339_MIPI_RAW";
			sensor_id_ = HI1339_SENSOR_ID;

		} else { // front camera
			_sensorIdxInfo.sensorDev = 2;
			_sensorIdxInfo.sensorId = 2211;
			_sensorIdxInfo.facing = 1;
			_sensorIdxInfo.moduleId = 0;
			_sensorIdxInfo.sensorName = "GC08A3_MIPI_RAW";
			sensor_id_ = GC08A3_SENSOR_ID;
		}
	} else if (platformIdx == 1) {
		if (sensor_idx_ == 0) { // back camera
			_sensorIdxInfo.sensorDev = 1;
			_sensorIdxInfo.sensorId = 2211;
			_sensorIdxInfo.facing = 0;
			_sensorIdxInfo.moduleId = 0;
			_sensorIdxInfo.sensorName = "GC08A3_MIPI_RAW";
			sensor_id_ = GC08A3_SENSOR_ID;
		} else { // front camera
			_sensorIdxInfo.sensorDev = 2;
			_sensorIdxInfo.sensorId = 1442;
			_sensorIdxInfo.facing = 1;
			_sensorIdxInfo.moduleId = 0;
			_sensorIdxInfo.sensorName = "GC05A2_MIPI_RAW";
			sensor_id_ = GC05A2_SENSOR_ID;
		}
	} else {
		LOG(MtkISP7, Error) << "Invalid platformIdx: " << platformIdx;
	}

	NvBufUtil::initSensorInfo(sensor_idx_, _sensorIdxInfo);

	m_hal3a_ = mtk::hal3a::IHal3A::GetInstance(sensor_idx_);
	mtk::hal3a::v1_0::mtk_3a_init init = {};

	sensor_info_ = SensorInfo::getInstance(sensor_idx_);

	if (sensor_info_) {
		sensor_info_->init(_sensorIdxInfo.sensorDev, _sensorIdxInfo.sensorId, _sensorIdxInfo.moduleId);
		sensor_info_->get_sensor_static_info(&init.sensor_static_info_array);
		sensor_info_->get_sensor_initial_dynamic_info(&init.sensor_init_dynamic_info);
		sensor_info_->get_cal_data(CAMERA_CAM_CAL_DATA_MODULE_VERSION, &init.cal_data);
		sensor_info_->get_cal_data(CAMERA_CAM_CAL_DATA_3A_GAIN, &init.cal_aa);
		sensor_info_->get_cal_data(CAMERA_CAM_CAL_DATA_SHADING_TABLE, &init.cal_lsc);
		sensor_info_->get_cal_data(CAMERA_CAM_CAL_DATA_PDAF, &init.cal_pdaf);
		init.is_vcm_support = sensor_info_->is_af_support();
	} else {
		LOG(MtkISP7, Info) << "sensor_info_ is null";
	}

	// Stereo Feature: init
	// m_Sync3AFlowCtrl->Init(sensor_idx_);

	m_hal3a_->Init(init);
}

void Hal3A::getInitialInfo()
{
	mtk::hal3a::v1_0::mtk_3a_config config = {};

	if (sensor_info_) {
		sensor_info_->get_sensor_static_info(&config.sensor_static_info_array);
		sensor_info_->get_sensor_initial_dynamic_info(&config.sensor_init_dynamic_info);
	} else {
		LOG(MtkISP7, Info) << "sensor_info_ is null";
	}

	// Replace m_meta_helper.convertToConfigRequest
	config.ae_target_mode = 0;
	config.isp_fus_num = 0;
	config.ae_sensor_min_fps = 0;
	config.ae_sensor_max_fps = 0;
	config.multiexp_hdr_mode = 0;
	config.ae_valid_exp = 0;
	config.tuning_feature = 0;
	config.tuning_feature_cap = 0;
	config.target_size_w = 0;
	config.target_size_h = 0;
	config.capture_feature = 5316725;
	config.ae_min_fps = 5000;
	config.ae_max_fps = 30000;
	config.zoom_ratio = 100;
	config.capture_intent = 1;
	config.aov_enable = 0;
	config.custom_feature = 0;
	config.custom_feature_cap = 0;
	config.is_subsample_mode = 0;

	config.sensor_idx = sensor_idx_;

	config.control_config.subsample_count = 1;
	config.control_config.request_count = 1;
	config.control_config.sensor_mode = 0;
	config.control_config.sensor_id = 0;
	config.control_config.bit_mode = 1;

	config.fno = 1.790000;
	config.focal_length = 4.710000;
	config.sensor_mode = 0;

	NSCam::IHalSensorList *const pHalSensorList = NSCam::IHalSensorList::get();
	if (!pHalSensorList) {
		LOG(MtkISP7, Fatal) << "Couldn't get IHalSensorList";
		return;
	}
	config.control_config.sensor_dev = pHalSensorList->querySensorDevIdx(sensor_idx_);
	//TODO, seperate the config for differnt module (geralt, ciri)
	config.orientation.sensor_orientation = (sensor_idx_ == 0) ? 0 : 270;
	config.orientation.facing = (sensor_idx_ == 0) ? 1 : 0;
	LOG(MtkISP7, Error) << "sensor_id: " << sensor_id_;
	switch (sensor_id_) {
	case HI1339_SENSOR_ID:
		config.control_config.sensor_tg_width = 4208;
		config.control_config.sensor_tg_height = 3120;
		config.tg_width = 4208;
		config.tg_height = 3120;
		break;
	case GC08A3_SENSOR_ID:

		config.control_config.sensor_tg_width = 3264;
		config.control_config.sensor_tg_height = 2448;
		config.tg_width = 3264;
		config.tg_height = 2448;
		break;
	case GC05A2_SENSOR_ID:
		config.control_config.sensor_tg_width = 2592;
		config.control_config.sensor_tg_height = 1944;
		config.tg_width = 2592;
		config.tg_height = 1944;
		break;
	default:
		LOG(MtkISP7, Error) << "Un-handle sensor_id: " << sensor_id_;
		config.control_config.sensor_tg_width = 2592;
		config.control_config.sensor_tg_height = 1944;
		config.tg_width = 2592;
		config.tg_height = 1944;
		break;
	}
	m_hal3a_->GetHwInitialSetting(config, initialSetting_);
	m_hal3a_->GetResultForceUpdate(r3AResult_);
	m_hal3a_->Set2aDataToLastPool();
}

void Hal3A::config()
{
	mtk::hal3a::v1_0::mtk_3a_config config = {};

	if (sensor_info_) {
		sensor_info_->get_sensor_perframe_dynamic_info(&config.sensor_perframe_dynamic_info);
	} else {
		LOG(MtkISP7, Error) << "sensor_info_ is null";
	}

	// Replace m_meta_helper.convertToConfigRequest
	config.ae_target_mode = 0;
	config.isp_fus_num = 0;
	config.ae_sensor_min_fps = 0;
	config.ae_sensor_max_fps = 0;
	config.multiexp_hdr_mode = 0;
	config.ae_valid_exp = 0;
	config.tuning_feature = 0;
	config.tuning_feature_cap = 0;
	config.target_size_w = 0;
	config.target_size_h = 0;
	config.capture_feature = 5316725;
	config.ae_min_fps = 5000;
	config.ae_max_fps = 30000;
	config.zoom_ratio = 100;
	config.capture_intent = 1;
	config.aov_enable = 0;
	config.custom_feature = 0;
	config.custom_feature_cap = 0;
	config.is_subsample_mode = 0;

	// TODO: remove if it's always 0.
	/*
	if (config.aov_enable) {
		peripheralController_->NotifyEvent(
			mtk::hal3a::IPeripheralController::kDisableSensorProvider, 0, 0, 0, 0);
	}
	*/
	config.control_config.subsample_count = 1;
	config.control_config.request_count = 1;
	config.control_config.sensor_mode = 0;
	config.control_config.sensor_id = 0;
	config.control_config.bit_mode = 1;

	NSCam::IHalSensorList *const pHalSensorList = NSCam::IHalSensorList::get();
	if (!pHalSensorList) {
		LOG(MtkISP7, Fatal) << "Couldn't get IHalSensorList";
		return;
	}

	config.control_config.sensor_dev = pHalSensorList->querySensorDevIdx(sensor_idx_);
	//TODO, seperate the config for differnt module (geralt, ciri)

	switch (sensor_id_) {
	case HI1339_SENSOR_ID:
		config.control_config.sensor_tg_width = 4208;
		config.control_config.sensor_tg_height = 3120;
		break;
	case GC08A3_SENSOR_ID:
		config.control_config.sensor_tg_width = 3264;
		config.control_config.sensor_tg_height = 2448;
		break;
	case GC05A2_SENSOR_ID:
		config.control_config.sensor_tg_width = 2592;
		config.control_config.sensor_tg_height = 1944;
		break;
	default:
		LOG(MtkISP7, Error) << "Un-handle sensor_id: " << sensor_id_;
		config.control_config.sensor_tg_width = 2592;
		config.control_config.sensor_tg_height = 1944;
		break;
	}

	config.sensor_idx = sensor_idx_;

	config.sub_flash_enable = (sensor_idx_ == 0) ? 0 : 1;
	config.orientation.facing = (sensor_idx_ == 0) ? 1 : 0;
	config.orientation.sensor_orientation = (sensor_idx_ == 0) ? 0 : 0;
	switch (sensor_id_) {
	case HI1339_SENSOR_ID:
		config.tg_width = 4208;
		config.tg_height = 3120;
		config.fno = 1.790000;
		config.focal_length = 4.710000;
		config.feature_mode = 0;
		config.sensor_mode = 0;
		break;
	case GC08A3_SENSOR_ID:
		config.tg_width = 3264;
		config.tg_height = 2448;
		config.fno = 1.790000;
		config.focal_length = 4.710000;
		config.feature_mode = 0;
		config.sensor_mode = 0;
		break;
	case GC05A2_SENSOR_ID:
		config.tg_width = 2592;
		config.tg_height = 1944;
		config.fno = 1.790000;
		config.focal_length = 4.710000;
		config.feature_mode = 0;
		config.sensor_mode = 0;
		break;
	default:
		LOG(MtkISP7, Error) << "Un-handle sensor_id: " << sensor_id_;
		config.tg_width = 2592;
		config.tg_height = 1944;
		config.fno = 1.790000;
		config.focal_length = 4.710000;
		config.feature_mode = 0;
		config.sensor_mode = 0;
		break;
	}

	m_hal3a_->Config(config);

	m_hal3a_->GetResult(r3AResult_);
}

void Hal3A::startInternal()
{
	// TODO: Check if we need to initialize focus position.

	mtk::hal3a::v1_0::mtk_3a_start r_3a_start;
	m_hal3a_->Start(r_3a_start);
}

void Hal3A::doCalculation(FrameBuffer *statistics0, uint64_t timestamp,
			  uint32_t internalRequestId, uint32_t camSysMetaRequestId,
			  bool isStillCapture, int rawMetaFd, unsigned char *rawMetaBuffer,
			  MtkCameraFaceMetadata *faceMetadata, GyroSensor::SensorSample gyroSample,
			  std::pair<uint32_t, uint32_t> *exposureAndGain,
			  AaaIspExchange *aaaIspExchange,
			  std::optional<uint32_t> internalRequestIdApplied)
{
	mtk::hal3a::mtk_camsys_info camSysInfo = {};
	if (sensor_idx_ == 0) { // back camera
		camSysInfo.size_after_frz.width = 4208;
		camSysInfo.size_after_frz.height = 3120;
	} else { // front camera
		camSysInfo.size_after_frz.width = 3264;
		camSysInfo.size_after_frz.height = 2448;
	}
	switch (sensor_id_) {
	case HI1339_SENSOR_ID:
		camSysInfo.size_after_frz.width = 4208;
		camSysInfo.size_after_frz.height = 3120;
		break;
	case GC08A3_SENSOR_ID:
		camSysInfo.size_after_frz.width = 3264;
		camSysInfo.size_after_frz.height = 2448;
		break;
	case GC05A2_SENSOR_ID:
		camSysInfo.size_after_frz.width = 2592;
		camSysInfo.size_after_frz.height = 1944;
		break;
	default:
		LOG(MtkISP7, Error) << "Un-handle sensor_id: " << sensor_id_;
		camSysInfo.size_after_frz.width = 2592;
		camSysInfo.size_after_frz.height = 1944;
		break;
	}
	camSysInfo.pixel_mode = 1;

	mtk::hal3a::v1_0::mtk_hal3a_setting setting = {};
	setting.raw_meta = &r3AResult_.raw_meta;

	if (internalRequestId == 0)
		m_hal3a_->GetResultOfCamsysChange(camSysInfo, &setting);

	mtk::hal3a::v1_0::mtk_3a_param r_3a_param = get3AParam(
		internalRequestId, faceMetadata, gyroSample, isStillCapture);
	m_hal3a_->SetParam(r_3a_param);

	mtk::hal3a::v1_0::mtk_3a_request r_3a_request = {};
	if (statistics0->planes().empty()) {
		LOG(MtkISP7, Fatal) << "Empty statistics0";
		return;
	}

	if (isStillCapture)
		r_3a_request.scenario = mtk::hal3a::Mtk3AScenario::kCaptureP1;
	else
		r_3a_request.scenario = mtk::hal3a::Mtk3AScenario::kPreview;

	r_3a_request.buf_info.request_id = camSysMetaRequestId;
	r_3a_request.buf_info.sof_timestamp = timestamp;
	if (internalRequestIdApplied) {
		onDeviceTuner_->tune3ARequest(internalRequestIdApplied.value(),
					      r_3a_request);
	}

	r_3a_request.stt_buf.fd = statistics0->planes()[0].fd.get();

	MappedFrameBuffer mappedFrameBuffer(statistics0, MappedFrameBuffer::MapFlag::Read);
	r_3a_request.stt_buf.buf = reinterpret_cast<const mtk_cam_uapi_meta_raw_stats_0 *>(
		mappedFrameBuffer.planes()[0].data());

	{
		DmaSyncer syncer(statistics0->planes()[0].fd.get());
		m_hal3a_->DoCalculation(r_3a_request);
	}

	auto *rawMeta =
		reinterpret_cast<mtk_cam_uapi_meta_raw_stats_cfg *>(rawMetaBuffer);
	m_hal3a_->GetResult(r3AResult_);

	resultHistory_.add(internalRequestId, r3AResult_);

	{
		DmaSyncer syncer(rawMetaFd);

		*rawMeta = r3AResult_.raw_meta;
		aaaIspExchange->aaaRequestId = internalRequestId;
		halIsp_->getCamSysMetaTuning(internalRequestId, internalRequestId,
					     rawMetaFd, (intptr_t)rawMetaBuffer, 0,
					     kRawMetaSize, isStillCapture,
					     faceMetadata, aaaIspExchange,
					     internalRequestIdApplied);
	}
	uint32_t exposureTimeMs;
	getExposureAndGain(exposureAndGain, exposureTimeMs);
	ControlList &aaaMetadata = aaaIspExchange->aaaMetadata;

	if (internalRequestIdApplied &&
	    onDeviceTuner_->isDumpStillCapture(internalRequestIdApplied.value())) {
		writeStillCaptureDebugMetadata(aaaMetadata, r3AResult_);
	}

	// ISO sensitivity = analogue gain multiplied by digital gain.
	// However, for now libcamera is assuming that ISO sensitivity
	// is simply equal to analogue gain.
	float floatIso = static_cast<float>(r3AResult_.ae_result.sensor_sensitivity);
	aaaIspExchange->aaaMetadata.set(controls::AnalogueGain, floatIso);
	aaaIspExchange->aaaMetadata.set(controls::ExposureTime, exposureTimeMs);
}

void Hal3A::doCalculationAF(FrameBuffer *statistics1, uint64_t timestamp,
			    uint32_t internalRequestId, uint32_t camSysMetaRequestId,
			    VcmFocusInformation vcmFocusInfo,
			    MtkCameraFaceMetadata *metadata,
			    GyroSensor::SensorSample gyroSample,
			    int32_t *position)
{
	mtk::hal3a::v1_0::mtk_3a_param r_3a_param =
		get3AParam(internalRequestId, metadata, gyroSample, true);
	m_hal3a_->SetParamAF(r_3a_param);

	mtk::hal3a::v1_0::mtk_af_request r_af_request = {};
	if (statistics1->planes().empty()) {
		LOG(MtkISP7, Fatal) << "Empty statistics1";
		return;
	}

	// TODO: Check when to use kAFTrigger.
	r_af_request.scenario = mtk::hal3a::Mtk3AScenario::kAFNormal;
	r_af_request.buf_info.request_id = camSysMetaRequestId;
	r_af_request.buf_info.sof_timestamp = timestamp;

	r_af_request.focus_info = vcmFocusInfo;

	r_af_request.afo_buf.fd = statistics1->planes()[0].fd.get();

	MappedFrameBuffer mappedFrameBuffer(statistics1, MappedFrameBuffer::MapFlag::Read);
	r_af_request.afo_buf.buf = reinterpret_cast<const mtk_cam_uapi_meta_raw_stats_1 *>(
		mappedFrameBuffer.planes()[0].data());

	{
		DmaSyncer syncer(statistics1->planes()[0].fd.get());
		m_hal3a_->DoCalculationAF(r_af_request);
	}

	mtk::hal3a::v1_0::mtk_lens_result lensResult = {};
	m_hal3a_->GetResultAF(lensResult);

	// TODO: notify application if it's kAFTrigger && lensResult.is_focus_finish.
	// TODO: Check if using |timestamp| makes sense.
	*position = lensResult.lens_position;
}

mtk::hal3a::v1_0::mtk_3a_param Hal3A::get3AParam(
	uint32_t internalRequestId,
	MtkCameraFaceMetadata *faceMetadata,
	GyroSensor::SensorSample gyroSample,
	bool isStillCapture, [[maybe_unused]] bool isAF)
{
	mtk::hal3a::v1_0::mtk_3a_param r_3a_param = {};

	// TODO: get parameters for SetParam properly
	r_3a_param.request_id = internalRequestId;
	// TODO: Track the right source in mtk's hal. 4 or 59.
	r_3a_param.active_items = 59;
	// TODO: check if we need false when no 2A / FD is updated.
	r_3a_param.updated = true;
	r_3a_param.is_dummy_request = false;
	r_3a_param.control_mode = 1;
	r_3a_param.scene_mode = 0;
	if (isStillCapture) {
		r_3a_param.capture_intent = 2;
	} else {
		r_3a_param.capture_intent = 1;
	}
	r_3a_param.inflight_capture = 0;
	r_3a_param.ae_lock = 0;
	r_3a_param.ae_mode = 1;
	r_3a_param.ae_precap_trigger = 0;
	r_3a_param.ae_anti_banding_mode = 3;
	r_3a_param.sensor_frame_duration = 33333333;
	r_3a_param.sensor_exposure = 10000000;
	r_3a_param.sensor_sensitivity = 100;
	r_3a_param.ae_exp_index = 0;
	r_3a_param.ae_exp_step = 0.500000;
	r_3a_param.ae_min_fps = 5000;
	r_3a_param.ae_max_fps = 30000;
	r_3a_param.ae_region.count = 1;
	r_3a_param.black_level_lock = 0;
	r_3a_param.set_converge = 0;
	r_3a_param.awb_lock = 0;
	r_3a_param.awb_mode = 1;
	if (isStillCapture) {
		r_3a_param.color_correct_mode = 2;
	} else {
		r_3a_param.color_correct_mode = 1;
	}
	for (unsigned int i = 0; i < kMaxColorGainsCount; i++) {
		r_3a_param.color_correct_gain[i] = 1.0;
	}
	for (unsigned int i = 0; i < 3; i++) {
		for (unsigned int j = 0; j < 3; j++) {
			if (i != j)
				continue;

			r_3a_param.color_correct_mat[i * 3 + j] = 1.0;
		}
	}
	r_3a_param.awb_default_pregain1 = 0;
	//TODO, seperate the config for differnt module (geralt, ciri)
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.af_mode = 4;
	} else { // front camera
		r_3a_param.af_mode = 0;
	}
	r_3a_param.af_trigger = 0;
	r_3a_param.af_focus_distance = 0.000000;
	r_3a_param.af_zoom_ratio = 0;
	r_3a_param.af_zoom_stop = 0;
	r_3a_param.af_region.count = 0;
	r_3a_param.lens_ois_mode = 0;
	r_3a_param.af_notify_timeout = 0;
	r_3a_param.strobe_mode = 0;
	r_3a_param.flash_type = mtk::hal3a::kNoFlash;
	if (isStillCapture) {
		r_3a_param.shading_mode = 2;
	} else {
		r_3a_param.shading_mode = 1;
	}
	r_3a_param.shadingmap_mode = 0;
	if (isStillCapture) {
		r_3a_param.tonemap_mode = 2;
	} else {
		r_3a_param.tonemap_mode = 1;
	}
	r_3a_param.lock_ratio = 0;
	r_3a_param.lock_shading = 0;
	r_3a_param.sensor_test_patten_mode = 0;
	// r_3a_param.sensor_test_patten_data = { valid = 1, Channel_R = 0, Channel_Gr = 0, Channel_Gb = 0, Channel_B = 0};
	r_3a_param.sensor_test_patten_data.valid = 1;
	r_3a_param.prolong_frame_length = 0;
	r_3a_param.face_detect_mode = 1;
	r_3a_param.face_detect_force = 1;
	r_3a_param.ae_custom_pline_mode = 0;
	r_3a_param.ae_manual_pline_idx = 0;
	r_3a_param.ae_iso_speed_mode = 0;
	r_3a_param.ae_meter_mode = 0;
	r_3a_param.ae_convergence_speed = 0;
	r_3a_param.ae_custom_metering_table_mode = 0;
	r_3a_param.ae_clusive_roi_mode = 0;
	// Empty
	// r_3a_param.ae_custom_metering_table =
	// r_3a_param.ae_pline_anchor =
	// r_3a_param.ae_clusive_roi =
	// r_3a_param.ae_manualarea_roi =
	r_3a_param.flash_cali_en = 0;
	r_3a_param.awb_convergence_speed = 6;
	r_3a_param.awb_warmstart_enable = 1;
	//  common tag
	r_3a_param.repeat_tag = 0;
	if (isStillCapture) {
		r_3a_param.get_exif = 1;
	} else {
		r_3a_param.get_exif = 0;
	}
	r_3a_param.is_center_region = 0;
	r_3a_param.imgo_type = 1;
	r_3a_param.app_mode = 0;
	r_3a_param.zoom_ratio = 100;
	// TODO: Track the right source in mtk's hal.
	r_3a_param.target_size_w = camsysYuvSize_.width;
	r_3a_param.target_size_h = camsysYuvSize_.height;
	// r_3a_param.prv_crop_region = { left = 0, top = 0, right = 3264, bottom = 2448, weight = 0 };
	// r_3a_param.prv_crop_normalize_region = { left = 0, top = 0, right = 3264, bottom = 2448, weight = 0 };

	//TODO, seperate the config for differnt module (geralt, ciri)

	switch (sensor_id_) {
	case HI1339_SENSOR_ID:
		r_3a_param.prv_crop_region.right = 4208;
		r_3a_param.prv_crop_region.bottom = 3120;

		r_3a_param.prv_crop_normalize_region.right = 4208;
		r_3a_param.prv_crop_normalize_region.bottom = 3120;
		break;
	case GC08A3_SENSOR_ID:
		r_3a_param.prv_crop_region.right = 3264;
		r_3a_param.prv_crop_region.bottom = 2448;

		r_3a_param.prv_crop_normalize_region.right = 3264;
		r_3a_param.prv_crop_normalize_region.bottom = 2448;
		break;
	case GC05A2_SENSOR_ID:
		r_3a_param.prv_crop_region.right = 2592;
		r_3a_param.prv_crop_region.bottom = 1944;

		r_3a_param.prv_crop_normalize_region.right = 2592;
		r_3a_param.prv_crop_normalize_region.bottom = 1944;
		break;
	default:
		LOG(MtkISP7, Error) << "Un-handle sensor_id: " << sensor_id_;
		r_3a_param.prv_crop_region.right = 2592;
		r_3a_param.prv_crop_region.bottom = 1944;

		r_3a_param.prv_crop_normalize_region.right = 2592;
		r_3a_param.prv_crop_normalize_region.bottom = 1944;
		break;
	}

	r_3a_param.low_fps = 0;
	r_3a_param.remosaic_enable = 0;
	// ae tag
	r_3a_param.ae_target_mode = 0;
	// Empty
	// r_3a_param.ae_exp_level =   custom_param:
	r_3a_param.custom_param.id = 0;
	r_3a_param.custom_param.gain_align = 0;
	r_3a_param.custom_param.target_gain_x1024 = 0;
	r_3a_param.custom_param.ev0_isp_gain_x1024 = 0;
	r_3a_param.custom_param.ev0_sensor_gain_x1024 = 0;
	r_3a_param.custom_param.ev0_exptime_us = 0;
	r_3a_param.custom_param.exptime_limit_us = 0;
	r_3a_param.custom_param.gain_limit_x1024 = 0;
	r_3a_param.custom_param.frame_index = 0;
	r_3a_param.custom_param.end_frame_index = 0;
	r_3a_param.ae_valid_exp = 0;
	r_3a_param.denoise_mode = 0;
	r_3a_param.ae_sensor_min_fps = 5000;
	r_3a_param.ae_sensor_max_fps = 30000;
	r_3a_param.ae_hal_exp_index = 0;
	r_3a_param.manual_ev_ctrl.frame_count = 0;
	r_3a_param.manual_ev_ctrl.frame_index = 0;
	r_3a_param.manual_ev_ctrl.ev_step = 0.000000;
	r_3a_param.manual_ev_ctrl.ev_begin = 0.000000;
	r_3a_param.manual_ev_ctrl.ae_lock = 0;
	r_3a_param.mwb_cct = 0;
	r_3a_param.pause_af = 0;
	if (gyroSample.timestamp != 0) {
		r_3a_param.gyro_valid = 1;
		r_3a_param.gyro_data.gyro[0] = gyroSample.x_value;
		r_3a_param.gyro_data.gyro[1] = gyroSample.y_value;
		r_3a_param.gyro_data.gyro[2] = gyroSample.z_value;
		r_3a_param.gyro_data.timestamp = gyroSample.timestamp;
	} else {
		r_3a_param.gyro_valid = 0;
	}
	r_3a_param.acce_valid = 0;
	r_3a_param.light_valid = 0;
	r_3a_param.als_all_valid[0] = 0;
	r_3a_param.als_all_valid[1] = 0;
	r_3a_param.flicker_valid[0] = 0;
	r_3a_param.flicker_valid[1] = 0;
	r_3a_param.color_valid[0] = 0;
	r_3a_param.color_valid[1] = 0;
	r_3a_param.is_stagger = 0;
	r_3a_param.is_mstream = 0;
	r_3a_param.flash_info.isAvailable = 0;
	r_3a_param.flash_info.isOn = 0;
	r_3a_param.flash_info.inCharge = 0;
	r_3a_param.flash_info.battVol = 0;
	r_3a_param.flash_info.isLowPower = 0;
	r_3a_param.flash_info.chargerStatus = 0;
	r_3a_param.flash_info.driverFault = 0;
	r_3a_param.flash_info.timeInfo.mfStartTime = 0;
	r_3a_param.flash_info.timeInfo.mfEndTime = 0;
	r_3a_param.flash_info.timeInfo.mfTimeout = 0;
	r_3a_param.flash_info.timeInfo.mfTimeoutLt = 0;
	r_3a_param.flash_info.timeInfo.mfIsTimeout = 0;
	r_3a_param.is_flash_force_off = 0;
	r_3a_param.flash_full_cali_en = 0;
	r_3a_param.flash_fast_cali_en = 0;
	r_3a_param.isp_fus_num = 0;
	// TODO: 0 or 1
	r_3a_param.subsample_sync_info = 0;
	r_3a_param.multiexp_hdr_mode = 0;
	r_3a_param.hdr_mode = 0;
	r_3a_param.fast_switch_param.sensor_mode = 0;
	// r_3a_param.fast_switch_param.tg_size = { w = 3264, h = 2448 };
	// r_3a_param.fast_switch_param.full_tg_size = { w = 3264, h = 2448 };

	//TODO, seperate the config for differnt module (geralt, ciri)
	switch (sensor_id_) {
	case HI1339_SENSOR_ID:
		r_3a_param.fast_switch_param.tg_size.w = 4208;
		r_3a_param.fast_switch_param.tg_size.h = 3120;

		r_3a_param.fast_switch_param.full_tg_size.w = 4208;
		r_3a_param.fast_switch_param.full_tg_size.h = 3120;
		break;
	case GC08A3_SENSOR_ID:
		r_3a_param.fast_switch_param.tg_size.w = 3264;
		r_3a_param.fast_switch_param.tg_size.h = 2448;

		r_3a_param.fast_switch_param.full_tg_size.w = 3264;
		r_3a_param.fast_switch_param.full_tg_size.h = 2448;
		break;
	case GC05A2_SENSOR_ID:
		r_3a_param.fast_switch_param.tg_size.w = 2592;
		r_3a_param.fast_switch_param.tg_size.h = 1944;

		r_3a_param.fast_switch_param.full_tg_size.w = 2592;
		r_3a_param.fast_switch_param.full_tg_size.h = 1944;
		break;
	default:
		LOG(MtkISP7, Error) << "Un-handle sensor_id: " << sensor_id_;
		r_3a_param.fast_switch_param.tg_size.w = 2592;
		r_3a_param.fast_switch_param.tg_size.h = 1944;

		r_3a_param.fast_switch_param.full_tg_size.w = 2592;
		r_3a_param.fast_switch_param.full_tg_size.h = 1944;
		break;
	}

	r_3a_param.fast_switch_param.ae_target_mode_next = 0;
	r_3a_param.fast_switch_param.ae_valid_exp_next = 1;
	r_3a_param.fast_switch_param.ae_sensor_mode_next = 0;
	r_3a_param.fast_switch_param.is_seamless = 0;
	r_3a_param.fast_switch_param.seam_policy = 0;

	if (faceMetadata) {
		r_3a_param.faces = *faceMetadata;
		r_3a_param.face_num = faceMetadata->number_of_faces;
		r_3a_param.is_fd_ready = true;
	} else {
		r_3a_param.face_num = 0;
		r_3a_param.is_fd_ready = false;
	}
	r_3a_param.is_fd_enable = 1;
	r_3a_param.ot_info.is_valid = 0;
	r_3a_param.gf_info.is_valid = 0;
	r_3a_param.tuning_feature = 0;
	r_3a_param.sensor_feature = 0;
	r_3a_param.custom_feature = 0;
	if (isStillCapture) {
		r_3a_param.tuning_feature_cap = 28;
	} else {
		r_3a_param.tuning_feature_cap = 0;
	}
	r_3a_param.custom_feature_cap = 0;
	r_3a_param.custom_00 = 0;
	r_3a_param.sync2a_mode = 0;

	//TODO, seperate the config for differnt module (geralt, ciri)
	if (sensor_idx_ == 0) { // back camera
		r_3a_param.master_idx = 0;
		r_3a_param.awb_master_idx = 0;
	} else { // front camera
		r_3a_param.master_idx = 1;
		r_3a_param.awb_master_idx = 1;
	}
	r_3a_param.iris_info.iris_status = 0;
	r_3a_param.iris_info.iris_position = 0;
	r_3a_param.iris_info.previous_iris_position = 0;
	r_3a_param.iris_info.moving_timestamp = 0;
	r_3a_param.iris_info.previous_moving_timestamp = 0;
	r_3a_param.iris_info.fn_cali = 0.000000;

	return r_3a_param;
}

void Hal3A::getExposureAndGain(
	std::pair<uint32_t, uint32_t> *exposureAndGain, uint32_t &exposureTimeMs)
{
	ae_exposure_setting_table ae_table = r3AResult_.ae_result.ae_exp_table;
	if (ae_table.cnt == 0)
		return;

	auto *const pSensorList = NSCam::IHalSensorList::get();

	if (!pSensorList) {
		LOG(MtkISP7, Error)
			<< "Get exposure and gain: No IHalSensorList";
		return;
	}

	auto dev_idx = pSensorList->querySensorDevIdx(sensor_idx_);
	auto *const pHalSensor = pSensorList->createSensor("pipemgrPerframeSet", sensor_idx_);

	if (!pHalSensor) {
		LOG(MtkISP7, Error) << "Get exposure and gain: "
				    << "Failed to create pipemgrPerframeSet with id: "
				    << sensor_idx_;
		return;
	}

	for (int exp = 0; exp < AE_EXP_MODE_MAX_T; ++exp) {
		if (ae_table.table[exp].mode <= 0)
			continue;

		// exp should be 4: AE_EXP_MODE_NE_T

		uint32_t gain = pHalSensor->convert_gain(dev_idx, ae_table.table[exp].afe_gain);
		uint32_t ex = ae_table.table[exp].exposure_line;
		exposureTimeMs = ae_table.table[exp].exposure_ns/1000;
		*exposureAndGain = std::make_pair(ex, gain);
		break;
	}

	pHalSensor->destroyInstance("pipemgrPerframeSet");
}

void Hal3A::writeStillCaptureDebugMetadata(ControlList &out,
					   mtk::hal3a::v1_0::mtk_3a_result &result)
{
	const unsigned int idx3ADebug = 6;
	const unsigned int idxIspDebug = 7;

	std::vector<uint16_t> jpegAppSegmentLength(16, 0);
	jpegAppSegmentLength[idx3ADebug] = sizeof(AAA_DEBUG_INFO1_T);
	jpegAppSegmentLength[idxIspDebug] = sizeof(AAA_DEBUG_INFO2_T);
	out.set(controls::JpegApplicationSegmentLength,
		Span<const uint16_t, 16>(jpegAppSegmentLength));

	size_t totalSize = sizeof(AAA_DEBUG_INFO1_T) +
			   sizeof(AAA_DEBUG_INFO2_T);
	std::vector<uint8_t> jpegAppSegmentContent(totalSize);

	uint8_t *app6Src = reinterpret_cast<uint8_t *>(
		&result.debug_3a_info);
	std::memcpy(jpegAppSegmentContent.data(), app6Src,
		    jpegAppSegmentLength[idx3ADebug]);

	uint8_t *app7Src = reinterpret_cast<uint8_t *>(
		&result.debug_isp_info);
	uint8_t *app7Dest = jpegAppSegmentContent.data() +
			    jpegAppSegmentLength[idx3ADebug];
	std::memcpy(app7Dest, app7Src, jpegAppSegmentLength[idxIspDebug]);

	out.set(controls::JpegApplicationSegmentContent, jpegAppSegmentContent);
}

} /* namespace libcamera */
