/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.cpp - MtkISP7 On Device Tuner module.
 */

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include <cstdint>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <optional>
#include <vector>

#include <libcamera/base/log.h>

#include <libcamera/control_ids.h>
#include <libcamera/request.h>
#include <libcamera/stream.h>

#include "debug_exif/aaa/dbg_aaa_param.h"
#include "linux/mtkisp7/drv/7.1/ctrl_meta.h"
#include "mtkcam-interfaces/utils/ndd/ndd_autogen_def.h"
#include "pipeline/mtkisp7/camsys/capture.h"
#include "pipeline/mtkisp7/imgsys/lpnr.h"
#include "pipeline/mtkisp7/imgsys/mcnr.h"
#include "pipeline/mtkisp7/odt/camsys_driver_debug.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/dump_metadata.h"
#include "platform/mtkisp7/halisp/IspControls.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

constexpr const char *kEnableTuningPath = "/run/camera/enable_tuning";
constexpr const char *kEnforceLowIsoLpnr = "/run/camera/enforce_low_iso_lpnr";
constexpr const char *kExportRequestPath = "/run/camera/export_dump";
constexpr const char *kImportRequestPath = "/run/camera/import_dump";
constexpr const char *kWorkDir = "/tmp/vendor/camera_dump";

} // namespace

std::vector<ImagiqAdapter::ExportResult> OnDeviceTuner::batchExport(
	const std::vector<Dump> &dumps)
{
	std::vector<ImagiqAdapter::ExportResult> results;
	results.reserve(dumps.size());
	for (auto dump : dumps) {
		if (dump.config.enableExport) {
			results.push_back(ImagiqAdapter::exportDump(dump));
		}
	}
	return results;
}

void OnDeviceTuner::batchImport(const std::vector<Dump> &dumps)
{
	for (auto dump : dumps) {
		ImagiqAdapter::importDump(dump);
	}
}

void OnDeviceTuner::batchPrepareReimport(
	const std::vector<ImagiqAdapter::ExportResult> &exportResults)
{
	for (const auto &result : exportResults) {
		if (!result.errorCode.has_value() &&
		    result.dump.config.writeReimportConfig) {
			ImagiqAdapter::prepareReimport(result);
		}
	}
}

void OnDeviceTuner::configure(
	const std::string &sensorId, unsigned int camsysIndex)
{
	if (!enabled_) {
		return;
	}

	enabled_ = true;
	exportBegin_ = 0;
	exportEnd_ = 0;
	importBegin_ = 0;
	importEnd_ = 0;
	prevStartedRequestNum_ = -1;
	prevEndedRequestNum_ = -1;
	sensorId_ = sensorId;
	camsysDebug_ = CamsysDebug::create(camsysIndex);
	sessionTimestamp_ = ImagiqAdapter::generateDumpTimestamp();

	if (std::filesystem::exists(kEnforceLowIsoLpnr)) {
		enforceLowIsoLpnr_ = true;
		LOG(MtkISP7, Warning) << "LPNR is forced to use low-ISO mode!";
	}

	// Immediately create one directory for any capture dumps.
	prepareNewExportDirectory();
	ImagiqAdapter::notifyNewSession(sensorId, sessionTimestamp_);
}

void OnDeviceTuner::initialize()
{
	enabled_ = false;
	enforceLowIsoLpnr_ = false;

	if (!std::filesystem::exists(kEnableTuningPath)) {
		return;
	}

	int ret = ImagiqAdapter::loadConfig(dumpConfig_, kWorkDir);
	if (ret) {
		LOG(MtkISP7, Error) << "Failed to load the config, error code: " << ret;
		return;
	}

	ret = ImagiqAdapter::enableMtkTuningTool(kWorkDir);
	if (ret) {
		LOG(MtkISP7, Error) << "Failed to enable MTK tuning tool";
		return;
	}

	LOG(MtkISP7, Warning) << "Pipeline tuning enabled";
	enabled_ = true;
}

bool OnDeviceTuner::isLowIsoLpnrEnforced()
{
	return enabled_ && enforceLowIsoLpnr_;
}

void OnDeviceTuner::loadTuneRequest(int requestNumber)
{
	if (!enabled_) {
		return;
	}
	std::ifstream exportRequestFile(kExportRequestPath);
	if (exportRequestFile.good()) {
		int exportRequestCount = 0;
		exportRequestFile >> exportRequestCount;
		LOG(MtkISP7, Info) << "Loaded dump export request from file: "
				   << exportRequestCount << " frames";
		exportRequestFile.close();
		std::filesystem::path path(kExportRequestPath);
		std::filesystem::remove(path);
		exportBegin_ = requestNumber;
		exportEnd_ = requestNumber + exportRequestCount;
		ImagiqAdapter::notifyExportRequest(exportRequestCount);
	}

	std::ifstream importRequestFile(kImportRequestPath);
	if (importRequestFile.good()) {
		int importRequestCount = 0;
		importRequestFile >> importRequestCount;
		LOG(MtkISP7, Info) << "Loaded dump import request from file: "
				   << importRequestCount << " frames";
		importRequestFile.close();
		std::filesystem::path path(kImportRequestPath);
		std::filesystem::remove(path);
		importBegin_ = requestNumber;
		importEnd_ = requestNumber + importRequestCount;
		ImagiqAdapter::notifyImportRequest(importRequestCount);
	}
}

InfoFrame OnDeviceTuner::getFrameInfoFromRequest(
	Request *request, FrameBuffer *buffer)
{
	const auto stream = request->findStream(buffer);
	if (!stream) {
		LOG(MtkISP7, Fatal) << "Buffer doesn't exist in request!";
	}
	const auto streamCfg = stream->configuration();

	/* Android requires NV12 to align with 64 for buffers from application */
	return InfoFrame(streamCfg.pixelFormat, streamCfg.size, buffer, 64);
}

bool OnDeviceTuner::isImgsysCaptureStage(PEU_Stage stage)
{
	return std::find(
		       kImgsysCaptureStages.begin(), kImgsysCaptureStages.end(), stage) !=
	       kImgsysCaptureStages.end();
}

void OnDeviceTuner::notifyRequestBegin(int requestNumber)
{
	if (!enabled_ || requestNumber <= prevStartedRequestNum_) {
		return;
	}
	prevStartedRequestNum_ = requestNumber;
	loadTuneRequest(requestNumber);
	ImagiqAdapter::notifyRequestBegin(
		sensorId_, requestNumber);
}

void OnDeviceTuner::notifyRequestEnd(int requestNumber)
{
	if (!enabled_ || requestNumber <= prevEndedRequestNum_) {
		return;
	}
	prevEndedRequestNum_ = requestNumber;
	stillCaptureRequestIds_.erase(requestNumber);
	ImagiqAdapter::notifyRequestEnd(
		sensorId_, requestNumber, sessionTimestamp_,
		shouldExportDumpNow(requestNumber), kWorkDir, currentExportPath_);
}

void OnDeviceTuner::notifyStillCapture(int requestNumber)
{
	stillCaptureRequestIds_.insert(requestNumber);
}

bool OnDeviceTuner::parseHalIspNdd(
	uint32_t internalRequestId,
	mtk::isphal::v1_0::NddInfo &ndd)
{
	uint32_t requestNumber = internalRequestId;
	bool isStillCapture = stillCaptureRequestIds_.count(requestNumber) == 1;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) && !isStillCapture)) {
		return false;
	}
	if (isStillCapture) {
		ndd.ndd_data.action =
			static_cast<int>(Action::Capture);
		ndd.ndd_category = NSCam::TuningUtils::eCategory::kCAPTURE;
		ndd.ndd_data.feature =
			static_cast<int>(Feature::Capture_lpnr);
	} else {
		ndd.ndd_category =
			NSCam::TuningUtils::eCategory::kSTREAMING;
		ndd.ndd_data.feature =
			static_cast<int>(Feature::Preview);
	}
	ndd.ndd_data.requestNo = internalRequestId;
	ndd.ndd_data.frameNo = internalRequestId;
	ndd.ndd_data.platform = 8188;
	ndd.ndd_data.timestamp = sessionTimestamp_;
	ndd.ndd_data.sensorId =
		ImagiqAdapter::sensorIdMap.at(sensorId_);
	ndd.ndd_data.dualCamId = NSCam::TuningUtils::eDualCamId::kINVALID;
	ndd.ndd_data.pixelHeight = -1;
	ndd.ndd_data.pixelWidth = -1;
	return true;
}

int OnDeviceTuner::prepareNewExportDirectory()
{
	std::filesystem::path workPath(kWorkDir);
	std::filesystem::path newPath =
		workPath /
		("UKey" + ImagiqAdapter::formatTimestamp(sessionTimestamp_));
	auto cmd = "mkdir -p " + newPath.string();
	int ret = system(cmd.c_str());
	if (ret != 0) {
		LOG(MtkISP7, Error) << "Failed to prepare dump directory, error code: "
				    << ret;
		return ret;
	}
	LOG(MtkISP7, Info) << "Current dump directory: " << newPath.string();
	currentExportPath_ = newPath;
	return ret;
}

bool OnDeviceTuner::shouldExportDumpNow(uint32_t requestNumber)
{
	return enabled_ && exportBegin_ <= requestNumber &&
	       requestNumber < exportEnd_;
}

bool OnDeviceTuner::shouldImportDumpNow(uint32_t requestNumber)
{
	return enabled_ && importBegin_ <= requestNumber &&
	       requestNumber < importEnd_;
}

void OnDeviceTuner::tune(
	uint32_t requestNumber,
	std::vector<NamedFrame> namedFrames,
	bool forceDump)
{
	if (!enabled_ && !forceDump &&
	    !shouldExportDumpNow(requestNumber) && !shouldImportDumpNow(requestNumber)) {
		return;
	}
	std::vector<Dump> dumps;
	for (auto namedFrame : namedFrames) {
		Dump::Metadata metadata = kDumpMetadata.at(namedFrame.id);
		Dump::Config config = dumpConfig_[namedFrame.id];
		dumps.push_back({ .id = namedFrame.id,
				  .requestNumber = requestNumber,
				  .sensorId = sensorId_,
				  .timestamp = sessionTimestamp_,
				  .workPath = currentExportPath_,
				  .frame = namedFrame.frame,
				  .array = std::nullopt,
				  .metadata = metadata,
				  .config = config });
	}
	if (forceDump || shouldExportDumpNow(requestNumber)) {
		const auto exportResults = batchExport(dumps);
		batchPrepareReimport(exportResults);
	}
	if (shouldImportDumpNow(requestNumber)) {
		batchImport(dumps);
	}
}

void OnDeviceTuner::tune(
	uint32_t requestNumber,
	std::vector<NamedPointer> namedPointers,
	bool forceDump)
{
	if (!enabled_ && !forceDump &&
	    !shouldExportDumpNow(requestNumber) && !shouldImportDumpNow(requestNumber)) {
		return;
	}
	std::vector<Dump> dumps;
	for (auto namedPtr : namedPointers) {
		Dump::Metadata metadata = kDumpMetadata.at(namedPtr.id);
		Dump::Config config = dumpConfig_[namedPtr.id];
		std::vector<uint8_t> buffer(namedPtr.size);
		std::memcpy(buffer.data(), namedPtr.ptr, namedPtr.size);
		dumps.push_back({ .id = namedPtr.id,
				  .requestNumber = requestNumber,
				  .sensorId = sensorId_,
				  .timestamp = sessionTimestamp_,
				  .workPath = currentExportPath_,
				  .frame = std::nullopt,
				  .array = buffer,
				  .metadata = metadata,
				  .config = config });
	}
	if (forceDump || shouldExportDumpNow(requestNumber)) {
		const auto exportResults = batchExport(dumps);
		batchPrepareReimport(exportResults);
	}
	// todo(yerlandinata, before merge): decide what to do next
	// if (shouldImportDumpNow(requestNumber)) {
	//     batchImport(dumps);
	// }
}

void OnDeviceTuner::tuneCamsys(uint32_t internalRequestId, CaptureFrames &frames)
{
	uint32_t requestNumber = internalRequestId;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		internalRequestId, { { Dump::Id::P1_IMGO, frames.raw->get() },
				       { Dump::Id::P1_YUVO_R1, frames.yuvo1->get() },
				       { Dump::Id::P1_YUVO_R2, frames.yuvo2->get() },
				       { Dump::Id::P1_DRZS4NO_R3, frames.me->get() },
				       { Dump::Id::P1_META_P1, frames.tuning->get() } });

	// Driver's registers
	Dump::Config drvRegConfig = dumpConfig_[Dump::Id::P1_REG_P1];
	if (shouldExportDumpNow(requestNumber) && drvRegConfig.enableExport && camsysDebug_) {
		Dump registerDump{
			.id = Dump::Id::P1_REG_P1,
			.requestNumber = requestNumber,
			.sensorId = sensorId_,
			.timestamp = sessionTimestamp_,
			.workPath = currentExportPath_,
			.frame = std::nullopt,
			.array = std::nullopt,
			.metadata = kDumpMetadata.at(Dump::Id::P1_REG_P1),
			.config = drvRegConfig
		};
		std::filesystem::path dumpPath =
			ImagiqAdapter::getDumpFileName(registerDump);
		camsysDebug_->exportDump(
			frames.raw->get().buffer()->metadata().hwSequence,
			requestNumber, dumpPath);
	}
}

bool OnDeviceTuner::tuneCamsysHalIsp(
	uint32_t internalRequestId,
	mtk::isphal::v1_0::TuningParamP1 &tuningParam,
	mtk::isphal::v1_0::ReturnParamP1 &tuningResult,
	mtk::hal3a::v1_0::mtk_3a_result &mtk3AResult)
{
	if (!enabled_) {
		return false;
	}
	tuningParam.cam_info->rNdd_info.ndd_data.stage =
		static_cast<int>(Stage::P1);
	tuningParam.cam_info->rNdd_info.ndd_data.action = -1;
	tuningParam.is_need_exif = 1;
	tuningResult.exif.valid = true;
	std::memcpy(tuningResult.exif.data, reinterpret_cast<uint8_t *>(&mtk3AResult.debug_isp_info), sizeof(AAA_DEBUG_INFO2_T));
	return parseHalIspNdd(internalRequestId, tuningParam.cam_info->rNdd_info);
}

void OnDeviceTuner::tuneExif(
	uint32_t internalRequestId,
	const mtk::isphal::v1_0::ExifInfo3A &exif3a,
	const mtk::isphal::v1_0::ExifInfoP2 &exifIsp,
	EStage_T stage)
{
	uint32_t requestNumber = internalRequestId;
	bool isStillCapture = stillCaptureRequestIds_.count(requestNumber) == 1;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) && !isStillCapture)) {
		return;
	}

	bool dumpIdFound = isStillCapture ? kLpnrExifDumpIdMap.count(stage) > 0 : kMcnrExifDumpIdMap.count(stage) > 0;

	if (!dumpIdFound) {
		LOG(MtkISP7, Info) << "No exif dump id for stage: " << static_cast<int>(stage);
		return;
	}

	Dump::Id dumpId = isStillCapture ? kLpnrExifDumpIdMap.at(stage) : kMcnrExifDumpIdMap.at(stage);

	std::vector<uint8_t> exifArray;
	ImagiqAdapter::serializeExif(exifArray, exif3a, exifIsp);

	batchExport({ {
		.id = dumpId,
		.requestNumber = requestNumber,
		.sensorId = sensorId_,
		.timestamp = sessionTimestamp_,
		.workPath = currentExportPath_,
		.frame = std::nullopt,
		.array = exifArray,
		.metadata = kDumpMetadata.at(dumpId),
		.config = dumpConfig_[dumpId],
	} });
}

void OnDeviceTuner::tuneImgsysHalIsp(
	uint32_t internalRequestId,
	mtk::isphal::v1_0::TuningParamDip &tuningParam,
	mtk::isphal::v1_0::ReturnParamDip &tuningResult,
	mtk::hal3a::v1_0::mtk_3a_result &mtk3AResult,
	EStage_T stage)
{
	if (!enabled_) {
		return;
	}
	tuningParam.cam_info.rNdd_info.ndd_data.stage = stage;
	tuningParam.cam_info.sr_para.decision_param.staticInfo.sensorId =
		static_cast<int32_t>(ImagiqAdapter::sensorIdMap.at(sensorId_));
	tuningParam.cam_info.rNdd_info.ndd_data.action =
		static_cast<int>(Action::Preview);
	tuningParam.is_need_exif = 1;
	tuningParam.exif_3a.size = sizeof(AAA_DEBUG_INFO1_T);
	tuningParam.exif_3a.data =
		reinterpret_cast<uint8_t *>(&mtk3AResult.debug_3a_info);

	tuningResult.exif.valid = true;
	tuningResult.exif.size = sizeof(AAA_DEBUG_INFO2_T);
	tuningResult.exif.data =
		reinterpret_cast<uint8_t *>(&mtk3AResult.debug_isp_info);
	parseHalIspNdd(internalRequestId, tuningParam.cam_info.rNdd_info);
}

void OnDeviceTuner::tuneImgsysMetadata(
	SingleDeviceRequest *sdRequest,
	InfoFrame &metaFrame)
{
	if (!enabled_) {
		return;
	}
	ctrl_meta_t *imgSysMetadata =
		reinterpret_cast<ctrl_meta_t *>(metaFrame.address(0));
	const auto stageEnums = sdRequest->getStageEnums();
	for (size_t i = 0; i < stageEnums.size(); i++) {
		if (kPeuStageDumpIdMap.count(stageEnums[i]) == 0) {
			LOG(MtkISP7, Error) << "Unrecognized stageEnum: "
					    << stageEnums[i];
			continue;
		}

		// Capture must always export dump.
		if (!shouldExportDumpNow(sdRequest->sequence()) &&
		    !isImgsysCaptureStage(stageEnums[i])) {
			continue;
		}
		Dump::Id id = kPeuStageDumpIdMap.at(stageEnums[i]);
		Dump::Metadata dumpMetadata = kDumpMetadata.at(id);
		Dump::Config config = dumpConfig_[id];
		imgSysMetadata[i].common.needDump = true;
		auto dumpFileName = ImagiqAdapter::getDumpFileName({
			.id = id,
			.requestNumber = sdRequest->sequence(),
			.sensorId = sensorId_,
			.timestamp = sessionTimestamp_,
			.workPath = currentExportPath_,
			.frame = std::nullopt,
			.array = std::nullopt,
			.metadata = dumpMetadata,
			.config = config,
		});
		strncpy(imgSysMetadata[i].common.nddfp, dumpFileName.c_str(),
			dumpFileName.size());
		LOG(MtkISP7, Info) << "Requested imgsys driver to dump register --"
				   << " request number: " << sdRequest->sequence()
				   << " stage: " << stageEnums[i]
				   << " dump file prefix: " << dumpFileName;
	}
}

void OnDeviceTuner::tune3ARequest(
	uint32_t internalRequestId, mtk::hal3a::v1_0::mtk_3a_request &aaaRequest)
{
	uint32_t requestNumber = internalRequestId;
	bool isStillCapture = stillCaptureRequestIds_.count(requestNumber) == 1;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) && !isStillCapture)) {
		return;
	}
	aaaRequest.ndd_data.timestamp = sessionTimestamp_;
	aaaRequest.ndd_data.requestNo = internalRequestId;
	aaaRequest.ndd_data.frameNo = internalRequestId;
	aaaRequest.ndd_data.platform = 8188;
	if (isStillCapture) {
		aaaRequest.ndd_data.feature = static_cast<int>(Feature::Capture_lpnr);
		aaaRequest.ndd_category = NSCam::TuningUtils::eCategory::kCAPTURE;
	} else {
		aaaRequest.ndd_data.feature = static_cast<int>(Feature::Preview);
		aaaRequest.ndd_category = NSCam::TuningUtils::eCategory::kSTREAMING;
	}
}

void OnDeviceTuner::tune3AState(uint32_t internalRequestId,
				CaptureFrames &frames,
				mtk::hal3a::v1_0::mtk_3a_result *mtk3AResult)
{
	uint32_t requestNumber = internalRequestId;
	bool isStillCapture = stillCaptureRequestIds_.count(requestNumber) == 1;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) && !isStillCapture)) {
		return;
	}
	MappedFrameBuffer mapped(
		frames.statistics0->get().buffer(),
		MappedFrameBuffer::MapFlag::Read);
	mtk_cam_uapi_meta_raw_stats_0 *stats =
		reinterpret_cast<mtk_cam_uapi_meta_raw_stats_0 *>(
			mapped.planes()[0].data());
	std::vector<uint8_t> merged2AHist;
	ImagiqAdapter::merge2AHistogram(merged2AHist, stats);
	std::vector<uint8_t> data(stats->ae_awb_stats.aao_buf.size);
	std::memcpy(data.data(), reinterpret_cast<uint8_t *>(stats) + stats->ae_awb_stats.aao_buf.offset, data.size());

	std::vector<NamedPointer> namedPointers{
		// 3A Statistics
		{
			.id = Dump::Id::P1_AAO,
			.ptr = reinterpret_cast<uint8_t *>(stats) + stats->ae_awb_stats.aao_buf.offset,
			.size = stats->ae_awb_stats.aao_buf.size },
		{ .id = Dump::Id::P1_AAHO,
		  .ptr = merged2AHist.data(),
		  .size = merged2AHist.size() },
		{ .id = Dump::Id::P1_TSFSO_R1,
		  .ptr = reinterpret_cast<uint8_t *>(stats) + stats->tsf_stats.tsfo_r1_buf.offset,
		  .size = stats->tsf_stats.tsfo_r1_buf.size },
		{ .id = Dump::Id::P1_TSFSO_R2,
		  .ptr = reinterpret_cast<uint8_t *>(stats) + stats->tsf_stats.tsfo_r2_buf.offset,
		  .size = stats->tsf_stats.tsfo_r2_buf.size },
		{ .id = Dump::Id::P1_LTMSO,
		  .ptr = reinterpret_cast<uint8_t *>(stats) + stats->ltm_stats.ltmso_buf.offset,
		  .size = stats->ltm_stats.ltmso_buf.size },
		{ .id = Dump::Id::P1_TNCSYO_R1,
		  .ptr = reinterpret_cast<uint8_t *>(stats) + stats->tncy_stats.tncsyo_buf.offset,
		  .size = stats->tncy_stats.tncsyo_buf.size },
		// 3A Result
		{
			.id = Dump::Id::P1_LTM_OUT,
			.ptr = reinterpret_cast<uint8_t *>(mtk3AResult->tone_result.p_ltm_alg_data),
			.size = static_cast<size_t>(mtk3AResult->tone_result.ltm_alg_data_size),
		},
		{
			.id = Dump::Id::P1_AE_OUT,
			.ptr = reinterpret_cast<uint8_t *>(mtk3AResult->ae_result.p_ae_alg_data),
			.size = static_cast<size_t>(mtk3AResult->ae_result.ae_alg_data_size),
		},
		{
			.id = Dump::Id::P1_FW_ME_TCY_P,
			.ptr = reinterpret_cast<uint8_t *>(mtk3AResult->tone_result.p_me_tcy_in_workbuf_data),
			.size = static_cast<size_t>(mtk3AResult->tone_result.me_tcy_in_workbuf_data_size),
		},
		{
			.id = Dump::Id::P1_FW_ME_TCY_O,
			.ptr = reinterpret_cast<uint8_t *>(mtk3AResult->tone_result.p_me_tcy_fst_o_data),
			.size = static_cast<size_t>(mtk3AResult->tone_result.me_tcy_fst_o_data_size),
		}
	};
	LOG(MtkISP7, Info) << "AE_OUT size: " << mtk3AResult->ae_result.ae_alg_data_size;
	LOG(MtkISP7, Info) << "FW_ME_TCY_P size: " << mtk3AResult->tone_result.me_tcy_in_workbuf_data_size;
	LOG(MtkISP7, Info) << "FW_ME_TCY_O size: " << mtk3AResult->tone_result.me_tcy_fst_o_data_size;
	tune(requestNumber, namedPointers, isStillCapture);
}

void OnDeviceTuner::tuneMeA(uint32_t internalRequestId, MeFrames &frames)
{
	uint32_t requestNumber = internalRequestId;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		internalRequestId, {
					     { Dump::Id::LTR_ME_L1_IMGI_T1, frames.in.meL0->get() },
					     { Dump::Id::LTR_ME_L1_YUVO_T2, frames.out.meL1->get() },
					     { Dump::Id::LTR_ME_L1_META_P2, frames.in.trMeTun->get() },
					     { Dump::Id::ME_3PASS_MODE0_MEI_L0, frames.in.meL0->get() },
					     { Dump::Id::ME_3PASS_MODE0_MEI_L0_P, frames.in.prevMeL0->get() },
					     { Dump::Id::ME_3PASS_MODE0_MEI_L1, frames.out.meL1->get() },
					     { Dump::Id::ME_3PASS_MODE0_MEI_L1_P, frames.in.prevMeL1->get() },
					     { Dump::Id::ME_3PASS_MODE0_MV_L1_M0_P, frames.in.prevMeAMv1->get() },
					     { Dump::Id::ME_3PASS_MODE0_MV_L0_M1_P, frames.in.prevMeBMv0->get() },
					     { Dump::Id::ME_3PASS_MODE0_CONF_MAP, frames.out.meConf0->get() },
					     { Dump::Id::ME_3PASS_MODE0_MV_L0, frames.out.meAMv0->get() },
					     { Dump::Id::ME_3PASS_MODE0_MV_L1, frames.out.meAMv1->get() },
					     { Dump::Id::ME_3PASS_MODE0_FMB_L0, frames.out.meAFmb0->get() },
					     { Dump::Id::ME_3PASS_MODE0_FMB_L1, frames.out.meAFmb1->get() },
					     { Dump::Id::ME_3PASS_MODE0_FST, frames.out.meAFst->get() },
					     { Dump::Id::ME_3PASS_MODE0_META_P2, frames.in.meATun->get() },

					     /* The following are input of ME_3PASS_MODE1 and output of ME_3PASS_MODE0.
					      * Because it will be overwriten by ME_3PASS_MODE1, it should be dumped right
					      * after ME_3PASS_MODE0. */
					     { Dump::Id::ME_3PASS_MODE1_MV_L0_M0, frames.out.meAMv0->get() }, // confirmed
					     { Dump::Id::ME_3PASS_MODE1_FMB_L1_M0, frames.out.meAFmb1->get() }, // ?
				     });
}

void OnDeviceTuner::tuneMeB(uint32_t internalRequestId, MeFrames &frames)
{
	uint32_t requestNumber = internalRequestId;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		internalRequestId, {
					     { Dump::Id::ME_3PASS_MODE1_MEI_L0, frames.in.meL0->get() },
					     { Dump::Id::ME_3PASS_MODE1_MEI_L0_P, frames.in.prevMeL0->get() },
					     { Dump::Id::ME_3PASS_MODE1_MEI_L1_P, frames.in.prevMeL1->get() },
					     { Dump::Id::ME_3PASS_MODE1_MIL, frames.in.meMil->get() },
					     { Dump::Id::ME_3PASS_MODE1_MMAP, frames.out.meMmap[0]->get() },
					     { Dump::Id::ME_3PASS_MODE1_CONF_MAP, frames.out.meConf0->get() },
					     { Dump::Id::ME_3PASS_MODE1_MV_L0, frames.out.meBMv0->get() }, // ??
					     { Dump::Id::ME_3PASS_MODE1_FMB_L0, frames.out.meBFmb0->get() }, // ?
					     { Dump::Id::ME_3PASS_MODE1_LMI, frames.out.meBLmi->get() },
					     { Dump::Id::ME_3PASS_MODE1_FST, frames.out.meBFst->get() },
					     { Dump::Id::ME_3PASS_MODE1_META_P2, frames.in.meBTun->get() },
				     });
}

void OnDeviceTuner::tuneMeMM(uint32_t internalRequestId,
			     SharedMailBox<InfoFrame> tuning)
{
	uint32_t requestNumber = internalRequestId;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		internalRequestId, {
					     { Dump::Id::ME_3PASS_MM_META_P2, tuning->get() },
				     });
}

void OnDeviceTuner::tuneTr(uint32_t internalRequestId, TrFrames &frames)
{
	uint32_t requestNumber = internalRequestId;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		internalRequestId, { { Dump::Id::TR_DSMAP_MMAP, frames.in.meMmap[0]->get() },
				       { Dump::Id::TR_DSMAP_MMAP_DS0, frames.in.meMmap[1]->get() },
				       { Dump::Id::TR_DSMAP_MMAP_DS1, frames.in.meMmap[2]->get() },
				       { Dump::Id::TR_DSMAP_MMAP_DS2, frames.in.meMmap[3]->get() },
				       { Dump::Id::TR_Y2Y_F1_IMGI_T1, frames.in.p1F1->get() },
				       { Dump::Id::TR_Y2Y_F1_YUVO_T2, frames.out.dipImgi[2]->get() },
				       { Dump::Id::TR_Y2Y_F1_YUVO_T3, frames.out.dipImgi[3]->get() },
				       { Dump::Id::TR_Y2Y_F1_YUVO_T4, frames.out.dipImgi[4]->get() },
				       { Dump::Id::TR_Y2Y_F1_META_P2, frames.in.trTunF1->get() },
				       { Dump::Id::TR_Y2Y_F4_IMGI_T1, frames.out.dipImgi[4]->get() },
				       { Dump::Id::TR_Y2Y_F4_YUVO_T2, frames.out.dipImgi[5]->get() },
				       { Dump::Id::TR_Y2Y_F4_YUVO_T3, frames.out.dipImgi[6]->get() },
				       { Dump::Id::TR_Y2Y_F4_META_P2, frames.in.trTunF4->get() },
				       { Dump::Id::TR_Y2Y_Conf_IMGI_T1, frames.in.meConf0->get() },
				       { Dump::Id::TR_Y2Y_Conf_F4_YUVO_T5, frames.out.meConf4->get() },
				       { Dump::Id::TR_Y2Y_Conf_F5_YUVO_T5, frames.out.meConf5->get() } });
}

void OnDeviceTuner::tuneDip1(uint32_t internalRequestId, Dip1Frames &frames)
{
	uint32_t requestNumber = internalRequestId;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	std::vector<NamedFrame> namedFrames{
		{ Dump::Id::WPE_LTR_Y2Y_F1_WPEI, frames.in.prevImg4oF1->get() },
		{ Dump::Id::WPE_LTR_Y2Y_F1_WPE_MAP, frames.out.meMmap[0]->get() },
		{ Dump::Id::WPE_LTR_Y2Y_F1_WPEO, frames.out.dipVipi[1]->get() },
		{ Dump::Id::WPE_LTR_Y2Y_F1_YUVO_T2, frames.out.dipVipi[2]->get() },
		{ Dump::Id::WPE_LTR_Y2Y_F1_YUVO_T3, frames.out.dipVipi[3]->get() },
		{ Dump::Id::WPE_LTR_Y2Y_F1_YUVO_T4, frames.out.dipVipi[4]->get() },
		{ Dump::Id::WPE_LTR_Y2Y_F1_YUVO_T5, frames.out.dipVbi[2]->get() },
		{ Dump::Id::WPE_LTR_Y2Y_F1_META_P2, frames.in.ltrTunF1->get() },
		{ Dump::Id::LTR_VBI_IMGI_T1, frames.out.dipVbi[2]->get() },
		{ Dump::Id::LTR_VBI_YUVO_T2, frames.out.dipVbi[3]->get() },
		{ Dump::Id::LTR_VBI_YUVO_T3, frames.out.dipVbi[4]->get() },
		{ Dump::Id::LTR_VBI_YUVO_T4, frames.out.dipVbi[5]->get() },
		{ Dump::Id::LTR_VBI_META_P2, frames.in.ltrTunVbi->get() },
		{ Dump::Id::LTR_Y2Y_F4_IMGI_T1, frames.out.dipVipi[4]->get() },
		{ Dump::Id::LTR_Y2Y_F4_YUVO_T2, frames.out.dipVipi[5]->get() },
		{ Dump::Id::LTR_Y2Y_F4_YUVO_T3, frames.out.dipVipi[6]->get() },
		{ Dump::Id::LTR_Y2Y_F4_META_P2, frames.in.ltrTunF4->get() },
		{ Dump::Id::WPE_WghtMap_META_P2, frames.in.wpeTun->get() },
		{ Dump::Id::P2_MS_F1_IMG4O, frames.out.img4oF1->get() },
		{ Dump::Id::P2_MS_F_SMALL_RECI_D1, frames.out.dipImgi[6]->get() }
	};

	// Stage WPE_WghtMap_WPEI_F0 until WPE_WghtMap_WPEI_F5
	for (int level = 0; level <= 5; level++) {
		namedFrames.push_back({ Dump::kWpeInputImageDumpIds[level],
					frames.in.prevDipTnrwo[level]->get() });
		namedFrames.push_back({ Dump::kWpeWeightMapDumpIds[level],
					frames.out.wpeVeci[level]->get() });
		namedFrames.push_back({ Dump::kWpeOutputImageDumpIds[level],
					frames.out.dipTnrwi[level]->get() });
	}

	// Stage P2_MS_F1 until P2_MS_F4 + P2_MS_F_SMALL (lv5) + P2_IDI (lv6)
	for (int i = 0; i < 6; i++) {
		int level = i + 1;
		namedFrames.push_back({ Dump::kDip1ImgiDumpIds[i],
					frames.out.dipImgi[level]->get() });
		namedFrames.push_back({ Dump::kDip1VipiDumpIds[i],
					frames.out.dipVipi[level]->get() });
		// P2_IDI_TNRSI uses previous frame's tnrso
		if (level == 6)
			namedFrames.push_back({ Dump::kDip1TnrsiDumpIds[i],
						frames.in.preDipTnrso->get() });
		else
			namedFrames.push_back({ Dump::kDip1TnrsiDumpIds[i],
						frames.out.dipTnrso->get() });

		namedFrames.push_back({ Dump::kDip1TnrsoDumpIds[i],
					frames.out.dipTnrso->get() });
		if (Dump::kDip1Img3oDumpIds[i] == Dump::Id::P2_IDI_IMG3O)
			namedFrames.push_back({ Dump::Id::P2_IDI_IMG3O,
						frames.out.tnrlfdi->get() });
		else
			namedFrames.push_back({ Dump::kDip1Img3oDumpIds[i],
						frames.out.img3o[level]->get() });
		namedFrames.push_back({ Dump::kDip1MetaP2DumpIds[i],
					frames.in.dipTun[level]->get() });
	}

	// Stage P2_MS_F1 until P2_MS_F4 + P2_MS_F_SMALL (lv5)
	for (int i = 0; i < 5; i++) {
		int level = i + 1;
		namedFrames.push_back({ Dump::kDip1TnrwiDumpIds[i],
					frames.out.dipTnrwi[level]->get() });
		namedFrames.push_back({ Dump::kDip1TnrciDumpIds[i],
					frames.out.dipTnrci[level]->get() });
		namedFrames.push_back({ Dump::kDip1TnrliDumpIds[i],
					frames.out.tnrlfdi->get() });
		namedFrames.push_back({ Dump::kDip1TnrvbiDumpIds[i],
					frames.out.dipVbi[level]->get() });
		namedFrames.push_back({ Dump::kDip1TnrmoDumpIds[i],
					frames.out.dipTnrmo[level]->get() });
		namedFrames.push_back({ Dump::kDip1TnrwoDumpIds[i],
					frames.out.dipTnrwo[level]->get() });
	}

	// Stage P2_MS_F1 until P2_MS_F4
	for (int i = 0; i < 4; i++) {
		int level = i + 1;
		namedFrames.push_back({ Dump::kDip1ReciDumpIds[i],
					frames.out.reci[level]->get() });
		namedFrames.push_back({ Dump::kDip1TnrmiDumpIds[i],
					frames.out.dipTnrmi[level]->get() });
	}
	tune(internalRequestId, namedFrames);
}

void OnDeviceTuner::tuneDip2(
	Request *request, uint32_t internalRequestId, Dip2Frames &frames,
	FrameBuffer *videoOut1, FrameBuffer *videoOut2)
{
	uint32_t requestNumber = internalRequestId;
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	std::vector<NamedFrame> namedFrames{
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_WPETI, frames.in.prevImg4oF0->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_WPET_MAP, frames.in.meMmap[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRSI, frames.out.dipTnrso->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRWI, frames.in.dipTnrwi[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRMI, frames.in.dipTnrmi[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRCI, frames.in.dipTnrci[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRLI, frames.in.tnrlfdi->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRSO, frames.out.dipTnrso->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRWO, frames.out.dipTnrwo[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_RECI_D1, frames.in.reci[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_IMG3O, frames.out.img3o[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_IMG4O, frames.out.img4oF0->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_IMGI_D1, frames.in.dipImgi[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_META_P2, frames.in.dipTun[0]->get() },
	};

	if (videoOut1) {
		InfoFrame video1 = getFrameInfoFromRequest(request, videoOut1);
		namedFrames.push_back({ Dump::Id::WPE_P2_PQDIP_MS_F0_WDMAO, video1 });
	}
	if (videoOut2) {
		InfoFrame video2 = getFrameInfoFromRequest(request, videoOut2);
		namedFrames.push_back({ Dump::Id::WPE_P2_PQDIP_MS_F0_WDMAO, video2 });
	}
	tune(internalRequestId, namedFrames);
}

void OnDeviceTuner::tuneXtr(uint32_t internalRequestId, XtrFrames &frames)
{
	if (!enabled_) {
		return;
	}
	// Capture: always export dumps!
	std::vector<NamedFrame> namedFrames{
		{ Dump::Id::TR_R2Y_IMGI_T1, frames.in.p1Raw->get() },
		{ Dump::Id::TR_R2Y_YUVO_T1, frames.out.dipImgi[0]->get() },
		{ Dump::Id::TR_R2Y_YUVO_T2, frames.out.dipImgi[1]->get() },
		{ Dump::Id::TR_R2Y_YUVO_T3, frames.out.dipImgi[2]->get() },
		{ Dump::Id::TR_R2Y_YUVO_T4, frames.out.dipImgi[3]->get() },
		{ Dump::Id::TR_R2Y_META_P2, frames.in.xtrTun->get() },
	};
	tune(internalRequestId, namedFrames, true);
}

void OnDeviceTuner::tuneLpnrDip(Request *request, uint32_t internalRequestId,
				LpnrDipFrames &frames,
				std::vector<SharedMailBox<InfoFrame>> reci,
				std::vector<SharedMailBox<InfoFrame>> dipImg3o,
				FrameBuffer *still1Output,
				FrameBuffer *still2Output)
{
	if (!enabled_) {
		return;
	}
	// Capture: always export dumps!
	std::vector<NamedFrame> namedFrames{
		{ Dump::Id::P2_MS_F3_IMGI_D1_LPNR, frames.in.dipImgi[3]->get() },
		{ Dump::Id::P2_MS_F3_IMG3O_LPNR, dipImg3o[3]->get() },
		{ Dump::Id::P2_MS_F3_META_P2_LPNR, frames.in.dipTun[3]->get() },
		{ Dump::Id::P2_MS_F2_IMGI_D1_LPNR, frames.in.dipImgi[2]->get() },
		{ Dump::Id::P2_MS_F2_RECI_D1_LPNR, reci[2]->get() },
		{ Dump::Id::P2_MS_F2_IMG3O_LPNR, dipImg3o[2]->get() },
		{ Dump::Id::P2_MS_F2_META_P2_LPNR, frames.in.dipTun[2]->get() },
		{ Dump::Id::P2_MS_F1_IMGI_D1_LPNR, frames.in.dipImgi[1]->get() },
		{ Dump::Id::P2_MS_F2_RECI_D1_LPNR, reci[1]->get() },
		{ Dump::Id::P2_MS_F1_IMG3O_LPNR, dipImg3o[1]->get() },
		{ Dump::Id::P2_MS_F1_META_P2_LPNR, frames.in.dipTun[1]->get() },
	};

	if ((frames.in.highIsoMode->valid() && !frames.in.highIsoMode->get())) {
		namedFrames.push_back({ Dump::Id::P2_MS_F0_PQ_DIP_IMGI_D1, frames.in.dipImgi[0]->get() });
		namedFrames.push_back({ Dump::Id::P2_MS_F0_PQ_DIP_RECI_D1, reci[0]->get() });
		namedFrames.push_back({ Dump::Id::P2_MS_F0_PQ_DIP_IMG3O, dipImg3o[0]->get() });

		if (still1Output) {
			InfoFrame still1Frame = getFrameInfoFromRequest(request, still1Output);
			namedFrames.push_back({ Dump::Id::P2_MS_F0_PQ_DIP_WDMAO, still1Frame });
		}

		if (still2Output) {
			InfoFrame still2Frame = getFrameInfoFromRequest(request, still2Output);
			namedFrames.push_back({ Dump::Id::P2_MS_F0_PQ_DIP_WDMAO, still2Frame });
		}

		namedFrames.push_back({ Dump::Id::P2_MS_F0_PQ_DIP_META_P2, frames.in.dipTunPq->get() });
	}

	tune(internalRequestId, namedFrames, true);
}

bool OnDeviceTuner::isDumpStillCapture(uint32_t internalRequestId)
{
	return (enabled_ && stillCaptureRequestIds_.count(internalRequestId) == 1);
}

} // namespace libcamera
