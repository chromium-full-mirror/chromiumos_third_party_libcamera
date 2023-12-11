/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.cpp - MtkISP7 On Device Tuner module.
 */

#include "pipeline/mtkisp7/odt/on_device_tuner.h"

#include <cstdint>
#include <filesystem>
#include <fstream>
#include <optional>

#include <libcamera/base/log.h>

#include <libcamera/stream.h>

#include "linux/mtkisp7/drv/7.1/ctrl_meta.h"
#include "pipeline/mtkisp7/camsys/capture.h"
#include "pipeline/mtkisp7/imgsys/lpnr.h"
#include "pipeline/mtkisp7/imgsys/mcnr.h"
#include "pipeline/mtkisp7/odt/camsys_driver_debug.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/static_metadata/dump_metadata.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

namespace {

constexpr const char *kEnableTuningPath = "/run/camera/enable_tuning";
constexpr const char *kExportRequestPath = "/run/camera/export_dump";
constexpr const char *kImportRequestPath = "/run/camera/import_dump";
// Dump key must be exactly 9 digits.
constexpr int kMinDumpKey = 1e8;
constexpr int kMaxDumpKey = 1e9 - 1;
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
		if (dump.config.enableImport) {
			ImagiqAdapter::importDump(dump);
		}
	}
}

void OnDeviceTuner::batchPrepareReimport(
	const std::vector<ImagiqAdapter::ExportResult> &exportResults)
{
	for (const auto &result : exportResults) {
		if (!result.errorCode.has_value()) {
			ImagiqAdapter::prepareReimport(result);
		}
	}
}

void OnDeviceTuner::configure(
	const std::string &sensorId, unsigned int camsysIndex)
{
	enabled_ = false;
	camsysDebug_.reset();

	if (!std::filesystem::exists(kEnableTuningPath)) {
		return;
	}

	int ret = ImagiqAdapter::loadConfig(dumpConfig_, kWorkDir);
	if (ret) {
		LOG(MtkISP7, Error) << "Attempted to enable pipeline tuning, but failed"
				    << " to load the config, error code: " << ret;
		return;
	}

	LOG(MtkISP7, Warning) << "Pipeline tuning enabled";
	enabled_ = true;
	exportBegin_ = 0;
	exportEnd_ = 0;
	importBegin_ = 0;
	importEnd_ = 0;
	sensorId_ = sensorId;
	camsysDebug_ = CamsysDebug::create(camsysIndex);

	// Immediately create one directory for any capture dumps.
	prepareNewExportDirectory();
}

void OnDeviceTuner::loadTuneRequest(int requestNumber)
{
	if (!enabled_) {
		return;
	}
	int exportRequestCount = 0;
	std::ifstream exportRequestFile(kExportRequestPath);
	if (exportRequestFile.good()) {
		exportRequestFile >> exportRequestCount;
		LOG(MtkISP7, Info) << "Loaded dump export request from file: "
				   << exportRequestCount << " frames";
		exportRequestFile.close();
		std::filesystem::path path(kExportRequestPath);
		std::filesystem::remove(path);
		exportBegin_ = requestNumber;
		exportEnd_ = requestNumber + exportRequestCount;
		prepareNewExportDirectory();
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
	return InfoFrame(streamCfg.pixelFormat, streamCfg.size, buffer);
}

bool OnDeviceTuner::isImgsysCaptureStage(PEU_Stage stage)
{
	return std::find(
		       kImgsysCaptureStages.begin(), kImgsysCaptureStages.end(), stage) !=
	       kImgsysCaptureStages.end();
}

int OnDeviceTuner::prepareNewExportDirectory()
{
	std::filesystem::path newPath;
	std::filesystem::path workPath(kWorkDir);
	int dumpKey;
	for (dumpKey = kMinDumpKey; dumpKey <= kMaxDumpKey; dumpKey++) {
		std::filesystem::path exportFolderName = "UKey" + std::to_string(dumpKey);
		newPath.assign(workPath / exportFolderName);
		if (!std::filesystem::exists(newPath)) {
			break;
		}
	}
	if (dumpKey > kMaxDumpKey) {
		LOG(MtkISP7, Error) << "Failed to create dump directory: "
				    << "too many dumps already.";
		return -EEXIST;
	}
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
				  .workPath = currentExportPath_,
				  .frame = namedFrame.frame,
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

void OnDeviceTuner::tuneCamsys(Request *request, CaptureFrames &frames)
{
	uint32_t requestNumber = request->sequence();
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		request->sequence(), { { Dump::Id::P1_IMGO, frames.raw->get() },
				       { Dump::Id::P1_YUVO_R1, frames.yuvo1->get() },
				       { Dump::Id::P1_YUVO_R2, frames.yuvo2->get() },
				       { Dump::Id::P1_DRZS4NO_R3, frames.me->get() } });

	// Driver's registers
	Dump::Config drvRegConfig = dumpConfig_[Dump::Id::P1_REG_P1];
	if (shouldExportDumpNow(requestNumber) && drvRegConfig.enableExport && camsysDebug_) {
		Dump registerDump{
			.id = Dump::Id::P1_REG_P1,
			.requestNumber = requestNumber,
			.sensorId = sensorId_,
			.workPath = currentExportPath_,
			.frame = std::nullopt,
			.metadata = kDumpMetadata.at(Dump::Id::P1_REG_P1),
			.config = drvRegConfig
		};
		std::filesystem::path dumpPath =
			ImagiqAdapter::getDumpFileName(registerDump);
		camsysDebug_->exportDump(
			frames.raw->get().buffer()->metadata().sequence,
			requestNumber, dumpPath);
	}
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
			.workPath = currentExportPath_,
			.frame = std::nullopt,
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

void OnDeviceTuner::tuneMe(Request *request, MeFrames &frames)
{
	uint32_t requestNumber = request->sequence();
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		request->sequence(), {
					     { Dump::Id::LTR_ME_L1_IMGI_T1, frames.in.meL0->get() },
					     { Dump::Id::LTR_ME_L1_YUVO_T2, frames.out.meL1->get() },
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
					     { Dump::Id::ME_3PASS_MODE1_MEI_L0, frames.in.meL0->get() },
					     { Dump::Id::ME_3PASS_MODE1_MEI_L0_P, frames.in.prevMeL0->get() },
					     { Dump::Id::ME_3PASS_MODE1_MEI_L1_P, frames.in.prevMeL1->get() },
					     { Dump::Id::ME_3PASS_MODE1_MV_L0_M0, frames.out.meAMv0->get() }, // confirmed
					     { Dump::Id::ME_3PASS_MODE1_MV_L0, frames.out.meBMv1->get() }, // ??
					     { Dump::Id::ME_3PASS_MODE1_MIL, frames.in.meMil->get() },
					     { Dump::Id::ME_3PASS_MODE1_MMAP, frames.out.meMmap[0]->get() },
					     { Dump::Id::ME_3PASS_MODE1_CONF_MAP, frames.out.meConf0->get() },
					     { Dump::Id::ME_3PASS_MODE1_FMB_L1_M0,  frames.out.meAFmb1->get()}, // ?
					     { Dump::Id::ME_3PASS_MODE1_FMB_L0, frames.out.meBFmb0->get() }, // ?
					     { Dump::Id::ME_3PASS_MODE1_LMI, frames.out.meBLmi->get() },
					     { Dump::Id::ME_3PASS_MODE1_FST, frames.out.meBFst->get() },
				     });
}

void OnDeviceTuner::tuneTr(Request *request, TrFrames &frames)
{
	uint32_t requestNumber = request->sequence();
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	tune(
		request->sequence(), { { Dump::Id::TR_DSMAP_MMAP, frames.in.meMmap[0]->get() },
				       { Dump::Id::TR_DSMAP_MMAP_DS0, frames.in.meMmap[1]->get() },
				       { Dump::Id::TR_DSMAP_MMAP_DS1, frames.in.meMmap[2]->get() },
				       { Dump::Id::TR_DSMAP_MMAP_DS2, frames.in.meMmap[3]->get() },
				       { Dump::Id::TR_Y2Y_F1_IMGI_T1, frames.in.p1F1->get() },
				       { Dump::Id::TR_Y2Y_F1_YUVO_T2, frames.out.dipImgi[2]->get() },
				       { Dump::Id::TR_Y2Y_F1_YUVO_T3, frames.out.dipImgi[3]->get() },
				       { Dump::Id::TR_Y2Y_F1_YUVO_T4, frames.out.dipImgi[4]->get() },
				       { Dump::Id::TR_Y2Y_F4_IMGI_T1, frames.out.dipImgi[4]->get() },
				       { Dump::Id::TR_Y2Y_F4_YUVO_T2, frames.out.dipImgi[5]->get() },
				       { Dump::Id::TR_Y2Y_F4_YUVO_T3, frames.out.dipImgi[6]->get() },
				       { Dump::Id::TR_Y2Y_Conf_IMGI_T1, frames.in.meConf0->get() },
				       { Dump::Id::TR_Y2Y_Conf_YUVO_T5, frames.out.meConf5->get() } });
}

void OnDeviceTuner::tuneDip1(Request *request, Dip1Frames &frames)
{
	uint32_t requestNumber = request->sequence();
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
		{ Dump::Id::LTR_VBI_IMGI_T1, frames.out.dipVbi[2]->get() },
		{ Dump::Id::LTR_VBI_YUVO_T2, frames.out.dipVbi[3]->get() },
		{ Dump::Id::LTR_VBI_YUVO_T3, frames.out.dipVbi[4]->get() },
		{ Dump::Id::LTR_VBI_YUVO_T4, frames.out.dipVbi[5]->get() },
		{ Dump::Id::LTR_Y2Y_F4_IMGI_T1, frames.out.dipVipi[4]->get() },
		{ Dump::Id::LTR_Y2Y_F4_YUVO_T2, frames.out.dipVipi[5]->get() },
		{ Dump::Id::LTR_Y2Y_F4_YUVO_T3, frames.out.dipVipi[6]->get() },
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
		namedFrames.push_back({ Dump::kDip1TnrsiDumpIds[i],
					frames.out.dipTnrso->get() });
		namedFrames.push_back({ Dump::kDip1TnrsoDumpIds[i],
					frames.out.dipTnrso->get() });
		namedFrames.push_back({ Dump::kDip1Img3oDumpIds[i],
					frames.out.img3o[level]->get() });
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
	tune(request->sequence(), namedFrames);
}

void OnDeviceTuner::tuneDip2(
	Request *request, Dip2Frames &frames,
	FrameBuffer *videoOut1, FrameBuffer *videoOut2)
{
	uint32_t requestNumber = request->sequence();
	if (!enabled_ || (!shouldExportDumpNow(requestNumber) &&
			  !shouldImportDumpNow(requestNumber))) {
		return;
	}
	std::vector<NamedFrame> namedFrames{
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_WPETI, frames.in.prevImg4oF0->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_WPET_MAP, frames.in.wpeVeci[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRSI, frames.out.dipTnrso->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRWI, frames.in.dipTnrwi[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRMI, frames.in.dipTnrmi[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRCI, frames.in.dipTnrci[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRLI, frames.in.tnrlfdi->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRSO, frames.out.dipTnrso->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_TNRWO, frames.out.dipTnrwo[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_RECI_D1, frames.in.reci[0]->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_IMG4O, frames.out.img4oF0->get() },
		{ Dump::Id::WPE_P2_PQDIP_MS_F0_IMGI_D1, frames.in.dipImgi[0]->get() }
	};

	if (videoOut1) {
		InfoFrame video1 = getFrameInfoFromRequest(request, videoOut1);
		namedFrames.push_back({ Dump::Id::WPE_P2_PQDIP_MS_F0_WDMAO, video1 });
	}
	if (videoOut2) {
		InfoFrame video2 = getFrameInfoFromRequest(request, videoOut2);
		namedFrames.push_back({ Dump::Id::WPE_P2_PQDIP_MS_F0_WDMAO, video2 });
	}
	tune(request->sequence(), namedFrames);
}

void OnDeviceTuner::tuneXtr(Request *request, XtrFrames &frames)
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
		{ Dump::Id::TR_R2Y_YUVO_T4, frames.out.dipImgi[3]->get() }
	};
	tune(request->sequence(), namedFrames, true);
}

void OnDeviceTuner::tuneLpnrDip(Request *request, LpnrDipFrames &frames,
				std::vector<SharedMailBox<InfoFrame>> reci,
				std::vector<SharedMailBox<InfoFrame>> dipImg3o,
				FrameBuffer *stillOutput)
{
	if (!enabled_) {
		return;
	}
	// Capture: always export dumps!
	InfoFrame stillFrame = getFrameInfoFromRequest(request, stillOutput);
	std::vector<NamedFrame> namedFrames{
		{ Dump::Id::P2_MS_F3_IMGI_D1_LPNR, frames.in.dipImgi[3]->get() },
		{ Dump::Id::P2_MS_F3_IMG3O_LPNR, dipImg3o[3]->get() },
		{ Dump::Id::P2_MS_F2_IMGI_D1_LPNR, frames.in.dipImgi[2]->get() },
		{ Dump::Id::P2_MS_F2_RECI_D1_LPNR, reci[2]->get() },
		{ Dump::Id::P2_MS_F2_IMG3O_LPNR, dipImg3o[2]->get() },
		{ Dump::Id::P2_MS_F1_IMGI_D1_LPNR, frames.in.dipImgi[1]->get() },
		{ Dump::Id::P2_MS_F2_RECI_D1_LPNR, reci[1]->get() },
		{ Dump::Id::P2_MS_F1_IMG3O_LPNR, dipImg3o[1]->get() },
		{ Dump::Id::P2_MS_F0_PQ_DIP_IMGI_D1, frames.in.dipImgi[0]->get() },
		{ Dump::Id::P2_MS_F0_PQ_DIP_RECI_D1, reci[0]->get() },
		{ Dump::Id::P2_MS_F0_PQ_DIP_IMG3O, dipImg3o[0]->get() },
		{ Dump::Id::P2_MS_F0_PQ_DIP_WDMAO, stillFrame }
	};
	tune(request->sequence(), namedFrames, true);
}

} // namespace libcamera
