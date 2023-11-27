/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * on_device_tuner.h - MtkISP7 On Device Tuner module.
 */

#pragma once

#include <cstdint>
#include <filesystem>
#include <map>
#include <memory>
#include <set>
#include <string>

#include <libcamera/request.h>

#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/aaa_hal_def.h"
#include "pipeline/mtkisp7/imgsys/single_device.h"
#include "pipeline/mtkisp7/odt/camsys_driver_debug.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/dump.h"
#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"
#include "platform/mtkisp7/halisp/IspControls.h"
#include "platform/mtkisp7/halisp/TuningParam.h"
#include "tuning_mapping/cam_idx_struct_ext_pub.h"

namespace libcamera {

struct CaptureFrames;
struct MeFrames;
struct TrFrames;
struct Dip1Frames;
struct Dip2Frames;
struct XtrFrames;
struct LpnrDipFrames;

class OnDeviceTuner
{
public:
	void initialize();
	void configure(const std::string &sensorId, unsigned int camsysIndex);

	void notifyRequestBegin(int requestNumber);
	void notifyRequestEnd(int requestNumber);
	void notifyStillCapture(int requestNumber);

	// P1 Camsys
	void tuneCamsys(Request *request, CaptureFrames &frames);

	// HAL ISP
	bool tuneCamsysHalIsp(
		Request *request, mtk::isphal::v1_0::TuningParamP1 &tuningParam);
	void tuneImgsysHalIsp(
		Request *request, mtk::isphal::v1_0::TuningParamDip &tuningParam,
		mtk::isphal::v1_0::ReturnParamDip &tuningResult,
		mtk::hal3a::v1_0::mtk_3a_result &mtk3AResult,
		EStage_T stage);
	void tuneExif(Request *request,
		      const mtk::isphal::v1_0::ExifInfo3A &exif3a,
		      const mtk::isphal::v1_0::ExifInfoP2 &exifIsp,
		      EStage_T stage);

	// 3A
	void tune3ARequest(
		Request *request, mtk::hal3a::v1_0::mtk_3a_request &r3aRequest);
	void tune3AState(Request *request, CaptureFrames &frames,
			 mtk::hal3a::v1_0::mtk_3a_result *mtk3AResult);

	// P2 Imgsys driver
	void tuneImgsysMetadata(
		SingleDeviceRequest *sdRequest,
		InfoFrame &metaFrame);

	// MCNR
	void tuneMe(Request *request, MeFrames &frames);
	void tuneTr(Request *request, TrFrames &frames);
	void tuneDip1(Request *request, Dip1Frames &frames);
	void tuneDip2(
		Request *request, Dip2Frames &frames,
		FrameBuffer *videoOut1, FrameBuffer *videoOut2);

	// LPNR
	void tuneXtr(Request *request, XtrFrames &frames);
	void tuneLpnrDip(Request *request, LpnrDipFrames &frames,
			 std::vector<SharedMailBox<InfoFrame>> reci,
			 std::vector<SharedMailBox<InfoFrame>> dipImg3o,
			 FrameBuffer *still1Output,
			 FrameBuffer *still2Output);
	bool isLowIsoLpnrEnforced();

private:
	struct NamedFrame {
		Dump::Id id;
		InfoFrame &frame;
	};
	struct NamedPointer {
		Dump::Id id;
		uint8_t *ptr;
		size_t size;
	};

	std::vector<ImagiqAdapter::ExportResult> batchExport(
		const std::vector<Dump> &dumps);
	void batchImport(const std::vector<Dump> &dumps);
	void batchPrepareReimport(
		const std::vector<ImagiqAdapter::ExportResult> &exportResults);
	InfoFrame getFrameInfoFromRequest(
		Request *request, FrameBuffer *buffer);
	bool isImgsysCaptureStage(PEU_Stage stage);
	void loadTuneRequest(int requestNumber);
	bool parseHalIspNdd(
		Request *request,
		mtk::isphal::v1_0::NddInfo &ndd);
	int prepareNewExportDirectory();
	bool shouldExportDumpNow(uint32_t requestNumber);
	bool shouldImportDumpNow(uint32_t requestNumber);
	void tune(uint32_t requestNumber,
		  std::vector<NamedFrame> namedFrames,
		  bool forceDump = false);
	void tune(uint32_t requestNumber,
		  std::vector<NamedPointer> namedPointers,
		  bool forceDump = false);

	bool enabled_;
	bool enforceLowIsoLpnr_;
	int sessionTimestamp_;
	std::string sensorId_;

	int prevStartedRequestNum_;
	int prevEndedRequestNum_;
	uint32_t exportBegin_;
	uint32_t exportEnd_;
	uint32_t importBegin_;
	uint32_t importEnd_;
	std::filesystem::path currentExportPath_;

	std::map<Dump::Id, Dump::Config> dumpConfig_;
	std::set<int> stillCaptureRequestIds_;

	std::unique_ptr<CamsysDebug> camsysDebug_;
};

} // namespace libcamera
