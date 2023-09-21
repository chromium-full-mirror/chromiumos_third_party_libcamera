/*
 * Copyright (C) 2023, Google Inc.
 *
 * hal_3a.h - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#pragma once

#include "libcamera/internal/dma_heaps.h"

#include "libcamera/framebuffer.h"
#include "mtkcam-core/aaa/peripheralcontroller/include/IPeripheralController.h"
#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/IHal3A.h"
#include "mtkcam-interfaces/include/mtkcam-interfaces/isphal/IHalISPAdapter.h"

namespace libcamera {

class Hal3A
{
public:
	Hal3A(const uint32_t sensor_idx, DmaHeap *dmaHeap);

	void start();

	void doCalculation(FrameBuffer *statistics0, uint64_t timestamp);

	std::pair<uint32_t, uint32_t> getExposureAndGain();

	mtk::hal3a::v1_0::mtk_3a_result r3AResult_ = {};

private:
	void init();
	void getInitialInfo();
	void config();
	void startInternal();

	const uint32_t sensor_idx_;
	DmaHeap *dmaHeap_;

	mtk::hal3a::IHal3A *m_hal3a_ = nullptr;

	std::shared_ptr<mtk::ispcf::IHalISPAdapter> m_isp_hal_;

	std::shared_ptr<mtk::hal3a::IPeripheralController> peripheralController_ = nullptr;

	mtk::hal3a::v1_0::mtk_hw_initial_setting initialSetting_ = {};

	UniqueFD fd_;
	mtk_cam_uapi_meta_raw_stats_cfg *meta_addr_;
};

} /* namespace libcamera */
