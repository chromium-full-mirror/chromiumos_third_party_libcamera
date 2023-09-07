/*
 * Copyright (C) 2023, Google Inc.
 *
 * hal_3a.h - Wrapper of MtkISP7 mtk::hal3a::IHal3A
 */

#include "mtkcam-core/aaa/peripheralcontroller/include/IPeripheralController.h"
#include "mtkcam-core/include/mtkcam-core/aaahal/aaa_hal/IHal3A.h"

namespace libcamera {

class Hal3A
{
public:
	Hal3A(const uint32_t sensor_idx);

	void start();

private:
	void init();
	void getInitialInfo();
	void config();
	void startInternal();

	const uint32_t sensor_idx_;

	mtk::hal3a::IHal3A *m_hal3a_ = nullptr;

	std::shared_ptr<mtk::hal3a::IPeripheralController> peripheralController_ = nullptr;

	mtk::hal3a::v1_0::mtk_hw_initial_setting initialSetting_ = {};
	mtk::hal3a::v1_0::mtk_3a_result r3AResult_ = {};
};

} /* namespace libcamera */
