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

	const uint32_t sensor_idx_;

	mtk::hal3a::IHal3A *m_hal3a_ = nullptr;

	std::shared_ptr<mtk::hal3a::IPeripheralController> peripheralController_ = nullptr;
};

} /* namespace libcamera */
