/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2021, Google inc.
 *
 * IPU3 Black Level Correction control
 */

#include "blc.h"

/**
 * \file blc.h
 * \brief IPU3 Black Level Correction control
 */

namespace libcamera {

namespace ipa::ipu3::algorithms {

/**
 * \class BlackLevelCorrection
 * \brief A class to handle black level correction
 *
 * The pixels output by the camera normally include a black level, because
 * sensors do not always report a signal level of '0' for black. Pixels at or
 * below this level should be considered black. To achieve that, the ImgU BLC
 * algorithm subtracts a configurable offset from all pixels.
 *
 * The black level can be measured at runtime from an optical dark region of the
 * camera sensor, or measured during the camera tuning process. The first option
 * isn't currently supported.
 */

BlackLevelCorrection::BlackLevelCorrection()
{
}

/**
 * \brief Fill in the parameter structure, and enable black level correction
 * \param[in] context The shared IPA context
 * \param[in] frame The frame context sequence number
 * \param[in] frameContext The FrameContext for this frame
 * \param[out] params The IPU3 parameters
 *
 * Populate the IPU3 parameter structure with the correction values for each
 * channel and enable the corresponding ImgU block processing.
 */
void BlackLevelCorrection::prepare([[maybe_unused]] IPAContext &context,
				   [[maybe_unused]] const uint32_t frame,
				   [[maybe_unused]] IPAFrameContext &frameContext,
				   [[maybe_unused]] ipu3_uapi_params *params)
{
	/*
   * Default black level values work better than the configured ones
   * so we can just use those.
	 */
}

REGISTER_IPA_ALGORITHM(BlackLevelCorrection, "BlackLevelCorrection")

} /* namespace ipa::ipu3::algorithms */

} /* namespace libcamera */
