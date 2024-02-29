/*
 * Copyright (C) 2024, Google Inc.
 *
 * imgsys_task.cpp - Base class of task to call getImgSysMetaTuning
 */

#include "imgsys_task.h"

#include "libcamera/base/log.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

ImgSysTask::ImgSysTask(Scheduler *scheduler, const std::string &id,
		       uint32_t camSysMetaRequestId, uint32_t internalRequestId,
		       Feature feature, IPADelegate *ipa)
	: Task(scheduler, id), camSysMetaRequestId_(camSysMetaRequestId),
	  internalRequestId_(internalRequestId), feature_(feature), ipa_(ipa)
{
}

void ImgSysTask::getImgSysMetaTuning(
	const bool needCropTNC16x9,
	const std::vector<ipa::mtkisp7::ImgMetaRequestData> &imgMetaRequests)
{
	ipa_->getImgSysMetaTuning(this, camSysMetaRequestId_,
				  internalRequestId_, needCropTNC16x9,
				  feature_, imgMetaRequests);
}

} /* namespace libcamera */
