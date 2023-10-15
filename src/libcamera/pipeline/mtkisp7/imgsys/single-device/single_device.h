/*
 * Copyright (C) 2022 MediaTek Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#pragma once

#include <stdint.h>
#include <cstddef>
#include <vector>

#include "libcamera/internal/info_frame.h"

#include "BuiltinTypes.h"
#include "eightcc.h"
#include "IImgStreamDef.h"
#include "ImageFormat.h"
#include "ImgPortDef.h"
#include "UITypes.h"

#include "linux/mtkisp7/drv/7.1/hw_definition.h"
#include "linux/mtkisp7/drv/7.1/common.h"

#include "single_device_helper.h"

namespace libcamera {

struct PortInfoEx {
	void set(const InfoFrame &info, uint32_t idx, int ratio, Size size)
	{
		set(info, idx, ratio, Rectangle(size));
	}

	void set(const InfoFrame &info, uint32_t idx, int ratio, Rectangle crop)
	{
		img.info = info;

		portIdx = idx;
		mResizeRatio = ratio;
		CropX = crop.x;
		CropY = crop.y;
		CropW = crop.width;
		CropH = crop.height;
		CropFloatX = 0;
		CropFloatY = 0;
		CropFloatW = 0;
		CropFloatH = 0;
	}

	NSCam::NSImgStream::IImageBuffer img;

	uint32_t portIdx;
	int mResizeRatio;
	int CropX;       //! X integer start position for cropping
	int CropY;       //! Y integer start position for crpping
	int CropW;       //! width integer of cropped image
	int CropH;       //! height integer of cropped image
	int CropFloatX;  //! X float start position for cropping
	int CropFloatY;  //! Y float start position for cropping
	int CropFloatW;  //! width float of cropped image
	int CropFloatH;  //! height float of cropped image
};

class StageEx {
public:
	StageEx(PEU_Stage stageEnum) { stageEnum_ = stageEnum; }
	~StageEx() = default;

	void input(const InfoFrame &info, uint32_t idx, int ratio, const Rectangle &crop);
	void input(const InfoFrame &info, uint32_t idx, int ratio, const Size &size) {
		input(info, idx, ratio, Rectangle{size});
	}

	void output(const InfoFrame &info, uint32_t idx, int ratio, const Rectangle &crop);
	void output(const InfoFrame &info, uint32_t idx, int ratio, const Size &size) {
		output(info, idx, ratio, Rectangle{size});
	}

	void setWpeInfo(NSCam::NSImgStream::IMG_EXTRA_PARAM_ID id,
			Size crop, NSCam::NSImgStream::WPE_MODE mode,
			unsigned int featureIndex);

	void setMvFrame(Size f0, Size me);
	void setMeInfo(NSCam::NSImgStream::ME_MODE mode);
	void setAplInfo();
	void setMultiScale(NSCam::NSImgStream::IMG_MULTI_SCALE_RATIO ratio,
			   uint32_t index, uint32_t total);
	void setPqInfo();
	void setImg4oCrop(const Rectangle &crop);
	void setCostLevel();

	void addNotify(uint32_t sync);
	void addWait(uint32_t sync);

	PEU_Stage getStageEnum() const { return stageEnum_; }

private:
	friend class SingleDeviceRequest;

	std::vector<PortInfoEx> inputs_;
	std::vector<PortInfoEx> outputs_;
	std::vector<NSCam::NSImgStream::ImgExtraParam> extra_;

	std::list<MUINT32> mSyncTokenNotifyList;
	std::list<MUINT32> mSyncTokenWaitList;

	PEU_Stage stageEnum_;
};

class SingleDeviceRequest {
public:
	StageEx &emplaceStage(PEU_Stage stageEnum) { return stages_.emplace_back(stageEnum); }
	std::vector<StageEx> &Stages() { return stages_; }

	void init(uint32_t sequence, uint32_t timestamp, const std::string &id) {
		sequence_ = sequence;
		timestamp_ = timestamp;
		id_ = id;
	}

	void setSequence(uint32_t sequence) { sequence_ = sequence; }
	uint32_t sequence() { return sequence_; }

	void setTimestamp(uint32_t timestamp) { timestamp_ = timestamp; }
	uint32_t timestamp() { return timestamp_; }

	void setUserId(const std::string &id) { id_ = id; }
	const std::string &id() { return id_; }

	void fillRequestBuffer(InfoFrame &infoCtrl, InfoFrame &infoDesc, int requestFd);

	std::vector<PEU_Stage> getStageEnums() const;

private:
	void fillFrameParams(std::vector<NSCam::NSImgStream::FrameParams> &mvFrameParams);

	uint32_t sequence_;
	uint32_t timestamp_;
	std::string id_;
	std::vector<StageEx> stages_;
};

} // namespace libcamera
