/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2020, Google Inc.
 *
 * thumbnailer.cpp - Simple image thumbnailer
 */

#include "thumbnailer.h"

#include <libcamera/base/log.h>

#include <libcamera/formats.h>

#include "libcamera/internal/mapped_framebuffer.h"

#include <libyuv/scale.h>

using namespace libcamera;

LOG_DEFINE_CATEGORY(Thumbnailer)

Thumbnailer::Thumbnailer()
	: valid_(false)
{
}

void Thumbnailer::configure(const libcamera::StreamConfiguration &cfg, PixelFormat pixelFormat)
{
	sourceSize_ = cfg.size;
	pixelFormat_ = pixelFormat;
	stride_ = cfg.stride;

	if (pixelFormat_ != formats::NV12) {
		LOG(Thumbnailer, Error)
			<< "Failed to configure: Pixel Format "
			<< pixelFormat_ << " unsupported.";
		return;
	}

	valid_ = true;
}

void Thumbnailer::createThumbnail(const FrameBuffer &source,
				  const Size &targetSize,
				  std::vector<unsigned char> *destination)
{
	MappedFrameBuffer frame(&source, MappedFrameBuffer::MapFlag::Read);
	if (!frame.isValid()) {
		LOG(Thumbnailer, Error)
			<< "Failed to map FrameBuffer : "
			<< strerror(frame.error());
		return;
	}

	if (!valid_) {
		LOG(Thumbnailer, Error) << "Config is unconfigured or invalid.";
		return;
	}

	const unsigned int stride = stride_;
	const unsigned int sw = sourceSize_.width;
	const unsigned int sh = sourceSize_.height;
	const unsigned int tw = targetSize.width;
	const unsigned int th = targetSize.height;

	ASSERT(frame.planes().size() == 2);
	ASSERT(tw % 2 == 0 && th % 2 == 0);

	size_t dstSize = (th * tw) + ((th / 2) * tw);
	destination->resize(dstSize);
	unsigned char *dst = destination->data();

	int ret = libyuv::NV12Scale(frame.planes()[0].data(), stride,
				    frame.planes()[1].data(), stride,
				    sw, sh,
				    dst, tw,
				    dst + th * tw, tw,
				    tw, th,
				    libyuv::FilterMode::kFilterBox);
	if (ret) {
		LOG(Thumbnailer, Error)
			<< "Failed to scale thumbnail with libyuv: " << ret;
	}
}
