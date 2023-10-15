/* SPDX-License-Identifier: LGPL-2.1-or-later */
/*
 * Copyright (C) 2023, Google Inc.
 *
 * imagiq_adapter.h - MtkISP7 OnDeviceTuner Imagiq Adapter
 */

#include "pipeline/mtkisp7/odt/imagiq_adapter/imagiq_adapter.h"

#include <filesystem>
#include <fstream>

#include <libcamera/base/log.h>

#include "libcamera/internal/formats.h"
#include "libcamera/internal/mapped_framebuffer.h"

namespace libcamera {

LOG_DECLARE_CATEGORY(MtkISP7)

int ImagiqAdapter::exportDump(const Dump &dump)
{
    // todo next CL: being actual adapter -> file name format
    std::filesystem::path exportPath =
            dump.workPath / (dump.sensorId.substr(dump.sensorId.size() - 9) + ".packed_word");
    std::ofstream exportFile(exportPath, std::ios::binary);
    if (!exportFile.good()) {
        LOG(MtkISP7, Error) << "Failed to open dump dump file: "
                            << exportPath;
        return -EIO;
    }
    MappedFrameBuffer mappedBuffer(
            dump.frame.buffer(), MappedFrameBuffer::MapFlag::Read);
    const PixelFormatInfo formatInfo =
            PixelFormatInfo::info(dump.frame.format());
    for (size_t i = 0; i < formatInfo.numPlanes(); i++) {
        exportFile.write(
                reinterpret_cast<char*>(mappedBuffer.planes()[i].data()),
                mappedBuffer.planes()[i].size());
        LOG(MtkISP7, Info) << "Dump " << dump.sensorId
                << " plane: " << i << " plane size: " << mappedBuffer.planes()[i].size()
                << " write file size: " << exportFile.tellp();
        if (!exportFile.good()) {
            LOG(MtkISP7, Error) << "Error writing plane " << i << " to file: "
                                << exportPath;
            return -EIO;
        }
    }
    return 0;
}

} // namespace libcamera
