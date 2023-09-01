# SPDX-License-Identifier: CC0-1.0

libcamalgo_fdft = cc.find_library('camalgo.fdft', required : false)
libcam_hal3a = cc.find_library('libcam.hal3a', required : true)
libcam_hal3a_v3_nvram_v4l2 = cc.find_library('libcam.hal3a.v3.nvram.v4l2', required : true)
libmtkcam_metadata_container = cc.find_library('libmtkcam_metadata.container', required : true, static : true)
libmtkcam_ulog = cc.find_library('libmtkcam_ulog', required : true)
libmtk_platform_log = cc.find_library('libmtk_platform_log', required : true)
libmtkcam_debugwrapper = cc.find_library('libmtkcam.debugwrapper', required : true)
libcam_halsensor = cc.find_library('libcam.halsensor', required : true)
libcam_peripheralcontroller = cc.find_library('libcam.peripheralcontroller', required : true)

libcamera_deps += [
    libcamalgo_fdft,
    libcam_hal3a,
    libcam_hal3a_v3_nvram_v4l2,
    libmtkcam_metadata_container,
    libmtkcam_ulog,
    libmtk_platform_log,
    libmtkcam_debugwrapper,
    libcam_halsensor,
    libcam_peripheralcontroller,
]

libcamera_sources += files([
    'camsys/camsys.cpp',
    'camsys/capture.cpp',
    'face_detect/aie.cpp',
    'face_detect/detector.cpp',
    'face_detect/parser.cpp',
    'hal3a/hal_3a.cpp',
    'imgsys/lpnr.cpp',
    'imgsys/imgsys.cpp',
    'imgsys/mcnr.cpp',
    'imgsys/single_device.cpp',
    'odt/imagiq_adapter/static_metadata/static_strings.cpp',
    'odt/imagiq_adapter/dump.cpp',
    'odt/imagiq_adapter/imagiq_adapter.cpp',
    'odt/camsys_driver_debug.cpp',
    'odt/on_device_tuner.cpp',
    'mtkisp7.cpp',
])

platform_libs += platform_mtkisp7_libs
