# SPDX-License-Identifier: CC0-1.0

libcamalgo_fdft = cc.find_library('camalgo.fdft', required : false)
libcam_hal3a = cc.find_library('libcam.hal3a', required : true)
libcam_hal3a_v3_nvram_v4l2 = cc.find_library('libcam.hal3a.v3.nvram.v4l2', required : true)
libmtkcam_metadata_container = cc.find_library('libmtkcam_metadata.container', required : true, static : true)
libmtkcam_ulog = cc.find_library('libmtkcam_ulog', required : true)
libmtk_platform_log = cc.find_library('libmtk_platform_log', required : true)
libmtkcam_debugwrapper = cc.find_library('libmtkcam.debugwrapper', required : true)
libcam_halsensor = cc.find_library('libcam.halsensor', required : true)
libcam_halisp_utils_pub = cc.find_library('libcam.halisp.utils.pub', required : true)

libcam_halisp_imp_v4l2 = cc.find_library('cam.halisp.imp.v4l2', required : true)
libcam_halisp_tuning_provider = cc.find_library('cam.halisp.TuningDataProvider', required : true)

libmtkcam_tuning_utils = cc.find_library('libmtkcam_tuning_utils', required : true)
libmtkcam_stdutils = cc.find_library('libmtkcam_stdutils', required : true)
libfeatureiodrv_mem  = cc.find_library('libfeatureiodrv_mem', required : true)

libcamera_deps += [
    libcamalgo_fdft,
    libcam_hal3a,
    libcam_hal3a_v3_nvram_v4l2,
    libmtkcam_metadata_container,
    libmtkcam_ulog,
    libmtk_platform_log,
    libmtkcam_debugwrapper,
    libcam_halsensor,
    libcam_halisp_utils_pub,
    libcam_halisp_imp_v4l2,
    libcam_halisp_tuning_provider,
    libmtkcam_tuning_utils,
    libmtkcam_stdutils,
    libfeatureiodrv_mem,
]

libcamera_sources += files([
    'camsys/camsys.cpp',
    'camsys/capture.cpp',
    'face_detect/aie.cpp',
    'face_detect/detector.cpp',
    'face_detect/parser.cpp',
    'hal3a/aaa.cpp',
    'hal3a/hal_3a.cpp',
    'halisp/hal_isp.cpp',
    'halisp/lpnr_tun.cpp',
    'halisp/mcnr_tun.cpp',
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
    'sensor/sensor_info.cpp'
])

platform_libs += platform_mtkisp7_libs
