# SPDX-License-Identifier: CC0-1.0

libcamalgo_fdft = cc.find_library('camalgo.fdft', required : false)

libcamera_deps += [
    libcamalgo_fdft,
]

libcamera_sources += files([
    'camsys/camsys.cpp',
    'camsys/capture.cpp',
    'face_detect/aie.cpp',
    'face_detect/detector.cpp',
    'face_detect/parser.cpp',
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
