# SPDX-License-Identifier: CC0-1.0

mtkisp7_platform_sources = files([
  'single_device_helper.cpp',
  'Tuning_Helper.cpp'
])

platform_mtkisp7_libs = shared_library('libcamera-mtkisp7-platform',
                                       mtkisp7_platform_sources,
                                       name_prefix : '',
                                       include_directories : libcamera_includes,
                                       install : true)
