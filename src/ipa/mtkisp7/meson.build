# SPDX-License-Identifier: CC0-1.0

ipa_name = 'ipa_mtkisp7'

mtkisp7_ipa_sources = files([
  # 'mtkisp7.cpp',
])

mod = shared_module(ipa_name,
                    [mtkisp7_ipa_sources, libcamera_generated_ipa_headers],
                    name_prefix : '',
                    include_directories : [ipa_includes, libipa_includes],
                    dependencies : libcamera_private,
                    link_with : libipa,
                    install : true,
                    install_dir : ipa_install_dir)

if ipa_sign_module
    custom_target(ipa_name + '.so.sign',
                  input : mod,
                  output : ipa_name + '.so.sign',
                  command : [ipa_sign, ipa_priv_key, '@INPUT@', '@OUTPUT@'],
                  install : false,
                  build_by_default : true)
endif

ipa_names += ipa_name
