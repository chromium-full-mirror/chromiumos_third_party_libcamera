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

#ifndef INCLUDE_MTKCAM_INTERFACES_UTILS_NDD_INDD_H_
#define INCLUDE_MTKCAM_INTERFACES_UTILS_NDD_INDD_H_

// #include <mtkcam-interfaces/utils/imgbuf/IImageBuffer.h>
// #include <mtkcam-interfaces/utils/metadata/IMetadata.h>
// #include <mtkcam-interfaces/utils/ndd/ndd_autogen_def.h>
// #include <mtkcam-interfaces/utils/ndd/ndd_def.h>

#include <cstdint>
#include <string>
#include <vector>

using std::string;
using std::vector;

// Usage
// for simple array  : NDD_DUMP(eCategory, eModule, NddData, void* buffer, size)
// for IImageBuffer : NDD_DUMP(eCategory, eModule, NddData, IImageBuffer*)

#define NDD_DUMP(...)                                                        \
  do {                                                                       \
    NSCam::TuningUtils::INdd* ins = NSCam::TuningUtils::INdd::getInstance(); \
    ins->submit_request(__VA_ARGS__);                                        \
  } while (0);

#define FILE_PATH_SIZE 512

namespace NSCam {
namespace TuningUtils {

/* Design for being used in service.cpp only
** since Ndd dumping threads' lifecycle are aligned with camerahalserver
**/

class NddInitializerImp;

class NddInitializer {
 public:
  NddInitializer();
  ~NddInitializer();
};

class INdd {
 public:
  static INdd* getInstance();
  INdd(const INdd&) = delete;
  INdd& operator=(const INdd&) = delete;

  /**
   * @brief Notify NDD that the camera is on
   *
   * @param[in] sensor_id The set of camera that is active
   * @param[in] uniqueKey The uniqueKey of the pipeline, a.k.a. 9-digit Code
   */
  virtual void stream_on(vector<int32_t> sensor_id, uint32_t uniqueKey) = 0;

  /**
   * @brief Notify NDD that the camera is off
   *
   * @param[in] sensor_id The set of camera that is being shut down
   */
  virtual void stream_off(vector<int32_t> sensor_id) = 0;

  /**
   * @brief Notify NDD the start of the certain frame
   *
   * @param[in] sensor_idx The sensor id of the informed frame
   * @param[in] frm_no The frame number of the informed frme
   */
  virtual void frame_begin(int32_t sensor_idx, int32_t frm_no) = 0;

  /**
   * @brief Notify NDD the end of the certain frame
   *
   * @param[in] sensor_idx The sensor id of the informed frame
   * @param[in] frm_no The frame number of the informed frme
   */
  virtual void frame_end(int32_t sensor_idx, int32_t frm_no) = 0;

 protected:
  INdd() {}
  virtual ~INdd() {}
};
}  // namespace TuningUtils
}  // namespace NSCam

#endif  // INCLUDE_MTKCAM_INTERFACES_UTILS_NDD_INDD_H_
