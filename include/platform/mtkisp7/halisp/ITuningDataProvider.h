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

#ifndef AAA_ISPHAL_SRC_INCLUDE_ISPHALIMP_V2_ITUNINGDATAPROVIDER_H_
#define AAA_ISPHAL_SRC_INCLUDE_ISPHALIMP_V2_ITUNINGDATAPROVIDER_H_

// #include <ispblocks/IspBlockControls.h>       // IspBlockControl
// #include <mtkcam-core/utils/mapping_mgr/idx_cache.h>  // NSIspTuning::IdxCache
#include <tuning_mapping/cam_idx_struct_ext_pub.h>
#include "mtkcam-chrom/custom/mt8188/hal/inc/isp_tuning/ver1/isp_tuning_cam_info_pub.h"

#include <memory>       // std::shared_ptr
#include <type_traits>  // std::is_same

namespace mtk {
namespace isphal {
namespace v1 {

typedef struct isp_lpnrthres_Param
{
    uint32_t LPNR_ISO_LOW_TH: 16;
    uint32_t LPNR_ISO_HIGH_TH: 16;
    uint32_t LPNR_RSV0: 16;
    uint32_t LPNR_RSV1: 16;
} isp_lpnrthres_Param;

/**
 * ITuningDataProvider is an interface class but basically TuningDataProvider is
 * a version based implementations, this base class is for represent the
 * instance of version based instance. Caller should invoke
 * TuningDataProviderTypeHelper<Version>::Type to cast it when
 * TuningDataProviderTypeHelper<Version>::Valid is true.
 */
class ITuningDataProvider {
 public:
  /**
   * @V0: TuningDataProvider version 1.0
   * @V1: TuningDataProvider version 1.1
   * ...
   */
  enum Version : int {
    V0 = 0,
    V1,
    V2,
    V3,
  };

  /**
  * Feature control enum for TuningProvider
  */
  enum kTuningProviderCmd_T {
    kTuningProviderCmd_getNvram_Feature_interpolation,
    kTuningProviderCmd_CalculateMsf_with_luma_mean,
    kTuningProviderCmd_MaxNum,
  };

// ISP group based ID, describes all SubGroupId. If derived ISP block is not
// built in all pipeline, do not inherit this base class.
struct IspGroupTypeBase {
enum SubGroupId {
     SubGroupId_Undefined = 0,
     SubGroupId_R1,
     SubGroupId_R2,
     SubGroupId_R3,
     SubGroupId_R4,
     SubGroupId_R5,
     SubGroupId_R6,
     SubGroupId_R7,
     SubGroupId_R8,
     SubGroupId_T1,
     SubGroupId_T2,
     SubGroupId_T3,
     SubGroupId_T4,
     SubGroupId_T5,
     SubGroupId_T6,
     SubGroupId_T7,
     SubGroupId_T8,
     SubGroupId_T9,
     SubGroupId_D1,
     SubGroupId_D2,
     SubGroupId_P1A,
     SubGroupId_P1B,
     SubGroupId_E1A,
     SubGroupId_E1B,
     SubGroupId_E1C,
     SubGroupId_Mraw,
     SubGroupId_Mfb,
     SubGroupId_Mss,
   };

constexpr inline static size_t subGroupId2Idx(SubGroupId gid) {
     if (gid == SubGroupId_Undefined)
       return SIZE_MAX;
     return gid - 1;
}
};

static std::shared_ptr<mtk::isphal::v1::ITuningDataProvider> createInstance(
    size_t sensor_index,
    size_t sensor_dev_id,
    uint64_t user_id = 0);

 public:
  /**
   * Query version info
   */
  virtual Version getVersion() const = 0;

  virtual bool readDataForFeature(void* p_out,
    int data_size,
    NSIspTuning::EModuleDB_T atms_module,
    const NSIspTuning::CAM_IDX_QRY_COMB_WITH_SYSTEM_INFO& qry_with_sys_info)
    = 0;

  virtual bool readDataForFeature(void* p_out,
    int data_size,
    NSIspTuning::EModuleDB_T atms_module,
    const NSIspTuning::CAM_IDX_QRY_COMB_ISP7& qry)
    = 0;

  virtual void getLatestMappingInfo(
    NSIspTuning::CAM_IDX_QRY_COMB_ISP7& output)
    = 0;

  virtual void getLatestMappingInfo(
    NSIspTuning::CAM_IDX_QRY_COMB_WITH_SYSTEM_INFO& output)
    = 0;

 public:
  virtual ~ITuningDataProvider() = default;

 protected:
  ITuningDataProvider() = default;
};

/**
 * Type helper, undefined type will be 'void'.
 *  @tparam Version of ITuningDataProvider.
 */
template <int V>
struct TuningDataProviderTypeHelper {
  typedef void Type;
  enum { Valid = false };
};

}      // namespace v1
}      // namespace isphal
}      // namespace mtk

namespace mtk {
namespace isphal {
namespace v1_0 {

/**
 * Tuning Data Provider is a class to provide the processed data based on the
 * given ISP Group Type "IspGroupType".
 */
class TuningDataProvider : public v1::ITuningDataProvider {
 public:
  static std::shared_ptr<mtk::isphal::v1::ITuningDataProvider> createInstance(
      size_t sensor_index,
      size_t sensor_dev_id,
      uint64_t user_id = 0);
  TuningDataProvider(size_t sensor_index,
                     size_t sensor_dev_id,
                     uint64_t user_id);

  virtual ~TuningDataProvider();

 public:  // Version
  enum : int { Ver = v1::ITuningDataProvider::V0 };

 public:  // re-implementations of ITuningDataProvider
  v1::ITuningDataProvider::Version getVersion() const override {
    return v1::ITuningDataProvider::V0;
  }

 public:
  bool readDataForFeature(
      void* p_out,
      int data_size,
      EModuleDB_T atms_module,
      const NSIspTuning::CAM_IDX_QRY_COMB_WITH_SYSTEM_INFO& qry_with_sys_info)
  override;

  bool readDataForFeature(void* p_out,
                          int data_size,
                          EModuleDB_T atms_module,
                          const NSIspTuning::CAM_IDX_QRY_COMB_ISP7& qry)
  override;

  void getLatestMappingInfo(CAM_IDX_QRY_COMB_ISP7& output) override;
  void getLatestMappingInfo(CAM_IDX_QRY_COMB_WITH_SYSTEM_INFO& output)
  override;

 private:
  size_t m_sensorid;  // current sensor id (not sensor index)
  size_t m_sensor_idx;
  int32_t m_debugEnable;
  int32_t m_i4DbCheckLogEn;
  ALL_ISP_INTERVAL m_all_interval;
  mutable std::mutex m_Lock;
  std::string path;

 private:
};
}  // namespace v1_0
}  // namespace isphal
}  // namespace mtk

#endif  // AAA_ISPHAL_SRC_INCLUDE_ISPHALIMP_V2_ITUNINGDATAPROVIDER_H_
