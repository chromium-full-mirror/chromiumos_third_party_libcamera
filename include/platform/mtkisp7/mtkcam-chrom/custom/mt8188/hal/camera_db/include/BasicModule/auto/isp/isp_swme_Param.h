#ifndef ISP_SWME_PARAM_H
#define ISP_SWME_PARAM_H

#include <stdint.h>
#if 5 == 2 || 5 == 4 // kStructType = Binary || kStructType = Dynamic
#ifdef __cplusplus
#include "isp_swme_ParamInclude.h"
#endif // #ifdef __cplusplus
#else // kStructType
#include "isp_swme_ParamInclude.h"
#endif // kStructType
typedef enum isp_swme_ParamIndex
{
#define APPLY_OPERATION(name, ...) isp_swme_kParam_##name,
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/camera_db/include/BasicModule/list/isp/isp_swme_list_Param.h"
    isp_swme_kNumOfParamIndex
} isp_swme_ParamIndex;

static const char* isp_swme_kParamNames[] =
{
#define APPLY_OPERATION(name, ...) #name,
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/camera_db/include/BasicModule/list/isp/isp_swme_list_Param.h"
};

static inline const char* isp_swme_getParamName(int index)
{
    return isp_swme_kParamNames[index];
}

#if 5 == 1 // kStructType = Array
typedef struct isp_swme_Param
{
#define APPLY_OPERATION(name, type, number, ...) type name[number];
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/camera_db/include/BasicModule/list/isp/isp_swme_list_Param.h"
} isp_swme_Param;

#elif 5 == 5 // kStructType = BitFeild
typedef struct isp_swme_Param
{
#define APPLY_OPERATION(name, type, number, bit, ...) type name : bit;
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/camera_db/include/BasicModule/list/isp/isp_swme_list_Param.h"
} isp_swme_Param;

#elif 5 == 2 || 5 == 4 // kStructType = Binary || kStructType = Dynamic
typedef struct isp_swme_Param
{
#ifdef __cplusplus
#define APPLY_OPERATION(name, type, ...) type name;
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/camera_db/include/BasicModule/list/isp/isp_swme_list_Param.h"
#else // #ifdef __cplusplus
    int dummy;
#endif // #ifdef __cplusplus
} isp_swme_Param;

#else // kStructType
typedef struct isp_swme_Param
{
#define APPLY_OPERATION(name, type, ...) type name;
#include "platform/mtkisp7/mtkcam-chrom/custom/mt8188/hal/camera_db/include/BasicModule/list/isp/isp_swme_list_Param.h"
} isp_swme_Param;

#endif // kStructType

#endif // ISP_SWME_PARAM_H
