/******************************************************************************
* File Name        : mtb_ml_model.h
*
* Description      : This is the header file of ModusToolbox ML middleware NN
*                    model module.
*
* Related Document : See README.md
*
********************************************************************************
 * (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
 * Technologies AG. All rights reserved.
 * This software, associated documentation and materials ("Software") is
 * owned by Infineon Technologies AG or one of its affiliates ("Infineon")
 * and is protected by and subject to worldwide patent protection, worldwide
 * copyright laws, and international treaty provisions. Therefore, you may use
 * this Software only as provided in the license agreement accompanying the
 * software package from which you obtained this Software. If no license
 * agreement applies, then any use, reproduction, modification, translation, or
 * compilation of this Software is prohibited without the express written
 * permission of Infineon.
 *
 * Disclaimer: UNLESS OTHERWISE EXPRESSLY AGREED WITH INFINEON, THIS SOFTWARE
 * IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING, BUT NOT LIMITED TO, ALL WARRANTIES OF NON-INFRINGEMENT OF
 * THIRD-PARTY RIGHTS AND IMPLIED WARRANTIES SUCH AS WARRANTIES OF FITNESS FOR A
 * SPECIFIC USE/PURPOSE OR MERCHANTABILITY.
 * Infineon reserves the right to make changes to the Software without notice.
 * You are responsible for properly designing, programming, and testing the
 * functionality and safety of your intended application of the Software, as
 * well as complying with any legal requirements related to its use. Infineon
 * does not guarantee that the Software will be free from intrusion, data theft
 * or loss, or other breaches ("Security Breaches"), and Infineon shall have
 * no liability arising out of any Security Breaches. Unless otherwise
 * explicitly approved by Infineon, the Software may not be used in any
 * application where a failure of the Product or any consequences of the use
 * thereof can reasonably be expected to result in personal injury.
*******************************************************************************/

#if !defined(__MTB_ML_MODEL_H__)
#define __MTB_ML_MODEL_H__

#if defined(__cplusplus)
extern "C" {
#endif

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "mtb_ml_common.h"
#include "mtb_ml_model_defs.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define MEM_FLAG_SHIFT_PERSISTENT       (0)
#define MEM_FLAG_SHIFT_SCRATCH          (1)

#define MTB_ML_MEM_DYNAMIC_PERSISTENT   (1 << MEM_FLAG_SHIFT_PERSISTENT)
#define MTB_ML_MEM_DYNAMIC_SCRATCH      (1 << MEM_FLAG_SHIFT_SCRATCH)

#define MTB_ML_MODEL_NAME_LEN           (64)

/*******************************************************************************
* Data Structures and Types
*******************************************************************************/
/**
 * ML model working buffer structure
 */
typedef struct
{
/** @name COMPONENT_ML_TFLM
 *  Buffer pareamters for TFLM.
 */
///@{
/** Model buffer parameters for TfLite-Micro inference engine */
    uint8_t* tensor_arena;              /**< the pointer of tensor arena buffer provided by application */
    size_t tensor_arena_size;           /**< the size of ML tensor arena buffer provided by application */
///@}
} mtb_ml_model_buffer_t;

/**
 * ML model structure
 */
typedef struct
{
/** @name
 *  Model name
 */
/**@{*/
    char name[MTB_ML_MODEL_NAME_LEN];   /**< the name of ML model */
/**@}*/
#if defined(COMPONENT_ML_TFLM)
/** @name COMPONENT_ML_TFLM
 *  Model pareamters for TFLM with interpreter
 */
///@{
    const uint8_t *      model_bin;     /**< the pointer of Tflite model */
    const unsigned int   model_size;    /**< the size of Tflite model */
    const int            arena_size;    /**< the size of arena buffer for Tflite model */
///@}
#endif
#if defined(COMPONENT_ML_TFLM_LESS)
/** @name COMPONENT_ML_TFLM_LESS
 *  Model pareamters for TFLM without interpreter
 */
///@{
    tflm_rmf_apis_t      rmf_bin;     /**< the data structure of Tflite APIs */
///@}
#endif
} mtb_ml_model_bin_t;

/**
 * ML model runtime object structure
 */
typedef struct
{
/** @name
 *  Model runtime object common fields
 */
/**@{*/
    char name[MTB_ML_MODEL_NAME_LEN];   /**< the name of ML model */
    int model_size;                     /**< the size of ML model */
    int buffer_size;                    /**<the size of ML model working buffer */
    int input_size;                     /**< array size of input data */
    int output_size;                    /**< array size of output data */
    int lib_error;                      /**< error code from ML inference library */
    MTB_ML_DATA_T *output;              /**< pointer of ML inference output buffer */
    MTB_ML_DATA_T *input;               /**< pointer of ML inference input buffer */
    void *tflm_obj;                     /**< pointer of Tflite-micro runtime object */
    int model_time_steps;               /*< number of model time steps */
    int recurrent_ts_size;              /**< number of data time steps in NN. 0 if non streaming RNN */
    int input_zero_point;               /**< zero point of input data */
    float input_scale;                  /**< scale of input data*/
    int output_zero_point;              /**< zero point of output data */
    float output_scale;                 /**< scale of output data */
    mtb_ml_profile_config_t profiling;  /**< flags of profiling */
    uint32_t m_cpu_cycles;              /**< CPU profiling cycles */
    uint32_t m_sum_frames;              /**< profiling frames */
    uint64_t m_cpu_sum_cycles;          /**< CPU profiling total cycles */
    uint32_t m_cpu_peak_frame;          /**< CPU profiling peak frame */
    uint32_t m_cpu_peak_cycles;         /**< CPU profiling peak cycles */
    bool is_rnn_streaming;              /**< Is the model an RNN streaming model */
/**@}*/
#if defined(COMPONENT_U55)
/** @name COMPONENT_U55
 *  Model runtime object fields for NPU cycle count
 */
/**@{*/
    uint64_t m_npu_cycles;          /**< NPU profiling cycles */
    uint64_t m_npu_sum_cycles;      /**< NPU total cycles */
    uint32_t m_npu_peak_frame;      /**< NPU profiling peak frame */
    uint32_t m_npu_peak_cycles;     /**< NPU profiling peak cycles */
/**@}*/
#endif
#if defined(COMPONENT_ML_TFLM)
/** @name COMPONENT_ML_TFLM
 *  Model runtime object fields for TFLM with interpreter
 */
/**@{*/
    uint8_t *arena_buffer;              /**< pointer of allocated tensor arena buffer */
#endif
#if defined(COMPONENT_ML_TFLM_LESS)
/** @name COMPONENT_ML_TFLM_LESS
 *  Model runtime object fields for TFLM without interpreter
 */
/**@{*/
    tflm_rmf_apis_t rmf_apis;           /**< data structure of Tflite-micro APIs */
/**@}*/
#endif
} mtb_ml_model_t;

/******************************************************************************
* Function prototype
******************************************************************************/
cy_rslt_t mtb_ml_model_init(const mtb_ml_model_bin_t *bin, const mtb_ml_model_buffer_t *buffer, mtb_ml_model_t **object);
cy_rslt_t mtb_ml_model_deinit(mtb_ml_model_t *object);
cy_rslt_t mtb_ml_model_run(mtb_ml_model_t *object, MTB_ML_DATA_T *input);
int mtb_ml_model_get_input_size(const mtb_ml_model_t *object);
cy_rslt_t mtb_ml_model_get_input_detail(const mtb_ml_model_t *object, int index,
                                            MTB_ML_DATA_T **in_pptr, size_t* size_ptr,
                                            int** dim_ptr, int* dim_len_ptr,
                                            int* zero_ptr, float* scale_ptr);
cy_rslt_t mtb_ml_model_get_output(const mtb_ml_model_t *object, MTB_ML_DATA_T **out_pptr, int* size_ptr);
cy_rslt_t mtb_ml_model_get_output_detail(const mtb_ml_model_t *object, int index,
                                            MTB_ML_DATA_T **out_pptr, size_t* size_ptr,
                                            int** dim_ptr, int* dim_len_ptr,
                                            int* zero_ptr, float* scale_ptr);
cy_rslt_t mtb_ml_model_rnn_reset_all_parameters(mtb_ml_model_t *object);
void* mtb_ml_model_get_inference_object(const mtb_ml_model_t *object);
int mtb_ml_model_profile_get_tsc(uint32_t *val);
cy_rslt_t mtb_ml_model_profile_config(mtb_ml_model_t *object, mtb_ml_profile_config_t config);
cy_rslt_t mtb_ml_model_profile_log(mtb_ml_model_t *object);

#if defined(__cplusplus)
}
#endif

#endif /* __MTB_ML_MODEL_H__ */

/* [] END OF FILE */