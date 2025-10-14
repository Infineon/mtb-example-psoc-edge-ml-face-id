/******************************************************************************
* File Name        : mtb_ml_model_tflm.cpp
*
* Description      : This file contains API calls to initialize and invoke 
*                    Tflite-Micro inference for model generated from CY ML 
*                    software.
*
* Related Document : See README.md
*
********************************************************************************
* (c) 2023-2025, Infineon Technologies AG, or an affiliate of Infineon
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <stdlib.h>
#include <inttypes.h>
#include "mtb_ml_model.h"
#include "mtb_ml.h"

#include <climits>

#include "tensorflow/lite/micro/tflite_bridge/micro_error_reporter.h"
#include "tensorflow/lite/micro/micro_op_resolver.h"
#include "tensorflow/lite/micro/recording_micro_interpreter.h"
#include "tensorflow/lite/micro/all_ops_resolver.h"
#include "tensorflow/lite/micro/micro_utils.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#ifdef COMPONENT_ML_INT8x8
#define MTB_TFLM_Class MTB_TFLM_int8
#else
#define MTB_TFLM_Class MTB_TFLM_flt
#endif

namespace tflite {

static tflite::AllOpsResolver resolver;

template <typename inputT>
class MTBTFLiteMicro {
 public:
  /* The lifetimes of model, op_resolver, tensor_arena must exceed
   that of the created MicroBenchmarkRunner object. */
  MTBTFLiteMicro(const uint8_t* model,
                       uint8_t* tensor_arena, int tensor_arena_size,
                       const tflite::MicroOpResolver& op_resolver)
      : interpreter_(GetModel(model), op_resolver, tensor_arena,
                     tensor_arena_size, nullptr, nullptr) {
      allocate_status_ = interpreter_.AllocateTensors();
      model_ = GetModel(model);
  }

  TfLiteStatus RunSingleIteration() {
    /* Run the model on this input and return the status. */
    return interpreter_.Invoke();
  }

  TfLiteTensor* Input(int index = 0)  { return interpreter_.input(index); }
  TfLiteTensor* Output(int index = 0) { return interpreter_.output(index); }

  TfLiteStatus AllocationStatus() { return allocate_status_; }

  /* Use for RNN state control. This will free subgraphs to the reset state */
  TfLiteStatus reset_all_variables() { return interpreter_.Reset(); }

  inputT* input_ptr(int index = 0) { return GetTensorData<inputT>(Input(index)); }
  size_t input_size(int index = 0) { return interpreter_.input(index)->bytes; }
  size_t input_elements(int index = 0) { return tflite::ElementCount(*(interpreter_.input(index)->dims)); }
  int    input_dims_len(int index=0) {return interpreter_.input(index)->dims->size; }
  int *  input_dims( int index=0) { return &interpreter_.input(index)->dims->data[0]; }
  int    input_zero_point( int index=0) { return interpreter_.input(index)->params.zero_point; }
  float  input_scale( int index=0) { return interpreter_.input(index)->params.scale; }
  int    output_zero_point( int index=0) { return interpreter_.output(index)->params.zero_point; }
  float  output_scale( int index=0) { return interpreter_.output(index)->params.scale; }

  inputT* output_ptr(int index = 0) { return GetTensorData<inputT>(Output(index)); }
  size_t output_size(int index = 0) { return interpreter_.output(index)->bytes; }
  size_t output_elements(int index = 0) { return  tflite::ElementCount(*(interpreter_.output(index)->dims));}
  int    output_dims_len(int index=0) {return interpreter_.output(index)->dims->size; }
  int *  output_dims( int index=0) { return &interpreter_.output(index)->dims->data[0]; }
  size_t get_used_arena_size() { return interpreter_.arena_used_bytes(); }
  int get_model_time_steps(int index=0) { return interpreter_.input(0)->dims->data[1]; }

 void SetInput(const inputT* custom_input, int recurrent_ts_size, int input_index = 0) {
    TfLiteTensor* input = interpreter_.input(input_index);
    inputT* input_buffer = tflite::GetTensorData<inputT>(input);
    int input_length = input->bytes / sizeof(inputT);
    for (int i = 0; i < input_length; i++) {
        input_buffer[i] = custom_input[i];
    }
  }

  void PrintAllocations() const {
    interpreter_.GetMicroAllocator().PrintAllocations();
  }

 private:
  tflite::RecordingMicroInterpreter interpreter_;
  TfLiteStatus allocate_status_;
  const Model* model_;

};

using MTB_TFLM_flt = MTBTFLiteMicro<float>;
using MTB_TFLM_int8 = MTBTFLiteMicro<int8_t>;

} /* namespace tflite */

#ifdef __cplusplus
extern "C" {
#endif  /* __cplusplus */

/* LCOV_EXCL_START (Excluded from the code coverage, until the STOP marker) */
int __attribute__((weak)) mtb_ml_model_profile_get_tsc(uint32_t *val)
{
    return 0;
}
/* LCOV_EXCL_STOP */

/*******************************************************************************
* Functions
*******************************************************************************/

/*******************************************************************************
* Function Name: mtb_ml_model_init
********************************************************************************
* Summary:
*  Initializes a machine learning model by allocating and configuring the model
*  object, tensor arena, and TensorFlow Lite Micro class. Sets up input and output
*  parameters based on the provided model binary and buffer data.
*
* Parameters:
*  const mtb_ml_model_bin_t *bin: Pointer to the model binary data structure
*  const mtb_ml_model_buffer_t *buffer: Pointer to the buffer configuration (optional)
*  mtb_ml_model_t **object: Pointer to the model object to be initialized
*
* Return:
*  cy_rslt_t: Result of the initialization (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/
cy_rslt_t mtb_ml_model_init(const mtb_ml_model_bin_t *bin, const mtb_ml_model_buffer_t *buffer, mtb_ml_model_t **object)
{
    mtb_ml_model_t *model_object = NULL;
    uint8_t * arena_buffer = NULL;
    int arena_size;
    tflite::MTB_TFLM_Class * TFLMClass;
    int ret = MTB_ML_RESULT_SUCCESS;

    /* Sanity check of input parameters */
    if (bin == NULL || bin->model_bin == NULL || object == NULL)
    {
        return MTB_ML_RESULT_BAD_ARG;
    }

    /* Allocate runtime object */
    model_object = (mtb_ml_model_t *)calloc(1, sizeof(mtb_ml_model_t));

    if (model_object == NULL)
    {
        return MTB_ML_RESULT_ALLOC_ERR;
    }

    /* Copy the model name */
    memcpy(model_object->name, bin->name, MTB_ML_MODEL_NAME_LEN);

    /* Get the arena size specified in model data file */
    arena_size = bin->arena_size;
    if (buffer != NULL)
    {
        if (buffer->tensor_arena_size != 0)
        {
            /* over-write with application provided value */
            arena_size = buffer->tensor_arena_size;
        }
        arena_buffer = buffer->tensor_arena;
    }

    /* Get model and buffer size */
    model_object->model_size = bin->model_size;
    model_object->buffer_size = arena_size;

    /* Allocate tensor arena if it is not specified */
    if (arena_buffer == NULL)
    {
        /*
        *  Vela compiler in coretools is set to 16 byte alignment for tensors.
        *  May need to change if configuration is made accesible by user
        */
#if defined (__ARMCC_VERSION)
        int ret_val = posix_memalign((void **)(&(model_object->arena_buffer)), (size_t) 16, arena_size);
        if (ret_val != 0)
        {
            ret = MTB_ML_RESULT_ALLOC_ERR;
            goto ret_err;
        }
#else
        /* Allocation must be a multiple of alignment for aligned_alloc */
        model_object->arena_buffer = (uint8_t *) aligned_alloc(16, arena_size + (16 - (arena_size % 16)));
        if (model_object->arena_buffer == NULL)
        {
            ret = MTB_ML_RESULT_ALLOC_ERR;
            goto ret_err;
        }
#endif
        arena_buffer = model_object->arena_buffer;
    }

    TFLMClass = new tflite::MTB_TFLM_Class(bin->model_bin, arena_buffer, arena_size, tflite::resolver);
    model_object->tflm_obj = reinterpret_cast<void *>(TFLMClass);
    if( model_object->tflm_obj == NULL)
    {
        ret = MTB_ML_RESULT_BAD_MODEL;
        goto ret_err;
    }

    /* Check Tensor allocation failure */
    if (TFLMClass->AllocationStatus() != kTfLiteOk)
    {
        ret = MTB_ML_RESULT_ALLOC_ERR;
        goto ret_err;
    }

    /* Input parameters */
    model_object->input = TFLMClass->input_ptr();
    model_object->input_size = TFLMClass->input_elements();
    model_object->input_zero_point = TFLMClass->input_zero_point();
    model_object->input_scale = TFLMClass->input_scale();
    model_object->model_time_steps = TFLMClass->get_model_time_steps();
    /* Output parameters*/
    model_object->output_size = TFLMClass->output_elements();
    model_object->output = TFLMClass->output_ptr();
    model_object->buffer_size = TFLMClass->get_used_arena_size();
    model_object->output_zero_point = TFLMClass->output_zero_point();
    model_object->output_scale = TFLMClass->output_scale();

    *object = model_object;
    return ret;
ret_err:
    free(model_object->arena_buffer);
    free(model_object);
    return ret;
}


/*******************************************************************************
* Function Name: mtb_ml_model_deinit
********************************************************************************
* Summary:
*  Deinitializes a machine learning model by freeing the TensorFlow Lite Micro
*  class, tensor arena buffer, and model object memory.
*
* Parameters:
*  mtb_ml_model_t *object: Pointer to the model object to be deinitialized
*
* Return:
*  cy_rslt_t: Result of the deinitialization (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/
cy_rslt_t mtb_ml_model_deinit(mtb_ml_model_t *object)
{
    /* Sanity check of input parameters */
    if (object == NULL)
    {
        return MTB_ML_RESULT_BAD_ARG;
    }
    delete reinterpret_cast<tflite::MTB_TFLM_Class *>(object->tflm_obj);
    free(object->arena_buffer);
    free(object);

    return MTB_ML_RESULT_SUCCESS;
}


/*******************************************************************************
* Function Name: mtb_ml_model_run
********************************************************************************
* Summary:
*  Runs a single iteration of the machine learning model with the provided input
*  data, optionally profiling CPU and NPU cycles. Updates model profiling metrics
*  if enabled and logs output if configured.
*
* Parameters:
*  mtb_ml_model_t *object: Pointer to the initialized model object
*  MTB_ML_DATA_T *input: Pointer to the input data (optional)
*
* Return:
*  cy_rslt_t: Result of the model execution (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/
cy_rslt_t mtb_ml_model_run(mtb_ml_model_t *object, MTB_ML_DATA_T *input)
{
    TfLiteStatus ret;
    /* Sanity check of input parameters */
    if (object == NULL) {
        return MTB_ML_RESULT_BAD_ARG;
    }

    tflite::MTB_TFLM_Class *Tflm = reinterpret_cast<tflite::MTB_TFLM_Class *>(object->tflm_obj);

    /* Set input data */
    if ( input )
        Tflm->SetInput(input, object->recurrent_ts_size);

    /* Model profiling */
    if (object->profiling & MTB_ML_PROFILE_ENABLE_MODEL)
    {
        mtb_ml_model_profile_get_tsc(&object->m_cpu_cycles);
    }

    ret = Tflm->RunSingleIteration();
    if ( ret != kTfLiteOk )
    {
        object->lib_error = ret;
        return MTB_ML_RESULT_INFERENCE_ERROR;
    }

    if (object->profiling & MTB_ML_PROFILE_ENABLE_MODEL)
    {
        uint32_t cycles = 0U;
        mtb_ml_model_profile_get_tsc(&cycles);
        object->m_cpu_cycles = cycles - object->m_cpu_cycles;
#if defined(COMPONENT_U55) && !defined(COMPONENT_RTOS)
        uint64_t norm_cpu_cycles = (uint64_t)(((float)object->m_cpu_cycles) * mtb_ml_norm_clk_freq);

        /* Check for bad cpu/npu count values so we don't overflow */
        if (mtb_ml_npu_cycles > norm_cpu_cycles)
        {
            return MTB_ML_RESULT_CYCLE_COUNT_ERROR;
        }
        else
        {
            object->m_cpu_cycles = (uint32_t)(norm_cpu_cycles - mtb_ml_npu_cycles);
        }
#endif
#if defined(COMPONENT_U55)
        object->m_npu_cycles = mtb_ml_npu_cycles;
#endif
        if (object->m_cpu_cycles > object->m_cpu_peak_cycles)
        {
            object->m_cpu_peak_cycles = object->m_cpu_cycles;
            object->m_cpu_peak_frame = object->m_sum_frames;
        }
#if defined(COMPONENT_U55)
        if (object->m_npu_cycles > object->m_npu_peak_cycles)
        {
            object->m_npu_peak_cycles = object->m_npu_cycles;
            object->m_npu_peak_frame = object->m_sum_frames;
        }
#endif
        object->m_sum_frames++;
        object->m_cpu_sum_cycles += object->m_cpu_cycles;
#if defined(COMPONENT_U55)
        object->m_npu_sum_cycles += object->m_npu_cycles;
#endif
    }
    else if (object->profiling & MTB_ML_LOG_ENABLE_MODEL_LOG)
    {
       MTB_ML_DATA_T * output_ptr = object->output;
       /**
       * This string must track ML_PROFILE_OUTPUT_STRING in mtb_ml_stream_impl.h,
       * as the header file is currently unable to be included due to conflicts.
       */
       printf(" output:");
       for (int j = 0; j < object->output_size; j++)
       {
          printf("%6.3f ", (float) (output_ptr[j]));
       }
       printf("\r\n");
    }

    return MTB_ML_RESULT_SUCCESS;
}


/*******************************************************************************
* Function Name: mtb_ml_model_get_output
********************************************************************************
* Summary:
*  Retrieves the output buffer and size from the machine learning model.
*
* Parameters:
*  const mtb_ml_model_t *object: Pointer to the initialized model object
*  MTB_ML_DATA_T **output_pptr: Pointer to store the output buffer pointer
*  int *size_ptr: Pointer to store the output size
*
* Return:
*  cy_rslt_t: Result of the operation (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/
cy_rslt_t mtb_ml_model_get_output(const mtb_ml_model_t *object, MTB_ML_DATA_T **output_pptr, int *size_ptr)
{
    /* Sanity check of input parameters */
    if (object == NULL)
    {
        return MTB_ML_RESULT_BAD_ARG;
    }

    if (output_pptr != NULL)
    {
        *output_pptr = object->output;
    }

    if (size_ptr != NULL)
    {
        *size_ptr = object->output_size;
    }

    return MTB_ML_RESULT_SUCCESS;
}


/*******************************************************************************
* Function Name: mtb_ml_model_get_input_detail
********************************************************************************
* Summary:
*  Retrieves detailed input parameters for a specific input index of the machine
*  learning model, including buffer pointer, size, dimensions, zero point, and scale.
*
* Parameters:
*  const mtb_ml_model_t *object: Pointer to the initialized model object
*  int index: Index of the input tensor
*  MTB_ML_DATA_T **in_pptr: Pointer to store the input buffer pointer
*  size_t *size_ptr: Pointer to store the input size
*  int **dim_ptr: Pointer to store the input dimensions
*  int *dim_len_ptr: Pointer to store the input dimensions length
*  int *zero_ptr: Pointer to store the input zero point
*  float *scale_ptr: Pointer to store the input scale
*
* Return:
*  cy_rslt_t: Result of the operation (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/

cy_rslt_t mtb_ml_model_get_input_detail(const mtb_ml_model_t *object, int index,
                                            MTB_ML_DATA_T **in_pptr, size_t* size_ptr,
                                            int** dim_ptr, int* dim_len_ptr,
                                            int* zero_ptr, float* scale_ptr)
{
    /* Sanity check of input parameters */
    if ( object == NULL ) {
        return MTB_ML_RESULT_BAD_ARG;
    }

    tflite::MTB_TFLM_Class    *Tflm  = reinterpret_cast<tflite::MTB_TFLM_Class *>(object->tflm_obj);
    TfLiteTensor            *input = Tflm->Input(index);

    if (size_ptr != NULL)        *size_ptr    = Tflm->input_elements(index);
    if (in_pptr != NULL)        *in_pptr    = Tflm->input_ptr(index);
    if (zero_ptr != NULL)        *zero_ptr     = input->params.zero_point;
    if (scale_ptr != NULL)        *scale_ptr     = input->params.scale;
    if (dim_len_ptr != NULL)    *dim_len_ptr= Tflm->input_dims_len(index);
    if (dim_ptr != NULL)        *dim_ptr     = Tflm->input_dims(index);

    return MTB_ML_RESULT_SUCCESS;
}


/*******************************************************************************
* Function Name: mtb_ml_model_get_output_detail
********************************************************************************
* Summary:
*  Retrieves detailed output parameters for a specific output index of the machine
*  learning model, including buffer pointer, size, dimensions, zero point, and scale.
*
* Parameters:
*  const mtb_ml_model_t *object: Pointer to the initialized model object
*  int index: Index of the output tensor
*  MTB_ML_DATA_T **out_pptr: Pointer to store the output buffer pointer
*  size_t *size_ptr: Pointer to store the output size
*  int **dim_ptr: Pointer to store the output dimensions
*  int *dim_len_ptr: Pointer to store the output dimensions length
*  int *zero_ptr: Pointer to store the output zero point
*  float *scale_ptr: Pointer to store the output scale
*
* Return:
*  cy_rslt_t: Result of the operation (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/
cy_rslt_t mtb_ml_model_get_output_detail(const mtb_ml_model_t *object, int index,
                                            MTB_ML_DATA_T **out_pptr, size_t* size_ptr,
                                            int** dim_ptr, int* dim_len_ptr,
                                            int* zero_ptr, float* scale_ptr)
{
    /* Sanity check of input parameters */
    if ( object == NULL ) {
        return MTB_ML_RESULT_BAD_ARG;
    }

    tflite::MTB_TFLM_Class *Tflm = reinterpret_cast<tflite::MTB_TFLM_Class *>(object->tflm_obj);

    if (size_ptr != NULL)       *size_ptr    = Tflm->output_elements(index);
    if (out_pptr != NULL)       *out_pptr    = Tflm->output_ptr(index);
    if (zero_ptr != NULL)        *zero_ptr     = Tflm->output_zero_point(index);
    if (scale_ptr != NULL)        *scale_ptr     = Tflm->output_scale(index);
    if (dim_len_ptr != NULL)    *dim_len_ptr= Tflm->output_dims_len(index);
    if (dim_ptr != NULL)        *dim_ptr     = Tflm->output_dims(index);

    return MTB_ML_RESULT_SUCCESS;
}


/*******************************************************************************
* Function Name: mtb_ml_model_get_input_size
********************************************************************************
* Summary:
*  Retrieves the input buffer size of the machine learning model.
*
* Parameters:
*  const mtb_ml_model_t *object: Pointer to the initialized model object
*
* Return:
*  int: Size of the input buffer
*
*******************************************************************************/
int mtb_ml_model_get_input_size(const mtb_ml_model_t *object)
{
    return object->input_size;
}


/*******************************************************************************
* Function Name: mtb_ml_model_profile_config
********************************************************************************
* Summary:
*  Configures profiling settings for the machine learning model, enabling or
*  disabling profiling and resetting profiling metrics if enabled.
*
* Parameters:
*  mtb_ml_model_t *object: Pointer to the initialized model object
*  mtb_ml_profile_config_t config: Profiling configuration to apply
*
* Return:
*  cy_rslt_t: Result of the operation (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/

cy_rslt_t mtb_ml_model_profile_config(mtb_ml_model_t *object, mtb_ml_profile_config_t config)
{
    /* Sanity check of input parameters */
    if (object == NULL)
    {
        return MTB_ML_RESULT_BAD_ARG;
    }

    object->profiling = config;
    if (object->profiling != MTB_ML_PROFILE_DISABLE)
    {
        object->m_sum_frames = 0;
        object->m_cpu_sum_cycles = 0;
        object->m_cpu_peak_frame = 0;
        object->m_cpu_peak_cycles = 0;
    }
#if defined(COMPONENT_U55)
        object->m_npu_sum_cycles = 0;
        object->m_npu_peak_frame = 0;
        object->m_npu_peak_cycles = 0;
#endif
    return MTB_ML_RESULT_SUCCESS;
}


/*******************************************************************************
* Function Name: mtb_ml_model_profile_log
********************************************************************************
* Summary:
*  Logs profiling information for the machine learning model, including average
*  and peak CPU and NPU cycles, if profiling is enabled.
*
* Parameters:
*  mtb_ml_model_t *object: Pointer to the initialized model object
*
* Return:
*  cy_rslt_t: Result of the operation (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/
cy_rslt_t mtb_ml_model_profile_log(mtb_ml_model_t *object)
{
    /* Sanity check of input parameters */
    if (object == NULL)
    {
        return MTB_ML_RESULT_BAD_ARG;
    }

    if (object->profiling & MTB_ML_PROFILE_ENABLE_MODEL)
    {
        printf("PROFILE_INFO, MTB ML model profile, avg_cpu_cyc=%-10.2f, peak_cpu_cyc=%-" PRIu32 ", peak_cpu_frame=%-" PRIu32 ", cpu_freq_Mhz=%-" PRIu32 "\r\n",
                (float)object->m_cpu_sum_cycles / object->m_sum_frames,
                object->m_cpu_peak_cycles,
                object->m_cpu_peak_frame,
                mtb_ml_cpu_clk_freq / 1000000);
#if defined(COMPONENT_U55)
        printf("PROFILE_INFO, MTB ML model profile, avg_npu_cyc=%-10.2f, peak_npu_cyc=%-" PRIu32 ", peak_npu_frame=%-" PRIu32 ", npu_freq_Mhz=%-" PRIu32 "\r\n",
                (float)object->m_npu_sum_cycles / object->m_sum_frames,
                object->m_npu_peak_cycles,
                object->m_npu_peak_frame,
                mtb_ml_npu_clk_freq / 1000000);
#endif
    }

    return MTB_ML_RESULT_SUCCESS;
}


/*******************************************************************************
* Function Name: mtb_ml_model_rnn_reset_all_parameters
********************************************************************************
* Summary:
*  Resets all parameters of a recurrent neural network (RNN) model to their initial
*  state.
*
* Parameters:
*  mtb_ml_model_t *object: Pointer to the initialized model object
*
* Return:
*  cy_rslt_t: Result of the operation (MTB_ML_RESULT_SUCCESS or error code)
*
*******************************************************************************/
cy_rslt_t mtb_ml_model_rnn_reset_all_parameters(mtb_ml_model_t *object)
{
    /* Sanity check of model object parameter */
    if (object == NULL)
    {
        return MTB_ML_RESULT_BAD_ARG;
    }
    tflite::MTB_TFLM_Class *Tflm = reinterpret_cast<tflite::MTB_TFLM_Class *>(object->tflm_obj);
    TfLiteStatus ret = Tflm->reset_all_variables();
    if (ret != kTfLiteOk)
    {
        object->lib_error = ret;
        return MTB_ML_RESULT_BAD_ARG;
    }
    return MTB_ML_RESULT_SUCCESS;
}

#ifdef __cplusplus
}
#endif  /* __cplusplus */

/* [] END OF FILE */
