/***************************************************************************//**
* \file ifx_face_id.h
*
* \brief
* This is the header file of detection utility functions.
*
*******************************************************************************
* (c) 2019-2025, Cypress Semiconductor Corporation (an Infineon company) or
* an affiliate of Cypress Semiconductor Corporation.  All rights reserved.
*******************************************************************************
* This software, including source code, documentation and related materials
* ("Software"), is owned by Cypress Semiconductor Corporation or one of its
* subsidiaries ("Cypress") and is protected by and subject to worldwide patent
* protection (United States and foreign), United States copyright laws and
* international treaty provisions. Therefore, you may use this Software only
* as provided in the license agreement accompanying the software package from
* which you obtained this Software ("EULA").
*
* If no EULA applies, Cypress hereby grants you a personal, non-exclusive,
* non-transferable license to copy, modify, and compile the Software source
* code solely for use in connection with Cypress's integrated circuit products.
* Any reproduction, modification, translation, compilation, or representation
* of this Software except as specified above is prohibited without the express
* written permission of Cypress.
*
* Disclaimer: THIS SOFTWARE IS PROVIDED AS-IS, WITH NO WARRANTY OF ANY KIND,
* EXPRESS OR IMPLIED, INCLUDING, BUT NOT LIMITED TO, NONINFRINGEMENT, IMPLIED
* WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE. Cypress
* reserves the right to make changes to the Software without notice. Cypress
* does not assume any liability arising out of the application or use of the
* Software or any product or circuit described in the Software. Cypress does
* not authorize its products for use in any products where a malfunction or
* failure of the Cypress product may reasonably be expected to result in
* significant property damage, injury or death ("High Risk Product"). By
* including Cypress's product in a High Risk Product, the manufacturer of such
* system or application assumes all risk of such use and in doing so agrees to
* indemnity Cypress against all liability.
*******************************************************************************/

#ifndef _IFX_FACEID_H_
#define _IFX_FACEID_H_

#ifdef __cplusplus
extern "C" {
#endif

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#include "cy_result.h"
#include "cy_retarget_io.h"


/*******************************************************************************
* Macros
*    DO NOT MODIFY UNLESS EXPERT USER!
*******************************************************************************/
// macros for embeddings database size
#define IFX_FACEID_MAX_DETECTIONS  (20)                         // maximum number of detections from NMS output for a single frame
#define MAX_IFX_FACEID_STRING_LEN  (20)                         // Max string length for enrolled user names coming from UART transfer
#define MAX_N_USER                 (5)                          // Max num users we wish to allow to enroll
#define NUM_EMBEDDINGS_PER_POSE    (7)                          // Number of embeddings to keep per pose in on-device enrolment
#define MAX_EXPERIMENT_KEY_LEN     (100)                        // Max experiment key length from UART tranfer
#define MAX_TIMESTAMP_STR_LEN      (100)                        // Max timestamp length from UART transfer
#define MAX_STRING_LEN             (20)                         // Max string length for enrolled user names coming from UART transfer
#define EMBEDDING_CHUNK_SIZE       (256)                        // Chunk size of embeddings sent over UART
#define MAX_NUM_POSES              (9)
#define MAX_N_EMBEDDING_CHUNKS     (MAX_NUM_POSES * NUM_EMBEDDINGS_PER_POSE * MAX_N_USER)         // Max number of total embeddings examples/chunks in array
#define MAX_EMBEDDINGS             (MAX_N_EMBEDDING_CHUNKS * EMBEDDING_CHUNK_SIZE)    // From embeddings_DB size
#define MAX_HASH_ID_LEN            (16)                         // Increased to 16 to accommodate longer hash IDs safely
#define MAX_PLATFORM_LEN           (16)                         // Platform name + null terminator + padding


/******************************************************************************
 * Typedefs
*****************************************************************************/

// Detection option structure
typedef struct {
    uint16_t    image_width;                        // Width of image
    uint16_t    image_height;                       // Height of image
    float       scaled_width;                       // Scaled width for model output to image space
    float       scaled_height;                      // Scaled height for model output to image space
} ifx_faceid_opt_t;

// Final output information structure for detected human faces.
typedef struct {
    int32_t     count;                              // Number of predictions above set threshold
    int16_t     *bbox_int16;                        // Bounding boxes for each prediction [xyxy]
    float       *conf;                              // Confidence score for prediction
    float       *landmarks;                         // Facial landmarks per prediction [xxxxxyyyyy]
    float       *similarity;                        // Similarity score to enrolled users per prediction
    char        **class_string;                     // Enrolled users name
    int8_t      *emb;                               // Predicted embeddings (quantized)
    int16_t     *id;                                // Prediction identification value (should always be 0)
    int16_t     *yaw;                                // Head pose yaw in Euler degrees
    int16_t     *roll;                                // Head pose pitch in Euler degrees
    int16_t     *pitch;                                // Head pose roll in Euler degrees
    bool        *frontal_face;                         // is face front facing (based on head pose ranges)
    bool        *is_tracked;                        // Indicates if this prediction is being tracked
    int8_t      max_detections;                     // Max number of detections supported by lib in a given frame
} ifx_faceid_prediction_t;

// Struct used to return aligned faces for further inspection.
typedef struct {
    uint8_t num_aligned_faces;                      // Number of aligned faces in aligned_faces
    int8_t* aligned_faces;                          // Aligned faces list.
    uint16_t face_width;                            // Aligned face width
    uint16_t face_height;                           // Aligned face height
} aligned_faces_t;

// Enrolment Embeddings
typedef struct
{
    char        experiment_key[MAX_EXPERIMENT_KEY_LEN];                 // Experiment key list
    char        header_file_timestamp_str[MAX_TIMESTAMP_STR_LEN];       // Timestamp for header file generation
    char        image_classes_string_DB[MAX_N_USER][MAX_STRING_LEN];    // FaceID image classification strings pointer
    char        hash_ids_DB[MAX_N_EMBEDDING_CHUNKS][MAX_HASH_ID_LEN];   // Hash IDs for each embedding
    char        enrollment_platforms_DB[MAX_N_EMBEDDING_CHUNKS][MAX_PLATFORM_LEN]; // Enrolment platforms for each embedding
    float       embeddings_mag_DB[MAX_N_EMBEDDING_CHUNKS];              // FaceID embeddings magnitudes pointer
    float       user_thresholds[MAX_N_USER];                            // Per-user similarity thresholds
    int8_t      embeddings_DB[MAX_EMBEDDINGS];                          // FaceID enrolled users embeddings pointer
    uint16_t    embeddings_num_DB;                                      // FaceID number of enrolled users embeddings (length of embeddings_DB array)
    uint16_t    embeddings_mag_num_DB;                                  // FaceID number of magnitude embeddings (length of embeddings_mag_DB array)
    uint16_t    image_classes_num_DB;                                   // FaceID number of image class strings indices (length of image_classes_DB array, same as number of enrolled embeddings)
    uint16_t    hash_ids_num_DB;                                        // Number of hash IDs (should match image_classes_num_DB)
    uint16_t    enrollment_platforms_num_DB;                            // Number of enrolment platforms (should match image_classes_num_DB)
    bool        k_means;                                                // Embeddings were generated using K-Means
    uint8_t     image_classes_string_num_DB;                            // FaceID number of image class strings. Equivalent to number of unique users in database.
    uint8_t     image_classes_DB[MAX_N_EMBEDDING_CHUNKS];               // FaceID image class strings indices pointer
    uint32_t    init_signature;                                         // Initialization signature
} ifx_faceid_embeddings_t;

// Configurable enrolment parameters for noisy sensors
typedef struct {
    float min_ref_image_conf;           // Minimum confidence for reference image (default: 0.50f)
    float min_main_ref_sim;             // Minimum similarity to main reference (default: 0.20f)  
    float min_pose_ref_sim;             // Minimum similarity to pose-specific reference (default: 0.25f)
    float enrollment_sim_warning_th;    // Warning threshold for similar users (default: 0.70f)
    uint8_t samples_per_pose_completion; // Samples required per pose (default: 5)
    uint8_t pose_buffer_size;           // Max samples per pose buffer (default: 10)
    uint32_t buffer_update_freq_ms;     // Min time between updates in ms (default: 500)
    uint8_t num_poses;                  // Number of enrolment poses to capture (3, 5 or 9)
    uint32_t is_initialized;            // Magic flag to indicate if initialized or not
} ifx_enrollment_config_t;

// Configurable inference parameters for noisy sensors
typedef struct {
    float default_face_id_sim_th;   // Thresh_faceid_sim_default - Default face similarity threshold (cosine)
    float thresh_det_conf;           // Confidence threshold for face detection
    float thresh_det_iou;           // IoU threshold for face detection
    float thresh_kp_conf;           // Confidence threshold for keypoint detection
    float thresh_kp_iou;            // IoU threshold for keypoint detection
    uint32_t is_initialized;        // Magic flag to indicate if initialized or not
} ifx_inference_config_t;

/**
 * @brief Result codes for FaceID operations
 *
 * These result codes provide specific information about the success or failure
 * of FaceID operations, allowing for more detailed error reporting.
 */
typedef enum {
    /* General results */
    IFX_FACEID_RSLT_SUCCESS                = 0x00000000U,  /**< Operation completed successfully */
    IFX_FACEID_RSLT_ERROR                  = 0x10000001U,  /**< Generic error */
    
    /* Parameter validation errors */
    IFX_FACEID_RSLT_BAD_ARG                = 0x10000010U,  /**< Invalid function argument */
    IFX_FACEID_RSLT_NULL_PTR               = 0x10000011U,  /**< Null pointer error */
    IFX_FACEID_RSLT_INVALID_STATE          = 0x10000012U,  /**< Operation not allowed in current state */
    IFX_FACEID_RSLT_INVALID_RANGE          = 0x10000013U,  /**< Invalid value range */

    /* Memory and resource errors */
    IFX_FACEID_RSLT_OUT_OF_MEMORY          = 0x10000020U,  /**< Memory allocation failure */
    IFX_FACEID_RSLT_RESOURCE_BUSY          = 0x10000021U,  /**< Required resource is busy */
    IFX_FACEID_RSLT_BUFFER_OVERFLOW        = 0x10000022U,  /**< Buffer not large enough */
    
    /* Model and inference errors */
    IFX_FACEID_RSLT_MODEL_INIT_FAILED      = 0x10000030U,  /**< Failed to initialize model */
    IFX_FACEID_RSLT_INFERENCE_FAILED       = 0x10000031U,  /**< Inference operation failed */
    IFX_FACEID_RSLT_UNSUPPORTED_MODEL      = 0x10000032U,  /**< Model type not supported */
    IFX_FACEID_RSLT_MEM_ALLOC_FAILED       = 0x10000033U,  /**< Memory allocation issue */
    
    /* Database errors */
    IFX_FACEID_RSLT_DB_FULL                = 0x10000040U,  /**< Database has reached maximum capacity */
    IFX_FACEID_RSLT_DB_NOT_FOUND           = 0x10000041U,  /**< Requested item not found in database */
    IFX_FACEID_RSLT_DB_CORRUPT             = 0x10000042U,  /**< Database corruption detected */
    IFX_FACEID_RSLT_DB_VERSION_MISMATCH    = 0x10000043U,  /**< Database version incompatible */
    
    /* Enrolment specific errors */
    IFX_FACEID_RSLT_ENROLL_WAITING         = 0x10000050U,  /**< Waiting for reference face */
    IFX_FACEID_RSLT_ENROLL_INSUFFICIENT    = 0x10000051U,  /**< Insufficient data for enrolment */
    IFX_FACEID_RSLT_ENROLL_LOW_QUALITY     = 0x10000052U,  /**< Face quality too low for enrolment */
    IFX_FACEID_RSLT_ENROLL_TOO_SIMILAR     = 0x10000053U,  /**< New enrolment too similar to existing user */
    IFX_FACEID_RSLT_ENROLL_POSE_OUT_OF_RANGE=0x10000054U,  /**< New enrolment out of supported pose ranges */
    IFX_FACEID_RSLT_ENROLL_FACE_AT_BOUNDARY= 0x10000055U,  /**< New enrolment out of image boundary */
    IFX_FACEID_RSLT_ENROLL_POSE_REF_LOW_CONF=0x10000056U, /**< Pose reference confidence too low */
    IFX_FACEID_RSLT_ENROLL_MULTI_FACE       =0x10000057U, /**< Multiple faces input to system */
    IFX_FACEID_RSLT_ENROLL_LOW_SIM_MAIN_REF =0x10000058U, /**< Pose similarity to main reference too low */
    IFX_FACEID_RSLT_ENROLL_LOW_SIM_POSE_REF =0x10000059U, /**< Pose similarity to pose reference too low */
    IFX_FACEID_RSLT_ENROLL_POSE_COMPLETE    =0x10000150U, /**< Pose complete */

    /* Communication errors */
    IFX_FACEID_RSLT_COMM_ERROR             = 0x10000060U,  /**< Communication error */
    IFX_FACEID_RSLT_TIMEOUT                = 0x10000061U,  /**< Operation timed out */
    IFX_FACEID_RSLT_NO_DATA                = 0x10000062U,  /**< No data received */

    /* Feature enablement */
    IFX_FACEID_RSLT_FEATURE_DISABLED       = 0x10000070U,  /**< Requested feature is disabled in this build */
    
    /* Configuration errors */
    IFX_FACEID_RSLT_CONFIG_INVALID         = 0x10000080U,  /**< Invalid configuration */

} ifx_faceid_rslt_t;

// On-Device Enrolment Head Pose Progress Tracking
typedef enum {
    PROGRESS_NOT_STARTED = 0,                               // enrolment has not started yet for that pose
    PROGRESS_IN_PROGRESS = 1,                               // enrolment is in-progress for that pose
    PROGRESS_COMPLETED = 2                                  // enrolment has completed for that pose
} ifx_enroll_pose_progress_t;

// On-Device Enrolment Status Updates (Off-Device is instantaneous so no need for its inclusion here)
typedef struct {
    ifx_enroll_pose_progress_t *enroll_progress;            // enrolment head-pose progress tracking
    char **pose_names;                                      // names of head poses
    int num_poses;                                          // number of head poses
} ifx_on_dev_enroll_stats_t;

/**
 * Enrolment operation modes for unified enrolment API
 */
typedef enum {
    ENROLL_MODE_INIT,                                       // Initialize and get reference face
    ENROLL_MODE_UPDATE,                                     // Update with new face data
    ENROLL_MODE_STATUS,                                     // Get current enrolment status
    ENROLL_MODE_AUTO                                        // Auto determine if init or update needed
} ifx_enroll_mode_t;


/*******************************************************************************
* Function Prototypes
*******************************************************************************/

/**
 * \brief               : Create and initialize NN models.
 *
 * \param[in]   opt        : Pointer of detection option structure.
 * \param[in]   pred    : Pointer of detection structure.
 * \param[in]   emb        : Pointer of embeddings DB structure.
 * \param[in]   inference_config: Inference config
 * \param[in]   enrollment_config: Enrolment config
 *
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_init(ifx_faceid_opt_t *opt, ifx_faceid_prediction_t* pred, ifx_faceid_embeddings_t* emb, ifx_inference_config_t* inference_config, ifx_enrollment_config_t *enroll_config);

/**
 * \brief       : Deactivate NN and free models from memory.
 *
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_deinit(void);

/**
 * \brief                               : Infer the NN models with the normalized input image. This API will perform pre processing before inference and post processing after inference.
 *
 * \param[in]   image_buf               : Pointer of input image buffer.
 * \param[out]  opt                     : Pointer of face detection options.
 * \param[out]  prediction              : Pointer of output information structure for detected human faces.
 * \param[in]   faceid_embeddings       : Pointer of embeddings database structure.
 * \param[in]   augment                 : Boolean to indicate augmentation applied.
 * \param[in]   config                  : Pointer of inference config structure.
 *
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_inference(uint8_t *image_buf, ifx_faceid_opt_t *opt, ifx_faceid_prediction_t *prediction, ifx_faceid_embeddings_t *faceid_embeddings_t, bool augment, const ifx_inference_config_t *config);

/**
 * \brief                                   : Populate faceid_embeddings_t with the embedding data from the .a file created by precompiled embeddings file.
 *                                            All these variables come from the .h file that gets generated from the enrolment GUI.
 * \param[in] faceid_embeddings_t           : Pointer to the ifx_faceid_embeddings_t struct.
 * \param[in] experiment_key                : Embeddings experiment key
 * \param[in] header_file_timestamp_str     : Embeddings timestamp
 * \param[in] k_means                       : Kmeans was enabled
 * \param[in] image_classes_string_num_DB   : Number of classes (i.e. enrolled users)
 * \param[in] image_classes_string_DB       : Pointer to the list of enrolled user names
 * \param[in] image_classes_num_DB          : Number of class indices
 * \param[in] image_classes_DB              : Pointer to the list of class indices
 * \param[in] embeddings_num_DB             : Number of embeddings for enrolled users
 * \param[in] embeddings_DB                 : Pointer to the list of embeddings for enrolled users
 * \param[in] embeddings_num_mag_DB         : Number of embeddings magnitudes
 * \param[in] embeddings_mag_DB             : Pointer to the list of embeddings magnitudes
 * \param[in] inference_config              : Pointer to inference config structure.
 * \return                                  : IFX_FACEID_RSLT_SUCCESS - success
 *                                          : Otherwise - check the return value for detail.
 */
ifx_faceid_rslt_t ifx_face_id_load_precompiled_embeddings(
    ifx_faceid_embeddings_t *faceid_embeddings_t, 
    char *experiment_key,
    char *header_file_timestamp_str,
    bool k_means,
    uint16_t image_classes_string_num_DB,
    char **image_classes_string_DB,
    uint16_t image_classes_num_DB,
    uint8_t *image_classes_DB,
    uint32_t embeddings_num_DB,
    int8_t *embeddings_DB,
    uint16_t embeddings_num_mag_DB,
    float *embeddings_mag_DB,
    uint16_t hash_ids_num_DB,
    char **hash_ids_DB,
    uint16_t enrollment_platforms_num_DB,
    char **enrollment_platforms_DB,
    const ifx_inference_config_t *inference_config);

/**
 * \brief                           : Clears all enrolled user embeddings in the database (on and off device enrolled users alike).
 * 
 * \param[in] faceid_embeddings     : Pointer to the embeddings database structure
 * \param[in] inference_config      : Pointer to the inference config structure
 * \return                          : ifx_faceid_rslt_t, IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_clear_all_enrolled_users(ifx_faceid_embeddings_t *faceid_embeddings, const ifx_inference_config_t *inference_config);

/**
 * \brief       : Print elepse times of the NN modules. This includes pre processing, inference and post processing times.
 *
 * \return      : IFX_FACEID_RSLT_SUCCESS - success.
 */
ifx_faceid_rslt_t ifx_face_id_print_profiling_info(void);

/**
 * @brief Unified on-device enrolment API
 *
 * Complete enrolment function that handles all enrolment operations:
 * - Initialization and reference face capture
 * - Collection of face variations from different poses
 * - Getting enrolment status information
 *
 * @param[in] pred      Pointer to face prediction data (can be NULL for ENROLL_MODE_STATUS)
 * @param[in] opt       Pointer of face detection options.
 * @param[in] mode      Operation mode (init, update, status, or auto)
 * @param[out] progress  Optional pointer to receive enrolment progress information (required for STATUS)
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful
 *                   IFX_FACEID_RSLT_ENROLL_WAITING if waiting for reference face
 *                   Error codes for failures
 */
ifx_faceid_rslt_t ifx_on_device_enrollment(
    ifx_faceid_prediction_t *pred,
    ifx_faceid_opt_t *opt,
    ifx_enroll_mode_t mode,
    ifx_on_dev_enroll_stats_t *progress,
    ifx_enrollment_config_t *config);

/**
 * @brief Complete the enrolment process
 * 
 * Processes the collected face embeddings, creates a user profile,
 * and adds it to the face database
 * 
 * @param[out] enrolled_db  Pointer to the face embeddings database
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_complete_on_device_enrollment(ifx_faceid_embeddings_t *enrolled_db, const ifx_enrollment_config_t *enrollment_config, const ifx_inference_config_t *inference_config);

/**
 * @brief Abort the ongoing enrolment process
 * 
 * Clears and exits the enrolment process.
 * 
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_abort_on_device_enrollment( const ifx_enrollment_config_t *config );

/**
 * @brief Print detailed information about the embeddings database
 * 
 * This function prints comprehensive information about the current state
 * of the embeddings database, including user names, counts, and data validation.
 * 
 * @param[out] faceid_embeddings     Pointer to the embeddings database structure
 * @return ifx_faceid_rslt_t    IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_print_embeddings_db_info(ifx_faceid_embeddings_t *faceid_embeddings);

/**
 * @brief Tune embeddings database with per user thresholds.
 * 
 * This function computes and updates adaptive thresholds for all users based on
 * inter-user similarities. It should be called whenever users are added to ensure
 * optimal threshold settings.
 * 
 * @param[out] faceid_embeddings     Pointer to the embeddings database structure
 * @return ifx_faceid_rslt_t    IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_tune_embeddings_db(ifx_faceid_embeddings_t *faceid_embeddings, const ifx_inference_config_t *inference_config);

/**
 * @brief Get aligned faces from the last inference operation
 * 
 * This function returns the aligned face data that was processed during the most
 * recent call to ifx_face_id_inference(). The face data is converted from int8_t
 * back to uint8_t format by removing the zero point offset.
 * 
 * @param[out] aligned_faces Pointer to structure that will contain the aligned faces data
 * 
 * @return IFX_FACEID_RSLT_SUCCESS on success, error code on failure
 * 
 * @note The caller is responsible for freeing the allocated memory in aligned_faces->aligned_faces
 * @note This function should be called after ifx_face_id_inference() and before the next inference
 * @note Face data is now always cached regardless of tracking settings
 */
ifx_faceid_rslt_t ifx_face_id_get_aligned_faces(aligned_faces_t *aligned_faces);

/**
 * @brief Get the pose bin index for given head angles
 * 
 * This function determines which pose bin the given head angles fall into,
 * using the same logic as the enrolment system.
 * 
 * @param yaw    Yaw angle in degrees
 * @param pitch  Pitch angle in degrees  
 * @param roll   Roll angle in degrees
 * @return int   Pose bin index (0 to num_poses-1), or -1 if not in any supported pose range
 */
int ifx_face_id_get_pose_bin_index(const ifx_enrollment_config_t *config, int16_t yaw, int16_t pitch, int16_t roll);

/**
 * @brief Get current enrolment configuration
 * 
 * @param[out] config  Pointer to receive current configuration
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_get_enrollment_config(ifx_enrollment_config_t *config);

/**
 * @brief Get current inference configuration
 * 
 * @param[out] config  Pointer to receive current configuration
 * @return ifx_faceid_rslt_t  IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_get_inference_config(ifx_inference_config_t *config);

/**
 * @brief Get the current number of enrolled users in the database
 * 
 * This function returns the count of unique users currently enrolled in the
 * face recognition database. Useful for checking database capacity before
 * starting enrolment operations.
 * 
 * @param[in] faceid_embeddings  Pointer to the embeddings database structure
 * @param[out] user_count        Pointer to receive the current user count
 * @return ifx_faceid_rslt_t     IFX_FACEID_RSLT_SUCCESS if successful, error code otherwise
 */
ifx_faceid_rslt_t ifx_face_id_get_enrolled_user_count(const ifx_faceid_embeddings_t *faceid_embeddings, 
                                                      uint8_t *user_count);

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _IFX_FACEID_H_ */
