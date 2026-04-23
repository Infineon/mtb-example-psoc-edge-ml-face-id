/******************************************************************************
* File Name        : chunked_stream.h
*
* Description      : Chunked streaming protocol for UART camera + Face ID data.
*                    Eliminates race conditions by sending data in small chunks
*                    that can be copied atomically before camera buffer updates.
*
* Related Document : See README.md
*
********************************************************************************
* (c) 2025-2026, Infineon Technologies AG, or an affiliate of Infineon
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

#ifndef CHUNKED_STREAM_H
#define CHUNKED_STREAM_H

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>

/*******************************************************************************
 * Protocol Constants
 *******************************************************************************/
/* Frame types */
#define CHUNK_TYPE_IMAGE_START      0x01    /* First chunk of image frame */
#define CHUNK_TYPE_IMAGE_DATA       0x02    /* Middle chunks of image */
#define CHUNK_TYPE_IMAGE_END        0x03    /* Last chunk of image */
#define CHUNK_TYPE_FACEID_META      0x04    /* Face ID metadata packet */
#define CHUNK_TYPE_HEARTBEAT        0x05    /* Keepalive */
#define CHUNK_TYPE_ENROLL_PROGRESS  0x06    /* Enrollment progress update */
#define CHUNK_TYPE_ENROLL_COMPLETE  0x07    /* Enrollment completed successfully */

/* Chunk size - small enough to copy atomically (1KB) */
#define CHUNK_PAYLOAD_SIZE          1024
#define CHUNK_HEADER_SIZE           12      /* Magic + type + seq + chunk_id + total_chunks + size + crc */

/* Protocol markers */
#define CHUNK_MAGIC_0               0xAB
#define CHUNK_MAGIC_1               0xCD

/* Max face detections in metadata */
#define MAX_FACES_PER_FRAME         4

/* Enrollment constants */
#define MAX_ENROLLMENT_POSES        5       /* Number of poses required for enrollment */

/*******************************************************************************
 * Data Structures
 *******************************************************************************/

/**
 * @brief Chunk header (12 bytes)
 */
typedef struct __attribute__((packed)) {
    uint8_t  magic[2];          /* 0xAB 0xCD */
    uint8_t  type;              /* CHUNK_TYPE_xxx */
    uint8_t  sequence;          /* Frame sequence number (wraps at 255) */
    uint16_t chunk_id;          /* Chunk number within frame (0-based) */
    uint16_t total_chunks;      /* Total chunks in this frame */
    uint16_t payload_size;      /* Actual bytes in this chunk (≤1024) */
    uint16_t crc16;             /* CRC-16 of payload only */
} chunk_header_t;

/**
 * @brief Face ID metadata for one detected face
 */
typedef struct __attribute__((packed)) {
    int16_t  bbox_x1;           /* Bounding box top-left X */
    int16_t  bbox_y1;           /* Bounding box top-left Y */
    int16_t  bbox_x2;           /* Bounding box bottom-right X */
    int16_t  bbox_y2;           /* Bounding box bottom-right Y */
    int16_t  user_id;           /* User ID (-1 = unknown, ≥0 = recognized) */
    uint16_t confidence;        /* Confidence × 1000 (e.g., 0.85 → 850) */
    char     user_name[16];     /* User name (null-terminated) */
} faceid_face_info_t;

/**
 * @brief Face ID metadata frame payload
 */
typedef struct __attribute__((packed)) {
    uint8_t  num_faces;         /* Number of detected faces (0-4) */
    uint8_t  reserved[3];       /* Padding for alignment */
    faceid_face_info_t faces[MAX_FACES_PER_FRAME];
} faceid_metadata_payload_t;

/**
 * @brief Enrollment progress update payload
 */
typedef struct __attribute__((packed)) {
    uint8_t  state;             /* 0=None, 1=Waiting, 2=Collecting, 3=Completing, 4=Completed */
    uint8_t  current_pose;      /* Current pose being captured (0-4) */
    uint8_t  total_poses;       /* Total poses required (usually 5) */
    uint8_t  progress_percent;  /* Overall progress 0-100% */
    uint8_t  pose_progress[MAX_ENROLLMENT_POSES];  /* Progress per pose (0-100) */
    char     user_name[16];     /* User name being enrolled */
} enrollment_progress_payload_t;

/*******************************************************************************
 * Function Prototypes
 *******************************************************************************/

/**
 * @brief Initialize chunked streaming system
 */
void chunked_stream_init(void);

/**
 * @brief Start streaming
 */
void chunked_stream_start(void);

/**
 * @brief Stop streaming
 */
void chunked_stream_stop(void);

/**
 * @brief Check if streaming is active
 * @return true if streaming, false otherwise
 */
bool chunked_stream_is_active(void);

/**
 * @brief Push a new camera frame for streaming (non-blocking)
 *
 * This function quickly copies ONE CHUNK worth of data from the camera buffer,
 * then returns. Subsequent chunks are sent by the UART task. This eliminates
 * race conditions because only 1KB is copied before camera can update buffer.
 *
 * @param rgb565 Pointer to RGB565 frame (320×240 = 153.6KB)
 * @return true if frame queued, false if previous frame still streaming
 */
bool chunked_stream_push_frame(const uint16_t *rgb565);

/**
 * @brief Push Face ID metadata for current frame (non-blocking)
 *
 * Call this AFTER pushing the image frame. The metadata will be sent
 * as a separate packet after all image chunks.
 *
 * @param num_faces Number of detected faces (0-4)
 * @param faces Array of face info structures
 * @return true if queued, false on error
 */
bool chunked_stream_push_faceid(uint8_t num_faces, const faceid_face_info_t *faces);

/**
 * @brief Push enrollment progress update (non-blocking)
 *
 * Call this during enrollment to send progress updates to UART.
 *
 * @param state Enrollment state (0=None, 1=Waiting, 2=Collecting, 3=Completing, 4=Completed)
 * @param current_pose Current pose being captured (0-4)
 * @param total_poses Total poses required
 * @param progress_percent Overall progress 0-100%
 * @param pose_progress Array of per-pose progress (0-100)
 * @param user_name User name being enrolled
 * @return true if queued, false on error
 */
bool chunked_stream_push_enrollment_progress(uint8_t state, uint8_t current_pose,
                                             uint8_t total_poses, uint8_t progress_percent,
                                             const uint8_t *pose_progress, const char *user_name);

/**
 * @brief CRC16-CCITT update (poly 0x1021, init 0xFFFF)
 * @param crc Current CRC state
 * @param byte Next input byte
 * @return Updated CRC value
 */
uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte);

/**
 * @brief CRC16-CCITT compute (poly 0x1021, init 0xFFFF)
 * @param data Pointer to input buffer
 * @param len Number of bytes
 * @return CRC16-CCITT value
 */
uint16_t crc16_ccitt_compute(const uint8_t *data, uint32_t len);

/**
 * @brief UART task worker - call this periodically to send queued chunks
 *
 * This should be called from the UART task main loop. It will send
 * one chunk per call, yielding CPU time between chunks.
 */
void chunked_stream_process(void);

/**
 * @brief Calculate CRC-16 for chunk payload
 *
 * Uses the CRC16-CCITT parameters used by UART framing.
 * @param data Pointer to data buffer
 * @param length Number of bytes
 * @return CRC-16 value
 */
uint16_t chunked_stream_crc16(const uint8_t *data, uint32_t length);

/**
 * @brief Set JPEG compression quality (0=BEST, 1=HIGH, 2=MED, 3=LOW)
 * @param quality Quality level 0-3
 */
void chunked_stream_set_quality(uint8_t quality);

/**
 * @brief Get current JPEG compression quality setting
 * @return Quality level 0-3
 */
uint8_t chunked_stream_get_quality(void);

#ifdef __cplusplus
}
#endif

#endif /* CHUNKED_STREAM_H */

/* [] END OF FILE */
