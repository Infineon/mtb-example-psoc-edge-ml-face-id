/******************************************************************************
* File Name        : chunked_stream.c
*
* Description      : Chunked streaming implementation - eliminates race conditions
*                    by copying only 1KB at a time from camera buffer.
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "chunked_stream.h"
#include "cy_scb_uart.h"
#include "cybsp.h"
#include "FreeRTOS.h"
#include "task.h"
#include "jpeg_encoder.h"
#include <string.h>
#include <stdio.h>
#include <inttypes.h>

/*******************************************************************************
* Macros
*******************************************************************************/
#define CAMERA_WIDTH    320
#define CAMERA_HEIGHT   240
#define RGB565_FRAME_SIZE (CAMERA_WIDTH * CAMERA_HEIGHT * 2)  /* 153.6 KB */

/* Streaming buffer in EXTERNAL PSRAM */
CY_SECTION(".cy_xip")
__attribute__((aligned(128), used))
static uint8_t g_jpeg_work_buffer[64 * 1024];  /* 64KB for JPEG output */

CY_SECTION(".cy_xip")
__attribute__((aligned(128), used))
static uint8_t g_chunk_copy_buffer[CHUNK_PAYLOAD_SIZE];  /* 1KB chunk staging */

/*******************************************************************************
* Static Variables
*******************************************************************************/
static volatile bool g_streaming_active = false;
static uint8_t g_frame_sequence = 0;
static uint8_t g_jpeg_quality = JPEG_QUALITY_BEST;  /* Default: Best quality (0) */

/* Current frame state */
typedef struct {
    const uint8_t *jpeg_data;       /* Pointer to JPEG compressed data */
    uint32_t jpeg_size;             /* Size of JPEG data in bytes */
    uint16_t current_chunk;         /* Current chunk being sent */
    uint16_t total_chunks;          /* Total chunks in frame */
    bool     sending;               /* True if frame is being sent */
} frame_state_t;

static frame_state_t g_frame_state = {0};

/* Face ID metadata state */
static faceid_metadata_payload_t g_faceid_metadata = {0};
static volatile bool g_faceid_pending = false;

/* Enrollment progress state */
static enrollment_progress_payload_t g_enrollment_progress = {0};
static volatile bool g_enrollment_pending = false;

/*******************************************************************************
* Static Functions
*******************************************************************************/

/*******************************************************************************
* Function Name: uart_send_blocking
********************************************************************************
* Summary:
*  Sends a data buffer over UART using a blocking FIFO-fill approach.
*  The function yields CPU time while waiting for TX FIFO space.
*
* Parameters:
*  const uint8_t *data: Pointer to the bytes to transmit
*  uint32_t len: Number of bytes to send
*
* Return:
*  uint32_t: Number of bytes sent
*
*******************************************************************************/
static uint32_t uart_send_blocking(const uint8_t *data, uint32_t len)
{
    uint32_t sent = 0;
    uint32_t fifo_size = Cy_SCB_GetFifoSize(CYBSP_TENXER_UART_HW);

    while (sent < len)
    {
        uint32_t in_fifo = Cy_SCB_UART_GetNumInTxFifo(CYBSP_TENXER_UART_HW);
        uint32_t space = fifo_size - in_fifo;

        if (space > 0)
        {
            uint32_t to_send = (len - sent > space) ? space : (len - sent);
            for (uint32_t i = 0; i < to_send; i++) {
                Cy_SCB_UART_Put(CYBSP_TENXER_UART_HW, (uint32_t)data[sent]);
                sent++;
            }
        }
        else
        {
            taskYIELD();  /* Wait for FIFO space */
        }
    }

    return sent;
}

/*******************************************************************************
* Function Name: send_chunk
********************************************************************************
* Summary:
*  Builds and sends one protocol chunk over UART, including chunk header
*  and payload data.
*
* Parameters:
*  uint8_t type: Chunk type identifier
*  uint8_t seq: Frame sequence number
*  uint16_t chunk_id: Current chunk index
*  uint16_t total_chunks: Total chunks in the frame
*  const uint8_t *payload: Pointer to chunk payload bytes
*  uint16_t payload_size: Payload size in bytes
*
* Return:
*  bool: true if header and payload were sent, false otherwise
*
*******************************************************************************/
static bool send_chunk(uint8_t type, uint8_t seq, uint16_t chunk_id,
                       uint16_t total_chunks, const uint8_t *payload, uint16_t payload_size)
{
    chunk_header_t header;

    /* Build header */
    header.magic[0] = CHUNK_MAGIC_0;
    header.magic[1] = CHUNK_MAGIC_1;
    header.type = type;
    header.sequence = seq;
    header.chunk_id = chunk_id;
    header.total_chunks = total_chunks;
    header.payload_size = payload_size;
    header.crc16 = chunked_stream_crc16(payload, payload_size);

    /* Send header */
    if (uart_send_blocking((const uint8_t*)&header, sizeof(header)) != sizeof(header))
    {
        return false;
    }

    /* Send payload */
    if (uart_send_blocking(payload, payload_size) != payload_size)
    {
        return false;
    }

    return true;
}

/*******************************************************************************
* Public Functions
*******************************************************************************/

/*******************************************************************************
* Function Name: chunked_stream_init
********************************************************************************
* Summary:
*  Initializes chunked-stream state and clears frame and metadata buffers.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void chunked_stream_init(void)
{
    g_streaming_active = false;
    g_frame_sequence = 0;
    memset(&g_frame_state, 0, sizeof(g_frame_state));
    memset(&g_faceid_metadata, 0, sizeof(g_faceid_metadata));
    g_faceid_pending = false;

    printf("[CHUNKED] Initialized - chunk size: %d bytes\n", CHUNK_PAYLOAD_SIZE);
}

/*******************************************************************************
* Function Name: chunked_stream_start
********************************************************************************
* Summary:
*  Enables chunked streaming and resets frame sequence tracking.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void chunked_stream_start(void)
{
    g_streaming_active = true;
    g_frame_sequence = 0;
    printf("[CHUNKED] Streaming started\n");
}

/*******************************************************************************
* Function Name: chunked_stream_stop
********************************************************************************
* Summary:
*  Disables chunked streaming and clears current frame transmission state.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void chunked_stream_stop(void)
{
    g_streaming_active = false;
    memset(&g_frame_state, 0, sizeof(g_frame_state));
    printf("[CHUNKED] Streaming stopped\n");
}

/*******************************************************************************
* Function Name: chunked_stream_is_active
********************************************************************************
* Summary:
*  Checks whether chunked streaming is currently enabled.
*
* Parameters:
*  void
*
* Return:
*  bool: true when streaming is active, false otherwise
*
*******************************************************************************/
bool chunked_stream_is_active(void)
{
    return g_streaming_active;
}

/*******************************************************************************
* Function Name: chunked_stream_push_frame
********************************************************************************
* Summary:
*  Encodes an RGB565 frame into JPEG, divides it into protocol chunks,
*  and queues it for chunked UART transmission.
*
* Parameters:
*  const uint16_t *rgb565: Pointer to RGB565 camera frame buffer
*
* Return:
*  bool: true if frame was accepted and queued, false on error or busy state
*
*******************************************************************************/
bool chunked_stream_push_frame(const uint16_t *rgb565)
{
    if (!g_streaming_active || !rgb565)
    {
        return false;
    }

    /* Drop frame if previous one still sending */
    if (g_frame_state.sending)
    {
        printf("[CHUNKED] Frame dropped - previous frame still sending\n");
        return false;
    }

    /* Compress frame to JPEG (this operates on static buffer, safe) */
    uint32_t jpeg_size = 0;
    int result = jpeg_encode_rgb565(rgb565,
                                    CAMERA_WIDTH, CAMERA_HEIGHT,
                                    g_jpeg_quality, /* quality 0=BEST, 1=HIGH, 2=MED, 3=LOW */
                                    JPEG_SUBSAMPLE_444, /* subsample: 0=4:4:4 (best quality) */
                                    g_jpeg_work_buffer, sizeof(g_jpeg_work_buffer),
                                    &jpeg_size);
    if (result < 0)
    {
        printf("[CHUNKED] JPEG encoding failed: %d\n", result);
        return false;
    }

    /* Calculate chunks needed */
    uint16_t total_chunks = (jpeg_size + CHUNK_PAYLOAD_SIZE - 1) / CHUNK_PAYLOAD_SIZE;

    /* Setup frame state for UART task to stream */
    g_frame_state.jpeg_data = g_jpeg_work_buffer;
    g_frame_state.jpeg_size = jpeg_size;
    g_frame_state.current_chunk = 0;
    g_frame_state.total_chunks = total_chunks;
    g_frame_state.sending = true;

    /* Increment sequence */
    g_frame_sequence++;
    if (g_frame_sequence == 0)
    {
        g_frame_sequence = 1;  /* Skip 0 */
    }

    return true;
}

/*******************************************************************************
* Function Name: chunked_stream_push_faceid
********************************************************************************
* Summary:
*  Queues Face ID metadata so it can be transmitted after the current
*  image frame is fully sent.
*
* Parameters:
*  uint8_t num_faces: Number of detected faces in the payload
*  const faceid_face_info_t *faces: Pointer to face metadata array
*
* Return:
*  bool: true if metadata was queued, false on invalid input or inactive stream
*
*******************************************************************************/
bool chunked_stream_push_faceid(uint8_t num_faces, const faceid_face_info_t *faces)
{
    if (!g_streaming_active)
    {
        return false;
    }

    if (num_faces > MAX_FACES_PER_FRAME)
    {
        printf("[CHUNKED] ERROR: Too many faces (%d > %d)\n", num_faces, MAX_FACES_PER_FRAME);
        return false;
    }

    /* Copy metadata (quick operation) */
    g_faceid_metadata.num_faces = num_faces;
    memset(g_faceid_metadata.reserved, 0, sizeof(g_faceid_metadata.reserved));

    if (num_faces > 0 && faces)
    {
        memcpy(g_faceid_metadata.faces, faces,
               num_faces * sizeof(faceid_face_info_t));
    }

    g_faceid_pending = true;

    /* Debug: Log metadata queued (throttled) */
    static uint32_t metadata_count = 0;
    metadata_count++;
    if ((metadata_count % 50) == 1)
    {
        printf("[CHUNKED] Face ID metadata queued: %d faces (count: %" PRIu32 ")\n", num_faces, metadata_count);
    }

    return true;
}

/*******************************************************************************
* Function Name: chunked_stream_push_enrollment_progress
********************************************************************************
* Summary:
*  Queues enrollment progress information for transmission as a dedicated
*  status packet.
*
* Parameters:
*  uint8_t state: Enrollment state code
*  uint8_t current_pose: Current pose index
*  uint8_t total_poses: Total number of poses required
*  uint8_t progress_percent: Overall enrollment completion percentage
*  const uint8_t *pose_progress: Per-pose progress values array
*  const char *user_name: User name associated with enrollment
*
* Return:
*  bool: true if progress data was queued, false otherwise
*
*******************************************************************************/
bool chunked_stream_push_enrollment_progress(uint8_t state, uint8_t current_pose,
                                             uint8_t total_poses, uint8_t progress_percent,
                                             const uint8_t *pose_progress, const char *user_name)
{
    if (!g_streaming_active)
    {
        return false;
    }

    /* Copy enrollment progress data */
    g_enrollment_progress.state = state;
    g_enrollment_progress.current_pose = current_pose;
    g_enrollment_progress.total_poses = total_poses;
    g_enrollment_progress.progress_percent = progress_percent;

    /* Copy pose progress array */
    if (pose_progress)
    {
        memcpy(g_enrollment_progress.pose_progress, pose_progress, MAX_ENROLLMENT_POSES);
    }
    else
    {
        memset(g_enrollment_progress.pose_progress, 0, MAX_ENROLLMENT_POSES);
    }

    /* Copy user name (ensure null-termination) */
    if (user_name)
    {
        strncpy(g_enrollment_progress.user_name, user_name, sizeof(g_enrollment_progress.user_name) - 1);
        g_enrollment_progress.user_name[sizeof(g_enrollment_progress.user_name) - 1] = '\0';
    }
    else
    {
        g_enrollment_progress.user_name[0] = '\0';
    }

    g_enrollment_pending = true;

    /* Debug log */
    printf("[CHUNKED] Enrollment progress queued: state=%d, pose=%d/%d, progress=%d%%\n",
           state, current_pose, total_poses, progress_percent);

    return true;
}

/*******************************************************************************
* Function Name: chunked_stream_process
********************************************************************************
* Summary:
*  Processes one transmission step for the chunked stream. Sends the next
*  image chunk and, when complete, sends pending metadata/progress packets.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void chunked_stream_process(void)
{
    if (!g_streaming_active)
    {
        return;
    }

    /* Send next image chunk if frame is active */
    if (g_frame_state.sending)
    {
        uint16_t chunk_id = g_frame_state.current_chunk;
        uint32_t offset = chunk_id * CHUNK_PAYLOAD_SIZE;
        uint32_t remaining = g_frame_state.jpeg_size - offset;
        uint16_t chunk_size = (remaining > CHUNK_PAYLOAD_SIZE) ? CHUNK_PAYLOAD_SIZE : remaining;

        /* Determine chunk type */
        uint8_t chunk_type;
        if (chunk_id == 0)
        {
            chunk_type = CHUNK_TYPE_IMAGE_START;
        }
        else if (chunk_id == g_frame_state.total_chunks - 1)
        {
            chunk_type = CHUNK_TYPE_IMAGE_END;
        }
        else
        {
            chunk_type = CHUNK_TYPE_IMAGE_DATA;
        }

        /* Send chunk */
        if (send_chunk(chunk_type, g_frame_sequence, chunk_id,
                      g_frame_state.total_chunks,
                      g_frame_state.jpeg_data + offset, chunk_size))
        {

            g_frame_state.current_chunk++;

            /* Check if frame complete */
            if (g_frame_state.current_chunk >= g_frame_state.total_chunks)
            {
                g_frame_state.sending = false;

                /* Now send Face ID metadata if pending */
                if (g_faceid_pending)
                {
                    static uint32_t meta_tx_count = 0;
                    if ((++meta_tx_count % 30) == 1)
                    {
                        printf("[CHUNKED] Transmitting Face ID metadata packet: %d faces (tx count: %" PRIu32 ")\n",
                               g_faceid_metadata.num_faces, meta_tx_count);
                    }
                    send_chunk(CHUNK_TYPE_FACEID_META, g_frame_sequence, 0, 1,
                              (const uint8_t*)&g_faceid_metadata,
                              sizeof(g_faceid_metadata));
                    g_faceid_pending = false;
                }

                /* Send enrollment progress if pending */
                if (g_enrollment_pending)
                {
                    uint8_t chunk_type = (g_enrollment_progress.state == 4) ?
                                        CHUNK_TYPE_ENROLL_COMPLETE : CHUNK_TYPE_ENROLL_PROGRESS;
                    send_chunk(chunk_type, g_frame_sequence, 0, 1,
                              (const uint8_t*)&g_enrollment_progress,
                              sizeof(g_enrollment_progress));
                    g_enrollment_pending = false;
                }
            }
        }
        else
        {
            printf("[CHUNKED] Failed to send chunk %d/%d\n",
                   chunk_id, g_frame_state.total_chunks);
            g_frame_state.sending = false;
        }

        /* Yield CPU after each chunk */
        taskYIELD();
    }
}

/*******************************************************************************
* Function Name: chunked_stream_crc16
********************************************************************************
* Summary:
*  Computes CRC16 for chunk payload data using CRC16-CCITT helper routine.
*
* Parameters:
*  const uint8_t *data: Pointer to payload buffer
*  uint32_t length: Payload length in bytes
*
* Return:
*  uint16_t: CRC16 value
*
*******************************************************************************/
uint16_t chunked_stream_crc16(const uint8_t *data, uint32_t length)
{
    return crc16_ccitt_compute(data, length);
}

/*******************************************************************************
* Function Name: crc16_ccitt_update
********************************************************************************
* Summary:
*  Updates CRC16-CCITT state with one input byte.
*
* Parameters:
*  uint16_t crc: Current CRC state
*  uint8_t byte: Next byte to process
*
* Return:
*  uint16_t: Updated CRC value
*
*******************************************************************************/
uint16_t crc16_ccitt_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte << 8;
    for (int i = 0; i < 8; ++i)
    {
        crc = (crc & 0x8000u) ? (uint16_t)((crc << 1) ^ 0x1021u) : (uint16_t)(crc << 1);
    }
    return crc;
}

/*******************************************************************************
* Function Name: crc16_ccitt_compute
********************************************************************************
* Summary:
*  Computes CRC16-CCITT over an input byte buffer.
*
* Parameters:
*  const uint8_t *data: Pointer to input buffer
*  uint32_t len: Number of bytes to process
*
* Return:
*  uint16_t: Final CRC16-CCITT value
*
*******************************************************************************/
uint16_t crc16_ccitt_compute(const uint8_t *data, uint32_t len)
{
    uint16_t crc = 0xFFFFu;

    if (data == NULL)
    {
        return crc;
    }

    for (uint32_t i = 0; i < len; i++)
    {
        crc = crc16_ccitt_update(crc, data[i]);
    }

    return crc;
}

/*******************************************************************************
* Function Name: chunked_stream_set_quality
********************************************************************************
* Summary:
*  Sets JPEG compression quality for subsequent encoded frames.
*
* Parameters:
*  uint8_t quality: Quality level (0=BEST, 1=HIGH, 2=MED, 3=LOW)
*
* Return:
*  void
*
*******************************************************************************/
void chunked_stream_set_quality(uint8_t quality)
{
    if (quality > JPEG_QUALITY_LOW)
    {
        quality = JPEG_QUALITY_LOW;  /* Clamp to valid range 0-3 */
    }
    g_jpeg_quality = quality;
    const char *qual_names[] = {"BEST", "HIGH", "MED", "LOW"};
    printf("[CHUNKED] JPEG quality set to %s (%d)\n", qual_names[quality], quality);
}

/*******************************************************************************
* Function Name: chunked_stream_get_quality
********************************************************************************
* Summary:
*  Returns current JPEG compression quality setting.
*
* Parameters:
*  void
*
* Return:
*  uint8_t: Current quality level
*
*******************************************************************************/
uint8_t chunked_stream_get_quality(void)
{
    return g_jpeg_quality;
}

/* [] END OF FILE */
