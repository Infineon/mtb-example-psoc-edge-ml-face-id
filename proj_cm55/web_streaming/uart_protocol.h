/*******************************************************************************
 * File Name: uart_protocol.h
 *
 * Description:
 *   UART image streaming protocol definitions.
 *   Simplified binary protocol for streaming RGB565/JPEG frames over UART.
 *
 *   Frame Structure:
 *     [Header: 12 bytes] + [Payload: N bytes]
 *
 *   Header Format (little-endian):
 *     - magic[4]:    'F' 'R' 'M' '0' (0x46, 0x52, 0x4D, 0x30)
 *     - width[2]:    Image width in pixels
 *     - height[2]:   Image height in pixels
 *     - format[1]:   1=RGB565, 2=JPEG
 *     - reserved[1]: Reserved for future use
 *     - crc16[2]:    CRC-16 CCITT of payload
 *
 ******************************************************************************
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
 ******************************************************************************/

#ifndef UART_PROTOCOL_H
#define UART_PROTOCOL_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/* Frame header magic bytes: "FRM0" */
#define UART_FRAME_MAGIC_0  (0x46u)  /* 'F' */
#define UART_FRAME_MAGIC_1  (0x52u)  /* 'R' */
#define UART_FRAME_MAGIC_2  (0x4Du)  /* 'M' */
#define UART_FRAME_MAGIC_3  (0x30u)  /* '0' */

/* Format codes */
#define UART_FORMAT_RGB565      (1u)
#define UART_FORMAT_JPEG        (2u)
#define UART_FORMAT_FACEID_META (3u)  /* Face ID metadata (bounding boxes, user IDs) */

/* Reserved byte usage: Face ID detection status */
#define UART_FACEID_NO_FACE     (0u)  /* No face detected */
#define UART_FACEID_UNKNOWN     (1u)  /* Face detected but not recognized */
#define UART_FACEID_RECOGNIZED  (2u)  /* Face recognized (user ID in lower 4 bits) */

/* Header size */
#define UART_FRAME_HEADER_SIZE  (12u)

/* Max faces in metadata frame */
#define UART_MAX_FACES_META     (4u)

/*******************************************************************************
 * Structures
 ******************************************************************************/

/**
 * @brief UART frame header (12 bytes, little-endian packed)
 */
#pragma pack(push, 1)
typedef struct {
    uint8_t  magic[4];      /* 'FRM0' magic bytes */
    uint16_t width;         /* Image width */
    uint16_t height;        /* Image height */
    uint8_t  format;        /* UART_FORMAT_RGB565 or UART_FORMAT_JPEG */
    uint8_t  reserved;      /* Reserved for future use */
    uint16_t crc16;         /* CRC-16 CCITT of payload */
} uart_frame_header_t;

/**
 * @brief Face detection info for a single face
 */
typedef struct {
    int16_t x1;             /* Bounding box top-left X (camera coords) */
    int16_t y1;             /* Bounding box top-left Y (camera coords) */
    int16_t x2;             /* Bounding box bottom-right X (camera coords) */
    int16_t y2;             /* Bounding box bottom-right Y (camera coords) */
    int16_t user_id;        /* User ID (-1 = unknown, >=0 = recognized) */
    uint8_t confidence;     /* Detection confidence (0-100) */
    uint8_t reserved;       /* Reserved for alignment */
} uart_face_info_t;

/**
 * @brief Face ID metadata payload (variable size)
 * Format: num_faces + array of face_info structures
 */
typedef struct {
    uint8_t num_faces;      /* Number of detected faces (0-4) */
    uart_face_info_t faces[UART_MAX_FACES_META];  /* Face info array */
} uart_faceid_metadata_t;
#pragma pack(pop)

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/*******************************************************************************
* Function Name: uart_encode_frame
********************************************************************************
* Summary:
* Constructs a complete UART frame consisting of a 12-byte header followed by
* the supplied payload. The header is populated with the magic bytes, image
* dimensions, format code, Face ID status, and a CRC-16 CCITT checksum
* computed over the payload bytes.
*
* Parameters:
*  out_buf      : Output buffer that receives the encoded frame
*                 (must be large enough to hold the header + payload)
*  out_cap      : Capacity of out_buf in bytes
*  payload      : Pointer to the payload data (RGB565 or JPEG bytes)
*  payload_len  : Length of the payload in bytes
*  width        : Image width in pixels
*  height       : Image height in pixels
*  format       : Pixel/compression format (UART_FORMAT_RGB565 or
*                 UART_FORMAT_JPEG)
*  faceid_status: Face ID detection status byte written to the reserved field
*                 (UART_FACEID_NO_FACE / UART_FACEID_UNKNOWN /
*                  UART_FACEID_RECOGNIZED)
*
* Return:
*  > 0 : Total frame size in bytes (header + payload) on success
*  -1  : Output buffer too small or invalid parameters
*
*******************************************************************************/
int uart_encode_frame(uint8_t *out_buf, uint32_t out_cap,
                      const uint8_t *payload, uint16_t payload_len,
                      uint16_t width, uint16_t height, uint8_t format,
                      uint8_t faceid_status);

/*******************************************************************************
* Function Name: uart_decode_header
********************************************************************************
* Summary:
* Validates the magic bytes of a received UART frame header and extracts the
* image dimensions, format code, and payload length. Does not validate the
* CRC; call uart_validate_crc() separately after confirming sufficient data
* is available.
*
* Parameters:
*  in_buf      : Input buffer containing the received frame
*  in_len      : Number of valid bytes in in_buf
*  width       : [OUT] Image width in pixels
*  height      : [OUT] Image height in pixels
*  format      : [OUT] Format code (UART_FORMAT_RGB565 or UART_FORMAT_JPEG)
*  payload_len : [OUT] Payload size in bytes as encoded in the header
*
* Return:
*   0 : Success
*  -1 : Invalid magic bytes
*  -2 : Insufficient data (in_len < UART_FRAME_HEADER_SIZE)
*
*******************************************************************************/
int uart_decode_header(const uint8_t *in_buf, uint32_t in_len,
                       uint16_t *width, uint16_t *height,
                       uint8_t *format, uint16_t *payload_len);

/*******************************************************************************
* Function Name: uart_validate_crc
********************************************************************************
* Summary:
* Recomputes the CRC-16 CCITT checksum over the payload portion of a received
* frame and compares it against the checksum stored in the frame header.
* The caller must ensure that in_buf contains at least
* UART_FRAME_HEADER_SIZE + payload_len valid bytes before calling this
* function.
*
* Parameters:
*  in_buf      : Input buffer containing the complete frame (header + payload)
*  in_len      : Total number of valid bytes in in_buf
*  payload_len : Payload size in bytes (obtained from uart_decode_header())
*
* Return:
*   1 : CRC valid — payload matches checksum in header
*   0 : CRC mismatch — payload is corrupt
*  -1 : Invalid parameters (NULL pointer or insufficient buffer length)
*
*******************************************************************************/
int uart_validate_crc(const uint8_t *in_buf, uint32_t in_len, uint16_t payload_len);

/*******************************************************************************
* Function Name: uart_encode_faceid_metadata
********************************************************************************
* Summary:
* Constructs a UART frame carrying Face ID metadata (bounding boxes and user
* IDs for all detected faces). The frame uses UART_FORMAT_FACEID_META as the
* format code. The image dimensions are included in the header for the
* receiver to map bounding box coordinates back to display space.
*
* Parameters:
*  out_buf    : Output buffer that receives the encoded frame
*               (must be large enough to hold the header +
*                sizeof(uart_faceid_metadata_t))
*  out_cap    : Capacity of out_buf in bytes
*  metadata   : Pointer to the Face ID metadata structure containing the
*               number of detected faces and per-face bounding box / user ID
*               information
*  img_width  : Width of the original image in pixels (reference for bbox
*               coordinate mapping on the receiver side)
*  img_height : Height of the original image in pixels
*
* Return:
*  > 0 : Total frame size in bytes (header + metadata) on success
*  -1  : Output buffer too small, NULL pointer, or encoding error
*
*******************************************************************************/
int uart_encode_faceid_metadata(uint8_t *out_buf, uint32_t out_cap,
                                const uart_faceid_metadata_t *metadata,
                                uint16_t img_width, uint16_t img_height);

#ifdef __cplusplus
}
#endif

#endif /* UART_PROTOCOL_H */
