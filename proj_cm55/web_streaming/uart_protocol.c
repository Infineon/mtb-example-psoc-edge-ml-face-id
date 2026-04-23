/*******************************************************************************
 * File Name: uart_protocol.c
 *
 * Description:
 *   UART image streaming protocol helpers.
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

#include "uart_protocol.h"

#include <string.h>

#include "chunked_stream.h"

/*******************************************************************************
* Function Name: uart_encode_frame
********************************************************************************
* Summary:
*  Encodes a UART frame by writing the protocol header, calculating payload CRC,
*  and appending payload bytes after the header.
*
* Parameters:
*  uint8_t *out_buf: Output buffer for header + payload
*  uint32_t out_cap: Output buffer capacity in bytes
*  const uint8_t *payload: Payload data pointer
*  uint16_t payload_len: Payload size in bytes
*  uint16_t width: Frame width metadata
*  uint16_t height: Frame height metadata
*  uint8_t format: Payload format code
*  uint8_t faceid_status: Face ID status placed in reserved header field
*
* Return:
*  int: Total encoded frame length on success, -1 on error
*
*******************************************************************************/
int uart_encode_frame(uint8_t *out_buf, uint32_t out_cap,
                      const uint8_t *payload, uint16_t payload_len,
                      uint16_t width, uint16_t height, uint8_t format,
                      uint8_t faceid_status)
{
    if (!out_buf || !payload)
    {
        return -1;
    }
    if (out_cap < (UART_FRAME_HEADER_SIZE + payload_len))
    {
        return -1;
    }

    uart_frame_header_t *hdr = (uart_frame_header_t *)out_buf;

    /* Fill magic bytes */
    hdr->magic[0] = UART_FRAME_MAGIC_0;
    hdr->magic[1] = UART_FRAME_MAGIC_1;
    hdr->magic[2] = UART_FRAME_MAGIC_2;
    hdr->magic[3] = UART_FRAME_MAGIC_3;

    /* Fill dimensions and format */
    hdr->width = width;
    hdr->height = height;
    hdr->format = format;
    hdr->reserved = faceid_status;  /* Face ID detection status */

    /* Calculate CRC-16 of payload */
    hdr->crc16 = crc16_ccitt_compute(payload, payload_len);

    /* Copy payload after header */
    memcpy(out_buf + UART_FRAME_HEADER_SIZE, payload, payload_len);

    return (int)(UART_FRAME_HEADER_SIZE + payload_len);
}

/*******************************************************************************
* Function Name: uart_decode_header
********************************************************************************
* Summary:
*  Validates UART frame magic bytes and extracts frame header metadata.
*
* Parameters:
*  const uint8_t *in_buf: Input buffer containing frame header
*  uint32_t in_len: Input buffer length
*  uint16_t *width: Output pointer for decoded width
*  uint16_t *height: Output pointer for decoded height
*  uint8_t *format: Output pointer for decoded format
*  uint16_t *payload_len: Reserved output pointer for payload length
*
* Return:
*  int: 0 on success, -1 on invalid magic, -2 on insufficient input
*
*******************************************************************************/
int uart_decode_header(const uint8_t *in_buf, uint32_t in_len,
                       uint16_t *width, uint16_t *height,
                       uint8_t *format, uint16_t *payload_len)
{
    (void)payload_len;

    if (!in_buf || in_len < UART_FRAME_HEADER_SIZE)
    {
        return -2;
    }

    const uart_frame_header_t *hdr = (const uart_frame_header_t *)in_buf;

    /* Verify magic */
    if (hdr->magic[0] != UART_FRAME_MAGIC_0 ||
        hdr->magic[1] != UART_FRAME_MAGIC_1 ||
        hdr->magic[2] != UART_FRAME_MAGIC_2 ||
        hdr->magic[3] != UART_FRAME_MAGIC_3)
    {
        return -1;  /* Invalid magic */
    }

    /* Extract fields */
    if (width)
    {
        *width = hdr->width;
    }
    if (height)
    {
        *height = hdr->height;
    }
    if (format)
    {
        *format = hdr->format;
    }
    /* payload_len must be determined by finding JPEG end marker for JPEG format */

    return 0;
}

/*******************************************************************************
* Function Name: uart_validate_crc
********************************************************************************
* Summary:
*  Validates frame payload integrity by computing CRC16-CCITT and comparing
*  it against the CRC value in the frame header.
*
* Parameters:
*  const uint8_t *in_buf: Input buffer containing full frame
*  uint32_t in_len: Total frame length in bytes
*  uint16_t payload_len: Payload length in bytes
*
* Return:
*  int: 1 if CRC matches, 0 if mismatch, -1 on input error
*
*******************************************************************************/
int uart_validate_crc(const uint8_t *in_buf, uint32_t in_len, uint16_t payload_len)
{
    if (!in_buf || in_len < UART_FRAME_HEADER_SIZE)
    {
        return -1;
    }

    const uart_frame_header_t *hdr = (const uart_frame_header_t *)in_buf;

    if (in_len < (UART_FRAME_HEADER_SIZE + payload_len))
    {
        return -1;
    }

    /* Calculate CRC of payload */
    const uint8_t *payload = in_buf + UART_FRAME_HEADER_SIZE;
    uint16_t calculated_crc = crc16_ccitt_compute(payload, payload_len);

    return (calculated_crc == hdr->crc16) ? 1 : 0;
}

/*******************************************************************************
* Function Name: uart_encode_faceid_metadata
********************************************************************************
* Summary:
*  Encodes Face ID metadata into a UART frame by building header and payload,
*  then appending CRC for payload integrity.
*
* Parameters:
*  uint8_t *out_buf: Output buffer for encoded metadata frame
*  uint32_t out_cap: Output buffer capacity in bytes
*  const uart_faceid_metadata_t *metadata: Face metadata structure
*  uint16_t img_width: Source image width for header metadata
*  uint16_t img_height: Source image height for header metadata
*
* Return:
*  int: Total encoded frame length on success, -1 on error
*
*******************************************************************************/
int uart_encode_faceid_metadata(uint8_t *out_buf, uint32_t out_cap,
                                const uart_faceid_metadata_t *metadata,
                                uint16_t img_width, uint16_t img_height)
{
    if (!out_buf || !metadata)
    {
        return -1;
    }

    /* Calculate payload size: 1 byte (num_faces) + num_faces * sizeof(uart_face_info_t) */
    uint16_t payload_size = 1 + (metadata->num_faces * sizeof(uart_face_info_t));

    if (out_cap < (UART_FRAME_HEADER_SIZE + payload_size))
    {
        return -1;
    }

    /* Build header */
    uart_frame_header_t *hdr = (uart_frame_header_t *)out_buf;
    hdr->magic[0] = UART_FRAME_MAGIC_0;
    hdr->magic[1] = UART_FRAME_MAGIC_1;
    hdr->magic[2] = UART_FRAME_MAGIC_2;
    hdr->magic[3] = UART_FRAME_MAGIC_3;
    hdr->width = img_width;
    hdr->height = img_height;
    hdr->format = UART_FORMAT_FACEID_META;
    hdr->reserved = 0;

    /* Build payload */
    uint8_t *payload = out_buf + UART_FRAME_HEADER_SIZE;
    payload[0] = metadata->num_faces;

    /* Copy face info array */
    if (metadata->num_faces > 0)
    {
        memcpy(payload + 1, metadata->faces, metadata->num_faces * sizeof(uart_face_info_t));
    }

    /* Calculate CRC over payload */
    hdr->crc16 = crc16_ccitt_compute(payload, payload_size);

    return (int)(UART_FRAME_HEADER_SIZE + payload_size);
}
