/*******************************************************************************
 * File Name: jpeg_encoder.h
 *
 * Description:
 *   Wrapper functions for JPEG encoding of RGB565 frames.
 *   Uses static JPEGENC instance to avoid stack overflow.
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

#ifndef JPEG_ENCODER_H
#define JPEG_ENCODER_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
 * Constants
 ******************************************************************************/

/* JPEG quality levels (0=best, 3=low) */
#define JPEG_QUALITY_BEST    (0u)
#define JPEG_QUALITY_HIGH    (1u)
#define JPEG_QUALITY_MEDIUM  (2u)
#define JPEG_QUALITY_LOW     (3u)

/* JPEG subsampling modes */
#define JPEG_SUBSAMPLE_444   (0u)  /* No chroma subsampling (best quality) */
#define JPEG_SUBSAMPLE_420   (1u)  /* 4:2:0 chroma subsampling (typical) */

/*******************************************************************************
 * Function Prototypes
 ******************************************************************************/

/*******************************************************************************
* Function Name: jpeg_encode_rgb565
********************************************************************************
* Summary:
* Encodes a raw RGB565 frame into a JPEG image using the static JPEGENC
* encoder instance. The function opens the encoder, configures the image
* dimensions, pixel format, chroma subsampling, and quality, submits the
* full frame in a single pass, and returns the compressed JPEG byte count.
*
* Parameters:
*  rgb565   : Pointer to the source RGB565 frame buffer (width * height * 2 bytes)
*  width    : Frame width in pixels
*  height   : Frame height in pixels
*  quality  : JPEG quality level (JPEGE_Q_HIGH / JPEGE_Q_MED / JPEGE_Q_LOW)
*  subsample: Chroma subsampling mode (JPEGE_SUBSAMPLE_444 / JPEGE_SUBSAMPLE_420)
*  out_buf  : Pointer to the output buffer that receives the JPEG bitstream
*  out_cap  : Capacity of out_buf in bytes (must be >= 1024)
*  out_size : Optional pointer to receive the compressed JPEG size in bytes;
*             may be NULL if the size is not needed by the caller
*
* Return:
*  > 0 : Compressed JPEG size in bytes (success)
*  -1  : Invalid input pointer or output buffer too small (< 1024 bytes)
*  -2  : Encoder open failed
*  -3  : encodeBegin failed
*  -4  : addFrame failed
*  -5  : Encoder close failed or output size exceeds out_cap
*
*******************************************************************************/
int jpeg_encode_rgb565(const uint16_t *rgb565,
                      uint16_t width, uint16_t height,
                      uint8_t quality, uint8_t subsample,
                      uint8_t *out_buf, uint32_t out_cap,
                      uint32_t *out_size);

#ifdef __cplusplus
}
#endif

#endif /* JPEG_ENCODER_H */
