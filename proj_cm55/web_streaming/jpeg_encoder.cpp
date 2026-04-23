/*******************************************************************************
 * File Name: jpeg_encoder.cpp
 *
 * Description:
 *   JPEG encoding wrapper implementation.
 *   CRITICAL: Uses static JPEGENC instance to avoid 3KB stack allocation.
 *
 ******************************************************************************
 * Copyright 2023-2025, Infineon Technologies AG
 ******************************************************************************/

#include "jpeg_encoder.h"
#include "JPEGENC.h"
#include <string.h>

/*******************************************************************************
 * Static Variables
 ******************************************************************************/

/* CRITICAL: Static JPEGENC instance (3172 bytes in .bss, NOT on stack!)
 * This avoids stack overflow in the UART task. */
static JPEGENC g_jpeg_encoder;

/*******************************************************************************
 * Function Implementations
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
                      uint32_t *out_size)
{
    /* Validate parameters */
    if (!rgb565 || !out_buf)
    {
        return -1;
    }

    if (out_cap < 1024)
    {
        return -1;  /* Output buffer too small */
    }

    if (quality > JPEGE_Q_LOW)
    {
        quality = JPEGE_Q_LOW;
    }

    if (subsample > JPEGE_SUBSAMPLE_420)
    {
        subsample = JPEGE_SUBSAMPLE_420;
    }

    /* Open encoder to output buffer (resets internal state) */
    int result = g_jpeg_encoder.open(out_buf, (int)out_cap);
    if (result != JPEGE_SUCCESS)
    {
        return -2;  /* Encoder open failed */
    }

    /* Begin encoding with specified parameters */
    JPEGENCODE ctx;
    result = g_jpeg_encoder.encodeBegin(&ctx, width, height,
                                       JPEGE_PIXEL_RGB565,
                                       subsample,
                                       quality);
    if (result != JPEGE_SUCCESS)
    {
        return -3;  /* Encode begin failed */
    }

    /* Add frame data (single pass for static images) */
    int pitch = width * 2;  /* RGB565 = 2 bytes per pixel */
    result = g_jpeg_encoder.addFrame(&ctx, (uint8_t*)rgb565, pitch);
    if (result != JPEGE_SUCCESS)
    {
        return -4;  /* Add frame failed */
    }

    /* Close encoder and get compressed size */
    int jpeg_size = g_jpeg_encoder.close();
    if (jpeg_size <= 0 || jpeg_size > (int)out_cap)
    {
        return -5;  /* Close failed or output too large */
    }

    /* Return compressed size */
    if (out_size)
    {
        *out_size = (uint32_t)jpeg_size;
    }

    return jpeg_size;
}
