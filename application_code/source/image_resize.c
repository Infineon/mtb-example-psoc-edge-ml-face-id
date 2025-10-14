/******************************************************************************
* File Name        : image_resize.c
*
* Description      : This file implements the resize image functions.
*
* Related Document : See README.md
*
*******************************************************************************
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
******************************************************************************/

/******************************************************************************
* Header Files
******************************************************************************/

#include <math.h>
#include "cy_syslib.h"
#include "ifx_image_utils.h"

/******************************************************************************
* Functions
******************************************************************************/

/******************************************************************************
* Function Name: image_resize_calc_map_Ratio
*******************************************************************************
* Summary:
*  Constructs a mapping from destination pixel X-position to source pixel
*  X-position for image resizing, using nearest-neighbor interpolation. 
*  Calculates a uniform scale to maintain the width-to-height ratio, ensuring 
*  only down-scaling is performed.
*
* Parameters:
*  int srcWidth: Width of the source image
*  int srcHeight: Height of the source image
*  int channels: Number of image channels
*  int dstWidth: Width of the destination image
*  int dstHeight: Height of the destination image
*  int *max_x: Pointer to store the maximum valid X-index in the target
*  int16_t src_x_map[]: Array to store the mapping from destination to source
*                       X-positions
*
* Return:
*  float: Uniform scale factor (dxy) to maintain the width-to-height ratio
*
******************************************************************************/
inline static float image_resize_calc_map_Ratio( int srcWidth, int srcHeight, int channels,
        int dstWidth, int dstHeight,
        int *max_x, int16_t src_x_map[] )
{
    float   dx  = (float)srcWidth  / (float)dstWidth;
    float   dy  = (float)srcHeight / (float)dstHeight;
    float    dxy = max( dx, dy );        /* uniform-scale to keep the width-height ratio */

    /* do not allow to enlarge a small image */
    dxy = max( dxy, 1.0f );

    float    src_x_f = 0.5f * dxy + 0.5f;    /* with round */
    int        j;

    for ( j = 0; j < dstWidth  &&  (int)(src_x_f) < srcWidth; j++, src_x_f += dxy ) {
        src_x_map[j] = (int)(src_x_f) * channels;
    }
    *max_x = j;

    return dxy;
}


/******************************************************************************
* Function Name: image_resize_Ratio
*******************************************************************************
* Summary:
*  Resizes an entire image while maintaining the width-to-height ratio using
*  nearest-neighbor interpolation. Supports both uint8_t and int8_t images via
*  casting. Extra destination rows or columns are filled with a padding value.
*  Only down-scaling is allowed.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer
*  int srcWidth: Width of the source image
*  int srcHeight: Height of the source image
*  int channels: Number of image channels
*  uint8_t *dstImage: Pointer to the destination image buffer
*  int dstWidth: Width of the destination image
*  int dstHeight: Height of the destination image
*  float *scaledWidth: Pointer to store the width scale factor
*  float *scaledHeight: Pointer to store the height scale factor
*  int pad_val: Padding value for unoccupied areas
*
* Return:
*  void
*
******************************************************************************/
static void image_resize_Ratio( uint8_t* srcImage, int srcWidth, int srcHeight, int channels,
        uint8_t* dstImage, int dstWidth, int dstHeight,
        float *scaledWidth, float *scaledHeight, int pad_val )
{
    int     srcLineWidth = channels * srcWidth;
    int     dstLineWidth = channels * dstWidth;
    int        max_j;
    int16_t    src_j_map[dstWidth];

    float    dxy = image_resize_calc_map_Ratio( srcWidth, srcHeight, channels, dstWidth, dstHeight, &max_j, src_j_map );

    /* scale from [0, 1] to the source image space */
    if (scaledWidth) {
        *scaledWidth  = floorf(dstWidth  * dxy + 0.5f);
        *scaledHeight = floorf(dstHeight * dxy + 0.5f);
    }

    /* initialize the output image with padding value
       if the ratios of the source and destination images are different */
    if ( srcHeight * dstWidth != srcWidth * dstHeight )
        memset( (void *)dstImage, pad_val, dstHeight * dstWidth * channels );

    float    src_i_f = 0.5f * dxy + 0.5f;    /* with round */

    if (channels == 1) {
        for ( int i = 0; i < dstHeight  &&  (int)(src_i_f) < srcHeight; i++, src_i_f += dxy ) {
            int        src_i   = (int)(src_i_f);
            uint8_t    *srcBlk = &srcImage[src_i * srcLineWidth];
            uint8_t    *pDst   = &dstImage[    i * dstLineWidth];

            for ( int j = 0; j < max_j; j++ ) {
                *pDst++ = srcBlk[src_j_map[j]];
            }
        }
    } else if (channels == 3) {
        for ( int i = 0; i < dstHeight  &&  (int)(src_i_f) < srcHeight; i++, src_i_f += dxy ) {
            int        src_i   = (int)(src_i_f);
            uint8_t    *srcBlk = &srcImage[src_i * srcLineWidth];
            uint8_t    *pDst   = &dstImage[    i * dstLineWidth];

            for ( int j = 0; j < max_j; j++ ) {
                uint8_t    *pSrc = &srcBlk[src_j_map[j]];

                // memcpy( (void *)&dstImage[dstIdx + j * 3], (void *)&srcImage[srcIdx + src_j * 3], 3 );
                *pDst++ = *pSrc++;
                *pDst++ = *pSrc++;
                *pDst++ = *pSrc;
            }
        }
    }
}


/******************************************************************************
* Function Name: ifx_image_resize_Ratio
*******************************************************************************
* Summary:
*  Resizes an entire uint8_t image while maintaining the width-to-height ratio
*  using nearest-neighbor interpolation. Extra destination rows or columns are
*  filled with a padding value of 128. Only down-scaling is allowed.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t srcWidth: Width of the source image
*  int32_t srcHeight: Height of the source image
*  int32_t channels: Number of image channels
*  uint8_t *dstImage: Pointer to the destination image buffer (uint8_t)
*  int32_t dstWidth: Width of the destination image
*  int32_t dstHeight: Height of the destination image
*  float *scaledWidth: Pointer to store the width scale factor
*  float *scaledHeight: Pointer to store the height scale factor
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Ratio( uint8_t* srcImage, int32_t srcWidth, int32_t srcHeight, int32_t channels,
        uint8_t* dstImage, int32_t dstWidth, int32_t dstHeight,
        float *scaledWidth, float *scaledHeight )
{
    image_resize_Ratio( srcImage, srcWidth, srcHeight, channels,
            dstImage, dstWidth, dstHeight, scaledWidth, scaledHeight, 128 );
}


/******************************************************************************
* Function Name: ifx_image_resize_Ratio_i8
*******************************************************************************
* Summary:
*  Resizes an entire int8_t image while maintaining the width-to-height ratio
*  using nearest-neighbor interpolation. Extra destination rows or columns are
*  filled with a padding value of 0. Only down-scaling is allowed.
*
* Parameters:
*  int8_t *srcImage: Pointer to the source image buffer (int8_t)
*  int32_t srcWidth: Width of the source image
*  int32_t srcHeight: Height of the source image
*  int32_t channels: Number of image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dstWidth: Width of the destination image
*  int32_t dstHeight: Height of the destination image
*  float *scaledWidth: Pointer to store the width scale factor
*  float *scaledHeight: Pointer to store the height scale factor
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Ratio_i8( int8_t* srcImage, int32_t srcWidth, int32_t srcHeight, int32_t channels,
        int8_t* dstImage, int32_t dstWidth, int32_t dstHeight,
        float *scaledWidth, float *scaledHeight )
{
    image_resize_Ratio( (uint8_t *)srcImage, srcWidth, srcHeight, channels,
            (uint8_t *)dstImage, dstWidth, dstHeight, scaledWidth, scaledHeight, 0 );
}


CY_SECTION_ITCM_BEGIN
/******************************************************************************
* Function Name: ifx_image_resize_Ratio_u2i
*******************************************************************************
* Summary:
*  Resizes an entire image from uint8_t to int8_t while maintaining the
*  width-to-height ratio using nearest-neighbor interpolation. Applies a
*  zero-point offset to the output. Extra destination rows or columns are 
*  filled with a padding value of 0. Only down-scaling is allowed.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t srcWidth: Width of the source image
*  int32_t srcHeight: Height of the source image
*  int32_t channels: Number of image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dstWidth: Width of the destination image
*  int32_t dstHeight: Height of the destination image
*  float *scaledWidth: Pointer to store the width scale factor
*  float *scaledHeight: Pointer to store the height scale factor
*  int32_t zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Ratio_u2i( uint8_t* srcImage, int32_t srcWidth, int32_t srcHeight, int32_t channels,
        int8_t*  dstImage, int32_t dstWidth, int32_t dstHeight,
        float *scaledWidth, float *scaledHeight, int32_t zero_point )
{
    int     srcLineWidth = channels * srcWidth;
    int     dstLineWidth = channels * dstWidth;
    int        max_j;
    int16_t    src_j_map[dstWidth];

    float    dxy = image_resize_calc_map_Ratio( srcWidth, srcHeight, channels, dstWidth, dstHeight, &max_j, src_j_map );

    /* scale from [0, 1] to the resized image size */
    if (scaledWidth) {
        *scaledWidth  = floorf(dstWidth  * dxy + 0.5f);
        *scaledHeight = floorf(dstHeight * dxy + 0.5f);
    }

    /* initialize the output image with the gray color, 0 */
    /* if the ratios of the source and destination images are different */
    if ( srcHeight * dstWidth != srcWidth * dstHeight )
        memset( (void *)dstImage, 0, dstHeight * dstWidth * channels );

    float    src_i_f = 0.5f * dxy + 0.5f;    /* with round */
    int        min_Height = (srcHeight / dxy) + 0.5f;

    min_Height = min(dstHeight, min_Height);

    if (channels == 1) {
        for ( int i = 0; i < min_Height; i++, src_i_f += dxy ) {
            int        src_i   = (int)(src_i_f);
            uint8_t    *srcBlk = &srcImage[src_i * srcLineWidth];
            int8_t    *pDst   = &dstImage[    i * dstLineWidth];

            for ( int j = 0; j < max_j; j++ ) {
                *pDst++ = (int8_t)(srcBlk[src_j_map[j]] + zero_point);
            }
        }
    } else if (channels == 3) {
        for ( int i = 0; i < min_Height; i++, src_i_f += dxy ) {
            int        src_i   = (int)(src_i_f);
            uint8_t    *srcBlk = &srcImage[src_i * srcLineWidth];
            int8_t    *pDst   = &dstImage[    i * dstLineWidth];

            for ( int j = 0; j < max_j; j++ ) {
                uint8_t    *pSrc = &srcBlk[src_j_map[j]];

                *pDst++ = (int8_t)(*pSrc++ + zero_point);
                *pDst++ = (int8_t)(*pSrc++ + zero_point);
                *pDst++ = (int8_t)(*pSrc   + zero_point);
            }
        }
    }
}
CY_SECTION_ITCM_END


/******************************************************************************
 * Function Name: image_resize_calc_map_Ratio_i32
*******************************************************************************
* Summary:
*  Constructs a mapping from destination pixel X-position to source pixel
*  X-position for image resizing using integer arithmetic (q15.16 format) and
*  nearest-neighbor interpolation. Calculates a uniform scale to maintain the
*  width-to-height ratio, ensuring only down-scaling is performed.
*
* Parameters:
*  int srcWidth: Width of the source image
*  int srcHeight: Height of the source image
*  int channels: Number of image channels
*  int dstWidth: Width of the destination image
*  int dstHeight: Height of the destination image
*  int *max_x: Pointer to store the maximum valid X-index in the target
*  int16_t src_x_map[]: Array to store the mapping from destination to source 
*                       X-positions
*
* Return:
*  int32_t: Uniform scale factor (dxy) in q15.16 format
*
******************************************************************************/
inline static int32_t image_resize_calc_map_Ratio_i32( int srcWidth, int srcHeight, int channels,
        int dstWidth, int dstHeight,
        int *max_x, int16_t src_x_map[] )
{
    int32_t    dx  = ((srcWidth  << 16) + (dstWidth  >> 1)) / dstWidth;    /* q15.16 */
    int32_t    dy  = ((srcHeight << 16) + (dstHeight >> 1)) / dstHeight;    /* q15.16 */
    int32_t    dxy = max( dx, dy );        /* uniform-scale to keep the width-height ratio */

    /* do not allow to enlarge a small image */
    dxy = max( dxy, (1 << 16) );

    int32_t    src_x_f = (dxy >> 1) + (1 << 15);        /* q15.16 with round */
    int        j;
    for ( j = 0; j < dstWidth  &&  (src_x_f >> 16) < srcWidth; j++, src_x_f += dxy ) {
        src_x_map[j] = (src_x_f >> 16) * channels;        /* int <= q15.16 */
    }
    *max_x = j;

    return dxy;
}


/******************************************************************************
* Function Name: ifx_image_resize_Ratio_u2i_i32
*******************************************************************************
* Summary:
*  Resizes an entire image from uint8_t to int8_t while maintaining the
*  width-to-height ratio using nearest-neighbor interpolation and integer
*  arithmetic (q15.16 format). Applies a zero-point offset to the output. Extra
*  destination rows or columns are filled with a padding value of 0. Only
*  down-scaling is allowed.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t srcWidth: Width of the source image
*  int32_t srcHeight: Height of the source image
*  int32_t channels: Number of image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dstWidth: Width of the destination image
*  int32_t dstHeight: Height of the destination image
*  float *scaledWidth: Pointer to store the width scale factor
*  float *scaledHeight: Pointer to store the height scale factor
*  int32_t zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Ratio_u2i_i32( uint8_t* srcImage, int32_t srcWidth, int32_t srcHeight, int32_t channels,
        int8_t*  dstImage, int32_t dstWidth, int32_t dstHeight,
        float *scaledWidth, float *scaledHeight, int32_t zero_point )
{
    int     srcLineWidth = channels * srcWidth;
    int     dstLineWidth = channels * dstWidth;
    int        max_j;
    int16_t    src_j_map[dstWidth];

    /* q15.16 for float */
    int32_t    dxy = image_resize_calc_map_Ratio_i32( srcWidth, srcHeight, channels, dstWidth, dstHeight, &max_j, src_j_map );

    /* scale from [0, 1] to the resized image size */
    if (scaledWidth) {
        *scaledWidth  = (dstWidth  * dxy + (1 << 15)) >> 16;
        *scaledHeight = (dstHeight * dxy + (1 << 15)) >> 16;
    }

    /* initialize the output image with the gray color, 0 */
    if ( srcHeight * dstWidth != srcWidth * dstHeight )
        memset( (void *)dstImage, 0, dstHeight * dstWidth * channels );

    int32_t    src_i_f = (dxy >> 1) + (1 << 15);        /* q15.16 with round */
    int        min_Height = (srcHeight << 16) / dxy;    /* q15.16 / q15.16 => int */

    min_Height = min(dstHeight, min_Height);

    if (channels == 1) {
        for ( int i = 0; i < min_Height; i++, src_i_f += dxy ) {
            int        src_i   = src_i_f >> 16;                /* int <= q15.16 */
            uint8_t    *srcBlk = &srcImage[src_i * srcLineWidth];
            int8_t    *pDst   = &dstImage[    i * dstLineWidth];

            for ( int j = 0; j < max_j; j++ ) {
                *pDst++ = (int8_t)(srcBlk[src_j_map[j]] + zero_point);
            }
        }
    } else if (channels == 3) {
        for ( int i = 0; i < min_Height; i++, src_i_f += dxy ) {
            int        src_i   = src_i_f >> 16;                /* int <= q15.16 */
            uint8_t    *srcBlk = &srcImage[src_i * srcLineWidth];
            int8_t    *pDst   = &dstImage[    i * dstLineWidth];

            for ( int j = 0; j < max_j; j++ ) {
                uint8_t    *pSrc = &srcBlk[src_j_map[j]];

                *pDst++ = (int8_t)(*pSrc++ + zero_point);
                *pDst++ = (int8_t)(*pSrc++ + zero_point);
                *pDst++ = (int8_t)(*pSrc   + zero_point);
            }
        }
    }
}


/******************************************************************************
* Function Name: ifx_image_resize_RGB565_to_RGB888_Ratio
*******************************************************************************
* Summary:
*  Resizes an entire image from RGB565 format (16 bits per pixel) to RGB888
*  format (24 bits per pixel) while maintaining the width-to-height ratio using
*  nearest-neighbor interpolation. Extra destination rows or columns are filled
*  with a specified padding value. Only down-scaling is allowed.
*
* Parameters:
*  uint16_t *srcImage_RGB565: Pointer to the source RGB565 image buffer
*  int32_t srcWidth: Width of the source image
*  int32_t srcHeight: Height of the source image
*  uint8_t *dstImage_RGB888: Pointer to the destination RGB888 image buffer
*  int32_t dstWidth: Width of the destination image
*  int32_t dstHeight: Height of the destination image
*  float *scaledWidth: Pointer to store the width scale factor
*  float *scaledHeight: Pointer to store the height scale factor
*  int32_t pad_val: Padding value for unoccupied areas
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_RGB565_to_RGB888_Ratio( uint16_t *srcImage_RGB565, int32_t srcWidth, int32_t srcHeight,
        uint8_t  *dstImage_RGB888, int32_t dstWidth, int32_t dstHeight,
        float *scaledWidth, float *scaledHeight, int32_t pad_val )
{
    int     srcLineWidth = srcWidth;
    int     dstLineWidth = 3 * dstWidth;
    int        max_j;
    int16_t    src_j_map[dstWidth];

    float    dxy = image_resize_calc_map_Ratio( srcWidth, srcHeight, 1, dstWidth, dstHeight, &max_j, src_j_map );

    /* scale from [0, 1] to the source image space */
    if (scaledWidth) {
        *scaledWidth  = floorf(dstWidth  * dxy + 0.5f);
        *scaledHeight = floorf(dstHeight * dxy + 0.5f);
    }

    if ( srcHeight * dstWidth != srcWidth * dstHeight )
        memset( (void *)dstImage_RGB888, pad_val, dstHeight * dstWidth * 3 );

    float    src_i_f = 0.5f * dxy + 0.5f;    /* with round */

    for ( int i = 0; i < dstHeight  &&  (int)(src_i_f) < srcHeight; i++, src_i_f += dxy ) {
        int            src_i   = (int)(src_i_f);
        uint16_t    *srcBlk = &srcImage_RGB565[src_i * srcLineWidth];
        uint8_t        *pDst   = &dstImage_RGB888[    i * dstLineWidth];

        for ( int j = 0; j < max_j; j++ ) {
            pDst = ifx_pixel_RGB565_to_RGB888( srcBlk[src_j_map[j]], pDst );
        }
    }
}


/******************************************************************************
* Function Name: image_resize_calc_map_Rect
*******************************************************************************
* Summary:
*  Constructs a mapping from destination pixel X-position to source pixel
*  X-position for resizing a rectangular region of an image, using nearest-neighbor
*  interpolation. Handles protrusive bounding boxes and ensures the mapping stays
*  within the source image boundaries.
*
* Parameters:
*  int src_w: Width of the source image
*  int dst_w: Width of the destination image
*  int src_ch: Number of source image channels
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  int *min_x: Pointer to store the minimum valid X-index in the target
*  int *max_x: Pointer to store the maximum valid X-index in the target
*  int16_t src_x_map[]: Array to store the mapping from destination to source 
*                       X-positions
*
* Return:
*  void
*
******************************************************************************/
inline static void image_resize_calc_map_Rect( int src_w, int dst_w, int src_ch, float xmin, float xmax,
        int *min_x, int *max_x, int16_t src_x_map[] )
{
    float   rect_w = xmax - xmin;
    float   step_x = rect_w / dst_w;
    float    src_j_f = xmin + step_x * 0.5f + 0.5f;    /* float target X position with round */
    int        j;

    /* lower boundary: skip outside of left boundary of the source image */
    for ( j = 0; src_j_f < 0  &&  j < dst_w; j++, src_j_f += step_x );
    *min_x = j;

    /* upper boundary */
    for ( ; j < dst_w  &&  (int)(src_j_f) < src_w; j++, src_j_f += step_x ) {
        src_x_map[j] = (int)(src_j_f) * src_ch;
    }
    *max_x = j;
}


/******************************************************************************
* Function Name: image_resize_Rect
*******************************************************************************
* Summary:
*  Resizes a rectangular region of the source image using nearest-neighbor
*  interpolation. Supports both uint8_t and int8_t images via casting. Extra
*  destination rows or columns are filled with a padding value. Handles protrusive
*  bounding boxes.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer
*  int src_w: Width of the source image
*  int src_h: Height of the source image
*  int src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer
*  int dst_w: Width of the destination image
*  int dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*  int pad_val: Padding value for unoccupied areas
*
* Return:
*  void
*
******************************************************************************/
static void image_resize_Rect( uint8_t *srcImage, int src_w, int src_h, int src_ch,
        uint8_t *dstImage, int dst_w, int dst_h,
        float xmin, float ymin, float xmax, float ymax, int pad_val )
{
    int     srcLineWidth = src_ch * src_w;
    int     dstLineWidth = src_ch * dst_w;
    float   rect_h = ymax - ymin;
    float   step_y = rect_h / dst_h;
    float   src_y_f = ymin + step_y * 0.5f + 0.5f;    /* float target Y position with round */
    int        min_j, max_j, y;
    int16_t    src_j_map[dst_w];

    /* initialize the output image with padding value */
    memset( (void *)dstImage, pad_val, dst_w * dst_h * src_ch );

    image_resize_calc_map_Rect( src_w, dst_w, src_ch, xmin, xmax, &min_j, &max_j, src_j_map );

    /* skip outside of the source image, if any */
    for ( y = 0; src_y_f < 0  &&  y < dst_h; y++, src_y_f += step_y );

    if (src_ch == 1) {
        for ( ; y < dst_h  &&  (int)(src_y_f) < src_h; y++, src_y_f += step_y ) {
            int        src_y   = (int)(src_y_f);        /* Y coordinate of the upper neighbors */
            uint8_t    *srcBlk = &srcImage[srcLineWidth * src_y];
            uint8_t    *pDst   = &dstImage[dstLineWidth * y + min_j];

            for ( int x = min_j; x < max_j; x++ ) {
                *pDst++ = srcBlk[src_j_map[x]];
            }
        }
    } else if (src_ch == 3) {
        for ( ; y < dst_h  &&  (int)(src_y_f) < src_h; y++, src_y_f += step_y ) {
            int        src_y   = (int)(src_y_f);        /* Y coordinate of the upper neighbors */
            uint8_t    *srcBlk = &srcImage[srcLineWidth * src_y];
            uint8_t    *pDst   = &dstImage[dstLineWidth * y + min_j * 3];

            for ( int x = min_j; x < max_j; x++ ) {
                uint8_t    *pSrc = &srcBlk[src_j_map[x]];

                *pDst++ = *pSrc++;
                *pDst++ = *pSrc++;
                *pDst++ = *pSrc;
            }
        }
    }
}


/******************************************************************************
* Function Name: ifx_image_resize_Rect
*******************************************************************************
* Summary:
*  Resizes a rectangular region of a uint8_t source image using nearest-
*  neighbor interpolation. Extra destination rows or columns are filled with
*  a paddingvalue of 128. Handles protrusive bounding boxes.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer (uint8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Rect( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        uint8_t *dstImage, int32_t dst_w, int32_t dst_h,
        float xmin, float ymin, float xmax, float ymax )
{
    image_resize_Rect( srcImage, src_w, src_h, src_ch,
            dstImage, dst_w, dst_h, xmin, ymin, xmax, ymax, 128 );
}


/******************************************************************************
* Function Name: ifx_image_resize_Rect_i8
*******************************************************************************
* Summary:
*  Resizes a rectangular region of an int8_t source image using nearest-
*  neighbor interpolation. Extra destination rows or columns are filled with 
*  a padding value of 0. Handles protrusive bounding boxes.
*
* Parameters:
*  int8_t *srcImage: Pointer to the source image buffer (int8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Rect_i8( int8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t *dstImage, int32_t dst_w, int32_t dst_h,
        float xmin, float ymin, float xmax, float ymax )
{
    image_resize_Rect( (uint8_t *)srcImage, src_w, src_h, src_ch,
            (uint8_t *)dstImage, dst_w, dst_h, xmin, ymin, xmax, ymax, 0 );
}


/******************************************************************************
* Function Name: ifx_image_resize_Rect_u2i
*******************************************************************************
* Summary:
*  Resizes a rectangular region of a uint8_t source image to an int8_t
*  destination image using nearest-neighbor interpolation. Applies a zero-point
*  offset to the output. Extra destination rows or columns are filled with a
*  padding value of 0. Handles protrusive bounding boxes.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*  int32_t zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Rect_u2i( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t  *dstImage, int32_t dst_w, int32_t dst_h,
        float xmin, float ymin, float xmax, float ymax, int32_t zero_point )
{
    int     srcLineWidth = src_ch * src_w;
    int     dstLineWidth = src_ch * dst_w;
    float   rect_h = ymax - ymin;
    float   step_y = rect_h / dst_h;
    float   src_y_f = ymin + step_y * 0.5f + 0.5f;        /* float target Y position */
    int        min_j, max_j, y;
    int16_t    src_j_map[dst_w];

    memset( (void *)dstImage, 0, dst_w * dst_h * src_ch );    /* initialize to mid-value of uint8 */

    image_resize_calc_map_Rect( src_w, dst_w, src_ch, xmin, xmax, &min_j, &max_j, src_j_map );

    /* skip outside of the source image, if any */
    for ( y = 0; src_y_f < 0  &&  y < dst_h; y++, src_y_f += step_y );

    if ( src_ch == 1 ) {
        for ( ; y < dst_h  &&  (int)(src_y_f) < src_h; y++, src_y_f += step_y ) {
            int     src_y = (int)(src_y_f);        /* Y coordinate of the upper neighbors */
            int8_t    *pDst = &dstImage[INDEX2D(y, min_j, dst_w)];
            for ( int x = min_j; x < max_j; x++ ) {
                int src_x = src_j_map[x];       /* X coordinate of left neighbors */

                *pDst++ = (int8_t)(srcImage[INDEX2D(src_y, src_x, src_w)] + zero_point);
            }
        }
    } else if ( src_ch == 3 ) {
        for ( ; y < dst_h  &&  (int)(src_y_f) < src_h; y++, src_y_f += step_y ) {
            int     src_y   = (int)(src_y_f);        /* Y coordinate of the upper neighbors */
            uint8_t    *srcBlk = &srcImage[srcLineWidth * src_y];
            int8_t    *pDst   = &dstImage[dstLineWidth * y + min_j * 3];

            for ( int x = min_j; x < max_j; x++ ) {
                uint8_t    *pSrc = &srcBlk[src_j_map[x]];

                *pDst++ = (int8_t)(*pSrc++ + zero_point);
                *pDst++ = (int8_t)(*pSrc++ + zero_point);
                *pDst++ = (int8_t)(*pSrc   + zero_point);
            }
        }
    }
}


/******************************************************************************
* Function Name: image_resize_calc_map_Rect_Linear
*******************************************************************************
* Summary:
*  Constructs a mapping from destination pixel X-position to source pixel
*  X-position for resizing a rectangular region of an image, using bilinear
*  interpolation. Calculates weights for left and right neighbors and handles
*  protrusive bounding boxes.
*
* Parameters:
*  int src_w: Width of the source image
*  int dst_w: Width of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  int *min_x: Pointer to store the minimum valid X-index in the target
*  int *max_x: Pointer to store the maximum valid X-index in the target
*  float fx_0[]: Array to store weights for right neighbors
*  float fx_1[]: Array to store weights for left neighbors
*  int16_t src_x_map[]: Array to store the mapping from destination to source 
*                       X-positions
*
* Return:
*  void
*
******************************************************************************/
static void image_resize_calc_map_Rect_Linear( int src_w, int dst_w, float xmin, float xmax,
        int *min_x, int *max_x,
        float fx_0[], float fx_1[], int16_t src_x_map[] )
{
    float   rect_w = xmax - xmin;
    float   step_x = rect_w / dst_w;
    float    src_x_f = xmin + step_x * 0.5f;        /* float target X position without round */
    int        x;

    /* lower boundary in X-axis: skip outside of left boundary of the source image */
    for ( x = 0; x < dst_w  &&  src_x_f < 0; x++, src_x_f += step_x );
    *min_x = x;

    /* upper boundary in X-axis */
    for ( ; x < dst_w  &&  (int)src_x_f < (src_w - 1); x++, src_x_f += step_x ) {
        int src_x = (int)src_x_f;        /* X coordinate of left neighbors */

        fx_0[x] = src_x_f - src_x;        /* weight for the right neighbors: (src_x + 1, y) */
        fx_1[x] = 1.0f - fx_0[x];        /* weight for the left neighbors: (src_x, y) */
        src_x_map[x] = src_x;            /* index of the left neighbor */
    }
    *max_x = x;
}


/******************************************************************************
* Function Name: ifx_image_resize_Rect_Linear
*******************************************************************************
* Summary:
*  Resizes a rectangular region of a uint8_t source image using bilinear
*  interpolation. Extra destination rows or columns are filled with a padding
*  value of 128. Handles protrusive bounding boxes.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer (uint8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Rect_Linear( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        uint8_t *dstImage, int32_t dst_w, int32_t dst_h,
        float xmin, float ymin, float xmax, float ymax )
{
    float   rect_h = ymax - ymin;
    float   step_y = rect_h / dst_h;
    int        min_x, max_x;              /* low, high boundary in X axis */
    int16_t    src_x_map[dst_w];          /* X-location map from destination to source */
    float    fx_0[dst_w], fx_1[dst_w];    /* weights for two X-neighbors of a target position */
    float    src_y_f = ymin + step_y * 0.5f;
    int        y;

    /* initialize to mid-value of uint8 */
    memset( (void *)dstImage, 128, dst_w * dst_h * src_ch );

    /* construct a map from dest-pixel-x-position to src-pixel-x-position */
    image_resize_calc_map_Rect_Linear( src_w, dst_w, xmin, xmax,
            &min_x, &max_x, fx_0, fx_1, src_x_map );

    /* skip negative index to the source image */
    for ( y = 0; src_y_f < 0  &&  y < dst_h; y++, src_y_f += step_y );

    for ( ; y < dst_h  &&  (int)src_y_f < (src_h - 1); y++, src_y_f += step_y ) {
        int     src_y = (int)src_y_f;        /* Y coordinate of the upper neighbors */
        float    fy_0  = src_y_f - src_y;    /* weight for the lower neighbors: (x, src_y + 1) */
        float    fy_1  = 1.0f - fy_0;        /* weight for the upper neighbors: (x, src_y) */
        uint8_t    *pDst = &dstImage[INDEX3D(y, min_x, 0, dst_w, src_ch)];

        for ( int x = min_x; x < max_x; x++ ) {
            int        src_x = src_x_map[x];        /* X coordinate of left neighbors */
            float    w00 = fy_1 * fx_1[x];
            float    w01 = fy_1 * fx_0[x];
            float    w10 = fy_0 * fx_1[x];
            float    w11 = fy_0 * fx_0[x];

            for ( int c = 0; c < src_ch; c++ ) {
                /* weighted sum of the four neighbors of the actual source position */
                *pDst++ = (uint8_t)( srcImage[INDEX3D(src_y,     src_x,     c, src_w, src_ch)] * w00
                        + srcImage[INDEX3D(src_y,     src_x + 1, c, src_w, src_ch)] * w01
                        + srcImage[INDEX3D(src_y + 1, src_x,     c, src_w, src_ch)] * w10
                        + srcImage[INDEX3D(src_y + 1, src_x + 1, c, src_w, src_ch)] * w11 + 0.5f );
            }
        }
    }
}


/******************************************************************************
* Function Name: ifx_image_resize_Rect_Linear_i8
*******************************************************************************
* Summary:
*  Resizes a rectangular region of an int8_t source image using bilinear
*  interpolation. Extra destination rows or columns are filled with a padding
*  value of 0. Handles protrusive bounding boxes.
*
* Parameters:
*  int8_t *srcImage: Pointer to the source image buffer (int8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Rect_Linear_i8( int8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t *dstImage, int32_t dst_w, int32_t dst_h,
        float xmin, float ymin, float xmax, float ymax )
{
    float   rect_h = ymax - ymin;
    float   step_y = rect_h / dst_h;
    int        min_x, max_x;              /* low, high boundary in X axis */
    int16_t    src_x_map[dst_w];          /* X-location map from destination to source */
    float    fx_0[dst_w], fx_1[dst_w];    /* weights for two X-neighbors of a target position */
    float    src_y_f = ymin + step_y * 0.5f;
    int        y;
    /* initialize to mid-value of int8 */
    memset( (void *)dstImage, 0, dst_w * dst_h * src_ch );

    /* construct a map from dest-pixel-x-position to src-pixel-x-position */
    image_resize_calc_map_Rect_Linear( src_w, dst_w, xmin, xmax,
            &min_x, &max_x, fx_0, fx_1, src_x_map );

    /* skip negative index to the source image */
    for ( y = 0; src_y_f < 0  &&  y < dst_h; y++, src_y_f += step_y );

    for ( ; y < dst_h  &&  (int)src_y_f < (src_h - 1); y++, src_y_f += step_y ) {
        int        src_y = (int)src_y_f;     /* Y coordinate of the upper neighbors */
        float    fy_0  = src_y_f - src_y;    /* weight for the lower neighbors: (x, src_y + 1) */
        float    fy_1  = 1.0f - fy_0;        /* weight for the upper neighbors: (x, src_y) */
        int8_t    *pDst = &dstImage[INDEX3D(y, min_x, 0, dst_w, src_ch)];

        for ( int x = min_x; x < max_x; x++ ) {
            int        src_x = src_x_map[x];        /* X coordinate of left neighbors */
            float    w00 = fy_1 * fx_1[x];
            float    w01 = fy_1 * fx_0[x];
            float    w10 = fy_0 * fx_1[x];
            float    w11 = fy_0 * fx_0[x];

            for ( int c = 0; c < src_ch; c++ ) {
                /* weighted sum of the four neighbors of the actual source position */
                *pDst++ = (int8_t)( srcImage[INDEX3D(src_y,     src_x,     c, src_w, src_ch)] * w00
                        + srcImage[INDEX3D(src_y,     src_x + 1, c, src_w, src_ch)] * w01
                        + srcImage[INDEX3D(src_y + 1, src_x,     c, src_w, src_ch)] * w10
                        + srcImage[INDEX3D(src_y + 1, src_x + 1, c, src_w, src_ch)] * w11 + 0.5f );
            }
        }
    }
}


/******************************************************************************
* Function Name: ifx_image_resize_Rect_Linear_u2i_org
*******************************************************************************
* Summary:
*  Resizes a rectangular region of a uint8_t source image to an int8_t
*  destination image using bilinear interpolation. Applies a zero-point offset
*  to the output. Extra destination rows or columns are filled with a padding
*  value of 0. Handles protrusive bounding boxes.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*  int32_t zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Rect_Linear_u2i_org( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t  *dstImage, int32_t dst_w, int32_t dst_h,
        float xmin, float ymin, float xmax, float ymax, int32_t zero_point )
{
    float   rect_h = ymax - ymin;
    float   step_y = rect_h / dst_h;
    int        min_x, max_x;              /* low, high boundary in X axis */
    int16_t    src_x_map[dst_w];          /* X-location map from destination to source */
    float    fx_0[dst_w], fx_1[dst_w];    /* weights for two X-neighbors of a target position */
    float    zero_point_f = zero_point + 0.5f;
    float    src_y_f = ymin + step_y * 0.5f;
    int        y;

    /* initialize to mid-value of int8 */
    memset( (void *)dstImage, 0, dst_w * dst_h * src_ch );

    /* construct a map from dest-pixel-x-position to src-pixel-x-position */
    image_resize_calc_map_Rect_Linear( src_w, dst_w, xmin, xmax,
            &min_x, &max_x, fx_0, fx_1, src_x_map );

    /* skip negative index to the source image */
    for ( y = 0; src_y_f < 0  &&  y < dst_h; y++, src_y_f += step_y );

    if ( src_ch == 1 ) {
        for ( ; y < dst_h  &&  (int)src_y_f < (src_h - 1); y++, src_y_f += step_y ) {
            int        src_y = (int)src_y_f;     /* Y coordinate of the upper neighbors */
            float    fy_0  = src_y_f - src_y;    /* weight for the lower neighbors: (x, src_y + 1) */
            float    fy_1  = 1.0f - fy_0;        /* weight for the upper neighbors: (x, src_y) */
            int8_t    *pDst = &dstImage[INDEX2D(y, min_x, dst_w)];

            for ( int x = min_x; x < max_x; x++ ) {
                int        src_x = src_x_map[x];        /* X coordinate of left neighbors */
                float    w00 = fy_1 * fx_1[x];
                float    w01 = fy_1 * fx_0[x];
                float    w10 = fy_0 * fx_1[x];
                float    w11 = fy_0 * fx_0[x];

                /* weighted sum of the four neighbors of the actual source position */
                *pDst++ = (int8_t)( srcImage[INDEX2D(src_y,     src_x,     src_w)] * w00
                        + srcImage[INDEX2D(src_y,     src_x + 1, src_w)] * w01
                        + srcImage[INDEX2D(src_y + 1, src_x,     src_w)] * w10
                        + srcImage[INDEX2D(src_y + 1, src_x + 1, src_w)] * w11 + zero_point_f );
            }
        }
    } else if ( src_ch == 3 ) {
        int     srcLineWidth = src_ch * src_w;
        int     dstLineWidth = src_ch * dst_w;

        for ( ; y < dst_h  &&  (int)src_y_f < (src_h - 1); y++, src_y_f += step_y ) {
            int        src_y = (int)src_y_f;     /* Y coordinate of the upper neighbors */
            float    fy_0  = src_y_f - src_y;    /* weight for the lower neighbors: (x, src_y + 1) */
            float    fy_1  = 1.0f - fy_0;        /* weight for the upper neighbors: (x, src_y) */
            uint8_t    *srcBlk = &srcImage[srcLineWidth * src_y];
            int8_t    *pDst   = &dstImage[dstLineWidth * y + min_x * 3];

            for ( int x = min_x; x < max_x; x++ ) {
                uint8_t    *pSrc = &srcBlk[src_x_map[x] * 3];
                float    w00 = fy_1 * fx_1[x];
                float    w01 = fy_1 * fx_0[x];
                float    w10 = fy_0 * fx_1[x];
                float    w11 = fy_0 * fx_0[x];

                /* weighted sum of the four neighbors of the actual source position */
                *pDst ++ = (int8_t)( pSrc[0]            * w00 + pSrc[3]                   * w01 +
                        pSrc[srcLineWidth] * w10 + pSrc[srcLineWidth + 3] * w11 + zero_point_f );
                pSrc++;        /* move to next channel */
                *pDst ++ = (int8_t)( pSrc[0]            * w00 + pSrc[3]                   * w01 +
                        pSrc[srcLineWidth] * w10 + pSrc[srcLineWidth + 3] * w11 + zero_point_f );
                pSrc++;        /* move to next channel */
                *pDst ++ = (int8_t)( pSrc[0]            * w00 + pSrc[3]                   * w01 +
                        pSrc[srcLineWidth] * w10 + pSrc[srcLineWidth + 3] * w11 + zero_point_f );
            }
        }
    }
}


/******************************************************************************
* Function Name: image_resize_calc_map_Rect_Linear_i32
*******************************************************************************
* Summary:
*  Constructs a mapping from destination pixel X-position to source pixel
*  X-position for resizing a rectangular region of an image, using bilinear
*  interpolation and integer arithmetic (q15.16 format). Calculates weights for
*  left and right neighbors and handles protrusive bounding boxes.
*
* Parameters:
*  int src_w: Width of the source image
*  int dst_w: Width of the destination image
*  int32_t src_ch: Number of source image channels
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  int *min_x: Pointer to store the minimum valid X-index in the target
*  int *max_x: Pointer to store the maximum valid X-index in the target
*  int32_t fx_0[]: Array to store weights for right neighbors (q23.8 format)
*  int32_t fx_1[]: Array to store weights for left neighbors (q23.8 format)
*  int16_t src_x_map[]: Array to store the mapping from destination to source 
*                       X-positions
*
* Return:
*  void
*
******************************************************************************/
static void image_resize_calc_map_Rect_Linear_i32( int src_w, int dst_w, int32_t src_ch, float xmin, float xmax,
        int *min_x, int *max_x,
        int32_t fx_0[], int32_t fx_1[], int16_t src_x_map[] )
{
    int32_t    xmin_i = (int32_t)(xmin * 65536.0f);    /* q15.16 */
    int32_t    xmax_i = (int32_t)(xmax * 65536.0f);    /* q15.16 */
    int32_t    rect_w = xmax_i - xmin_i;
    int32_t    step_x = rect_w / dst_w;                /* q15.16 */
    int32_t    src_x_f = xmin_i + (step_x >> 1);        /* q15.16, float target X position without round */
    int        x;

    /* lower boundary in X-axis: skip outside of left boundary of the source image */
    for ( x = 0; src_x_f < 0  &&  x < dst_w; x++, src_x_f += step_x );
    *min_x = x;

    /* upper boundary in X-axis */
    for ( ; x < dst_w  &&  src_x_f < ((src_w - 1) << 16); x++, src_x_f += step_x ) {
        fx_0[x] = (src_x_f >> 8) & 0x0FF;           /* q23.8, weight for the right neighbors: (src_x + 1, y) */
        fx_1[x] = (1 << 8) - fx_0[x];               /* q23.8, weight for the left neighbors: (src_x, y) */
        src_x_map[x] = (src_x_f >> 16) * src_ch;    /* index of the left neighbor */
    }
    *max_x = x;
}


CY_SECTION_ITCM_BEGIN
/******************************************************************************
* Function Name: ifx_image_resize_Rect_Linear_u2i
*******************************************************************************
* Summary:
*  Resizes a rectangular region of a uint8_t source image to an int8_t
*  destination image using bilinear interpolation and integer arithmetic
*  (q15.16 format). Applies a zero-point offset to the output. Extra destination
*  rows or columns are filled with a padding value of 0. Handles protrusive
*  bounding boxes.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  float xmin: Minimum X-coordinate of the rectangle in the source image
*  float ymin: Minimum Y-coordinate of the rectangle in the source image
*  float xmax: Maximum X-coordinate of the rectangle in the source image
*  float ymax: Maximum Y-coordinate of the rectangle in the source image
*  int32_t zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Rect_Linear_u2i( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t  *dstImage, int32_t dst_w, int32_t dst_h,
        float xmin, float ymin, float xmax, float ymax, int32_t zero_point )
{
    int32_t    ymax_i = (int32_t)(ymax * 65536.0f);    /* q15.16 */
    int32_t    ymin_i = (int32_t)(ymin * 65536.0f);    /* q15.16 */
    int32_t rect_h = ymax_i - ymin_i;
    int32_t step_y = rect_h / dst_h;                    /* q15.16 */
    int        min_x, max_x;                            /* low, high boundary in X axis */
    int16_t    src_x_map[dst_w];                        /* X-location map from destination to source */
    int32_t    fx_0[dst_w], fx_1[dst_w];                /* q23.8, weights for two X-neighbors of a target position */
    int32_t    src_y_f = ymin_i + (step_y >> 1);        /* q15.16, no round */
    int        y;

    /* initialize to mid-value of int8 */
    memset( (void *)dstImage, 0, dst_w * dst_h * src_ch );

    /* construct a map from dest-pixel-x-position to src-pixel-x-position */
    image_resize_calc_map_Rect_Linear_i32( src_w, dst_w, src_ch, xmin, xmax,
            &min_x, &max_x, fx_0, fx_1, src_x_map );

    /* skip negative index to the source image */
    for ( y = 0; src_y_f < 0  &&  y < dst_h; y++, src_y_f += step_y );

    if ( src_ch == 1 ) {
        for ( ; y < dst_h  &&  src_y_f < ((src_h - 1) << 16); y++, src_y_f += step_y ) {
            int        src_y = src_y_f >> 16;           /* Y coordinate of the upper neighbors */
            float    fy_0  = (src_y_f >> 8) & 0x0FF;    /* weight for the lower neighbors: (x, src_y + 1) */
            float    fy_1  = (1 << 8) - fy_0;           /* weight for the upper neighbors: (x, src_y) */
            int8_t    *pDst = &dstImage[INDEX2D(y, min_x, dst_w)];

            for ( int x = min_x; x < max_x; x++ ) {
                int        src_x = src_x_map[x];    /* X coordinate of left neighbors */
                int32_t    w00 = fy_1 * fx_1[x];    /* q23.8 * q23.8 => q15.16 */
                int32_t    w01 = fy_1 * fx_0[x];    /* q23.8 * q23.8 => q15.16 */
                int32_t    w10 = fy_0 * fx_1[x];    /* q23.8 * q23.8 => q15.16 */
                int32_t    w11 = fy_0 * fx_0[x];    /* q23.8 * q23.8 => q15.16 */

                /* weighted sum of the four neighbors of the actual source position */
                *pDst++ = (int8_t)(((srcImage[INDEX2D(src_y,     src_x,     src_w)] * w00
                        + srcImage[INDEX2D(src_y,     src_x + 1, src_w)] * w01
                        + srcImage[INDEX2D(src_y + 1, src_x,     src_w)] * w10
                        + srcImage[INDEX2D(src_y + 1, src_x + 1, src_w)] * w11) >> 16) + zero_point);
            }
        }
    } else if ( src_ch == 3 ) {
        int     srcLineWidth = src_ch * src_w;
        int     dstLineWidth = src_ch * dst_w;

        for ( ; y < dst_h  &&  src_y_f < ((src_h - 1) << 16); y++, src_y_f += step_y ) {
            int        src_y = src_y_f >> 16;            /* Y coordinate of the upper neighbors */
            int32_t    fy_0  = (src_y_f >> 8) & 0x0FF;   /* q23.8, weight for the lower neighbors: (x, src_y + 1) */
            int32_t    fy_1  = (1 << 8) - fy_0;          /* q23.8, weight for the upper neighbors: (x, src_y) */
            uint8_t    *srcBlk = &srcImage[srcLineWidth * src_y];
            int8_t    *pDst   = &dstImage[dstLineWidth * y + min_x * 3];

            for ( int x = min_x; x < max_x; x++ ) {
                uint8_t    *pSrc = &srcBlk[src_x_map[x]];
                int32_t    w00 = fy_1 * fx_1[x];    /* q23.8 * q23.8 => q15.16 */
                int32_t    w01 = fy_1 * fx_0[x];    /* q23.8 * q23.8 => q15.16 */
                int32_t    w10 = fy_0 * fx_1[x];    /* q23.8 * q23.8 => q15.16 */
                int32_t    w11 = fy_0 * fx_0[x];    /* q23.8 * q23.8 => q15.16 */

                /* weighted sum of the four neighbors of the actual source position */
                *pDst++ = (int8_t)(((pSrc[0]             * w00 + pSrc[3]                * w01 +
                        pSrc[srcLineWidth] * w10 + pSrc[srcLineWidth + 3] * w11) >> 16) + zero_point);
                pSrc++;        /* move to next channel */
                *pDst++ = (int8_t)(((pSrc[0]             * w00 + pSrc[3]                * w01 +
                        pSrc[srcLineWidth] * w10 + pSrc[srcLineWidth + 3] * w11) >> 16) + zero_point);
                pSrc++;        /* move to next channel */
                *pDst++ = (int8_t)(((pSrc[0]             * w00 + pSrc[3]                * w01 +
                        pSrc[srcLineWidth] * w10 + pSrc[srcLineWidth + 3] * w11) >> 16) + zero_point);
            }
        }
    }
}
CY_SECTION_ITCM_END


/******************************************************************************
* Function Name: ifx_warpAffine
*******************************************************************************
* Summary:
*  Applies a 2D warp transformation matrix to a point in the destination domain
*  to compute the corresponding point in the source domain.
*
* Parameters:
*  float mtx[][3]: 2D warp transformation matrix
*  float *pt: Input point in the destination domain (x, y)
*  float *out: Output point in the source domain (x, y)
*
* Return:
*  void
*
******************************************************************************/
void ifx_warpAffine( float mtx[][3], float *pt, float *out )
{
    out[0]  = mtx[0][0] * pt[0] + mtx[0][1] * pt[1] + mtx[0][2];
    out[1]  = mtx[1][0] * pt[0] + mtx[1][1] * pt[1] + mtx[1][2];

}


/******************************************************************************
* Function Name: ifx_image_resize_Matrix_u2i_org
*******************************************************************************
* Summary:
*  Resizes a region of a uint8_t source image to an int8_t destination image
*  using a 2D warp matrix and nearest-neighbor interpolation. Applies a zero-
*  point offset to the output. Pixels mapping outside the source image are set 
*  to the zero-point value.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int imgWidth: Width of the source image
*  int imgHeight: Height of the source image
*  int imgCh: Number of source image channels
*  int8_t *subImage: Pointer to the destination image buffer (int8_t)
*  int dstWidth: Width of the destination image
*  int dstHeight: Height of the destination image
*  float warpMtx[][3]: 2D warp transformation matrix
*  int zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Matrix_u2i_org( uint8_t *srcImage, int imgWidth, int imgHeight, int imgCh,
        int8_t *subImage, int dstWidth, int dstHeight,
        float warpMtx[][3], int zero_point )
{
    float   alignedPt[2] = {0, 0};   /* [x, y] */
    float   warpedPt[2];
    int        dstIdx = 0;

    for ( int yy = 0; yy < dstHeight; yy++, alignedPt[1] += 1.0f ) {
        alignedPt[0] = 0;   /* x */
        for ( int xx = 0; xx < dstWidth; xx++, alignedPt[0] += 1.0f ) {
            /* get coordinates in source image */
            ifx_warpAffine( warpMtx, alignedPt, warpedPt );
            int ix = warpedPt[0] + 0.5f;
            int iy = warpedPt[1] + 0.5f;

            /* check if the mapped coordinate is inside the source image */
            if ( 0 <= ix  &&  ix < imgWidth  &&
                    0 <= iy  &&  iy < imgHeight ) {
                int srcIdx = INDEX3D(iy, ix, 0, imgWidth, imgCh);

                subImage[dstIdx++] = srcImage[srcIdx++] + zero_point;   /* red */
                subImage[dstIdx++] = srcImage[srcIdx++] + zero_point;   /* green */
                subImage[dstIdx++] = srcImage[srcIdx++] + zero_point;   /* blue */
            } else {
                subImage[dstIdx++] = zero_point;
                subImage[dstIdx++] = zero_point;
                subImage[dstIdx++] = zero_point;
            }
        }
    }
}


/******************************************************************************
* Function Name: is_in_image_pixel
*******************************************************************************
* Summary:
*  Checks if a warped coordinate is inside the source image boundaries after
*  applying a 2D warp transformation matrix.
*
* Parameters:
*  float warpMtx[][3]: 2D warp transformation matrix
*  int imgWidth: Width of the source image
*  int imgHeight: Height of the source image
*  float x0: X-coordinate of the point in the destination domain
*  float y0: Y-coordinate of the point in the destination domain
*
* Return:
*  int: 0 if the warped coordinate is inside the source image, 1 otherwise
*
******************************************************************************/
static int is_in_image_pixel( float warpMtx[][3], int imgWidth, int imgHeight, float x0, float y0 )
{
    float   alignedPt[2] = {x0, y0};    /* [x, y] */
    float   warpedPt[2];

    ifx_warpAffine( warpMtx, alignedPt, warpedPt );
    int ix = warpedPt[0] + 0.5f;
    int iy = warpedPt[1] + 0.5f;

    /* check if the mapped coordinate is inside the source image */
    return ( 0 <= ix  &&  ix < imgWidth  &&
            0 <= iy  &&  iy < imgHeight ) ? 0 : 1;
}


/******************************************************************************
* Function Name: is_in_image_line
*******************************************************************************
* Summary:
*  Checks if the warped endpoints of a line segment [(0,0) to (x1,y1)] are
*  inside the source image boundaries after applying a 2D warp transformation
*  matrix.
*
* Parameters:
*  float warpMtx[][3]: 2D warp transformation matrix
*  int imgWidth: Width of the source image
*  int imgHeight: Height of the source image
*  int x1: X-coordinate of the line endpoint in the destination domain
*  int y1: Y-coordinate of the line endpoint in the destination domain
*
* Return:
*  int: 0 if both endpoints are inside the source image, negative error code 
*       otherwise
*
******************************************************************************/
static int is_in_image_line( float warpMtx[][3], int imgWidth, int imgHeight, int x1, int y1 )
{
    float   fx1 = (float) x1;
    float   fy1 = (float) y1;

    if ( is_in_image_pixel( warpMtx, imgWidth, imgHeight,   0, fy1 ) != 0 )
        return -1;
    if ( is_in_image_pixel( warpMtx, imgWidth, imgHeight, fx1, fy1 ) != 0 )
        return -2;

    return 0;
}


/******************************************************************************
* Function Name: is_in_image_rect
*******************************************************************************
* Summary:
*  Checks if the warped corners of a rectangular region [(0,0) to (x1,y1)] are
*  inside the source image boundaries after applying a 2D warp transformation
*  matrix.
*
* Parameters:
*  float warpMtx[][3]: 2D warp transformation matrix
*  int imgWidth: Width of the source image
*  int imgHeight: Height of the source image
*  int x1: Maximum X-coordinate of the rectangle in the destination domain
*  int y1: Maximum Y-coordinate of the rectangle in the destination domain
*
* Return:
*  int: 0 if all corners are inside the source image, negative error code 
*       otherwise
*
******************************************************************************/
static int is_in_image_rect( float warpMtx[][3], int imgWidth, int imgHeight, int x1, int y1 )
{
    float   fx1 = (float) x1;
    float   fy1 = (float) y1;

    if ( is_in_image_pixel( warpMtx, imgWidth, imgHeight,   0,   0 ) != 0 )
        return -1;
    if ( is_in_image_pixel( warpMtx, imgWidth, imgHeight, fx1, fy1 ) != 0 )
        return -3;
    if ( is_in_image_pixel( warpMtx, imgWidth, imgHeight, fx1,   0 ) != 0 )
        return -2;
    if ( is_in_image_pixel( warpMtx, imgWidth, imgHeight,   0, fy1 ) != 0 )
        return -4;

    return 0;
}


/******************************************************************************
* Function Name: ifx_image_resize_Matrix_u2i_in_Rect
*******************************************************************************
* Summary:
*  Resizes a region of a uint8_t source image to an int8_t destination image
*  using a 2D warp matrix and nearest-neighbor interpolation, optimized with MVE
*  (Matrix Vector Extension) instructions. Applies a zero-point offset to the
*  output. Assumes all pixels map within the source image boundaries.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int imgWidth: Width of the source image
*  int imgHeight: Height of the source image
*  int imgCh: Number of source image channels
*  int8_t *subImage: Pointer to the destination image buffer (int8_t)
*  int dstWidth: Width of the destination image
*  int dstHeight: Height of the destination image
*  float warpMtx[][3]: 2D warp transformation matrix
*  int zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Matrix_u2i_in_Rect( uint8_t *srcImage, int imgWidth, int imgHeight, int imgCh,
        int8_t *subImage, int dstWidth, int dstHeight,
        float warpMtx[][3], int zero_point )
{
    image_resize_Matrix_u2i_i32_mve_var_decl();
    int    dstIdx = 0;
    int srcIdx;
    int ix, iy;

    for ( int yy = 0; yy < dstHeight; yy++ ) {
        int32x4_t   vfx = vfx0;     /* q15.16 */
        int32x4_t   vfy = vfy0;     /* q15.16 */

        for ( int xx = 0; xx < (dstWidth >> 2); xx++ ) {
            int32x4_t vix = vshrq_n_s32( vfx, 16 );      /* q31.0 */
            int32x4_t viy = vshrq_n_s32( vfy, 16 );      /* q31.0 */

            write_pixel_MVE_no_boundary_check( 0 );
            write_pixel_MVE_no_boundary_check( 1 );
            write_pixel_MVE_no_boundary_check( 2 );
            write_pixel_MVE_no_boundary_check( 3 );

            vfx = vaddq_s32( vfx, vdx_x );     /* imcrement in X by X-inc */
            vfy = vaddq_s32( vfy, vdy_x );     /* imcrement in Y by X-inc */
        }

        vfx0 = vaddq_s32( vfx0, vdx_y );    /* imcrement in X by Y-inc */
        vfy0 = vaddq_s32( vfy0, vdy_y );    /* imcrement in Y by Y-inc */
    }
}


CY_SECTION_ITCM_BEGIN
/******************************************************************************
* Function Name: ifx_image_resize_Matrix_u2i
*******************************************************************************
* Summary:
*  Resizes a region of a uint8_t source image to an int8_t destination image
*  using a 2D warp matrix and nearest-neighbor interpolation, optimized with 
*  MVE instructions. Applies a zero-point offset to the output. Pixels mapping 
*  outside the source image are set to the zero-point value.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int imgWidth: Width of the source image
*  int imgHeight: Height of the source image
*  int imgCh: Number of source image channels
*  int8_t *subImage: Pointer to the destination image buffer (int8_t)
*  int dstWidth: Width of the destination image
*  int dstHeight: Height of the destination image
*  float warpMtx[][3]: 2D warp transformation matrix
*  int zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Matrix_u2i( uint8_t *srcImage, int imgWidth, int imgHeight, int imgCh,
        int8_t *subImage, int dstWidth, int dstHeight,
        float warpMtx[][3], int zero_point )
{
    image_resize_Matrix_u2i_i32_mve_var_decl();
    int    dstIdx = 0;
    int ix, iy;
    int red, green, blue;

    for ( int yy = 0; yy < dstHeight; yy++ ) {
        int32x4_t   vfx = vfx0;     /* q15.16 */
        int32x4_t   vfy = vfy0;     /* q15.16 */

        if ( is_in_image_line( warpMtx, imgWidth, imgHeight, dstWidth - 1, yy ) == 0 ) {
            int srcIdx;

            for ( int xx = 0; xx < (dstWidth >> 2); xx++ ) {
                int32x4_t vix = vshrq_n_s32( vfx, 16 );      /* q31.0 */
                int32x4_t viy = vshrq_n_s32( vfy, 16 );      /* q31.0 */

                write_pixel_MVE_no_boundary_check( 0 );
                write_pixel_MVE_no_boundary_check( 1 );
                write_pixel_MVE_no_boundary_check( 2 );
                write_pixel_MVE_no_boundary_check( 3 );

                vfx = vaddq_s32( vfx, vdx_x );     /* imcrement in X by X-inc */
                vfy = vaddq_s32( vfy, vdy_x );     /* imcrement in Y by X-inc */
            }
        } else {
            for ( int xx = 0; xx < (dstWidth >> 2); xx++ ) {
                int32x4_t vix = vshrq_n_s32( vfx, 16 );      /* q31.0 */
                int32x4_t viy = vshrq_n_s32( vfy, 16 );      /* q31.0 */

                write_pixel_MVE( 0 );
                write_pixel_MVE( 1 );
                write_pixel_MVE( 2 );
                write_pixel_MVE( 3 );

                vfx = vaddq_s32( vfx, vdx_x );     /* imcrement in X by X-inc */
                vfy = vaddq_s32( vfy, vdy_x );     /* imcrement in Y by X-inc */
            }
        }

        vfx0 = vaddq_s32( vfx0, vdx_y );    /* imcrement in X by Y-inc */
        vfy0 = vaddq_s32( vfy0, vdy_y );    /* imcrement in Y by Y-inc */
    }
}
CY_SECTION_ITCM_END


/******************************************************************************
* Function Name: ifx_image_resize_Matrix_u2i_check
*******************************************************************************
* Summary:
*  Resizes a region of a uint8_t source image to an int8_t destination image
*  using a 2D warp matrix and nearest-neighbor interpolation. Checks if the
*  warped rectangular region is fully within the source image boundaries and
*  selects the appropriate resizing function (with or without boundary checks).
*  Applies a zero-point offset to the output.
*
* Parameters:
*  uint8_t*srcImage: Pointer to the source image buffer (uint8_t)
*  int imgWidth: Width of the source image
*  int imgHeight: Height of the source image
*  int imgCh: Number of source image channels
*  int8_t*subImage: Pointer to the destination image buffer (int8_t)
*  int dstWidth: Width of the destination image
*  int dstHeight: Height of the destination image
*  float warpMtx[][3]: 2D warp transformation matrix
*  int zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_resize_Matrix_u2i_check( uint8_t *srcImage, int imgWidth, int imgHeight, int imgCh,
        int8_t *subImage, int dstWidth, int dstHeight,
        float warpMtx[][3], int zero_point )
{
    if ( is_in_image_rect( warpMtx, imgWidth, imgHeight, dstWidth - 1, dstHeight - 1 ) == 0 ) {
        ifx_image_resize_Matrix_u2i_in_Rect( srcImage, imgWidth, imgHeight, imgCh,
                subImage, dstWidth, dstHeight,
                warpMtx, zero_point );
    } else {
        ifx_image_resize_Matrix_u2i( srcImage, imgWidth, imgHeight, imgCh,
                subImage, dstWidth, dstHeight,
                warpMtx, zero_point );
    }
}


/******************************************************************************/
// Extract / copy a rectanglar region from the source image
/******************************************************************************
* Function Name: image_copy_Rect
*******************************************************************************
* Summary:
*  Copies a rectangular region from the source image to the destination image.
*  The region is defined by its width and height, starting at specified 
*  coordinates in both source and destination images. Supports both uint8_t 
*  and int8_t images via casting.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer
*  int src_x: X-coordinate of the copy region in the source image
*  int src_y: Y-coordinate of the copy region in the source image
*  int src_w: Width of the source image
*  int src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer
*  int dst_x: X-coordinate of the copy region in the destination image
*  int dst_y: Y-coordinate of the copy region in the destination image
*  int dst_w: Width of the destination image
*  int rect_w: Width of the copy region
*  int rect_h: Height of the copy region
*
* Return:
*  void
*
******************************************************************************/
inline static void image_copy_Rect( uint8_t *srcImage, int src_x, int src_y, int src_w, int src_ch,
        uint8_t *dstImage, int dst_x, int dst_y, int dst_w,
        int rect_w, int rect_h )
{
    uint8_t    *dp = &dstImage[INDEX3D(dst_y, dst_x, 0, dst_w, src_ch)];
    uint8_t    *sp = &srcImage[INDEX3D(src_y, src_x, 0, src_w, src_ch)];

    for ( int ii = 0; ii < rect_h; ii++ ) {
        memcpy( dp, sp, rect_w * src_ch );
        dp += dst_w * src_ch;
        sp += src_w * src_ch;
    }
}


/******************************************************************************
* Function Name: image_extract
*******************************************************************************
* Summary:
*  Extracts a region from the source image to the destination image. If the 
*  source is larger, copies the center region; if smaller, copies the entire 
*  source to the upper-left region of the destination. Extra destination rows 
*  or columns are filled with a padding value. Supports both uint8_t and 
*  int8_t images via casting.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer
*  int src_w: Width of the source image
*  int src_h: Height of the source image
*  int src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer
*  int dst_w: Width of the destination image
*  int dst_h: Height of the destination image
*  int pad_val: Padding value for unoccupied areas
*
* Return:
*  void
*
******************************************************************************/
static void image_extract( uint8_t *srcImage, int src_w, int src_h, int src_ch,
        uint8_t *dstImage, int dst_w, int dst_h, int pad_val )
{
    int    rect_w   = min( dst_w, src_w );           /* width of copying region */
    int    rect_h   = min( dst_h, src_h );           /* height of copying region */
    int    offset_w = ( src_w - rect_w ) / 2;        /* X-offset of copying region in the source */
    int    offset_h = ( src_h - rect_h ) / 2;        /* Y-offset of copying region in the source */

    memset( (void *)dstImage, pad_val, dst_w * dst_h * src_ch );    /* initialize the target image */

    image_copy_Rect( srcImage, offset_w, offset_h, src_w, src_ch,
            dstImage, 0, 0, dst_w, rect_w, rect_h );
}


/******************************************************************************
* Function Name: ifx_image_extract
*******************************************************************************
* Summary:
*  Extracts a region from a uint8_t source image to a uint8_t destination 
*  image. If the source is larger, copies the center region; if smaller, copies 
*  the entire source to the upper-left region. Extra destination rows or columns 
*  are filled with a padding value of 128.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer (uint8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_extract( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        uint8_t *dstImage, int32_t dst_w, int32_t dst_h )
{
    image_extract( srcImage, src_w, src_h, src_ch, dstImage, dst_w, dst_h, 128 );
}


/******************************************************************************
* Function Name: ifx_image_extract_i8
*******************************************************************************
* Summary:
*  Extracts a region from an int8_t source image to an int8_t destination image.
*  If the source is larger, copies the center region; if smaller, copies the 
*  entire source to the upper-left region. Extra destination rows or columns 
*  are filled with a padding value of 0.
*
* Parameters:
*  int8_t *srcImage: Pointer to the source image buffer (int8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_extract_i8( int8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t *dstImage, int32_t dst_w, int32_t dst_h )
{
    image_extract( (uint8_t *)srcImage, src_w, src_h, src_ch, (uint8_t *)dstImage, dst_w, dst_h, 0 );
}


/******************************************************************************
* Function Name: ifx_image_extract_u2i
*******************************************************************************
* Summary:
*  Extracts a region from a uint8_t source image to an int8_t destination image.
*  If the source is larger, copies the center region; if smaller, copies the 
*  entire source to the upper-left region. Applies a zero-point offset to the 
*  output. Extra destination rows or columns are filled with a padding value 
*  of 0.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  int32_t zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_extract_u2i( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t  *dstImage, int32_t dst_w, int32_t dst_h, int32_t zero_point )
{
    int    rect_w   = min( dst_w, src_w );
    int    rect_h   = min( dst_h, src_h );
    int    offset_w = ( src_w - rect_w ) / 2;
    int    offset_h = ( src_h - rect_h ) / 2;

    memset( (void *)dstImage, 0, dst_w * dst_h * src_ch );    /* initialize to mid-value of uint8 */

    int8_t    *dp = dstImage;
    uint8_t    *sp = &srcImage[INDEX3D(offset_h, offset_w, 0, src_w, src_ch )];

    for ( int ii = 0; ii < rect_h; ii++ ) {
        for ( int jj = 0; jj < rect_w * src_ch; jj++ )
            dp[jj] = (int8_t)(sp[jj] + zero_point);
        dp += dst_w * src_ch;
        sp += src_w * src_ch;
    }
}


/******************************************************************************
* Function Name: image_extract_Rect
*******************************************************************************
* Summary:
*  Extracts a rectangular region from the source image starting at (xmin, ymin)
*  to the destination image. Extra destination rows or columns are filled with
*  a padding value. Handles cases where the region extends beyond the source 
*  image boundaries. Supports both uint8_t and int8_t images via casting.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer
*  int src_w: Width of the source image
*  int src_h: Height of the source image
*  int src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer
*  int dst_w: Width of the destination image
*  int dst_h: Height of the destination image
*  int xmin: X-coordinate of the top-left corner in the source image
*  int ymin: Y-coordinate of the top-left corner in the source image
*  int pad_val: Padding value for unoccupied areas
*
* Return:
*  void
*
******************************************************************************/
static void image_extract_Rect( uint8_t *srcImage, int src_w, int src_h, int src_ch,
        uint8_t *dstImage, int dst_w, int dst_h,
        int xmin, int ymin, int pad_val )
{
    /* initialize the target image */
    memset( (void *)dstImage, pad_val, dst_w * dst_h * src_ch );

    /* check if the crop area is out of image */
    if ( xmin >= src_w  ||  ymin >= src_w )
        return;

    /* width and height of the copying region */
    int    rect_w = dst_w;
    int    rect_h = dst_h;
    int    offset_w = 0;
    int    offset_h = 0;

    /* check if the starting location if out of image, then set it to 0. */
    if ( xmin < 0 ) {
        offset_w -= xmin;        /* move the copying location in the target image */
        rect_w   += xmin;        /* reduce the copying region */
        xmin = 0;                /* reset the copying location in the source image */
    }
    if ( ymin < 0 ) {
        offset_h -= ymin;        /* move the copying location in the target image */
        rect_h   += ymin;        /* reduce the copying region */
        ymin = 0;                /* reset the copying location in the source image */
    }

    /* adjust width and height of the crop area, if necessary */
    rect_w = min( rect_w, src_w - xmin );
    rect_h = min( rect_h, src_h - ymin );

    if ( rect_w > 0  &&  rect_h > 0 )
        image_copy_Rect( srcImage, xmin, ymin, src_w, src_ch,
                dstImage, offset_w, offset_h, dst_w, rect_w, rect_h );
}


/******************************************************************************
* Function Name: ifx_image_extract_Rect
*******************************************************************************
* Summary:
*  Extracts a rectangular region from a uint8_t source image starting at
*  (xmin, ymin) to a uint8_t destination image. Extra destination rows or 
*  columns are filled with a padding value of 128. Handles cases where the 
*  region extends beyond the source image boundaries.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  uint8_t *dstImage: Pointer to the destination image buffer (uint8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  int32_t xmin: X-coordinate of the top-left corner in the source image
*  int32_t ymin: Y-coordinate of the top-left corner in the source image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_extract_Rect( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        uint8_t *dstImage, int32_t dst_w, int32_t dst_h,
        int32_t xmin, int32_t ymin )
{
    image_extract_Rect( srcImage, src_w, src_h, src_ch,
            dstImage, dst_w, dst_h, xmin, ymin, 128 );
}


/******************************************************************************
* Function Name: ifx_image_extract_Rect_i8
*******************************************************************************
* Summary:
*  Extracts a rectangular region from an int8_t source image starting at
*  (xmin, ymin) to an int8_t destination image. Extra destination rows or 
*  columns are filled with a padding value of 0. Handles cases where the 
*  region extends beyond the source image boundaries.
*
* Parameters:
*  int8_t *srcImage: Pointer to the source image buffer (int8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  int32_t xmin: X-coordinate of the top-left corner in the source image
*  int32_t ymin: Y-coordinate of the top-left corner in the source image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_extract_Rect_i8( int8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t *dstImage, int32_t dst_w, int32_t dst_h,
        int32_t xmin, int32_t ymin )
{
    image_extract_Rect( (uint8_t *)srcImage, src_w, src_h, src_ch,
            (uint8_t *)dstImage, dst_w, dst_h, xmin, ymin, 0 );
}


/******************************************************************************
* Function Name: ifx_image_extract_Rect_u2i
*******************************************************************************
* Summary:
*  Extracts a rectangular region from a uint8_t source image starting at
*  (xmin, ymin) to an int8_t destination image. Applies a zero-point offset to
*  the output. Extra destination rows or columns are filled with a padding 
*  value of 0. Handles cases where the region extends beyond the source image 
*  boundaries.
*
* Parameters:
*  uint8_t *srcImage: Pointer to the source image buffer (uint8_t)
*  int32_t src_w: Width of the source image
*  int32_t src_h: Height of the source image
*  int32_t src_ch: Number of source image channels
*  int8_t *dstImage: Pointer to the destination image buffer (int8_t)
*  int32_t dst_w: Width of the destination image
*  int32_t dst_h: Height of the destination image
*  int32_t xmin: X-coordinate of the top-left corner in the source image
*  int32_t ymin: Y-coordinate of the top-left corner in the source image
*  int32_t zero_point: Zero-point offset for quantization
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_extract_Rect_u2i( uint8_t *srcImage, int32_t src_w, int32_t src_h, int32_t src_ch,
        int8_t  *dstImage, int32_t dst_w, int32_t dst_h,
        int32_t xmin, int32_t ymin, int32_t zero_point )
{
    /* initialize to mid-value of int8_t */
    memset( (void *)dstImage, 0, dst_w * dst_h * src_ch );

    /* check if the crop area is out of image */
    if ( xmin >= src_w  ||  ymin >= src_w )
        return;

    /* width and height of the crop area */
    int    rect_w   = dst_w;
    int    rect_h   = dst_h;
    int    offset_w = 0;
    int    offset_h = 0;

    /* check if the starting location if out of image, then set it to 0. */
    if ( xmin < 0 ) {
        offset_w -= xmin;
        rect_w   += xmin;
        xmin = 0;
    }
    if ( ymin < 0 ) {
        offset_h -= ymin;
        rect_h   += ymin;
        ymin = 0;
    }

    /* adjust width and height of the crop area, if necessary */
    rect_w = min( rect_w, src_w - xmin );
    rect_h = min( rect_h, src_h - ymin );

    if ( rect_w > 0  &&  rect_h > 0 ) {
        int8_t    *dp = &dstImage[INDEX3D(offset_h, offset_w, 0, dst_w, src_ch)];
        uint8_t    *sp = &srcImage[INDEX3D(ymin, xmin, 0, src_w, src_ch)];

        for ( int ii = 0; ii < rect_h; ii++ ) {
            for ( int jj = 0; jj < rect_w * src_ch; jj++ )
                dp[jj] = (int8_t)(sp[jj] + zero_point);
            dp += dst_w * src_ch;
            sp += src_w * src_ch;
        }
    }
}

/* [] END OF FILE */
