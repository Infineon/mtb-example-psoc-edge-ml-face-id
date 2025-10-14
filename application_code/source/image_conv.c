/******************************************************************************
* File Name        : image_conv.c
*
* Description      : This file implements the image format conversions.
*
* Related Document : See README.md
*
*******************************************************************************
* $ Copyright 2025 Cypress Semiconductor $
******************************************************************************/

/******************************************************************************
* Header Files
******************************************************************************/
#include "cy_syslib.h"
#include "ifx_image_utils.h"

/******************************************************************************
* Macros
******************************************************************************/
/*
 #define    BGR888_FULL_8_BIT
 usual shift-conversion function sets the low 2 or 3 bits in each channel to 0.
 so, the brightest 16-bit color (0xFFFF) becomes the 24-bit color of R=0xF8, G=0xFC, B=0xF8,
 which is not the brightest 24-bit color (0xFFFFFF).
 fill these low 2 or 3 bits by burning more cycles.
 red, blue:   ([0, 0xF8] * 66) >> 6 => ([0, 248] * 66) >> 6 => [0, 16368] >> 6 => ([0, 255.75]) => [0, 255]
 green:       ([0, 0xFC] * 65) >> 6 => ([0, 252] * 65) >> 6 => [0, 16380] >> 6 => ([0, 255.94]) => [0, 255]
 */

/* Get most significant byte */
#define GET_MSB(a, bits, ofs)  ((((a) >> (8 - (bits))) & ((1 << (bits)) - 1)) << (ofs))

/******************************************************************************
* Functions
******************************************************************************/

/******************************************************************************
* Function Name: ifx_pixel_RGB888_to_RGB565
*******************************************************************************
* Summary:
*  Converts a color pixel value from RGB888 format (8 bits per channel) to
*  RGB565 format (16 bits total: 5 bits red, 6 bits green, 5 bits blue).
*
* Parameters:
*  uint8_t r: Red color value (0-255)
*  uint8_t g: Green color value (0-255)
*  uint8_t b: Blue color value (0-255)
*
* Return:
*  uint16_t: 16-bit color value in RGB565 format
*
******************************************************************************/
inline uint16_t ifx_pixel_RGB888_to_RGB565(uint32_t r, uint32_t g, uint32_t b)
{
    b = GET_MSB(b, 5,  0);
    g = GET_MSB(g, 6,  5);
    r = GET_MSB(r, 5, 11);

    return (uint16_t)(r | g | b);
}


/******************************************************************************
* Function Name: ifx_pixel_RGB565_to_RGB888_value
*******************************************************************************
* Summary:
*  Converts a color pixel value from RGB565 format (16 bits: 5 bits red, 6 bits
*  green, 5 bits blue) to RGB888 format (24 bits: 8 bits per channel). Supports
*  multiple conversion modes: default (low 2 or 3 bits set to 0), full 8-bit
*  usage, or full 8-bit usage with rounding.
*
* Parameters:
*  uint16_t rgb565: 16-bit color value in RGB565 format
*
* Return:
*  uint32_t: 24-bit color value in RGB888 format
*
******************************************************************************/
inline uint32_t ifx_pixel_RGB565_to_RGB888_value( uint32_t rgb565 )
{
    /* Extract B, G, and R components from BGR565 pixel (already in BGR order) */
    uint32_t    blue  = (rgb565 << 3) & 0xF8u;      /* [0, 248], Extract lower 5 bits for blue */
    uint32_t    green = (rgb565 >> 3) & 0xFCu;      /* [0, 252], Extract middle 6 bits for green, right shift 5 for 8-bit representation */
    uint32_t    red   = (rgb565 >> 8) & 0xF8u;      /* [0, 248], Extract upper 5 bits for red, right shift 11 for 8-bit representation */

#ifdef BGR888_FULL_8_BIT
    red   = (red   * 66) >> 6;
    green = (green * 65) >> 6;
    blue  = (blue  * 66) >> 6;
#endif  /* BGR888_FULL_8_BIT */

    return (uint32_t)((blue << 16) | (green << 8) | red);   /* image array sequence */
}


/******************************************************************************
* Function Name: ifx_pixel_RGB565_to_RGB888
*******************************************************************************
* Summary:
*  Converts a color pixel value from RGB565 format (16 bits: 5 bits red, 6 
*  bits green, 5 bits blue) to RGB888 format (24 bits: 8 bits per channel) 
*  and stores the result in the provided buffer. Supports multiple conversion
*  modes: default (low 2 or 3 bits set to 0), full 8-bit usage, or full 8-bit 
*  usage with rounding.
*
* Parameters:
*  uint16_t rgb565: 16-bit color value in RGB565 format
*  uint8_t *rgb888: Pointer to the 24-bit color buffer for storing RGB888 
*                   values
*
* Return:
*  uint8_t*: Pointer to the next position in the 24-bit color buffer
*
******************************************************************************/
inline uint8_t * ifx_pixel_RGB565_to_RGB888( uint32_t rgb565, uint8_t *rgb888 )
{
    /* Extract B, G, and R components from BGR565 pixel (already in BGR order) */
    uint32_t    blue  = (rgb565 << 3) & 0xF8u;      /* [0, 248], Extract lower 5 bits for blue */
    uint32_t    green = (rgb565 >> 3) & 0xFCu;      /* [0, 252], Extract middle 6 bits for green, right shift 5 for 8-bit representation */
    uint32_t    red   = (rgb565 >> 8) & 0xF8u;      /* [0, 248], Extract upper 5 bits for red, right shift 11 for 8-bit representation */

#ifdef BGR888_FULL_8_BIT
    red   = (red   * 66) >> 6;
    green = (green * 65) >> 6;
    blue  = (blue  * 66) >> 6;
#endif  /* BGR888_FULL_8_BIT */

    *rgb888++ = (uint8_t)red;
    *rgb888++ = (uint8_t)green;
    *rgb888++ = (uint8_t)blue;

    return rgb888;
}


/******************************************************************************
* Function Name: ifx_pixel_2_RGB565_to_RGB888
*******************************************************************************
* Summary:
*  Converts two RGB565 color pixel values (packed in a 32-bit integer) to 
*  RGB888 format and stores the result in the provided buffer. Supports full 
*  8-bit usage conversion if BGR888_FULL_8_BIT is defined. Processes two 
*  pixels at a time.
*
* Parameters:
*  uint32_t rgb565: 32-bit value containing two RGB565 pixels (low 16-bit: 
*                   first pixel, high 16-bit: second pixel)
*  uint8_t *rgb888: Pointer to the 24-bit RGB888 color buffer for storing 
*                   the result
*
* Return:
*  uint8_t*: Pointer to the next position in the RGB888 color buffer
*
******************************************************************************/
inline uint8_t * ifx_pixel_2_RGB565_to_RGB888( uint32_t rgb565, uint8_t *rgb888 )
{
    /* Extract B, G, and R components from BGR565 pixel (already in BGR order) */
    /* low 16-bit: the first pixel, high 16-bit: the second pixel */
    uint32_t    blue  = (rgb565 << 3) & 0x0F800F8u;     /* xxB1 | xxB0 */
    uint32_t    green = (rgb565 >> 3) & 0x0FC00FCu;     /* xxG1 | xxG0 */
    uint32_t    red   = (rgb565 >> 8) & 0x0F800F8u;     /* xxR1 | xxR0 */

#ifdef BGR888_FULL_8_BIT
    red   = (red   * 66) >> 6;
    green = (green * 65) >> 6;
    blue  = (blue  * 66) >> 6;
#endif  /* BGR888_FULL_8_BIT */

    uint16_t    *rgb888_16p = (uint16_t *)rgb888;
    uint32_t    rg = (green << 8) | red;        /* G1R1 | G0R0* */
    uint32_t    br = (red   >> 8) | blue;       /* xxB1 | R1B0* */
    uint32_t    gb = (blue  << 8) | green;      /* B1G1*| B0G0 */

    *rgb888_16p++ = (uint16_t)rg;
    *rgb888_16p++ = (uint16_t)br;
    *rgb888_16p++ = (uint16_t)(gb >> 16);

    return (uint8_t *)rgb888_16p;
}


/******************************************************************************
* Function Name: ifx_pixel_RGB565_to_RGB888_u2i
*******************************************************************************
* Summary:
*  Converts a single RGB565 color pixel value to RGB888 format with an 
*  optional zero-point offset for int8_t output. Stores the result in the 
*  provided buffer.Supports full 8-bit usage conversion if BGR888_FULL_8_BIT 
*  is defined.
*
* Parameters:
*  uint32_t rgb565: 16-bit RGB565 color value
*  int8_t *rgb888: Pointer to the 24-bit RGB888 color buffer for storing the 
*                   result
*  int32_t zero_point: Zero-point offset to apply to the output values
*
* Return:
*  int8_t*: Pointer to the next position in the RGB888 color buffer
*
******************************************************************************/
inline int8_t * ifx_pixel_RGB565_to_RGB888_u2i( uint32_t rgb565, int8_t *rgb888, int32_t zero_point )
{
    /* Extract B, G, and R components from BGR565 pixel (already in BGR order) */
    int    blue   = (rgb565 << 3) & 0x0F8;         /* Extract lower 5 bits for blue */
    int    green  = (rgb565 >> 3) & 0x0FC;         /* Extract middle 6 bits for green, right shift 5 for 8-bit representation */
    int red    = (rgb565 >> 8) & 0x0F8;            /* Extract upper 5 bits for red, right shift 11 for 8-bit representation */

#ifdef BGR888_FULL_8_BIT
    red   = (red   * 66) >> 6;
    green = (green * 65) >> 6;
    blue  = (blue  * 66) >> 6;
#endif  /* BGR888_FULL_8_BIT */

    *rgb888++ = red   + zero_point;
    *rgb888++ = green + zero_point;
    *rgb888++ = blue  + zero_point;

    return rgb888;
}


/******************************************************************************
* Function Name: ifx_pixel_2_RGB565_to_RGB888_u2i
*******************************************************************************
* Summary:
*  Converts two RGB565 color pixel values (packed in a 32-bit integer) to 
*  RGB888 format with an optional zero-point offset for int8_t output. Stores 
*  the result in the provided buffer. Supports full 8-bit usage conversion if 
*  BGR888_FULL_8_BIT is defined and uses SIMD instructions for optimization if 
*  ARM_MATH_DSP is defined.
*
* Parameters:
*  uint32_t rgb565: 32-bit value containing two RGB565 pixels (low 16-bit: 
*                   first pixel, high 16-bit: second pixel)
*  int8_t *rgb888: Pointer to the 24-bit RGB888 color buffer for storing the 
*                   result
*  int32_t zero_point: Zero-point offset to apply to the output values
*
* Return:
*  int8_t*: Pointer to the next position in the RGB888 color buffer
*
******************************************************************************/
int8_t * ifx_pixel_2_RGB565_to_RGB888_u2i( uint32_t rgb565, int8_t *rgb888, int32_t zero_point )
{
    /* Extract B, G, and R components from BGR565 pixel (already in BGR order) */
    /* low 16-bit: the first pixel, high 16-bit: the second pixel */
    int32_t    blue   = (rgb565 << 3) & 0x0F800F8;     /* Extract lower 5 bits for blue */
    int32_t    green  = (rgb565 >> 3) & 0x0FC00FC;     /* Extract middle 6 bits for green, right shift 5 for 8-bit representation */
    int32_t    red    = (rgb565 >> 8) & 0x0F800F8;     /* Extract upper 5 bits for red, right shift 11 for 8-bit representation */

#ifdef BGR888_FULL_8_BIT
    red   = (red   * 66) >> 6;
    green = (green * 65) >> 6;
    blue  = (blue  * 66) >> 6;
#endif  /* BGR888_FULL_8_BIT */

#if defined (ARM_MATH_DSP)
    int32_t zero_point_packed;      /* Offset packed to 32 bit */

    /* Offset is packed to 32 bit in order to use SIMD32 for addition */
    zero_point_packed = __PKHBT(zero_point, zero_point, 16);
    /* Add offset and store result in destination buffer (2 samples at a time). */
    red   = __QADD16(red,   zero_point_packed);
    green = __QADD16(green, zero_point_packed);
    blue  = __QADD16(blue,  zero_point_packed);

    /* the first pixel */
    *rgb888++ = (int8_t)(red);
    *rgb888++ = (int8_t)(green);
    *rgb888++ = (int8_t)(blue);
    /* the second pixel */
    *rgb888++ = (int8_t)((red   >> 16));
    *rgb888++ = (int8_t)((green >> 16));
    *rgb888++ = (int8_t)((blue  >> 16));
#else
    /* the first pixel */
    *rgb888++ = (int8_t)((red  ) + zero_point);
    *rgb888++ = (int8_t)((green) + zero_point);
    *rgb888++ = (int8_t)((blue ) + zero_point);
    /* the second pixel */
    *rgb888++ = (int8_t)(((red   >> 16)) + zero_point);
    *rgb888++ = (int8_t)(((green >> 16)) + zero_point);
    *rgb888++ = (int8_t)(((blue  >> 16)) + zero_point);
#endif

    return rgb888;
}

#ifdef ARM_MATH_MVEI

#define get_write_RGB(src)    \
        *rgb888_16p++ = (uint16_t)vgetq_lane_u16( RG, src );      \
        *rgb888_16p++ = (uint16_t)vgetq_lane_u16( BR, src + 1 );  \
        *rgb888_16p++ = (uint16_t)vgetq_lane_u16( GB, src + 1 )

/* ifx_image_conv_RGB565_to_RGB888_16_narrow */
CY_SECTION_ITCM_BEGIN
/******************************************************************************
* Function Name: ifx_image_conv_RGB565_to_RGB888
*******************************************************************************
* Summary:
*  Converts the entire image from RGB565 format (16 bits per pixel) to RGB888
*  format (24 bits per pixel). If the source image is wider than the ,
*  destination copies the center part and ignores the side portions. If the 
*  source image is shorter in height than the destination, fills the extra 
*  destination rows with padding.
*
* Parameters:
*  uint16_t *rgb565: Pointer to the source 16-bit RGB565 image buffer
*  int width: Width of the source image
*  int height: Height of the source image
*  uint8_t *rgb888: Pointer to the destination 24-bit RGB888 image buffer
*  int dst_w: Width of the destination image
*  int dst_h: Height of the destination image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGB565_to_RGB888( uint8_t *src_bgr565, int32_t width, int32_t height,
        uint8_t *dst_rgb888, int32_t dst_width, int32_t dst_height )
{
    int min_height = min( height, dst_height );
    int min_width  = min( width,  dst_width );
    int offset_width = ( width - min_width ) >> 1;
    int blk_cnt      = min_width >> 3;              /* 8-pixel block: 16-byte: 128-bit */
    int blk_remainder= min_width & 0x07;
    uint16_t   *src_bgr565_16p = (uint16_t *) &src_bgr565[offset_width << 1];   /* 2 bytes per pixel */
    uint16x8_t  vmask_red_blue = vdupq_n_u16( 0xF800u );
    uint16x8_t  vmask_green    = vdupq_n_u16( 0xFC00u );

    for ( int i = 0; i < min_height; i++ ) {
        uint16_t    *rgb888_16p = (uint16_t *)&dst_rgb888[i * dst_width * 3];   /* 3 bytes per pixel */
        uint16_t    *bgr565_16p = &src_bgr565_16p[i * width];

        for ( int jj = blk_cnt; jj > 0; jj--, bgr565_16p += 8 ) {
            /* process 8 16-bit RGB565 colors (pixels) simultaneously in the vector unit */
            uint16x8_t  bgr565 = vldrhq_u16( bgr565_16p );  /* load 8 16-bit RGB565 into a vector reg. */
            uint16x8_t  red    = bgr565;                    /* move R to the high-bytes */
            uint16x8_t  green  = vshlq_n_u16( bgr565,  5 ); /* move G to the high-bytes */
            uint16x8_t  blue   = vshlq_n_u16( bgr565, 11 ); /* move B to the high-bytes */

            /* clear not-relavant bits in each color vector */
            red   = vandq_u16( red,   vmask_red_blue );     /* R3-- | R2-- | R1-- | R0-- */
            green = vandq_u16( green, vmask_green );        /* G3-- | G2-- | G1-- | G0-- */
            blue  = vandq_u16( blue,  vmask_red_blue );     /* B3-- | B2-- | B1-- | B0-- */

            /* shift colors to the high-bytes to combine with the low-byte in other colors */
            uint16x8_t  Bs = (uint16x8_t)vshlq_n_u32( (uint32x4_t)blue, 8 );            /* --B2 | --xx | --B0 | --xx */

            /* combine the high-bytes and the low-bytes into 2-byte words */
            uint16x8_t  RG = (uint16x8_t)vshrnbq_n_u16( (uint8x16_t)green, red, 8 );    /* G3R3 |*G2R2*| G1R1 |*G0R0* */
            uint16x8_t  BR = vorrq_u16( Bs, red );                                      /**R3B2*| R1xx |*R1B0*| R0xx */
            uint16x8_t  GB = (uint16x8_t)vshrnbq_n_u16( (uint8x16_t)blue, green, 8 );   /**B3G3*| B2G2 |*B1G1*| B0G0 */

            get_write_RGB(0);
            get_write_RGB(2);
            get_write_RGB(4);
            get_write_RGB(6);
        }
        if ( blk_remainder > 0 ) {
            uint8_t *rgb888 = (uint8_t *)rgb888_16p;

            for ( int jj = blk_remainder; jj > 0; jj-- ) {
                uint16_t    pixel = *bgr565_16p++;

                rgb888 = ifx_pixel_RGB565_to_RGB888( pixel, rgb888 );
            }
        }
    }
}


CY_SECTION_ITCM_END

#define get_write_RGB_u2i(idx)    \
        *rgb888_8p++ = (int8_t)vgetq_lane_s16( red,   idx );  \
        *rgb888_8p++ = (int8_t)vgetq_lane_s16( green, idx );  \
        *rgb888_8p++ = (int8_t)vgetq_lane_s16( blue,  idx )

/******************************************************************************
* Function Name: ifx_image_conv_RGB565_to_RGB888_u2i
*******************************************************************************
* Summary:
*  Converts an entire image from RGB565 format (16 bits per pixel) to RGB888
*  format (24 bits per pixel) with a zero-point offset for int8_t output, using
*  SIMD optimizations for processing eight pixels simultaneously. If the source
*  image is wider than the destination, copies the center part and ignores side
*  portions. If the source image is shorter in height, processes only the 
*  minimum height.
*
* Parameters:
*  uint8_t *src_bgr565: Pointer to the source RGB565 image buffer
*  int32_t width: Width of the source image
*  int32_t height: Height of the source image
*  int8_t *dst_rgb888_i8: Pointer to the destination RGB888 image buffer (int8_t)
*  int32_t dst_width: Width of the destination image
*  int32_t dst_height: Height of the destination image
*  int32_t zero_point: Zero-point offset to apply to the output values
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGB565_to_RGB888_u2i( uint8_t *src_bgr565, int32_t width, int32_t height,
        int8_t *dst_rgb888_i8, int32_t dst_width, int32_t dst_height,
        int32_t zero_point )
{
    int min_height = min( height, dst_height );
    int min_width  = min( width,  dst_width );
    int offset_width = ( width - min_width ) >> 1;
    int blk_cnt      = width >> 3;
    int blk_remainder= width & 0x07;
    uint16_t   *src_bgr565_16p = (uint16_t *) &src_bgr565[offset_width << 1];   /* 2 bytes per pixel */
    int16x8_t   vmask_red_blue = vdupq_n_s16( 0x0F8 );
    int16x8_t   vmask_green    = vdupq_n_s16( 0x0FC );
    int16x8_t   vzero_point    = vdupq_n_s16( zero_point );

    for ( int i = 0; i < min_height; i++ ) {
        int8_t      *rgb888_8p  = &dst_rgb888_i8[i * dst_width * 3];   /* 3 bytes per pixel */
        uint16_t    *bgr565_16p = &src_bgr565_16p[i * width];

        for ( int jj = blk_cnt; jj > 0; jj--, bgr565_16p += 8 ) {
            /* process 8 16-bit RGB565 colors (pixels) simultaneously in the vector unit */
            uint16x8_t bgr565 = vldrhq_u16( bgr565_16p );                           /* load 8 16-bit RGB565 into a vector reg.*/
            int16x8_t  red    = vreinterpretq_s16_u16(vshrq_n_u16( bgr565, 8 ));    /* move R to the low-bytes */
            int16x8_t  green  = vreinterpretq_s16_u16(vshrq_n_u16( bgr565, 3 ));    /* move G to the low-bytes */
            int16x8_t  blue   = vreinterpretq_s16_u16(vshlq_n_u16( bgr565, 3 ));    /* adjust B to the low-bytes */

            /* clear not-relavant bits in each color vector */
            red   = vandq_s16( red,   vmask_red_blue );     /* --R3 | --R2 | --R1 | --R0 */
            green = vandq_s16( green, vmask_green );        /* --G3 | --G2 | --G1 | --G0 */
            blue  = vandq_s16( blue,  vmask_red_blue );     /* --B3 | --B2 | --B1 | --B0 */

            red   = vaddq_s16( vzero_point, red );
            green = vaddq_s16( vzero_point, green );
            blue  = vaddq_s16( vzero_point, blue );

            get_write_RGB_u2i(0);
            get_write_RGB_u2i(1);
            get_write_RGB_u2i(2);
            get_write_RGB_u2i(3);
            get_write_RGB_u2i(4);
            get_write_RGB_u2i(5);
            get_write_RGB_u2i(6);
            get_write_RGB_u2i(7);
        }
        if ( blk_remainder > 0 ) {
            for ( int jj = blk_remainder; jj > 0; jj-- ) {
                uint16_t    pixel = *bgr565_16p++;

                rgb888_8p = ifx_pixel_RGB565_to_RGB888_u2i( pixel, rgb888_8p, zero_point );
            }
        }
    }
}


#else

CY_SECTION_ITCM_BEGIN
/******************************************************************************
* Function Name: ifx_image_conv_RGB565_to_RGB888
*******************************************************************************
* Summary:
*  Converts an entire image from RGB565 format (16 bits per pixel) to RGB888
*  format (24 bits per pixel). If the source image is wider than the ,
*  destination copies the center part and ignores side portions. If the source 
*  image is shorter in height, processes only the minimum height. Processes 
*  two pixels at a time for efficiency.
*
* Parameters:
*  uint8_t*src_bgr565: Pointer to the source RGB565 image buffer
*  int32_t width: Width of the source image
*  int32_t height: Height of the source image
*  uint8_t*dst_rgb888: Pointer to the destination RGB888 image buffer
*  int32_t dst_width: Width of the destination image
*  int32_t dst_height: Height of the destination image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGB565_to_RGB888( uint8_t *src_bgr565, int32_t width, int32_t height,
        uint8_t *dst_rgb888, int32_t dst_width, int32_t dst_height )
{
    int min_height = min( height, dst_height );
    int min_width  = min( width,  dst_width );
    int offset_width = ( width - min_width ) >> 1;      /* positive offset, if the source is wider */
    uint16_t    *src_bgr565_16p = (uint16_t *) &src_bgr565[offset_width << 1];   /* 2 bytes per pixel */

    for ( int i = 0; i < min_height; i++ ) {
        uint8_t     *rgb888     = &dst_rgb888[i * dst_width * 3];   // 3 bytes per pixel
        uint32_t    *bgr565_32p = (uint32_t *)&src_bgr565_16p[i * width];

        for ( int j = (min_width >> 1); j > 0; j-- ) {
            // two-pixel RGB565 memory-access: one 32-bit read of 4-byte
            uint32_t    pixel = *bgr565_32p++;

#if 1
            rgb888 = ifx_pixel_2_RGB565_to_RGB888( pixel, rgb888 );
#else
            /* process low (15:0) first pixel */
            rgb888 = ifx_pixel_RGB565_to_RGB888( pixel, rgb888 );

            /* process high (31:16) second pixel */
            rgb888 = ifx_pixel_RGB565_to_RGB888( pixel >> 16, rgb888 );
#endif  /* 0/1 */
        }
    }
}
CY_SECTION_ITCM_END


/******************************************************************************
* Function Name: ifx_image_conv_RGB565_to_RGB888_u2i
*******************************************************************************
* Summary:
*  Converts an entire image from RGB565 format (16 bits per pixel) to RGB888
*  format (24 bits per pixel) with a zero-point offset for int8_t output. If 
*  the source image is wider than the destination, copies the center part and 
*  ignores side portions. If the source image is shorter in height, processes 
*  only the minimum height. Processes two pixels at a time for efficiency.
*
* Parameters:
*  uint8_t *src_bgr565: Pointer to the source RGB565 image buffer
*  int32_t width: Width of the source image
*  int32_t height: Height of the source image
*  int8_t *dst_rgb888_i8: Pointer to the destination RGB888 image buffer 
*                         (int8_t)
*  int32_t dst_width: Width of the destination image
*  int32_t dst_height: Height of the destination image
*  int32_t zero_point: Zero-point offset to apply to the output values
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGB565_to_RGB888_u2i( uint8_t *src_bgr565, int32_t width, int32_t height,
        int8_t *dst_rgb888_i8, int32_t dst_width, int32_t dst_height,
        int32_t zero_point )
{
    int min_height = min( height, dst_height );
    int min_width  = min( width, dst_width );
    int offset_width = ( width - min_width ) >> 1;
    uint16_t    *src_bgr565_16p = (uint16_t *) &src_bgr565[offset_width << 1];  /* 2 bytes per pixel */

    for ( int i = 0; i < min_height; i++ ) {
        int8_t      *rgb888     = &dst_rgb888_i8[i * dst_width * 3];    /* 3 bytes per pixel */
        uint32_t    *bgr565_32p = (uint32_t *)&src_bgr565_16p[i * width];

        for ( int j = (min_width >> 1); j > 0; j-- ) {
            /* two-pixel RGB565 memory-access: one 32-bit read of 4-byte */
            uint32_t    pixel = bgr565_32p++;

#if 1
            rgb888 = ifx_pixel_2_RGB565_to_RGB888_u2i( pixel, rgb888, zero_point );
#else
            /* process low (15:0) first pixel */
            rgb888 = ifx_pixel_RGB565_to_RGB888_u2i( pixel, rgb888, zero_point );

            /* process high (31:16) second pixel */
            rgb888 = ifx_pixel_RGB565_to_RGB888_u2i( pixel >> 16, rgb888, zero_point );
#endif  /* 0/1 */
        }
    }
}

#endif  /* ARM_MATH_MVEI */


CY_SECTION_ITCM_BEGIN
/******************************************************************************
* Function Name: ifx_image_conv_RGB888_to_RGB565
*******************************************************************************
* Summary:
*  Converts an entire image from RGB888 format (24 bits per pixel) to RGB565
*  format (16 bits per pixel). If the source image is wider or taller than the
*  destination, processes only the minimum width and height. Uses the
*  ifx_pixel_RGB888_to_RGB565 helper function for pixel conversion.
*
* Parameters:
*  uint8_t *src_rgb888: Pointer to the source RGB888 image buffer
*  int32_t width: Width of the source image
*  int32_t height: Height of the source image
*  uint8_t *dst_bgr565: Pointer to the destination RGB565 image buffer
*  int32_t dst_width: Width of the destination image
*  int32_t dst_height: Height of the destination image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGB888_to_RGB565(uint8_t *src_rgb888, int32_t width, int32_t height,
        uint8_t *dst_bgr565, int32_t dst_width, int32_t dst_height)
{
    int min_height = min(height, dst_height);
    int min_width = min(width, dst_width);
    uint16_t *dst_bgr565_16p = (uint16_t *)dst_bgr565;

    for (int i = 0; i < min_height; i++) {
        uint8_t *rgb888 = &src_rgb888[i * width * 3];
        uint16_t *bgr565_16p = &dst_bgr565_16p[i * dst_width];

        for (int j = 0; j < min_width; j++) {
            uint8_t r = *rgb888++;
            uint8_t g = *rgb888++;
            uint8_t b = *rgb888++;

            /* Convert RGB888 to BGR565 using the existing helper function */
            *bgr565_16p++ = ifx_pixel_RGB888_to_RGB565(r, g, b);
        }
    }
}
CY_SECTION_ITCM_END


/******************************************************************************
* Function Name: ifx_image_conv_RGB565_to_RGB888_quant
*******************************************************************************
* Summary:
*  Converts an entire image from RGB565 format (16 bits per pixel) to RGB888
*  format (24 bits per pixel) with quantization, applying a scale factor and
*  zero-point offset for int8_t output. If the source image is wider or taller
*  than the destination, processes only the minimum width and height.
*
* Parameters:
*  uint8_t *src_bgr565: Pointer to the source RGB565 image buffer
*  int32_t width: Width of the source image
*  int32_t height: Height of the source image
*  int8_t *dst_rgb888_i8: Pointer to the destination RGB888 image buffer 
*                         (int8_t)
*  int32_t dst_width: Width of the destination image
*  int32_t dst_height: Height of the destination image
*  float scale: Scale factor for quantization
*  int32_t zero_point: Zero-point offset to apply to the output values
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGB565_to_RGB888_quant( uint8_t *src_bgr565, int32_t width, int32_t height,
        int8_t *dst_rgb888_i8, int32_t dst_width, int32_t dst_height,
        float scale, int32_t zero_point )
{
    float   inv_scale    = 1.0f / scale;
    float   zero_point_f = (float) zero_point;
    uint8_t *dst_rgb888  = (uint8_t *) dst_rgb888_i8;

    ifx_image_conv_RGB565_to_RGB888( src_bgr565, width, height, dst_rgb888, dst_width, dst_height );

    for ( int ii = 0; ii < dst_width * dst_height; ii++ ) {
        *dst_rgb888_i8++ = (int8_t)( inv_scale * (*dst_rgb888++) + zero_point_f );
        *dst_rgb888_i8++ = (int8_t)( inv_scale * (*dst_rgb888++) + zero_point_f );
        *dst_rgb888_i8++ = (int8_t)( inv_scale * (*dst_rgb888++) + zero_point_f );
    }
}


/******************************************************************************
* Function Name: RGB565_to_RGB888_i8
*******************************************************************************
* Summary:
*  Converts a single RGB565 color pixel value to RGB888 format with a fixed
*  zero-point offset of -128 for int8_t output. Supports multiple conversion
*  modes:default (low bits set to 0), full 8-bit usage, or full 8-bit usage 
*  with rounding based on BGR888_FULL_8_BIT and BGR888_FULL_8_BIT_ROUND 
*  definitions.
*
* Parameters:
*  uint32_t pixel: 16-bit RGB565 color value
*  int8_t *rgb888_8p: Pointer to the destination RGB888 buffer (int8_t)
*
* Return:
*  int8_t*: Pointer to the next position in the RGB888 buffer
*
******************************************************************************/
inline int8_t * RGB565_to_RGB888_i8( uint32_t pixel, int8_t *rgb888_8p )
{
    /* Extract B, G, and R components from BGR565 pixel (already in BGR order) */
    uint8_t    blue   = (pixel << 3) & 0x0F8;            /* Extract lower 5 bits for blue */
    uint8_t    green  = (pixel >> 3) & 0x0FC;            /* Extract middle 6 bits for green, right shift 5 for 8-bit representation */
    uint8_t    red    = (pixel >> 8) & 0x0F8;            /* Extract upper 5 bits for red, right shift 11 for 8-bit representation */

#ifndef BGR888_FULL_8_BIT
    *rgb888_8p++ = red   - 128;
    *rgb888_8p++ = green - 128;
    *rgb888_8p++ = blue  - 128;
#else

#ifndef BGR888_FULL_8_BIT_ROUND
    *rgb888_8p++ = (red    | (red    >> 5))  - 128;
    *rgb888_8p++ = (green  | (green  >> 6))  - 128;
    *rgb888_8p++ = (blue   | (blue   >> 5))  - 128;
#else
    *rgb888_8p++ = (red    | (red    >> 5) | ((red   >> 4) & 0x01))  - 128;    /* (significant 5-bit) | (low 3-bit with high 3-bit) | (round with 4th high bit) */
    *rgb888_8p++ = (green  | (green  >> 6) | ((green >> 5) & 0x01))  - 128;    /* (significant 6-bit) | (low 2-bit with high 2-bit) | (round with 3rd high bit) */
    *rgb888_8p++ = (blue   | (blue   >> 5) | ((blue  >> 4) & 0x01))  - 128;    /* (significant 5-bit) | (low 3-bit with high 3-bit) | (round with 4th high bit) */
#endif  /* BGR888_FULL_8_BIT_ROUND */

#endif  /* BGR888_FULL_8_BIT */

    return rgb888_8p;
}


/******************************************************************************
* Function Name: ifx_image_conv_RGB565_to_RGB888_i8
*******************************************************************************
* Summary:
*  Converts an entire image from RGB565 format (16 bits per pixel) to RGB888
*  format (24 bits per pixel) with a fixed zero-point offset of -128 for 
*  int8_t output. If the source image is wider than the destination, copies 
*  the center part and ignores side portions. Supports 32-bit or 16-bit memory 
*  access based on BGR565_32bit_MEMORY_ACCESS definition.
*
* Parameters:
*  uint8_t *src_bgr565: Pointer to the source RGB565 image buffer
*  int width: Width of the source image
*  int height: Height of the source image
*  int8_t *dst_rgb888_i8: Pointer to the destination RGB888 image buffer 
*                         (int8_t)
*  int dst_width: Width of the destination image
*  int dst_height: Height of the destination image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGB565_to_RGB888_i8( uint8_t *src_bgr565, int width, int height,
        int8_t *dst_rgb888_i8, int dst_width, int dst_height )
{
    int min_height = min( height, dst_height );
    int min_width  = min( width, dst_width );
    int offset_width = ( width - min_width ) / 2;

    for ( int i = 0; i < min_height; i++ ) {
        int8_t  *rgb888_8p = &dst_rgb888_i8[i * dst_width * 3];
        uint8_t *bgr565_8p = &src_bgr565[(i * width + offset_width) * 2];
#ifdef BGR565_32bit_MEMORY_ACCESS    /* 32-bit two-pixel memory-access */
        uint32_t    *bgr565_32p = (uint32_t *)bgr565_8p;

        for ( int j = 0; j < min_width / 2; j++ ) {
            uint32_t    pixel = *bgr565_32p++;

            /* process low (15:0) first pixel */
            rgb888_8p = RGB565_to_RGB888_i8( pixel, rgb888_8p );

            /* process high (31:16) second pixel */
            rgb888_8p = RGB565_to_RGB888_i8( pixel >> 16, rgb888_8p );
        }
#else    /* 16-bit one-pixel memory-access */
        uint16_t    *bgr565_16p = (uint16_t *)bgr565_8p;
        for ( int j = 0; j < min_width; j++ ) {
            uint16_t pixel = *bgr565_16p++;

            rgb888_8p = RGB565_to_RGB888_i8( pixel, rgb888_8p );
        }
#endif    /* BGR565_32bit_MEMORY_ACCESS */
    }
}


/******************************************************************************
* Function Name: ifx_pixel_RGB888_to_RGBX32
*******************************************************************************
* Summary:
*  Converts RGB888 color values (8 bits per channel) to a 32-bit RGBX format,
*  combining red, green, and blue components into a single 32-bit value with
*  an unused byte.
*
* Parameters:
*  uint32_t r: Red color value (0-255)
*  uint32_t g: Green color value (0-255)
*  uint32_t b: Blue color value (0-255)
*
* Return:
*  uint32_t: 32-bit RGBX color value
*
******************************************************************************/
inline uint32_t ifx_pixel_RGB888_to_RGBX32( uint32_t r, uint32_t g, uint32_t b )
{
    return ((r << 16) | (g << 8) | b);
}


/******************************************************************************
* Function Name: ifx_pixel_xRGB32_to_RGB565
*******************************************************************************
* Summary:
*  Converts a 32-bit RGBX color value to RGB565 format (16 bits: 5 bits red,
*  6 bits green, 5 bits blue).
*
* Parameters:
*  uint32_t xRGB: 32-bit RGBX color value
*
* Return:
*  uint16_t: 16-bit RGB565 color value
*
******************************************************************************/
inline uint16_t ifx_pixel_xRGB32_to_RGB565( uint32_t xRGB )
{
    uint16_t    r = (xRGB >> 8) & 0xF800u;
    uint16_t    g = (xRGB >> 5) & 0x07E0u;
    uint16_t    b = (xRGB >> 3) & 0x001Fu;

    return (uint16_t)(r | g | b);
}


/******************************************************************************
* Function Name: ifx_image_conv_RGBX32_to_RGB24
*******************************************************************************
* Summary:
*  Converts an entire image from RGBX32 format (32 bits per pixel) to RGB888
*  format (24 bits per pixel). If the source image is wider than the 
*  destination,copies the center part and ignores side portions. If the 
*  source image is shorter in height, processes only the minimum height.
*
* Parameters:
*  uint8_t *src_rgbx32: Pointer to the source RGBX32 image buffer
*  int32_t width: Width of the source image
*  int32_t height: Height of the source image
*  uint8_t *dst_rgb888: Pointer to the destination RGB888 image buffer
*  int32_t dst_width: Width of the destination image
*  int32_t dst_height: Height of the destination image
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGBX32_to_RGB24( uint8_t *src_rgbx32, int32_t width, int32_t height,
        uint8_t *dst_rgb888, int32_t dst_width, int32_t dst_height )
{
    int min_height = min( height, dst_height );
    int min_width  = min( width, dst_width );
    int offset_width = ( width - min_width ) >> 1;
    uint32_t    *src_32p = (uint32_t *)&src_rgbx32[offset_width << 1];

    for ( int i = 0; i < min_height; i++ ) {
        uint8_t     *dp = &dst_rgb888[i * dst_width * 3];
        uint32_t    *sp = &src_32p[i * width];

        for ( int j = 0; j < min_width; j++ ) {
            uint32_t    pixel = *sp++;

            *dp++ = (uint8_t)(pixel >> 16);
            *dp++ = (uint8_t)(pixel >>  8);
            *dp++ = (uint8_t)(pixel);
        }
    }
}


/******************************************************************************
* Function Name: ifx_image_conv_RGBX32_to_RGB24_u2i
*******************************************************************************
* Summary:
*  Converts an entire image from RGBX32 format (32 bits per pixel) to RGB888
*  format (24 bits per pixel) with a zero-point offset for int8_t output. If 
*  the source image is wider than the destination, copies the center part and 
*  ignores side portions. If the source image is shorter in height, processes 
*  only the minimum height.
*
* Parameters:
*  uint8_t *src_rgbx32: Pointer to the source RGBX32 image buffer
*  int32_t width: Width of the source image
*  int32_t height: Height of the source image
*  int8_t *dst_rgb888: Pointer to the destination RGB888 image buffer (int8_t)
*  int32_t dst_width: Width of the destination image
*  int32_t dst_height: Height of the destination image
*  int32_t zero_point: Zero-point offset to apply to the output values
*
* Return:
*  void
*
******************************************************************************/
void ifx_image_conv_RGBX32_to_RGB24_u2i( uint8_t *src_rgbx32, int32_t width, int32_t height,
        int8_t  *dst_rgb888, int32_t dst_width, int32_t dst_height,
        int32_t zero_point )
{
    int min_height = min( height, dst_height );
    int min_width  = min( width, dst_width );
    int offset_width = ( width - min_width ) >> 1;
    uint32_t    *src_32p = (uint32_t *)&src_rgbx32[offset_width << 1];

    for ( int i = 0; i < min_height; i++ ) {
        int8_t      *dp = &dst_rgb888[i * dst_width * 3];
        uint32_t    *sp = &src_32p[i * width];

        for ( int j = 0; j < min_width; j++ ) {
            uint32_t    pixel = *sp++;

            *dp++ = (int8_t)((pixel >> 16) + zero_point);
            *dp++ = (int8_t)((pixel >>  8) + zero_point);
            *dp++ = (int8_t)((pixel      ) + zero_point);
        }
    }
}


/******************************************************************************
* Function Name: draw_image_pixel
*******************************************************************************
* Summary:
*  Draws a single pixel on an LCD display by converting RGB888 color values to
*  RGB565 format and writing to the specified position in the destination 
*  buffer, provided the coordinates are within the LCD dimensions.
*
* Parameters:
*  uint16_t *pDst: Pointer to the destination LCD buffer (RGB565)
*  uint16_t x: X-coordinate of the pixel
*  uint16_t y: Y-coordinate of the pixel
*  uint16_t r: Red color value (0-255)
*  uint16_t g: Green color value (0-255)
*  uint16_t b: Blue color value (0-255)
*  uint16_t lcd_w: Width of the LCD display
*  uint16_t lcd_h: Height of the LCD display
*
* Return:
*  void
*
******************************************************************************/
void draw_image_pixel( uint16_t *pDst, uint16_t x, uint16_t y, uint16_t r, uint16_t g, uint16_t b, uint16_t lcd_w, uint16_t lcd_h )
{
    if ( x < lcd_w  &&  y < lcd_h ) {
        uint16_t rgb565 = ifx_pixel_RGB888_to_RGB565(r, g, b);

        pDst[(y * lcd_w) + x] = rgb565;
    }
}

/* [] END OF FILE */
