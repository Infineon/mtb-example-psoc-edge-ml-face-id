/******************************************************************************
* File Name        : lcd_task.h
*
* Description      : This is the header file of LCD display functions.
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

#ifndef _LCD_TASK_H_
#define _LCD_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "vg_lite.h"
#include "vg_lite_platform.h"
#include "ifx_face_id.h"
#include "mtb_disp_dsi_waveshare_4p3.h"
#include "oob_uart_cmd.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define NUM_IMAGE_BUFFERS                    (2U)

#define GPU_INT_PRIORITY                    (4U)
#define DC_INT_PRIORITY                     (4U)
#define I2C_CONTROLLER_IRQ_PRIORITY         (4U)
#define COLOR_DEPTH                         (16U)
#define BITS_PER_PIXEL                      (8U)
#define VG_PARAMS_POS                       (0UL)

/* 64 KB */
#define DEFAULT_GPU_CMD_BUFFER_SIZE         ((64U) * (1024U))
#define GPU_TESSELLATION_BUFFER_SIZE        ((MTB_DISP_WAVESHARE_4P3_VER_RES) * 128U)
#define FRAME_BUFFER_SIZE                   ((MTB_DISP_WAVESHARE_4P3_HOR_RES) * (MTB_DISP_WAVESHARE_4P3_VER_RES) * ((COLOR_DEPTH) / (BITS_PER_PIXEL)))
#define VGLITE_HEAP_SIZE                    (((FRAME_BUFFER_SIZE) * (3)) + \
                                             (((DEFAULT_GPU_CMD_BUFFER_SIZE) + (GPU_TESSELLATION_BUFFER_SIZE)) * (NUM_IMAGE_BUFFERS)) + \
                                             ((CAMERA_BUFFER_SIZE) * (NUM_IMAGE_BUFFERS + 1)))
#define GPU_MEM_BASE                        (0x0U)
#define WHITE_COLOR                         (0x00FFFFFFU)
#define BLACK_COLOR                         (0x00000000U)

/* Touch and UI constants */
#define TOUCH_TIMER_PERIOD_MS               (150U)
#define DEBOUNCE_TIME_MS                    (200U)
#define LCD_TEXT_TOP_MARGIN                 (10U)
#define HOME_BTN_OFFSET                     (40U)
#define FACE_ENROL_BTN_OFFSET               (10U)
#define FACE_ENROL_BTN_Y_OFFSET             (30U)
#define CANCEL_BTN_Y_OFFSET                 (70U)
#define CLEAR_BTN_Y_OFFSET                  (70U)
#define FPS_TXT_Y_OFFSET                    (30U)
#define NO_CAMERA_IMG_OFFSET                (10U)
#define CAMERA_NOT_SUPPORTED_IMG_OFFSET     (10U)


/* Color array size */
#define COLOR_ARRAY_SIZE                    (8U)

/* Database overlay constants */
#define DATABASE_FULL_OVERLAY_DURATION_MS   (5000.0f)

/* Grid layout constants */
#define GRID_SIZE                           (60U)
#define GRID_SPACING                        (15U)
#define TEXT_OFFSET_Y                       (24U)
#define TEXT_OFFSET_X                       (5U)
#define CHAR_STRING_LEN                     (64U)
#define LINE_HEIGHT                         (20U)
#define LEGEND_SQUARE_SIZE                  (15U)
#define LEGEND_TEXT_OFFSET                  (20U)
#define LEGEND_Y_OFFSET                     (2U)
#define TITLE_X_OFFSET                      (80U)
#define TITLE_Y_OFFSET                      (35U)

/* Image processing constants */
#define BYTES_PER_PIXEL                     (2U)
#define RGB888_BYTES_PER_PIXEL              (3U)

/* Reset/Initialization value macros */
#define RESET_VALUE_INDEX                   (0U)

/* Hex value macros */
#define HEX_MEMORY_FILL_VALUE               (0xff)
#define HEX_RED_5BIT_MASK                   (0x1F)
#define HEX_GREEN_6BIT_MASK                 (0x3F)
#define HEX_BLUE_5BIT_MASK                  (0x1F)
#define HEX_RED_BACKGROUND                  (0xFF)
#define HEX_BIT_SHIFT_3                     (3)
#define HEX_BIT_SHIFT_2                     (2)
#define HEX_BIT_SHIFT_11                    (11)
#define HEX_BIT_SHIFT_5                     (5)

/* Color value macros */
#define COLOR_WHITE                         (255)
#define COLOR_RED                           (255)
#define COLOR_GREEN                         (255)
#define COLOR_BLUE                          (255)
#define COLOR_YELLOW                        (255)
#define COLOR_CYAN                          (255)
#define COLOR_PURPLE                        (255)
#define COLOR_BLACK                         (0)
#define COLOR_ORANGE                        (255)
#define COLOR_DARK_RED                      (128)
#define COLOR_DARK_GREEN                    (128)
#define COLOR_DARK_BLUE                     (128)
#define COLOR_DARK_GRAY                     (64)

/* Numeric value macros */
#define NUM_DOUBLE_BUFFERS                  (2)
#define NUM_TEMP_ARRAY_SIZE                 (4)
#define NUM_STRING_BUFFER_SIZE              (32)
#define NUM_FONT_DIVISOR_16                 (16)
#define NUM_FONT_DIVISOR_18                 (18)
#define NUM_FONT_OFFSET_1                   (1)
#define NUM_FONT_INDEX_0                    (0)
#define NUM_LOOP_START                      (0)
#define NUM_LOOP_INCREMENT_1                (1)
#define NUM_STREAM_STOPPED                  (1)
#define NUM_THROW_AWAY_FRAME                (1)
#define NUM_BUF_READY_FLAG                  (1)
#define NUM_DISPLAY_OFFSET_40               (40)
#define NUM_DISPLAY_OFFSET_120              (120)
#define NUM_DISPLAY_OFFSET_15               (15)
#define NUM_DISPLAY_OFFSET_11               (11)
#define NUM_DISPLAY_OFFSET_10               (10)
#define NUM_DISPLAY_OFFSET_20               (20)
#define NUM_DISPLAY_OFFSET_5                (5)
#define NUM_DISPLAY_OFFSET_50               (50)
#define NUM_DISPLAY_OFFSET_3                (3)
#define NUM_DISPLAY_OFFSET_2                (2)
#define NUM_DISPLAY_OFFSET_1                (1)
#define NUM_DISPLAY_OFFSET_0                (0)

/* Additional numeric value macros */
#define NUM_ONE_DETECTION_WIDTH             (320)
#define NUM_ONE_DETECTION_HEIGHT            (240)
#define NUM_ONE_DETECTION_SIZE_MULTIPLIER   (3)
#define NUM_DISPLAY_DIVISOR_3               (3)
#define NUM_PLOT_ALIGNED_X                  (5)
#define NUM_PLOT_ALIGNED_Y                  (50)
#define NUM_WATCHDOG_TIMEOUT_MS             (10000.0f)
#define NUM_DISPLAY_OFFSET_32               (32)
#define NUM_TOUCH_OFFSET_10                 (10)
#define NUM_TOUCH_OFFSET_5                  (5)
#define NUM_CY_ASSERT_VALUE                 (0)
#define NUM_TIMER_START_VALUE               (0)
#define NUM_LOOP_INCREMENT_1                (1)
#define NUM_DISPLAY_OFFSET_1                (1)
#define NUM_GRID_ROWS_1                     (1)
#define NUM_GRID_COLS_3                     (3)
#define NUM_TIMEOUT_MS_5000                 (5000)
#define NUM_CAMERA_ENABLED                  (1)
#define NUM_CAMERA_DISABLED                 (0)
#define NUM_DEVICE_DISCONNECTED             (0)
#define NUM_FOUND_FLAG                      (0)
#define NUM_GUID_INDEX_0                    (0)
#define NUM_FRAME_INTERVAL_INDEX            (0)
#define NUM_TERMINAL_UNIT_INDEX             (0)

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern vg_lite_buffer_t    *renderTarget;
extern vg_lite_buffer_t    usb_yuv_frames[];

/* we will use these to check vs build display option for correct params */
extern cy_stc_gfx_dc_config_t GFXSS_dc_config;
extern cy_stc_mipidsi_display_params_t GFXSS_mipidsi_display_params;
extern bool fid_home_start_enrollment_flag;
extern bool fid_home_cancel_enrollment_flag;
extern bool fid_home_clear_enrolled_users_flag;

extern volatile uint8_t _device_connected;
extern bool camera_not_supported;

/*******************************************************************************
* Data Structures and Types
*******************************************************************************/

/* Define enrolment types */
typedef enum {
    ENROLLMENT_TYPE_NONE,          /* No enrolment active */
    ENROLLMENT_TYPE_ON_DEVICE,     /* On-device enrolment active */
    ENROLLMENT_TYPE_OFF_DEVICE     /* Off-device enrolment via UART active */
} enrollment_type_t;

/* Track display mode */
typedef enum {
    DISPLAY_MODE_INFERENCE,       /* Regular inference without tracking */
    DISPLAY_MODE_ENROLLMENT    /* On-device enrolment mode */
} display_mode_t;

typedef enum {
    FACE_ENROL_BUTTON_ID = 0,
    CANCEL_ENROL_BUTTON_ID,
    CLEAR_ENROL_BUTTON_ID,
    BUTTON_ID_MAX
} button_id_t;

typedef enum {
    TOUCH_BUTTON_EVENT_RELEASED = 0,
    TOUCH_BUTTON_EVENT_PRESSED,
} touch_button_event_type_t;

/* Define enrolment states for face capture process */
typedef enum {
    ENROLLMENT_STATE_NONE,                /* No enrolment in progress */
    ENROLLMENT_STATE_WAITING_FOR_REFERENCE, /* Waiting for reference face */
    ENROLLMENT_STATE_COLLECTING,          /* Collecting face samples */
    ENROLLMENT_STATE_COMPLETING,          /* Processing and saving enrolment */
    ENROLLMENT_STATE_UART_TRANSFER        /* Special state for off-device UART transfer */
} enrollment_state_t;

typedef struct {
    bool pressed;
    bool released;
    touch_button_event_type_t event;
} touch_button_event_t;

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _LCD_TASK_H_ */

/* [] END OF FILE */
