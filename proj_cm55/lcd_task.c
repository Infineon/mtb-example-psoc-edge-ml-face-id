/*******************************************************************************
* File Name        : lcd_task.c
*
* Description      : This file implements the LCD display modules for Face ID
*                    demo.
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

/*******************************************************************************
* Header Files
*******************************************************************************/
#include "cybsp.h"
#include "lcd_task.h"
#include "FreeRTOS.h"
#include "task.h"
#include "FreeRTOSConfig.h"
#include "cy_graphics.h"
#include "ifx_face_id.h"
#include "lcd_graphics.h"
#include "ifx_image_utils.h"
#include "ifx_time_utils.h"
#include "usb_camera_task.h"
#include "lcd_task.h"
#include "vg_lite.h"
#include "mtb_ctp_ft5406.h"
#include "start_face_enrolment_btn_img.h"
#include "save_face_enrolment_btn_img.h"
#include "cancel_released_btn_img.h"
#include "cancel_pressed_btn_img.h"
#include "clear_released_btn_img.h"
#include "clear_pressed_btn_img.h"
#include "no_camera_img.h"
#include "camera_not_supported_img.h"

#ifdef USE_BIGGER_FONT
#include "ifx_gui_render.h"
#include "oob_uart_cmd.h"

#endif /* USE_BIGGER_FONT */


/*******************************************************************************
* Macros
*******************************************************************************/
#ifdef TEST_SAVED_IMAGE
#include "one_detection.h"   /* Contains static uint8_t test_image_array[size] */
#define ONE_DETECTION_WIDTH  NUM_ONE_DETECTION_WIDTH    /* Your test image width */
#define ONE_DETECTION_HEIGHT NUM_ONE_DETECTION_HEIGHT   /* Your test image height */
#define ONE_DETECTION_SIZE   (ONE_DETECTION_WIDTH * ONE_DETECTION_HEIGHT * NUM_ONE_DETECTION_SIZE_MULTIPLIER)
#endif

/* Display I2C controller */
#ifdef USE_KIT_PSE84_AI
#define DISPLAY_I2C_CONTROLLER_HW     CYBSP_I2C_DISPLAY_CONTROLLER_HW
#define DISPLAY_I2C_CONTROLLER_IRQ    CYBSP_I2C_DISPLAY_CONTROLLER_IRQ
#define DISPLAY_I2C_CONTROLLER_config CYBSP_I2C_DISPLAY_CONTROLLER_config
#else
#define DISPLAY_I2C_CONTROLLER_HW     CYBSP_I2C_CONTROLLER_HW
#define DISPLAY_I2C_CONTROLLER_IRQ    CYBSP_I2C_CONTROLLER_IRQ
#define DISPLAY_I2C_CONTROLLER_config CYBSP_I2C_CONTROLLER_config
#endif

#define HOME_BTN_X_POS                 (MTB_DISP_WAVESHARE_4P3_HOR_RES \
                                            - HOME_BUTTON_IMG_WIDTH - HOME_BTN_OFFSET)
#define HOME_BTN_Y_POS                 (LCD_TEXT_TOP_MARGIN)
#define FACE_ENROL_BTN_X_POS           (FACE_ENROL_BTN_OFFSET)
#define FACE_ENROL_BTN_Y_POS           ((MTB_DISP_WAVESHARE_4P3_VER_RES / NUM_DISPLAY_DIVISOR_3) + FACE_ENROL_BTN_Y_OFFSET)
#define CANCEL_BTN_X_POS               (FACE_ENROL_BTN_OFFSET)
#define CANCEL_BTN_Y_POS               (FACE_ENROL_BTN_Y_POS + CANCEL_BTN_Y_OFFSET + FACE_ENROL_BTN_Y_OFFSET)
#define CLEAR_BTN_X_POS                (FACE_ENROL_BTN_OFFSET)
#define CLEAR_BTN_Y_POS                (CANCEL_BTN_Y_POS + CLEAR_BTN_Y_OFFSET + FACE_ENROL_BTN_Y_OFFSET)
#define NO_CAMERA_IMG_X_POS            ((MTB_DISP_WAVESHARE_4P3_HOR_RES / 2U) \
                                            - ((NO_CAMERA_IMG_WIDTH / 2U) + NO_CAMERA_IMG_OFFSET))
#define NO_CAMERA_IMG_Y_POS            ((MTB_DISP_WAVESHARE_4P3_VER_RES / 2U) \
                                            - (NO_CAMERA_IMG_HEIGHT / 2U))
#define CAMERA_NOT_SUPPORTED_IMG_X_POS ((MTB_DISP_WAVESHARE_4P3_HOR_RES / 2U) \
                                            - ((CAMERA_NOT_SUPPORTED_IMG_WIDTH / 2U) + CAMERA_NOT_SUPPORTED_IMG_OFFSET))
#define CAMERA_NOT_SUPPORTED_IMG_Y_POS ((MTB_DISP_WAVESHARE_4P3_VER_RES / 2U) \
                                            - (CAMERA_NOT_SUPPORTED_IMG_HEIGHT / 2U))
#define FPS_TXT_Y_POS                  (MTB_DISP_WAVESHARE_4P3_VER_RES - FPS_TXT_Y_OFFSET)

#define DISPLAY_W                       (MTB_DISP_WAVESHARE_4P3_HOR_RES)
#define DISPLAY_H                       (MTB_DISP_WAVESHARE_4P3_VER_RES)

#define PLOT_ALIGNED_X                  (NUM_PLOT_ALIGNED_X)
#define PLOT_ALIGNED_Y                  (NUM_PLOT_ALIGNED_Y)

/*******************************************************************************
* Global Variables
*******************************************************************************/
vg_lite_buffer_t    *renderTarget; /* used by lcd_bsp.c */
vg_lite_buffer_t    usb_yuv_frames[NUM_IMAGE_BUFFERS]; /* used by usb_camera_task.c */

cy_stc_scb_i2c_context_t disp_i2c_controller_context;
vg_lite_buffer_t home_button_buffer;
vg_lite_matrix_t home_button_matrix;

static vg_lite_matrix_t    matrix;
static cy_stc_gfx_context_t gfx_context;
static vg_lite_buffer_t    bgr565;
static vg_lite_buffer_t    display_buffer[NUM_DOUBLE_BUFFERS];      /* double display frame buffers */

static GFXSS_Type    *base = (GFXSS_Type*) GFXSS;

static enrollment_state_t enrollment_state = ENROLLMENT_STATE_NONE;
static enrollment_type_t enrollment_type = ENROLLMENT_TYPE_NONE;

/* Heap memory for VGLite to allocate memory for buffers, command, and tessellation buffers */
CY_SECTION(".cy_xip") __attribute__((aligned(128), used))
        uint8_t contiguous_mem[VGLITE_HEAP_SIZE] = {HEX_MEMORY_FILL_VALUE};
volatile void *vglite_heap_base = &contiguous_mem;

CY_SECTION(".cy_socmem_data")
__attribute__((aligned(64)))
static uint8_t Image_buf_bgr565[CAMERA_BUFFER_SIZE] = {RESET_VALUE_INDEX};

CY_SECTION(".cy_socmem_data")
__attribute__((aligned(64)))
static uint8_t  Image_buf_bgr888[(IMAGE_HEIGHT) * (IMAGE_WIDTH) * RGB888_BYTES_PER_PIXEL] = {RESET_VALUE_INDEX};

/* scale factor from the camera image to the display */
static float    scale_Cam2Disp;
static int      display_offset_x = RESET_VALUE_INDEX;
static int      display_offset_y = RESET_VALUE_INDEX;

/* 8 colors: red, green, blue, yellow, cyan, purple, white, black */
static uint8_t    color_r[COLOR_ARRAY_SIZE] = {COLOR_RED,   COLOR_BLACK, COLOR_BLACK, COLOR_RED,   COLOR_BLACK, COLOR_RED,   COLOR_RED,   COLOR_BLACK};
static uint8_t    color_g[COLOR_ARRAY_SIZE] = {COLOR_BLACK, COLOR_GREEN, COLOR_BLACK, COLOR_GREEN, COLOR_GREEN, COLOR_BLACK, COLOR_GREEN, COLOR_BLACK};
static uint8_t    color_b[COLOR_ARRAY_SIZE] = {COLOR_BLACK, COLOR_BLACK, COLOR_BLUE,  COLOR_BLACK, COLOR_BLUE,  COLOR_BLUE,  COLOR_BLUE,  COLOR_BLACK};

#ifdef SHOW_INFERENCING_TIME
static float    model_inferencing_time = RESET_VALUE_INDEX;
#else
static float    fps = RESET_VALUE_INDEX;
#endif /* SHOW_INFERENCING_TIME */

static float    Time_getInputImage;
static float    Prep_Wait_Buf = RESET_VALUE_INDEX;
static float    Prep_YUV422_to_bgr565 = RESET_VALUE_INDEX;
static float    Prep_bgr565_to_Disp = RESET_VALUE_INDEX;
static float    Prep_RGB565_to_RGB888 = RESET_VALUE_INDEX;

/* USB race condition recovery mode watchdog variables */
static float last_successful_frame_time = RESET_VALUE_INDEX;
static float watchdog_timeout = NUM_WATCHDOG_TIMEOUT_MS; /* milliseconds */
static int recovery_attempts = RESET_VALUE_INDEX;

/* Static Variables for Database Status (Merged System) */
static uint8_t current_user_count = RESET_VALUE_INDEX;
static bool database_full_overlay_flag = RESET_VALUE_INDEX;
static float database_full_overlay_timestamp = RESET_VALUE_INDEX;
/* Show overlay for 5 seconds */
static const float DATABASE_FULL_OVERLAY_DURATION_MS_CONST = DATABASE_FULL_OVERLAY_DURATION_MS;

#ifdef PLOT_ALIGNED_FACES
static aligned_faces_t aligned_faces;
#endif /* PLOT_ALIGNED_FACES*/

static ifx_faceid_embeddings_t faceid_embeddings;

volatile bool button_debouncing = false;
volatile uint32_t button_debounce_timestamp = RESET_VALUE_INDEX;

/* place in non-volatile flash memory */
CY_SECTION(".cy_nvm_user_data") __attribute__((used))ifx_faceid_embeddings_t enrolled_user_faceid_embeddings;

/* DC IRQ Config */
const cy_stc_sysint_t dc_irq_cfg =
{
        .intrSrc = GFXSS_DC_IRQ,
        .intrPriority = DC_INT_PRIORITY
};

/* GPU IRQ Config */
const cy_stc_sysint_t gpu_irq_cfg =
{
        .intrSrc = GFXSS_GPU_IRQ,
        .intrPriority = GPU_INT_PRIORITY
};


const cy_stc_sysint_t i2c_controller_irq_cfg =
{
        .intrSrc      = DISPLAY_I2C_CONTROLLER_IRQ,
        .intrPriority = I2C_CONTROLLER_IRQ_PRIORITY,
};

mtb_ctp_ft5406_config_t ft5406_config =
{
        .i2c_base    = DISPLAY_I2C_CONTROLLER_HW,
        .i2c_context = &disp_i2c_controller_context
};

touch_button_event_t touch_button_event[BUTTON_ID_MAX];

/*******************************************************************************
* Functions
*******************************************************************************/

/*******************************************************************************
* Function Name: mirrorImage
********************************************************************************
* Summary:
*  Mirrors an image horizontally by swapping pixels across the width of the image
*  buffer, assuming a fixed bytes-per-pixel format.
*
* Parameters:
*  vg_lite_buffer_t *buffer: Pointer to the image buffer to be mirrored
*
* Return:
*  void
*
*******************************************************************************/
static void mirrorImage(vg_lite_buffer_t *buffer) {
    uint8_t temp[NUM_TEMP_ARRAY_SIZE];
    uint8_t *start, *end;
    int m, n;
    int bytes_per_pixel  = RESET_VALUE_INDEX ;

    bytes_per_pixel = BYTES_PER_PIXEL;

    for (m = NUM_LOOP_START; m < CAMERA_HEIGHT ; m++) {

        start = buffer->memory + m * (CAMERA_WIDTH * bytes_per_pixel);

        end = start + (CAMERA_WIDTH - NUM_LOOP_INCREMENT_1) * bytes_per_pixel;


        for (n = NUM_LOOP_START; n < CAMERA_WIDTH / NUM_DISPLAY_OFFSET_2; n++) {

            for (int i = NUM_LOOP_START; i < bytes_per_pixel; i++) {
                temp[i] = start[i];
            }

            for (int i = NUM_LOOP_START; i < bytes_per_pixel; i++) {
                start[i] = end[i];
            }

            for (int i = NUM_LOOP_START; i < bytes_per_pixel; i++) {
                end[i] = temp[i];
            }

            start += bytes_per_pixel;
            end -= bytes_per_pixel;
        }
    }
}


/*******************************************************************************
* Function Name: mirrorImage
********************************************************************************
* Summary:
*  Update the current user count from the database.
*
* Parameters:
*  void
*
* Return:
*  ifx_faceid_rslt_t Status of the operation
*
*******************************************************************************/
static ifx_faceid_rslt_t update_database_user_count(void) {
    ifx_faceid_rslt_t result = ifx_face_id_get_enrolled_user_count(&faceid_embeddings, &current_user_count);
    if (result != IFX_FACEID_RSLT_SUCCESS) {
        printf("[LCD_TASK][ERROR] Failed to get user count: %d\n", result);
        current_user_count = RESET_VALUE_INDEX; /* Default to 0 if we can't check */
    }
    return result;
}


/********************************************************************************
* Function Name: is_database_full
*********************************************************************************
* Summary:
*  Check if database is currently at capacity.
*
* Parameters:
*  void
*
* Return:
*  bool
*
********************************************************************************/
static bool is_database_full(void)
{
    return (current_user_count >= MAX_N_USER);
}


/********************************************************************************
* Function Name: trigger_database_full_overlay
*********************************************************************************
* Summary:
*  Trigger the temporary overlay warning message
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void trigger_database_full_overlay(void) {
    database_full_overlay_flag = true;
    database_full_overlay_timestamp = ifx_time_get_ms_f();
    printf("[LCD_TASK] Database full overlay triggered - will display warning\n");
}


/********************************************************************************
* Function Name: should_show_database_full_overlay
*********************************************************************************
* Summary:
*  Check if database full overlay should still be displayed
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static bool should_show_database_full_overlay(void) {
    if (!database_full_overlay_flag) {
        return false;
    }

    float current_time = ifx_time_get_ms_f();
    float elapsed = current_time - database_full_overlay_timestamp;

    if (elapsed >= DATABASE_FULL_OVERLAY_DURATION_MS_CONST) {
        database_full_overlay_flag = false; /* Clear flag after timeout */
        return false;
    }

    return true;
}


/********************************************************************************
* Function Name: display_database_status
*********************************************************************************
* Summary:
*  Display database status in top-left area (always visible)
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void display_database_status(void) {
    /* Always show current user count */
    ifx_lcd_set_FGcolor(COLOR_WHITE, COLOR_WHITE, COLOR_WHITE); /* White text */
    ifx_lcd_set_BGcolor(COLOR_BLACK, COLOR_BLACK, COLOR_BLACK);       /* Black background */
    ifx_lcd_printf(((DISPLAY_W - NUM_DISPLAY_OFFSET_11) / NUM_DISPLAY_OFFSET_2 - NUM_DISPLAY_OFFSET_120), LCD_TEXT_TOP_MARGIN, "USERS: %d/%d", current_user_count, MAX_N_USER);

    /* Show capacity warning if close to full or full */
    if (is_database_full()) {
        ifx_lcd_set_FGcolor(COLOR_RED, COLOR_BLACK, COLOR_BLACK);     /* Red text */
        ifx_lcd_set_BGcolor(COLOR_BLACK, COLOR_BLACK, COLOR_BLACK);       /* Black background */
        ifx_lcd_printf(((DISPLAY_W - NUM_DISPLAY_OFFSET_15) / NUM_DISPLAY_OFFSET_2), LCD_TEXT_TOP_MARGIN, "DATABASE FULL!");


        if (TOUCH_BUTTON_EVENT_PRESSED == touch_button_event[FACE_ENROL_BUTTON_ID].event)
        {
            touch_button_event[FACE_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_RELEASED;
            touch_button_event[FACE_ENROL_BUTTON_ID].pressed = false;
        }

    } else if (current_user_count >= MAX_N_USER - NUM_LOOP_INCREMENT_1) {
        ifx_lcd_set_FGcolor(COLOR_YELLOW, COLOR_YELLOW, COLOR_BLACK);   /* Yellow text */
        ifx_lcd_set_BGcolor(COLOR_BLACK, COLOR_BLACK, COLOR_BLACK);       /* Black background */
        ifx_lcd_printf(((DISPLAY_W - NUM_DISPLAY_OFFSET_15) / NUM_DISPLAY_OFFSET_2), LCD_TEXT_TOP_MARGIN, "ALMOST FULL");
    }
}


/********************************************************************************
* Function Name: display_database_full_overlay
*********************************************************************************
* Summary:
*  Display database full warning overlay (temporary, center screen)
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void display_database_full_overlay(void) {
    /* Display prominent warning message */
    ifx_lcd_set_FGcolor(COLOR_WHITE, COLOR_WHITE, COLOR_WHITE); /* White text */
    ifx_lcd_set_BGcolor(COLOR_RED, COLOR_BLACK, COLOR_BLACK);     /* Red background for urgency */

    /* Main warning message */
    ifx_lcd_printf(ALIGN_CENTER, DISPLAY_H / NUM_DISPLAY_OFFSET_2 - NUM_DISPLAY_OFFSET_40, "DATABASE FULL!");

    /* Additional info */
    ifx_lcd_set_FGcolor(COLOR_YELLOW, COLOR_YELLOW, COLOR_BLACK);   /* Yellow text */
    ifx_lcd_set_BGcolor(COLOR_DARK_RED, COLOR_BLACK, COLOR_BLACK);     /* Dark red background */
    ifx_lcd_printf(ALIGN_CENTER, DISPLAY_H / NUM_DISPLAY_OFFSET_2 - NUM_DISPLAY_OFFSET_10, "MAX %d USERS ENROLLED", MAX_N_USER);
    ifx_lcd_printf(ALIGN_CENTER, DISPLAY_H / NUM_DISPLAY_OFFSET_2 + NUM_DISPLAY_OFFSET_20, "CLEAR USERS TO ADD NEW");


    ifx_lcd_set_FGcolor(COLOR_WHITE, COLOR_WHITE, COLOR_WHITE); /* White text */
    ifx_lcd_set_BGcolor(COLOR_BLACK, COLOR_BLACK, COLOR_DARK_BLUE);     /* Blue background */
    ifx_lcd_printf(ALIGN_CENTER, DISPLAY_H / NUM_DISPLAY_OFFSET_2 + NUM_DISPLAY_OFFSET_50, "TOUCH CLEAR BUTTON");
}


/********************************************************************************
* Function Name: ifx_copy_embeddings_to_nvm
*********************************************************************************
* Summary:
*  Copies a specified number of bytes from a source address to non-volatile memory
*  (NVM) at the given address, using a blocking API to perform the write operation.
*  Checks for AHB bus errors and updates the status accordingly.
*
* Parameters:
*  uint32_t nvm_addr: The address in non-volatile memory to write to
*  uint8_t *src_addr: Pointer to the source data to be copied
*  uint32_t num_bytes: Number of bytes to copy
*
* Return:
*  cy_en_rram_status_t: Status of the write operation (CY_RRAM_SUCCESS or error
*   code)
*
********************************************************************************/
static cy_en_rram_status_t ifx_copy_embeddings_to_nvm(uint32_t nvm_addr,
        uint8_t *src_addr,
        uint32_t num_bytes)
{
    cy_en_rram_status_t status = CY_RRAM_SUCCESS;

    CY_ASSERT(NULL != src_addr);

    taskENTER_CRITICAL();
    /* using blocking API as hardfault observed with non-blocking one */
    status = Cy_RRAM_NvmWriteByteArray(RRAMC0,
            nvm_addr,
            src_addr,
            num_bytes);
    if (CY_RRAM_SUCCESS == status)
    {
        cy_en_rram_hresp_t ahb_error = Cy_RRAM_GetAHBError(RRAMC0);
        if (CY_RRAM_AHB_NO_ERROR != ahb_error)
        {
            printf("[ERROR] AHB bus error while accessing NVM database 0x%08x\r\n", ahb_error);
            printf("[INFO] NVM status 0x%08lx\r\n", Cy_RRAM_GetStatus(RRAMC0));

            status = CY_RRAM_WRITE_OPERATION_ERROR;
        }
    }

    taskEXIT_CRITICAL();

    return status;
}


#ifdef PLOT_ALIGNED_FACES

/********************************************************************************
* Function Name: plot_aligned_faces_bottom
*********************************************************************************
* Summary:
*  Plot just the first aligned face at bottom of screen
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void plot_aligned_faces_bottom() {
    ifx_faceid_rslt_t result;

    result = ifx_face_id_get_aligned_faces(&aligned_faces);
    if (result != IFX_FACEID_RSLT_SUCCESS || aligned_faces.num_aligned_faces == RESET_VALUE_INDEX) {
        return;
    }

    /* Use the int8_t display function directly */
    ifx_lcd_display_Rect_i8(PLOT_ALIGNED_X, PLOT_ALIGNED_Y,
            aligned_faces.aligned_faces,  /* int8_t data */
            aligned_faces.face_width,
            aligned_faces.face_height);
}
#endif /* PLOT_ALIGNED_FACES */


/********************************************************************************
* Function Name: touch_read_timer_cb
*********************************************************************************
* Summary:
*  Timer callback function that reads touch input from a CTP FT5406 touch
*  controller, processes touch events for button interactions, and handles
*  debouncing. Updates touch button states for face enrolment, cancel, and
*  clear enrolled users buttons based on touch coordinates and device connection
*  status.
*
* Parameters:
*  TimerHandle_t xTimer: Handle to the timer that triggered the callback
*
* Return:
*  void
*
********************************************************************************/
static void touch_read_timer_cb(TimerHandle_t xTimer)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    static int touch_x = RESET_VALUE_INDEX;
    static int touch_y = RESET_VALUE_INDEX;
    volatile int x_val = RESET_VALUE_INDEX;
    volatile int y_val = RESET_VALUE_INDEX;
    mtb_ctp_touch_event_t touch_event = MTB_CTP_TOUCH_UP;

    i2c_status = mtb_ctp_ft5406_get_single_touch(&touch_event, &touch_x, &touch_y);

    if ((CY_SCB_I2C_SUCCESS == i2c_status) &&
            ((MTB_CTP_TOUCH_DOWN == touch_event) || (MTB_CTP_TOUCH_CONTACT == touch_event)))
    {
        if (!button_debouncing)
        {
            /* Set the debouncing flag */
            button_debouncing = true;

            /* Record the current timestamp */
            button_debounce_timestamp = (uint32_t) (xTaskGetTickCount());
        }

        if (button_debouncing && ((xTaskGetTickCount() - button_debounce_timestamp) >= pdMS_TO_TICKS(DEBOUNCE_TIME_MS)))
        {
            button_debouncing = false;
            x_val = ((ifx_lcd_get_Display_Width() - NUM_LOOP_INCREMENT_1) - NUM_DISPLAY_OFFSET_32) - touch_x;
            y_val = (ifx_lcd_get_Display_Height() - NUM_LOOP_INCREMENT_1) - touch_y;


            /* Start/Save Face Enrolment Button */
            if (_device_connected && (x_val >= (FACE_ENROL_BTN_X_POS - NUM_TOUCH_OFFSET_10)) && (x_val <= (FACE_ENROL_BTN_X_POS + START_FACE_ENROLMENT_BUTTON_WIDTH)) &&
                    (y_val >= (FACE_ENROL_BTN_Y_POS - NUM_TOUCH_OFFSET_10)) && (y_val <= (FACE_ENROL_BTN_Y_POS + START_FACE_ENROLMENT_BUTTON_HEIGHT)))
            {
                switch (touch_button_event[FACE_ENROL_BUTTON_ID].event)
                {
                case TOUCH_BUTTON_EVENT_RELEASED:
                    touch_button_event[FACE_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_PRESSED;
                    touch_button_event[FACE_ENROL_BUTTON_ID].pressed = true;
                    touch_button_event[FACE_ENROL_BUTTON_ID].released = false;
                    break;

                case TOUCH_BUTTON_EVENT_PRESSED:
                    touch_button_event[FACE_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_RELEASED;
                    touch_button_event[FACE_ENROL_BUTTON_ID].pressed = false;
                    touch_button_event[FACE_ENROL_BUTTON_ID].released = true;
                    break;
                }
            }
            /* Cancel Face Enrolment Button */
            else if (_device_connected && (x_val >= (CANCEL_BTN_X_POS - NUM_TOUCH_OFFSET_10)) && (x_val <= (CANCEL_BTN_X_POS + CANCEL_RELEASED_BUTTON_WIDTH)) &&
                    (y_val >= (CANCEL_BTN_Y_POS - NUM_TOUCH_OFFSET_5)) && (y_val <= (CANCEL_BTN_Y_POS + CANCEL_RELEASED_BUTTON_HEIGHT)))
            {
                touch_button_event[CANCEL_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_PRESSED;
                touch_button_event[CANCEL_ENROL_BUTTON_ID].pressed = true;
            }
            /* Clear Enrolled Users Button */
            else if (_device_connected && (x_val >= (CLEAR_BTN_X_POS - NUM_TOUCH_OFFSET_10)) && (x_val <= (CLEAR_BTN_X_POS + CLEAR_RELEASED_BUTTON_WIDTH)) &&
                    (y_val >= (CLEAR_BTN_Y_POS - NUM_TOUCH_OFFSET_5)) && (y_val <= (CLEAR_BTN_Y_POS + CLEAR_RELEASED_BUTTON_HEIGHT)))
            {
                touch_button_event[CLEAR_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_PRESSED;
                touch_button_event[CLEAR_ENROL_BUTTON_ID].pressed = true;
            }
        }
    }
}


/********************************************************************************
* Function Name: touch_input_dev_init
*********************************************************************************
* Summary:
*  Initializes the touch input device using the CTP FT5406 touch controller and
*  creates a periodic timer to read touch inputs. Starts the timer to trigger the
*  touch_read_timer_cb callback function periodically.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void touch_input_dev_init(void)
{
    cy_en_scb_i2c_status_t i2c_status = CY_SCB_I2C_SUCCESS;
    TimerHandle_t touch_timer_handle;

    i2c_status = mtb_ctp_ft5406_init(&ft5406_config);

    if (CY_SCB_I2C_SUCCESS != i2c_status)
    {
        printf("[ERROR] Touch driver initialization failed: %d\r\n", i2c_status);
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    touch_timer_handle = xTimerCreate("touch_read_timer",
            pdMS_TO_TICKS(TOUCH_TIMER_PERIOD_MS),
            pdTRUE,
            (void*)RESET_VALUE_INDEX,
            touch_read_timer_cb);

    if (touch_timer_handle != NULL)
    {
        xTimerStart(touch_timer_handle, NUM_TIMER_START_VALUE);
    }
    else
    {
        printf("[ERROR] Touch read timer creation failed\r\n");
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }
}


/********************************************************************************
* Function Name: i2c_controller_interrupt
*********************************************************************************
* Summary:
*  I2C controller ISR which invokes Cy_SCB_I2C_Interrupt to perform I2C transfer
*  as controller.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void i2c_controller_interrupt(void)
{
    Cy_SCB_I2C_Interrupt(DISPLAY_I2C_CONTROLLER_HW, &disp_i2c_controller_context);
}


/********************************************************************************
* Function Name: cleanup
*********************************************************************************
* Summary:
*  Deallocates all resources and frees up memory by closing the vector graphics
*  library.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
inline static void cleanup(void)
{
    /* Deallocate all the resource and free up all the memory */
    vg_lite_close();
}


/********************************************************************************
* Function Name: init_buffer_system
*********************************************************************************
* Summary:
*  Initializes the buffer system by resetting image buffer states, clearing the
*  last buffer index, and resetting timeout tracking variables for a clean startup.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void init_buffer_system(void) {
    printf("[LCD_TASK] Initializing buffer system\n");

    /* Ensure clean startup state */
    for (int i = NUM_LOOP_START; i < NUM_IMAGE_BUFFERS; i++) {
        _ImageBuff[i].BufReady = RESET_VALUE_INDEX;
        _ImageBuff[i].NumBytes = RESET_VALUE_INDEX;
    }
    __DMB();

    lastBuffer = RESET_VALUE_INDEX;
    __DMB();

    /* Reset timeout tracking */
    last_successful_frame_time = RESET_VALUE_INDEX;
    recovery_attempts = RESET_VALUE_INDEX;

    printf("[LCD_TASK] Buffer system initialized\n");
}


/********************************************************************************
* Function Name: draw_filled_rect_to_framebuffer
*********************************************************************************
* Summary:
*  Draws a filled rectangle directly to the framebuffer with the specified RGB565
*  color, clamping coordinates to ensure they are within the display bounds.
*
* Parameters:
*  uint16_t *framebuffer: Pointer to the framebuffer to draw to
*  int width: Width of the framebuffer
*  int height: Height of the framebuffer
*  int x1: Starting x-coordinate of the rectangle
*  int y1: Starting y-coordinate of the rectangle
*  int x2: Ending x-coordinate of the rectangle
*  int y2: Ending y-coordinate of the rectangle
*  uint16_t rgb565_color: Color of the rectangle in RGB565 format
*
* Return:
*  void
*
********************************************************************************/
static void draw_filled_rect_to_framebuffer(uint16_t *framebuffer, int width, int height,
        int x1, int y1, int x2, int y2, uint16_t rgb565_color)
{
    /* Clamp coordinates to display bounds */
    if (x1 < RESET_VALUE_INDEX) x1 = RESET_VALUE_INDEX;
    if (y1 < RESET_VALUE_INDEX) y1 = RESET_VALUE_INDEX;
    if (x2 >= width) x2 = width - NUM_LOOP_INCREMENT_1;
    if (y2 >= height) y2 = height - NUM_LOOP_INCREMENT_1;

    if (x1 >= x2 || y1 >= y2) return;  /* Invalid rectangle */

    /* Fill the rectangle directly in the framebuffer */
    for (int y = y1; y <= y2; y++) {
        uint16_t *row_ptr = &framebuffer[y * width + x1];
        for (int x = NUM_LOOP_START; x <= (x2 - x1); x++) {
            row_ptr[x] = rgb565_color;
        }
    }
}


/********************************************************************************
* Function Name: is_enrollment_complete
*********************************************************************************
* Summary:
*  Checks if all poses in the enrolment process are completed by examining the
*  enrolment progress structure.
*
* Parameters:
*  ifx_on_dev_enroll_stats_t *progress: Pointer to the enrolment progress
*  structure
*
* Return:
*  bool: True if all poses are completed, false otherwise
*
********************************************************************************/
static bool is_enrollment_complete(ifx_on_dev_enroll_stats_t *progress) {
    if (progress == NULL) {
        return false;
    }

    for (int i = NUM_LOOP_START; i < progress->num_poses; i++) {
        if (progress->enroll_progress[i] != PROGRESS_COMPLETED) {
            return false;
        }
    }

    printf("[LCD_TASK] All %d poses completed - auto-completing enrolment\n", progress->num_poses);
    return true;
}


/********************************************************************************
* Function Name: draw_enrollment_progress_grid
*********************************************************************************
* Summary:
*  Draws a grid in the top-right corner of the display to show enrolment progress
*  for each pose, with color-coded cells indicating progress status (not started,
*  in progress, completed). Includes a title, pose labels, and a legend. Updates
*  the display with the drawn grid.
*
* Parameters:
*  ifx_on_dev_enroll_stats_t *progress: Pointer to the enrolment progress status
*                                       structure
*  int current_pose: Index of the current pose being processed (-1 if none or
*                    out of range)
*  ifx_faceid_prediction_t *prediction: Pointer to the current face detection
*                                       prediction data.
*  ifx_enrollment_config_t *config: Pointer to the enrolment configuration
*                                   structure
*
* Return:
*  void
*
********************************************************************************/
static void draw_enrollment_progress_grid(ifx_on_dev_enroll_stats_t *progress, int current_pose, ifx_faceid_prediction_t *prediction, ifx_enrollment_config_t *config) {
    const int grid_size = GRID_SIZE;         /* Size of each grid cell */
    const int grid_spacing = GRID_SPACING;      /* Spacing between cells */
    const int text_offset_y = TEXT_OFFSET_Y;     /* Offset for better text centering vertically */
    const int text_offset_x = TEXT_OFFSET_X;      /* Small offset from left edge */

    /* Determine grid dimensions and layout based on num_poses */
    int grid_rows, grid_cols;
    int pose_positions[MAX_NUM_POSES][2]; /* [pose_idx][row, col] - enough space for any layout */

    if (config->num_poses == NUM_DISPLAY_OFFSET_3) {
        grid_rows = NUM_GRID_ROWS_1;
        grid_cols = NUM_GRID_COLS_3;
        /* Linear layout: Left, Front, Right */
        pose_positions[NUM_LOOP_START][NUM_LOOP_START] = RESET_VALUE_INDEX; pose_positions[NUM_LOOP_START][NUM_LOOP_INCREMENT_1] = RESET_VALUE_INDEX;  /* Left */
        pose_positions[NUM_LOOP_INCREMENT_1][NUM_LOOP_START] = RESET_VALUE_INDEX; pose_positions[NUM_LOOP_INCREMENT_1][NUM_LOOP_INCREMENT_1] = NUM_LOOP_INCREMENT_1;  /* Front */
        pose_positions[NUM_DISPLAY_OFFSET_2][NUM_LOOP_START] = RESET_VALUE_INDEX; pose_positions[NUM_DISPLAY_OFFSET_2][NUM_LOOP_INCREMENT_1] = NUM_DISPLAY_OFFSET_2;  /* Right */
    }
    else if (config->num_poses == 5) {
        grid_rows = 3;
        grid_cols = 3;
        /* Cross layout: Up, Down, Left, Right, Front (center) */
        pose_positions[0][0] = 0; pose_positions[0][1] = 1;  /* Up (top center) */
        pose_positions[1][0] = 2; pose_positions[1][1] = 1;  /* Down (bottom center) */
        pose_positions[2][0] = 1; pose_positions[2][1] = 0;  /* Left (middle left) */
        pose_positions[3][0] = 1; pose_positions[3][1] = 2;  /* Right (middle right) */
        pose_positions[4][0] = 1; pose_positions[4][1] = 1;  /* Front (center) */
    }
    else { /* config->num_poses == 9 */
        grid_rows = 3;
        grid_cols = 3;
        /* Full 3x3 grid */
        for (int i = 0; i < 9; i++) {
            pose_positions[i][0] = i / 3;  /* row */
            pose_positions[i][1] = i % 3;  /* col */
        }
    }

    /* Position in bottom right corner */
    const int grid_x = DISPLAY_W - ((grid_size + grid_spacing) * grid_cols + grid_spacing + 40);
    const int grid_y = DISPLAY_H - ((grid_size + grid_spacing) * grid_rows + grid_spacing + 100);

    /* Create RGB565 colors for each state */
    const uint16_t color_not_started = RGB565(255, 165, 0);  /* Orange */
    const uint16_t color_in_progress = RGB565(255, 0, 255);  /* Magenta */
    const uint16_t color_completed = RGB565(0, 255, 0);      /* Green */
    const uint16_t color_black = RGB565(0, 0, 0);            /* Black for border */
    const uint16_t color_red = RGB565(255, 0, 0);            /* Red for current pose highlight */
    const uint16_t color_dark_gray = RGB565(64, 64, 64);     /* Dark gray for background */

    /* Draw background for the entire grid area (dark gray) */
    draw_filled_rect_to_framebuffer(LCD_Addr, DISPLAY_W, DISPLAY_H,
            grid_x - 20,
            grid_y - 20,
            grid_x + (grid_size + grid_spacing) * grid_cols + 20,
            grid_y + (grid_size + grid_spacing) * grid_rows + 80,
            color_dark_gray);

    /* For 5-pose layout, first fill empty positions with dark gray */
    if (config->num_poses == 5) {
        for (int row = 0; row < grid_rows; row++) {
            for (int col = 0; col < grid_cols; col++) {
                /* Check if this position is used by any pose */
                bool position_used = false;
                for (int pose_idx = 0; pose_idx < config->num_poses; pose_idx++) {
                    if (pose_positions[pose_idx][0] == row && pose_positions[pose_idx][1] == col) {
                        position_used = true;
                        break;
                    }
                }

                /* Fill unused positions with darker background */
                if (!position_used) {
                    int x = grid_x + col * (grid_size + grid_spacing);
                    int y = grid_y + row * (grid_size + grid_spacing);
                    draw_filled_rect_to_framebuffer(LCD_Addr, DISPLAY_W, DISPLAY_H,
                            x, y, x + grid_size, y + grid_size, color_dark_gray);
                }
            }
        }
    }

    /* Draw grid showing progress for each pose */
    for (int pose_idx = 0; pose_idx < config->num_poses; pose_idx++) {
        int row = pose_positions[pose_idx][0];
        int col = pose_positions[pose_idx][1];

        int x = grid_x + col * (grid_size + grid_spacing);
        int y = grid_y + row * (grid_size + grid_spacing);
        uint16_t color = 0;
        uint16_t border_color = color_black;  /* Default border color */
        int border_thickness = 3;             /* Default border thickness */

        /* Set color based on progress state */
        switch (progress->enroll_progress[pose_idx]) {
        case PROGRESS_NOT_STARTED:
            color = color_not_started;
            break;
        case PROGRESS_IN_PROGRESS:
            color = color_in_progress;
            break;
        case PROGRESS_COMPLETED:
            color = color_completed;
            break;
        default:
            color = RGB565(128, 128, 128);  /* Gray fallback */
            break;
        }

        /* Check if this is the current pose - highlight with red border and make it thicker */
        if (current_pose == pose_idx) {
            border_color = color_red;
            border_thickness = 5;  /* Thicker border for current pose */
        }

        /* Draw border with appropriate color and thickness */
        draw_filled_rect_to_framebuffer(LCD_Addr, DISPLAY_W, DISPLAY_H,
                x - border_thickness, y - border_thickness,
                x + grid_size + border_thickness, y + grid_size + border_thickness,
                border_color);

        /* Draw the colored grid cell */
        draw_filled_rect_to_framebuffer(LCD_Addr, DISPLAY_W, DISPLAY_H,
                x, y,
                x + grid_size, y + grid_size,
                color);

        /* Extract RGB components from RGB565 color for background */
        uint8_t bg_r = ((color >> HEX_BIT_SHIFT_11) & HEX_RED_5BIT_MASK) << HEX_BIT_SHIFT_3;  /* Red: 5 bits to 8 bits */
        uint8_t bg_g = ((color >> HEX_BIT_SHIFT_5) & HEX_GREEN_6BIT_MASK) << HEX_BIT_SHIFT_2;   /* Green: 6 bits to 8 bits */
        uint8_t bg_b = (color & HEX_BLUE_5BIT_MASK) << HEX_BIT_SHIFT_3;          /* Blue: 5 bits to 8 bits */

        /* Draw pose label with proper positioning */
        char label[CHAR_STRING_LEN];

        /* Split long names into two lines if needed */
        const char* pose_name = progress->pose_names[pose_idx];
        if (strstr(pose_name, "-")) {
            /* Split at the hyphen for two-word names */
            char line1[CHAR_STRING_LEN], line2[CHAR_STRING_LEN];
            const char* hyphen_pos = strchr(pose_name, '-');
            int len1 = hyphen_pos - pose_name;

            strncpy(line1, pose_name, len1);
            line1[len1] = '\0';
            strcpy(line2, hyphen_pos + 1);

            /* Draw first line */
            ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
            ifx_lcd_set_BGcolor(bg_r, bg_g, bg_b);  /* Set background to match cell color */
            ifx_lcd_printf(x + text_offset_x, y + text_offset_y - 8, line1);

            /* Draw second line */
            ifx_lcd_printf(x + text_offset_x, y + text_offset_y + 8, line2);
        } else {
            /* Single line for simple names */
            sprintf(label, "%s", pose_name);
            ifx_lcd_set_FGcolor(0, 0, 0);  /* Black text */
            ifx_lcd_set_BGcolor(bg_r, bg_g, bg_b);  /* Set background to match cell color */
            ifx_lcd_printf(x + text_offset_x, y + text_offset_y, label);
        }
    }

    /* Add title above the grid */
    ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
    ifx_lcd_set_BGcolor(64, 64, 64);     /* Dark gray background */
    int title_x = grid_x + ((grid_size + grid_spacing) * grid_cols) / 2 - TITLE_X_OFFSET;
    ifx_lcd_printf(title_x, grid_y - TITLE_Y_OFFSET, "HEAD POSE PROGRESS");

    /* Vertical legend at the bottom right, each item on new line */
    int legend_x = grid_x + (grid_size + grid_spacing) * grid_cols - 100;  /* Right side of grid */
    int legend_y = grid_y + (grid_size + grid_spacing) * grid_rows + 10;
    int line_height = LINE_HEIGHT;

    /* Not Started */
    draw_filled_rect_to_framebuffer(LCD_Addr, DISPLAY_W, DISPLAY_H,
            legend_x, legend_y,
            legend_x + LEGEND_SQUARE_SIZE, legend_y + LEGEND_SQUARE_SIZE,
            color_not_started);
    ifx_lcd_set_BGcolor(64, 64, 64);  /* Back to dark gray for text */
    ifx_lcd_printf(legend_x + LEGEND_TEXT_OFFSET, legend_y + LEGEND_Y_OFFSET, "Not Started");

    /* In Progress */
    legend_y += line_height;
    draw_filled_rect_to_framebuffer(LCD_Addr, DISPLAY_W, DISPLAY_H,
            legend_x, legend_y,
            legend_x + LEGEND_SQUARE_SIZE, legend_y + LEGEND_SQUARE_SIZE,
            color_in_progress);
    ifx_lcd_printf(legend_x + LEGEND_TEXT_OFFSET, legend_y + LEGEND_Y_OFFSET, "In Progress");

    /* Completed */
    legend_y += line_height;
    draw_filled_rect_to_framebuffer(LCD_Addr, DISPLAY_W, DISPLAY_H,
            legend_x, legend_y,
            legend_x + LEGEND_SQUARE_SIZE, legend_y + LEGEND_SQUARE_SIZE,
            color_completed);
    ifx_lcd_printf(legend_x + LEGEND_TEXT_OFFSET, legend_y + LEGEND_Y_OFFSET, "Completed");

    /* Important: Update the display after all drawing is complete */
    ifx_lcd_draw_Buffer();
    __DMB();
}


/********************************************************************************
* Function Name: update_enrollment_display
*********************************************************************************
* Summary:
*  Updates the LCD display with pose-specific information during on-device face
*  enrolment, showing progress, guidance messages, and feedback based on the
*  enrolment state and face detection results. Displays a progress grid and
*  handles auto-completion or manual completion instructions.
*
* Parameters:
*  ifx_faceid_prediction_t *prediction: Pointer to the face detection prediction
*                                       data
*  ifx_enrollment_config_t *enrollment_config: Pointer to the enrolment
*                                              configuration structure
*  enrollment_state_t state: Current state of the enrolment process
*  enrollment_type_t type: Type of enrolment (e.g., on-device)
*  ifx_on_dev_enroll_stats_t *progress: Pointer to the enrolment progress status
*                                      structure
*  ifx_faceid_rslt_t enrollment_feedback: Feedback result from the enrolment
*                                         process
*
* Return:
*  void
*
********************************************************************************/
static void update_enrollment_display(ifx_faceid_prediction_t *prediction,
        ifx_enrollment_config_t *enrollment_config,
        enrollment_state_t state,
        enrollment_type_t type,
        ifx_on_dev_enroll_stats_t *progress,
        ifx_faceid_rslt_t enrollment_feedback) {
    if (type != ENROLLMENT_TYPE_ON_DEVICE) {
        return; /* Only show special display for on-device enrolment */
    }

    /* No need to get enrolment progress here since it's passed in */
    if (progress == NULL) {
        printf("[LCD_TASK][ERROR] No enrolment progress data available\n");
        return; /* Can't display without progress data */
    }

    /* Calculate current pose index for highlighting */
    int current_pose = -1;  /* Default to no pose detected */
    if (prediction->count == 1) {
        /* Use the same pose detection logic as in the enrolment system */
        /* If pose is out of range, the API will return -1 */
        current_pose = ifx_face_id_get_pose_bin_index(enrollment_config, prediction->yaw[0], prediction->pitch[0], prediction->roll[0]);
    }

    /* Draw the progress grid with current pose highlighting */
    draw_enrollment_progress_grid(progress, current_pose, prediction, enrollment_config);

    /* Show enrolment state */
    ifx_lcd_set_FGcolor(COLOR_WHITE, COLOR_WHITE, COLOR_WHITE); /* White text */
    ifx_lcd_set_BGcolor(0, 0, 128);     /* Dark blue background */

    if (state == ENROLLMENT_STATE_WAITING_FOR_REFERENCE) {
        ifx_lcd_printf(ALIGN_LEFT, 40, "WAITING FOR REFERENCE FACE");

        /* Show guidance for getting a good reference face */
        if (prediction->count == 1) {
            float conf_pct = prediction->conf[0] * 100.0f;

            if (conf_pct >= 90.0f && prediction->frontal_face[0]) {
                ifx_lcd_set_FGcolor(0, 255, 0);  /* Green text */
                ifx_lcd_printf(ALIGN_LEFT, 70, "GOOD FACE DETECTED! HOLD STEADY");
            } else {
                ifx_lcd_set_FGcolor(255, 0, 0);  /* Red text */
                ifx_lcd_printf(ALIGN_LEFT, 70, "LOOK DIRECTLY AT CAMERA (%.1f%%)", conf_pct);
            }
        } else if (prediction->count > 1) {
            ifx_lcd_set_FGcolor(255, 0, 0);  /* Red text */
            ifx_lcd_printf(ALIGN_LEFT, 70, "TOO MANY FACES (%d). NEED JUST ONE.", prediction->count);
        } else {
            ifx_lcd_set_FGcolor(255, 0, 0);  /* Red text */
            ifx_lcd_printf(ALIGN_LEFT, 70, "NO FACE DETECTED");
        }

    }
    else if (state == ENROLLMENT_STATE_COLLECTING) {
        ifx_lcd_printf(ALIGN_LEFT, 40, "COLLECTING POSES");

        /* Count completed and in-progress poses */
        int complete = 0;
        int in_progress = 0;
        for (int i = NUM_LOOP_START; i < progress->num_poses; i++) {
            if (progress->enroll_progress[i] == PROGRESS_COMPLETED) {
                complete++;
            } else if (progress->enroll_progress[i] == PROGRESS_IN_PROGRESS) {
                in_progress++;
            }
        }

        /* Show progress counts */
        ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
        ifx_lcd_printf(ALIGN_LEFT, 70, "PROGRESS: %d COMPLETE, %d IN PROGRESS", complete, in_progress);

        /* Show specific feedback based on enrolment result */
        if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_REF_LOW_CONF) {
            /* Show specific message for low pose reference confidence */
            ifx_lcd_set_FGcolor(255, 165, 0);  /* Orange text */
            ifx_lcd_printf(ALIGN_LEFT, 100, "HOLD CURRENT POSE STEADY - QUALITY TOO LOW");
        } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_OUT_OF_RANGE) {
            ifx_lcd_set_FGcolor(255, 0, 0);  /* Red text */
            ifx_lcd_printf(ALIGN_LEFT, 100, "ADJUST HEAD POSE TO SUPPORTED RANGE");
        } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_FACE_AT_BOUNDARY) {
            ifx_lcd_set_FGcolor(255, 0, 0);  /* Red text */
            ifx_lcd_printf(ALIGN_LEFT, 100, "MOVE AWAY FROM IMAGE EDGES");
        } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_MULTI_FACE) {
            /* Handle multiple faces detected */
            ifx_lcd_set_FGcolor(255, 0, 0);  /* Red text */
            ifx_lcd_printf(ALIGN_LEFT, 100, "TOO MANY FACES - ONLY ONE PERSON ALLOWED");
        } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_LOW_SIM_MAIN_REF) {
            /* Handle low similarity (different person) */
            ifx_lcd_set_FGcolor(128, 0, 128);  /* Purple text */
            ifx_lcd_printf(ALIGN_LEFT, 100, "FACE TOO DIFFERENT TO MAIN REFERENCE");
        } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_LOW_SIM_POSE_REF) {
            /* Handle low similarity (different person) */
            ifx_lcd_set_FGcolor(128, 0, 128);  /* Purple text */
            ifx_lcd_printf(ALIGN_LEFT, 100, "FACE TOO DIFFERENT TO POSE REFERENCE");
        } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_COMPLETE) {
            /* Handle pose completion */
            ifx_lcd_set_FGcolor(0, 255, 0);  /* Bright green text */
            if (current_pose >= 0 && current_pose < progress->num_poses) {
                ifx_lcd_printf(ALIGN_LEFT, 100, "EXCELLENT! %s POSE COMPLETED", progress->pose_names[current_pose]);
            } else {
                ifx_lcd_printf(ALIGN_LEFT, 100, "EXCELLENT! POSE COMPLETED");
            }
        } else {
            /* Default instruction */
            ifx_lcd_set_FGcolor(255, 255, 0);  /* Yellow text */
            ifx_lcd_printf(ALIGN_LEFT, 100, "MOVE HEAD SLOWLY IN DIFFERENT POSES");
        }

        /* AUTO-COMPLETION LOGIC */
        /* Check if we're ready for auto-completion or show manual completion option */
        if (complete == progress->num_poses) {
            /* All poses completed - show auto-completion message */
            ifx_lcd_set_FGcolor(0, 255, 0);  /* Bright green text */
            ifx_lcd_set_BGcolor(0, 128, 0);  /* Dark green background */
            ifx_lcd_printf(ALIGN_LEFT, 130, "ALL POSES COMPLETE - AUTO-SAVING...");
        } else {
            /* Still collecting poses - show manual completion option */

            ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
            ifx_lcd_set_BGcolor(0, 0, 128);      /* Dark blue background */
            ifx_lcd_printf(ALIGN_LEFT, 130, "TOUCH BUTTON TO FINISH EARLY OR WAIT FOR AUTO-COMPLETE");
        }
    }
    else if (state == ENROLLMENT_STATE_COMPLETING) {
        ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
        ifx_lcd_set_BGcolor(0, 0, 128);      /* Dark blue background */
        ifx_lcd_printf(ALIGN_LEFT, 40, "PROCESSING ENROLLMENT...");

        /* Show a completion status message */
        ifx_lcd_set_FGcolor(0, 255, 0);      /* Green text */
        ifx_lcd_set_BGcolor(0, 128, 0);      /* Dark green background */
        ifx_lcd_printf(ALIGN_LEFT, 70, "SAVING USER TO DATABASE...");
    }
}


/********************************************************************************
* Function Name: dc_irq_handler
*********************************************************************************
* Summary:
*  Interrupt handler for the display controller (DC) that clears the DC interrupt
*  using the graphics subsystem context.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void dc_irq_handler(void)
{
    Cy_GFXSS_Clear_DC_Interrupt(base, &gfx_context);
}


/********************************************************************************
* Function Name: gpu_irq_handler
*********************************************************************************
* Summary:
*  Interrupt handler for the GPU that clears the GPU interrupt using the graphics
*  subsystem context and calls the vector graphics library interrupt handler.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
static void gpu_irq_handler(void)
{
    Cy_GFXSS_Clear_GPU_Interrupt(base, &gfx_context);
    vg_lite_IRQHandler();
}


/********************************************************************************
* Function Name: update_lcd_display
*********************************************************************************
* Summary:
*  Updates the LCD display with face detection information, rendering camera
*  images,bounding boxes, and enrolment-specific visualizations. Handles
*  different display modes (inference and enrolment), shows database status,
*  and provides timing information. Supports optional features like head pose
*  axes, facial landmarks, and aligned face plotting.
*
* Parameters:
*  ifx_faceid_prediction_t *prediction: Pointer to the face detection prediction
*                                       data
*  display_mode_t current_display_mode: Current mode of the display
*                                       (e.g., inference or enrolment)
*  ifx_faceid_rslt_t enrollment_feedback: Feedback result from the enrolment
*                                         process
*  ifx_enrollment_config_t *enrollment_config: Pointer to the enrolment
*                                              configuration structure
*
* Return:
*  void
*
********************************************************************************/
void update_lcd_display(ifx_faceid_prediction_t *prediction,
        display_mode_t current_display_mode,
        ifx_faceid_rslt_t enrollment_feedback,
        ifx_enrollment_config_t *enrollment_config )
{
    int16_t * bbox;
    int     id, cid, xtext, ytext;
    int16_t xmin, ymin, xmax, ymax;
    char display_name[2*CHAR_STRING_LEN]; /* give some headroom */

    /* Clear renderTarget buffer */
    float time_draw_start = get_time_in_ms();
    vg_lite_error_t error = vg_lite_clear(renderTarget, NULL, BLACK_COLOR);
    if (error) {
        printf("\r\n[LCD_TASK][ERROR] vg_lite_clear() returned error %d\r\n", error);
        cleanup();
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Draw camera image */
    error = vg_lite_blit(renderTarget,
            &bgr565,
            &matrix,
            VG_LITE_BLEND_NONE,
            0,
            VG_LITE_FILTER_POINT);
    if (error) {
        printf("\r\n[LCD_TASK][ERROR] vg_lite_blit() (320x240 BGR565 ==> display BGR565) returned error %d\r\n", error);
        cleanup();
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }
    /* ensure that the GPU operations complete before continuing */
    vg_lite_flush();
    vg_lite_finish();    /* need to wait for completion ? */
    float time_draw_end = get_time_in_ms();
    Prep_bgr565_to_Disp = time_draw_end - time_draw_start;

    /* Draw detected faces with enrolment-aware styling */
    for (int i = 0; i < prediction->count; i++)
    {
        bbox = &prediction->bbox_int16[i << 2];
        id = prediction->id[i];
        xmin  = (int16_t)(bbox[0] * scale_Cam2Disp) + display_offset_x;
        ymin  = (int16_t)(bbox[1] * scale_Cam2Disp) + display_offset_y;
        xmax  = (int16_t)(bbox[2] * scale_Cam2Disp) + display_offset_x;
        ymax  = (int16_t)(bbox[3] * scale_Cam2Disp) + display_offset_y;

        xmin = min(max(xmin, 1), DISPLAY_W-1);
        ymin = min(max(ymin, 1), DISPLAY_H-1);
        xmax = min(max(xmax, 1), DISPLAY_W-1);
        ymax = min(max(ymax, 1), DISPLAY_H-1);

        xtext = xmin;
        ytext = (ymin - 16 >= 0) ? ymin - 16 : ymax;

        /* Different styling based on display mode */
        if (current_display_mode == DISPLAY_MODE_ENROLLMENT)
        {
            /* ENROLLMENT MODE: Simple green boxes with minimal text */
            ifx_lcd_set_FGcolor(0, 255, 0);  /* Green */

            /* Draw rectangle */
            ifx_lcd_draw_Rect(xmin, ymin, xmax, ymax);

            /* Simple text with enrolment feedback */
            if (enrollment_state == ENROLLMENT_STATE_COLLECTING)
            {
#ifdef USE_BIGGER_FONT
                /* Handle different enrolment feedback cases */
                if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_OUT_OF_RANGE)
                {
                    /* Show pose out of range message */
                    ifx_set_fg_color((255 << 16) | (255 << 8));  /* Yellow text */
                    ifx_set_bg_color(255 << 16); /* Red background*/
                    ifx_print_to_buffer(xtext, ytext - 20, "ADJUST HEAD POSE");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_FACE_AT_BOUNDARY) {
                    /* Show face at boundary message */
                    ifx_set_fg_color((255 << 16) | (255 << 8));  /* Yellow text */
                    ifx_set_bg_color(255 << 16);    /* Red background */
                    ifx_print_to_buffer(xtext, ytext - 20, "MOVE AWAY FROM EDGE");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_REF_LOW_CONF) {
                    /* Show pose reference confidence too low message */
                    ifx_set_fg_color((255 << 16) | (255 << 8));  /* Yellow text */
                    ifx_set_bg_color((255 << 16) | (165 << 8));  /* Orange background */
                    ifx_print_to_buffer(xtext, ytext - 20, "IMPROVE POSE QUALITY");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_MULTI_FACE) {
                    /* Show multiple faces detected message */
                    ifx_set_fg_color((255 << 16) | (255 << 8) | (255));   /* White text */
                    ifx_set_bg_color(255 << 16);    /* Red background */
                    ifx_print_to_buffer(xtext, ytext - 20, "TOO MANY FACES");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_COMPLETE) {
                    /* Show pose completed message */
                    ifx_set_fg_color(0);         /* Black text */
                    ifx_set_bg_color(255 << 8);  /* Bright green background */
                    ifx_print_to_buffer(xtext, ytext - 20, "POSE COMPLETE!");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_LOW_SIM_MAIN_REF || enrollment_feedback == IFX_FACEID_RSLT_ENROLL_LOW_SIM_POSE_REF) {
                    /* Show low similarity message */
                    ifx_set_fg_color((255 << 16) | (255 << 8) | (255));   /* White text */
                    ifx_set_bg_color(128 << 16 | 128); /* Purple background */
                    ifx_print_to_buffer(xtext, ytext - 20, "DIFFERENT PERSON?");
                } else {
                    /* Normal enrolment message */
                    ifx_set_fg_color((255 << 16) | (255 << 8) | (255));   /* White text */
                    ifx_set_bg_color(128 << 8);      /* Dark green background */
                    ifx_print_to_buffer(xtext, ytext - 20, "ENROLLING...");
                }
                ifx_draw_buffer( renderTarget->memory );
#else
                /* Handle different enrolment feedback cases */
                if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_OUT_OF_RANGE)
                {
                    /* Show pose out of range message */
                    ifx_lcd_set_FGcolor(255, 255, 0);  /* Yellow text */
                    ifx_lcd_set_BGcolor(255, 0, 0);    /* Red background */
                    ifx_lcd_printfToBuffer(xtext, ytext, "ADJUST HEAD POSE");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_FACE_AT_BOUNDARY) {
                    /* Show face at boundary message  */
                    ifx_lcd_set_FGcolor(255, 255, 0);  /* Yellow text */
                    ifx_lcd_set_BGcolor(255, 0, 0);    /* Red background */
                    ifx_lcd_printfToBuffer(xtext, ytext, "MOVE AWAY FROM EDGE");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_REF_LOW_CONF) {
                    /* Show pose reference confidence too low message */
                    ifx_lcd_set_FGcolor(255, 255, 0);  /* Yellow text */
                    ifx_lcd_set_BGcolor(255, 165, 0);  /* Orange background */
                    ifx_lcd_printfToBuffer(xtext, ytext, "IMPROVE POSE QUALITY");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_MULTI_FACE) {
                    /* Show multiple faces detected message */
                    ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
                    ifx_lcd_set_BGcolor(255, 0, 0);      /* Red background */
                    ifx_lcd_printfToBuffer(xtext, ytext, "TOO MANY FACES");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_POSE_COMPLETE) {
                    /* Show pose completed message */
                    ifx_lcd_set_FGcolor(0, 0, 0);        /* Black text */
                    ifx_lcd_set_BGcolor(0, 255, 0);      /* Bright green background */
                    ifx_lcd_printfToBuffer(xtext, ytext, "POSE COMPLETE!");
                } else if (enrollment_feedback == IFX_FACEID_RSLT_ENROLL_LOW_SIM_MAIN_REF || enrollment_feedback == IFX_FACEID_RSLT_ENROLL_LOW_SIM_POSE_REF) {
                    /* Show low similarity message */
                    ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
                    ifx_lcd_set_BGcolor(128, 0, 128);    /* Purple background */
                    ifx_lcd_printfToBuffer(xtext, ytext, "DIFFERENT PERSON?");
                } else {
                    /* Normal enrolment message */
                    ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
                    ifx_lcd_set_BGcolor(0, 128, 0);      /* Dark green background */
                    ifx_lcd_printfToBuffer(xtext, ytext, "ENROLLING...");
                }
#endif /* USE_BIGGER_FONT */
            }
            else
            {
#ifdef USE_BIGGER_FONT
                /* Default enrolment message for other states */
                ifx_set_fg_color((255 << 16) | (255 << 8) | (255));   /* White text */
                ifx_set_bg_color(128 << 8);      /* Dark green background */
                ifx_print_to_buffer(xtext, ytext - 20, "ENROLLING...");
                ifx_draw_buffer( renderTarget->memory );
#else
                /* Default enrolment message for other states */
                ifx_lcd_set_FGcolor(255, 255, 255);  /* White text */
                ifx_lcd_set_BGcolor(0, 128, 0);      /* Dark green background */
                ifx_lcd_printfToBuffer(xtext, ytext, "ENROLLING...");
#endif /* USE_BIGGER_FONT */
            }
        }
        else
        {
            /* INFERENCE MODE: Original colorful styling with user identification */
            cid = (id >= 0) ? (id % 7) + 1 : 0;
            ifx_lcd_set_FGcolor(color_r[cid], color_g[cid], color_b[cid]);
            /* draw rectangle */
            ifx_lcd_draw_Rect(xmin, ymin, xmax, ymax);

#ifdef DRAW_HEAD_POSE_AXES
            /* optional draw of head pose axes (same color as bbox) */
            int16_t tdx = (int16_t)(xmin + ((xmax - xmin) / 2));
            int16_t tdy = (int16_t)(ymin + ((ymax - ymin) / 2));
            ifx_lcd_draw_HeadPoseAxes(tdx, tdy, prediction->yaw[i], prediction->pitch[i], prediction->roll[i]);
#endif /* DRAW_HEAD_POSE_AXES */
#ifdef DRAW_FACIAL_LANDMARKS
            int16_t plot_landmarks[10];
            for (int l=0; l<10; l++){ /* xxxxxyyyyy */
                if (l < 5){
                    /* x */
                    plot_landmarks[l] = (int16_t)(prediction->landmarks[l] * scale_Cam2Disp) + display_offset_x;
                } else {
                    /* y */
                    plot_landmarks[l] = (int16_t)(prediction->landmarks[l] * scale_Cam2Disp) + display_offset_y;
                }
            }
            ifx_lcd_draw_FacialLandmarks(plot_landmarks);
#endif /*DRAW_FACIAL_LANDMARKS */

#ifdef USE_BIGGER_FONT
            ifx_set_fg_color(((255 - color_r[cid]) << 16) | ((255 - color_g[cid]) << 8) | (255 - color_b[cid]));
#else
            ifx_lcd_set_FGcolor(255 - color_r[cid], 255 - color_g[cid], 255 - color_b[cid]);
#endif /* USE_BIGGER_FONT */

            if (id >= 0)
            {
                if (prediction->is_tracked[i])
                {
                    snprintf(display_name, sizeof(display_name), "*%s*", prediction->class_string[i]); /* indicates it is being tracked */
                }
                else
                {
                    snprintf(display_name, sizeof(display_name), "%s", prediction->class_string[i]);
                }

#ifdef USE_BIGGER_FONT
                ifx_set_bg_color( (color_r[cid] << 16) | (color_g[cid] << 8) | color_b[cid] );
#else
                ifx_lcd_set_BGcolor(color_r[cid], color_g[cid], color_b[cid]);
#endif /* USE_BIGGER_FONT */

                /* only display the class name - without confidence/similarity */
#ifdef USE_BIGGER_FONT
#ifdef PRINT_SCORES
                ifx_print_to_buffer(xtext, ytext - 20, "%s, %.2f %d", display_name, prediction->similarity[i], id);
#else
                ifx_print_to_buffer(xtext, ytext - 20, "%s", display_name);
#endif
                ifx_draw_buffer( renderTarget->memory );
#else
#ifdef PRINT_SCORES
                ifx_lcd_printfToBuffer(xtext, ytext, "%s, %.2f %d", display_name, prediction->similarity[i], id);
#else
                ifx_lcd_printfToBuffer(xtext, ytext, "%s", display_name);
#endif
#endif /* USE_BIGGER_FONT */
            }
            else
            {
#ifdef USE_BIGGER_FONT
                ifx_set_fg_color((255 << 16) | (255 << 8) | 255 ); /* white text */
                ifx_set_bg_color(255 << 16); /* red back-ground */
#ifdef PRINT_SCORES
                ifx_print_to_buffer(xtext, ytext - 20, "Unknown, %.2f", prediction->similarity[i]);
#else
                ifx_print_to_buffer(xtext, ytext - 20, "Unknown");
#endif
                ifx_draw_buffer( renderTarget->memory );
#else
                ifx_lcd_set_FGcolor(COLOR_WHITE, COLOR_WHITE, COLOR_WHITE); /* white text */
                ifx_lcd_set_BGcolor(HEX_RED_BACKGROUND, 0, 0);    /* red back-ground */
#ifdef PRINT_SCORES
                ifx_lcd_printfToBuffer(xtext, ytext, "Unknown, %.2f", prediction->similarity[i]);
#else
                ifx_lcd_printfToBuffer(xtext, ytext, "Unknown");
#endif
#endif /* USE_BIGGER_FONT */
            }
        }

        ifx_lcd_draw_Buffer();
    }

    /* Display database status in top-left area (always visible) */
    display_database_status();

    /* Display mode indicator - moved down to avoid overlap with database status */
    ifx_lcd_set_FGcolor(255, 255, 0); /* Yellow text */
    ifx_lcd_set_BGcolor(0, 0, 255);   /* Blue background */
    if (current_display_mode == DISPLAY_MODE_INFERENCE) {
        ifx_lcd_printf(ALIGN_LEFT, 10, "INFERENCE MODE");
    } else {
#ifndef DISABLE_ONDEV_ENROLL
        ifx_lcd_printf(ALIGN_LEFT, 10, "ENROLMENT MODE");

        /* If in enrolment mode, show additional enrolment information */
        if (enrollment_type == ENROLLMENT_TYPE_ON_DEVICE) {
            /* Get current enrolment progress for display */
            ifx_on_dev_enroll_stats_t enrollment_progress;
            /* we just want status, early return so we can pass in some NULLs */
            ifx_faceid_rslt_t prog_result = ifx_on_device_enrollment(NULL, NULL, ENROLL_MODE_STATUS, &enrollment_progress, enrollment_config);

            if (prog_result == IFX_FACEID_RSLT_SUCCESS) {
                update_enrollment_display(prediction, enrollment_config, enrollment_state, enrollment_type, &enrollment_progress, enrollment_feedback);
            }
        }
#endif /* DISABLE_ONDEV_ENROLL */
    }

    /* Display some timing information in bottom-right */
    ifx_lcd_set_FGcolor(0, 255, 0);     /* green text */
    ifx_lcd_set_BGcolor(0, 0, 0);       /* black background */

#ifdef SHOW_INFERENCING_TIME
    ifx_lcd_printf(ALIGN_RIGHT, FPS_TXT_Y_POS, "Model %5.1f ms", model_inferencing_time);
#else
    ifx_lcd_printf(ALIGN_RIGHT, FPS_TXT_Y_POS, "%5.1f Model FPS", fps);
#endif /* SHOW_INFERENCING_TIME */

#ifdef PLOT_ALIGNED_FACES
    if (prediction->count > 0) {
        plot_aligned_faces_bottom();
    }
#endif /*PLOT_ALIGNED_FACES */

    /* Show database full overlay if needed (temporary 5-second warning) */
    if (should_show_database_full_overlay()) {
        display_database_full_overlay();
    }

    __DMB();
}


/********************************************************************************
* Function Name: getInputImage
*********************************************************************************
* Summary:
*  Retrieves and processes an input image from a camera buffer, selecting a valid
*  buffer with fallback logic, performing color format conversions (YUV422 to
*  RGB565, and RGB565 to RGB888), and optionally mirroring the image. Handles
*  timeout and error conditions, clears buffer status after processing, and
*  returns the processed image data for AI inference. Supports test image
*  conversion if enabled.
*
* Parameters:
*  void
*
* Return:
*  uint8_t *: Pointer to the processed RGB888 image buffer, or NULL on error or
*             timeout
*
********************************************************************************/
uint8_t * getInputImage()
{
    vg_lite_error_t error;
    float time_draw_start = get_time_in_ms();
    float current_time = time_draw_start;

    /* **KEY CHANGE**: More robust buffer selection with fallback */
    __DMB();
    uint8_t workBuffer = lastBuffer;
    uint8_t bufReady = 0;

    /* **KEY CHANGE**: Validate buffer index first */
    if (workBuffer >= NUM_IMAGE_BUFFERS) {
        printf("[LCD_TASK] Invalid lastBuffer %d, using buffer 0\n", workBuffer);
        workBuffer = 0;
    }

    bufReady = _ImageBuff[workBuffer].BufReady;
    __DMB();

    /* **KEY CHANGE**: If primary buffer not ready, try the other buffer */
    if (bufReady == 0) {
        uint8_t altBuffer = (workBuffer + 1) % NUM_IMAGE_BUFFERS;
        __DMB();
        if (_ImageBuff[altBuffer].BufReady == 1) {
            workBuffer = altBuffer;
            bufReady = 1;
            printf("[LCD_TASK] Primary buffer %d not ready, using buffer %d\n", lastBuffer, workBuffer);
        }
        __DMB();
    }

    /* Simple timeout without aggressive recovery */
    if (bufReady == 0) {
        if (last_successful_frame_time == 0) {
            last_successful_frame_time = current_time;
        }

        float time_since_last_frame = current_time - last_successful_frame_time;

        if (time_since_last_frame > watchdog_timeout) {
            if (recovery_attempts == 0) {
                printf("[LCD_TASK][TIMEOUT] No frames for %.1f seconds\n", time_since_last_frame / 1000.0f);
                recovery_attempts = 1;
            }
        }
        return NULL;
    }

    /* Success path - frame available */
    if (recovery_attempts > 0) {
        printf("[LCD_TASK][RECOVERY] Frame received after timeout\n");
        recovery_attempts = 0;
    }
    last_successful_frame_time = current_time;

    /* Validate buffer data before processing */
    __DMB();
    if (_ImageBuff[workBuffer].NumBytes != CAMERA_BUFFER_SIZE) {
        /* Clear the bad buffer */
        _ImageBuff[workBuffer].NumBytes = 0;
        __DMB();
        _ImageBuff[workBuffer].BufReady = 0;
        __DMB();
        return NULL;
    }

    /* Process the frame */
    float time_draw_1 = get_time_in_ms();
    error = vg_lite_blit(&bgr565, &usb_yuv_frames[workBuffer], NULL,
            VG_LITE_BLEND_NONE, 0, VG_LITE_FILTER_POINT);
    if (error) {
        printf("\r\n[LCD_TASK][ERROR] vg_lite_blit() error %d\r\n", error);

        /* **KEY CHANGE**: Clear buffer even on error to prevent stuck state */
        _ImageBuff[workBuffer].NumBytes = 0;
        __DMB();
        _ImageBuff[workBuffer].BufReady = 0;
        __DMB();

        cleanup();
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }
    vg_lite_flush();
    vg_lite_finish();

    /* **KEY CHANGE**: Clear buffer status AFTER successful processing */
    /* Use more robust atomic clearing */
    _ImageBuff[workBuffer].NumBytes = 0;
    __DMB();
    _ImageBuff[workBuffer].BufReady = 0;
    __DMB();


    if (!point3mp_camera_enabled){
        mirrorImage(&bgr565);
    }

    /* display 320x240 BGR565 image on LCD (scaled display BGR565) */
    float time_draw_3 = get_time_in_ms();
    ifx_image_conv_RGB565_to_RGB888(bgr565.memory, CAMERA_WIDTH, CAMERA_HEIGHT,
            Image_buf_bgr888, IMAGE_WIDTH, IMAGE_HEIGHT);

    float time_draw_end = get_time_in_ms();

    /* Performance measurements */
    Time_getInputImage      = time_draw_end - time_draw_start;
    Prep_Wait_Buf           = time_draw_1   - time_draw_start;
    Prep_YUV422_to_bgr565   = time_draw_3   - time_draw_1;
    Prep_RGB565_to_RGB888   = time_draw_end - time_draw_3;

#ifdef TEST_SAVED_IMAGE

    /* Convert test image (RGB888) to BGR565 for display */
    ifx_image_conv_RGB888_to_RGB565(one_detection, ONE_DETECTION_WIDTH, ONE_DETECTION_HEIGHT,
            bgr565.memory, CAMERA_WIDTH, CAMERA_HEIGHT);

    /* Copy test image to RGB888 for AI inference */
    /* Make sure sizes match: both should be IMAGE_WIDTH * IMAGE_HEIGHT * 3 */
    size_t copy_size = min(ONE_DETECTION_SIZE, sizeof(Image_buf_bgr888));
    memcpy(Image_buf_bgr888, one_detection, copy_size);

#endif /* TEST_SAVED_IMAGE */

    return Image_buf_bgr888;
}


/********************************************************************************
* Function Name: VG_LITE_ERROR_EXIT
*********************************************************************************
* Summary:
*  Handles vector graphics library errors by printing an error message with the
*  specified status, performing cleanup, and triggering an assertion to halt
*  execution.
*
* Parameters:
*  char *msg: Error message to be printed
*  vg_lite_error_t vglite_status: Vector graphics library error status code

* Return:
*  void
*
********************************************************************************/
void VG_LITE_ERROR_EXIT(char *msg, vg_lite_error_t vglite_status)
{
    printf("[LCD_TASK][ERROR] %s %d\r\n", msg, vglite_status);
    cleanup();
    CY_ASSERT(0);
}


/********************************************************************************
* Function Name: VG_switch_frame
*********************************************************************************
* Summary:
*  Switches the display frame buffer by setting the new frame buffer address in
*  the graphics subsystem, transferring it to the display controller, and swapping
*  between double-buffered render targets.
*
* Parameters:
*  void
*
* Return:
*  void
*
********************************************************************************/
void VG_switch_frame()
{
    static int    _current_buffer = 0;

    vg_lite_flush();
    vg_lite_finish();

    /* Sets Video/Graphics layer buffer address and transfer the frame buffer to DC */
    Cy_GFXSS_Set_FrameBuffer(base, (uint32_t*) renderTarget->address, &gfx_context);
    __DMB();

    /* Swap buffers */
    _current_buffer ^= 1;
    renderTarget = &display_buffer[_current_buffer];

    __DMB();
}


/********************************************************************************
* Function Name: cm55_ns_gfx_task
*********************************************************************************
* Summary:
*  Main graphics task for the CM55 non-secure domain, responsible for initializing
*  the graphics subsystem, display, touch input, and face ID library. Continuously
*  processes camera images, performs face detection and enrolment, updates the
*  LCD display based on the current mode (inference or enrolment), and handles
*  touch button events for enrolment control. Manages buffer states, database
*  operations, and error handling.
*
* Parameters:
*  void *arg: Task argument
*
* Return:
*  void
*
********************************************************************************/
void cm55_ns_gfx_task(void *arg)
{
    CY_UNUSED_PARAMETER(arg);
    ifx_faceid_rslt_t   ifx_result = IFX_FACEID_RSLT_SUCCESS;
    ifx_faceid_rslt_t db_update_result;

    cy_en_gfx_status_t  status = CY_GFX_SUCCESS;
    cy_en_sysint_status_t sysint_status = CY_SYSINT_SUCCESS;
    vg_lite_error_t     vglite_status = VG_LITE_SUCCESS;

    uint8_t *image_buf = NULL;

    /* initialize the detection options and output structs */
    ifx_faceid_prediction_t prediction;
    ifx_faceid_opt_t opt = {
            .image_width    = IMAGE_WIDTH,  /* input buffer width (320) */
            .image_height   = IMAGE_HEIGHT, /* input buffer height (240) */
            .scaled_width   = 0,            /* will get set on init of model */
            .scaled_height  = 0             /* will get set on init of model */
    };

    /* Display mode tracking */
    display_mode_t current_display_mode = DISPLAY_MODE_INFERENCE;

    float time_model_start = 0.0f;
    float time_model_end = 0.0f;

    /* Set frame buffer address to the GFXSS configuration structure */
    GFXSS_config.dc_cfg->gfx_layer_config->buffer_address    = (gctADDRESS *) vglite_heap_base;
    GFXSS_config.dc_cfg->gfx_layer_config->uv_buffer_address = (gctADDRESS *) vglite_heap_base;

    /* Initializes the graphics subsystem according to the configuration */
    status = Cy_GFXSS_Init(base, &GFXSS_config, &gfx_context);
    if (CY_GFX_SUCCESS != status) {
        printf("[LCD_TASK][ERROR] Graphics subsystem initialization failed: Cy_GFXSS_Init() returned error %d\r\n", status);
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Initialize GFXXs DC interrupt */
    sysint_status = Cy_SysInt_Init(&dc_irq_cfg, dc_irq_handler);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        printf("[LCD_TASK][ERROR] Error in registering DC interrupt: %d\r\n", sysint_status);
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }
    NVIC_EnableIRQ(GFXSS_DC_IRQ);        /* Enable interrupt in NVIC. */
    Cy_GFXSS_Clear_DC_Interrupt(base, &gfx_context);

    /* Initialize GFX GPU interrupt */
    sysint_status = Cy_SysInt_Init(&gpu_irq_cfg, gpu_irq_handler);
    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        printf("[LCD_TASK][ERROR] Error in registering GPU interrupt: %d\r\n", sysint_status);
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Enable GPU interrupt */
    Cy_GFXSS_Enable_GPU_Interrupt(base);
    NVIC_EnableIRQ(GFXSS_GPU_IRQ);        /* Enable GPU interrupt in NVIC. */

    cy_en_scb_i2c_status_t i2c_result = CY_SCB_I2C_SUCCESS;

    /* Initialize the I2C in controller mode. */
    i2c_result = Cy_SCB_I2C_Init(DISPLAY_I2C_CONTROLLER_HW,
            &DISPLAY_I2C_CONTROLLER_config, &disp_i2c_controller_context);

    if (CY_SCB_I2C_SUCCESS != i2c_result)
    {
        printf("I2C controller initialization failed !!\r\n");
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Initialize the I2C interrupt */
    sysint_status = Cy_SysInt_Init(&i2c_controller_irq_cfg,
            &i2c_controller_interrupt);

    if (CY_SYSINT_SUCCESS != sysint_status)
    {
        printf("I2C controller interrupt initialization failed\r\n");
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Enable the I2C interrupts. */
    NVIC_EnableIRQ(i2c_controller_irq_cfg.intrSrc);

    /* Enable the I2C */
    Cy_SCB_I2C_Enable(DISPLAY_I2C_CONTROLLER_HW);

    /* Allow I2C to be stabalized to initialize the display */
    Cy_SysLib_Delay(50);

    i2c_result = mtb_disp_waveshare_4p3_init(DISPLAY_I2C_CONTROLLER_HW,
            &disp_i2c_controller_context);

    if (CY_SCB_I2C_SUCCESS != i2c_result)
    {
        printf("Waveshare 4.3-Inch display init failed with status = %u\r\n", (unsigned int) i2c_result);
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Set the display size for LCD utils */
    ifx_lcd_set_Display_size(DISPLAY_W, DISPLAY_H);
    /***************************************************************/
    /* Init VGLite */
    vg_module_parameters_t vg_params;
    vg_params.register_mem_base = (uint32_t) GFXSS_GFXSS_GPU_GCNANO;
    vg_params.gpu_mem_base[VG_PARAMS_POS] = GPU_MEM_BASE;
    vg_params.contiguous_mem_base[VG_PARAMS_POS] = vglite_heap_base;
    vg_params.contiguous_mem_size[VG_PARAMS_POS] = VGLITE_HEAP_SIZE;

    vg_lite_init_mem(&vg_params);

    /* Initialize the draw */

    vglite_status = vg_lite_init(256, 256);
    if (VG_LITE_SUCCESS != vglite_status) {
        VG_LITE_ERROR_EXIT("vg_lite engine init failed: vg_lite_init() returned error %d\r\n", vglite_status);
    }

    /* setup double display-frame-buffers */
    for (int ii = 0; ii < 2; ii++) {
        display_buffer[ii].width  = DISPLAY_W;
        display_buffer[ii].height = DISPLAY_H;
        display_buffer[ii].format = VG_LITE_BGR565;
        vglite_status = vg_lite_allocate(&display_buffer[ii]);
        if (VG_LITE_SUCCESS != vglite_status) {
            VG_LITE_ERROR_EXIT("display_buffer[] allocation failed in vglite space: vg_lite_allocate() returned error %d\r\n", vglite_status);
        }
    }
    renderTarget = &display_buffer[0];

    /* Allocate the camera buffers */
    for (int i = 0; i < NUM_IMAGE_BUFFERS; i++) {
        usb_yuv_frames[i].width  = CAMERA_WIDTH;
        usb_yuv_frames[i].height = CAMERA_HEIGHT;
        usb_yuv_frames[i].format = VG_LITE_YUYV;
        usb_yuv_frames[i].image_mode = VG_LITE_NORMAL_IMAGE_MODE;
        vglite_status = vg_lite_allocate(&usb_yuv_frames[i]);
        if (VG_LITE_SUCCESS != vglite_status) {
            VG_LITE_ERROR_EXIT("camera buffers allocation failed in vglite space: vg_lite_allocate() returned error %d\r\n", vglite_status);
        }
    }

    /* Allocate the work camera buffer */
    bgr565.width  = CAMERA_WIDTH;
    bgr565.height = CAMERA_HEIGHT;
    bgr565.format = VG_LITE_BGR565;
    bgr565.image_mode = VG_LITE_NORMAL_IMAGE_MODE;
    bgr565.stride = CAMERA_WIDTH * 2;
    bgr565.handle = 0;
    bgr565.memory = (void *)Image_buf_bgr565;
    bgr565.address = (uint32_t)Image_buf_bgr565;

    /* Clear the buffer with Black */
    vglite_status = vg_lite_clear(renderTarget, NULL, BLACK_COLOR);
    if (VG_LITE_SUCCESS != vglite_status) {
        VG_LITE_ERROR_EXIT("Clear failed: vg_lite_clear() returned error %d\r\n", vglite_status);
    }

    /***************************************************************/

    /* Define the transformation matrix that will be applied from the camera image to the display */
    vg_lite_identity(&matrix);

    /* scale factor from the camera image to the display */
    float scale_Cam2Disp_x = (float)DISPLAY_W / (float)CAMERA_WIDTH;
    float scale_Cam2Disp_y = (float)DISPLAY_H / (float)CAMERA_HEIGHT;
    /* this decides if you want to display the whole image and add boundaries (min),
     or display with no boundaries but crop off some of the image (camera vs display aspect ratio mismatch handling) */

    scale_Cam2Disp = max(scale_Cam2Disp_x, scale_Cam2Disp_y);  /* 4.3" uses max to fill screen */

    vg_lite_scale(scale_Cam2Disp, scale_Cam2Disp, &matrix);

    /* Move the scaled-frame to the display center */
    float translate_x = (DISPLAY_W / scale_Cam2Disp - CAMERA_WIDTH) * 0.5f;
    float translate_y = (DISPLAY_H / scale_Cam2Disp - CAMERA_HEIGHT) * 0.5f;

    /* Move the scaled image buffer frame to the display center */
    display_offset_x = (DISPLAY_W - scale_Cam2Disp * IMAGE_WIDTH) / 2;
    display_offset_y = (DISPLAY_H - scale_Cam2Disp * CAMERA_HEIGHT) / 2;

    vg_lite_translate(translate_x, translate_y, &matrix);

    touch_input_dev_init();

    for (uint8_t i = 0; i < BUTTON_ID_MAX; i++)
    {
        touch_button_event[i].event = TOUCH_BUTTON_EVENT_RELEASED;
        touch_button_event[i].pressed = false;
        touch_button_event[i].released = false;
    }

    /* Initialize RRAM for NVM */
    cy_en_rram_status_t rram_status = Cy_RRAM_TSReadByteArray(RRAMC0,
            (uint32_t) &enrolled_user_faceid_embeddings,
            (uint8_t *) &faceid_embeddings,
            sizeof(enrolled_user_faceid_embeddings));

    if (CY_RRAM_SUCCESS != rram_status)
    {
        printf("[ERROR] Failed to read enrolled user data from NVM database\r\n");
    }

    /* Configure enrolment parameters (set to defaults for now, but change for threshold tuning) */
    printf("[LCD_TASK] Configuring enrolment parameters...\n");
    ifx_enrollment_config_t enrollment_config = {0};
    /* Get current defaults first */
    ifx_result = ifx_face_id_get_enrollment_config(&enrollment_config);
    if (ifx_result != IFX_FACEID_RSLT_SUCCESS) {
        printf("[LCD_TASK][ERROR] Failed to get enrolment config: %d\n", ifx_result);
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }
    /* For now, just set num_poses based on your cm55's Makefile NUM_POSES arg */
    enrollment_config.num_poses = NUM_POSES;

    /* Configure inference parameters (set to defaults for now, but change for threshold tuning) */
    printf("[LCD_TASK] Configuring inference parameters...\n");
    ifx_inference_config_t inference_config = {0};
    /* Get current defaults first */
    ifx_result = ifx_face_id_get_inference_config(&inference_config);
    if (ifx_result == IFX_FACEID_RSLT_SUCCESS) {
        if(Logitech_camera_enabled == 0){
            /* if we are using the WPP webcam (0.3 MP instead of the C920e 5 MP), we need to handle more noise and blur etc */
            /* lets tune some thresholds here to make our life a bit easier when using it */
            /* the main issue I notice is that it misses more faces at detect, so lets start by just tuning those thresholds */
            inference_config.thresh_kp_conf = 0.7;/* was previously 0.8 by default. Increase if you notice non-face detected boxes */
            inference_config.thresh_det_conf = 0.275;   /* was previously 0.375 by default. Increase if you notice non-face detected boxes */
            inference_config.default_face_id_sim_th = 0.5; /* was previously 0.5, need it to be stricter */
        }
    } else {
        printf("[LCD_TASK][ERROR] Failed to get inference config: %d\n", ifx_result);
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Initialize faceID library */
    ifx_result = ifx_face_id_init(&opt, &prediction, &faceid_embeddings, &inference_config, &enrollment_config);
    if (IFX_FACEID_RSLT_SUCCESS != ifx_result)
    {
        printf("\r\n[LCD_TASK][ERROR] ifx_face_id_init: fail\r\n\n");
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* check the display values in the design.modus generated config source code match the desired build values */
    uint16_t desired_display_h, desired_display_w;
    desired_display_h = DISPLAY_H;
    desired_display_w = DISPLAY_W;

    bool display_error = true;
    if (GFXSS_dc_config.display_width != desired_display_w) {
        printf("\n[LCD_ERROR] GFXSS_dc_config.display_width = %ld\n", GFXSS_dc_config.display_width);
    } else if (GFXSS_dc_config.display_height != desired_display_h) {
        printf("\n[LCD_ERROR] GFXSS_dc_config.display_height = %ld\n", GFXSS_dc_config.display_height);
    } else if (GFXSS_dc_config.gfx_layer_config->width != desired_display_w) {
        printf("\n[LCD_ERROR] GFXSS_dc_config.gfx_layer_config.width = %d\n", GFXSS_dc_config.gfx_layer_config->width);
    } else if (GFXSS_dc_config.gfx_layer_config->height != desired_display_h) {
        printf("\n[LCD_ERROR] GFXSS_dc_config.gfx_layer_config.height = %d\n", GFXSS_dc_config.gfx_layer_config->height);
    } else if (GFXSS_mipidsi_display_params.hdisplay != desired_display_w) {
        printf("\n[LCD_ERROR] GFXSS_mipidsi_display_params.hdisplay = %d\n", GFXSS_mipidsi_display_params.hdisplay);
    } else if (GFXSS_mipidsi_display_params.vdisplay != desired_display_h) {
        printf("\n[LCD_ERROR] GFXSS_mipidsi_display_params.vdisplay = %d\n", GFXSS_mipidsi_display_params.vdisplay);
    } else {
        display_error = false; /* all is good if it reaches else */
    }
    if (display_error == true) {
        printf("\nPlease adjust your design.modus as appropraite for your selected build option display.\r\n");
        CY_ASSERT(NUM_CY_ASSERT_VALUE);
    }

    /* Initialize buffer states - clear any startup artifacts */
    printf("[LCD_TASK] Clearing all buffers at startup\n");
    for (int i = 0; i < NUM_IMAGE_BUFFERS; i++) {
        _ImageBuff[i].BufReady = 0;
        _ImageBuff[i].NumBytes = 0;
    }
    __DMB();

    init_buffer_system();

    /* Initialize database user count */
    printf("[LCD_TASK] Initializing database user count...\n");
    ifx_faceid_rslt_t db_result = update_database_user_count();
    if (db_result != IFX_FACEID_RSLT_SUCCESS) {
        printf("[LCD_TASK][WARNING] Failed to initialize database user count: %d\n", db_result);
    } else {
        printf("[LCD_TASK] Database initialized with %d users\n", current_user_count);
    }

    ifx_faceid_rslt_t current_enrollment_feedback = IFX_FACEID_RSLT_SUCCESS;

    /* Delay for USB enumeration to complete before rendering
     * data to the display
     */
    vTaskDelay(pdMS_TO_TICKS(1200));

    printf("[LCD_TASK] Starting while loop for model inference ...\n");

    while (1)
    {
        if (_device_connected)
        {
            /* get the latest input image to the input image buffer (image_buf_int8) */
            image_buf = getInputImage();
            if (image_buf == NULL) {

                vTaskDelay(pdMS_TO_TICKS(10)); /* Needed to avoid race condition */

                continue;
            }

            /* Update database user count every frame (lightweight operation) */
            db_update_result = update_database_user_count();
            if (db_update_result != IFX_FACEID_RSLT_SUCCESS) {
                printf("[LCD_TASK][WARNING] Failed to update database user count: %d\n", db_update_result);
            }

            /* we want to enable augmentation when collecting data (after reference acquired) */
            bool augment = enrollment_state == ENROLLMENT_STATE_COLLECTING;
            time_model_start = ifx_time_get_ms_f(); /* start timer for when Face ID model system starts to execute */

            /* Inference */
            ifx_result = ifx_face_id_inference(
                    image_buf,
                    &opt,
                    &prediction,
                    &faceid_embeddings,
                    augment,
                    &inference_config);
            if (ifx_result != IFX_FACEID_RSLT_SUCCESS)
            {
                printf("[LCD_TASK][ERROR] Face inference failed with error: %u\n", ifx_result);
            }
            else if (enrollment_type == ENROLLMENT_TYPE_ON_DEVICE)
            {
                /* Process on-device enrolment states */
                ifx_on_dev_enroll_stats_t progress;
                ifx_faceid_rslt_t enroll_result = IFX_FACEID_RSLT_SUCCESS;

                if (enrollment_state == ENROLLMENT_STATE_WAITING_FOR_REFERENCE) {
                    /* Try to capture reference image using init mode */
                    enroll_result = ifx_on_device_enrollment(&prediction, &opt, ENROLL_MODE_INIT, &progress, &enrollment_config);
                    current_enrollment_feedback = enroll_result;  /* Store feedback */
                    if (enroll_result == IFX_FACEID_RSLT_SUCCESS) {
                        /* Reference captured successfully */
                        printf("[LCD_TASK] Reference face acquired, now collecting poses\n");
                        enrollment_state = ENROLLMENT_STATE_COLLECTING;
                    } else if (enroll_result != IFX_FACEID_RSLT_ENROLL_WAITING) {
                        /* Error occurred */
                        printf("[LCD_TASK][ERROR] Failed to initialize enrolment\n");
                        enrollment_state = ENROLLMENT_STATE_NONE;
                        enrollment_type = ENROLLMENT_TYPE_NONE;
                        current_display_mode = DISPLAY_MODE_INFERENCE;
                    }
                    /* If enroll_result == IFX_FACEID_RSLT_ENROLL_WAITING, keep waiting for reference */
                }
                else if (enrollment_state == ENROLLMENT_STATE_COLLECTING) {
                    /* Automatically update enrolment with the current face data */
                    enroll_result = ifx_on_device_enrollment(&prediction, &opt, ENROLL_MODE_UPDATE, &progress, &enrollment_config);
                    current_enrollment_feedback = enroll_result;  /* Store feedback */
                    /* Check if all poses are completed and auto-complete enrolment */
                    if (is_enrollment_complete(&progress)) {
                        enrollment_state = ENROLLMENT_STATE_COMPLETING;
                        /* Complete the enrolment process and add user to database */
                        ifx_result = ifx_complete_on_device_enrollment(&faceid_embeddings, &enrollment_config, &inference_config);
                        if (ifx_result == IFX_FACEID_RSLT_SUCCESS)
                        {
                            printf("[LCD_TASK] On-device enrolment completed automatically!\n");

                            if (CY_RRAM_SUCCESS != ifx_copy_embeddings_to_nvm((uint32_t) &enrolled_user_faceid_embeddings,
                                    (uint8_t *) &faceid_embeddings,
                                    sizeof(faceid_embeddings)))
                            {
                                printf("[LCD_TASK][ERROR] Failed to add enrolled user data to the NVM database\r\n");
                            }
                            else
                            {
                                printf("[LCD_TASK] Added enrolled user data to the NVM database successfully!\r\n");

                                touch_button_event[FACE_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_RELEASED;
                                touch_button_event[FACE_ENROL_BUTTON_ID].pressed = false;

                            }

                            /* Update user count after successful enrolment */
                            db_update_result = update_database_user_count();
                            if (db_update_result != IFX_FACEID_RSLT_SUCCESS) {
                                printf("[LCD_TASK][WARNING] Failed to update user count after enrolment: %d\n", db_update_result);
                            }
                        }
                        else if (ifx_result == IFX_FACEID_RSLT_DB_FULL)
                        {
                            printf("[LCD_TASK][ERROR] Database is full - cannot auto-complete enrolment\n");
                            trigger_database_full_overlay(); /* Show overlay warning */

                            /* Call the abort function to clean up */
                            ifx_result = ifx_abort_on_device_enrollment(&enrollment_config);
                            if (ifx_result != IFX_FACEID_RSLT_SUCCESS) {
                                printf("[LCD_TASK][ERROR] Failed to properly abort enrolment\n");
                            }
                        }
                        else
                        {
                            printf("[LCD_TASK][ERROR] Failed to complete auto-enrolment\n");
                        }
                        /* Reset enrolment state */
                        enrollment_state = ENROLLMENT_STATE_NONE;
                        enrollment_type = ENROLLMENT_TYPE_NONE;
                        current_display_mode = DISPLAY_MODE_INFERENCE;
                    }
                    if (enroll_result != IFX_FACEID_RSLT_SUCCESS) {
                        /* Handle serious errors if needed */
                    }
                }

                /* Use progress data to update display */
                update_enrollment_display(&prediction, &enrollment_config, enrollment_state, enrollment_type, &progress, enroll_result);
            }

#ifdef ENABLE_FACEID_MODEL_PROFILE
            ifx_face_id_print_profiling_info();
#endif /* ENABLE_FACEID_MODEL_PROFILE */


            /* Clear button handler */
            if (touch_button_event[CLEAR_ENROL_BUTTON_ID].pressed || fid_home_clear_enrolled_users_flag)

            {

                touch_button_event[CLEAR_ENROL_BUTTON_ID].pressed = false;

                fid_home_clear_enrolled_users_flag = false;
                printf("Clearing all enrolled users from database\n");
                /* Clear the user database */
                ifx_result = ifx_face_id_clear_all_enrolled_users(&faceid_embeddings, &inference_config);
                if (ifx_result == IFX_FACEID_RSLT_SUCCESS)
                {
                    printf("\n[LCD_TASK] Successfully cleared all enrolled users from the database\n");


                    if (CY_RRAM_SUCCESS != ifx_copy_embeddings_to_nvm((uint32_t) &enrolled_user_faceid_embeddings, (uint8_t *) &faceid_embeddings, sizeof(faceid_embeddings)))
                    {
                        printf("[LCD_TASK][ERROR] Failed to clear enrolled user data from the NVM database\r\n");
                    }
                    else
                    {
                        printf("[LCD_TASK] Successfully cleared all enrolled users from the NVM database\r\n");
                    }

                    /* Update user count after clearing */
                    db_update_result = update_database_user_count();
                    if (db_update_result != IFX_FACEID_RSLT_SUCCESS) {
                        printf("[LCD_TASK][WARNING] Failed to update user count after clearing: %d\n", db_update_result);
                    }
                }
                else
                {
                    printf("\n[LCD_TASK][ERROR] Failed to clear enrolled users database\n");
                }
            }


            /* Cancel button handler */
            if (touch_button_event[CANCEL_ENROL_BUTTON_ID].pressed || fid_home_cancel_enrollment_flag)

            {
                fid_home_cancel_enrollment_flag = false;

                touch_button_event[CANCEL_ENROL_BUTTON_ID].pressed = false;
                if (TOUCH_BUTTON_EVENT_PRESSED == touch_button_event[FACE_ENROL_BUTTON_ID].event)
                {
                    touch_button_event[FACE_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_RELEASED;
                    touch_button_event[FACE_ENROL_BUTTON_ID].pressed = false;
                }

                /* Check if we're in on-device enrolment */
                if (enrollment_state != ENROLLMENT_STATE_NONE && enrollment_type == ENROLLMENT_TYPE_ON_DEVICE)
                {

                    printf("[LCD_TASK] Cancelling on-device enrolment via BTN1\n");

                    /* Call the abort function to clean up */
                    ifx_result = ifx_abort_on_device_enrollment(&enrollment_config);
                    if (ifx_result != IFX_FACEID_RSLT_SUCCESS) {
                        printf("[LCD_TASK][ERROR] Failed to properly abort enrolment\n");
                    }

                    /* Reset enrolment state */
                    enrollment_state = ENROLLMENT_STATE_NONE;
                    enrollment_type = ENROLLMENT_TYPE_NONE;
                    current_display_mode = DISPLAY_MODE_INFERENCE;
                }

                else if (enrollment_state == ENROLLMENT_STATE_UART_TRANSFER) {
                    /* Can't cancel UART transfer mid-process */
                    printf("\n[LCD_TASK] Can't interrupt off-device enrolment during UART transfer\n");
                }
            }

#ifndef DISABLE_ONDEV_ENROLL

            /* Start/stop enrolment button handler */
            if (touch_button_event[FACE_ENROL_BUTTON_ID].pressed || touch_button_event[FACE_ENROL_BUTTON_ID].released || fid_home_start_enrollment_flag)

            {
                fid_home_start_enrollment_flag = false;
                /* If not enrolling, start on-device enrolment */
                if (enrollment_state == ENROLLMENT_STATE_NONE) {
                    /* CHECK DATABASE CAPACITY BEFORE STARTING ENROLLMENT */
                    if (is_database_full())
                    {
                        printf("[LCD_TASK] Cannot start enrolment - database is full (%d/%d users)\n",
                                current_user_count, MAX_N_USER);
                        trigger_database_full_overlay(); /* Show 5-second overlay warning */

                        touch_button_event[FACE_ENROL_BUTTON_ID].pressed = false;

                    }
                    else
                    {
                        printf("[LCD_TASK] Starting on-device enrolment - waiting for high-quality reference face\n");
                        enrollment_state = ENROLLMENT_STATE_WAITING_FOR_REFERENCE;
                        enrollment_type = ENROLLMENT_TYPE_ON_DEVICE;
                        current_display_mode = DISPLAY_MODE_ENROLLMENT;

                        touch_button_event[FACE_ENROL_BUTTON_ID].pressed = false;

                    }
                }
                /* If already in the collecting state, complete enrolment */
                else if (enrollment_state == ENROLLMENT_STATE_COLLECTING &&
                        enrollment_type == ENROLLMENT_TYPE_ON_DEVICE)
                {
                    printf("[LCD_TASK] Completing on-device enrolment...\n");
                    enrollment_state = ENROLLMENT_STATE_COMPLETING;

                    /* Complete the enrolment process and add user to database */
                    ifx_result = ifx_complete_on_device_enrollment(&faceid_embeddings, &enrollment_config, &inference_config);
                    if (ifx_result == IFX_FACEID_RSLT_SUCCESS)
                    {
                        printf("[LCD_TASK] On-device enrolment completed successfully!\n");

                        if (CY_RRAM_SUCCESS != ifx_copy_embeddings_to_nvm((uint32_t) &enrolled_user_faceid_embeddings,
                                (uint8_t *) &faceid_embeddings,
                                sizeof(faceid_embeddings)))
                        {
                            printf("[LCD_TASK][ERROR] Failed to add enrolled user data to the NVM database\r\n");
                        }
                        else
                        {
                            printf("[LCD_TASK] Added enrolled user data to the NVM database successfully!\r\n");
                        }

                        /* Update user count after successful enrolment */
                        db_update_result = update_database_user_count();
                        if (db_update_result != IFX_FACEID_RSLT_SUCCESS) {
                            printf("[LCD_TASK][WARNING] Failed to update user count after enrolment: %d\n", db_update_result);
                        }
                    }
                    else if (ifx_result == IFX_FACEID_RSLT_DB_FULL)
                    {
                        printf("[LCD_TASK][ERROR] Database is full - cannot add new user\n");
                        trigger_database_full_overlay(); /* Show overlay warning */

                        /* Call the abort function to clean up */
                        ifx_result = ifx_abort_on_device_enrollment(&enrollment_config);
                        if (ifx_result != IFX_FACEID_RSLT_SUCCESS) {
                            printf("[LCD_TASK][ERROR] Failed to properly abort enrolment\n");
                        }
                    }
                    else
                    {
                        printf("[LCD_TASK][ERROR] Failed to complete on-device enrolment\n");

                        /* Call the abort function to clean up */
                        ifx_result = ifx_abort_on_device_enrollment(&enrollment_config);
                        if (ifx_result != IFX_FACEID_RSLT_SUCCESS) {
                            printf("[LCD_TASK][ERROR] Failed to properly abort enrolment\n");
                        }
                    }

                    /* Reset enrolment state */
                    enrollment_state = ENROLLMENT_STATE_NONE;
                    enrollment_type = ENROLLMENT_TYPE_NONE;
                    current_display_mode = DISPLAY_MODE_INFERENCE;

                    touch_button_event[FACE_ENROL_BUTTON_ID].released = false;

                }
                /* If in waiting state, cancel enrolment */
                else if (enrollment_state == ENROLLMENT_STATE_WAITING_FOR_REFERENCE &&
                        enrollment_type == ENROLLMENT_TYPE_ON_DEVICE) {
                    printf("[LCD_TASK] Cancelling on-device enrolment\n");
                    enrollment_state = ENROLLMENT_STATE_NONE;
                    enrollment_type = ENROLLMENT_TYPE_NONE;
                    current_display_mode = DISPLAY_MODE_INFERENCE;

                    touch_button_event[FACE_ENROL_BUTTON_ID].released = false;

                }
                /* If in UART transfer state, do nothing (can't cancel easily) */
                else if (enrollment_state == ENROLLMENT_STATE_UART_TRANSFER) {
                    printf("[LCD_TASK] Can't interrupt off-device enrolment during UART transfer\n");

                    touch_button_event[FACE_ENROL_BUTTON_ID].released = false;

                }
            }
#endif /* DISABLE_ONDEV_ENROLL */

            time_model_end = ifx_time_get_ms_f();

#ifdef SHOW_INFERENCING_TIME
            model_inferencing_time = (time_model_end - time_model_start);
#else
            fps = 1000.0f / (time_model_end - time_model_start);
#endif /* SHOW_INFERENCING_TIME */

            /* Use different display update based on mode */
            update_lcd_display(&prediction, current_display_mode, current_enrollment_feedback, &enrollment_config);



            /* Check for start face enrolment button state */
            switch (touch_button_event[FACE_ENROL_BUTTON_ID].event)
            {
            case TOUCH_BUTTON_EVENT_RELEASED: /* Start face enrolment button in released state */
                /* Display start face enrolment button */
                ifx_lcd_display_Rect(FACE_ENROL_BTN_X_POS, FACE_ENROL_BTN_Y_POS, (uint8_t *) start_face_enrolment_btn_img_map, START_FACE_ENROLMENT_BUTTON_WIDTH, START_FACE_ENROLMENT_BUTTON_HEIGHT);
                break;

            case TOUCH_BUTTON_EVENT_PRESSED: /* Save face enrolment button in pressed state */
                /* Display save face enrolment button */
                ifx_lcd_display_Rect(FACE_ENROL_BTN_X_POS, FACE_ENROL_BTN_Y_POS, (uint8_t *) save_face_enrolment_btn_img_map, SAVE_FACE_ENROLMENT_BUTTON_WIDTH, SAVE_FACE_ENROLMENT_BUTTON_HEIGHT);
                break;
            }

            /* Check if clear enrolled users button is pressed */
            if (TOUCH_BUTTON_EVENT_PRESSED == touch_button_event[CLEAR_ENROL_BUTTON_ID].event)
            {
                /* Display clear enrolled users button in pressed state */
                touch_button_event[CLEAR_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_RELEASED;
                ifx_lcd_display_Rect(CLEAR_BTN_X_POS, CLEAR_BTN_Y_POS, (uint8_t *) clear_pressed_btn_img_map, CLEAR_PRESSED_BUTTON_WIDTH, CLEAR_PRESSED_BUTTON_HEIGHT);
            }
            else
            {
                /* Display clear enrolled users button in released state */
                ifx_lcd_display_Rect(CLEAR_BTN_X_POS, CLEAR_BTN_Y_POS, (uint8_t *) clear_released_btn_img_map, CLEAR_RELEASED_BUTTON_WIDTH, CLEAR_RELEASED_BUTTON_HEIGHT);
            }

            /* Check if cancel face enrolment button is pressed */
            if (TOUCH_BUTTON_EVENT_PRESSED == touch_button_event[CANCEL_ENROL_BUTTON_ID].event)
            {
                /* Display cancel face enrolment button in pressed state */
                touch_button_event[CANCEL_ENROL_BUTTON_ID].event = TOUCH_BUTTON_EVENT_RELEASED;
                ifx_lcd_display_Rect(CANCEL_BTN_X_POS, CANCEL_BTN_Y_POS, (uint8_t *) cancel_pressed_btn_img_map, CANCEL_PRESSED_BUTTON_WIDTH, CANCEL_PRESSED_BUTTON_HEIGHT);
            }
            else
            {
                /* Display cancel face enrolment button in released state */
                ifx_lcd_display_Rect(CANCEL_BTN_X_POS, CANCEL_BTN_Y_POS, (uint8_t *) cancel_released_btn_img_map, CANCEL_RELEASED_BUTTON_WIDTH, CANCEL_RELEASED_BUTTON_HEIGHT);
            }


            /* switch frames to latest */
            VG_switch_frame();

            __DMB();
        }

        else
        {
            for (uint8_t i = 0; i < NUM_IMAGE_BUFFERS; i++)
            {
                vg_lite_clear(renderTarget, NULL, BLACK_COLOR);
                vg_lite_finish();


                if (camera_not_supported)
                {
                    /* Display camera not supported image */
                    ifx_lcd_display_Rect(CAMERA_NOT_SUPPORTED_IMG_X_POS , CAMERA_NOT_SUPPORTED_IMG_Y_POS,  (uint8_t *) camera_not_supported_img_map, CAMERA_NOT_SUPPORTED_IMG_WIDTH, CAMERA_NOT_SUPPORTED_IMG_HEIGHT);
                }
                else
                {
                    /* Display no camera image */
                    ifx_lcd_display_Rect(NO_CAMERA_IMG_X_POS , NO_CAMERA_IMG_Y_POS,  (uint8_t *) no_camera_img_map, NO_CAMERA_IMG_WIDTH, NO_CAMERA_IMG_HEIGHT);
                }

                VG_switch_frame();
                __DMB();

                /* Delay between publishing frame buffers to the display */
                vTaskDelay(pdMS_TO_TICKS(30));
            }
        }

    }
}

/* [] END OF FILE */
