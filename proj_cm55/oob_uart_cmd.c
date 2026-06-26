/******************************************************************************
* File Name        : oob_uart_cmd.c
*
* Description      : Polling RX → frame parse → response via mtb_hal_uart_put →
*                    print result. Uses ONLY your HAL read pattern (readable →
*                    get one byte).
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
#include "oob_uart_cmd.h"
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include "FreeRTOS.h"
#include "cy_scb_uart.h"
#include "lcd_task.h"
#include "task.h"
#include "cy_pdl.h"
#include "cybsp.h"
#ifdef ENABLE_WEB_STREAMING
#include "uart_protocol.h"
#include "jpeg_encoder.h"
#include "chunked_stream.h"
#endif

/*******************************************************************************
* Macros
*******************************************************************************/
#define UART_INTR_NUM  ((IRQn_Type) CYBSP_TENXER_UART_IRQ)
#define APP_CMD_RECEIVED_DATA_LEN        0x05

/*******************************************************************************
* Global Variables
*******************************************************************************/
cy_stc_scb_uart_context_t MODBUS_UART_context;
bool boReadUARTData;
/* The application should set this to the active app context (0x01..0x05). */
volatile uint8_t g_oob_current_app_id = OOB_APP_FACE_ID; /* default application */

#ifdef ENABLE_WEB_STREAMING
/*******************************************************************************
* Streaming Global Variables
*******************************************************************************/
/* CRITICAL: Allocate large buffers in EXTERNAL PSRAM to preserve SOCMEM */
CY_SECTION(".cy_xip")
__attribute__((aligned(128), used))
static uint8_t g_jpeg_buffer[64 * 1024];  /* 64 KB for compressed JPEG */

CY_SECTION(".cy_xip")
__attribute__((aligned(128), used))
static uint8_t g_frame_buffer[64 * 1024];  /* 64 KB for frame assembly */

/* Double buffering for RGB565 frames (prevent race conditions with USB camera task) */
CY_SECTION(".cy_xip")
__attribute__((aligned(128), used))
static uint16_t g_rgb565_buffer_0[CAMERA_WIDTH * CAMERA_HEIGHT];  /* 153.6 KB */

CY_SECTION(".cy_xip")
__attribute__((aligned(128), used))
static uint16_t g_rgb565_buffer_1[CAMERA_WIDTH * CAMERA_HEIGHT];  /* 153.6 KB */

/* Streaming state */
static volatile bool g_streaming = false;
static volatile uint8_t g_write_buffer_idx = 0;  /* Buffer being written by camera (0 or 1) */
static volatile uint8_t g_read_buffer_idx = 0;   /* Buffer being read by UART task (0 or 1) */
static volatile bool g_frame_ready = false;      /* New frame available flag */
static uint32_t g_frame_dropped = 0;             /* Dropped frames counter */
static volatile uint8_t g_faceid_status = 0;     /* Face ID detection status for current frame */

/*******************************************************************************
* Functions
*******************************************************************************/

/* Forward declarations */
void stream_push_frame(const uint16_t *rgb565);

/*******************************************************************************
* Function Name: uart_send_blocking
********************************************************************************
* Summary:
*  Send data over UART in blocking mode (waits for TX FIFO space).
*
* Parameters:
*  data: Pointer to data buffer
*  len: Number of bytes to send
*
* Return:
*  Number of bytes sent
*******************************************************************************/
static uint32_t uart_send_blocking(const uint8_t *data, uint32_t len)
{
    uint32_t sent = 0;
    uint32_t fifo_size = Cy_SCB_GetFifoSize(CYBSP_TENXER_UART_HW);

    while (sent < len)
    {
        /* Get available space in TX FIFO */
        uint32_t in_fifo = Cy_SCB_UART_GetNumInTxFifo(CYBSP_TENXER_UART_HW);
        uint32_t space = fifo_size - in_fifo;

        if (space > 0)
        {
            /* Fill FIFO with as many bytes as possible */
            uint32_t to_send = (len - sent > space) ? space : (len - sent);

            for (uint32_t i = 0; i < to_send; i++)
            {
                Cy_SCB_UART_Put(CYBSP_TENXER_UART_HW, (uint32_t)data[sent]);
                sent++;
            }
        } else {
            /* FIFO full, yield briefly to avoid busy-waiting */
            taskYIELD();
        }
    }

    return sent;
}

/*******************************************************************************
* Function Name: is_streaming_active
********************************************************************************
* Summary:
*  Check if UART streaming is currently active.
*
* Parameters:
*  void
*
* Return:
*  bool: true if streaming, false otherwise
*******************************************************************************/
bool is_streaming_active(void)
{
    return g_streaming;
}

/*******************************************************************************
* Function Name: stream_push_frame_with_faceid
********************************************************************************
* Summary:
*  Push a new frame with Face ID detection status for streaming.
*
* Parameters:
*  rgb565: Pointer to RGB565 frame buffer from camera (320x240)
*  faceid_status: Face ID status byte (0=no face, 1=unknown, 2=recognized)
*
* Return:
*  void
*******************************************************************************/
void stream_push_frame_with_faceid(const uint16_t *rgb565, uint8_t faceid_status)
{
    g_faceid_status = faceid_status;  /* Store Face ID status */
    stream_push_frame(rgb565);        /* Push frame normally */
}

/*******************************************************************************
* Function Name: stream_send_faceid_metadata
********************************************************************************
* Summary:
*  Send Face ID metadata frame containing bounding boxes and user IDs.
*  This should be called AFTER sending the image frame.
*
* Parameters:
*  metadata: Pointer to Face ID metadata structure
*
* Return:
*  true if sent successfully, false on error
*******************************************************************************/
bool stream_send_faceid_metadata(const uart_faceid_metadata_t *metadata)
{
    if (!metadata || !g_streaming)
    {
        return false;
    }

    /* Encode metadata frame */
    int frame_size = uart_encode_faceid_metadata(
        g_frame_buffer,
        sizeof(g_frame_buffer),
        metadata,
        CAMERA_WIDTH,
        CAMERA_HEIGHT
    );

    if (frame_size <= 0)
    {
        return false;
    }

    /* Transmit metadata frame */
    uint32_t sent = uart_send_blocking(g_frame_buffer, (uint32_t)frame_size);

    return (sent == (uint32_t)frame_size);
}

/*******************************************************************************
* Function Name: stream_push_frame
********************************************************************************
* Summary:
*  Push a new frame from the camera for streaming (called from lcd_task).
*  Uses double buffering to prevent race conditions with UART streaming task.
*
* Parameters:
*  rgb565: Pointer to RGB565 frame buffer from camera (320x240)
*
* Return:
*  void
*******************************************************************************/
void stream_push_frame(const uint16_t *rgb565)
{
    if (!g_streaming || !rgb565)
    {
        return;
    }

    /* Check if previous frame is still being processed */
    if (g_frame_ready)
    {
        /* UART task hasn't processed the last frame yet - drop this one */
        g_frame_dropped++;
        return;
    }

    /* Debug: Print once when first frame is queued */
    static bool first_frame = true;
    if (first_frame)
    {
        printf("[UART] First frame queued for streaming\r\n");
        first_frame = false;
    }

    /* Select write buffer (the one NOT being read) */
    uint8_t write_idx = g_write_buffer_idx;
    uint16_t *dst_buffer = (write_idx == 0) ? g_rgb565_buffer_0 : g_rgb565_buffer_1;

    /* Copy frame to double buffer with memory barrier to prevent race conditions */
    __DMB();  /* Data Memory Barrier - ensure all previous memory ops complete */
    memcpy(dst_buffer, rgb565, CAMERA_WIDTH * CAMERA_HEIGHT * sizeof(uint16_t));
    __DMB();  /* Ensure memcpy completes before updating indices */

    /* Atomically swap buffers and signal frame ready */
    g_read_buffer_idx = write_idx;           /* UART task will read from this buffer */
    g_write_buffer_idx = (write_idx + 1) & 1; /* Next camera frame goes to other buffer */
    __DMB();
    g_frame_ready = true;                     /* Signal new frame available */
}

/*******************************************************************************
* Function Name: handle_stream_command
********************************************************************************
* Summary:
*  Handle single-character streaming commands:
*    'S'/'s' - Start streaming
*    'Q'/'q' - Stop streaming
*    'E'/'e' - Start enrollment
*    'C'/'c' - Cancel enrollment
*    'X'/'x' - Clear all enrolled users
*
* Parameters:
*  cmd: Command character
*
* Return:
*  void
*******************************************************************************/
static void handle_stream_command(char cmd)
{
    switch (cmd)
    {
        case 'S':
        case 's':
            chunked_stream_start();
            printf("[UART Stream] Chunked streaming started\r\n");
            break;

        case 'Q':
        case 'q':
            chunked_stream_stop();
            printf("[UART Stream] Chunked streaming stopped\r\n");
            break;

        case 'E':
        case 'e':
            fid_home_start_enrollment_flag = true;
            printf("[UART Enroll] Starting enrollment via UART command\r\n");
            break;

        case 'C':
        case 'c':
            fid_home_cancel_enrollment_flag = true;
            printf("[UART Enroll] Canceling enrollment via UART command\r\n");
            break;

        case 'X':
        case 'x':
            fid_home_clear_enrolled_users_flag = true;
            printf("[UART Enroll] Clearing all enrolled users via UART command\r\n");
            break;

        case 'F':
        case 'f':
            fid_home_complete_enrollment_flag = true;
            printf("[UART Enroll] Force completing enrollment via UART command\r\n");
            break;

        case '0':  /* JPEG Quality BEST */
            chunked_stream_set_quality(0);
            break;

        case '1':  /* JPEG Quality HIGH */
            chunked_stream_set_quality(1);
            break;

        case '2':  /* JPEG Quality MEDIUM */
            chunked_stream_set_quality(2);
            break;

        case '3':  /* JPEG Quality LOW */
            chunked_stream_set_quality(3);
            break;

        default:
            /* Ignore unknown commands */
            break;
    }
}
#endif

/*******************************************************************************
* Function Name: UART_Isr
********************************************************************************
* Summary:
*  Interrupt service routine for the UART, handles UART interrupts by calling the
*  SCB UART interrupt handler and sets a flag to indicate that UART data is ready
*  to be read.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void UART_Isr( void )
{
    Cy_SCB_UART_Interrupt(CYBSP_TENXER_UART_HW, &MODBUS_UART_context);
    boReadUARTData = true;
}


/*******************************************************************************
* Function Name: initUART
********************************************************************************
* Summary:
*  Initializes the UART peripheral with the specified configuration, sets up the
*  UART interrupt service routine, enables interrupts, and activates the UART for
*  operation.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void initUART(void)
{
    /* Configure UART to operate */
    (void) Cy_SCB_UART_Init(CYBSP_TENXER_UART_HW, &CYBSP_TENXER_UART_config, &MODBUS_UART_context);

    /* Populate configuration structure (code specific for CM4) */
    cy_stc_sysint_t uartIntrConfig =
    {
        .intrSrc      = UART_INTR_NUM,
        .intrPriority = 4,
    };
    /* Hook interrupt service routine and enable interrupt */
    (void) Cy_SysInt_Init(&uartIntrConfig, &UART_Isr);
    NVIC_EnableIRQ(UART_INTR_NUM);

    __enable_irq();

    /* Enable UART to operate */
    Cy_SCB_UART_Enable(CYBSP_TENXER_UART_HW);
    printf("successfully init CYBSP_TENXER_UART_HW \r\n");
#ifdef ENABLE_WEB_STREAMING
    Cy_SCB_UART_Put(CYBSP_TENXER_UART_HW, 'a');
#endif
}


/*******************************************************************************
* Function Name: OOB_CommandFeedback
********************************************************************************
* Summary:
*  Sends a UART response frame with a start byte, length, sequence number, and
*  response byte (ACK, NAK, or app ID for specific global commands). Handles
*  special cases for global WHO_AM_I and BACK_TO_CAROUSEL commands.
*
* Parameters:
*  uint8_t seq: Sequence number of the command
*  uint8_t app: Application ID
*  uint8_t opcode: Operation code
*  bool success: Indicates if the command was successful (true for ACK, false
*                for NAK)
*
* Return:
*  void
*
*******************************************************************************/
void OOB_CommandFeedback(uint8_t seq, uint8_t app, uint8_t opcode, bool success)
{
    /* Decide payload byte */
    uint8_t resp_byte = success ? OOB_RSP_ACK : OOB_RSP_NAK;

    /* Special case: global WHO_AM_I / BACK_TO_CAROUSEL return current app id */
    if ((app == OOB_APP_GLOBAL) && (opcode == WHO_AM_I || opcode == BACK_TO_CAROUSEL))
    {
        resp_byte = (uint8_t)g_oob_current_app_id;
    }

    Cy_SCB_UART_Put(CYBSP_TENXER_UART_HW, (uint32_t)OOB_RSP_START_BYTE);
    Cy_SCB_UART_Put(CYBSP_TENXER_UART_HW, (uint32_t)0x02u);
    Cy_SCB_UART_Put(CYBSP_TENXER_UART_HW, (uint32_t)seq);
    Cy_SCB_UART_Put(CYBSP_TENXER_UART_HW, (uint32_t)resp_byte);
}


/*******************************************************************************
* Function Name: OOB_UartCMD_Task
********************************************************************************
* Summary:
*  Task that polls the UART for incoming out-of-band (OOB) command frames,
*  validates the frame structure and content, processes commands based on
*  application ID and opcode (e.g., face ID enrolment, cancel, clear users,
*  or global commands), and sends appropriate feedback responses. Resets the
*  frame buffer on completion or error.
*
* Parameters:
*  void *args: Task argument
*
* Return:
*  void
*
*******************************************************************************/
void OOB_UartCMD_Task(void *args)
{
    CY_UNUSED_PARAMETER(args);
    uint8_t rx_char;
    size_t available;

    /* Simple frame buffer/state (max: 1+1+(3+8)=13 bytes) */
    uint8_t  frame[13];
    uint8_t  idx = 0;          /* buffered so far */
    uint8_t  want = 0;         /* total wire bytes expected: 2 + Len */

    uint8_t len;
    uint8_t seq;
    uint8_t app;
    uint8_t op;

    initUART();
#ifdef ENABLE_WEB_STREAMING
    chunked_stream_init();  /* Initialize chunked streaming */
#endif
    for (;;)
    {
        /* Process chunked streaming (sends one chunk per iteration) */
#ifdef ENABLE_WEB_STREAMING
        chunked_stream_process();
#endif
        available = Cy_SCB_UART_GetNumInRxFifo(CYBSP_TENXER_UART_HW);
        if (available)
        {
            rx_char = Cy_SCB_UART_Get(CYBSP_TENXER_UART_HW);
#ifdef ENABLE_WEB_STREAMING
            /* Check for streaming commands (single character) */
            if (rx_char == 'S' || rx_char == 's' ||
                rx_char == 'Q' || rx_char == 'q' ||
                rx_char == 'E' || rx_char == 'e' ||
                rx_char == 'C' || rx_char == 'c' ||
                rx_char == 'F' || rx_char == 'f' ||
                rx_char == 'X' || rx_char == 'x' ||
                rx_char == '0' || rx_char == '1' || rx_char == '2' || rx_char == '3') {
                handle_stream_command((char)rx_char);
                continue;  /* Skip frame parsing for stream commands */
            }
#endif
            if (idx == 0)
            {
                if (rx_char == OOB_REQ_START_BYTE)
                {
                    frame[idx++] = rx_char;         /* Start */
                }
                /* else ignore until Start */
            }
            else if (idx == 1)
            {
                frame[idx++] = rx_char;             /* Len (excludes Start & Len) */
                /* expected wire size = Start(1) + Len(1) + Len */
                want = (uint8_t)(2u + rx_char);

                /* Quick sanity: must at least hold Seq+App+Opcode */
                if (rx_char < OOB_REQ_MIN_LEN_NO_PAYLOAD || want > sizeof(frame))
                {
                    /* invalid length; resync */
                    idx = 0;
                    want = 0;
                }
            }
            else
            {
                if (idx < sizeof(frame))
                {
                    frame[idx++] = rx_char;
                }
                if (want && idx == want)
                {
                    /* ---- Validate frame ---- */
                    bool ok = true;

                    len = frame[1];           /* excludes start & len */
                    seq = frame[2];
                    app = frame[3];
                    op  = frame[4];

                    /* payload length derived from len: len = 3 + payload_len */
                    uint8_t payload_len = (len >= 3u) ? (uint8_t)(len - 3u) : 0xFFu;

                    /* Basic checks */
                    if (frame[0] != OOB_REQ_START_BYTE) ok = false;
                    if (seq == 0x00u)                   ok = false;
                    if (payload_len > OOB_REQ_MAX_PAYLOAD) ok = false;

                    /* Compare against current app context if AppID != 0x00 */
                    if (ok && app != OOB_APP_GLOBAL && app != (uint8_t)g_oob_current_app_id)
                        ok = false;

                    /* Opcode checks per AppID */
                    if (ok)
                    {
                        switch (app)
                        {
                        case OOB_APP_FACE_ID:
                            if(op == FID_HOME_START_ENROLLMENT)
                            {
                                fid_home_start_enrollment_flag = true;
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);

                            }
                            else if(op == FID_HOME_CANCLE_ENROLLMENT)
                            {
                                fid_home_cancel_enrollment_flag = true;
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);

                            }
                            else if(op == FID_HOME_CLEAR_ENROLLED_USERS)
                            {
                                fid_home_clear_enrolled_users_flag = true;
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);
                            }
                            break;

                        case OOB_APP_GLOBAL:
                            if(op == WHO_AM_I)
                            {
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);
                            }
                            else if(op==BACK_TO_CAROUSEL)
                            {
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);
                                uart_de_init();
                                vTaskDelay(pdMS_TO_TICKS(1));
                            }
                            break;

                        default:
                            ok = false;
                            OOB_CommandFeedback(seq, app, op, ok);
                            break;
                        }
                    }
                    else{
                        ok = false;
                        OOB_CommandFeedback(seq, app, op, ok);
                    }

                    /* Reset for next frame */
                    idx = 0;
                    want = 0;
                }

                /* Overflow guard */
                if (idx >= sizeof(frame))
                {
                    idx = 0;
                    want = 0;
                }
            }
        }

        /* Only delay when not actively streaming to maximize FPS
         * When streaming, taskYIELD() in uart_send_blocking provides enough context switching */
#ifdef ENABLE_WEB_STREAMING
        if (!g_streaming)
        {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
#else
    vTaskDelay(pdMS_TO_TICKS(50)); /* keep CPU cool, same cadence you use */
#endif

    }
}


/*******************************************************************************
* Function Name: uart_de_init
********************************************************************************
* Summary:
*  Deinitializes the UART peripheral by disabling it and releasing its resources.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
void uart_de_init( void )
{
    Cy_SCB_UART_Disable(CYBSP_TENXER_UART_HW, &MODBUS_UART_context);
    Cy_SCB_UART_DeInit(CYBSP_TENXER_UART_HW);

}

/* [] END OF FILE */
