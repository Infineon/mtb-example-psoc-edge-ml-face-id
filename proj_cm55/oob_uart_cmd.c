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
 * (c) 2023-2026, Infineon Technologies AG, or an affiliate of Infineon
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
#include "FreeRTOS.h"
#include "cy_scb_uart.h"
#include "lcd_task.h"
#include "task.h"
#include "cy_pdl.h"
#include "cybsp.h"

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
bool fid_home_start_enrollment_flag = false;
bool fid_home_cancel_enrollment_flag = false;
bool fid_home_clear_enrolled_users_flag = false;
/* The application should set this to the active app context (0x01..0x05). */
volatile uint8_t g_oob_current_app_id = OOB_APP_FACE_ID; /* default application */

/*******************************************************************************
* Functions
*******************************************************************************/

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

    for (;;)
    {
        available = Cy_SCB_UART_GetNumInRxFifo(CYBSP_TENXER_UART_HW);
        if (available)
        {
            rx_char = Cy_SCB_UART_Get(CYBSP_TENXER_UART_HW);
            if (idx == 0)
            {
                if (rx_char == OOB_REQ_START_BYTE) {
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
                if (rx_char < OOB_REQ_MIN_LEN_NO_PAYLOAD || want > sizeof(frame)) {
                    /* invalid length; resync */
                    idx = 0;
                    want = 0;
                }
            }
            else
            {
                if (idx < sizeof(frame)) {
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
                            if(op == FID_HOME_START_ENROLLMENT){
                                fid_home_start_enrollment_flag = true;
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);

                            }else if(op == FID_HOME_CANCLE_ENROLLMENT){
                                fid_home_cancel_enrollment_flag = true;
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);

                            }else if(op == FID_HOME_CLEAR_ENROLLED_USERS){
                                fid_home_clear_enrolled_users_flag = true;
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);

                            }
                            break;

                        case OOB_APP_GLOBAL:
                            if(op == WHO_AM_I){
                                ok = true;
                                OOB_CommandFeedback(seq, app, op, ok);
                            }else if(op==BACK_TO_CAROUSEL){
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
                if (idx >= sizeof(frame)) {
                    idx = 0;
                    want = 0;
                }
            }
        }

        vTaskDelay(pdMS_TO_TICKS(50)); /* keep CPU cool, same cadence you use */
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
