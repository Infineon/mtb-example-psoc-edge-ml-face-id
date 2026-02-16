/******************************************************************************
* File Name        : oob_uart_cmd.h
*
* Description      : OOB UART command parser (polling) + minimal response.
*                     Request : [ 0xCC | Len | Seq | AppID | Opcode | 
*                               Payload(0..8) ]
*                               Len excludes Start & Len  (Len = 3 + payload_len)
*                     Response: [ 0xAA | 0x02 | Seq | Resp ]
*                             Resp = 0xFA (OK) / 0xFB (FAIL)
*                             Special: if AppID==0x00 and Opcode==WHO_AM_I(0xDD) 
*                                      or BACK_TO_CAROUSEL(0xEE),
*                                    Resp = current app id (g_oob_current_app_id).
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

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <stdint.h>
#include <stdbool.h>
#include "cy_scb_uart.h"
#include "FreeRTOS.h"

/*******************************************************************************
 * Macros
 *******************************************************************************/
/* --------- Protocol constants --------- */
#define OOB_REQ_START_BYTE          ((uint8_t)0xCC)
#define OOB_RSP_START_BYTE          ((uint8_t)0xAA)

#define OOB_RSP_ACK                 ((uint8_t)0xFA)
#define OOB_RSP_NAK                 ((uint8_t)0xFB)

#define OOB_SEQ_MIN                 ((uint8_t)0x01)
#define OOB_SEQ_MAX                 ((uint8_t)0xFF)

#define OOB_REQ_MIN_LEN_NO_PAYLOAD  ((uint8_t)0x03) /* Seq(1)+App(1)+Op(1) */
#define OOB_REQ_MAX_PAYLOAD         (8u)

/* --------- Application Identifiers --------- */
#define OOB_APP_GLOBAL                  ((uint8_t)0x00)
#define OOB_APP_EDGE_DEMO_CAROUSEL      ((uint8_t)0x01)
#define OOB_APP_MUSIC_PLAYER            ((uint8_t)0x02)
#define OOB_APP_FACE_ID                 ((uint8_t)0x03)
#define OOB_APP_RPS                     ((uint8_t)0x04)
#define OOB_APP_VIDEO_PLAYER            ((uint8_t)0x05)

/* --------- Opcodes --------- */
/* Global (AppID 0x00) */
#define WHO_AM_I                        ((uint8_t)0xDD)
#define BACK_TO_CAROUSEL                ((uint8_t)0xEE)

/* Edge Demo (AppID 0x01) */
#define ED_HOME_SWIPE_LEFT              ((uint8_t)0x01)
#define ED_HOME_SWIPE_RIGHT             ((uint8_t)0x02)
#define ED_HOME_SELECT_OOB              ((uint8_t)0x03)
#define ED_HOME_GFXTOUCH_MODE           ((uint8_t)0x04)
#define ED_HOME_VOICE_MODE              ((uint8_t)0x05)
#define ED_HOME_GESTURE_MODE            ((uint8_t)0x06)
#define ED_OOBINFO_CLOSE                ((uint8_t)0x07)
#define ED_OOBINFO_PLAY_OOB             ((uint8_t)0x08)

/* Music Player (AppID 0x02) */
#define MP_HOME_TOGGLE_PLAY_PAUSE       ((uint8_t)0x01)
#define MP_HOME_NEXT                    ((uint8_t)0x02)
#define MP_HOME_PREV                    ((uint8_t)0x03)
#define MP_HOME_TOGGLE_SHUFFLE          ((uint8_t)0x04)
#define MP_HOME_TOGGLE_REPEAT           ((uint8_t)0x05)
#define MP_HOME_VOL_UP_ONE_STEP         ((uint8_t)0x06)
#define MP_HOME_VOL_DOWN_ONE_STEP       ((uint8_t)0x07)
#define MP_HOME_INFO                    ((uint8_t)0x08)
#define MP_HOME_INFO_CLOSE              ((uint8_t)0x09)
#define MP_HOME_SHOW_TRACK_LIST         ((uint8_t)0x0A)
#define MP_TLIST_BACK_TO_PLAYER         ((uint8_t)0x0B)
#define MP_TLIST_TOGGLE_PLAY_PAUSE_T1   ((uint8_t)0x0C)
#define MP_TLIST_TOGGLE_PLAY_PAUSE_T2   ((uint8_t)0x0D)
#define MP_TLIST_TOGGLE_PLAY_PAUSE_T3   ((uint8_t)0x0E)
#define MP_TLIST_TOGGLE_PLAY_PAUSE_T4   ((uint8_t)0x0F)
#define MP_TLIST_TOGGLE_PLAY_PAUSE_T5   ((uint8_t)0x10)

/* Face ID (AppID 0x03) — per examples (0x01..0x03) */
#define FID_HOME_START_ENROLLMENT       ((uint8_t)0x01)
#define FID_HOME_CANCLE_ENROLLMENT      ((uint8_t)0x02)
#define FID_HOME_CLEAR_ENROLLED_USERS   ((uint8_t)0x03)

/*******************************************************************************
* Global Variables
*******************************************************************************/
void OOB_UartCMD_Task(void *args);
extern volatile uint8_t g_oob_current_app_id;
void OOB_CommandFeedback(uint8_t seq, uint8_t app, uint8_t opcode, bool success);
void uart_de_init( void );

#ifdef __cplusplus
}
#endif

/* [] END OF FILE */
