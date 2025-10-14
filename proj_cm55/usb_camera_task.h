/******************************************************************************
* File Name        : usb_camera_task.h
*
* Description      : This is the header file of USB Webcam functions.
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

#ifndef _USB_CAMERA_TASK_H_
#define _USB_CAMERA_TASK_H_

#ifdef __cplusplus
extern "C" {
#endif

/*******************************************************************************
* Header Files
*******************************************************************************/
#include <stdio.h>
/* FreeRTOS header file */
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"

/*******************************************************************************
* Macros
*******************************************************************************/
/* YUYV 422 encoding - 2 bytes per pixel*/
#define CAMERA_BUFFER_SIZE              ((CAMERA_WIDTH) * (CAMERA_HEIGHT) * 2) 
#define FORMAT                          USBH_VIDEO_VS_FORMAT_UNCOMPRESSED

/*
 Typical frame interval values:
 333333  (30  FPS)
 400000  (25  FPS)    // NOT work for Logitech C920e
 416666
 500000  (20  FPS)
 666666  (15  FPS)
 1000000 (10  FPS)
 1333333 (7.5 FPS)
 2000000 (5   FPS)
 This value is given in 100ns steps, 1s/333333*100ns = 30 frames per second
*/
 /* configurable FPS Logitech C920 HD */
#define FRAME_INTERVAL_1                   (1000000)
#define FRAME_INTERVAL_2                   (333332) /* 30 FPS 0.3 MP cam */
#define FRAME_INTERVAL                     (333333) /* 30 FPS 2 MP cam */
#define MAX_VIDEO_INTERFACES               (4)
#define TASK_PRIO_USBH_MAIN                (configMAX_PRIORITIES - 2)
#define TASK_PRIO_USBH_ISR                 (configMAX_PRIORITIES - 1)
#define LOGI_TECH_C920_VID                 (0x046D)
#define LOGI_TECH_C920_PID                 (0x08E5)      /* Logitech C920 */
#define HBV_CAM_0P3_VID                    (0x058F)      /* 0.3MP HBVCAM */
#define HBV_CAM_0P3_PID                    (0x5608)
#define HBV_CAM_2P0_VID                    (0x05A3)      /* 2MP HBVCAM */
#define HBV_CAM_2P0_PID                    (0x2B01)

/* USB Camera constants */
#define USB_DEVICE_CONNECTED               (1U)
#define USB_DEVICE_DISCONNECTED            (0U)
#define USB_STREAM_ERROR_THRESHOLD         (10U)
#define USB_MAILBOX_NO_WAIT                (0U)
#define USB_HEALTH_CHECK_INTERVAL_MS       (2000U)
#define USB_FRAME_TIMEOUT_MS               (10000U)
#define USB_BUFFER_CLEAR_VALUE             (0U)
#define USB_STRING_BUFFER_SIZE             (128U)
#define USB_MB_EVENT_SIZE                  (1U)

/* Reset/Initialization value macros */
#define RESET_VALUE_INDEX                  (0U)

/* Hex value macros */
#define HEX_ERROR_DEVICE_DISCONNECTED      (0xff)
#define HEX_ERROR_TRANSFER_STATUS          (0xfe)
#define HEX_BACKLIGHT_COMPENSATION_VALUE   (0xFFFF)
#define HEX_MB_EVENT_VALUE                 (0xfe)

/* Numeric value macros */
#define NUM_LOOP_START                     (0)
#define NUM_LOOP_INCREMENT_1               (1)
#define NUM_STREAM_STOPPED                 (1)
#define NUM_THROW_AWAY_FRAME               (1)
#define NUM_BUF_READY_FLAG                 (1)
#define NUM_STRING_BUFFER_SIZE             (32)
#define NUM_FONT_INDEX_0                   (0)
#define NUM_FONT_OFFSET_1                  (1)

/* Additional numeric value macros */
#define NUM_TIMEOUT_MS_5000                (5000)
#define NUM_CAMERA_ENABLED                 (1)
#define NUM_CAMERA_DISABLED                (0)
#define NUM_DEVICE_DISCONNECTED            (0)
#define NUM_FOUND_FLAG                     (0)
#define NUM_GUID_INDEX_0                   (0)
#define NUM_FRAME_INTERVAL_INDEX           (0)
#define NUM_TERMINAL_UNIT_INDEX            (0)
#define NUM_LOOP_INCREMENT_1               (1)

/*******************************************************************************
* Data Structures and Types
*******************************************************************************/
typedef struct VideoBuffer
{
    uint32_t    NumBytes; /* Number of video data bytes in the buffer. */
    uint8_t     BufReady; /* Buffer ready flag */
} VideoBuffer_t;

/*******************************************************************************
* Global Variables
*******************************************************************************/
extern VideoBuffer_t        _ImageBuff[];
extern volatile uint8_t     lastBuffer;
extern bool Logitech_camera_enabled;
extern bool point3mp_camera_enabled;

#if defined(__cplusplus)
}
#endif /* __cplusplus */

#endif /* _USB_CAMERA_TASK_H_ */

/* [] END OF FILE */
