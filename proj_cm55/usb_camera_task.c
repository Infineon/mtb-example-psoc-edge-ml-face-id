/******************************************************************************
* File Name        : usb_camera_task.c
*
* Description      : This file implements the USB Webcam functions.
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
#include "cy_pdl.h"
#include "FreeRTOS.h"
#include "task.h"
#include "event_groups.h"
#include "queue.h"
#include "USBH.h"
#include "USBH_Util.h"
#include "USBH_VIDEO.h"
#include <stdio.h>
#include "lcd_task.h"
#include "usb_camera_task.h"
#include "ifx_image_utils.h"
#include "ifx_time_utils.h"

/*******************************************************************************
* Macros
*******************************************************************************/
#define STR_HELPER(x)                            (#x)
#define STR(x)                                   (STR_HELPER(x))

#define ERROR_DEVICE_DISCONNECTED                HEX_ERROR_DEVICE_DISCONNECTED
#define ERROR_TRANSFER_STATUS                    HEX_ERROR_TRANSFER_STATUS

/*******************************************************************************
* Global Variables
*******************************************************************************/
TaskHandle_t usbh_main_task_handle;
TaskHandle_t usbh_isr_task_handle;

static QueueHandle_t _VIDEO_MailBox;
static QueueHandle_t _DeviceStateMailBox;
QueueHandle_t _NewFrameMailBox;

VideoBuffer_t    _ImageBuff[NUM_IMAGE_BUFFERS];

volatile uint8_t _device_connected = USB_DEVICE_DISCONNECTED;
static U8 _aVIDEO_DevIndexes[MAX_VIDEO_INTERFACES];
static char _ac[USB_STRING_BUFFER_SIZE];
static char _acMBEvent[USB_MB_EVENT_SIZE];

volatile uint8_t lastBuffer = RESET_VALUE_INDEX;
bool Logitech_camera_enabled = RESET_VALUE_INDEX;
bool point3mp_camera_enabled = RESET_VALUE_INDEX;
bool camera_not_supported = false;

/* Frame monitoring variables*/
float last_check_time = RESET_VALUE_INDEX;
static int _StreamErrCnt;
float frame_timeout_ms = USB_FRAME_TIMEOUT_MS; /* 10 seconds without frames is considered stalled */
static USBH_NOTIFICATION_HOOK   Hook;

/*******************************************************************************
* Functions
*******************************************************************************/

CY_SECTION_ITCM_BEGIN
/*******************************************************************************
* Function Name: _cbOnAddRemoveDevice
********************************************************************************
* Summary:
*  Callback function invoked when a USB device is added or removed. Runs in the
*  context of the USBH_Task and performs non-blocking operations to update 
*  device connection status, reset buffer states, and signal the LCD task via a 
*  queue as a mailbox.
*
* Parameters:
*  void *pContext: User context
*  U8 DevIndex: Index of the device
*  USBH_DEVICE_EVENT Event: Event type (add or remove)
*
* Return:
*  void
*
*******************************************************************************/
static void _cbOnAddRemoveDevice ( void *pContext, U8 DevIndex, USBH_DEVICE_EVENT Event )
{
    (void) pContext;

    switch ( Event )
    {
    case USBH_DEVICE_EVENT_ADD :
        _device_connected = USB_DEVICE_CONNECTED;
        USBH_Logf_Application("**** VIDEO Device added [%d]\n", DevIndex);

        /* **IMPROVED**: More thorough buffer reset on device connect */
        printf("[USB] Resetting all buffer states for new device\r\n");
        for (int i = NUM_LOOP_START; i < NUM_IMAGE_BUFFERS; i++) {
            _ImageBuff[i].BufReady = USB_BUFFER_CLEAR_VALUE;
            _ImageBuff[i].NumBytes = USB_BUFFER_CLEAR_VALUE;
        }

        /* Reset static variables in _cbOnData */
        lastBuffer = RESET_VALUE_INDEX;
        __DMB();

        /* Signal LCD task to reinitialize if needed */
        xQueueSend(_VIDEO_MailBox, &DevIndex, USB_MAILBOX_NO_WAIT); /* 0 = no wait */

        break;

    case USBH_DEVICE_EVENT_REMOVE :
        _device_connected = USB_DEVICE_DISCONNECTED;
        USBH_Logf_Application("**** VIDEO Device removed [%d]\n", DevIndex);

        /* **IMPROVED**: Clean shutdown */
        printf("[USB] Cleaning up buffer states for removed device\r\n");
        for (int i = NUM_LOOP_START; i < NUM_IMAGE_BUFFERS; i++) {
            _ImageBuff[i].BufReady = USB_BUFFER_CLEAR_VALUE;
            _ImageBuff[i].NumBytes = USB_BUFFER_CLEAR_VALUE;
        }
        __DMB();

        U8 disconnectEvent = ERROR_DEVICE_DISCONNECTED;
        xQueueSend(_DeviceStateMailBox, &disconnectEvent, USB_MAILBOX_NO_WAIT); /* 0 = no wait */

        _StreamErrCnt = RESET_VALUE_INDEX;
        break;

    default :
        ; /* Should never happen */
    }
}


/*******************************************************************************
* Function Name: monitor_buffer_health
********************************************************************************
* Summary:
*  Periodically monitors the health of image buffers by checking their readiness
*  states every 2 seconds to ensure proper buffer management.
*
* Parameters:
*  void
*
* Return:
*  void
*
*******************************************************************************/
static void monitor_buffer_health(void) {
    static float last_health_check = RESET_VALUE_INDEX;
    float current_time = ifx_time_get_ms_f();

    /* Check every 2 seconds */
    if (current_time - last_health_check > USB_HEALTH_CHECK_INTERVAL_MS) {

        last_health_check = current_time;
    }
}


/*******************************************************************************
* Function Name: _cbOnData
********************************************************************************
* Summary:
*  Callback function invoked when a USB video device buffer is ready. Handles 
*  data copying, error management, frame validation, and buffer state updates
*  with atomic operations. Manages end-of-frame processing and stream error 
*  recovery.
*
* Parameters:
*  USBH_VIDEO_DEVICE_HANDLE hDevice: Handle to the video device
*  USBH_VIDEO_STREAM_HANDLE hStream: Handle to the video stream
*  USBH_STATUS Status: Status of the data transfer
*  const U8 *pData: Pointer to the received data
*  unsigned NumBytes: Number of bytes received
*  U32 Flags: Flags indicating frame status (e.g., end of frame)
*  void *pUserDataContext: User context
*
* Return:
*  void
*
*******************************************************************************/
static void _cbOnData(
        USBH_VIDEO_DEVICE_HANDLE hDevice,
        USBH_VIDEO_STREAM_HANDLE hStream,
        USBH_STATUS Status,
        const U8    *pData,
        unsigned    NumBytes,
        U32         Flags,
        void        *pUserDataContext)
{
    (void) pData;
    (void) hDevice;
    (void) pUserDataContext;

    static size_t _frame_bytes = RESET_VALUE_INDEX;
    static uint8_t _current_Buffer = RESET_VALUE_INDEX;
    static uint8_t _throw_away_frame = RESET_VALUE_INDEX;
    USBH_STATUS Status1;
    bool endOfFrame;
    I8 IsStreamStopped;

    if (Status != USBH_STATUS_SUCCESS) {

        _StreamErrCnt++;
        if (_StreamErrCnt >= USB_STREAM_ERROR_THRESHOLD) {
            Status1 = USBH_STATUS_DEVICE_ERROR; /* Indicate failure. */
        } else {
            if ((Status != USBH_STATUS_DEVICE_REMOVED)) {
                Status1 = USBH_VIDEO_GetStreamState(hStream, &IsStreamStopped);
                if (Status1 == USBH_STATUS_SUCCESS) {
                    if (IsStreamStopped == NUM_STREAM_STOPPED) {
                        Status1 = USBH_VIDEO_RestartStream(hStream);
                        if (Status1!= USBH_STATUS_SUCCESS) {
                            USBH_Logf_Application("_cbOnData: USBH_VIDEO_RestartStream failed %s", USBH_GetStatusStr(Status));
                        }
                    } else {
                        USBH_Warnf_Application("_cbOnData: _StreamErrCnt %d, but stream is running", _StreamErrCnt);
                    }
                } else {
                    USBH_Logf_Application("_cbOnData: USBH_VIDEO_GetStreamState failed %s", USBH_GetStatusStr(Status));
                }
            }
        }

        /*
            If the consecutive error count reaches max,
            the device was disconnected,
            or the stream could not be restarted - remove the video device.
         */
        if ((Status == USBH_STATUS_DEVICE_REMOVED) ||
                (Status1  != USBH_STATUS_SUCCESS)) {
            USBH_Logf_Application("_cbOnData device removed or max conseq. errors exceeded");
            U8 disconnect_event = ERROR_DEVICE_DISCONNECTED;
            xQueueSend(_DeviceStateMailBox, &disconnect_event, USB_MAILBOX_NO_WAIT); /* 0 = no wait */

            _StreamErrCnt = RESET_VALUE_INDEX;
        }

        /* Reset frame state on error but don't change buffer states */
        _frame_bytes = RESET_VALUE_INDEX;
        _throw_away_frame = RESET_VALUE_INDEX;
        USBH_VIDEO_Ack(hStream);
    }

    _StreamErrCnt = RESET_VALUE_INDEX;
    endOfFrame = (Flags & USBH_UVC_END_OF_FRAME) == USBH_UVC_END_OF_FRAME;

    /* Validate buffer index */
    if (_current_Buffer >= NUM_IMAGE_BUFFERS) {

        _current_Buffer = RESET_VALUE_INDEX;
        _frame_bytes = RESET_VALUE_INDEX;
        _throw_away_frame = NUM_THROW_AWAY_FRAME;
        USBH_VIDEO_Ack(hStream);
        return;
    }

    /* Check if we should throw away this frame */
    if (_throw_away_frame) {
        _frame_bytes = RESET_VALUE_INDEX;
        if (endOfFrame) {
            _throw_away_frame = RESET_VALUE_INDEX;
        }
        USBH_VIDEO_Ack(hStream);
        return;
    }

    /* Copy data with bounds checking */
    if (_frame_bytes + NumBytes <= CAMERA_BUFFER_SIZE) {
        memcpy(((uint8_t *)usb_yuv_frames[_current_Buffer].memory) + _frame_bytes,
                pData, NumBytes);
        _frame_bytes += NumBytes;
    } else {

#if 0
        leftover_bytes = (_frame_bytes + NumBytes) - CAMERA_BUFFER_SIZE;
        memcpy(((uint8_t *)usb_yuv_frames[_current_Buffer].memory) + _frame_bytes,
                pData, CAMERA_BUFFER_SIZE);
        _frame_bytes = CAMERA_BUFFER_SIZE + leftover_bytes;
#endif

        _throw_away_frame = NUM_THROW_AWAY_FRAME;
        _frame_bytes = RESET_VALUE_INDEX;
        USBH_VIDEO_Ack(hStream);
        return;
    }

    /* Handle end of frame */
    if (endOfFrame) {
        if (_frame_bytes == CAMERA_BUFFER_SIZE) {

            /* **KEY CHANGE**: More robust buffer management
             First, mark current buffer as ready ATOMICALLY */
            _ImageBuff[_current_Buffer].NumBytes = _frame_bytes;
            __DMB();
            _ImageBuff[_current_Buffer].BufReady = NUM_BUF_READY_FLAG;
            __DMB();

            monitor_buffer_health();

            /* **KEY CHANGE**: Update lastBuffer AFTER marking ready */
            lastBuffer = _current_Buffer;
            __DMB();

            /* **KEY CHANGE**: Improved buffer switching with timeout */
            _current_Buffer = (_current_Buffer + NUM_LOOP_INCREMENT_1) % NUM_IMAGE_BUFFERS;
            _ImageBuff[_current_Buffer].BufReady = USB_BUFFER_CLEAR_VALUE;
            _ImageBuff[_current_Buffer].NumBytes = USB_BUFFER_CLEAR_VALUE;
            _throw_away_frame = _ImageBuff[_current_Buffer].BufReady;

        }
        _frame_bytes = RESET_VALUE_INDEX;
    }

    USBH_VIDEO_Ack(hStream);
}


CY_SECTION_ITCM_END
/*******************************************************************************
* Function Name: _GetFrameIntervals
********************************************************************************
* Summary:
*  Formats a string representation of frame intervals from a USB video frame
*  descriptor for logging purposes.
*
* Parameters:
*  USBH_VIDEO_FRAME_INFO *pFrame: Pointer to the frame information structure
*
* Return:
*  void
*
*******************************************************************************/
static void _GetFrameIntervals ( USBH_VIDEO_FRAME_INFO *pFrame )
{
    char ac[NUM_STRING_BUFFER_SIZE];
    unsigned i;

    USBH_MEMSET(_ac, 0, sizeof(_ac));
    USBH_MEMSET(ac, 0, sizeof(ac));
    for ( i = NUM_LOOP_START; i < pFrame->bFrameIntervalType; i++ )
    {
        if ( i == NUM_LOOP_START )
        {
            SEGGER_snprintf(ac, sizeof(ac), "%lu", pFrame->u.dwFrameInterval[i]);
        }
        else
        {
            SEGGER_snprintf(ac, sizeof(ac), ", %lu", pFrame->u.dwFrameInterval[i]);
        }
        strncat(_ac, ac, sizeof(_ac) - (strlen(_ac) + 1));
    }
}


/*******************************************************************************
* Function Name: _TermType2Str
********************************************************************************
* Summary:
*  Converts a USB video terminal type code to a human-readable string.
*
* Parameters:
*  U8 Type: Terminal type code
*
* Return:
*  const char*: String representation of the terminal type
*
*******************************************************************************/
static const char* _TermType2Str ( U8 Type )
{
    switch ( Type )
    {
    case USBH_VIDEO_VC_INPUT_TERMINAL :
        return "Input";
    case USBH_VIDEO_VC_OUTPUT_TERMINAL :
        return "Output";
    case USBH_VIDEO_VC_SELECTOR_UNIT :
        return "Selector";
    case USBH_VIDEO_VC_PROCESSING_UNIT :
        return "Processing";
    case USBH_VIDEO_VC_EXTENSION_UNIT :
        return "Extension";
    default :
        return "Unknown";
    }
}


/*******************************************************************************
* Function Name: _FormatType2Str
********************************************************************************
* Summary:
*  Converts a USB video format type code to a human-readable string.
*
* Parameters:
*  U8 Type: Format type code
*
* Return:
*  const char*: String representation of the format type
*
*******************************************************************************/
static const char* _FormatType2Str ( U8 Type )
{
    switch ( Type )
    {
    case USBH_VIDEO_VS_FORMAT_UNCOMPRESSED :
        return "Uncompressed";
    case USBH_VIDEO_VS_FORMAT_MJPEG :
        return "MJPEG";
    case USBH_VIDEO_VS_FORMAT_FRAME_BASED :
        return "H.264";
    default :
        return "Unknown";
    }
}


/*******************************************************************************
* Function Name: _FrameType2Str
********************************************************************************
* Summary:
*  Converts a USB video frame type code to a human-readable string.
*
* Parameters:
*  U8 Type: Frame type code
*
* Return:
*  const char*: String representation of the frame type
*
*******************************************************************************/
static const char* _FrameType2Str ( U8 Type )
{
    switch ( Type )
    {
    case USBH_VIDEO_VS_FRAME_UNCOMPRESSED :
        return "Uncompressed";
    case USBH_VIDEO_VS_FRAME_MJPEG :
        return "MJPEG";
    case USBH_VIDEO_VS_FRAME_FRAME_BASED :
        return "H.264";
    default :
        return "Unknown";
    }
}


/*******************************************************************************
* Function Name: _PrintInputTermInfo
********************************************************************************
* Summary:
*  Logs information about a USB video input terminal, including camera-specific
*  details like focal lengths and control size, if applicable.
*
* Parameters:
*  USBH_VIDEO_TERM_UNIT_INFO *pTermInfo: Pointer to the terminal information 
*  structure
*
* Return:
*  void
*
*******************************************************************************/
static void _PrintInputTermInfo ( USBH_VIDEO_TERM_UNIT_INFO *pTermInfo )
{
    if ( pTermInfo->u.Terminal.TerminalType != USBH_VIDEO_ITT_CAMERA )
    {
        USBH_Logf_Application("  Input terminal type 0x%x is not handled...", pTermInfo->u.Terminal.TerminalType);
    }
    else
    {
        USBH_Logf_Application("  Associated terminal ID %d", pTermInfo->u.Terminal.u.CameraTerm.bAssocTerminal);
        USBH_Logf_Application(
                "  Objective focal length min %d",
                pTermInfo->u.Terminal.u.CameraTerm.wObjectiveFocalLengthMin);
        USBH_Logf_Application(
                "  Objective focal length max %d",
                pTermInfo->u.Terminal.u.CameraTerm.wObjectiveFocalLengthMax);
        USBH_Logf_Application("  Ocular focal length %d", pTermInfo->u.Terminal.u.CameraTerm.wOcularFocalLength);
        USBH_Logf_Application("  Control Size %d", pTermInfo->u.Terminal.u.CameraTerm.bControlSize);
    }
}


/*******************************************************************************
* Function Name: _PrintOutputTermInfo
********************************************************************************
* Summary:
*  Logs information about a USB video output terminal, including associated
*  terminal ID and source ID, if applicable.
*
* Parameters:
*  USBH_VIDEO_TERM_UNIT_INFO *pTermInfo: Pointer to the terminal information 
*  structure
*
* Return:
*  void
*
*******************************************************************************/
static void _PrintOutputTermInfo ( USBH_VIDEO_TERM_UNIT_INFO *pTermInfo )
{
    if ( pTermInfo->u.Terminal.TerminalType != USBH_VIDEO_TT_STREAMING )
    {
        USBH_Logf_Application("  Ouput terminal type 0x%x is not handled...", pTermInfo->u.Terminal.TerminalType);
    }
    else
    {
        USBH_Logf_Application("  Associated terminal ID %d", pTermInfo->u.Terminal.u.OutputTerm.bAssocTerminal);
        USBH_Logf_Application("  Source ID %d", pTermInfo->u.Terminal.u.OutputTerm.bSourceID);
    }
}


/*******************************************************************************
* Function Name: _PrintSelectorUnitInfo
********************************************************************************
* Summary:
*  Logs information about a USB video selector unit, including unit ID and input
*  pin details.
*
* Parameters:
*  USBH_VIDEO_TERM_UNIT_INFO *pTermInfo: Pointer to the terminal information 
*  structure
*
* Return:
*  void
*
*******************************************************************************/
static void _PrintSelectorUnitInfo ( USBH_VIDEO_TERM_UNIT_INFO *pTermInfo )
{
    unsigned i;

    USBH_Logf_Application("  Unit ID %d", pTermInfo->bTermUnitID);
    USBH_Logf_Application("  Number of pins %d", pTermInfo->u.SelectUnit.bNrInPins);
    for ( i = NUM_LOOP_START; i < pTermInfo->u.SelectUnit.bNrInPins; i++ )
    {
        USBH_Logf_Application("  Input pin: %d", pTermInfo->u.SelectUnit.baSourceID[i]);
    }
}


/*******************************************************************************
* Function Name: _PrintProcessingUnitInfo
********************************************************************************
* Summary:
*  Logs information about a USB video processing unit, including unit ID, source
*  ID, max multiplier, and control size.
*
* Parameters:
*  USBH_VIDEO_TERM_UNIT_INFO *pTermInfo: Pointer to the terminal information 
*  structure
*
* Return:
*  void
*
*******************************************************************************/
static void _PrintProcessingUnitInfo ( USBH_VIDEO_TERM_UNIT_INFO *pTermInfo )
{
    USBH_Logf_Application("  Unit ID %d", pTermInfo->bTermUnitID);
    USBH_Logf_Application("  Source ID %d", pTermInfo->u.ProcessUnit.bSourceID);
    USBH_Logf_Application("  Max multiplier %d", pTermInfo->u.ProcessUnit.wMaxMultiplier);
    USBH_Logf_Application("  Control size %d", pTermInfo->u.ProcessUnit.bControlSize);
}


/*******************************************************************************
* Function Name: _PrintExtensionUnitInfo
********************************************************************************
* Summary:
*  Logs information about a USB video extension unit, including unit ID, GUID,
 *  number of pins, and control size.
*
* Parameters:
*  USBH_VIDEO_TERM_UNIT_INFO *pTermInfo: Pointer to the terminal information 
*  structure
*
* Return:
*  void
*
*******************************************************************************/
static void _PrintExtensionUnitInfo ( USBH_VIDEO_TERM_UNIT_INFO *pTermInfo )
{
    USBH_Logf_Application("  Unit ID %d", pTermInfo->bTermUnitID);
    USBH_Logf_Application("  Unit GUID 0x%x...", pTermInfo->u.ExtUnit.guidExtensionCode[NUM_GUID_INDEX_0]);
    USBH_Logf_Application("  Number of pins %d", pTermInfo->u.ExtUnit.bNrInPins);
    USBH_Logf_Application("  Controls size %d", pTermInfo->u.ExtUnit.bControlSize);
}


/*******************************************************************************
* Function Name: _PrintOutputTermInfo
********************************************************************************
* Summary:
*  Logs information about a USB video terminal or unit by dispatching to specific
*  print functions based on the terminal/unit type.
*
* Parameters:
*  USBH_VIDEO_TERM_UNIT_INFO *pTermInfo: Pointer to the terminal information 
*  structure
*
* Return:
*  void
*
*******************************************************************************/
static void _PrintTermUnitInfo ( USBH_VIDEO_TERM_UNIT_INFO *pTermInfo )
{
    USBH_Logf_Application("  Terminal/Unit ID %d type %s", pTermInfo->bTermUnitID, _TermType2Str(pTermInfo->Type));
    switch ( pTermInfo->Type )
    {
    case USBH_VIDEO_VC_INPUT_TERMINAL :
        _PrintInputTermInfo(pTermInfo);
        break;
    case USBH_VIDEO_VC_OUTPUT_TERMINAL :
        _PrintOutputTermInfo(pTermInfo);
        break;
    case USBH_VIDEO_VC_SELECTOR_UNIT :
        _PrintSelectorUnitInfo(pTermInfo);
        break;
    case USBH_VIDEO_VC_PROCESSING_UNIT :
        _PrintProcessingUnitInfo(pTermInfo);
        break;
    case USBH_VIDEO_VC_EXTENSION_UNIT :
        _PrintExtensionUnitInfo(pTermInfo);
        break;
    default :
        USBH_Logf_Application("  Type %d is not handled...", pTermInfo->Type);
        break;
    }
    USBH_Logf_Application("---------------");
}


/*******************************************************************************
* Function Name: _OnDevReady
********************************************************************************
* Summary:
*  Handles the initialization and configuration of a USB video device when it is
*  ready. Retrieves interface, terminal, and format information, configures the
*  video stream with the desired format and frame settings, and monitors stream
*  status with timeout handling.
*
* Parameters:
*  U8 DevIndex: Index of the USB device
*
* Return:
*  void
*
*******************************************************************************/
static void _OnDevReady ( U8 DevIndex )
{
    USBH_VIDEO_INPUT_HEADER_INFO    InputHeaderInfo;
    USBH_VIDEO_DEVICE_HANDLE    hDevice;
    USBH_VIDEO_INTERFACE_INFO   IfaceInfo;
    USBH_VIDEO_TERM_UNIT_INFO   TermInfo;
    USBH_VIDEO_STREAM_CONFIG    StreamInfo;
    USBH_VIDEO_COLOR_INFO       ColorInfo;
    USBH_VIDEO_STREAM_HANDLE    Stream;
    USBH_VIDEO_FORMAT_INFO      Format;
    USBH_VIDEO_FRAME_INFO       Frame;
    USBH_STATUS                 Status;
    unsigned    RequestedFrameIntervalIdx;
    unsigned    RequestedFormatIdx;
    unsigned    RequestedFrameIdx;
    unsigned    NumFrameDescriptors;
    unsigned    Found;
    unsigned    i;
    unsigned    j;
    unsigned    k;
    U8          MBEvent;
    U32         FrameIntervalfrmvidpid;
    /*
      Open the device, the device index is retrieved from the notification callback.
     */
    RequestedFormatIdx = RESET_VALUE_INDEX;
    RequestedFrameIdx = RESET_VALUE_INDEX;
    RequestedFrameIntervalIdx = NUM_FRAME_INTERVAL_INDEX;
    Found = NUM_FOUND_FLAG;

    memset( &hDevice, 0, sizeof(hDevice) );
    memset( &Stream, 0, sizeof(Stream) );
    /* Add timeout tracking for device initialization */
    float timeout_start_time = ifx_time_get_ms_f();
    float timeout_ms = NUM_TIMEOUT_MS_5000; /* 5 seconds timeout */

    Status = USBH_VIDEO_Open( DevIndex, &hDevice );
    if ( Status != USBH_STATUS_SUCCESS ) {
        USBH_Logf_Application("USBH_VIDEO_Open returned with error: %s", USBH_GetStatusStr(Status));
        return;
    }

    Status = USBH_VIDEO_GetInterfaceInfo( hDevice, &IfaceInfo );
    if ( Status != USBH_STATUS_SUCCESS ) {
        USBH_Logf_Application("USBH_VIDEO_GetInterfaceInfo returned with error: %s", USBH_GetStatusStr(Status));
        return;
    }
    USBH_Logf_Application("====================================================================");
    USBH_Logf_Application("Vendor  Id = 0x%0.4X", IfaceInfo.VendorId);
    USBH_Logf_Application("Product Id = 0x%0.4X", IfaceInfo.ProductId);

    if ((LOGI_TECH_C920_VID == IfaceInfo.VendorId) && (LOGI_TECH_C920_PID == IfaceInfo.ProductId))
    {
        camera_not_supported = false;
        FrameIntervalfrmvidpid = FRAME_INTERVAL_1;
        Logitech_camera_enabled = NUM_CAMERA_ENABLED;
        point3mp_camera_enabled = NUM_CAMERA_DISABLED;
    }
    else if ((HBV_CAM_0P3_VID == IfaceInfo.VendorId) && (HBV_CAM_0P3_PID == IfaceInfo.ProductId))
    {
        camera_not_supported = false;
        FrameIntervalfrmvidpid = FRAME_INTERVAL_2;
        Logitech_camera_enabled = NUM_CAMERA_DISABLED;
        point3mp_camera_enabled = NUM_CAMERA_ENABLED;
    }
    else if ((HBV_CAM_2P0_VID == IfaceInfo.VendorId) && (HBV_CAM_2P0_PID == IfaceInfo.ProductId))
    {
        camera_not_supported = false;
        FrameIntervalfrmvidpid = FRAME_INTERVAL;
        Logitech_camera_enabled = NUM_CAMERA_DISABLED;
        point3mp_camera_enabled = NUM_CAMERA_DISABLED;
    }
    else
    {
        camera_not_supported = true;
        _device_connected = NUM_DEVICE_DISCONNECTED;
        return;
    }
    /*
     List all terminals/units
     */
     for ( i = NUM_LOOP_START; i < IfaceInfo.NumTermUnits; i++ ) {
         /* Check if we've been waiting too long for initialization */
         if ((ifx_time_get_ms_f() - timeout_start_time) > timeout_ms) {
             USBH_Logf_Application("Timeout waiting for device initialization during terminal enumeration. Resetting...");
             Found = NUM_FOUND_FLAG;
             /*
              Close the device
              */
             USBH_Logf_Application("Closing video device...");
             USBH_VIDEO_Close(hDevice);
             return;
         }

         Status = USBH_VIDEO_GetTermUnitInfo( hDevice, i, &TermInfo );
         if ( Status == USBH_STATUS_SUCCESS ) {
             _PrintTermUnitInfo(&TermInfo);

         } else {
             USBH_Logf_Application("USBH_VIDEO_GetTermUnitInfo returned with error: %s", USBH_GetStatusStr(Status));
         }
     }

     /*
     This sets the text information fields and the webcam frame window visible.
      */
     memset( &IfaceInfo, 0, sizeof(IfaceInfo) );
     if ( Status != USBH_STATUS_SUCCESS && Status != USBH_STATUS_NOT_FOUND ) {
         USBH_Logf_Application("USBH_VIDEO_GetFirstTermUnitInfo returned with error: %s", USBH_GetStatusStr(Status));
     } else {
         /*
         Configure the VIDEO device.
          */
         Status = USBH_VIDEO_GetInputHeader( hDevice, &InputHeaderInfo );
         if ( Status != USBH_STATUS_SUCCESS ) {
             USBH_Logf_Application("USBH_VIDEO_GetInputHeader returned with error: %s", USBH_GetStatusStr(Status));
         } else {
             USBH_Logf_Application(
                     "Video Interface with index %d (%d formats reported, still capture method %d):",
                     DevIndex,
                     InputHeaderInfo.bNumFormats,
                     InputHeaderInfo.bStillCaptureMethod);
             /*
             Iterate over all formats
              */
             for ( i = 0; i < InputHeaderInfo.bNumFormats; i++ ) {
                 /* Check if we've been waiting too long for initialization */
                 if ((ifx_time_get_ms_f() - timeout_start_time) > timeout_ms) {
                     USBH_Logf_Application("Timeout waiting for device initialization during format discovery. Resetting...");
                     Found = NUM_FOUND_FLAG; /* Ensure we don't try to use this device */
                     break; /* Exit format discovery loop */
                 }

                 Status = USBH_VIDEO_GetFormatInfo(hDevice, i, &Format);
                 if ( Status != USBH_STATUS_SUCCESS ) {
                     USBH_Logf_Application(
                             "USBH_VIDEO_GetFormatInfo returned with error: %s",
                             USBH_GetStatusStr(Status));
                 } else {
                     switch ( Format.FormatType ) {
                     case USBH_VIDEO_VS_FORMAT_UNCOMPRESSED :
                         NumFrameDescriptors = Format.u.UncompressedFormat.bNumFrameDescriptors;
                         break;
                     case USBH_VIDEO_VS_FORMAT_MJPEG :
                         NumFrameDescriptors = Format.u.MJPEG_Format.bNumFrameDescriptors;
                         break;
                     case USBH_VIDEO_VS_FORMAT_FRAME_BASED :
                         NumFrameDescriptors = Format.u.H264_Format.bNumFrameDescriptors;
                         break;
                     default :
                         USBH_Logf_Application("  Format type %d is not supported!:", Format.FormatType);
                         Status = USBH_STATUS_ERROR;
                     }
                     /*
                     If we have a valid format type - proceed with parsing.
                      */
                      if ( Status == USBH_STATUS_SUCCESS ) {
                          USBH_Logf_Application("  Format Index %d, type %s:", i, _FormatType2Str(Format.FormatType));
                          /*
                         Check if the format has a color matching descriptor
                           */
                          Status = USBH_VIDEO_GetColorMatchingInfo(hDevice, i, &ColorInfo);
                          if ( Status != USBH_STATUS_SUCCESS ) {
                              USBH_Logf_Application("  No color matching descriptor (%s)", USBH_GetStatusStr(Status));
                          } else {
                              USBH_Logf_Application(
                                      "  Color matching descriptor: bColorPrimaries 0x%x, bTransferCharacteristics 0x%x, bMatrixCoefficients 0x%x",
                                      ColorInfo.bColorPrimaries,
                                      ColorInfo.bTransferCharacteristics,
                                      ColorInfo.bMatrixCoefficients);
                          }
                          /*
                         Iterate over all frame types
                           */
                          for ( j = 0; j < NumFrameDescriptors; j++ ) {
                              /* Check timeout during frame discovery */
                              if ((ifx_time_get_ms_f() - timeout_start_time) > timeout_ms) {
                                  USBH_Logf_Application("Timeout waiting for device initialization during frame discovery. Resetting...");
                                  Found = NUM_FOUND_FLAG;
                                  break; /* Exit frame discovery loop */
                              }

                              Status = USBH_VIDEO_GetFrameInfo(hDevice, i, j, &Frame);
                              if ( Status != USBH_STATUS_SUCCESS ) {
                                  USBH_Logf_Application(
                                          "USBH_VIDEO_GetFrameInfo returned with error: %s",
                                          USBH_GetStatusStr(Status));
                              } else {
                                  if ( Frame.bFrameIntervalType == 0 ) {
                                      /*
                                     Frame interval type zero (continuous frame interval)
                                     is almost never used by any device.
                                     This sample does not explicitly handle it.
                                       */
                                      USBH_Logf_Application(
                                              "    Frame Index %d, type %s, x %d, y %d, %d min frame interval, %d max frame interval, %d interval step",
                                              j,
                                              _FrameType2Str(Frame.FrameType),
                                              Frame.wWidth,
                                              Frame.wHeight,
                                              Frame.bFrameIntervalType,
                                              Frame.u.dwMinFrameInterval,
                                              Frame.u.dwMaxFrameInterval,
                                              Frame.u.dwFrameIntervalStep);
                                  } else {
                                      _GetFrameIntervals(&Frame);
                                      USBH_Logf_Application(
                                              "    Frame Index %d, type %s, x %d, y %d, intervals (%d): { %s }",
                                              j,
                                              _FrameType2Str(Frame.FrameType),
                                              Frame.wWidth,
                                              Frame.wHeight,
                                              Frame.bFrameIntervalType,
                                              _ac);
                                      /*
                                      Try to find a setting which matches our defines
                                       */
                                      if ( Format.FormatType == FORMAT ) {
                                          if ( (Frame.wHeight == CAMERA_HEIGHT) && (Frame.wWidth == CAMERA_WIDTH) ) {
                                              for ( k = 0; k < Frame.bFrameIntervalType; k++ )
                                              {

                                                  if (Frame.u.dwFrameInterval[k] == FrameIntervalfrmvidpid)
                                                  {
                                                      if ( Found != 1u ) {
                                                          RequestedFormatIdx = i;
                                                          RequestedFrameIdx = j;
                                                          RequestedFrameIntervalIdx = k + 1;
                                                          Found = 1;
                                                          USBH_Logf_Application(
                                                                  "--->Found requested setting (%dx%d @ %d FPS): Format Index %d, Frame Index %d, Frame Interval Index %d",
                                                                  CAMERA_WIDTH,
                                                                  CAMERA_HEIGHT,
                                                                  10000000 / Frame.u.dwFrameInterval[k],
                                                                  RequestedFormatIdx,
                                                                  RequestedFrameIdx,
                                                                  RequestedFrameIntervalIdx);
                                                      }
                                                  }
                                              }
                                          }
                                      }
                                  }
                              }
                          }
                      }
                 }
             }

             /*If we didn't find an exact format match, try to use first available format */
             if (Found != 1u && InputHeaderInfo.bNumFormats > 0) {
                 USBH_Logf_Application("WARNING: Couldn't find exact requested format. Using first available format instead.");
                 /* Fall back to first available format if any exist */
                 RequestedFormatIdx = 0;
                 RequestedFrameIdx = 0;
                 RequestedFrameIntervalIdx = 1; /* First interval */
                 Found = 1;
             }
         }

         /*
          Receive frame data.
          */
         memset(&StreamInfo, 0, sizeof(USBH_VIDEO_STREAM_CONFIG));
         StreamInfo.Flags = 0;
         if ( Found == 1u ) {
             StreamInfo.FormatIdx       = RequestedFormatIdx;
             StreamInfo.FrameIdx        = RequestedFrameIdx;
             StreamInfo.FrameIntervalIdx = RequestedFrameIntervalIdx;
             StreamInfo.pfDataCallback   = _cbOnData;

             /* Initialize the frame monitoring variables */
             lastBuffer = 0;
             last_check_time = ifx_time_get_ms_f();

             xQueueReset(_DeviceStateMailBox);

             Status = USBH_VIDEO_OpenStream( hDevice, &StreamInfo, &Stream );
             if ( Status != USBH_STATUS_SUCCESS ) {
                 USBH_Logf_Application("USBH_VIDEO_OpenStream returned with error: %s", USBH_GetStatusStr(Status));
             } else {
                 /*
                  Wait until the device has been disconnected.
                  */
                 for ( ;; ) {
                     /* Check for stalled camera frames */
                     float current_time = ifx_time_get_ms_f();
                     if ((current_time - last_check_time) > 1000) {
                         /* Check once per second */
                         last_check_time = current_time;
                     }
                     /* checking if the queue operation was successful */
                     if (  xQueueReceive(_DeviceStateMailBox, &MBEvent, pdMS_TO_TICKS(100)) == pdTRUE ) {
                         if (MBEvent == ERROR_DEVICE_DISCONNECTED) {
                             USBH_Logf_Application("USBH_VIDEO_OpenStream breaking - Device was disconnected");
                             break;
                         } else if (MBEvent == ERROR_TRANSFER_STATUS) {
                             USBH_Logf_Application("USBH_VIDEO_OpenStream breaking - Transfer status error");
                             break;
                         } else {
                             USBH_Logf_Application("USBH_VIDEO_OpenStream received unknown error code: 0x%02x", MBEvent);

                             if(_device_connected)
                                 MBEvent = HEX_MB_EVENT_VALUE;
                             break;
                         }
                     }
                 }
             }
         } else {
             /*
              Display error text until webcam is removed
              */
             while ( _device_connected ) {
                 USBH_OS_Delay(50);
             }
         }
     }
     if (MBEvent == ERROR_TRANSFER_STATUS || MBEvent == ERROR_DEVICE_DISCONNECTED)
     {
         Found = 0;
         USBH_VIDEO_CloseStream(Stream);
         USBH_Logf_Application("MBEVent Closing video device...");
         USBH_VIDEO_Close(hDevice) ;
         if (ERROR_DEVICE_DISCONNECTED == MBEvent)
         {
             _device_connected =  0;
         }
     }
}


/*******************************************************************************
* Function Name: USBH_Task_Wrapper
********************************************************************************
* Summary:
*  Wrapper function for the USB host task, delegating to the USBH_Task function.
*
* Parameters:
*  void *arg: Task argument
*
* Return:
*  void
*
*******************************************************************************/
void USBH_Task_Wrapper(void *arg) {
    (void)arg;
    USBH_Task();
}


/*******************************************************************************
* Function Name: USBH_ISRTask_Wrapper
********************************************************************************
* Summary:
*  Wrapper function for the USB host interrupt service task, delegating to the
*  USBH_ISRTask function.
*
* Parameters:
*  void *arg: Task argument
*
* Return:
*  void
*
*******************************************************************************/
void USBH_ISRTask_Wrapper(void *arg) {
    (void)arg;
    USBH_ISRTask();
}


/*******************************************************************************
* Function Name: cm55_usb_webcam_task
********************************************************************************
* Summary:
*  Main task for handling USB webcam functionality on the CM55 processor. 
*  Initializes the USB host stack, creates USB host and interrupt tasks, sets 
*  up notification callbacks, and processes device connection events via a queue 
*  as mailbox.
*
* Parameters:
*  void *arg: Task argument
*
* Return:
*  void
*
*******************************************************************************/
void cm55_usb_webcam_task( void *arg )
{
    const uint32_t  task_stack_size_bytes = 8192;
    U8              DevIndex;

    BaseType_t xRes;
    ifx_time_start(); /* start the timer */

    USBH_Init();
    xRes = xTaskCreate(USBH_Task_Wrapper, "USBH_Task", task_stack_size_bytes, NULL, (uint8_t) TASK_PRIO_USBH_MAIN, &usbh_main_task_handle);
    if (xRes != pdPASS) {
        CY_ASSERT(0);
    }

    xRes = xTaskCreate(USBH_ISRTask_Wrapper, "USBH_isr", task_stack_size_bytes, NULL, (uint8_t) TASK_PRIO_USBH_ISR, &usbh_isr_task_handle);
    if (xRes != pdPASS) {
        CY_ASSERT(0);
    }

    _VIDEO_MailBox = xQueueCreate(SEGGER_COUNTOF(_aVIDEO_DevIndexes), sizeof(U8));
    configASSERT(_VIDEO_MailBox != NULL);

    _DeviceStateMailBox = xQueueCreate(sizeof(_acMBEvent)/sizeof(U8), sizeof(U8));
    configASSERT(_DeviceStateMailBox != NULL);

    USBH_VIDEO_Init();
    USBH_VIDEO_AddNotification( &Hook, _cbOnAddRemoveDevice, NULL );

    USBH_Logf_Application("Webcam Task Starting Loop ...");
    while ( true ) {
        /* Process device connection events */
        if (pdTRUE == xQueueReceive(_VIDEO_MailBox, &DevIndex, pdMS_TO_TICKS(100)))
        {
            _OnDevReady(DevIndex);
        }
    }
}

/* [] END OF FILE */
