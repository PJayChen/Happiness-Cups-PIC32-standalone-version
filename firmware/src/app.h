/*******************************************************************************
  MPLAB Harmony Application Header File

  Company:
    Microchip Technology Inc.

  File Name:
    app.h

  Summary:
    This header file provides prototypes and definitions for the application.

  Description:
    This header file provides function prototypes and data type definitions for
    the application.  Some of these are required by the system (such as the
    "APP_Initialize" and "APP_Tasks" prototypes) and some of them are only used
    internally by the application (such as the "APP_STATES" definition).  Both
    are defined here for convenience.
*******************************************************************************/

//DOM-IGNORE-BEGIN
/*******************************************************************************
Copyright (c) 2013-2014 released Microchip Technology Inc.  All rights reserved.

Microchip licenses to you the right to use, modify, copy and distribute
Software only when embedded on a Microchip microcontroller or digital signal
controller that is integrated into your product or third party product
(pursuant to the sublicense terms in the accompanying license agreement).

You should refer to the license agreement accompanying this Software for
additional information regarding your rights and obligations.

SOFTWARE AND DOCUMENTATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY KIND,
EITHER EXPRESS OR IMPLIED, INCLUDING WITHOUT LIMITATION, ANY WARRANTY OF
MERCHANTABILITY, TITLE, NON-INFRINGEMENT AND FITNESS FOR A PARTICULAR PURPOSE.
IN NO EVENT SHALL MICROCHIP OR ITS LICENSORS BE LIABLE OR OBLIGATED UNDER
CONTRACT, NEGLIGENCE, STRICT LIABILITY, CONTRIBUTION, BREACH OF WARRANTY, OR
OTHER LEGAL EQUITABLE THEORY ANY DIRECT OR INDIRECT DAMAGES OR EXPENSES
INCLUDING BUT NOT LIMITED TO ANY INCIDENTAL, SPECIAL, INDIRECT, PUNITIVE OR
CONSEQUENTIAL DAMAGES, LOST PROFITS OR LOST DATA, COST OF PROCUREMENT OF
SUBSTITUTE GOODS, TECHNOLOGY, SERVICES, OR ANY CLAIMS BY THIRD PARTIES
(INCLUDING BUT NOT LIMITED TO ANY DEFENSE THEREOF), OR OTHER SIMILAR COSTS.
 *******************************************************************************/
//DOM-IGNORE-END

#ifndef _APP_H
#define _APP_H


// *****************************************************************************
// *****************************************************************************
// Section: Included Files
// *****************************************************************************
// *****************************************************************************

#include <stdint.h>
#include <assert.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>

/* PIC32 related includes */
#include <p32xxxx.h>
#include <xc.h>

#include "system/system.h"
#include "system_config.h"
#include "system_definitions.h"
#include "debug/sys_debug.h"
#include "peripheral/oc/plib_oc.h"
#include "peripheral/i2c/plib_i2c.h"
#include "peripheral/nvm/plib_nvm.h"
#include "bsp_config.h"

/* BT stack related Includes */
#include "cdbt/bt/bt_std.h"
#include "cdbt/bt/bt_hcitr.h"
#include "cdbt/bt/bt_timer.h"
#include "cdbt/hci/hci.h"
#include "cdbt/hci/hcitr_uart.h"
#include "cdbt/extra/csr/csr.h"
#include "cdbt/bt/bt_system.h"
#include "cdbt/spp/spp.h"
#include "cdbt/hci/baseband.h"
#include "cdbt/bt/bt_signal.h"
#include "cdbt/bt/bt_oem.h"

#include "btapp_spp_sdpdb.h"
#include "btapp_device.h"


/* BT Configuration related Includes */
#include "bttimer_config.h"
#include "btcontroller.h"
#include "btconfig.h"

/* BT Task related Includes */
#include "btport.h"
#include "bttask.h"
#include "btstorage.h"
#include "bttimer.h"

/* Temperature Sensor functionality related Includes */
#include "accelerometer.h"
#include "accelerometer_bma250_config.h"

/* RGB LED functionality related Includes */
#include "cree_led.h"

/* BT Stack user specific configuration */
#include "user_config.h"

/* Application specific NVM functions */
#include "app_nvm.h"

/* FreeRTOS Library */
#include "queue.h"

// *****************************************************************************
// *****************************************************************************
// Section: FreeRTOS
// *****************************************************************************
// *****************************************************************************
QueueHandle_t xAccelQueue;
QueueHandle_t xAccelRawQueue;
QueueHandle_t xDataReceivedFromBTQueue;

// *****************************************************************************
// *****************************************************************************
// Section: LED Control parameter
// *****************************************************************************
// *****************************************************************************
#define SHAKE_V 0
#define SHAKE_H 1
#define DRINKING 2
#define SWAYING 3
#define TOASTING 4

// *****************************************************************************
// *****************************************************************************
// Section: Constants
// *****************************************************************************
// *****************************************************************************

/* Accelerometer BMA250E typical sensitivity
 * shift right 2 bit because we used 8-bit variable to contain 10-bit accel value.
 * so ignore 2 LSB bits
 */

#define SENSITIVITY_2G 0.015625   // 1/(256 >> 2)   <g/LSB>
#define SENSITIVITY_4G 0.03125    // 1/(128 >> 2)
#define SENSITIVITY_8G 0.0625     // 1/(64 >> 2)
#define SENSITIVITY_16G 0.125     // 1/(32 >> 2)

#define RADIANS_TO_DEGREES  57.29582791f // 180/3.14159f

#define APP_BT_PAIRING_STORAGE_SUPPORTED		false

/* BT module reset duration */
#define APP_BT_MODULE_RESET_DURATION_MS                  10
// *****************************************************************************
// *****************************************************************************
// Section: Type Definitions
// *****************************************************************************
// *****************************************************************************


typedef struct _ACCEL_XYZ
{
    ACCEL_DATA acc_x;
    ACCEL_DATA acc_y;
    ACCEL_DATA acc_z;
}ACCEL_XYZ_RAW;

typedef struct _ACCEL_XYZf
{
    float acc_x;
    float acc_y;
    float acc_z;
}ACCEL_XYZf;

// *****************************************************************************
/* Application states

  Summary:
    Application states enumeration

  Description:
    This enumeration defines the valid application states.  These states
    determine the behavior of the application at various times.
*/

typedef enum
{
    APP_STATE_BT_REGISTER_TICK_HANDLER,
    APP_STATE_BT_RESET_MODULE_START,
    APP_STATE_BT_RESET_MODULE_PROCESSING,
    APP_STATE_BT_SETUP_USART_DRIVER,
    APP_STATE_BT_TASK_RUN,
    APP_STATE_ERROR
} APP_STATES;


// *****************************************************************************
/* Application USART client for BT

  Summary:
    Application USART client for BT.

  Description:
    This object holds the BT USART's client handle, read and write buffer handle
    created and the context
*/
typedef struct
{
    DRV_HANDLE handle;
    DRV_USART_BUFFER_HANDLE readBufHandle;
    DRV_USART_BUFFER_HANDLE writeBufHandle;
    uintptr_t context;
} APP_USART_CLIENT;


// *****************************************************************************
/* Application Data

  Summary:
    Holds application data

  Description:
    This structure holds the application's data.

  Remarks:
    Application strings and buffers are be defined outside this structure.
 */

typedef struct
{
    /* Application's current state*/
    APP_STATES state;

    /* USART client handle */
    APP_USART_CLIENT usartClient;

    /* delay ms handle */
    SYS_TMR_HANDLE delayMSHandle;

    /* BT tick timer handle */
    SYS_TMR_HANDLE tickSysTmrHandle;
} APP_DATA;


// *****************************************************************************
// *****************************************************************************
// Section: Application Callback Routines
// *****************************************************************************
// *****************************************************************************
/* These routines are called by drivers when certain events occur.
*/

	
// *****************************************************************************
// *****************************************************************************
// Section: Application Initialization and State Machine Functions
// *****************************************************************************
// *****************************************************************************

/*******************************************************************************
  Function:
    void APP_Initialize ( void )

  Summary:
     MPLAB Harmony application initialization routine.

  Description:
    This function initializes the Harmony application.  It places the 
    application in its initial state and prepares it to run so that its 
    APP_Tasks function can be called.

  Precondition:
    All other system initialization routines should be called before calling
    this routine (in "SYS_Initialize").

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    APP_Initialize();
    </code>

  Remarks:
    This routine must be called from the SYS_Initialize function.
*/

void APP_Initialize ( void );


/*******************************************************************************
 * 
 */

void ACCEL_Task ( void );

void Bluetooth_Tasks ( void );

void LEDcontrol_Tasks ( void );

/*******************************************************************************
  Function:
    void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
        DRV_USART_BUFFER_HANDLE handle, uintptr_t context )

  Summary:
    Event Handler for USART Task.

  Description:
    This is the Event Handler for USART Tx and Rx Complete Events.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
*/
void APP_USARTBufferEventHandler(DRV_USART_BUFFER_EVENT event,
        DRV_USART_BUFFER_HANDLE handle, uintptr_t context );

/*******************************************************************************
  Function:
    void APP_BTTimerTickCallbackHandler(uintptr_t context, uint32_t currTick )

  Summary:
    Event Handler for BT Tick Task.

  Description:
    This is the Event Handler for BT Tick 10ms handler.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    context         - A client parameter that's passed in the callback function.
                        This will help to identify the callback.
    currTick        - The current system tick when the notification is called.

  Returns:
    None.

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
*/
void APP_BTTimerTickCallbackHandler(uintptr_t context, uint32_t currTick );

/*******************************************************************************
  Function:
    uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void);

  Summary:
    Read System Core Timer Value.

  Description:
    Read System Core Timer Value.

  Precondition:
    The system and application initialization ("SYS_Initialize") should be
    called before calling this.

  Parameters:
    None

  Returns:
    uint32_t - Current core timer tick value

  Example:
    <code>
    </code>

  Remarks:
    This routine must be called from SYS_Tasks() routine.
*/
uint32_t __attribute__((nomips16)) APP_ReadCoreTimer(void);

// *****************************************************************************
// *****************************************************************************
// Section: extern declarations
// *****************************************************************************
// *****************************************************************************
extern APP_DATA appData;
extern void onBluetoothPortStarted(void);




#endif /* _APP_H */
/*******************************************************************************
 End of File
 */

