/* ************************************************************************** */
/** Descriptive File Name

  @Company
    Company Name

  @File Name
    filename.h

  @Summary
    Brief description of the file.

  @Description
    Describe the purpose of this file.
 */
/* ************************************************************************** */

#ifndef _RECOGNITION_TASK_H    /* Guard against multiple inclusion */
#define _RECOGNITION_TASK_H


/* ************************************************************************** */
/* ************************************************************************** */
/* Section: Included Files                                                    */
/* ************************************************************************** */
/* ************************************************************************** */
#include "app.h"


/* FreeRTOS Library */
#include "FreeRTOS.h"
#include "queue.h"


#define FRAME_SIZE 64
#define POWER_THRESHOLD 400
#define GRAVITY_ACC 64

/* Provide C++ Compatibility */
#ifdef __cplusplus
extern "C" {
#endif

typedef struct _ACCEL_ZM
{
    ACCEL_DATA acc_z;
    ACCEL_DATA acc_magnitude;
}ACCEL_ZM;    


enum { FRAMING, DETECT_MOTION, FIND_END_FRAME, IDENTIFY_MOTION };

// *****************************************************************************
// *****************************************************************************
// Section: FreeRTOS
// *****************************************************************************
// *****************************************************************************
QueueHandle_t xAccelZMQueue;
QueueHandle_t xActiveMotionIntervalQueue;



void Recognition_Tasks ( void );

    /* Provide C++ Compatibility */
#ifdef __cplusplus
}
#endif

#endif /* _EXAMPLE_FILE_NAME_H */

/* *****************************************************************************
 End of File
 */
