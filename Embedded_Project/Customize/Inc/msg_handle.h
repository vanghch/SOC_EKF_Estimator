#ifndef __MSG_HANDLE_H
#define __MSG_HANDLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

extern osMessageQueueId_t battery_status_bq769xx_to_socHandle;
extern osMessageQueueId_t battery_status_bq769xx_to_publishHandle;
extern osMessageQueueId_t soc_status_soc_to_publishHandle;

#ifdef __cplusplus
}
#endif

#endif
