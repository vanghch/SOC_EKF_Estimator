#ifndef __PUBLISH_H
#define __PUBLISH_H

#ifdef __cplusplus
extern "C" {
#endif

#include "usart.h"

#include "cmsis_os.h"

#include "system.h"
#include "msg.h"
#include "msg_handle.h"

#define BAT_STATUS_ID 0xF1
#define SOC_STATUS_ID 0xF2

void publish_update(void);

void send_float_data(uint8_t id, float *sdata, uint8_t n);

#ifdef __cplusplus
}
#endif

#endif
