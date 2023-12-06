#ifndef __MSG_H
#define __MSG_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"

#define __LOG_DEBUG

#ifdef __LOG_DEBUG
#define RTOS_ERROR(fmt, args...) printf("[ERROR][%s %d] "fmt"\r\n", __FILE__, __LINE__, ##args)
#define RTOS_WARN(fmt, args...) printf("[WARN][%s %d] "fmt"\r\n", __FILE__, __LINE__, ##args)
#define RTOS_INFO(fmt, args...) printf("[INFO][%s %d] "fmt"\r\n", __FILE__, __LINE__, ##args)
#else
#define RTOS_ERROR(fmt, ...)
#define RTOS_WARN(fmt, ...)
#define RTOS_INFO(fmt, ...)
#endif

typedef struct __attribute__((__packed__))
{
    uint32_t timestamp;
    float vol;
    float cell_vol;
    float current;
    float temperature;
}BatteryStatusStructure;

typedef struct
{
    uint32_t timestamp;
    struct SOCInfoStructure
    {
        float Uoc_original;                 // 通过OCV计算得到的初始电量
        float SOC_AHIntegrate;              // 通过电流积分法计算得到的SOC
        float EKF_Estimate;                 // 通过EKF估计出的SOC
    }soc;                                   // soc
    float terminal_voltage_detect;          // 端电压测量值
    float terminal_voltage_estimate;        // 端电压估计值(通过EKF估计)
    float polarization_voltage_estimate;    // 极化电压估计值(通过EKF估计)
}SocStatusStructure;

#ifdef __cplusplus
}
#endif

#endif
