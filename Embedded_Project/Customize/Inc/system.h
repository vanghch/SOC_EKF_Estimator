#ifndef __SYSTEM_H
#define __SYSTEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include "stdio.h"
#include "stddef.h"
#include "stdbool.h"
#include "string.h"

#include "bq769xx.h"
#include "led.h"
#include "soc.h"
#include "publish.h"

#define POWER_MODE_HARDWARE false
#define POWER_MODE_SOFTWARE true
#define POWER_ON    true
#define POWER_OFF   false

#define MULTI_SCD   2
#define MULTI_OCD   1.2

typedef struct
{
    float capacity;         // 电芯容量（AH）
    uint8_t serial;         // 电池串数
    uint8_t parallel;       // 电池并数
    float rate;             // 放电倍率
    uint8_t tmp_cnt;
}PropertyInfoStructure;

typedef struct
{
    float threshold_UV;     // 电芯欠压阈值
    float threshold_OV;     // 电芯过压阈值
    float threshold_SCD;    // 电池短路电流阈值
    float threshold_OCD;    // 电池放电过流阈值
    float threshold_TMax;   // 电池最高工作温度
    float threshold_TMin;   // 电池最低工作温度
}ConfigInfoStructure;

typedef struct
{
    float vol_cell[15];     // 第x片电芯电压
    float vol_pack;         // 电池组电压
    float current;          // 电流
    float thermistor[3];    // 热敏电阻x温度
}CollectionInfoStructure;

typedef struct
{
    uint32_t timestamp;     // 以ms为单位计数
    PropertyInfoStructure property;
    ConfigInfoStructure config;
    CollectionInfoStructure collection;
}BatteryInfoStructure;

/**
 * @brief 对主器件BQ769xx开机/关机操作
 * 
 * @param mode true:由软件实现，实际器件仍然处于上电状态 false:由硬件实现，关闭后器件完全断电
 * @param on_off true:开启 false:关闭
 */
void Power_On_Off(bool mode, bool on_off);

void System_Init(void);

#ifdef __cplusplus
}
#endif

#endif  /* __SYSTEM_H */
