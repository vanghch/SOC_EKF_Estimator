#include "system.h"

const char version[] = "v1.0.0";
BatteryInfoStructure battery;

void Power_On_Off(bool mode, bool on_off)
{
    if (mode) {
        /* 软关机，对bq79640器件来说相当于依次对SYS_CTRL1寄存器的低两位写入00，01，10，目前暂时不实现 */
    } else {
        if (on_off) {
            /* 根据芯片使用手册，bq769xx可以在TS1引脚为高时被唤醒 TMin=10us TMax=2000us */
            HAL_GPIO_WritePin(POWER_ON_GPIO_Port, POWER_ON_Pin, GPIO_PIN_SET);
            osDelay(1000);
            HAL_GPIO_WritePin(WAKE_GPIO_Port, WAKE_Pin, GPIO_PIN_RESET);
        } else {
            HAL_GPIO_WritePin(POWER_ON_GPIO_Port, POWER_ON_Pin, GPIO_PIN_RESET);
            osDelay(10);
        }
    }
}

void System_Init(void)
{
    battery.property.capacity = 3.0;
    battery.property.serial = 13;
    battery.property.parallel = 1;
    battery.property.rate = 5.0;
    battery.property.tmp_cnt = 3;

    battery.config.threshold_UV = 2.45;
    battery.config.threshold_OV = 4.25;
    battery.config.threshold_SCD = (battery.property.rate * battery.property.parallel) * MULTI_SCD;
    battery.config.threshold_OCD = (battery.property.rate * battery.property.parallel) * MULTI_OCD;
    battery.config.threshold_TMax = 60.0;
    battery.config.threshold_TMin = 5.0;

    RTOS_INFO("\r\n\r\n\r\n***************\r\nBMS Initialized\r\nver=%s\r\n***************\r\n\r\n", version);
}
