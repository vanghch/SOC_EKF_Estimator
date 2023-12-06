#include "publish.h"

void publish_update(void)
{
    float bat_buf[5] = {0.0f};
    float soc_buf[7] = {0.0f};

    BatteryStatusStructure battery_status = {0};
    SocStatusStructure soc_status = {0};

    /* 获取BAT状态 */
    if (0 != osMessageQueueGetCount(battery_status_bq769xx_to_publishHandle)) {
        if (osOK == osMessageQueueGet(battery_status_bq769xx_to_publishHandle, &battery_status, 0, 10)) {
            bat_buf[0] = battery_status.timestamp * 1e-3;
            bat_buf[1] = battery_status.vol;
            bat_buf[2] = battery_status.cell_vol;
            bat_buf[3] = battery_status.current;
            bat_buf[4] = battery_status.temperature;

            send_float_data(BAT_STATUS_ID, bat_buf, 5);
        }
    }

    /* 获取SOC状态 */
    if (0 != osMessageQueueGetCount(soc_status_soc_to_publishHandle)) {
        if (osOK == osMessageQueueGet(soc_status_soc_to_publishHandle, &soc_status, 0, 10)) {
            soc_buf[0] = soc_status.timestamp * 1e-3;
            soc_buf[1] = soc_status.soc.Uoc_original;
            soc_buf[2] = soc_status.soc.SOC_AHIntegrate;
            soc_buf[3] = soc_status.soc.EKF_Estimate;
            soc_buf[4] = soc_status.terminal_voltage_detect;
            soc_buf[5] = soc_status.terminal_voltage_estimate;
            soc_buf[6] = soc_status.polarization_voltage_estimate;

            send_float_data(SOC_STATUS_ID, soc_buf, 7);
        }
    }

}

void send_float_data(uint8_t id, float *sdata, uint8_t n)
{
    const uint8_t len = sizeof(float) * n;
    uint8_t data[100];
    uint8_t sumcheck = 0;
    uint8_t addcheck = 0;

    data[0] = 0xAA;
    data[1] = 0xFF;
    data[2] = id;
    data[3] = len;
    
    for (int i=0; i<len; i++) {
        data[4+4*i] = (uint8_t)(((uint32_t)(sdata[i] * 1000.0f) >> 0) & 0xff);
        data[5+4*i] = (uint8_t)(((uint32_t)(sdata[i] * 1000.0f) >> 8) & 0xff);
        data[6+4*i] = (uint8_t)(((uint32_t)(sdata[i] * 1000.0f) >> 16) & 0xff);
        data[7+4*i] = (uint8_t)(((uint32_t)(sdata[i] * 1000.0f) >> 24) & 0xff);
    }

    for (int i=0; i<len+4; i++) {
        sumcheck += data[i];
        addcheck += sumcheck;
    }

    data[len+4] = sumcheck;
    data[len+5] = addcheck;

    HAL_UART_Transmit(&huart1, data, len+6, 1000);
    // HAL_UART_Transmit_DMA(&huart1, data, len+6);
}
