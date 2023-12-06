#include "bq769xx.h"

BQ796xxInfoStructure bq769xx;
BQ769xxRegisterInfoStructure bq769xx_reg;

extern BatteryInfoStructure battery;

void BQ769xx_Init(void)
{
    uint8_t ret;
    
    BQ769xx_init_parameters();
    ret = BQ769xx_read_gain_offset();
    ret = BQ769xx_write_config();

    if (ret != HAL_OK) {
        RTOS_ERROR("BQ769xx init failed: %d", ret);
    }
}

void BQ769xx_init_parameters(void)
{
    bq769xx_reg.SysCtrl1.SysCtrl1Byte = 0x00;
    bq769xx_reg.SysCtrl1.SysCtrl1Bit.TEMP_SEL = 1;      // 使能热敏电阻ADC采集
    bq769xx_reg.SysCtrl1.SysCtrl1Bit.ADC_EN = 1;        // 使能ADC电压电流转换 该位开启关闭与过压保护同步

    bq769xx_reg.SysCtrl2.SysCtrl2Byte = 0x00;
    bq769xx_reg.SysCtrl2.SysCtrl2Bit.DSG_ON = 1;        // 开启放电
    bq769xx_reg.SysCtrl2.SysCtrl2Bit.CHG_ON = 1;        // 开启充电
    bq769xx_reg.SysCtrl2.SysCtrl2Bit.CC_EN = 1;         // 开启电流计连续转换

    bq769xx_reg.Protect1.Protect1Byte = 0x00;
    bq769xx_reg.Protect1.Protect1Bit.SCD_THRESH = find_current_map(CUR_HRANGE_80A);
    bq769xx_reg.Protect1.Protect1Bit.RSNS = 1;          // 短路电流检测量程翻倍 原最大量程40A
    bq769xx_reg.Protect1.Protect1Bit.SCD_DELAY = 0x03;  // 短路检测延迟400us

    bq769xx_reg.Protect2.Protect2Byte = 0x00;
    bq769xx_reg.Protect2.Protect2Bit.OCD_THRESH = 0x0f; // 过流保护阈值40A
    bq769xx_reg.Protect2.Protect2Bit.OCD_DELAY = 0x07;  // 过流检测延迟1280ms

    bq769xx_reg.Protect3.Protect3Byte = 0x00;
    bq769xx_reg.Protect3.Protect3Bit.OV_DELAY = 0x01;   // 过压保护检测延迟2s
    bq769xx_reg.Protect3.Protect3Bit.UV_DELAY = 0x03;   // 欠压保护检测延迟16s

    bq769xx_reg.CCCfg = 0x19;
}

uint8_t BQ769xx_read_gain_offset(void)
{
    uint8_t ret;
    uint8_t gain1=0, gain2=0;
    uint32_t gain;
    uint8_t offset=0;

    ret = Bq769xx_i2c_read(ADCGAIN1_RegAddr, &gain1, 1);
    ret = Bq769xx_i2c_read(ADCGAIN2_RegAddr, &gain2, 1);
    gain = 365 + ((gain1 & 0x0c) << 1) + ((gain2 & 0xe0) >> 5);
    ret = Bq769xx_i2c_read(ADCOFFSET_RegAddr, &offset, 1);

    bq769xx.gain_uv = gain;
    bq769xx.gain_mv = gain / 1000;
    bq769xx.offset = offset;
    bq769xx.ov_threshold = limit_max_min(battery.config.threshold_OV * 1000, OV_THRESHOLD_MIN, OV_THRESHOLD_MAX);
    bq769xx.uv_threshold = limit_max_min(battery.config.threshold_UV * 1000, UV_THRESHOLD_MIN, UV_THRESHOLD_MAX);

    bq769xx_reg.OVTrip = (uint8_t) (((((unsigned long)(bq769xx.ov_threshold - bq769xx.offset)*1000) / bq769xx.gain_uv - OV_THRESHOLD_BASE) >> 4) & 0xff);
    bq769xx_reg.UVTrip = (uint8_t) (((((unsigned long)(bq769xx.uv_threshold - bq769xx.offset)*1000) / bq769xx.gain_uv - OV_THRESHOLD_BASE) >> 4) & 0xff);

    if (ret != HAL_OK) {
        RTOS_ERROR("BQ769xx read gain and offset failed: %d", ret);
    }

    return ret;
}

uint8_t BQ769xx_write_config(void)
{
    uint8_t ret;
    uint8_t bq769xx_config[8] = {0};
    uint8_t err = 0, count = 0;

    do {
        ret = Bq769xx_i2c_write(SYS_CTRL1_RegAddr, &(bq769xx_reg.SysCtrl1.SysCtrl1Byte), 1); osDelay(10);
        ret = Bq769xx_i2c_write(SYS_CTRL2_RegAddr, &(bq769xx_reg.SysCtrl2.SysCtrl2Byte), 1); osDelay(10);
        ret = Bq769xx_i2c_write(PROTECT1_RegAddr, &(bq769xx_reg.Protect1.Protect1Byte), 1); osDelay(10);
        ret = Bq769xx_i2c_write(PROTECT2_RegAddr, &(bq769xx_reg.Protect2.Protect2Byte), 1); osDelay(10);
        ret = Bq769xx_i2c_write(PROTECT3_RegAddr, &(bq769xx_reg.Protect3.Protect3Byte), 1); osDelay(10);
        ret = Bq769xx_i2c_write(OV_TRIP_RegAddr, &(bq769xx_reg.OVTrip), 1); osDelay(10);
        ret = Bq769xx_i2c_write(UV_TRIP_RegAddr, &(bq769xx_reg.UVTrip), 1); osDelay(10);
        ret = Bq769xx_i2c_write(CC_CONFIG_RegAddr, &(bq769xx_reg.CCCfg), 1); osDelay(10);

        osDelay(100);

        ret = Bq769xx_i2c_read(SYS_CTRL1_RegAddr, &bq769xx_config[0], 1); osDelay(10);
        ret = Bq769xx_i2c_read(SYS_CTRL2_RegAddr, &bq769xx_config[1], 1); osDelay(10);
        ret = Bq769xx_i2c_read(PROTECT1_RegAddr, &bq769xx_config[2], 1); osDelay(10);
        ret = Bq769xx_i2c_read(PROTECT2_RegAddr, &bq769xx_config[3], 1); osDelay(10);
        ret = Bq769xx_i2c_read(PROTECT3_RegAddr, &bq769xx_config[4], 1); osDelay(10);
        ret = Bq769xx_i2c_read(OV_TRIP_RegAddr, &bq769xx_config[5], 1); osDelay(10);
        ret = Bq769xx_i2c_read(UV_TRIP_RegAddr, &bq769xx_config[6], 1); osDelay(10);
        ret = Bq769xx_i2c_read(CC_CONFIG_RegAddr, &bq769xx_config[7], 1); osDelay(10);

        osDelay(100);

        if (ret != HAL_OK) {
            RTOS_WARN("re-check register config failed, read error:%d, %d, %d", ret, err, count);
        }

        if ((bq769xx_config[0] & 0x1f) != (bq769xx_reg.SysCtrl1.SysCtrl1Byte & 0x1f)
            || (bq769xx_config[1] & 0xe7) != (bq769xx_reg.SysCtrl2.SysCtrl2Byte & 0xe7)
            || (bq769xx_config[2] & 0x9f) != (bq769xx_reg.Protect1.Protect1Byte & 0x9f)
            || (bq769xx_config[3] & 0x7f) != (bq769xx_reg.Protect2.Protect2Byte & 0x7f)
            || (bq769xx_config[4] & 0xff) != (bq769xx_reg.Protect3.Protect3Byte & 0xff)
            || (bq769xx_config[5] & 0xff) != (bq769xx_reg.OVTrip & 0xff)
            || (bq769xx_config[6] & 0xff) != (bq769xx_reg.UVTrip & 0xff)
            || (bq769xx_config[7] & 0x3f) != (bq769xx_reg.CCCfg & 0x3f)) {
                err++;
        }
        count++;
    }while((err != 0) && (count < 5));

    if ((err == 0) || (count < 5)) {
        ret = HAL_OK;
    } else {
        RTOS_INFO("re-check register config failed, read error:%d, %d, %d", ret, err, count);
        RTOS_INFO("SYS_CTRL1_RegAddr set:%x, read:%x", bq769xx_reg.SysCtrl1.SysCtrl1Byte, bq769xx_config[0]);
        RTOS_INFO("SYS_CTRL2_RegAddr set:%x, read:%x", bq769xx_reg.SysCtrl2.SysCtrl2Byte, bq769xx_config[1]);
        RTOS_INFO("PROTECT1_RegAddr set:%x, read:%x", bq769xx_reg.Protect1.Protect1Byte, bq769xx_config[2]);
        RTOS_INFO("PROTECT2_RegAddr set:%x, read:%x", bq769xx_reg.Protect2.Protect2Byte, bq769xx_config[3]);
        RTOS_INFO("PROTECT3_RegAddr set:%x, read:%x", bq769xx_reg.Protect3.Protect3Byte, bq769xx_config[4]);
        RTOS_INFO("OV_TRIP_RegAddr set:%x, read:%x", bq769xx_reg.OVTrip, bq769xx_config[5]);
        RTOS_INFO("UV_TRIP_RegAddr set:%x, read:%x", bq769xx_reg.UVTrip, bq769xx_config[6]);
        RTOS_INFO("CC_CONFIG_RegAddr set:%x, read:%x", bq769xx_reg.CCCfg, bq769xx_config[7]);
        ret = HAL_ERROR;
    }

    return ret;
}

uint8_t BQ769xx_get_data(void)
{
    uint8_t ret;
    uint8_t bq769xx_data_gather[40];
    int i=0, j=0, k=0;

    ret = Bq769xx_i2c_read(VC1_HI_RegAddr, bq769xx_data_gather, 40);
    if (ret == HAL_OK) {

        battery.timestamp = osKernelGetTickCount();

        for (i=0; i < battery.property.serial; i++) {
            if (bq769xx.offset & 0x10) {
                uint8_t vol_offset = 0x100 - bq769xx.offset;
                battery.collection.vol_cell[j] = (((uint16_t)(((bq769xx_data_gather[2*i] & 0x3f) << 8) + (bq769xx_data_gather[2*i+1] & 0xff)) * bq769xx.gain_uv) / 1000 - vol_offset) * 0.001;
            } else {
                uint8_t vol_offset = bq769xx.offset;
                battery.collection.vol_cell[j] = (((uint16_t)(((bq769xx_data_gather[2*i] & 0x3f) << 8) + (bq769xx_data_gather[2*i+1] & 0xff)) * bq769xx.gain_uv) / 1000 + vol_offset) * 0.001;
            }
            j++;
        }

        battery.collection.vol_pack = ((uint16_t)((bq769xx_data_gather[30] << 8) + bq769xx_data_gather[31]) * BATVOLTLSB) * 0.001;

        for (k=0; k<battery.property.tmp_cnt; k++) {
            battery.collection.thermistor[k] = (((uint16_t)(bq769xx_data_gather[k*2+32] << 8) + bq769xx_data_gather[k*2+33]) * 382) * 0.001;
        }

        uint16_t adc_cur = ((uint16_t)(bq769xx_data_gather[38] << 8) + bq769xx_data_gather[39]);
        if (adc_cur & 0x8000) {
            adc_cur = 0x8000 - (adc_cur & 0x7fff);
            battery.collection.current = (float)(-(adc_cur * 8.44 * 1e-6 / 0.0025));
        } else {
            battery.collection.current = (float)(adc_cur * 8.44 * 1e-6 / 0.0025);
        }
    }

    return ret;
}

uint8_t find_current_map(uint8_t cur_range)
{
    uint8_t ret;

    switch (cur_range) {
        case 0x00:
        case 0x10: ret = 0x00; break;
        case 0x01:
        case 0x11: ret = 0x01; break;
        case 0x02:
        case 0x12: ret = 0x02; break;
        case 0x03:
        case 0x13: ret = 0x03; break;
        case 0x04:
        case 0x14: ret = 0x04; break;
        case 0x05:
        case 0x15: ret = 0x05; break;
        case 0x06:
        case 0x16: ret = 0x06; break;
        case 0x07:
        case 0x17: ret = 0x07; break;
        default: ret = 0x07; break;
    }

    return ret;
}

uint8_t IIC_CRC8Bytes(uint8_t *ptr, uint8_t len, uint8_t key)
{
    uint8_t i;
    uint8_t crc=0;
    while(len--!=0)
    {
        for(i=0x80; i!=0; i/=2) {
            
            if((crc & 0x80) != 0) {
                crc *= 2;
                crc ^= key;
            } else {
                crc *= 2;
            }

            if((*ptr & i)!=0) {
                crc ^= key;
            }
        }
        ptr++;
    }
    return crc;
}

uint8_t Bq769xx_i2c_read(uint8_t reg, uint8_t *data, const uint8_t len)
{
    uint8_t ret;
    uint8_t received_data[len*2];   // 每读一个寄存器数据之后都会紧接着返回一个crc校验值 所以总的读取数据大小为len*2
    uint8_t crc_data[len*2+1];      // 除了存放被读取的数据外 还要存放一位从机地址

    crc_data[0] = (BQ769xxAddr << 1) | 0x01;

    ret = HAL_I2C_Mem_Read(&hi2c1, (BQ769xxAddr<<1)&0xfe, reg, I2C_MEMADD_SIZE_8BIT, received_data, sizeof(received_data), 100);
    
    if(ret == HAL_OK) {

        memcpy(&crc_data[1], received_data, len*2);

        if (IIC_CRC8Bytes(crc_data, 2, 0x07) != crc_data[2]) {
            RTOS_ERROR("read #reg: 0x%x crc pre-check fail", reg);
            goto fail;
        }

        for (int i=1; i<len; i++) {

            if (IIC_CRC8Bytes(&crc_data[i*2+1], 1, 0x07) != crc_data[i*2+2]) {

                RTOS_ERROR("read #reg:0x%x crc multi-check fail", reg);
                goto fail;
            }
        }
        
        for (int j=0; j<len; j++) {

            data[j] = crc_data[j*2+1];
        }
    }

fail:
    return ret;
}

uint8_t Bq769xx_i2c_write(uint8_t reg, uint8_t *data, const uint8_t len)
{
    uint8_t send_data[len + 3];

    send_data[0] = (BQ769xxAddr << 1) & 0xfe;
    send_data[1] = reg;
    memcpy(&send_data[2], data, len);
    send_data[2 + len] = IIC_CRC8Bytes(send_data, sizeof(send_data)-1, 0x07);

    /* 计算CRC的时候需要将从机地址和寄存器地址都算在内，HAL_I2C_Mem_Write函数在发送数据时就已经在函数内部发送过从机地址和寄存器地址了，所以send_Data里就不用再重复发送了 */
    uint8_t ret = HAL_I2C_Mem_Write(&hi2c1, send_data[0], send_data[1], I2C_MEMADD_SIZE_8BIT, &send_data[2], len + 1, 10);

    return ret;
}

float limit_max_min(float n, float min, float max)
{
    if (n < min) {
        return min;
    } else if (n > max) {
        return max;
    } else {
        return n;
    }
}

float get_min_cell(float *cell, uint8_t len)
{
    float max = cell[0];
    float min = cell[0];

    for (int i=0; i<len; i++) {
        if (cell[i] > max) {
            max = cell[i];
        } else if (cell[i] < min) {
            min = cell[i];
        }
    }

    return min;
}

void battery_status_publish(void)
{
    BatteryStatusStructure bat_status;

    bat_status.timestamp = battery.timestamp;
    bat_status.cell_vol = get_min_cell(battery.collection.vol_cell, battery.property.serial);
    bat_status.vol = battery.collection.vol_pack;
    bat_status.current = battery.collection.current;
    bat_status.temperature = (battery.collection.thermistor[0] + battery.collection.thermistor[1] + battery.collection.thermistor[2]) / 3;

    if (osOK != osMessageQueuePut(battery_status_bq769xx_to_socHandle, &bat_status, 0, 10)) {
        RTOS_WARN("[BMS ERROR]publish battery message from BQ769XX to SOC on time:%d", bat_status.timestamp);
    }

    if (osOK != osMessageQueuePut(battery_status_bq769xx_to_publishHandle, &bat_status, 0, 10)) {
        RTOS_WARN("[BMS ERROR]publish battery message from BQ769XX to PUBLISH on time:%d", bat_status.timestamp);
    }
}

void bq769xx_discharge_set(bool on_off)
{
    uint8_t ret;
    uint8_t sys_ctrl2_reg;

    ret = Bq769xx_i2c_read(SYS_CTRL2_RegAddr, &sys_ctrl2_reg, 1);

    if (on_off) {
        sys_ctrl2_reg |= 0x02;
        ret = Bq769xx_i2c_write(SYS_CTRL2_RegAddr, &sys_ctrl2_reg, 1);
    } else {
        sys_ctrl2_reg &= 0xfd;
        ret = Bq769xx_i2c_write(SYS_CTRL2_RegAddr, &sys_ctrl2_reg, 1);
    }

    if (ret != HAL_OK) {
        RTOS_ERROR("set discharge state error");
    }
}

void bq769xx_charge_set(bool on_off)
{
    uint8_t ret;
    uint8_t sys_ctrl2_reg;

    ret = Bq769xx_i2c_read(SYS_CTRL2_RegAddr, &sys_ctrl2_reg, 1);

    if (on_off) {
        sys_ctrl2_reg |= 0x01;
        ret = Bq769xx_i2c_write(SYS_CTRL2_RegAddr, &sys_ctrl2_reg, 1);
    } else {
        sys_ctrl2_reg &= 0xfe;
        ret = Bq769xx_i2c_write(SYS_CTRL2_RegAddr, &sys_ctrl2_reg, 1);
    }

    if (ret != HAL_OK) {
        RTOS_ERROR("set charge state error");
    }
}

void system_discharge_check(void)
{
    uint8_t ret=0, res=0;

    ret = Bq769xx_i2c_read(SYS_STAT_RegAddr, &res, 1);

    /* 消除芯片内部错误导致的异常标志 */
    if (res & 0x20) {
        res |= 0x20;
        Bq769xx_i2c_write(SYS_STAT_RegAddr, &res, 1);
        RTOS_INFO("[%d]Device_XReady fault: sys status=%x", osKernelGetTickCount(), res);
    }

    /* 消除欠压标志被置位后导致的禁止放电 */
    if (res & 0x08) {
        res |= 0x08;
        Bq769xx_i2c_write(SYS_STAT_RegAddr, &res, 1);
        RTOS_INFO("[%d]UV fault: sys status=%x", osKernelGetTickCount(), res);
    }

    bq769xx_discharge_set(true);
    bq769xx_charge_set(true);
    ret = Bq769xx_i2c_read(SYS_STAT_RegAddr, &res, 1);

    if (ret != HAL_OK) {
        RTOS_INFO("[SYS_DSG_CHECK]read #reg:SYS_STAT_RegAddr failed");
    }
}
