#ifndef __BQ76940_H
#define __BQ76940_H

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "i2c.h"

/***************************************/
/* 这里包括所有需要发布和订阅的队列头文件 */
#include "msg.h"
#include "msg_handle.h"

#define BQ769xxAddr         0x08
#define SYS_STAT_RegAddr    0x00
#define CELLBAL1_RegAddr    0x01
#define CELLBAL2_RegAddr    0x02
#define CELLBAL3_RegAddr    0x03
#define SYS_CTRL1_RegAddr   0x04
#define SYS_CTRL2_RegAddr   0x05
#define PROTECT1_RegAddr    0x06
#define PROTECT2_RegAddr    0x07
#define PROTECT3_RegAddr    0x08
#define OV_TRIP_RegAddr     0x09
#define UV_TRIP_RegAddr     0x0A
#define CC_CONFIG_RegAddr   0x0B

#define VC1_HI_RegAddr      0x0C
#define VC1_LO_RegAddr      0x0D
#define VC2_HI_RegAddr      0x0E
#define VC2_LO_RegAddr      0x0F
#define VC3_HI_RegAddr      0x10
#define VC3_LO_RegAddr      0x11
#define VC4_HI_RegAddr      0x12
#define VC4_LO_RegAddr      0x13
#define VC5_HI_RegAddr      0x14
#define VC5_LO_RegAddr      0x15

#define VC6_HI_RegAddr      0x16
#define VC6_LO_RegAddr      0x17
#define VC7_HI_RegAddr      0x18
#define VC7_LO_RegAddr      0x19
#define VC8_HI_RegAddr      0x1A
#define VC8_LO_RegAddr      0x1B
#define VC9_HI_RegAddr      0x1C
#define VC9_LO_RegAddr      0x1D
#define VC10_HI_RegAddr     0x1E
#define VC10_LO_RegAddr     0x1F

#define VC11_HI_RegAddr     0x20
#define VC11_LO_RegAddr     0x21
#define VC12_HI_RegAddr     0x22
#define VC12_LO_RegAddr     0x23
#define VC13_HI_RegAddr     0x24
#define VC13_LO_RegAddr     0x25
#define VC14_HI_RegAddr     0x26
#define VC14_LO_RegAddr     0x27
#define VC15_HI_RegAddr     0x28
#define VC15_LO_RegAddr     0x29

#define BAT_HI_RegAddr      0x2A
#define BAT_LO_RegAddr      0x2B
#define TS1_HI_RegAddr      0x2C
#define TS1_LO_RegAddr      0x2D
#define TS2_HI_RegAddr      0x2E
#define TS2_LO_RegAddr      0x2F
#define TS3_HI_RegAddr      0x30
#define TS3_LO_RegAddr      0x31
#define CC_HI_RegAddr       0x32
#define CC_LO_RegAddr       0x33

#define ADCGAIN1_RegAddr    0x50
#define ADCOFFSET_RegAddr   0x51
#define ADCGAIN2_RegAddr    0x59

#define CUR_LRANGE_8_8A     0x00
#define CUR_LRANGE_13_2A    0x01
#define CUR_LRANGE_17_6A    0x02
#define CUR_LRANGE_22_4A    0x03
#define CUR_LRANGE_26_8A    0x04
#define CUR_LRANGE_31_2A    0x05
#define CUR_LRANGE_35_6A    0x06
#define CUR_LRANGE_100A     0x07
#define CUR_HRANGE_17_6A    0x10
#define CUR_HRANGE_26_8A    0x11
#define CUR_HRANGE_35_6A    0x12
#define CUR_HRANGE_44_4A    0x13
#define CUR_HRANGE_53_2A    0x14
#define CUR_HRANGE_62A      0x15
#define CUR_HRANGE_71_2A    0x16
#define CUR_HRANGE_80A      0x17

#define OV_THRESHOLD_MAX    4250
#define OV_THRESHOLD_MIN    3150
#define UV_THRESHOLD_MAX    3500
#define UV_THRESHOLD_MIN    1800
#define OV_THRESHOLD_BASE   0x2008  // 具体含义见芯片使用手册
#define UV_THREAHOLD_BASE   0x1000  // 具体含义见芯片使用手册
#define BATVOLTLSB          1.532
#define CRUUVOLTLSB         8.44

typedef struct
{
    union {
        struct {
            uint8_t OCD             :1;
            uint8_t SCD             :1;
            uint8_t OV              :1;
            uint8_t UV              :1;
            uint8_t OVRD_ALERT      :1;
            uint8_t DEVICE_XREADY   :1;
            uint8_t RSVD            :1;
            uint8_t CC_READY        :1;
        }SysStatBit;
        uint8_t SysStatByte;
    }SysStat;

    union {
        struct {
            uint8_t CB1             :1;
            uint8_t CB2             :1;
            uint8_t CB3             :1;
            uint8_t CB4             :1;
            uint8_t CB5             :1;
            uint8_t RSVD            :3;
        }CellBal1Bit;
        uint8_t CellBal1Byte;
    }CellBal1;

    union {
        struct {
            uint8_t CB6             :1;
            uint8_t CB7             :1;
            uint8_t CB8             :1;
            uint8_t CB9             :1;
            uint8_t CB10            :1;
            uint8_t RSVD            :3;
        }CellBal2Bit;
        uint8_t CellBal2Byte;
    }CellBal2;

    union {
        struct {
            uint8_t CB11            :1;
            uint8_t CB12            :1;
            uint8_t CB13            :1;
            uint8_t CB14            :1;
            uint8_t CB15            :1;
            uint8_t RSVD            :3;
        }CellBal3Bit;
        uint8_t CellBal3Byte;
    }CellBal3;

    union {
        struct {
            uint8_t SHUT_B          :1;
            uint8_t SHUT_A          :1;
            uint8_t RSVD1           :1;
            uint8_t TEMP_SEL        :1;
            uint8_t ADC_EN          :1;
            uint8_t RSVD2           :2;
            uint8_t LOAD_PRESENT    :1;
        }SysCtrl1Bit;
        uint8_t SysCtrl1Byte;
    }SysCtrl1;

    union {
        struct {
            uint8_t CHG_ON          :1;
            uint8_t DSG_ON          :1;
            uint8_t RSVD            :1;
            uint8_t CC_ONESHOT      :3;
            uint8_t CC_EN           :1;
            uint8_t DELAY_DIS       :1;
        }SysCtrl2Bit;
        uint8_t SysCtrl2Byte;
    }SysCtrl2;

    union {
        struct {
            uint8_t SCD_THRESH      :3;
            uint8_t SCD_DELAY       :2;
            uint8_t RSVD            :2;
            uint8_t RSNS            :1;
        }Protect1Bit;
        uint8_t Protect1Byte;
    }Protect1;

    union {
        struct {
            uint8_t OCD_THRESH      :4;
            uint8_t OCD_DELAY       :3;
            uint8_t RSVD            :1;
        }Protect2Bit;
        uint8_t Protect2Byte;
    }Protect2;

    union {
        struct {
            uint8_t RSVD            :4;
            uint8_t OV_DELAY        :2;
            uint8_t UV_DELAY        :2;
        }Protect3Bit;
        uint8_t Protect3Byte;
    }Protect3;

    uint8_t OVTrip;
    uint8_t UVTrip;
    uint8_t CCCfg;

    uint8_t VCx[30];
}BQ769xxRegisterInfoStructure;

typedef struct
{
    uint32_t gain_uv;
    uint16_t gain_mv;
    uint8_t offset;
    uint16_t ov_threshold;
    uint16_t uv_threshold;
}BQ796xxInfoStructure;

/* @brief: 初始化需要写入bq769xx采集芯片的各项参数，并通过IIC写入，同时读取芯片的ADC增益和偏置
 * @param: none
 * @retval: none
 */
void BQ769xx_Init(void);

/* @brief: 初始化需要写入bq769xx采集芯片的各项参数
 * @param: none
 * @retval: none
 */
void BQ769xx_init_parameters(void);

/* @brief: 读取芯片的ADC增益和偏置
 * @param: none
 * @retval: 返回读取数据的结果
 */
uint8_t BQ769xx_read_gain_offset(void);

/* @brief: 通过IIC写入bq769xx采集芯片的各项参数
 * @param: none
 * @retval: 返回读取数据的结果
 */
uint8_t BQ769xx_write_config(void);

/* @brief: 获取芯片采集到的电流电压值和热敏电阻温度
 * @param: none
 * @retval: 返回读取数据的结果
 */
uint8_t BQ769xx_get_data(void);

uint8_t find_current_map(uint8_t cur_range);

/* @brief: 计算CRC校验值
 * @param:
 *          ptr: 需要计算校验的数据
 *          len: 需要校验的数据长度
 *          key: 校验key
 * @retval: none
 */
uint8_t IIC_CRC8Bytes(uint8_t *ptr, uint8_t len, uint8_t key);

/* @brief: bq769xx读取指定长度的数据
 * @param: 略
 * @retval: 返回读取数据的结果
 */
uint8_t Bq769xx_i2c_read(uint8_t reg, uint8_t *data, const uint8_t len);

/* @brief: bq769xx写入指定长度的数据
 * @param: 略
 * @retval: 返回读取数据的结果
 * note: 当前暂时不支持写入制定长度的数据，原因是每个数据进行发送的时候都需要单独做校验，检验方法参考read函数，后续有必要时再添加
 */
uint8_t Bq769xx_i2c_write(uint8_t reg, uint8_t *data, const uint8_t len);

float limit_max_min(float n, float min, float max);

float get_min_cell(float *cell, uint8_t len);

/* @brief: 发布采集到的电池数据
 * @param: none
 * @retval: none
 */
void battery_status_publish(void);

/* @brief: bq769xx开启/关闭放电mos
 * @param: on_off 打开/关闭放电功能
 * @retval: none
 */
void bq769xx_discharge_set(bool on_off);

/* @brief: bq769xx开启/关闭充电mos
 * @param: on_off 打开/关闭充电功能
 * @retval: none
 */
void bq769xx_charge_set(bool on_off);

/* @brief: 检查SYS_STATUS的状态，保证在任何情况下放电不受影响
 * @param: none
 * @retval: none
 */
void system_discharge_check(void);

#ifdef __cplusplus
}
#endif

#endif  /* __BQ76940_H */
