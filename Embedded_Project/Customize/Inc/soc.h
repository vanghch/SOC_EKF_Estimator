/**********************************************************************************
 * SOC估计
 * 
 * 共包含两种计算方式:安时积分法和扩展卡尔曼滤波法
 *
 * SOC校准均采用稳态下开路电压计算，其中:
 * EMF与SOC的5阶拟合系数矩阵为[1.6026 -8.2896 14.8878 -12.0873 5.5781 2.5047]
 * SOC与EMF的5阶拟合系数矩阵为[0.0920 -1.8708 14.6606 -55.3596 101.3616	-72.3796]
 *
 * EKF估计使用一阶Thevnin模型作为状态方程和观测方程基础
 *********************************************************************************/
#ifndef __SOC_H
#define __SOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include "system.h"
#include "msg.h"
#include "msg_handle.h"

#include "usart.h"

#define POWER5(x) (x*x*x*x*x)
#define POWER4(x) (x*x*x*x)
#define POWER3(x) (x*x*x)
#define POWER2(x) (x*x)

#define SOC_RUN_SAMPLE 50
#define CALIBRATE_CUR 0.2f

void soc_update(void);

float calibration_soc_use_uoc(float vol);

float estimator_soc_use_ah_integrate(float vol, float cur, float dt, float soc_prev);
float estimator_soc_use_thevenin_ekf(float vol, float cur, float dt, float soc_prev);

void reset_ah_integrate(void);
double kahan_sum(double sum_previous, double input, double *accumulator);

void soc_estimator_ekf_initialize(float vol, float cur, float soc);
void soc_estimator_ekf(float Uk_obs, float Ik_obs, float SOCk_obs, float dt, float *Uk_est, float *Ik_est, float *SOCk_est);

void soc_status_publish(void);

#ifdef __cplusplus
}
#endif

#endif
