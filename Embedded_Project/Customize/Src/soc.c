#include "soc.h"
#include "math.h"

/**************************************************************/
/* EKF估计相关参数与电池系统参数定义，初始化在soc_estimator_ekf_initialize函数中进行 */
float R0;       // 电池内阻
float R1;       // 电池极化电阻
float C1;       // 电池极化电容
float Eta;      // 电池放电效率
float C_N;      // 电池AH数

float P[4];     // 
float Q;        // 
float R;        // 
float Wk[2];    // 
float Vk[2];    // 

float Uk;
float Ik;
float SOCk;

float Uoc_est = 0.0f;
/**************************************************************/

bool isInitialized_soc_estimator_ekf = false;

SocStatusStructure soc_status = {0};
BatteryStatusStructure last_battery_status = {0};

extern BatteryInfoStructure battery;

uint16_t battery_stable_cnt = 0;
bool soc_initialized = false;
static float soc_calculation_origin = 0.0f;
float dt = 0.0f;
static double ah_integrate_sum = 0.0;
static double ah_integrate_accumulator = 0.0;

BatteryStatusStructure battery_status = {0};

void soc_update(void)
{
    if (0 != osMessageQueueGetCount(battery_status_bq769xx_to_socHandle)) {
        if (osOK == osMessageQueueGet(battery_status_bq769xx_to_socHandle, &battery_status, 0, 10)) {

            // if (fabs(battery_status.current) < CALIBRATE_CUR) {

            //      if (!soc_initialized && battery_stable_cnt*SOC_RUN_SAMPLE > 3*1000) {
            //         /* BMS刚开机小电流情况下如果SOC还未初始化，则仅用3s后的Uoc校准SOC值 */
            //         soc_initialized = true;
            //         soc_calculation_origin = calibration_soc_use_uoc(battery_status.cell_vol);

            //         RTOS_INFO("SOC calculate initialized, start running soc estimator");

            //     } else if (soc_initialized && battery_stable_cnt * SOC_RUN_SAMPLE > 30*1000) {
            //         /* 如果BMS已初始化完成且电流在30s时间内稳定，则可以启用Uoc校准SOC的逻辑 */
            //         // soc_calculation_origin = calibration_soc_use_uoc(battery_status.cell_vol);
            //         // TODO:这里还要对AH积分器做重置以及对EKF做校准操作
            //         // reset_ah_integrate();

            //     } else {
            //         /* 防止计数器溢出，做范围限制 */
            //         if (battery_stable_cnt < 65535) {
            //             battery_stable_cnt++;
            //         } else {
            //             battery_stable_cnt = 65535;
            //         }

            //     }

            // } else {
            //     battery_stable_cnt = 0;

            //     if (soc_initialized) {

            //         dt = (battery_status.timestamp - last_battery_status.timestamp) * 1e-3;

            //         soc_status.timestamp = osKernelGetTickCount();
            //         soc_status.soc.Uoc_original = soc_calculation_origin;
            //         soc_status.soc.SOC_AHIntegrate = estimator_soc_use_ah_integrate(battery_status.cell_vol, battery_status.current, dt, soc_status.soc.SOC_AHIntegrate);
            //         soc_status.soc.EKF_Estimate = estimator_soc_use_thevenin_ekf(battery_status.cell_vol, battery_status.current, dt, soc_status.soc.EKF_Estimate);
            //         soc_status.soc.EKF_Estimate = estimator_soc_use_thevenin_ekf(battery_status.cell_vol, battery_status.current, dt, soc_status.soc.EKF_Estimate);
            //         soc_status.terminal_voltage_detect = battery_status.cell_vol;
            //         soc_status.terminal_voltage_estimate = Uoc_est;
            //         soc_status.polarization_voltage_estimate = Uk;

            //     } else {
            //         RTOS_INFO("large system current discharge is detected when soc estimator start");

            //     }
            // }

/***************************************************************************************************************************************************/
            if (fabs(battery_status.current) < CALIBRATE_CUR && !soc_initialized) {

                if (battery_stable_cnt < 65535) {
                    battery_stable_cnt++;
                } else {
                    battery_stable_cnt = 65535;
                }

                if (battery_stable_cnt*SOC_RUN_SAMPLE > 3*1000) {
                /* BMS刚开机小电流情况下如果SOC还未初始化，则仅用3s后的Uoc校准SOC值 */
                soc_initialized = true;
                soc_calculation_origin = calibration_soc_use_uoc(battery_status.cell_vol);

                RTOS_INFO("SOC calculate initialized, start running soc estimator");

                }
            }

            if (soc_initialized) {

                dt = (battery_status.timestamp - last_battery_status.timestamp) * 1e-3;

                soc_status.timestamp = osKernelGetTickCount();
                soc_status.soc.Uoc_original = calibration_soc_use_uoc(battery_status.cell_vol);
                soc_status.soc.SOC_AHIntegrate = estimator_soc_use_ah_integrate(battery_status.cell_vol, battery_status.current, dt, soc_status.soc.SOC_AHIntegrate);
                soc_status.soc.EKF_Estimate = estimator_soc_use_thevenin_ekf(battery_status.cell_vol, battery_status.current, dt, soc_status.soc.EKF_Estimate);
                soc_status.terminal_voltage_detect = battery_status.cell_vol;
                soc_status.terminal_voltage_estimate = Uoc_est;
                soc_status.polarization_voltage_estimate = Uk;

            }
/***************************************************************************************************************************************************/

            last_battery_status = battery_status;

            soc_status_publish();

            // RTOS_INFO("Uoc: %.4f, AH_Int: %.4f, EKF: %.4f", soc_status.soc.Uoc_original, soc_status.soc.SOC_AHIntegrate, soc_status.soc.EKF_Estimate);
            // RTOS_INFO("Vdet: %.2f, Vest: %.2f, Vpol: %.2f\r\n", soc_status.terminal_voltage_detect, soc_status.terminal_voltage_estimate, soc_status.polarization_voltage_estimate);
        }
    }
}

float calibration_soc_use_uoc(float vol)
{
    float soc;

    reset_ah_integrate();

    soc = 0.0920f*POWER5(vol) - 1.8708f*POWER4(vol) + 14.6606f*POWER3(vol) - 55.3596f*POWER2(vol) + 101.3616f*vol - 72.3796f;

    soc_status.soc.Uoc_original = soc;
    soc_status.soc.SOC_AHIntegrate = soc;
    soc_status.soc.EKF_Estimate = soc;

    return soc;
}

float estimator_soc_use_ah_integrate(float vol, float cur, float dt, float soc_prev)
{
    float soc;
    const double integrand = (double)(cur * dt);
    const double FULL_CELL_CULLEN = (battery.property.capacity * battery.property.parallel) * 3600.0f;

    if (dt < 0.001f) {

        RTOS_INFO("too tiny for soc(ah-integrate) update time");
        dt = 0.001f;
    }

    if (dt > 1.0f) {

        RTOS_INFO("too large for soc(ah-integrate) update time");
        dt = 0.5f;
    }

    /* AH积分中使用卡汉求和的方式，避免大数吃小数导致累计误差进一步增大 */
    ah_integrate_sum = kahan_sum(ah_integrate_sum, integrand, &ah_integrate_accumulator);
    /* SOC电流计算的过程中电流有正有负，需要注意这个地方的电流符号 */
    soc = soc_prev + (ah_integrate_sum / FULL_CELL_CULLEN);

    return soc;
}

float estimator_soc_use_thevenin_ekf(float vol, float cur, float dt, float soc_prev)
{
    if (dt < 0.001f) {

        RTOS_INFO("too tiny for soc(EKF) update time");
        dt = 0.001f;
    }

    if (dt > 1.0f) {

        RTOS_INFO("too large for soc(EKF) update time");
        dt = 1.0f;
    }

    soc_estimator_ekf(vol, cur, soc_prev, dt, &Uk, &Ik, &SOCk);
    
    Uoc_est = Uk + (1.6026F * POWER5(SOCk) - 8.2896F * POWER4(SOCk) + 14.8878F * POWER3(SOCk) - 12.0873F * POWER2(SOCk) + 5.5781F * SOCk + 2.5047F) + Ik*R0;

    return SOCk;
}

double kahan_sum(double sum_previous, double input, double *accumulator)
{
    const double y = input - *accumulator;
	const double t = sum_previous + y;
	*accumulator = (t - sum_previous) - y;

	return t;
}

void reset_ah_integrate(void)
{
    ah_integrate_sum = 0.0;
    ah_integrate_accumulator = 0.0;
}

/* EKF估计器初始化
 * Arguments    : void
 * Return Type  : void
 */
void soc_estimator_ekf_initialize(float vol, float cur, float soc)
{
    P[0] = 2.3217018641422492E-9;
    P[1] = 4.0734431925977668E-9;
    P[2] = 4.0734431925977668E-9;
    P[3] = 4.5299999999807036E-6;

    R = 1000.0;
    Q = 0.001;
    Vk[0] = 1000.0;
    Wk[0] = 0.001;
    Vk[1] = 1000.0;
    Wk[1] = 0.001;

    C_N = 3.0f * 3600.0f;
    Eta = 0.97;
    R0 = 0.0096;
    C1 = 50.0;
    R1 = 0.0071;

    Uk = vol;
    Ik = cur;
    SOCk = soc;

    isInitialized_soc_estimator_ekf = true;

    RTOS_INFO("soc estimate method: EKF-Thevnin, init finish");
}

/* EKF估计极化电压
 * {
 *  使用EKF估计电池组电量状态
 *  状态方程和观测方程使用一阶戴维宁模型计算，电池参数可配置
 *  Note:当前电池内阻为恒定值，实际上应当根据系统辨识结果将其作为soc的函数，待补充
 * }
 *  使用Thevnin模型 初始化电池模型内部参数
 * Arguments    : float Uk_obs 电池带载或开路时的端电压，不是极化电压或者虚拟开路电压
 *                float Ik_obs 回路电流，电流方向放电为负，充电为正
 *                float SOCk_obs 观测到的SOC值，可以用AH积分的SOC，也可以使用上一次的估计SOC值作为输入
 *                float dt EKF更新间隔
 *                float *Uk_est EKF估计出的极化电压，不是端电压，需要加上内阻压降和估计SOC值对应电池电压之后才是估计的虚拟开路电压
 *                float *Ik_est EKF内部没有对电流进行估计，这里输出的就是观测电流
 *                float *SOCk_est EKF估计出的SOC值，可以直接使用
 * Return Type  : void
 */
void soc_estimator_ekf(float Uk_obs, float Ik_obs, float SOCk_obs, float dt, float *Uk_est, float *Ik_est, float *SOCk_est)
{
    float Ak[4];
    float P_[4];
    float Ck_idx_1;
    float Kk_idx_0;
    float Kk_idx_1;
    float Xk__idx_0;
    float Xk__idx_1;
    float d;
    float d1;
    float d2;
    float d3;
    float d4;
    int i;
    int i1;

    if (!isInitialized_soc_estimator_ekf) {
        soc_estimator_ekf_initialize(Uk_obs, Ik_obs, SOCk_obs);
    }

    /*  使用Thevnin模型 初始化电池模型内部参数 */
    /*  协方差矩阵 过程噪声以及观测噪声矩阵 */
    /*  迭代中的各种状态变量 */
    /*  状态方程 可以根据状态转移是否为线性或者非线性选择使用f(xk, uk)的形式表示或者用矩阵形式表示 */
    Kk_idx_1 = exp(-dt / (R1 * C1));

    /*  这里先验估计的Uk_为极化电压 */
    /*  这里先验估计的SOCk为系统SOC状态 */
    Xk__idx_0 = Kk_idx_1 * Uk + R1 * (1.0 - Kk_idx_1) * Ik;
    Xk__idx_1 = SOCk + Eta * dt / C_N * Ik;

    /*  先验估计状态 */
    Ak[0] = Kk_idx_1;
    Ak[2] = 0.0;
    Ak[1] = 0.0;
    Ak[3] = 1.0;

    /*  Ak为状态方程两个公式所对应的雅可比矩阵 */
    /*  计算误差协方差 */
    d = P[0];
    Ck_idx_1 = P[1];
    d1 = P[2];
    d2 = P[3];

    for (i = 0; i < 2; i++) {

        i1 = (int)Ak[i + 2];
        d3 = Ak[i];
        d4 = d3 * d + (float)i1 * Ck_idx_1;
        d3 = d3 * d1 + (float)i1 * d2;
        P_[i] = d4 * Kk_idx_1 + d3 * 0.0;
        P_[i + 2] = d4 * 0.0 + d3;
    }

    Kk_idx_1 = Wk[0] * Q;
    Ck_idx_1 = Wk[1] * Q;
    P_[0] += Kk_idx_1 * Wk[0];
    P_[1] += Ck_idx_1 * Wk[0];
    P_[2] += Kk_idx_1 * Wk[1];
    P_[3] += Ck_idx_1 * Wk[1];

    /*  先验估计计算出的误差协方差矩阵 */
    /*  观测方程 EMF与SOC的5阶拟合系数矩阵为[1.6026 -8.2896 14.8878 -12.0873 5.5781 2.5047] */
    /*  TODO:在观测方程中R0也是SOC的函数，此处用一个恒定的值代替，后续待补充 */
    /*  这里的Uk_pola为计算出的极化电压 */
    /*  观测状态 */
    Ck_idx_1 = (((8.013 * POWER4(SOCk) - 33.1584 * POWER3(SOCk)) + 44.6634 * POWER2(SOCk)) - 24.1746 * SOCk) + 5.5781;

    /*  Ck为两个观测方程对应的雅可比矩阵 */
    /*  计算卡尔曼增益系数 */
    d = Ck_idx_1 * P_[3];
    Kk_idx_1 = ((P_[0] + Ck_idx_1 * P_[1]) + (P_[2] + d) * Ck_idx_1) + (Vk[0] * R * Vk[0] + Vk[1] * R * Vk[1]);
    Kk_idx_0 = (P_[0] + P_[2] * Ck_idx_1) / Kk_idx_1;
    Kk_idx_1 = (P_[1] + d) / Kk_idx_1;

    /*  更新卡尔曼系数 */
    /*  计算卡尔曼滤波输出 */
    /*  后验更新估计状态 */
    /*  更新误差协方差矩阵 */
    Ak[0] = 1.0 - Kk_idx_0;
    Ak[1] = 0.0 - Kk_idx_1;
    Ak[2] = 0.0 - Kk_idx_0 * Ck_idx_1;
    Ak[3] = 1.0 - Kk_idx_1 * Ck_idx_1;
    d = P_[0];
    Ck_idx_1 = P_[1];
    d1 = P_[2];
    d2 = P_[3];

    for (i = 0; i < 2; i++) {

        d3 = Ak[i + 2];
        d4 = Ak[i];
        P[i] = d4 * d + d3 * Ck_idx_1;
        P[i + 2] = d4 * d1 + d3 * d2;
    }

    /*  后验更新误差协方差矩阵 */
    *Uk_est = Xk__idx_0 + Kk_idx_0 * (((Uk_obs - R0 * Ik_obs) - (((((1.6026 * POWER5(SOCk_obs) - 8.2896 * POWER4(SOCk_obs)) + 14.8878 * POWER3(SOCk_obs)) - 12.0873 * POWER2(SOCk_obs)) + 5.5781 * SOCk_obs) + 2.5047)) - Xk__idx_0);
    *Ik_est = Ik_obs;
    *SOCk_est = Xk__idx_1 + Kk_idx_1 * (SOCk_obs - Xk__idx_1);
}

void soc_status_publish(void)
{
    if (osOK != osMessageQueuePut(soc_status_soc_to_publishHandle, &soc_status, 0, 10)) {
        RTOS_ERROR("publish battery message on time:%d", soc_status.timestamp);
    }
}
