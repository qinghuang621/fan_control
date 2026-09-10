//=====================================================================================================
// MahonyAHRS.h
//=====================================================================================================
//
// Madgwick's implementation of Mahony's AHRS algorithm.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
// 【本项目改动说明】
//   1. 积分项由"直接清零"改为"限幅"（clamp），详见 MahonyAHRS.c 中的积分限幅段。
//   2. sampleFreq 由写死的 1000.0f 改为可运行时设置（MahonyAHRS_setSampleFreq）。
//   3. 六轴（IMU）路径为主用；九轴函数保留，本轮不接磁力计故不会被调用。
//=====================================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h

//----------------------------------------------------------------------------------------------------
// Variable declaration

extern volatile float twoKp;			// 2 * proportional gain (Kp)
extern volatile float twoKi;			// 2 * integral gain (Ki)

//---------------------------------------------------------------------------------------------------
// Function declarations

/**
 * @brief 九轴姿态更新（含磁力计）。本轮不接磁力计，mx/my/mz 传 0 时自动退化为六轴。
 */
void MahonyAHRSupdate(float q[4], float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);

/**
 * @brief 六轴姿态更新（陀螺 + 加速度计）。本项目主用路径。
 */
void MahonyAHRSupdateIMU(float q[4], float gx, float gy, float gz, float ax, float ay, float az);

/**
 * @brief 设置采样频率（Hz），须与实际调用周期一致，否则姿态收敛速度会失真。
 */
void MahonyAHRS_setSampleFreq(float freq);

/**
 * @brief 取当前采样频率
 */
float MahonyAHRS_getSampleFreq(void);

/**
 * @brief 复位积分项与四元数（重新上电/姿态丢失时调用）
 */
void MahonyAHRS_reset(void);

#endif
//=====================================================================================================
// End of file
