#ifndef INS_TASK_H
#define INS_TASK_H

#include "struct_typedef.h"

/* ============================ 姿态快照 ============================
 * InsTask 以 1kHz 频率解算姿态，其他任务（Modbus/Led/Fan）以低频读取。
 * 直接读 float 数组会在写入中途被抢占而读到"半新半旧"的数据，
 * 因此采用 seqlock：写者写 seq(奇数) -> 写数据 -> 写 seq(偶数)，
 * 读者前后各读一次 seq，只有一致且为偶数才认为数据有效。
 * 该方案不需要关中断，读取失败时用上一次的有效值即可。 */
typedef struct
{
    float roll;      /* deg, 绕 X 轴（左右翻滚），向右倾为正 */
    float pitch;     /* deg, 绕 Y 轴（前后俯仰），抬头为正 */
    float yaw;       /* deg, 绕 Z 轴；六轴模式下会缓慢漂移，本轮仅作占位输出 */
    float gyro_x;    /* rad/s, 机体坐标系 */
    float gyro_y;
    float gyro_z;
    float accel_x;   /* g */
    float accel_y;
    float accel_z;
    float temperature;  /* ℃, BMI088 板载传感器 */
} ins_snapshot_t;

/* ============================ 运行状态 ============================ */
typedef enum
{
    INS_STATUS_OFFLINE = 0,   /* 未初始化完成或初始化失败 */
    INS_STATUS_WARMUP,        /* 恒温加热中，姿态尚未对外输出 */
    INS_STATUS_RUNNING,       /* 姿态解算正常，数据可用 */
    INS_STATUS_ERROR          /* IMU 读取连续失败 */
} ins_status_e;

/* ============================ 时间参数 ============================ */
#define INS_TASK_INIT_TIME        7U        /* 上电后等 IMU 上电稳定的时间(ms) */
#define INS_TEMP_BOOT_TIMEOUT_MS  20000U    /* 恒温加热最长时间(ms)，超时后强制进入 RUNNING */
#define INS_TEMP_CONFIRM_CNT      200U      /* 温度进入目标带宽后需连续确认的次数 */
#define INS_SAMPLE_PERIOD_MS      1U        /* 姿态解算周期(ms) -> 1kHz */

/* ============================ 目标温度 ============================ */
/* BMI088 手册建议温度 = 环境温度 + 15~20℃（恒温的目的是抑制零偏温漂，
 * 而不是把 IMU 烤到某个固定值）。45℃ 是例程的固定目标。
 * 通过 ins_set_target_temp() 可在运行时覆盖（如由 Modbus 参数下发）。 */
#define INS_TEMP_DEFAULT_TARGET   45.0f

/* ============================ 对外接口 ============================ */

/**
 * @brief  姿态任务入口（提供给 xTaskCreate）
 */
void InsTask_Entry(void *argument);

/* ---- 采集链路的 ISR 由本模块提供（理由见 ins_task.c 注释）---- */
void DMA2_Stream2_IRQHandler(void);   /* SPI1 RX 完成：推进状态机 + 续发下一通道 */
void DMA2_Stream3_IRQHandler(void);   /* SPI1 TX 完成：清标志 */
void EXTI0_IRQHandler(void);          /* 软触发：唤醒 InsTask */
void EXTI4_IRQHandler(void);          /* INT1_ACCEL(PC4)：加速度计/温度就绪 */
void EXTI9_5_IRQHandler(void);        /* INT1_GYRO(PC5)：陀螺就绪 */

/**
 * @brief  初始化 BMI088 与姿态解算（必须在 FreeRTOS 调度器启动后调用）
 * @retval 0 成功，非 0 失败
 */
uint8_t ins_init(void);

/**
 * @brief  取一次姿态快照（seqlock 读）
 * @param  out 输出结构体指针
 * @retval 1 数据有效；0 数据正在被更新（调用方可沿用上次值）
 */
uint8_t INS_get_snapshot(ins_snapshot_t *out);

/**
 * @brief  取当前运行状态
 */
ins_status_e ins_get_status(void);

/**
 * @brief  设置恒温目标温度（℃），用于调试或上位机下发
 */
void ins_set_target_temp(float temp);

/**
 * @brief  取当前恒温目标温度
 */
float ins_get_target_temp(void);

/**
 * @brief  取最近一次加热 PWM 值（用于诊断）
 */
uint16_t ins_get_heater_pwm(void);

#endif
