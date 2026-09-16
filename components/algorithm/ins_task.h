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

    /* 姿态四元数，顺序 (w, x, y, z)，表示**机体系 → 世界系**的旋转，
     * 即 v_world = R(q) · v_body。
     *
     * ⚠️ 加在结构体**末尾**是刻意的：不动已有字段的内存布局，
     *    既有消费方（bsp_led_tilt.c 等）无需改动。
     *
     * ⚠️ 与 ROS2 `geometry_msgs/Quaternion` 的字段顺序**相反**
     *    （ROS 是 x,y,z,w；我们是 w,x,y,z）。发布时必须显式映射，**不能内存直拷**。
     *    详见 `接口文档.md` §6.8。 */
    float quat_w;
    float quat_x;
    float quat_y;
    float quat_z;
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
#define INS_TEMP_CONFIRM_CNT      30U       /* 温度超过目标后需连续确认的次数。
                                             * 预热循环周期 50ms → 30×50ms = **1.5 s**。
                                             * 2026-09-16 由 200 降到 30：200 次 = 10 s，
                                             * 在"温度 2 s 就到 45℃"的前提下白白拖长预热，
                                             * 而这段时间自动模式不接管（见 fan_auto_update()
                                             * 的 ins_get_status() 门），是有风险的窗口。 */
#define INS_SAMPLE_PERIOD_MS      1U        /* 姿态解算周期(ms) -> 1kHz */

/* 温控"提前切闭环 PID"的入口带宽(℃)：温度一旦升到 `目标 − 本值` 就交 PID。
 * 为什么需要它（2026-09-16 实测）：
 *   旧实现要等 `s_first_temperate`（= 连续 30 次确认，1.5 s）才切 PID，
 *   而**那段确认窗口里仍在满功率(90%)加热**（因为"温度超过目标"只是累加计数，
 *   随后照样落到 `s_heater_pwm = MAX_OUT - 1`）→ 已经在 45℃ 以上还猛烧 1.5 s，
 *   把过冲从 ~1℃ 推到 ~4℃（实测冲到 **49℃** 才停）。
 *   加热控制与"何时开始对外发快照"是两件事，本值用于解耦前者。
 * 取 2.0℃：`Kp=1600` 极大，误差 2℃ 时 P 项已达 3200（叠加预置 Iout≈2250 即饱和），
 * 所以入口取得更早没有额外收益；而在最后 2℃ 内输出会从 90% 平滑降到 45%，起到软着陆作用。 */
#define INS_TEMP_PID_ENGAGE_MARGIN 2.0f

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
