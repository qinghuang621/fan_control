/**
  ******************************************************************************
  * @file    ins_task.c
  * @brief   BMI088 六轴姿态解算任务 + 恒温加热控制
  *
  * 采集链路完全对齐例程 661c2-main/18.ins_task/application/INS_task.c：
  *
  *   BMI088 硬件数据就绪中断 (INT1_GYRO=PC5 / INT1_ACCEL=PC4)
  *        ↓ HAL_GPIO_EXTI_Callback 置 *_update_flag 的 DR 位
  *        ↓ imu_cmd_spi_dma() 选择要发起的通道并拉低对应 CS
  *        ↓ SPI1_DMA_enable() 裸启动 DMA
  *   DMA2_Stream2 TC 中断
  *        ↓ 拉高 CS、状态由 SPI_SHIFT 推进到 UPDATE_SHIFT
  *        ↓ imu_cmd_spi_dma() 发起下一个通道（陀螺→加速度→温度 轮转）
  *        ↓ 任一通道到了 UPDATE 位就 __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0)
  *   EXTI0 IRQ
  *        ↓ HAL_GPIO_EXTI_Callback 里 GPIO_PIN_0 分支 vTaskNotifyGiveFromISR
  *   任务被唤醒，解析缓冲区、解算姿态、发布快照
  *
  * ⚠️ 关键：驱动层（BMI088driver.c / bsp_spi.c）**不使用 HAL 的收发 API**，
  *    而是裸操作 DMA 寄存器 + 自己的 DMA 完成处理。因此
  *    HAL_SPI_TxRxCpltCallback **不会被调用**，绝不能把状态机挂在那里。
  *    这也是例程把 DMA2_Stream2_IRQHandler 直接写在 INS_task.c 里的原因。
  *
  * 本文件相对例程的差异：
  *   1. 六轴：只用 MahonyAHRSupdateIMU，不接 IST8310（yaw 仅占位）。
  *   2. MahonyAHRS 积分项由"清零"改为"限幅"，采样率运行时设置。
  *   3. 快照改用 seqlock，姿态按 deg 输出。
  ******************************************************************************
  */

#include "ins_task.h"
#include "main.h"

#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "MahonyAHRS.h"
#include "pid.h"
#include "bsp_spi.h"
#include "bsp_imu_pwm.h"
#include "spi.h"
#include "tim.h"

#include "FreeRTOS.h"
#include "task.h"

#include <math.h>

/* ============================ 采样缓冲 ============================ */
/* tx 缓冲区首字节 = 寄存器地址 | 0x80(读标志)。
 *   陀螺   ：0x80|0x02 = 0x82  (BMI088_GYRO_X_L)
 *   加速度 ：0x80|0x12 = 0x92  (BMI088_ACCEL_XOUT_L)
 *   温度   ：0x80|0x22 = 0xA2  (BMI088_TEMP_M)
 * 例程用 0xFF 填充剩余字节，此处保持一致（DUMMY 不影响读回）。 */
#define BMI088_GYRO_RX_BUF_DATA_OFFSET   1U
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET  2U

#define SPI_DMA_GYRO_LENGHT       8U
#define SPI_DMA_ACCEL_LENGHT      9U
#define SPI_DMA_ACCEL_TEMP_LENGHT 4U

static uint8_t gyro_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint8_t gyro_rx_buf[SPI_DMA_GYRO_LENGHT] = {0};

static uint8_t accel_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92U, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU};
static uint8_t accel_rx_buf[SPI_DMA_ACCEL_LENGHT] = {0};

static uint8_t accel_temp_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2U, 0xFFU, 0xFFU, 0xFFU};
static uint8_t accel_temp_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0};

/* 采集状态机（与例程一致）：
 *   bit0(DR)     : 硬件数据就绪，等待发起 SPI
 *   bit1(SPI)    : SPI/DMA 传输中
 *   bit2(UPDATE) : 数据已收齐，等待任务解析
 * 每次 DMA 完成中断把状态位从 SPI 推进到 UPDATE。 */
#define IMU_DR_SHFITS      0U
#define IMU_SPI_SHFITS     1U
#define IMU_UPDATE_SHFITS  2U

static volatile uint8_t gyro_update_flag = 0;
static volatile uint8_t accel_update_flag = 0;
static volatile uint8_t accel_temp_update_flag = 0;

/* 初始化完成后置 1，允许数据就绪中断发起 DMA */
static volatile uint8_t imu_start_dma_flag = 0;

/* 任务句柄，供 EXTI0 软中断唤醒使用 */
static TaskHandle_t s_ins_task_handle = NULL;

/* ============================ 姿态数据 ============================ */
static float q[4] = {1.0f, 0.0f, 0.0f, 0.0f};   /* 四元数 w,x,y,z */

static volatile ins_status_e s_ins_status = INS_STATUS_OFFLINE;

/* seqlock 快照 */
static volatile uint32_t s_snap_seq = 0;
static ins_snapshot_t s_snap = {0};

/* 恒温 PID */
static pid_type_def s_heater_pid;
static const fp32 s_heater_pid_param[3] = {1600.0f, 0.2f, 0.0f};
#define HEATER_PID_MAX_OUT   4500.0f
#define HEATER_PID_MAX_IOUT  4400.0f
static volatile float s_target_temp = INS_TEMP_DEFAULT_TARGET;
static volatile uint16_t s_heater_pwm = 0;

/* 上一帧读数 */
static float s_gyro[3]  = {0};
static float s_accel[3] = {0};
static float s_temp     = 0.0f;

/* 温度首次达标标志：达标前满功率加热，达标后交给 PID。
 * 与例程的 first_temperate 同名同义。 */
static uint8_t s_first_temperate = 0U;

#define RAD_TO_DEG  57.29577951308232f

/* ============================ 内部辅助 ============================ */

/**
 * @brief 四元数 -> 欧拉角（deg）
 * @note  采用与例程 get_angle() 相同的公式，保证符号约定一致：
 *        roll  向右倾为正；pitch 抬头为正；yaw 逆时针为正。
 *        六轴下 yaw 会漂移，属预期。
 */
static void quat_to_euler(const float qq[4], float *roll, float *pitch, float *yaw)
{
    float w = qq[0], x = qq[1], y = qq[2], z = qq[3];
    float v;

    /* yaw (Z) */
    *yaw = atan2f(2.0f * (w * z + x * y), 2.0f * (w * w + x * x) - 1.0f);

    /* pitch (Y)。asin 输入钳位，避免浮点误差越界产生 NaN */
    v = -2.0f * (x * z - w * y);
    if (v > 1.0f)  v = 1.0f;
    if (v < -1.0f) v = -1.0f;
    *pitch = asinf(v);

    /* roll (X) */
    *roll = atan2f(2.0f * (w * x + y * z), 2.0f * (w * w + z * z) - 1.0f);

    *yaw   *= RAD_TO_DEG;
    *pitch *= RAD_TO_DEG;
    *roll  *= RAD_TO_DEG;
}

static void publish_snapshot(void)
{
    ins_snapshot_t tmp;

    quat_to_euler(q, &tmp.roll, &tmp.pitch, &tmp.yaw);
    tmp.gyro_x = s_gyro[0];
    tmp.gyro_y = s_gyro[1];
    tmp.gyro_z = s_gyro[2];
    tmp.accel_x = s_accel[0];
    tmp.accel_y = s_accel[1];
    tmp.accel_z = s_accel[2];
    tmp.temperature = s_temp;

    /* 四元数**原样上抛**，不做任何转换。
     * 理由：① 没有万向节死锁（欧拉角的 pitch 被 asin 限死在 ±90°）；
     *       ② 信息完整，上位机可自行换算成任意表示；
     *       ③ ROS2 原生就是四元数，省掉节点里的一次转换。
     * 内部 q[] 由 MahonyAHRSupdateIMU 维护，约定为 (w,x,y,z)、机体系→世界系
     * （依据：其更新式为右乘 q̇ = 0.5·q⊗ω_body）。 */
    tmp.quat_w = q[0];
    tmp.quat_x = q[1];
    tmp.quat_y = q[2];
    tmp.quat_z = q[3];

    s_snap_seq++;                    /* -> 奇数，标志"正在写" */
    __DMB();
    s_snap = tmp;
    __DMB();
    s_snap_seq++;                    /* -> 偶数，写完 */
}

/**
 * @brief 依据当前状态位选择下一个要发起的 SPI 通道
 * @note  与原例程 imu_cmd_spi_dma() 等价。三个通道互斥（不能同时占用 SPI），
 *        优先级 gyro > accel > accel_temp。
 *        并发保护：靠 DMA 的 EN 位判断总线是否空闲，而非关中断。
 */
static void imu_cmd_spi_dma(void)
{
    if (hspi1.hdmatx == NULL || hspi1.hdmarx == NULL)
    {
        return;
    }

    /* SPI 总线必须空闲：收发 DMA 都未使能 */
    if ((hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) || (hspi1.hdmarx->Instance->CR & DMA_SxCR_EN))
    {
        return;
    }

    /* 陀螺通道 */
    if ((gyro_update_flag & (1U << IMU_DR_SHFITS)) &&
        !(accel_update_flag & (1U << IMU_SPI_SHFITS)) &&
        !(accel_temp_update_flag & (1U << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= (uint8_t)~(1U << IMU_DR_SHFITS);
        gyro_update_flag |= (uint8_t)(1U << IMU_SPI_SHFITS);

        BMI088_GYRO_NS_L();
        SPI1_DMA_enable((uint32_t)gyro_tx_buf, (uint32_t)gyro_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }

    /* 加速度计通道 */
    if ((accel_update_flag & (1U << IMU_DR_SHFITS)) &&
        !(gyro_update_flag & (1U << IMU_SPI_SHFITS)) &&
        !(accel_temp_update_flag & (1U << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= (uint8_t)~(1U << IMU_DR_SHFITS);
        accel_update_flag |= (uint8_t)(1U << IMU_SPI_SHFITS);

        BMI088_ACCEL_NS_L();
        SPI1_DMA_enable((uint32_t)accel_tx_buf, (uint32_t)accel_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    /* 温度通道（与加速度计共用 CS1_ACCEL） */
    if ((accel_temp_update_flag & (1U << IMU_DR_SHFITS)) &&
        !(gyro_update_flag & (1U << IMU_SPI_SHFITS)) &&
        !(accel_update_flag & (1U << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= (uint8_t)~(1U << IMU_DR_SHFITS);
        accel_temp_update_flag |= (uint8_t)(1U << IMU_SPI_SHFITS);

        BMI088_ACCEL_NS_L();
        SPI1_DMA_enable((uint32_t)accel_temp_tx_buf, (uint32_t)accel_temp_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

/* ============================ EXTI 回调（核心链路） ============================
 * 由 HAL_GPIO_EXTI_IRQHandler() 在 EXTI4/EXTI9_5/EXTI0 中断里调用。
 * ⚠️ 这是整个采集链路唯一的入口，缺了它数据永远不会更新。 */
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if (GPIO_Pin == INT1_ACCEL_Pin)
    {
        /* 加速度计与温度共用 DRDY：一次中断同时置两个通道的 DR 位 */
        accel_update_flag      |= (uint8_t)(1U << IMU_DR_SHFITS);
        accel_temp_update_flag |= (uint8_t)(1U << IMU_DR_SHFITS);

        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == INT1_GYRO_Pin)
    {
        gyro_update_flag |= (uint8_t)(1U << IMU_DR_SHFITS);

        if (imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if (GPIO_Pin == GPIO_PIN_0)
    {
        /* EXTI0 由 DMA 完成中断软触发，用来唤醒 InsTask */
        if ((xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) && (s_ins_task_handle != NULL))
        {
            BaseType_t xHigherPriorityTaskWoken = pdFALSE;
            vTaskNotifyGiveFromISR(s_ins_task_handle, &xHigherPriorityTaskWoken);
            portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
        }
    }
}

/**
 * @brief  DMA2_Stream2 传输完成：SPI1 RX 结束
 * @note   放在本文件而非 stm32f4xx_it.c，与例程一致——
 *         因为这里要直接访问采集状态位，耦合度高，集中在一处更清晰。
 *         注意：**不能**调 HAL_DMA_IRQHandler()，因为 SPI 收发是裸寄存器启动的，
 *         HAL 句柄的 State 仍为 READY，HAL 会认定"无传输"直接返回，
 *         导致状态机永不推进（这正是采集卡死的典型坑）。
 */
void DMA2_Stream2_IRQHandler(void)
{
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) == RESET)
    {
        return;
    }
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

    /* 陀螺收完 */
    if (gyro_update_flag & (1U << IMU_SPI_SHFITS))
    {
        gyro_update_flag &= (uint8_t)~(1U << IMU_SPI_SHFITS);
        gyro_update_flag |= (uint8_t)(1U << IMU_UPDATE_SHFITS);
        BMI088_GYRO_NS_H();
    }

    /* 加速度计收完 */
    if (accel_update_flag & (1U << IMU_SPI_SHFITS))
    {
        accel_update_flag &= (uint8_t)~(1U << IMU_SPI_SHFITS);
        accel_update_flag |= (uint8_t)(1U << IMU_UPDATE_SHFITS);
        BMI088_ACCEL_NS_H();
    }

    /* 温度收完 */
    if (accel_temp_update_flag & (1U << IMU_SPI_SHFITS))
    {
        accel_temp_update_flag &= (uint8_t)~(1U << IMU_SPI_SHFITS);
        accel_temp_update_flag |= (uint8_t)(1U << IMU_UPDATE_SHFITS);
        BMI088_ACCEL_NS_H();
    }

    /* 立刻接着发下一个通道 */
    imu_cmd_spi_dma();

    /* 任一通道就绪即唤醒任务。
     * 这里做了相对例程的加固：例程只在 gyro 的 UPDATE 位触发，
     * 若陀螺 DRDY 断掉（SPI 过快/PC5 接触不良/BMI088 异常），
     * 即使温度通道正常也永远唤不醒任务 → 温控停摆。
     * 改为三通道任一 UPDATE 都触发，保证温度链路能独立维持。 */
    if ((gyro_update_flag | accel_update_flag | accel_temp_update_flag) & (1U << IMU_UPDATE_SHFITS))
    {
        __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
    }
}

/**
 * @brief  DMA2_Stream3（SPI1 TX）完成中断
 * @note   裸寄存器启动的 DMA，TX 侧无需额外处理，清标志即可。
 *         仍保留处理函数以满足向量表。 */
void DMA2_Stream3_IRQHandler(void)
{
    if (__HAL_DMA_GET_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx)) == RESET)
    {
        return;
    }
    __HAL_DMA_CLEAR_FLAG(hspi1.hdmatx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmatx));
}

/* ============================ EXTI 中断入口 ============================
 * 这三个中断与采集链路同源，统一放在此处（stm32f4xx_it.c 里已说明原因）。
 * HAL_GPIO_EXTI_IRQHandler 会清 EXTI 挂起位并回调 HAL_GPIO_EXTI_Callback。 */

/* INT1_ACCEL = PC4 → EXTI4：加速度计/温度数据就绪 */
void EXTI4_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(INT1_ACCEL_Pin);
}

/* INT1_GYRO = PC5 → EXTI9_5：陀螺数据就绪 */
void EXTI9_5_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(INT1_GYRO_Pin);
}

/* EXTI0：DMA 完成中断软触发，用于唤醒 InsTask。
 * 注意用 HAL_GPIO_EXTI_IRQHandler 而不是直接调 Callback —— 前者会清挂起位，
 * 后者不会，漏清会导致中断反复重入。 */
void EXTI0_IRQHandler(void)
{
    HAL_GPIO_EXTI_IRQHandler(GPIO_PIN_0);
}

/* ============================ 恒温控制 ============================ */

/**
 * @brief BMI088 恒温控制（与例程 imu_temp_control 逻辑等价）
 * @note  达标前满功率加热；首次达到目标温度后把积分项预置为半量程加速收敛，
 *        再交给 PID。同时带 20s 时间兜底，避免"永远达不到目标温度就永远满功率"。
 */
static void imu_temp_control(float temp)
{
    static uint16_t temp_constant_time = 0U;
    static uint32_t boot_tick = 0U;
    uint32_t now = xTaskGetTickCount();

    if (boot_tick == 0U)
    {
        boot_tick = now;
    }

    if (s_first_temperate)
    {
        PID_calc(&s_heater_pid, temp, s_target_temp);
        if (s_heater_pid.out < 0.0f)
        {
            s_heater_pid.out = 0.0f;
        }
        s_heater_pwm = (uint16_t)s_heater_pid.out;
        imu_pwm_set(s_heater_pwm);
        return;
    }

    /* ---- 尚未达标：满功率加热 ---- */
    if (temp > s_target_temp)
    {
        if (++temp_constant_time > INS_TEMP_CONFIRM_CNT)
        {
            s_first_temperate = 1U;
            s_heater_pid.Iout = HEATER_PID_MAX_OUT / 2.0f;
        }
    }
    else
    {
        temp_constant_time = 0U;

        /* 时间兜底：按调用次数累加在 accel_temp 更新率不稳定时会失效，
         * 所以改用 tick 判据。 */
        if ((now - boot_tick) * portTICK_PERIOD_MS > INS_TEMP_BOOT_TIMEOUT_MS)
        {
            s_first_temperate = 1U;
            s_heater_pid.Iout = HEATER_PID_MAX_OUT / 2.0f;
        }
    }

    s_heater_pwm = (uint16_t)(HEATER_PID_MAX_OUT - 1.0f);
    imu_pwm_set(s_heater_pwm);
}

/* ============================ 任务主体 ============================ */

uint8_t ins_init(void)
{
    uint8_t err;

    /* DWT 计数器用于微秒级忙等（BMI088 时序要求） */
    bsp_dwt_init();

    vTaskDelay(pdMS_TO_TICKS(INS_TASK_INIT_TIME));

    err = BMI088_init();
    if (err != BMI088_NO_ERROR)
    {
        s_ins_status = INS_STATUS_ERROR;
        return err;
    }

    /* 读一次原始数据（初始化自检，同时为 Mahony 提供初值参考） */
    BMI088_read(s_gyro, s_accel, &s_temp);

    /* MahonyAHRS：按实际采样率设置 */
    MahonyAHRS_setSampleFreq(1000.0f / (float)INS_SAMPLE_PERIOD_MS);
    MahonyAHRS_reset();
    q[0] = 1.0f; q[1] = q[2] = q[3] = 0.0f;

    /* 恒温 PID */
    PID_init(&s_heater_pid, PID_POSITION, s_heater_pid_param,
             HEATER_PID_MAX_OUT, HEATER_PID_MAX_IOUT);

    imu_pwm_ensure_started();
    imu_pwm_set(0);

    /* 提高 SPI 时钟：初始化阶段用的 256 分频太慢（84MHz/256 ≈ 328kHz），
     * 采样时改为 8 分频 ≈ 10.5MHz（BMI088 SPI 上限 10MHz，例程同款取值）。 */
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    if (HAL_SPI_Init(&hspi1) != HAL_OK)
    {
        s_ins_status = INS_STATUS_ERROR;
        return 1U;
    }

    /* 配置 DMA 通道（外设地址 + 接收缓冲 + 计数器） */
    SPI1_DMA_init((uint32_t)gyro_tx_buf, (uint32_t)gyro_rx_buf, SPI_DMA_GYRO_LENGHT);

    s_ins_task_handle = xTaskGetCurrentTaskHandle();

    /* ⚠️ imu_start_dma_flag 推迟到预热循环之后再置 1（见下方）。
     * 预热阶段用的是**阻塞式**读温度（BMI088_read_write_byte → HAL_SPI_TransmitReceive），
     * 而 DRDY→DMA 走的是**裸寄存器自管**的 SPI1 收发。两者共用同一个 SPI1，
     * 若在预热期就放行 DMA：两边同时写 SPI1->DR、同时抢 RXNE、同时切换 CS，
     * 结果是温度读数变脏（PID 永远判不到达标，每次上电都满功率烧满 20s 超时），
     * DMA 侧收到的是被偷走字节的残帧，CS 也可能被另一条路径打断。
     * 例程没有预热循环（温控就在主循环里做），所以它能"初始化完立刻放行"；
     * 本工程插了预热循环，就必须把放行点后移到循环之后，
     * 以保持"进入 notify 等待循环前才放行 DMA"这一等价语义。 */

    /* 等温度首次达标（最长 20s，超时照样放行） */
    s_ins_status = INS_STATUS_WARMUP;
    {
        uint32_t t0 = xTaskGetTickCount();
        uint32_t elapsed_ms;

        while (!s_first_temperate)
        {
            /* 加热期间也需要读温度，但数据流依赖 DRDY 中断；
             * 这里直接用阻塞式读温度，简单可靠（加热阶段对实时性无要求）。 */
            float t = get_BMI088_temperate();
            imu_temp_control(t);

            /* 独立超时兜底：不依赖 imu_temp_control() 内部是否置位成功。
             * 曾出现"永久停在 WARMUP、灯一直蓝绿呼吸"的现象——只要内部
             * 达标判据因任何原因失效（目标温度被改、温度换算异常、确认
             * 计数被反复清零），这里就会无限等下去，整个系统卡死。
             * 本工程的 INS_TEMP_BOOT_TIMEOUT_MS 单位是 ms，tick 率 1kHz，
             * 故用 pdMS_TO_TICKS 换算后比较，避免手动乘 portTICK_PERIOD_MS
             * 这种易错写法。 */
            elapsed_ms = (uint32_t)((xTaskGetTickCount() - t0) * portTICK_PERIOD_MS);
            if (elapsed_ms > INS_TEMP_BOOT_TIMEOUT_MS)
            {
                s_first_temperate = 1U;
                s_heater_pid.Iout = HEATER_PID_MAX_OUT / 2.0f;
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    /* 预热结束，阻塞读全部退场：现在才允许 DRDY 中断发起 DMA。
     * 此后数据流完全由硬件 DRDY 驱动，任务只在 ulTaskNotifyTake 上等待，
     * 不再主动轮询发起 —— 与例程语义一致。 */
    imu_start_dma_flag = 1U;

    s_ins_status = INS_STATUS_RUNNING;
    return 0;
}

/**
 * @brief 姿态任务主循环
 */
void InsTask_Entry(void *argument)
{
    (void)argument;

    if (ins_init() != 0)
    {
        /* 初始化失败：任务保留但空转，不阻塞其他任务 */
        for (;;)
        {
            vTaskDelay(pdMS_TO_TICKS(100));
        }
    }

    for (;;)
    {
        /* 等 DRDY 链路把数据收齐（DMA 完成中断 → EXTI0 → 通知） */
        if (ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(20)) == 0U)
        {
            /* 20ms 超时：DRDY 可能断了。这里不报错，继续等，
             * 状态位由状态区暴露给上位机观察。 */
            continue;
        }

        if (gyro_update_flag & (1U << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= (uint8_t)~(1U << IMU_UPDATE_SHFITS);
            BMI088_gyro_read_over(gyro_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, s_gyro);
        }

        if (accel_update_flag & (1U << IMU_UPDATE_SHFITS))
        {
            accel_update_flag &= (uint8_t)~(1U << IMU_UPDATE_SHFITS);
            BMI088_accel_read_over(accel_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, s_accel, NULL);
        }

        if (accel_temp_update_flag & (1U << IMU_UPDATE_SHFITS))
        {
            accel_temp_update_flag &= (uint8_t)~(1U << IMU_UPDATE_SHFITS);
            BMI088_temperature_read_over(accel_temp_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &s_temp);
            imu_temp_control(s_temp);
        }

        /* 六轴姿态解算：用加速度计修正 roll/pitch，yaw 由陀螺积分（会漂） */
        MahonyAHRSupdateIMU(q, s_gyro[0], s_gyro[1], s_gyro[2],
                               s_accel[0], s_accel[1], s_accel[2]);

        publish_snapshot();
    }
}

/* ============================ 对外接口 ============================ */

uint8_t INS_get_snapshot(ins_snapshot_t *out)
{
    uint32_t s1, s2;

    if (out == NULL)
    {
        return 0;
    }

    s1 = s_snap_seq;
    if (s1 & 1u)          /* 奇数说明正在写，直接放弃 */
    {
        return 0;
    }
    __DMB();
    *out = s_snap;
    __DMB();
    s2 = s_snap_seq;

    return (s1 == s2) ? 1 : 0;
}

ins_status_e ins_get_status(void)
{
    return s_ins_status;
}

void ins_set_target_temp(float temp)
{
    if (temp >= 20.0f && temp <= 70.0f)   /* 合理区间保护 */
    {
        s_target_temp = temp;
    }
}

float ins_get_target_temp(void)
{
    return s_target_temp;
}

uint16_t ins_get_heater_pwm(void)
{
    return s_heater_pwm;
}
