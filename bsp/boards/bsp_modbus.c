
#include "bsp_modbus.h"

#include "main.h"
#include "usart.h"
#include "bsp_fric.h"
#include "bsp_pulse.h"
#include "ins_task.h"

/* CAN 发送等待邮箱时需要 vTaskDelay() 让出 CPU，见 motor_can_send_frame() */
#include "FreeRTOS.h"
#include "task.h"

#include <string.h>
#include <math.h>

#define MODBUS_SLAVE_ADDR 0x01U
#define MODBUS_RX_RING_SIZE 128U
/* 保持寄存器总数。
 * 0x0000~0x00BF 与 running（Core/Inc/registers.h, REG_HOLDING_COUNT=256）逐地址一致：
 *   0x0000~0x003F 控制区 / 0x0040~0x008F CAN 电机状态 / 0x0090~0x00BF 参数区
 * 0x00C0~0x00FF 为 running 的保留区，本工程同样留空。
 * 0x0100 起是本工程后加的风机区，必须扩到 512 才落得下——
 * 若仍是 256，风机占空比 0x0100 会被下面的 addr >= MODBUS_REG_COUNT 直接拒绝，
 * 表现是上位机写 40257~40260 静默无效。 */
#define MODBUS_REG_COUNT 512U
#define MODBUS_RTU_TIMEOUT_MS 20U

#define FLASH_RETENTIVE_A_ADDR 0x080C0000UL
#define FLASH_RETENTIVE_B_ADDR 0x080E0000UL
#define FLASH_RETENTIVE_MAGIC 0x524D4352UL
#define FLASH_RETENTIVE_VERSION 1UL
#define FLASH_RETENTIVE_PARAM_COUNT 48U
#define FLASH_RETENTIVE_BLOCK_SIZE 112U

/* 参考 running 项目的 Modbus 地址规划；为避免与电机控制区冲突，风机占空比控制区被挪到 0x0100 以后。
 * 0x0000~0x0019 : CAN 电机控制区（保持与 running 完全一致）
 * 0x0040~0x008F : CAN 电机状态区（只读）
 * 0x0090~0x00BF : 参数区
 * 0x0100~0x0103 : 风机占空比控制区（四个风机）
 * 0x00C0~0x00FF : 保留区
 */
#define REG_MOTOR_EN_BASE 0x0000U
#define REG_MOTOR_CLR_BASE 0x0004U
#define REG_MOTOR_VEL_BASE 0x000AU
#define REG_MOTOR_VY_HI 0x0014U
#define REG_MOTOR_VX_HI 0x0016U
#define REG_MOTOR_WZ_HI 0x0018U
#define REG_MOTOR_CMD_COUNT 6U
#define REG_MOTOR_STATUS_M1 0x0064U
#define REG_MOTOR_STATUS_M2 0x006EU
#define REG_MOTOR_STATUS_M3 0x0078U
#define REG_MOTOR_STATUS_M4 0x0082U
#define REG_MOTOR_STATUS_STRIDE 10U
#define REG_MOTOR_TX_ID_BASE 0x201U

/* 达妙速度模式（SPD_MODE = 0x200）下，使能/失能/清错/速度四类命令共用同一个
 * CAN ID = 0x200 + 电机ID = 0x201~0x204，靠数据域的最后一字节区分（FC/FD/FB）。
 * 与 running 的 c_cmdEnable/c_cmdDisable/c_cmdClearErr 一致（Core/Src/freertos.c:83），
 * 取自厂家 Doc/dm_motor_drv.c 的 enable_motor_mode() / disable_motor_mode()。 */
static const uint8_t c_cmd_enable[8]    = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFCU};
static const uint8_t c_cmd_disable[8]   = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFDU};
static const uint8_t c_cmd_clear_err[8] = {0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFFU, 0xFBU};

#define REG_MOTOR_COUNT 4U
#define REG_MOTOR_W_MAX 30.0f
#define REG_MOTOR_LX 0.15f
#define REG_MOTOR_LY 0.15f
#define REG_MOTOR_R 0.05f
/* 发送邮箱满时的等待上限（tick）。configTICK_RATE_HZ = 1000，即 5ms，
 * 与 running CanTask_Send() 中 "osDelay(1) 最多 5 次" 一致 */
#define MOTOR_CAN_TX_WAIT_TICKS 5U
#define REG_FAN_DUTY_BASE 0x0100U
#define REG_FAN_CNT 4U
/* 风机状态区：本工程后加，独立放在 0x0110 起，**不得**占用 running 的电机状态区
 * （0x0064/0x006E/0x0078/0x0082）。每台风機 10 个寄存器，布局沿用电机状态语义：
 * +0 ERR / +1 保留 / +2+3 POS / +4+5 VEL / +6+7 T / +8 T_MOS / +9 T_Rotor */
#define REG_FAN_STATUS_BASE 0x0110U
#define REG_FAN_STATUS_STRIDE 10U
#define REG_CAN_STATUS_BASE 0x0040U
#define REG_CAN_STATUS_END 0x008FU
#define REG_STATUS_STRIDE 10U
#define REG_PARAM_BASE 0x0090U
#define REG_PARAM_END 0x00BFU
#define REG_PARAM_UART_CFG 0x0090U
#define REG_PARAM_SLAVE_ADDR 0x0091U

/* ==================== 风机自动模式（姿态前馈）参数区 0x0140~0x014F ====================
 * 风机占空比不再只由外部写 0x0100 决定，而是可以由 InsTask 依据 IMU 姿态自动生成。
 * 控制律（重力分量补偿开环前馈）：
 *     duty = DUTY_FLAT + SLOPE_GAIN * (1 - cos(theta))
 * 其中 theta 由 roll/pitch 合成（见 fan_auto_tick）。物理含义：
 * 斜面夹角越大，重力沿坡分量越大，需要更大吸附力防止打滑，故 duty 单调递增。
 * 这些寄存器全部读写，支持上位机在线调参（便于现场"调试占空比曲线"）。 */
#define REG_AUTO_BASE          0x0140U
#define REG_AUTO_END           0x014FU
#define REG_AUTO_FAN_MODE      0x0140U  /* 0=手动(跟随 0x0100) 1=自动(姿态前馈) */
#define REG_AUTO_FAN_AUTO_EN   0x0141U  /* 自动模式总开关，0=强制手动 */
#define REG_AUTO_DUTY_FLAT     0x0142U  /* 水平姿态基准占空比(%)，建议 >=30 */
#define REG_AUTO_DUTY_MIN      0x0143U  /* 自动输出下限(%)，**必须 > 0**，建议 >=30 */
#define REG_AUTO_DUTY_MAX      0x0144U  /* 自动输出上限(%) */
#define REG_AUTO_SLOPE_GAIN    0x0145U  /* 坡度增益(每单位 (1-cosθ) 增加的占空比%) */
#define REG_AUTO_KP_PITCH      0x0146U  /* 俯仰前馈增益（当前实现并入 SLOPE_GAIN，预留） */
#define REG_AUTO_KP_ROLL       0x0147U  /* 横滚前馈增益（当前实现并入 SLOPE_GAIN，预留） */
#define REG_AUTO_PITCH_OFFSET  0x0148U  /* 俯仰零位偏置(deg)，安装误差补偿 */
#define REG_AUTO_ROLL_OFFSET   0x0149U  /* 横滚零位偏置(deg)，安装误差补偿 */
#define REG_AUTO_MAG_ENABLE    0x014AU  /* 磁力计使能：本轮恒 0（只读，写无效） */
#define REG_AUTO_MAG_STATUS    0x014BU  /* 磁力计状态：本轮恒 2=未接入（只读） */
#define REG_AUTO_MAG_CAL_CMD   0x014CU  /* 磁校准命令：保留，本轮无实现 */
#define REG_AUTO_HEATER_TARGET 0x014DU  /* 恒温目标温度(℃)，float32 需占 2 个寄存器到 0x014E */
/* 0x014F 预留 */

/* ==================== IMU 姿态输出区 0x0150~0x015F（只读） ====================
 * 每个量为 float32，高字在前（与电机状态区同序），占 2 个寄存器。
 * yaw 在六轴模式下会缓慢漂移，本轮仅作占位输出。 */
#define REG_IMU_BASE           0x0150U
#define REG_IMU_END            0x015FU
#define REG_IMU_ROLL           0x0150U  /* deg */
#define REG_IMU_PITCH          0x0152U  /* deg */
#define REG_IMU_YAW            0x0154U  /* deg（六轴，会漂移，仅占位） */
#define REG_IMU_GYRO_X         0x0156U  /* rad/s */
#define REG_IMU_GYRO_Y         0x0158U  /* rad/s */
#define REG_IMU_GYRO_Z         0x015AU  /* rad/s */
#define REG_IMU_AUTO_DUTY      0x015CU  /* 自动模式下当前计算的占空比(%) */
#define REG_IMU_STATUS         0x015EU  /* 0=离线 1=加热中 2=运行 3=错误 */

static volatile uint8_t s_rx_ring[MODBUS_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;
static uint8_t s_rx_byte = 0U;
static uint16_t s_holding_regs[MODBUS_REG_COUNT];
static uint8_t s_param_dirty = 0U;
static uint32_t s_param_dirty_tick_ms = 0U;
static uint32_t s_retentive_seq = 0U;
static uint8_t s_retentive_bank = 0U;

typedef struct
{
    uint32_t magic;
    uint32_t version;
    uint32_t seq;
    uint32_t crc;
    uint16_t values[FLASH_RETENTIVE_PARAM_COUNT];
} retentive_block_t;

static void motor_can_poll_rx(void);

static uint16_t crc16_update(uint16_t crc, uint8_t byte)
{
    crc ^= (uint16_t)byte;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        if ((crc & 0x0001U) != 0U)
        {
            crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
        }
        else
        {
            crc >>= 1U;
        }
    }
    return crc;
}

static uint16_t crc16_buffer(const uint8_t *buf, uint16_t len)
{
    uint16_t crc = 0xFFFFU;
    for (uint16_t i = 0U; i < len; ++i)
    {
        crc = crc16_update(crc, buf[i]);
    }
    return crc;
}

static uint32_t crc32_update(uint32_t crc, uint8_t byte)
{
    crc ^= (uint32_t)byte;
    for (uint8_t i = 0U; i < 8U; ++i)
    {
        if ((crc & 0x00000001UL) != 0UL)
        {
            crc = (crc >> 1U) ^ 0xEDB88320UL;
        }
        else
        {
            crc >>= 1U;
        }
    }
    return crc;
}

static uint32_t crc32_buffer(const uint8_t *buf, uint32_t len)
{
    uint32_t crc = 0xFFFFFFFFUL;
    for (uint32_t i = 0U; i < len; ++i)
    {
        crc = crc32_update(crc, buf[i]);
    }
    return (crc ^ 0xFFFFFFFFUL);
}

static uint16_t ring_len(void)
{
    if (s_rx_head >= s_rx_tail)
    {
        return (uint16_t)(s_rx_head - s_rx_tail);
    }
    return (uint16_t)(MODBUS_RX_RING_SIZE - s_rx_tail + s_rx_head);
}

static uint8_t ring_pop(uint8_t *out_byte)
{
    if (s_rx_head == s_rx_tail)
    {
        return 0U;
    }
    *out_byte = s_rx_ring[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % MODBUS_RX_RING_SIZE);
    return 1U;
}

static void ring_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % MODBUS_RX_RING_SIZE);
    if (next == s_rx_tail)
    {
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % MODBUS_RX_RING_SIZE);
    }
    s_rx_ring[s_rx_head] = byte;
    s_rx_head = next;
}

static void rs485_set_tx(uint8_t enable)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static void modbus_send_frame(const uint8_t *frame, uint16_t len)
{
    rs485_set_tx(1U);
    HAL_UART_Transmit(&huart2, (uint8_t *)frame, len, MODBUS_RTU_TIMEOUT_MS);
    rs485_set_tx(0U);
}

static uint16_t clamp_duty(uint16_t v)
{
    if (v > 100U)
    {
        return 100U;
    }
    return v;
}

static void write_float32_le(uint16_t *regs, uint16_t addr, float value)
{
    union
    {
        float f;
        uint32_t u32;
    } conv;

    conv.f = value;
    regs[addr] = (uint16_t)((conv.u32 >> 16U) & 0xFFFFU);
    regs[(uint16_t)(addr + 1U)] = (uint16_t)(conv.u32 & 0xFFFFU);
}

static uint8_t is_can_status_region(uint16_t addr)
{
    return (addr >= REG_CAN_STATUS_BASE) && (addr <= REG_CAN_STATUS_END);
}

static uint8_t is_param_region(uint16_t addr)
{
    return (addr >= REG_PARAM_BASE) && (addr <= REG_PARAM_END);
}

static uint8_t is_auto_param_region(uint16_t addr)
{
    return (addr >= REG_AUTO_BASE) && (addr <= REG_AUTO_END);
}

/* IMU 姿态输出区为只读：上位机写这些地址一律静默忽略（不返回异常码，
 * 保持与既有状态区 0x0040~0x008F 相同的"写无效但不报错"行为，便于上位机统一处理）。 */
static uint8_t is_imu_status_region(uint16_t addr)
{
    return (addr >= REG_IMU_BASE) && (addr <= REG_IMU_END);
}

static uint8_t get_modbus_slave_addr(void)
{
    uint16_t addr = s_holding_regs[REG_PARAM_SLAVE_ADDR];
    if ((addr == 0U) || (addr > 247U))
    {
        return MODBUS_SLAVE_ADDR;
    }
    return (uint8_t)addr;
}

static void refresh_fan_state_registers(void)
{
    for (uint16_t i = 0U; i < REG_FAN_CNT; ++i)
    {
        /* 风机状态写自己的区段，绝不碰电机状态区（running 地址 0x0064 起） */
        uint16_t status_base = (uint16_t)(REG_FAN_STATUS_BASE + i * REG_FAN_STATUS_STRIDE);
        uint16_t duty = clamp_duty(s_holding_regs[REG_FAN_DUTY_BASE + i]);
        uint32_t rpm = pulse_get_rpm((uint8_t)(i + 1U), 2U);
        uint32_t pulse_cnt = pulse_get_pulse_count((uint8_t)(i + 1U));

        /* 字段布局沿用与电机状态区相同的 10 寄存器格式（上位机可复用解析），
         * 但地址在风机独立区 0x0110 起，与 running 的电机状态区 0x0064 起互不重叠：
         * +0 ERR    : 运行标志（占空比 > 0 为 1）
         * +1 保留   : 0
         * +2/+3 POS : FG 脉冲累计计数
         * +4/+5 VEL : 实测转速 RPM
         * +6/+7 T   : 当前输出占空比 0~100
         * +8 T_MOS  : 估算 MOS 温度
         * +9 T_Rotor: 估算转子温度
         */
        s_holding_regs[status_base + 0U] = (uint16_t)(duty > 0U ? 1U : 0U);
        s_holding_regs[status_base + 1U] = 0U;
        write_float32_le(s_holding_regs, status_base + 2U, (float)pulse_cnt);
        write_float32_le(s_holding_regs, status_base + 4U, (float)rpm);
        write_float32_le(s_holding_regs, status_base + 6U, (float)duty);
        s_holding_regs[status_base + 8U] = (uint16_t)(duty * 2U);
        s_holding_regs[status_base + 9U] = (uint16_t)(duty * 3U);
    }
}

static void sync_fan_outputs_from_regs(void)
{
    uint16_t fan1 = clamp_duty(s_holding_regs[REG_FAN_DUTY_BASE + 0U]);
    uint16_t fan2 = clamp_duty(s_holding_regs[REG_FAN_DUTY_BASE + 1U]);
    uint16_t fan3 = clamp_duty(s_holding_regs[REG_FAN_DUTY_BASE + 2U]);
    uint16_t fan4 = clamp_duty(s_holding_regs[REG_FAN_DUTY_BASE + 3U]);

    fric_set_channel_duty(1U, (uint8_t)fan1);
    fric_set_channel_duty(2U, (uint8_t)fan2);
    fric_set_channel_duty(3U, (uint8_t)((fan3 > fan4) ? fan3 : fan4));
    refresh_fan_state_registers();
}

/* ==================== 姿态前馈自动占空比 ====================
 * 控制律：duty = DUTY_FLAT + SLOPE_GAIN * (1 - cos(theta))
 *
 * 其中 theta 为车身相对水平面的总倾角，由 roll/pitch 合成：
 *     cos(theta) = cos(pitch) * cos(roll)
 * （两个方向的小角度旋转可解耦相乘，误差在 1% 量级内，对前馈足够）
 *
 * 为什么用 (1 - cosθ) 而不是 θ：
 *   - 重力沿坡分量 ∝ sinθ，而吸附力需求增量与"损失的法向分量"∝ (1-cosθ) 同阶；
 *   - (1-cosθ) 在 θ=0 处导数为 0，水平附近天然平滑，不会因姿态噪声抖动；
 *   - 有界于 [0,1]，增益物理意义直观：SLOPE_GAIN 就是"竖直墙面时额外加多少点"。
 *
 * 注意：这是开环前馈，roll/pitch 的作用是"告知坡度"，不是闭环反馈。
 *       实际吸附效果还需靠 DUTY_MIN 兜底 + 现场标定 SLOPE_GAIN。 */
static uint16_t fan_auto_duty = 0U;

static void fan_auto_update(void)
{
    ins_snapshot_t snap;
    float roll_deg, pitch_deg;
    float cos_theta;
    float duty_f;
    uint16_t duty_flat, duty_min, duty_max, slope_gain;
    uint16_t out;

    /* 只有"自动模式 + 总开关打开"才接管；否则沿用 0x0100 的手动值 */
    if ((s_holding_regs[REG_AUTO_FAN_MODE] == 0U) ||
        (s_holding_regs[REG_AUTO_FAN_AUTO_EN] == 0U))
    {
        fan_auto_duty = 0U;
        return;
    }

    /* 读姿态快照：失败说明正在写，沿用上一轮结果，不阻塞 */
    if (!INS_get_snapshot(&snap))
    {
        return;
    }

    duty_flat  = s_holding_regs[REG_AUTO_DUTY_FLAT];
    duty_min   = s_holding_regs[REG_AUTO_DUTY_MIN];
    duty_max   = s_holding_regs[REG_AUTO_DUTY_MAX];
    slope_gain = s_holding_regs[REG_AUTO_SLOPE_GAIN];

    /* 零位偏置：补偿 IMU 安装误差 */
    pitch_deg = snap.pitch - (float)(int16_t)s_holding_regs[REG_AUTO_PITCH_OFFSET];
    roll_deg  = snap.roll  - (float)(int16_t)s_holding_regs[REG_AUTO_ROLL_OFFSET];

    /* 转弧度后合成总倾角余弦 */
    cos_theta = cosf(pitch_deg * 0.017453292f) * cosf(roll_deg * 0.017453292f);

    duty_f = (float)duty_flat + (float)slope_gain * (1.0f - cos_theta);

    /* 下限兜底：吸附力绝不能为零 */
    if (duty_f < (float)duty_min) duty_f = (float)duty_min;
    if (duty_f > (float)duty_max) duty_f = (float)duty_max;
    if (duty_f > 100.0f) duty_f = 100.0f;

    out = (uint16_t)duty_f;
    fan_auto_duty = out;

    /* 四路同值：风机 3/4 共用 PWM7，不做分轴 */
    fric_set_channel_duty(1U, (uint8_t)out);
    fric_set_channel_duty(2U, (uint8_t)out);
    fric_set_channel_duty(3U, (uint8_t)out);
}

/* 把姿态与自动占空比刷进 0x0150~0x015F 只读区 */
static void refresh_imu_registers(void)
{
    ins_snapshot_t snap;

    if (!INS_get_snapshot(&snap))
    {
        return;   /* 正在更新，下轮再来 */
    }

    write_float32_le(s_holding_regs, REG_IMU_ROLL,     snap.roll);
    write_float32_le(s_holding_regs, REG_IMU_PITCH,    snap.pitch);
    write_float32_le(s_holding_regs, REG_IMU_YAW,      snap.yaw);
    write_float32_le(s_holding_regs, REG_IMU_GYRO_X,   snap.gyro_x);
    write_float32_le(s_holding_regs, REG_IMU_GYRO_Y,   snap.gyro_y);
    write_float32_le(s_holding_regs, REG_IMU_GYRO_Z,   snap.gyro_z);
    write_float32_le(s_holding_regs, REG_IMU_AUTO_DUTY, (float)fan_auto_duty);
    s_holding_regs[REG_IMU_STATUS] = (uint16_t)ins_get_status();
}

static uint8_t retentive_read_block(uint32_t addr, retentive_block_t *out_block)
{
    const volatile retentive_block_t *flash_block = (const volatile retentive_block_t *)addr;
    uint32_t calc = 0U;

    if (flash_block->magic != FLASH_RETENTIVE_MAGIC)
    {
        return 0U;
    }
    if (flash_block->version != FLASH_RETENTIVE_VERSION)
    {
        return 0U;
    }

    calc = crc32_buffer((const uint8_t *)&flash_block->magic, (uint32_t)sizeof(retentive_block_t) - sizeof(flash_block->crc));
    if (calc != flash_block->crc)
    {
        return 0U;
    }

    *out_block = *flash_block;
    return 1U;
}

static uint8_t retentive_load_params(void)
{
    retentive_block_t bank_a = {0};
    retentive_block_t bank_b = {0};
    uint8_t valid_a = retentive_read_block(FLASH_RETENTIVE_A_ADDR, &bank_a);
    uint8_t valid_b = retentive_read_block(FLASH_RETENTIVE_B_ADDR, &bank_b);
    uint32_t max_seq = 0U;

    if (!valid_a && !valid_b)
    {
        return 0U;
    }

    if (valid_a && bank_a.seq > max_seq)
    {
        max_seq = bank_a.seq;
        s_retentive_bank = 0U;
        memcpy(&s_holding_regs[REG_PARAM_BASE], bank_a.values, sizeof(bank_a.values));
    }
    if (valid_b && bank_b.seq > max_seq)
    {
        max_seq = bank_b.seq;
        s_retentive_bank = 1U;
        memcpy(&s_holding_regs[REG_PARAM_BASE], bank_b.values, sizeof(bank_b.values));
    }
    s_retentive_seq = max_seq;
    return 1U;
}

static uint8_t retentive_commit_params(void)
{
    uint32_t base_addr = s_retentive_bank == 0U ? FLASH_RETENTIVE_B_ADDR : FLASH_RETENTIVE_A_ADDR;
    uint32_t sector = (base_addr == FLASH_RETENTIVE_A_ADDR) ? FLASH_SECTOR_10 : FLASH_SECTOR_11;
    FLASH_EraseInitTypeDef erase = {0};
    uint32_t error = 0U;
    retentive_block_t block = {0};
    uint8_t *block_bytes = (uint8_t *)&block;
    uint32_t *block_words = (uint32_t *)&block;

    block.magic = FLASH_RETENTIVE_MAGIC;
    block.version = FLASH_RETENTIVE_VERSION;
    block.seq = s_retentive_seq + 1U;
    memcpy(block.values, &s_holding_regs[REG_PARAM_BASE], sizeof(block.values));
    block.crc = crc32_buffer(block_bytes, (uint32_t)sizeof(retentive_block_t) - sizeof(block.crc));

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR | FLASH_FLAG_PGAERR | FLASH_FLAG_PGPERR | FLASH_FLAG_PGSERR);
    erase.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase.Banks = FLASH_BANK_1;
    erase.Sector = sector;
    erase.NbSectors = 1U;
    erase.VoltageRange = FLASH_VOLTAGE_RANGE_3;
    if (HAL_FLASHEx_Erase(&erase, &error) != HAL_OK)
    {
        HAL_FLASH_Lock();
        return 0U;
    }

    for (uint32_t i = 0U; i < (uint32_t)(sizeof(retentive_block_t) / sizeof(uint32_t)); ++i)
    {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, base_addr + i * 4U, block_words[i]) != HAL_OK)
        {
            HAL_FLASH_Lock();
            return 0U;
        }
    }

    s_retentive_bank = (s_retentive_bank == 0U) ? 1U : 0U;
    s_retentive_seq = block.seq;
    s_param_dirty = 0U;
    HAL_FLASH_Lock();
    return 1U;
}

static void retentive_poll(void)
{
    if (!s_param_dirty)
    {
        return;
    }
    if ((HAL_GetTick() - s_param_dirty_tick_ms) < 2000U)
    {
        return;
    }
    retentive_commit_params();
}

static float read_float32_regs(const uint16_t *regs, uint16_t addr)
{
    union
    {
        float f;
        uint32_t u32;
    } conv;

    conv.u32 = ((uint32_t)regs[addr] << 16U) | regs[(uint16_t)(addr + 1U)];
    return conv.f;
}

static void write_float32_regs(uint16_t *regs, uint16_t addr, float value)
{
    union
    {
        float f;
        uint32_t u32;
    } conv;

    conv.f = value;
    regs[addr] = (uint16_t)((conv.u32 >> 16U) & 0xFFFFU);
    regs[(uint16_t)(addr + 1U)] = (uint16_t)(conv.u32 & 0xFFFFU);
}

/* ===== 运动学：与 running 项目 Core/Src/kinematics.c 保持一致 =====
 *
 * 底盘构型：4 个正交全向轮
 *   LF(电机1) / RR(电机3) = X 轮，只吃 vx
 *   RF(电机2) / LR(电机4) = Y 轮，只吃 vy
 * 因此每个轮速只依赖 vx 或 vy 之一是**正确**的，不要按麦克纳姆模型去改。
 *
 * 调用时机：与 running 的 CanTask 一致 —— 每 10ms 发速度命令前调用一次，
 * 而不是上位机写寄存器时立即触发。这样 vx/vy/wz 必然来自同一时刻的快照，
 * 不会用到"半新半旧"的值。
 */
#define KIN_LINK_TIMEOUT_MS 500U

static volatile uint32_t s_kin_last_cmd_ms   = 0U;
static volatile uint8_t  s_kin_ever_received = 0U;

/* NaN / Inf 视为 0，避免污染电机命令（对应 running 的 prvSafe/isfinite） */
static float kin_safe(float v)
{
    if (v != v)          { return 0.0f; }
    if (v >  1.0e30f)    { return 0.0f; }
    if (v < -1.0e30f)    { return 0.0f; }
    return v;
}

static void motor_kinematics_feed_heartbeat(void)
{
    s_kin_last_cmd_ms   = HAL_GetTick();
    s_kin_ever_received = 1U;
}

static void motor_kinematics_zero_outputs(void)
{
    for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
    {
        write_float32_regs(s_holding_regs, (uint16_t)(REG_MOTOR_VEL_BASE + i * 2U), 0.0f);
    }
}

static void motor_kinematics_resolve(void)
{
    /* 1. 通信丢失保护（对应 running Kinematics_Resolve 的第 1 步）：
     *    从未收到上位机命令，或超过 KIN_LINK_TIMEOUT_MS 没新命令 -> 四轮强制 0 */
    if ((!s_kin_ever_received) ||
        ((uint32_t)(HAL_GetTick() - s_kin_last_cmd_ms) > KIN_LINK_TIMEOUT_MS))
    {
        motor_kinematics_zero_outputs();
        return;
    }

    /* 2. 读车体速度命令（大端 float，与 running 一致） */
    float vy = kin_safe(read_float32_regs(s_holding_regs, REG_MOTOR_VY_HI));
    float vx = kin_safe(read_float32_regs(s_holding_regs, REG_MOTOR_VX_HI));
    float wz = kin_safe(read_float32_regs(s_holding_regs, REG_MOTOR_WZ_HI));

    /* 3. 正交全向轮逆运动学 —— 与 running kinematics.c 逐行一致
     *    half_ly = half_lx = 0.075, inv_r = 20（KIN_LX/LY=0.15, KIN_R=0.05） */
    float half_ly = REG_MOTOR_LY / 2.0f;
    float half_lx = REG_MOTOR_LX / 2.0f;
    float inv_r   = 1.0f / REG_MOTOR_R;

    float w_lf =  (vx + half_ly * wz) * inv_r;
    float w_rf =  (vy + half_lx * wz) * inv_r;
    float w_rr = -(vx - half_ly * wz) * inv_r;
    float w_lr = -(vy - half_lx * wz) * inv_r;

    float vals[REG_MOTOR_COUNT] = {w_lf, w_rf, w_rr, w_lr};

    /* 4. 等比例限幅（对应 running 的 prvScaleAll）
     *    注意：取绝对值只用于求比例系数，绝不能写回 vals，否则负数轮速会被
     *    吃掉符号，电机永远不会反转。 */
    float max_abs = 0.0f;
    for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
    {
        float a = (vals[i] < 0.0f) ? -vals[i] : vals[i];
        if (a > max_abs)
        {
            max_abs = a;
        }
    }
    if (max_abs > REG_MOTOR_W_MAX)
    {
        float scale = REG_MOTOR_W_MAX / max_abs;
        for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
        {
            vals[i] *= scale;
        }
    }

    /* 5. 写回 0x000A~0x0011：LF / RF / RR / LR 顺时针顺序 */
    for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
    {
        write_float32_regs(s_holding_regs, (uint16_t)(REG_MOTOR_VEL_BASE + i * 2U), vals[i]);
    }
}

static void MX_CAN1_Init(void)
{
    /* CAN1 引脚：PD0 = CAN1_RX，PD1 = CAN1_TX（AF9），依据 RoboMaster C 板用户手册。
     * 注意：早期版本误配为 PA11/PA12（那是 USB_OTG_FS 的 D-/D+），已修正。
     * PD0/PD1 在本工程无其它用途。 */
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = GPIO_PIN_0 | GPIO_PIN_1;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    /* 达妙电机侧的收发器/终端电阻一般自带偏置，此处用 NOPULL 避免与总线既有上下拉打架 */
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOD, &gpio_init);

    CAN1->MCR |= CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) == 0U)
    {
    }

    /* 与 running (hcan1.Init.AutoRetransmission = ENABLE) 对齐：保持自动重传开启，
     * 总线上的偶发错误帧能自动补发，不会丢关键命令。
     *
     * 自动重传的代价是"总线上无节点应答时硬件会无限重传并占满三个发送邮箱"，
     * 这个风险由 motor_can_send_frame() 负责兜底：邮箱满时先短暂等待（vTaskDelay
     * 让出 CPU），仍满就中止三个 pending 邮箱，保证当前关键帧能发出去。
     *
     * 注：此前曾开启 CAN_MCR_NART（禁止自动重传）来缓解邮箱占满，但那样会丢掉
     * 偶发错误的帧，且与 running 语义不一致，故改回自动重传 + 超时中止的方案。 */
    CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;

    /* 位时间配置，PCLK1 = 42MHz，目标 1Mbps，与 running (PSC=3 / BS1=11TQ / BS2=2TQ / SJW=1TQ) 一致：
     *   SJW[25:24] = 0   -> 1TQ
     *   TS2 [22:20] = 1  -> 2TQ
     *   TS1 [19:16] = 10 -> 11TQ
     *   BRP [9:0]   = 2  -> 分频 3
     * 位时间 = 1 + 11 + 2 = 14 TQ  ->  42MHz / (3 * 14) = 1.000 Mbps，采样点 85.7%
     * 合值 0x001A0002。
     *
     * 修正前为 (3<<16)|(13<<8)|(2<<4)，13 落入保留位、2 把 BRP 撑成 289，
     * 实际波特率仅 42MHz/(289*6) ≈ 24.2 kbps，达妙电机不会响应。 */
    CAN1->BTR = (uint32_t)((0U << 24U) |    /* SJW  = 0  -> 1TQ  */
                           (1U << 20U) |    /* TS2  = 1  -> 2TQ  */
                           (10U << 16U) |   /* TS1  = 10 -> 11TQ */
                           2U);             /* BRP  = 2  -> PSC=3 */

    CAN1->FMR |= CAN_FMR_FINIT;
    CAN1->FA1R = 0U;
    CAN1->FM1R = 0U;
    CAN1->FS1R = 0x00000001U;
    CAN1->FFA1R = 0U;
    CAN1->sFilterRegister[0].FR1 = 0x00000000U;
    CAN1->sFilterRegister[0].FR2 = 0x00000000U;
    CAN1->FA1R |= 1U;
    CAN1->FMR &= ~CAN_FMR_FINIT;

    CAN1->MCR &= ~CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) != 0U)
    {
    }
}

/* 取空闲发送邮箱号：0/1/2 = 可用邮箱，3 = 三个邮箱都忙 */
static uint8_t motor_can_free_mailbox(void)
{
    if ((CAN1->TSR & CAN_TSR_TME0) != 0U)
    {
        return 0U;
    }
    if ((CAN1->TSR & CAN_TSR_TME1) != 0U)
    {
        return 1U;
    }
    if ((CAN1->TSR & CAN_TSR_TME2) != 0U)
    {
        return 2U;
    }
    return 3U;
}

static uint8_t motor_can_send_frame(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    uint8_t mailbox = motor_can_free_mailbox();
    uint8_t wait = 0U;

    if (len > 8U)
    {
        return 0U;
    }

    /* 与 running 的 CanTask_Send() 对齐（Core/Src/freertos.c:288）：
     * 保持自动重传开启（NART=0）后，总线上没有节点应答时硬件会无限重传并占满三个
     * 发送邮箱，因此邮箱满时必须能逃生。三步：
     *
     *   1) 先等待几个 tick，且用 vTaskDelay() 让出 CPU。
     *      原实现是 while 空转死等，而本函数运行在最高优先级的 ModbusTask 里，
     *      空转会把它下面三个任务（Fan/Pulse/Led）全部饿死，RS485 也跟着不回应。
     *   2) 等待超时后中止三个 pending 邮箱，等价 HAL_CAN_AbortTxRequest()：
     *      SET_BIT(TSR, ABRQx)。这是自动重传下的唯一逃生通道。
     *   3) 中止后仍无空闲邮箱则放弃本帧并返回 0，绝不无限等待。 */
    while ((mailbox > 2U) && (wait < MOTOR_CAN_TX_WAIT_TICKS))
    {
        vTaskDelay(1U);
        mailbox = motor_can_free_mailbox();
        wait++;
    }

    if (mailbox > 2U)
    {
        CAN1->TSR |= (CAN_TSR_ABRQ0 | CAN_TSR_ABRQ1 | CAN_TSR_ABRQ2);

        wait = 0U;
        while ((mailbox > 2U) && (wait < MOTOR_CAN_TX_WAIT_TICKS))
        {
            vTaskDelay(1U);
            mailbox = motor_can_free_mailbox();
            wait++;
        }
    }

    if (mailbox > 2U)
    {
        return 0U;
    }

    CAN1->sTxMailBox[mailbox].TIR = ((std_id & 0x7FFU) << 21U) | CAN_TI0R_TXRQ;
    CAN1->sTxMailBox[mailbox].TDTR = len;
    CAN1->sTxMailBox[mailbox].TDLR = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
    CAN1->sTxMailBox[mailbox].TDHR = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) | ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);
    return 1U;
}

static void motor_can_send_speed(uint16_t motor_index, float speed)
{
    union
    {
        float f;
        uint8_t b[4];
    } u;

    u.f = speed;
    /* 与 running CanTask_WriteFloat() 一致：厂家 spd_ctrl 为 4 字节小端 float，DLC=4。
     * 旧实现填 8 字节（后 4 字节补 0），DLC 冗余，已对齐 running。 */
    motor_can_send_frame((uint32_t)(REG_MOTOR_TX_ID_BASE + motor_index), u.b, 4U);
}

static void motor_can_tick(void)
{
    static uint32_t last_tick_ms = 0U;
    /* 使能/清错寄存器上一次的值：首次调用时从寄存器初值读取，避免上电误触发边沿。
     * 与 running CanTask 一致（Core/Src/freertos.c:341）。 */
    static uint16_t en_prev[REG_MOTOR_COUNT];
    static uint16_t clr_prev[REG_MOTOR_COUNT];
    static uint8_t edge_init_done = 0U;

    if ((HAL_GetTick() - last_tick_ms) < 10U)
    {
        return;
    }
    last_tick_ms = HAL_GetTick();

    if (edge_init_done == 0U)
    {
        for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
        {
            en_prev[i] = s_holding_regs[REG_MOTOR_EN_BASE + i];
            clr_prev[i] = s_holding_regs[REG_MOTOR_CLR_BASE + i];
        }
        edge_init_done = 1U;
    }

    /* 与 running CanTask 一致：发速度命令前先做运动学解算 */
    motor_kinematics_resolve();

    for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
    {
        uint16_t enable_addr = (uint16_t)(REG_MOTOR_EN_BASE + i);
        uint16_t clear_addr = (uint16_t)(REG_MOTOR_CLR_BASE + i);
        float speed = read_float32_regs(s_holding_regs, (uint16_t)(REG_MOTOR_VEL_BASE + i * 2U));
        uint16_t en = s_holding_regs[enable_addr];
        uint16_t clr = s_holding_regs[clear_addr];

        /* 使能边沿检测：0->非0 发使能，非0->0 发失能（与 running 一致）。
         * 旧实现是"电平触发 + 数据格式错"：使能寄存器非 0 时每 10ms 重复发
         * {i, 0x01, 0...}，电机在速度模式下会把前 4 字节当 float 解析成
         * 一个"速度≈0"的命令，真正的使能命令从未发出过，电机不会转。 */
        if (en != en_prev[i])
        {
            if (en != 0U)
            {
                motor_can_send_frame((uint32_t)(REG_MOTOR_TX_ID_BASE + i), c_cmd_enable, 8U);
            }
            else
            {
                motor_can_send_frame((uint32_t)(REG_MOTOR_TX_ID_BASE + i), c_cmd_disable, 8U);
            }
            en_prev[i] = en;
        }

        /* 清错边沿检测：仅 0->非0 上升沿发一次（与 running 一致）。
         * 注：旧实现发完后会把寄存器自动清零，running 不做清零，已对齐 running。 */
        if (clr != clr_prev[i])
        {
            if (clr != 0U)
            {
                motor_can_send_frame((uint32_t)(REG_MOTOR_TX_ID_BASE + i), c_cmd_clear_err, 8U);
            }
            clr_prev[i] = clr;
        }

        motor_can_send_speed(i, speed);
    }
}

/* ===== 电机状态帧解析：与 running Core/Src/freertos.c CanTask 的 RX 分支一致 =====
 * 满量程取值来自 running freertos.c:116-118 */
#define MOTOR_PMAX 12.5f   /* 位置 [-12.5, 12.5]，16 位定点 */
#define MOTOR_VMAX 200.0f  /* 速度 [-200, 200]，12 位定点 */
#define MOTOR_TMAX 10.0f   /* 扭矩 [-10, 10]，12 位定点 */

/* 无符号定点整数线性映射到 [x_min, x_max]（厂家 uint_to_float 原样移植） */
static float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
    float span = x_max - x_min;
    float offset = x_min;
    return ((float)x_int) * span / ((float)((1 << bits) - 1)) + offset;
}

/* 达妙电机状态上报帧：
 *   CAN ID = 0、DLC = 8（其余 ID 一律丢弃，避免污染电机状态区）
 *   data[0] 低 4 位 = 电机号 1~4，高 4 位 = 错误码
 *   data[1]:data[2]              POS 16 位定点
 *   data[3]:data[4] 高 4 位      VEL 12 位定点
 *   data[4] 低 4 位:data[5]      T   12 位定点
 *   data[6] T_MOS   data[7] T_Rotor
 */
static void motor_status_update_from_can(uint32_t std_id, const uint8_t *data, uint8_t dlc)
{
    if ((std_id != 0U) || (dlc < 8U))
    {
        return;
    }

    uint8_t motor_id = (uint8_t)(data[0] & 0x0FU);
    uint8_t err = (uint8_t)((data[0] >> 4U) & 0x0FU);
    if ((motor_id < 1U) || (motor_id > REG_MOTOR_COUNT))
    {
        return;
    }

    uint16_t m = (uint16_t)(motor_id - 1U);
    uint16_t base = (uint16_t)(REG_MOTOR_STATUS_M1 + m * REG_MOTOR_STATUS_STRIDE);

    s_holding_regs[base + 0U] = (uint16_t)err;
    s_holding_regs[base + 1U] = 0U;

    uint16_t pos_raw = (uint16_t)(((uint16_t)data[1] << 8U) | (uint16_t)data[2]);
    write_float32_regs(s_holding_regs, base + 2U,
                       uint_to_float((int)pos_raw, -MOTOR_PMAX, MOTOR_PMAX, 16));

    uint16_t vel_raw = (uint16_t)(((uint16_t)data[3] << 4U) | (uint16_t)(data[4] >> 4U));
    write_float32_regs(s_holding_regs, base + 4U,
                       uint_to_float((int)vel_raw, -MOTOR_VMAX, MOTOR_VMAX, 12));

    uint16_t t_raw = (uint16_t)(((uint16_t)(data[4] & 0x0FU) << 8U) | (uint16_t)data[5]);
    write_float32_regs(s_holding_regs, base + 6U,
                       uint_to_float((int)t_raw, -MOTOR_TMAX, MOTOR_TMAX, 12));

    s_holding_regs[base + 8U] = (uint16_t)data[6];
    s_holding_regs[base + 9U] = (uint16_t)data[7];
}

static void modbus_write_single_register(uint16_t addr, uint16_t value)
{
    if (addr >= MODBUS_REG_COUNT)
    {
        return;
    }

    if (is_can_status_region(addr))
    {
        return;
    }

    /* IMU 姿态输出区只读 */
    if (is_imu_status_region(addr))
    {
        return;
    }

    if ((addr >= REG_MOTOR_EN_BASE) && (addr <= (REG_MOTOR_WZ_HI + 1U)))
    {
        s_holding_regs[addr] = value;
        /* 写 0x0014~0x0019（vy/vx/wz）只更新通信心跳，不在这里解算。
         * 解算统一放在 motor_can_tick() 每 10ms 一次，与 running 的 CanTask 一致，
         * 保证 3 个 float 来自同一时刻快照；同时覆盖到 0x0019（wz 低字），
         * 旧实现的上限是 0x0018，写低字不会触发解算，wz 会用到半新半旧的值。 */
        if ((addr >= REG_MOTOR_VY_HI) && (addr <= (REG_MOTOR_WZ_HI + 1U)))
        {
            motor_kinematics_feed_heartbeat();
        }
        return;
    }

    if (addr >= REG_FAN_DUTY_BASE && addr <= (REG_FAN_DUTY_BASE + REG_FAN_CNT - 1U))
    {
        s_holding_regs[addr] = value;
        /* 自动模式下 0x0100 仅作为"手动备份值"保存，不驱动输出——
         * 输出由 fan_auto_update() 每 5ms 覆盖，否则手动值与姿态值会互相打架。
         * 切回手动（0x0140=0）后立即恢复生效，无需重新写一遍。 */
        if (s_holding_regs[REG_AUTO_FAN_MODE] == 0U)
        {
            sync_fan_outputs_from_regs();
        }
        return;
    }

    /* 自动模式参数区：任意一项被改写后立即重新计算一次输出，
     * 让上位机调参时能立刻看到效果（不必等下一轮 poll）。 */
    if (is_auto_param_region(addr))
    {
        /* 磁力计相关为只读/保留项：本轮强行维持固定值 */
        if ((addr == REG_AUTO_MAG_ENABLE) ||
            (addr == REG_AUTO_MAG_STATUS) ||
            (addr == REG_AUTO_MAG_CAL_CMD))
        {
            return;
        }
        /* 恒温目标温度：float32 占 2 个寄存器，由 Modbus 侧转发给 InsTask */
        s_holding_regs[addr] = value;
        if (addr == REG_AUTO_HEATER_TARGET)
        {
            /* 两个寄存器拼成 float32（高字在前），写完低字后再解释 */
            float t;
            uint32_t raw = ((uint32_t)s_holding_regs[REG_AUTO_HEATER_TARGET] << 16)
                         | (uint32_t)s_holding_regs[REG_AUTO_HEATER_TARGET + 1U];
            memcpy(&t, &raw, sizeof(t));
            ins_set_target_temp(t);
        }
        fan_auto_update();
        return;
    }

    if (addr == REG_PARAM_SLAVE_ADDR)
    {
        value = (value == 0U || value > 247U) ? 1U : value;
    }

    s_holding_regs[addr] = value;
    if (is_param_region(addr))
    {
        s_param_dirty = 1U;
        s_param_dirty_tick_ms = HAL_GetTick();
    }
}

static uint8_t build_exception(uint8_t slave_addr, uint8_t function_code, uint8_t exc_code, uint8_t *resp)
{
    resp[0] = slave_addr;
    resp[1] = (uint8_t)(function_code | 0x80U);
    resp[2] = exc_code;
    uint16_t crc = crc16_buffer(resp, 3U);
    resp[3] = (uint8_t)(crc & 0xFFU);
    resp[4] = (uint8_t)((crc >> 8U) & 0xFFU);
    return 5U;
}

static uint8_t process_request(const uint8_t *request, uint16_t len, uint8_t *response)
{
    if (len < 8U)
    {
        return 0U;
    }
    uint8_t slave = request[0];
    uint8_t func = request[1];
    uint16_t crc_expected = (uint16_t)((uint16_t)request[len - 2U] | ((uint16_t)request[len - 1U] << 8U));
    uint16_t crc_actual = crc16_buffer(request, (uint16_t)(len - 2U));
    if (crc_expected != crc_actual)
    {
        return 0U;
    }
    if (slave != get_modbus_slave_addr())
    {
        return 0U;
    }

    switch (func)
    {
        case 0x03U:
        case 0x04U:
        {
            uint16_t start = (uint16_t)((uint16_t)request[2] << 8U | request[3]);
            uint16_t qty = (uint16_t)((uint16_t)request[4] << 8U | request[5]);
            if ((qty == 0U) || ((uint32_t)start + (uint32_t)qty > MODBUS_REG_COUNT))
            {
                return build_exception(slave, func, 0x02U, response);
            }
            response[0] = slave;
            response[1] = func;
            response[2] = (uint8_t)(qty * 2U);
            uint16_t pos = 3U;
            for (uint16_t i = 0U; i < qty; ++i)
            {
                uint16_t v = s_holding_regs[start + i];
                response[pos++] = (uint8_t)((v >> 8U) & 0xFFU);
                response[pos++] = (uint8_t)(v & 0xFFU);
            }
            uint16_t crc = crc16_buffer(response, pos);
            response[pos++] = (uint8_t)(crc & 0xFFU);
            response[pos++] = (uint8_t)((crc >> 8U) & 0xFFU);
            return pos;
        }

        case 0x06U:
        {
            uint16_t addr = (uint16_t)((uint16_t)request[2] << 8U | request[3]);
            uint16_t value = (uint16_t)((uint16_t)request[4] << 8U | request[5]);
            if (addr >= MODBUS_REG_COUNT)
            {
                return build_exception(slave, func, 0x02U, response);
            }
            modbus_write_single_register(addr, value);
            memcpy(response, request, 8U);
            return 8U;
        }

        case 0x10U:
        {
            uint16_t addr = (uint16_t)((uint16_t)request[2] << 8U | request[3]);
            uint16_t qty = (uint16_t)((uint16_t)request[4] << 8U | request[5]);
            uint8_t byte_count = request[6];
            if ((qty == 0U) || (byte_count != (uint8_t)(qty * 2U)) || ((uint32_t)addr + (uint32_t)qty > MODBUS_REG_COUNT))
            {
                return build_exception(slave, func, 0x02U, response);
            }
            for (uint16_t i = 0U; i < qty; ++i)
            {
                uint16_t value = (uint16_t)((uint16_t)request[7U + i * 2U] << 8U | request[8U + i * 2U]);
                modbus_write_single_register((uint16_t)(addr + i), value);
            }
            response[0] = slave;
            response[1] = func;
            response[2] = request[2];
            response[3] = request[3];
            response[4] = request[4];
            response[5] = request[5];
            uint16_t crc = crc16_buffer(response, 6U);
            response[6] = (uint8_t)(crc & 0xFFU);
            response[7] = (uint8_t)((crc >> 8U) & 0xFFU);
            return 8U;
        }

        default:
            return build_exception(slave, func, 0x01U, response);
    }
}

void modbus_init(void)
{
    memset(s_holding_regs, 0, sizeof(s_holding_regs));

    s_holding_regs[REG_PARAM_UART_CFG] = 0U;
    s_holding_regs[REG_PARAM_SLAVE_ADDR] = MODBUS_SLAVE_ADDR;
    s_holding_regs[REG_FAN_DUTY_BASE + 0U] = 0U;
    s_holding_regs[REG_FAN_DUTY_BASE + 1U] = 0U;
    s_holding_regs[REG_FAN_DUTY_BASE + 2U] = 0U;
    s_holding_regs[REG_FAN_DUTY_BASE + 3U] = 0U;

    /* ---- 风机自动模式（姿态前馈）默认参数 ----
     * 默认手动模式（FAN_MODE=0），避免上电瞬间风机动起来；
     * 但 DUTY_FLAT / DUTY_MIN 等默认值已按"风洞爬行"场景预置，
     * 上位机把 0x0140 写成 1、0x0141 写成 1 即可切入自动。
     *
     * 关键：DUTY_MIN 默认 30 —— 这个下限不是随便取的。
     * 重力分量补偿的物理前提是"吸附力始终存在"，
     * 一旦占空比掉到 0，风机停转、吸附力消失，小车直接脱离壁面。
     * 因此即使姿态水平（1-cosθ = 0），输出也必须 >= DUTY_MIN。 */
    s_holding_regs[REG_AUTO_FAN_MODE]      = 0U;
    s_holding_regs[REG_AUTO_FAN_AUTO_EN]   = 0U;
    s_holding_regs[REG_AUTO_DUTY_FLAT]     = 30U;
    s_holding_regs[REG_AUTO_DUTY_MIN]      = 30U;
    s_holding_regs[REG_AUTO_DUTY_MAX]      = 100U;
    s_holding_regs[REG_AUTO_SLOPE_GAIN]    = 60U;
    s_holding_regs[REG_AUTO_KP_PITCH]      = 0U;
    s_holding_regs[REG_AUTO_KP_ROLL]       = 0U;
    s_holding_regs[REG_AUTO_PITCH_OFFSET]  = 0U;
    s_holding_regs[REG_AUTO_ROLL_OFFSET]   = 0U;
    s_holding_regs[REG_AUTO_MAG_ENABLE]    = 0U;   /* 本轮不接磁力计 */
    s_holding_regs[REG_AUTO_MAG_STATUS]    = 2U;   /* 2 = 未接入 */
    s_holding_regs[REG_AUTO_MAG_CAL_CMD]   = 0U;   /* 保留 */

    MX_CAN1_Init();
    retentive_load_params();
    refresh_fan_state_registers();

    s_rx_head = 0U;
    s_rx_tail = 0U;
    rs485_set_tx(0U);
    HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
}

void modbus_poll(void)
{
    static uint8_t frame[64];
    static uint16_t frame_len = 0U;

    retentive_poll();
    motor_can_poll_rx();
    motor_can_tick();

    /* 姿态前馈自动占空比：每轮 poll(5ms) 更新一次。
     * 放在 ModbusTask 而非 InsTask 的原因：
     *   - 风机输出是"执行器动作"，本就该由唯一的输出任务统一决策，避免两处竞态；
     *   - InsTask 只负责算姿态并发布快照，职责单一；
     *   - 5ms 刷新率对风洞爬行（速度很慢）而言已远超需求。 */
    fan_auto_update();
    refresh_imu_registers();

    while (ring_len() > 0U)
    {
        uint8_t byte = 0U;
        if (!ring_pop(&byte))
        {
            break;
        }
        frame[frame_len++] = byte;

        if (frame_len >= sizeof(frame))
        {
            frame_len = 0U;
            continue;
        }

        if (frame_len >= 8U)
        {
            uint8_t function = frame[1];
            uint16_t expected_len = (function == 0x10U) ? 9U : 8U;
            if (frame_len >= expected_len)
            {
                uint16_t total_len = expected_len;
                if (function == 0x10U)
                {
                    uint16_t qty = (uint16_t)((uint16_t)frame[4U] << 8U | frame[5U]);
                    total_len = (uint16_t)(9U + qty * 2U);
                }
                if (frame_len >= total_len)
                {
                    uint16_t crc_actual = crc16_buffer(frame, (uint16_t)(total_len - 2U));
                    uint16_t crc_expected = (uint16_t)((uint16_t)frame[total_len - 2U] | ((uint16_t)frame[total_len - 1U] << 8U));
                    if (crc_actual == crc_expected)
                    {
                        uint8_t response[128];
                        uint16_t response_len = process_request(frame, total_len, response);
                        if (response_len > 0U)
                        {
                            modbus_send_frame(response, response_len);
                        }
                        memset(frame, 0, sizeof(frame));
                        frame_len = 0U;
                        continue;
                    }
                    /* CRC mismatch: drop first byte and keep newest stream */
                    memmove(frame, frame + 1U, frame_len - 1U);
                    frame_len--;
                    continue;
                }
            }
        }
    }
}

static void motor_can_poll_rx(void)
{
    while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0U)
    {
        uint32_t rir = CAN1->sFIFOMailBox[0].RIR;
        uint32_t std_id = (rir >> 21U) & 0x7FFU;
        uint8_t dlc = (uint8_t)(CAN1->sFIFOMailBox[0].RDTR & 0x0FU);
        uint8_t rx_data[8] = {0};

        rx_data[0] = (uint8_t)(CAN1->sFIFOMailBox[0].RDLR & 0xFFU);
        rx_data[1] = (uint8_t)((CAN1->sFIFOMailBox[0].RDLR >> 8U) & 0xFFU);
        rx_data[2] = (uint8_t)((CAN1->sFIFOMailBox[0].RDLR >> 16U) & 0xFFU);
        rx_data[3] = (uint8_t)((CAN1->sFIFOMailBox[0].RDLR >> 24U) & 0xFFU);
        rx_data[4] = (uint8_t)(CAN1->sFIFOMailBox[0].RDHR & 0xFFU);
        rx_data[5] = (uint8_t)((CAN1->sFIFOMailBox[0].RDHR >> 8U) & 0xFFU);
        rx_data[6] = (uint8_t)((CAN1->sFIFOMailBox[0].RDHR >> 16U) & 0xFFU);
        rx_data[7] = (uint8_t)((CAN1->sFIFOMailBox[0].RDHR >> 24U) & 0xFFU);

        if (dlc > 8U)
        {
            dlc = 8U;
        }
        motor_status_update_from_can(std_id, rx_data, dlc);
        CAN1->RF0R |= CAN_RF0R_RFOM0;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        ring_push(s_rx_byte);
        HAL_UART_Receive_IT(&huart2, &s_rx_byte, 1U);
    }
}
