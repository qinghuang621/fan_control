
#include "bsp_modbus.h"

#include "main.h"
#include "usart.h"
#include "bsp_fric.h"
#include "bsp_pulse.h"
#include "ins_task.h"

/* CAN 使用 HAL API + 中断队列（对齐 running），邮箱等待需 vTaskDelay() 让出 CPU */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"

#include <string.h>
#include <math.h>

#define MODBUS_SLAVE_ADDR 0x01U
#define MODBUS_RX_RING_SIZE 128U
/* 帧间静默超时（ms）：Modbus RTU 用 3.5 字符间隔分帧。
 * 115200 8N1 下一个字符约 87us，3.5 字符 ≈ 305us；取 4ms 留足余量，
 * 远小于上位机默认 1000ms 扫描周期，不会误切正常帧。用来丢弃残缺帧。 */
#define MODBUS_FRAME_GAP_MS 4U

/* ==================== RS485 底层：裸寄存器 + 自管中断 ====================
 * 与 running 的 Middlewares/Third_Party/FreeModbus/modbus/port/portserial.c 对齐：
 * **完全绕过 HAL UART 驱动**，USART6_IRQHandler 里直接读 SR/DR 处理收发。
 *
 * 为什么必须这么做（都是实际踩过的坑）：
 *   1. HAL_UART_Receive_IT 是逐字节状态机。一旦发生 ORE 溢出，HAL 会**关闭 RXNE 中断**
 *      并进入错误回调；若没接 HAL_UART_ErrorCallback，接收就**永久停摆**
 *      （RxState 卡在 BUSY_RX），但 TX 仍正常 —— 现象是"发得出去、收不到回包"。
 *      running 的做法是**每次中断都检查并清 ORE**，根本不给它停摆的机会。
 *   2. HAL_UART_Transmit 返回时最后一个字节可能还在移位寄存器里，
 *      此时立刻拉低 RS485 方向脚（PG8）会**截断最后一位**。
 *      running 用 **TC（发送完成）中断** 才切回接收，保证移出完整。
 *
 * RS485 方向：PG8 高=发送，低=接收。
 */
static volatile uint8_t s_rx_buf[MODBUS_RX_RING_SIZE];
static volatile uint16_t s_rx_head = 0U;
static volatile uint16_t s_rx_tail = 0U;

/* 发送状态：s_tx_buf 为 NULL 表示空闲 */
static const uint8_t *s_tx_buf = NULL;
static volatile uint16_t s_tx_len = 0U;
static volatile uint16_t s_tx_pos = 0U;

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
/* ⚠️ 改过 values[] 长度就必须升版本：旧镜像的 CRC 会算不过，
 * 但显式升版本更干净、也便于排查"为什么参数没恢复"。 */
#define FLASH_RETENTIVE_VERSION 2UL
/* 保持区覆盖 0x0090~0x0173（连续）：48 → 228。
 * 为什么要扩（2026-09-15）：原先只保持 0x0090~0x00BF，导致 `0x0140~0x014F`（风机自动参数）
 * 与 LUT **复位即丢** → 复位后 FAN_MODE 回 0、0x0100~0x0103 回 0 → 风机全停 → 小车掉壁。
 * 顺带把手动占空比 0x0100~0x0103 也纳入了保持。
 * 块大小 = 16 字节头 + 228*2 = 472 字节（仍是 4 字节对齐，逐字烧写 118 个 word）。
 *
 * ⚠️ 已核实：扩区**不会**造成每 5ms 写 Flash。所有每轮镜像刷新
 * （refresh_fan_state_registers / refresh_imu_registers / modbus_poll 里的 0x014D/4E/4F）
 * 都是**直接赋值** s_holding_regs[...]，**不走 modbus_write_single_register()**；
 * 脏标志只在那个函数末尾置位。 */
#define FLASH_RETENTIVE_PARAM_COUNT 228U
#define FLASH_RETENTIVE_BLOCK_SIZE 472U

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
/* 四轮角速度等比例限幅（rad/s）。**纯安全钳位**：NaN/Inf 已单独归零，
 * 这里只在"命令离谱"时按比例整体缩小，正常操作不该碰到它。
 * 取值依据（2026-09-11 长老要求放宽，30 → 50）：
 *   1) 上位机 `D:\stm32\host` 的 Fast 档最坏组合 vx=2.0 / vy=1.2 / wz=4.0
 *      → 最大轮速 20*(2.0 + 0.075*4) = 46.0 rad/s；取 50 留余量、全程不触发限幅。
 *      旧值 30 会把 Fast 档 vx 单独削到 30/40 = 0.75 倍（有效只剩 1.5 m/s）。
 *   2) 电机 DM-S2325-1EC 输出轴额定 600 rpm ≈ 62.8 rad/s，50 rad/s(≈478 rpm)
 *      仍留约 20% 余量。
 * 标定用参考值：vx=1/vy=1/wz=1 → 21.5 rad/s，离钳位很远。 */
#define REG_MOTOR_W_MAX 50.0f
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

/* 串口配置枚举表（对齐 running Core/Src/uart_config.c）。
 * 保持寄存器 0x0090 存枚举下标，上电查表得到波特率/校验，下次断电重启生效。
 * 之前本工程把 0x0090 写死成 0 且从不使用 —— 上位机改这个寄存器毫无效果，
 * 属功能缺口，故补齐。越界一律回退 index 0（115200 8N1）。 */
typedef struct
{
    uint32_t baud;
    uint32_t parity;   /* UART_PARITY_NONE / EVEN / ODD */
} uart_cfg_t;

static const uart_cfg_t s_uart_cfg_table[] = {
    { 115200U, UART_PARITY_NONE },   /* 0: 115200 8N1（默认） */
    { 115200U, UART_PARITY_EVEN },   /* 1: 115200 8E1 */
    { 19200U,  UART_PARITY_NONE },   /* 2: 19200 8N1 */
    { 19200U,  UART_PARITY_EVEN },   /* 3: 19200 8E1 */
    { 9600U,   UART_PARITY_NONE },   /* 4: 9600 8N1 */
    { 9600U,   UART_PARITY_EVEN },   /* 5: 9600 8E1 */
    { 38400U,  UART_PARITY_NONE },   /* 6: 38400 8N1 */
    { 57600U,  UART_PARITY_NONE },   /* 7: 57600 8N1（本工程扩展项） */
};
#define UART_CFG_COUNT (sizeof(s_uart_cfg_table) / sizeof(s_uart_cfg_table[0]))

static const uart_cfg_t *uart_cfg_lookup(uint16_t idx)
{
    if (idx >= (uint16_t)UART_CFG_COUNT)
    {
        idx = 0U;
    }
    return &s_uart_cfg_table[idx];
}

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
#define REG_AUTO_HEATER_PWM    0x014FU  /* 诊断用：当前加热 PWM 值（只读，0~5000） */

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
/* 诊断用：温度×10 的有符号整数（如 43.5℃ -> 435）。放不下 float32，
 * 故用整数形式占最后 1 个寄存器。用于排查"卡在 WARMUP"这类问题。 */
#define REG_IMU_TEMP_X10       0x015FU

/* ==================== 风机查表（LUT）模式 0x0160~0x016F ====================
 * 为什么需要：现行 `duty = FLAT + GAIN*(1-cosθ)` 是**单调**直线，只在 [0°,90°] 标定。
 * 而风洞外围爬行要覆盖到 180°（完全倒立），实际所需吸附力在**侧壁 90° 达峰、
 * 到倒立区反而回落** —— 形状根本不符，不是精度问题。论证与标定流程见 `标定手册.md` §8。
 * 这里只做三件事：存表、查表、线性插值。**不做任何拟合。** */
#define REG_LUT_BASE       0x0160U  /* DUTY_LUT[0..12]，θ = 0°,15°,…,180° */
#define REG_LUT_COUNT      13U
#define REG_LUT_MAGIC      0x016DU  /* 写 0xA5C3 表示表有效；否则自动回退线性模型 */
#define REG_LUT_MAGIC_VAL  0xA5C3U
#define REG_IMU_THETA      0x016EU  /* 实时 θ(deg) float32，**只读**，占 0x016E~0x016F */

/* 断电保持区的**上界**（见下方 FLASH_RETENTIVE_* 常量）。2026-09-15 由 0x00BF 扩到这里，
 * 目的：`0x0140~0x014F` 与 LUT 复位即丢 → 复位后 FAN_MODE 回 0、风机全停 → 掉壁。 */
#define REG_RETENTIVE_END  0x0173U

/* ==================== IMU 四元数输出 0x0174~0x017B（只读） ====================
 * 4 个 float32，高字在前，每 5ms 刷新。
 * **刻意放在断电保持区之外**（保持区到 0x0173 为止）：这是实时量，
 * 持久化它没有意义，还会白占 8 个寄存器的 Flash 空间。
 * 顺序 (w,x,y,z)、**机体系 → 世界系**（v_world = R(q)·v_body）。
 * ⚠️ 与 ROS2 的 `geometry_msgs/Quaternion` 字段顺序 (x,y,z,w) **相反**，
 *    发布时必须显式映射，不能内存直拷。详见 `接口文档.md` §6.8。 */
#define REG_IMU_QUAT_W     0x0174U
#define REG_IMU_QUAT_X     0x0176U
#define REG_IMU_QUAT_Y     0x0178U
#define REG_IMU_QUAT_Z     0x017AU
#define REG_IMU_QUAT_END   0x017BU

#define LUT_STEP_DEG       15.0f
#define LUT_DEG2RAD        0.017453292519943295f
#define LUT_RAD2DEG        57.29577951308232f

/* 13 个**占位值**：由理论模型算出，**没有任何实测依据**，等小车能跑后逐点标定覆盖。
 * 模型：`N_req/mg = sinθ/μ`（θ≤90°，摩擦/下滑主导）
 *                    `= max(sinθ/μ, -cosθ)`（θ>90°，脱离壁面主导）
 *       占空比→吸力按 `N ∝ duty²`（轴流推力 ∝ 转速²）⇒ `duty ∝ √N`
 * 锚点：保留原默认值在 θ=0° 与 θ=90° 两处不变（30% / 90%），只修正中间与倒立区的形状。
 * 取 μ=0.7（橡胶/涂装面量级）。**μ≤1 时峰值落在 90°；μ>1 才移到 180°。**
 * 生成脚本与敏感度表见 `标定手册.md` §8.9。 */
static const uint16_t s_duty_lut_default[REG_LUT_COUNT] = {
     30U,  46U,  64U,  76U,  84U,  88U,  90U,
     88U,  84U,  76U,  70U,  74U,  75U,
};

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

/* ---- 编译期断言：把"必须同步修改"的几个常量钉死 ----
 * 背景（2026-09-15）：FLASH_RETENTIVE_BLOCK_SIZE 原先只被定义、**从未被任何代码引用**，
 * 纯粹是个"文档性常量"。也就是说改 PARAM_COUNT 而忘了同步 BLOCK_SIZE，
 * 编译器不会有任何反应 —— 这次扩区（48→228）是靠手工验算才发现两者对得上的。
 * 加断言后，这三处一旦不同步就**直接编译失败**，不用再靠人眼。
 *
 * 断言内容：
 *   ① 结构体真实大小 == FLASH_RETENTIVE_BLOCK_SIZE
 *      （= 16 字节头 + PARAM_COUNT×2）
 *   ② 保持区寄存器范围 [REG_PARAM_BASE, REG_PARAM_BASE+PARAM_COUNT-1]
 *      必须正好结束于 REG_RETENTIVE_END —— 否则 is_param_region() 与
 *      实际落盘的寄存器范围不一致，会出现"能写但不落盘"或"落盘了但只读得回一半"。
 *
 * ⚠️ 断言消息**必须用 ASCII/英文**：源文件是 UTF-8，而 GCC 对字符串字面量按默认字符集
 *    处理，中文消息会被编成 `\37777777745...` 之类的乱码，等于没有提示。
 *    （本文件的中文注释不受影响 —— 编译期就被剥掉了。） */
_Static_assert(sizeof(retentive_block_t) == FLASH_RETENTIVE_BLOCK_SIZE,
               "retentive_block_t size != FLASH_RETENTIVE_BLOCK_SIZE");
_Static_assert((REG_PARAM_BASE + FLASH_RETENTIVE_PARAM_COUNT - 1U) == REG_RETENTIVE_END,
               "retentive region end != REG_RETENTIVE_END");

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

/* ==================== 收发底层（裸寄存器，对齐 running portserial.c） ==================== */

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
    *out_byte = s_rx_buf[s_rx_tail];
    s_rx_tail = (uint16_t)((s_rx_tail + 1U) % MODBUS_RX_RING_SIZE);
    return 1U;
}

/* 只在 USART6 中断里调用 */
static void ring_push_isr(uint8_t byte)
{
    uint16_t next = (uint16_t)((s_rx_head + 1U) % MODBUS_RX_RING_SIZE);
    if (next == s_rx_tail)
    {
        /* 溢出：丢掉最旧一个字节，保证最新数据优先（与 running 的覆盖语义一致） */
        s_rx_tail = (uint16_t)((s_rx_tail + 1U) % MODBUS_RX_RING_SIZE);
    }
    s_rx_buf[s_rx_head] = byte;
    s_rx_head = next;
}

/* RS485 方向：PG8 高=发送，低=接收 */
static void rs485_set_tx(uint8_t enable)
{
    HAL_GPIO_WritePin(GPIOG, GPIO_PIN_8, enable ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

/* 清 ORE/FE/NE/PE：F4 上「读 SR 再读 DR」即为硬件要求的清除序列。
 * 与 running portserial.c 里 `(volatile void)huart2.Instance->DR;` 等价。 */
static void usart6_clear_err_flags(void)
{
    (void)USART6->SR;
    (void)USART6->DR;
}

/* 使能/关闭接收（对齐 running vMBPortSerialEnable） */
static void rs485_rx_enable(uint8_t enable)
{
    if (enable)
    {
        rs485_set_tx(0U);                  /* 切到接收方向 */
        usart6_clear_err_flags();          /* 清掉残留 ORE，再开中断 */
        SET_BIT(USART6->CR1, USART_CR1_RXNEIE);
    }
    else
    {
        CLEAR_BIT(USART6->CR1, USART_CR1_RXNEIE);
    }
}

/* 启动一次中断驱动的发送。
 * 与 running 一致：**先切到发送方向，再打开 TXE 中断**，由中断逐字节吐数据，
 * 最后用 TC 中断切回接收。这样不会像 HAL_UART_Transmit 那样提前拉低方向脚、
 * 把最后一个字节（尤其奇偶校验位）截断。 */
static void modbus_send_frame(const uint8_t *frame, uint16_t len)
{
    if ((len == 0U) || (s_tx_buf != NULL))
    {
        return;   /* 上一帧还没发完，丢弃本帧（不会发生：Modbus 一问一答） */
    }

    /* 发送是中断驱动的，s_tx_buf 只保存指针。frame 必须活到 TC 中断触发为止。
     * 因此调用方**必须传入常驻缓冲**（如 modbus_poll 里的 static response[]），
     * 不能用栈上的局部数组 —— 函数返回后栈帧可能被覆盖，中断吐出的就是脏数据。 */
    s_tx_buf = frame;
    s_tx_len = len;
    s_tx_pos = 0U;

    rs485_set_tx(1U);                      /* 切到发送方向 */
    __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
    SET_BIT(USART6->CR1, USART_CR1_TXEIE); /* 打开 TXE 中断，开始吐数据 */
}

/* 等待上一帧发送完成（超时兜底）。返回 1=已发完。
 * ⚠️ 本函数跑在 ModbusTask（最高优先级 idle+4），**必须让出 CPU**：
 * 串口中断优先级低于任务，若任务在这里空转，中断永远进不来，会死等到超时。
 * 故每轮 vTaskDelay(1) 阻塞 1ms —— 发送一帧最多 3.5ms，4 轮就够。 */
static uint8_t modbus_tx_wait_done(uint32_t timeout_ms)
{
    uint32_t t0 = HAL_GetTick();
    while ((s_tx_buf != NULL) && ((HAL_GetTick() - t0) < timeout_ms))
    {
        vTaskDelay(1);
    }
    return (s_tx_buf == NULL) ? 1U : 0U;
}

/* ---------------- USART6 中断服务（由 Src/stm32f4xx_it.c 调用） ----------------
 * 与 running 的 vMBPortSerialIRQHandler 结构一致：
 *   ① RXNE  → 读 DR 存入 FIFO
 *   ② 错误标志 → 读 DR 清除（每次中断都查，ORE 不给它停摆的机会）
 *   ③ TXE   → 写下一字节；写完关 TXE、开 TC
 *   ④ TC    → 关 TC、切回接收方向，发送结束
 * 注意：**不调用 HAL_UART_IRQHandler**，避免 HAL 状态机与裸寄存器操作打架。 */
void usart6_irq_handler(void)
{
    uint32_t sr  = USART6->SR;
    uint32_t cr1 = USART6->CR1;

    /* ① RXNE：收到一个字节 */
    if ((sr & USART_SR_RXNE) != 0U)
    {
        uint8_t byte = (uint8_t)(USART6->DR & 0xFFU);
        if ((cr1 & USART_CR1_RXNEIE) != 0U)
        {
            ring_push_isr(byte);
        }
        /* 未使能接收时读 DR 即已丢弃，同时防止 ORE */
    }

    /* ② ORE/FE/NE/PE：读 SR 再读 DR 清除。RXNE 分支已经读过 DR，
     *    这里再显式清一次，覆盖"只有错误没有 RXNE"的情况。 */
    if ((sr & (USART_SR_ORE | USART_SR_FE | USART_SR_NE | USART_SR_PE)) != 0U)
    {
        usart6_clear_err_flags();
    }

    /* 上面的读操作可能改变标志，重新采样 */
    sr  = USART6->SR;
    cr1 = USART6->CR1;

    /* ③ TXE：数据寄存器空，写下一字节 */
    if (((sr & USART_SR_TXE) != 0U) && ((cr1 & USART_CR1_TXEIE) != 0U))
    {
        if ((s_tx_buf != NULL) && (s_tx_pos < s_tx_len))
        {
            USART6->DR = (uint8_t)s_tx_buf[s_tx_pos++];
        }

        if ((s_tx_buf != NULL) && (s_tx_pos >= s_tx_len))
        {
            /* 最后一个字节已写入 DR，关 TXE、开 TC，等它完全移出 */
            CLEAR_BIT(USART6->CR1, USART_CR1_TXEIE);
            __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);
            SET_BIT(USART6->CR1, USART_CR1_TCIE);
        }
    }

    /* ④ TC：最后一个字节完全移出，切回接收方向并结束发送 */
    if (((sr & USART_SR_TC) != 0U) && ((cr1 & USART_CR1_TCIE) != 0U))
    {
        CLEAR_BIT(USART6->CR1, USART_CR1_TCIE);
        __HAL_UART_CLEAR_FLAG(&huart2, UART_FLAG_TC);

        s_tx_buf = NULL;
        s_tx_len = 0U;
        s_tx_pos = 0U;

        rs485_set_tx(0U);   /* 切回接收（等 TC 才切，不截断最后一位） */
    }
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

/* 断电保持区：0x0090~0x0173（含风机自动参数 0x0140~0x014F 与 LUT 0x0160~0x016F）。
 * 写入后由 s_param_dirty 延迟 2s 去抖落盘，见 retentive_poll()。
 * ⚠️ 上界是 REG_RETENTIVE_END 而不是 REG_PARAM_END —— 扩区前这两者恰好相邻，
 * 改的时候漏一个就会"参数能写但不落盘"。 */
static uint8_t is_param_region(uint16_t addr)
{
    return (addr >= REG_PARAM_BASE) && (addr <= REG_RETENTIVE_END);
}

static uint8_t is_auto_param_region(uint16_t addr)
{
    return (addr >= REG_AUTO_BASE) && (addr <= REG_AUTO_END);
}

/* IMU 姿态输出区为只读：上位机写这些地址一律静默忽略（不返回异常码，
 * 保持与既有状态区 0x0040~0x008F 相同的"写无效但不报错"行为，便于上位机统一处理）。
 * 覆盖三段：① `0x0150~0x015F` 姿态/角速度/温度；
 *          ② `0x016E~0x016F` TILT_THETA（float32 实时 θ）；
 *          ③ `0x0174~0x017B` 姿态四元数。
 * 这些都是每 5ms 被 refresh_imu_registers() 覆盖的实时量，允许写只会静默失效、徒增困惑。 */
static uint8_t is_imu_status_region(uint16_t addr)
{
    return ((addr >= REG_IMU_BASE) && (addr <= REG_IMU_END)) ||
           ((addr >= REG_IMU_THETA) && (addr <= (uint16_t)(REG_IMU_THETA + 1U))) ||
           ((addr >= REG_IMU_QUAT_W) && (addr <= REG_IMU_QUAT_END));
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

        /* ⚠️ 占空比反馈取「**实际加到定时器上的值**」，而不是 0x0100~0x0103 寄存器值。
         * 原因：自动模式（0x0140=1 且 0x0141=1）下 0x0100~0x0103 只作"手动备份值"保存、
         * **不驱动输出**，输出由 fan_auto_update() 每 5ms 用姿态算出来直接写 CCR。
         * 若按寄存器回读，自动模式下这一格会永远停在最后一个手动值（曾是本工程的缺陷）。
         * 通道映射与 sync_fan_outputs_from_regs() 保持一致：
         *   风机 1 -> 通道 1（PWM5）、风机 2 -> 通道 2（PWM6）、
         *   风机 3 与风机 4 -> 共用通道 3（PWM7），故两者读到同一个值（这是物理事实）。 */
        uint8_t  ch   = (uint8_t)((i < 2U) ? (i + 1U) : 3U);
        uint16_t duty = clamp_duty((uint16_t)fric_get_channel_duty(ch));
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

/* 查表 + 线性插值：θ(deg) → 占空比(%)。
 * 表步长 15°、13 个点覆盖 θ ∈ [0°,180°]；越界一律取端点值（不外推）。 */
static float fan_lut_interp(float theta_deg)
{
    float pos = theta_deg / LUT_STEP_DEG;
    float a, b;
    uint16_t i;

    if (pos <= 0.0f)
    {
        return (float)s_holding_regs[REG_LUT_BASE];
    }
    if (pos >= (float)(REG_LUT_COUNT - 1U))
    {
        return (float)s_holding_regs[REG_LUT_BASE + REG_LUT_COUNT - 1U];
    }

    i = (uint16_t)pos;      /* 上面已保证 0 < pos < 12，不会越界 */
    a = (float)s_holding_regs[REG_LUT_BASE + i];
    b = (float)s_holding_regs[REG_LUT_BASE + i + 1U];
    return a + (b - a) * (pos - (float)i);
}

static void fan_auto_update(void)
{
    ins_snapshot_t snap;
    float roll_deg, pitch_deg;
    float cos_theta, theta_deg;
    float duty_f;
    uint16_t duty_min, duty_max, mode;
    uint16_t out;

    /* 只有"自动模式 + 总开关打开"才接管；否则沿用 0x0100 的手动值 */
    mode = s_holding_regs[REG_AUTO_FAN_MODE];
    if ((mode == 0U) || (s_holding_regs[REG_AUTO_FAN_AUTO_EN] == 0U))
    {
        fan_auto_duty = 0U;
        return;
    }

    /* ⚠️ IMU 未进入 RUNNING 之前**不接管**，让 0x0100~0x0103 的手动值生效。
     * 为什么必须有这道门：InsTask 在 WARMUP 期间以 `while (!s_first_temperate)`
     * 阻塞轮询、**根本不调用 publish_snapshot()**，而 s_snap_seq 初值为 0（偶数）→
     * INS_get_snapshot() 会**返回"成功"但给出全 0 的快照** → 本函数算得 pitch=roll=0
     * → θ=0 → 输出"水平档"占空比。该窗口上限 INS_TEMP_BOOT_TIMEOUT_MS = **20 秒**
     * （且因加热功率不足，实际大概率跑满超时而不是温度达标）。
     * 若小车停在侧壁上时板子复位（掉电/看门狗/EMI/手按复位），这 20 秒会把它吹下来。
     *
     * 回退到手动通道即可自保：只要事先把 0x0100~0x0103 写成高位值
     * （该值 2026-09-15 起已随断电保持一起持久化）。地上开机时它是 0 → 风机不转，
     * 等 RUNNING 后自动接管，行为不意外。 */
    if (ins_get_status() != INS_STATUS_RUNNING)
    {
        fan_auto_duty = 0U;
        return;
    }

    /* 读姿态快照：失败说明正在写，沿用上一轮结果，不阻塞 */
    if (!INS_get_snapshot(&snap))
    {
        return;
    }

    duty_min = s_holding_regs[REG_AUTO_DUTY_MIN];
    duty_max = s_holding_regs[REG_AUTO_DUTY_MAX];

    /* 零位偏置：补偿 IMU 安装误差 */
    pitch_deg = snap.pitch - (float)(int16_t)s_holding_regs[REG_AUTO_PITCH_OFFSET];
    roll_deg  = snap.roll  - (float)(int16_t)s_holding_regs[REG_AUTO_ROLL_OFFSET];

    /* 合成总倾角余弦。
     * ⚠️ cos(pitch)*cos(roll) **恒等于旋转矩阵的 R33**，是四元数的光滑函数 ——
     * 即使 pitch 撞上万向节死锁(±90°)，这个乘积仍然稳（数值验证 duty 标准差 <0.01%）。
     * 反过来，单独取 roll / pitch 用会在死锁处剧烈抖动（圆周标准差可达 157°）。
     * **所以不要"为了更清楚"把它拆成两个角分别用。** */
    cos_theta = cosf(pitch_deg * LUT_DEG2RAD) * cosf(roll_deg * LUT_DEG2RAD);

    if ((mode == 2U) && (s_holding_regs[REG_LUT_MAGIC] == REG_LUT_MAGIC_VAL))
    {
        /* ---- 查表模式 ----
         * 魔数不匹配时**自动回退线性模型**，绝不输出 0：
         * 未标定时整张表是 0，直接驱动会让吸附力归零、小车掉壁。
         * 这是设计上的安全阀，"写了表但不写魔数 → 不生效"是有意为之。 */
        if (cos_theta < -1.0f) { cos_theta = -1.0f; }
        if (cos_theta >  1.0f) { cos_theta =  1.0f; }
        theta_deg = acosf(cos_theta) * LUT_RAD2DEG;
        duty_f = fan_lut_interp(theta_deg);
    }
    else
    {
        /* ---- 线性模式（默认；行为与 2026-09-15 之前逐字一致）----
         * 只在 [0°,90°] 标定是自洽的；风洞爬行（要覆盖 180°）请用 mode 2。 */
        float duty_flat  = (float)s_holding_regs[REG_AUTO_DUTY_FLAT];
        float slope_gain = (float)s_holding_regs[REG_AUTO_SLOPE_GAIN];
        duty_f = duty_flat + slope_gain * (1.0f - cos_theta);
    }

    /* 下限兜底：吸附力绝不能为零。
     * 查表模式下同样要过这一关 —— 表里可能被写进低于 DUTY_MIN 的值。 */
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
    float temp;
    int32_t t10;

    /* 状态与诊断信息先刷：即使 seqlock 读失败也要更新，
     * 否则"卡在 WARMUP"时上位机看到的状态是陈旧的，无法定位。 */
    s_holding_regs[REG_IMU_STATUS] = (uint16_t)ins_get_status();

    if (!INS_get_snapshot(&snap))
    {
        return;   /* 姿态数据正在更新，下轮再来 */
    }

    write_float32_le(s_holding_regs, REG_IMU_ROLL,     snap.roll);
    write_float32_le(s_holding_regs, REG_IMU_PITCH,    snap.pitch);
    write_float32_le(s_holding_regs, REG_IMU_YAW,      snap.yaw);
    write_float32_le(s_holding_regs, REG_IMU_GYRO_X,   snap.gyro_x);
    write_float32_le(s_holding_regs, REG_IMU_GYRO_Y,   snap.gyro_y);
    write_float32_le(s_holding_regs, REG_IMU_GYRO_Z,   snap.gyro_z);
    write_float32_le(s_holding_regs, REG_IMU_AUTO_DUTY, (float)fan_auto_duty);

    /* 实时总倾角 θ = acos(cos(pitch−off)·cos(roll−off))，单位度，范围 0~180。
     * **专为 LUT 标定加的**：标定时需要知道"现在摆到几度了"，
     * 只看 roll/pitch 自己在脑子里合算很容易摆错。
     * 零位偏置取法与 fan_auto_update() 一致，保证这里读到的 θ
     * 与控制器内部真正用的 θ 是同一个数（否则标定会系统性偏移）。 */
    {
        float p_off = snap.pitch - (float)(int16_t)s_holding_regs[REG_AUTO_PITCH_OFFSET];
        float r_off = snap.roll  - (float)(int16_t)s_holding_regs[REG_AUTO_ROLL_OFFSET];
        float ct = cosf(p_off * LUT_DEG2RAD) * cosf(r_off * LUT_DEG2RAD);
        if (ct < -1.0f) { ct = -1.0f; }
        if (ct >  1.0f) { ct =  1.0f; }
        write_float32_le(s_holding_regs, REG_IMU_THETA, acosf(ct) * LUT_RAD2DEG);
    }

    /* 姿态四元数，顺序 (w,x,y,z)、机体系→世界系。**原样上抛，不做任何转换**：
     * 没有万向节死锁（欧拉角的 pitch 被 asin 限死在 ±90°），信息也完整，
     * 上位机可自行换算成欧拉角/旋转矩阵/轴角，ROS2 更是原生就用四元数。
     * ⚠️ ROS2 的 geometry_msgs/Quaternion 字段顺序是 (x,y,z,w)，与本表**相反**。 */
    write_float32_le(s_holding_regs, REG_IMU_QUAT_W, snap.quat_w);
    write_float32_le(s_holding_regs, REG_IMU_QUAT_X, snap.quat_x);
    write_float32_le(s_holding_regs, REG_IMU_QUAT_Y, snap.quat_y);
    write_float32_le(s_holding_regs, REG_IMU_QUAT_Z, snap.quat_z);

    /* 温度 ×10 存为有符号整数（诊断用）。钳位到 int16 范围防溢出。 */
    temp = snap.temperature;
    t10 = (int32_t)(temp * 10.0f);
    if (t10 > 32767)  { t10 = 32767; }
    if (t10 < -32768) { t10 = -32768; }
    s_holding_regs[REG_IMU_TEMP_X10] = (uint16_t)(int16_t)t10;
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

/* ==================== CAN：HAL API + 中断队列（对齐 running can.c / freertos.c） ==================== */

/* 电机状态上报帧队列元素（与 running freertos.c 的 CanRxMsg_t 逐字段一致） */
typedef struct
{
    uint32_t id;
    uint8_t  data[8];
    uint8_t  dlc;
} CanRxMsg_t;

static CAN_HandleTypeDef hcan1;
static QueueHandle_t s_can_rx_queue = NULL;

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

    /* 位时间参数与 running（Core/Src/can.c）逐项一致：
     *   Prescaler=3 / SJW=1TQ / BS1=11TQ / BS2=2TQ / AutoRetransmission=ENABLE
     * PCLK1 = 42MHz，位时间 = 1+11+2 = 14TQ → 42MHz/(3*14) = 1.000 Mbps，采样点 85.7% */
    hcan1.Instance = CAN1;
    hcan1.Init.Prescaler = 3;
    hcan1.Init.Mode = CAN_MODE_NORMAL;
    hcan1.Init.SyncJumpWidth = CAN_SJW_1TQ;
    hcan1.Init.TimeSeg1 = CAN_BS1_11TQ;
    hcan1.Init.TimeSeg2 = CAN_BS2_2TQ;
    hcan1.Init.TimeTriggeredMode = DISABLE;
    hcan1.Init.AutoBusOff = DISABLE;
    hcan1.Init.AutoWakeUp = DISABLE;
    /* 保持自动重传开启，与 running 一致：总线上的偶发错误帧能自动补发，不丢关键命令。
     * 代价是"总线上无节点应答时硬件会无限重传并占满三个发送邮箱"，
     * 由 motor_can_send_frame() 里"邮箱满 → abort 三个 pending 邮箱"兜底。 */
    hcan1.Init.AutoRetransmission = ENABLE;
    hcan1.Init.ReceiveFifoLocked = DISABLE;
    hcan1.Init.TransmitFifoPriority = DISABLE;
    if (HAL_CAN_Init(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* 过滤器：32 位 IDMASK，全 0 掩码接收所有标准帧（与 running can.c 一致）。
     * SlaveStartFilterBank 只是给双 CAN（CAN2 起始 bank）用的提示，
     * 本芯片只用 CAN1，填 14 与 running 保持一致，无副作用。 */
    CAN_FilterTypeDef filter = {0};
    filter.FilterBank = 0U;
    filter.FilterMode = CAN_FILTERMODE_IDMASK;
    filter.FilterScale = CAN_FILTERSCALE_32BIT;
    filter.FilterIdHigh = 0x0000U;
    filter.FilterIdLow = 0x0000U;
    filter.FilterMaskIdHigh = 0x0000U;
    filter.FilterMaskIdLow = 0x0000U;
    filter.FilterFIFOAssignment = CAN_FILTER_FIFO0;
    filter.FilterActivation = CAN_FILTER_ENABLE;
    filter.SlaveStartFilterBank = 14U;
    if (HAL_CAN_ConfigFilter(&hcan1, &filter) != HAL_OK)
    {
        Error_Handler();
    }
    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    /* CAN1_RX0 中断优先级 5（与 running can.c 的 HAL_CAN_MspInit 一致）。
     * 本工程没有独立的 HAL_CAN_MspInit，GPIO/时钟已在上面手配，NVIC 在此补齐。
     * 中断服务函数 CAN1_RX0_IRQHandler 在 Src/stm32f4xx_it.c，
     * 内部调 HAL_CAN_IRQHandler → HAL_CAN_RxFifo0MsgPendingCallback。 */
    HAL_NVIC_SetPriority(CAN1_RX0_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(CAN1_RX0_IRQn);
}

static uint8_t motor_can_send_frame(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    CAN_TxHeaderTypeDef txh;
    uint32_t mb = 0U;

    if (len > 8U)
    {
        return 0U;
    }

    txh.StdId = std_id;
    txh.IDE = CAN_ID_STD;
    txh.RTR = CAN_RTR_DATA;
    txh.DLC = len;
    txh.TransmitGlobalTime = DISABLE;
    txh.ExtId = 0U;

    /* 与 running 的 CanTask_Send() 对齐（Core/Src/freertos.c:289）：
     * 保持自动重传开启（NART=0）后，总线上没有节点应答时硬件会无限重传并占满三个
     * 发送邮箱，因此邮箱满时必须能逃生。三步：
     *
     *   1) 先等待几个 tick，且用 vTaskDelay() 让出 CPU。
     *      本函数运行在最高优先级的 ModbusTask 里，空转会把它下面三个任务
     *      （Fan/Pulse/Led）全部饿死，RS485 也跟着不回应。
     *   2) 等待超时后中止三个 pending 邮箱，等价 HAL_CAN_AbortTxRequest()：
     *      这是自动重传下的唯一逃生通道（与 running 逐条对应）。
     *   3) 中止后仍无空闲邮箱则放弃本帧并返回 0，绝不无限等待。 */
    if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
    {
        uint8_t wait = 0U;
        while ((HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U) && (wait < MOTOR_CAN_TX_WAIT_TICKS))
        {
            vTaskDelay(1U);
            wait++;
        }
        if (HAL_CAN_GetTxMailboxesFreeLevel(&hcan1) == 0U)
        {
            /* 三个邮箱都还在重传，全部中止，优先保证当前关键帧能发出去。
             * 逐个判断 TME 位：只有"非空"的邮箱才需要 abort（与 running 写法一致）。 */
            if ((CAN1->TSR & CAN_TSR_TME0) == 0U) { (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX0); }
            if ((CAN1->TSR & CAN_TSR_TME1) == 0U) { (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX1); }
            if ((CAN1->TSR & CAN_TSR_TME2) == 0U) { (void)HAL_CAN_AbortTxRequest(&hcan1, CAN_TX_MAILBOX2); }
        }
    }

    return (HAL_CAN_AddTxMessage(&hcan1, &txh, (uint8_t *)data, &mb) == HAL_OK) ? 1U : 0U;
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
        /* ⚠️ 必须在这里显式置脏标志：本分支**提前 return**，走不到函数末尾那段
         * "if (is_param_region(addr)) s_param_dirty = 1U"。
         * 2026-09-15 之前这个分支一直没有置位 → 风机自动参数**从不落盘**，
         * 复位后 FAN_MODE 回 0、风机全停。现已纳入断电保持区，特此补上。 */
        s_param_dirty = 1U;
        s_param_dirty_tick_ms = HAL_GetTick();
        fan_auto_update();
        return;
    }

    /* LUT 查表区 0x0160~0x016D：可写，写完立刻重算一次输出，
     * 让调表时能马上看到效果（与自动参数区行为一致）。
     * 同样要显式置脏标志 —— 本分支也提前 return。
     * （0x016E/0x016F 是只读的 TILT_THETA，已在上面 is_imu_status_region 拦掉。） */
    if ((addr >= REG_LUT_BASE) && (addr <= REG_LUT_MAGIC))
    {
        s_holding_regs[addr] = value;
        s_param_dirty = 1U;
        s_param_dirty_tick_ms = HAL_GetTick();
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
            /* 边界检查三段：
             *   ① qty==0 或 起始+数量 超表 → 非法数据地址(0x02)
             *   ② **qty > 125** → 非法数据值(0x03)。
             *      Modbus 规定 0x03/0x04 单次最多读 125 个寄存器（响应字节计数上限 250）。
             *      旧实现只判 ①，若上位机一次读 >125（例如读 200 个），
             *      下面 `response[2] = (uint8_t)(qty*2)` 会**回绕**（200*2=400 → (uint8_t)400=144），
             *      且响应体 3+2*200+2 = 405 字节**远超 response[128] → 栈/静态缓冲越界写**，
             *      上位机收到的就是长度不符的短包/脏包 → 报 Insufficient bytes received。
             *      这是"怎么调都失败"的典型来源之一。 */
            if ((qty == 0U) || ((uint32_t)start + (uint32_t)qty > MODBUS_REG_COUNT))
            {
                return build_exception(slave, func, 0x02U, response);
            }
            if (qty > 125U)
            {
                return build_exception(slave, func, 0x03U, response);
            }
            response[0] = slave;
            response[1] = func;
            response[2] = (uint8_t)(qty * 2U);   /* 现在 qty<=125，乘 2 最大 250，不会回绕 */
            uint16_t pos = 3U;
            /* 短临界区（对齐 running Regs_HoldingSnapshot）：
             * 多字 float32 由两个寄存器组成，若不加锁，
             * CAN RX 中断（优先级 5）可能在本轮复制中途更新后半字，
             * 上位机就会读到"半新半旧"的撕裂值（表现为速度/位置偶发跳变）。
             * 逐字节接收已在任务上下文，复制很快，临界区开销可忽略。 */
            taskENTER_CRITICAL();
            for (uint16_t i = 0U; i < qty; ++i)
            {
                uint16_t v = s_holding_regs[start + i];
                response[pos++] = (uint8_t)((v >> 8U) & 0xFFU);
                response[pos++] = (uint8_t)(v & 0xFFU);
            }
            taskEXIT_CRITICAL();
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

    /* ---- 风机查表（LUT）模式默认值 ----
     * ⚠️ 这 13 个值是**理论模型算出来的占位值，没有任何实测依据**
     *（见 s_duty_lut_default 的说明与 `标定手册.md` §8.9）。
     * 现在填进去是因为暂时没有大角度/倒立工装，先用模型值把链路跑通；
     * 等小车能跑后逐点标定、用 Modbus 覆盖这 13 格即可，**固件不用改**。
     *
     * 魔数一并置位，让 FAN_MODE=2 **开箱即可用**（否则要先写魔数才生效，
     * 多一道无谓的手续）。安全上可接受：占位值全部 ≥ DUTY_MIN，
     * 即使模型不准也只是"吸力偏大/偏小"，不会出现吸附力归零。
     * 若想改成"必须显式标定后才允许查表"，把下面这行改成 = 0U 即可。 */
    for (uint16_t i = 0U; i < REG_LUT_COUNT; ++i)
    {
        s_holding_regs[REG_LUT_BASE + i] = s_duty_lut_default[i];
    }
    s_holding_regs[REG_LUT_MAGIC] = REG_LUT_MAGIC_VAL;

    MX_CAN1_Init();
    retentive_load_params();
    refresh_fan_state_registers();

    /* CAN RX 队列 + 中断使能（与 running CanTask_Entry 一致）：
     * 先建队列再激活通知，避免中断在队列就绪前触发导致丢帧。 */
    if (s_can_rx_queue == NULL)
    {
        s_can_rx_queue = xQueueCreate(16U, sizeof(CanRxMsg_t));
    }
    if (s_can_rx_queue != NULL)
    {
        if (HAL_CAN_ActivateNotification(&hcan1, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
        {
            Error_Handler();
        }
    }

    s_rx_head = 0U;
    s_rx_tail = 0U;
    s_tx_buf = NULL;
    s_tx_len = 0U;
    s_tx_pos = 0U;

    /* 按 0x0090 枚举查表重配 USART6（对齐 running UartTask_Entry 第 3 步）。
     * 必须放在 retentive_load_params() 之后、开接收中断之前 ——
     * 若在 HAL_UART_Init 之前就开了 RXNE，重配过程中会收到脏字节。
     * 改动下次断电重启生效（与 running 语义一致）。 */
    const uart_cfg_t *cfg = uart_cfg_lookup(s_holding_regs[REG_PARAM_UART_CFG]);
    MX_USART6_RS485_UART_Init_With(cfg->baud, cfg->parity);

    /* 接收中断由我们自管（不经过 HAL），先确保 RXNE/TC/TXE 都关，清残留标志，
     * 再打开 RXNE 开始接收。HAL_UART_Init 内部已配好波特率/帧格式，此处只管中断。 */
    CLEAR_BIT(USART6->CR1, USART_CR1_RXNEIE | USART_CR1_TXEIE | USART_CR1_TCIE);
    rs485_rx_enable(1U);
}

void modbus_poll(void)
{
    static uint8_t frame[64];
    static uint16_t frame_len = 0U;
    static uint32_t last_rx_ms = 0U;

    retentive_poll();
    motor_can_poll_rx();
    motor_can_tick();

    /* 上一帧响应若还在中断驱动发送中，先等它发完（最多 20ms）。
     * 必要性：s_tx_buf 指向 static response[]，若不等发完就处理下一帧，
     * 新响应会覆写同一块缓冲，正在移出的字节就变成脏数据。
     * Modbus 是一问一答，正常情况下早就发完了，这里只是兜底。
     * 发送周期 5ms，一帧 8~40 字节约 0.7~3.5ms @115200，20ms 上限绰绰有余。 */
    (void)modbus_tx_wait_done(MODBUS_RTU_TIMEOUT_MS);

    /* 姿态前馈自动占空比：每轮 poll(5ms) 更新一次。
     * 放在 ModbusTask 而非 InsTask 的原因：
     *   - 风机输出是"执行器动作"，本就该由唯一的输出任务统一决策，避免两处竞态；
     *   - InsTask 只负责算姿态并发布快照，职责单一；
     *   - 5ms 刷新率对风洞爬行（速度很慢）而言已远超需求。 */
    fan_auto_update();

    /* 风机状态区无条件每轮刷新（放在这里而不是 fan_auto_update() 内部的原因：
     * 后者有"非自动模式"和"姿态快照读失败"两条提前 return 路径，
     * 把刷新挂在里面会漏掉这两条；而状态反馈应当始终反映真实输出，
     * 与当前处于哪种模式无关。见 refresh_fan_state_registers() 的说明。 */
    refresh_fan_state_registers();

    refresh_imu_registers();

    /* 加热 PWM 是只读诊断量，每轮刷新（不参与参数区的读写） */
    s_holding_regs[REG_AUTO_HEATER_PWM] = ins_get_heater_pwm();

    /* 目标温度也做成本工程内的「真值镜像」。
     * 坑：REG_AUTO_HEATER_TARGET(0x014D) 在 modbus_init 的默认参数表里**没有被初始化**，
     * 也从不与 ins_task 内的 s_target_temp 同步 → 上电后这一格恒读 0，
     * 而上位机文档写的是 45.0f，会让人误判"温控目标没设上"。
     * 这里每轮从 ins_get_target_temp() 回写，保证：
     *   ① 上电即显示真实目标（45.0f）；
     *   ② 上位机写 0x014D → ins_set_target_temp() 改的是同一份真值，
     *      下一轮镜像自然跟上，不会打架。 */
    write_float32_le(s_holding_regs, REG_AUTO_HEATER_TARGET, ins_get_target_temp());

    while (ring_len() > 0U)
    {
        uint8_t byte = 0U;
        if (!ring_pop(&byte))
        {
            break;
        }
        frame[frame_len++] = byte;
        last_rx_ms = HAL_GetTick();

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
                        /* response 必须是 static：modbus_send_frame 走中断驱动发送，
                         * 函数返回后若响应还在栈上就会被后续函数调用覆盖。
                         * 同一时刻只有一帧在发（Modbus 一问一答），无需加锁。
                         *
                         * 大小 = 256：0x03/0x04 最坏 125 个寄存器 →
                         *   1(站号)+1(功能码)+1(字节数)+250(数据)+2(CRC) = 255 字节。
                         *   （旧值 128 在 qty>62 时会越界写 —— 已配合上面的
                         *     qty<=125 检查一起修掉。）0x10 写响应 8 字节、异常响应 5 字节，都在内。 */
                        static uint8_t response[256];
                        uint16_t response_len = process_request(frame, total_len, response);
                        if (response_len > 0U)
                        {
                            modbus_send_frame(response, response_len);
                        }
                        memset(frame, 0, sizeof(frame));
                        frame_len = 0U;
                        continue;
                    }

                    /* CRC 校验失败：说明这一帧从一开始就错位了（多了/少了字节，
                     * 或掺进了脏数据）。旧实现只丢 1 个首字节然后立刻重判，
                     * 在"持续错位"时（如首字节丢失导致后续全部左移）会一直错下去，
                     * 永远解不出帧，表现就是上位机持续报 Insufficient bytes received。
                     *
                     * 改为**整帧丢弃**：Modbus RTU 靠 3.5 字符静默间隔分帧，
                     * 下一帧到来时天然是干净的，重新同步最省事也最可靠。
                     * 注意：丢弃后下面 while 会继续 pop，但 frame_len 已归零，
                     * 相当于把当前残余全部当作无效数据抛掉。 */
                    memset(frame, 0, sizeof(frame));
                    frame_len = 0U;
                    continue;
                }
            }
        }
    }

    /* 帧间静默超时（Modbus RTU 的 3.5 字符间隔）。
     * 115200 8N1 下一个字符约 87us，3.5 字符 ≈ 305us；取 4ms 留足余量，
     * 远小于 Modbus Poll 默认 1000ms 的扫描周期，不会误切正常帧。
     *
     * 为什么必须加：轮询周期 5ms，一帧 8 字节传输只要约 0.7ms。
     * 若上一帧因干扰残缺（如只收到 3 字节就断了），残余会一直留在 frame 里，
     * 等下一次请求到来时与前半截拼在一起 → frame[1] 不再是功能码 → 永远 CRC 错。
     * 有了超时，残缺帧会在下一次请求到来之前被清掉。 */
    if ((frame_len > 0U) && ((HAL_GetTick() - last_rx_ms) > MODBUS_FRAME_GAP_MS))
    {
        memset(frame, 0, sizeof(frame));
        frame_len = 0U;
    }
}

/* RX 帧处理：从队列取帧，在任务上下文更新寄存器（与 running CanTask 的 RX 分支一致）。
 * 中断回调只负责"把帧搬进队列"，不碰寄存器 —— 这样避免中断里长时间持锁，
 * 也保证 s_holding_regs 只在任务上下文被写（Modbus 读写同任务，天然串行）。 */
static void motor_can_poll_rx(void)
{
    if (s_can_rx_queue == NULL)
    {
        return;
    }

    CanRxMsg_t rx;
    while (xQueueReceive(s_can_rx_queue, &rx, 0U) == pdPASS)
    {
        motor_status_update_from_can(rx.id, rx.data, rx.dlc);
    }
}

/* CAN RX0 中断回调（由 HAL 在 CAN1_RX0_IRQHandler → HAL_CAN_IRQHandler 中调用）。
 * 与 running freertos.c:515 一致：**只把帧搬进队列**，寄存器写入留给任务上下文。
 * 队列未就绪（调度器启动前）直接丢弃，避免空指针。 */
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
    if ((hcan->Instance != CAN1) || (s_can_rx_queue == NULL))
    {
        return;
    }

    CanRxMsg_t msg;
    CAN_RxHeaderTypeDef hdr;
    if (HAL_CAN_GetRxMessage(hcan, CAN_RX_FIFO0, &hdr, msg.data) == HAL_OK)
    {
        msg.id  = hdr.StdId;
        msg.dlc = (uint8_t)hdr.DLC;
        BaseType_t higher_woken = pdFALSE;
        (void)xQueueSendFromISR(s_can_rx_queue, &msg, &higher_woken);
        portYIELD_FROM_ISR(higher_woken);
    }
}

/* CAN1_RX0 中断服务入口，供 Src/stm32f4xx_it.c 调用。
 * 走 HAL，与 running 一致（running 的 CanTask 用 HAL_CAN_ActivateNotification）。 */
void motor_can_irq_handler(void)
{
    HAL_CAN_IRQHandler(&hcan1);
}

/* 说明：本工程 RS485 收发**不使用 HAL UART 中断状态机**，
 * 因此没有 HAL_UART_RxCpltCallback / HAL_UART_ErrorCallback。
 * 取而代之的是下方 usart6_irq_handler()——在 USART6 中断里直接读 SR/DR，
 * 每次中断都顺手清 ORE/FE/NE/PE。这样从架构上就不存在
 * "HAL 关掉 RXNE 中断后接收永久停摆"的可能，与 running 的 portserial.c 一致。
 *
 * 历史教训（2026-09-10）：曾用 HAL_UART_Receive_IT 逐字节接收 + 补 HAL_UART_ErrorCallback
 * 的方式打补丁，但根子在于 HAL 状态机本身脆弱，故整体改为裸寄存器方案。 */

