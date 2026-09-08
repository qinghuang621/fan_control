#include "bsp_modbus.h"

#include "main.h"
#include "usart.h"
#include "bsp_fric.h"
#include "bsp_pulse.h"

#include <string.h>

#define MODBUS_SLAVE_ADDR 0x01U
#define MODBUS_RX_RING_SIZE 128U
#define MODBUS_REG_COUNT 256U
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
#define REG_MOTOR_COUNT 4U
#define REG_MOTOR_W_MAX 30.0f
#define REG_MOTOR_LX 0.15f
#define REG_MOTOR_LY 0.15f
#define REG_MOTOR_R 0.05f
#define REG_FAN_DUTY_BASE 0x0100U
#define REG_FAN_CNT 4U
#define REG_CAN_STATUS_BASE 0x0040U
#define REG_CAN_STATUS_END 0x008FU
#define REG_STATUS_STRIDE 10U
#define REG_PARAM_BASE 0x0090U
#define REG_PARAM_END 0x00BFU
#define REG_PARAM_UART_CFG 0x0090U
#define REG_PARAM_SLAVE_ADDR 0x0091U

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
    const uint16_t motor_status_base[REG_FAN_CNT] = {
        REG_MOTOR_STATUS_M1,
        REG_MOTOR_STATUS_M2,
        REG_MOTOR_STATUS_M3,
        REG_MOTOR_STATUS_M4
    };

    for (uint16_t i = 0U; i < REG_FAN_CNT; ++i)
    {
        uint16_t status_base = motor_status_base[i];
        uint16_t duty = clamp_duty(s_holding_regs[REG_FAN_DUTY_BASE + i]);
        uint32_t rpm = pulse_get_rpm((uint8_t)(i + 1U), 2U);
        uint32_t pulse_cnt = pulse_get_pulse_count((uint8_t)(i + 1U));

        /* keep running 项目定义的状态布局：
         * +0 ERR    : 运行状态/错误码（这里用 0/1 展示使能状态）
         * +1 保留   : 0
         * +2/+3 POS : 浮点位置反馈（采用 pulse count 近似）
         * +4/+5 VEL : 浮点速度反馈（RPM）
         * +6/+7 T   : 浮点扭矩估计（占空比）
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

static void motor_kinematics_resolve(void)
{
    float vy = read_float32_regs(s_holding_regs, 0x0014U);
    float vx = read_float32_regs(s_holding_regs, 0x0016U);
    float wz = read_float32_regs(s_holding_regs, 0x0018U);
    float half_ly = 0.15f / 2.0f;
    float half_lx = 0.15f / 2.0f;
    float inv_r = 1.0f / 0.05f;
    float w_lf = (vx + half_ly * wz) * inv_r;
    float w_rf = (vy + half_lx * wz) * inv_r;
    float w_rr = -(vx - half_ly * wz) * inv_r;
    float w_lr = -(vy - half_lx * wz) * inv_r;
    float vals[REG_MOTOR_COUNT] = {w_lf, w_rf, w_rr, w_lr};
    float max_val = 0.0f;

    for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
    {
        if (vals[i] < 0.0f)
        {
            vals[i] = -vals[i];
        }
        if (vals[i] > max_val)
        {
            max_val = vals[i];
        }
    }

    if (max_val > REG_MOTOR_W_MAX)
    {
        float scale = REG_MOTOR_W_MAX / max_val;
        for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
        {
            vals[i] *= scale;
        }
    }

    write_float32_regs(s_holding_regs, 0x000AU, vals[0]);
    write_float32_regs(s_holding_regs, 0x000CU, vals[1]);
    write_float32_regs(s_holding_regs, 0x000EU, vals[2]);
    write_float32_regs(s_holding_regs, 0x0010U, vals[3]);
}

static void MX_CAN1_Init(void)
{
    __HAL_RCC_CAN1_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();

    GPIO_InitTypeDef gpio_init = {0};
    gpio_init.Pin = GPIO_PIN_11 | GPIO_PIN_12;
    gpio_init.Mode = GPIO_MODE_AF_PP;
    gpio_init.Pull = GPIO_NOPULL;
    gpio_init.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
    gpio_init.Alternate = GPIO_AF9_CAN1;
    HAL_GPIO_Init(GPIOA, &gpio_init);

    CAN1->MCR |= CAN_MCR_INRQ;
    while ((CAN1->MSR & CAN_MSR_INAK) == 0U)
    {
    }

    CAN1->MCR |= CAN_MCR_ABOM | CAN_MCR_AWUM | CAN_MCR_TXFP;
    CAN1->BTR = (uint32_t)((0U << 30U) |
                           (3U << 16U) |
                           (13U << 8U) |
                           (2U << 4U) |
                           0U);

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

static uint8_t motor_can_send_frame(uint32_t std_id, const uint8_t *data, uint8_t len)
{
    uint8_t mailbox = 0U;

    if (len > 8U)
    {
        return 0U;
    }

    while (((CAN1->TSR & CAN_TSR_TME0) == 0U) && ((CAN1->TSR & CAN_TSR_TME1) == 0U) && ((CAN1->TSR & CAN_TSR_TME2) == 0U))
    {
    }

    if ((CAN1->TSR & CAN_TSR_TME0) != 0U)
    {
        mailbox = 0U;
    }
    else if ((CAN1->TSR & CAN_TSR_TME1) != 0U)
    {
        mailbox = 1U;
    }
    else
    {
        mailbox = 2U;
    }

    CAN1->sTxMailBox[mailbox].TIR = ((std_id & 0x7FFU) << 21U) | CAN_TI0R_TXRQ;
    CAN1->sTxMailBox[mailbox].TDTR = len;
    CAN1->sTxMailBox[mailbox].TDLR = (uint32_t)data[0] | ((uint32_t)data[1] << 8U) | ((uint32_t)data[2] << 16U) | ((uint32_t)data[3] << 24U);
    CAN1->sTxMailBox[mailbox].TDHR = (uint32_t)data[4] | ((uint32_t)data[5] << 8U) | ((uint32_t)data[6] << 16U) | ((uint32_t)data[7] << 24U);
    return 1U;
}

static void motor_can_send_speed(uint16_t motor_index, float speed)
{
    uint8_t payload[8] = {0};
    union
    {
        float f;
        uint32_t u32;
    } conv;

    conv.f = speed;
    payload[0] = (uint8_t)(conv.u32 & 0xFFU);
    payload[1] = (uint8_t)((conv.u32 >> 8U) & 0xFFU);
    payload[2] = (uint8_t)((conv.u32 >> 16U) & 0xFFU);
    payload[3] = (uint8_t)((conv.u32 >> 24U) & 0xFFU);
    payload[4] = 0U;
    payload[5] = 0U;
    payload[6] = 0U;
    payload[7] = 0U;
    motor_can_send_frame((uint32_t)(0x201U + motor_index), payload, 8U);
}

static void motor_can_tick(void)
{
    static uint32_t last_tick_ms = 0U;
    if ((HAL_GetTick() - last_tick_ms) < 10U)
    {
        return;
    }
    last_tick_ms = HAL_GetTick();

    for (uint16_t i = 0U; i < REG_MOTOR_COUNT; ++i)
    {
        uint16_t enable_addr = (uint16_t)(REG_MOTOR_EN_BASE + i);
        uint16_t clear_addr = (uint16_t)(REG_MOTOR_CLR_BASE + i);
        float speed = read_float32_regs(s_holding_regs, (uint16_t)(REG_MOTOR_VEL_BASE + i * 2U));

        if (s_holding_regs[enable_addr] != 0U)
        {
            uint8_t payload[8] = {0U};
            payload[0] = (uint8_t)i;
            payload[1] = 0x01U;
            motor_can_send_frame((uint32_t)(0x200U + i + 1U), payload, 8U);
        }

        if (s_holding_regs[clear_addr] != 0U)
        {
            uint8_t payload[8] = {0U};
            payload[0] = (uint8_t)i;
            payload[1] = 0x02U;
            motor_can_send_frame((uint32_t)(0x200U + i + 1U), payload, 8U);
            s_holding_regs[clear_addr] = 0U;
        }

        motor_can_send_speed(i, speed);
    }
}

static void motor_status_update_from_can(uint32_t std_id, const uint8_t *data)
{
    if (std_id < 0x201U || std_id > 0x204U)
    {
        return;
    }

    uint16_t motor_index = (uint16_t)(std_id - 0x201U);
    uint16_t status_base = (uint16_t)(REG_MOTOR_STATUS_M1 + motor_index * REG_MOTOR_STATUS_STRIDE);
    union
    {
        float f;
        uint32_t u32;
    } conv;

    if (data[0] != 0U)
    {
        s_holding_regs[status_base + 0U] = (uint16_t)data[0];
    }
    s_holding_regs[status_base + 1U] = 0U;

    conv.u32 = (uint32_t)data[4U] << 24U | (uint32_t)data[5U] << 16U | (uint32_t)data[6U] << 8U | (uint32_t)data[7U];
    write_float32_regs(s_holding_regs, status_base + 2U, conv.f);
    conv.u32 = (uint32_t)data[0U] << 24U | (uint32_t)data[1U] << 16U | (uint32_t)data[2U] << 8U | (uint32_t)data[3U];
    write_float32_regs(s_holding_regs, status_base + 4U, conv.f);
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

    if ((addr >= REG_MOTOR_EN_BASE) && (addr <= (REG_MOTOR_WZ_HI + 1U)))
    {
        s_holding_regs[addr] = value;
        if ((addr >= REG_MOTOR_VY_HI) && (addr <= REG_MOTOR_WZ_HI))
        {
            motor_kinematics_resolve();
        }
        return;
    }

    if (addr >= REG_FAN_DUTY_BASE && addr <= (REG_FAN_DUTY_BASE + REG_FAN_CNT - 1U))
    {
        s_holding_regs[addr] = value;
        sync_fan_outputs_from_regs();
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
        motor_status_update_from_can(std_id, rx_data);
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
