/**
 * @file    bsp_led_tilt.c
 * @brief   把 C 板 RGB LED 映射为"倾斜方向 + 系统状态"指示
 *
 * 设计原则（沿用 661c2-main/18.ins_task 的 led_tilt_task.c，v3）：
 *   1) 色相(hue) 永远表示倾斜方向 —— 任何时候都能看出"往哪歪"
 *   2) 饱和度(sat) 表示倾斜幅度
 *   3) 亮度(val) 承载系统状态 —— 不再用颜色去覆盖姿态信息
 *        · 加热未完成   → 1Hz 呼吸（亮度起伏，越接近目标温度谷底越高）
 *        · 加热完成     → 常亮
 *        · IMU 未就绪   → 暗红慢呼吸 / 红色常亮（真故障）
 *
 * 与原例程的差异（本工程接口不同，必须适配）：
 *   - 例程快照里的 status 是位标志（INS_STATUS_RUNNING 等），本工程
 *     ins_status_e 是枚举，因此状态判断改用 ins_get_status()；
 *   - 例程快照带 seq 用于"数据流存活检测"，本工程快照无 seq，改用
 *     INS_get_snapshot() 的返回值（seqlock 读失败率）间接判断；
 *   - 不建独立任务，由 LedTask 以 5ms 周期调用 led_tilt_update()
 *     （LED_TILT_PERIOD_MS 内部累加限流到 20ms，实际 50Hz 刷新）。
 *
 * ⚠️ 与彩虹呼吸灯互斥：两者都写 TIM5 CCR1/2/3，只能二选一。
 *    本工程已移除 led_tick()，LedTask 只调本模块。
 */

#include "bsp_led_tilt.h"
#include "bsp_led.h"
#include "ins_task.h"
#include <math.h>

#define RAD_TO_DEG_L   57.2957795f
#define TWO_PI_L       6.28318531f

/* TIM5 通道 ↔ LED 颜色（与 bsp_led.c 的 aRGB_led_show 保持一致）：
 *   CH1 = PH10 = 蓝
 *   CH2 = PH11 = 绿
 *   CH3 = PH12 = 红 */
#define LED_TILT_R_CH   TIM_CHANNEL_3
#define LED_TILT_G_CH   TIM_CHANNEL_2
#define LED_TILT_B_CH   TIM_CHANNEL_1

/* ---------- HSV -> RGB (0..1) ---------- */
static void hsv_to_rgb01(float h_deg, float s, float v,
                         float *r, float *g, float *b)
{
    float c, x, m, r1, g1, b1;

    if (s < 0.f) s = 0.f; else if (s > 1.f) s = 1.f;
    if (v < 0.f) v = 0.f; else if (v > 1.f) v = 1.f;
    h_deg = fmodf(h_deg, 360.f);
    if (h_deg < 0.f) h_deg += 360.f;

    c = v * s;
    x = c * (1.f - fabsf(fmodf(h_deg / 60.f, 2.f) - 1.f));
    m = v - c;
    if      (h_deg <  60.f) { r1 = c; g1 = x; b1 = 0; }
    else if (h_deg < 120.f) { r1 = x; g1 = c; b1 = 0; }
    else if (h_deg < 180.f) { r1 = 0; g1 = c; b1 = x; }
    else if (h_deg < 240.f) { r1 = 0; g1 = x; b1 = c; }
    else if (h_deg < 300.f) { r1 = x; g1 = 0; b1 = c; }
    else                    { r1 = c; g1 = 0; b1 = x; }
    *r = r1 + m;
    *g = g1 + m;
    *b = b1 + m;
}

static inline float clampf_lohi(float v, float lo, float hi)
{
    return v < lo ? lo : (v > hi ? hi : v);
}

/**
 * @brief 算出"纯姿态色"（不含任何系统状态指示）
 * @note  色相 = atan2(pitch, roll)，即倾斜方向：
 *          pitch>0, roll=0 (前倾) -> hue  90 (黄绿)
 *          pitch=0, roll>0 (右倾) -> hue   0 (红)
 *          pitch<0, roll=0 (后倾) -> hue 270 (紫)
 *          pitch=0, roll<0 (左倾) -> hue 180 (青)
 */
static void tilt_color(float pitch, float roll, uint32_t tick_ms,
                       float *r, float *g, float *b)
{
    float mag = sqrtf(pitch * pitch + roll * roll);
    float hue, sat;

    if (mag < LED_TILT_DEAD_ZONE_DEG) {
        /* 死区：暗绿 OK 指示 */
        *r = 0.f; *g = 0.20f; *b = 0.06f;
        return;
    }

    hue = atan2f(pitch, roll) * RAD_TO_DEG_L;
    sat = clampf_lohi(mag / LED_TILT_FULL_SAT_DEG, 0.f, 1.f);

    if (mag >= LED_TILT_WARN_DEG) {
        /* >=30°：5Hz 快闪告警（色相不变，只闪亮度） */
        float flash = ((tick_ms / 100U) & 1U) ? 1.f : 0.30f;
        hsv_to_rgb01(hue, sat, flash, r, g, b);
    } else if (mag >= LED_TILT_OK_DEG) {
        hsv_to_rgb01(hue, sat, 1.f, r, g, b);
    } else {
        /* 轻倾斜：降低饱和度，避免颜色过艳干扰判断 */
        hsv_to_rgb01(hue, sat * 0.55f + 0.15f, 1.f, r, g, b);
    }
}

static void led_set_rgb01(float r, float g, float b)
{
    r = clampf_lohi(r, 0.f, 1.f);
    g = clampf_lohi(g, 0.f, 1.f);
    b = clampf_lohi(b, 0.f, 1.f);
    aRGB_led_show(((uint32_t)255U << 24)   |
                  ((uint32_t)(r * 255.f) << 16) |
                  ((uint32_t)(g * 255.f) <<  8) |
                  ((uint32_t)(b * 255.f) <<  0));
}

/**
 * @brief 按快照与状态渲染 LED
 * @param snap_ok 本帧 seqlock 是否一次读成功（仅用于诊断，数据已保证有效）
 */
static void led_tilt_render(const ins_snapshot_t *s, ins_status_e st,
                            uint32_t tick_ms, uint8_t snap_ok)
{
    float r = 0.f, g = 0.f, b = 0.f;

    (void)snap_ok;

    /* ---- 硬故障 / 尚未就绪 ---- */
    if (st == INS_STATUS_ERROR) {
        /* IMU 读取连续失败：红色常亮（最醒目的故障指示） */
        led_set_rgb01(1.f, 0.f, 0.f);
        return;
    }
    if (st == INS_STATUS_OFFLINE) {
        /* 上电初期，还没出第一帧解算结果：暗红慢呼吸 = 等待中 */
        float phase = (tick_ms % 2000U) / 2000.f;
        float v = 0.12f + 0.18f * (0.5f - 0.5f * cosf(phase * TWO_PI_L));
        led_set_rgb01(v, 0.f, 0.f);
        return;
    }
    if (st == INS_STATUS_WARMUP) {
        /* 恒温加热中（姿态已可读但尚未对外）：偏青色慢呼吸，与 OFFLINE 的暗红区分 */
        float phase = (tick_ms % 1000U) / 1000.f;
        float v = 0.25f + 0.35f * (0.5f - 0.5f * cosf(phase * TWO_PI_L));
        led_set_rgb01(0.f, v, v);
        return;
    }

    /* ---- 基础色 = 姿态色（方向 + 幅度） ---- */
    tilt_color(s->pitch, s->roll, tick_ms, &r, &g, &b);
    led_set_rgb01(r, g, b);
}

/**
 * @brief 周期调用（建议 5ms 一次），内部限流到 LED_TILT_PERIOD_MS 刷新
 */
void led_tilt_update(void)
{
    static uint32_t tick_ms = 0U;
    static uint32_t accum_ms = 0U;
    static ins_snapshot_t last_snap = {0};   /* 上一帧有效快照，读取失败时沿用 */
    ins_snapshot_t snap;
    ins_status_e   st;
    uint8_t        ok;

    accum_ms += LED_TILT_TASK_CALL_PERIOD_MS;
    if (accum_ms < LED_TILT_PERIOD_MS) {
        return;
    }
    accum_ms = 0U;
    tick_ms += LED_TILT_PERIOD_MS;

    st = ins_get_status();
    ok = INS_get_snapshot(&snap);
    if (ok) {
        last_snap = snap;
    } else {
        /* seqlock 读失败：本帧数据不完整，沿用它上次的有效值，
         * 避免把未初始化的 snap 内容画到灯上 */
        snap = last_snap;
    }

    led_tilt_render(&snap, st, tick_ms, ok);
}
