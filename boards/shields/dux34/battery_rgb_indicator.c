/*
 * battery_rgb_indicator.c
 *
 * 电池状态 RGB LED 指示灯（D1 / P0.06 / WS2812）
 * 通过 ZMK RGB Underglow API 控制 LED（与 ZMK underglow 子系统协同，不直接操作硬件）
 *
 * 状态逻辑：
 *   充电中 + 电量 < 100%  → 黄色呼吸灯（慢速正弦渐变）
 *   充电中 + 电量 = 100%  → 绿色常亮
 *   未充电 + 电量 < 90%   → 红色闪烁（0.5s 周期）
 *   其他                  → 熄灭（省电）
 *
 * MCU:   NRF52840 (Nice!Nano v2)
 * LED:   WS2812-compatible, SPI3 MOSI = D1 (P0.06)
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/rgb_underglow.h>
#include <zmk/usb.h>

LOG_MODULE_REGISTER(battery_rgb, CONFIG_ZMK_LOG_LEVEL);

/* ─── 用户可调参数 ──────────────────────────────────────────── */

/** 低电量阈值（%），低于此值时红色闪烁 */
#define LOW_BATT_THRESHOLD  90

/** 红色闪烁半周期（ms）: 亮 500ms / 灭 500ms */
#define FLASH_HALF_MS       500

/** 呼吸灯每步间隔（ms）: 64步 × 30ms ≈ 1.9s 一次完整呼吸 */
#define BREATHE_STEP_MS     30

/** LED 亮度（0–100，ZMK HSB 的 B 分量） */
#define MAX_BRIGHTNESS      70

/* ─── ZMK HSB 颜色定义 ─────────────────────────────────────── */
/* zmk_led_hsb: h=色相(0-360), s=饱和度(0-100), b=亮度(0-100)  */

/* 绿色：充满电 */
#define COLOR_GREEN  ((struct zmk_led_hsb){.h = 120, .s = 100, .b = MAX_BRIGHTNESS})

/* 红色：低电量 */
#define COLOR_RED    ((struct zmk_led_hsb){.h = 0,   .s = 100, .b = MAX_BRIGHTNESS})

/* 黄色：充电中（暖黄，色相约 45°） */
#define COLOR_YELLOW ((struct zmk_led_hsb){.h = 45,  .s = 100, .b = MAX_BRIGHTNESS})

/* 熄灭（亮度为 0） */
#define COLOR_OFF    ((struct zmk_led_hsb){.h = 0,   .s = 0,   .b = 0})

/* ─── 正弦呼吸亮度查找表（64步，0→MAX→0，一个完整周期）─── */
/* 公式：round(127.5 * (1 - cos(2π * i / 64))) → 映射到 0-100  */
static const uint8_t breathe_lut[64] = {
      0,   0,   1,   2,   4,   6,   8,  11,   /*  0- 7 */
     15,  18,  22,  26,  31,  35,  40,  45,   /*  8-15 */
     50,  55,  60,  64,  69,  73,  78,  82,   /* 16-23 */
     85,  88,  91,  94,  96,  98,  99, 100,   /* 24-31 */
    100,  99,  98,  96,  94,  91,  88,  85,   /* 32-39 */
     82,  78,  73,  69,  64,  60,  55,  50,   /* 40-47 */
     45,  40,  35,  31,  26,  22,  18,  15,   /* 48-55 */
     11,   8,   6,   4,   2,   1,   0,   0,   /* 56-63 */
};

/* ─── 状态变量 ─────────────────────────────────────────────── */
static uint8_t  batt_level  = 100;   /* 当前电量百分比 */
static bool     usb_powered = false; /* USB 已接入 */
static bool     flash_lit   = false; /* 闪烁状态 */
static uint8_t  breathe_idx = 0;     /* 呼吸步进索引 */

static struct k_work_delayable indicator_work;

/* ─── 工作项处理 ─────────────────────────────────────────────*/

static void indicator_work_handler(struct k_work *work)
{
    if (usb_powered) {
        if (batt_level >= 100) {
            /* 充满电：绿色常亮，设置后不再重调度 */
            zmk_rgb_underglow_set_hsb(COLOR_GREEN);
            zmk_rgb_underglow_on();
            LOG_DBG("RGB: green solid (fully charged)");
            return;
        }

        /* 充电中：黄色呼吸灯 */
        uint8_t luma = breathe_lut[breathe_idx];
        breathe_idx = (breathe_idx + 1U) % 64U;

        struct zmk_led_hsb color = COLOR_YELLOW;
        color.b = luma; /* 用查找表动态调节亮度 */
        zmk_rgb_underglow_set_hsb(color);
        zmk_rgb_underglow_on();

        LOG_DBG("RGB: yellow breathe step=%u luma=%u", breathe_idx, luma);
        k_work_schedule(&indicator_work, K_MSEC(BREATHE_STEP_MS));
        return;
    }

    /* ── 未插 USB ── */
    if (batt_level < LOW_BATT_THRESHOLD) {
        /* 低电量：红色闪烁 */
        flash_lit = !flash_lit;
        if (flash_lit) {
            zmk_rgb_underglow_set_hsb(COLOR_RED);
            zmk_rgb_underglow_on();
        } else {
            zmk_rgb_underglow_off();
        }
        LOG_DBG("RGB: red flash lit=%d (batt=%u%%)", flash_lit, batt_level);
        k_work_schedule(&indicator_work, K_MSEC(FLASH_HALF_MS));
        return;
    }

    /* 正常状态（电量充足，未充电）：熄灭省电 */
    zmk_rgb_underglow_off();
    LOG_DBG("RGB: off (batt=%u%%, no USB)", batt_level);
}

/* ─── 状态触发 ───────────────────────────────────────────────*/

static void trigger_update(void)
{
    k_work_cancel_delayable(&indicator_work);
    breathe_idx = 0;
    flash_lit   = false;
    k_work_schedule(&indicator_work, K_NO_WAIT);
}

/* ─── ZMK 事件监听 ───────────────────────────────────────────*/

static int on_battery_state_changed(const zmk_event_t *eh)
{
    const struct zmk_battery_state_changed *ev =
        as_zmk_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    LOG_INF("Battery: %u%%", ev->state_of_charge);
    if (ev->state_of_charge != batt_level) {
        batt_level = ev->state_of_charge;
        trigger_update();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

static int on_usb_conn_state_changed(const zmk_event_t *eh)
{
    const struct zmk_usb_conn_state_changed *ev =
        as_zmk_usb_conn_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    bool powered = (ev->conn_state != ZMK_USB_CONN_NONE);
    LOG_INF("USB: %s", powered ? "connected" : "disconnected");
    if (powered != usb_powered) {
        usb_powered = powered;
        trigger_update();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

/* ─── 模块初始化 ─────────────────────────────────────────────*/

static int battery_rgb_init(void)
{
    k_work_init_delayable(&indicator_work, indicator_work_handler);

    /*
     * 延迟 3s 后首次刷新，等待 ZMK underglow 子系统和电量采样完成初始化。
     * （ZMK underglow 在 APPLICATION 阶段初始化，我们优先级相同需稍后运行）
     */
    k_work_schedule(&indicator_work, K_SECONDS(3));

    LOG_INF("Battery RGB indicator ready (low_batt_threshold=%d%%)",
            LOW_BATT_THRESHOLD);
    return 0;
}

SYS_INIT(battery_rgb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(batt_rgb_batt, on_battery_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_batt, zmk_battery_state_changed);

ZMK_LISTENER(batt_rgb_usb, on_usb_conn_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_usb, zmk_usb_conn_state_changed);
