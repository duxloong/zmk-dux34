/*
 * battery_rgb_indicator.c
 *
 * 电池状态 RGB LED 指示灯（D1 / P0.06 / WS2812）
 * 通过 ZMK RGB Underglow API 控制 LED，从而将 SPI 耗时操作交给专门的背景线程，
 * 彻底避免阻塞系统工作队列（防止按键失效和蓝牙连接断开）。
 *
 * 状态逻辑：
 *   充电中 + 电量 < 100%  → 黄色呼吸灯（慢速正弦渐变）
 *   充电中 + 电量 = 100%  → 绿色常亮
 *   未充电 + 电量 < 20%   → 红色闪烁（0.5s 周期）
 *   其他                  → 熄灭（省电）
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

/** 低电量阈值（%），低于此值时红色闪烁（业界惯例：20%） */
#define LOW_BATT_THRESHOLD  20

/** 红色闪烁半周期（ms）: 亮 500ms / 灭 500ms */
#define FLASH_HALF_MS       500

/** 呼吸灯每步间隔（ms）: 64步 × 30ms ≈ 1.9s 一次完整呼吸 */
#define BREATHE_STEP_MS     30

/** LED 亮度（0–100，ZMK HSB 的 B 分量） */
#define MAX_BRIGHTNESS      70

/* ─── ZMK HSB 颜色定义 ─────────────────────────────────────── */

#define COLOR_GREEN  ((struct zmk_led_hsb){.h = 120, .s = 100, .b = MAX_BRIGHTNESS})
#define COLOR_RED    ((struct zmk_led_hsb){.h = 0,   .s = 100, .b = MAX_BRIGHTNESS})
#define COLOR_YELLOW ((struct zmk_led_hsb){.h = 45,  .s = 100, .b = MAX_BRIGHTNESS})
#define COLOR_OFF    ((struct zmk_led_hsb){.h = 0,   .s = 0,   .b = 0})

/* ─── 正弦呼吸亮度查找表（64步，0→100→0，一个完整周期）─── */
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
static uint8_t  batt_level  = 100;
static bool     usb_powered = false;
static bool     flash_lit   = false;
static uint8_t  breathe_idx = 0;

static struct k_work_delayable indicator_work;

/* ─── 工作项处理 ─────────────────────────────────────────────*/

static void indicator_work_handler(struct k_work *work)
{
    if (usb_powered) {
        if (batt_level >= 100) {
            zmk_rgb_underglow_set_hsb(COLOR_GREEN);
            zmk_rgb_underglow_on();
            return;
        }

        uint8_t luma = breathe_lut[breathe_idx];
        breathe_idx = (breathe_idx + 1U) % 64U;

        struct zmk_led_hsb color = COLOR_YELLOW;
        /* 缩放亮度 */
        color.b = (uint8_t)((uint32_t)luma * MAX_BRIGHTNESS / 100U);
        zmk_rgb_underglow_set_hsb(color);
        zmk_rgb_underglow_on();

        k_work_schedule(&indicator_work, K_MSEC(BREATHE_STEP_MS));
        return;
    }

    if (batt_level < LOW_BATT_THRESHOLD) {
        flash_lit = !flash_lit;
        if (flash_lit) {
            zmk_rgb_underglow_set_hsb(COLOR_RED);
            zmk_rgb_underglow_on();
        } else {
            zmk_rgb_underglow_off();
        }
        k_work_schedule(&indicator_work, K_MSEC(FLASH_HALF_MS));
        return;
    }

    /* 正常状态熄灭 */
    zmk_rgb_underglow_off();
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
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev == NULL) return ZMK_EV_EVENT_BUBBLE;

    if (ev->state_of_charge != batt_level) {
        batt_level = ev->state_of_charge;
        trigger_update();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

static int on_usb_conn_state_changed(const zmk_event_t *eh)
{
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (ev == NULL) return ZMK_EV_EVENT_BUBBLE;

    bool powered = (ev->conn_state != ZMK_USB_CONN_NONE);
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
    k_work_schedule(&indicator_work, K_SECONDS(3));
    return 0;
}

SYS_INIT(battery_rgb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(batt_rgb_batt, on_battery_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_batt, zmk_battery_state_changed);

ZMK_LISTENER(batt_rgb_usb, on_usb_conn_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_usb, zmk_usb_conn_state_changed);
