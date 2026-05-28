/*
 * battery_rgb_indicator.c
 *
 * 电池状态 RGB LED 指示灯（D1 / P0.06 / WS2812）
 * 直接通过 Zephyr LED Strip 驱动（led_strip_update_rgb）控制 WS2812
 * 不依赖 ZMK underglow 子系统，不影响蓝牙功能
 *
 * 状态逻辑：
 *   充电中 + 电量 < 100%  → 黄色呼吸灯（慢速正弦渐变）
 *   充电中 + 电量 = 100%  → 绿色常亮
 *   未充电 + 电量 < 90%   → 红色闪烁（0.5s 周期）
 *   其他                  → 熄灭（省电）
 *
 * 硬件：NRF52840 (Nice!Nano v2), WS2812, SPI3 MOSI = D1 (P0.06)
 * 配置依赖：CONFIG_SPI=y, CONFIG_LED_STRIP=y, CONFIG_WS2812_STRIP_SPI=y
 */

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

LOG_MODULE_REGISTER(battery_rgb, CONFIG_ZMK_LOG_LEVEL);

/* ─── 用户可调参数 ──────────────────────────────────────────── */

/** 低电量阈值（%），低于此值时红色闪烁 */
#define LOW_BATT_THRESHOLD  90

/** 红色闪烁半周期（ms）: 亮 500ms / 灭 500ms */
#define FLASH_HALF_MS       500

/** 呼吸灯每步间隔（ms）: 64步 × 30ms ≈ 1.9s 一次完整呼吸 */
#define BREATHE_STEP_MS     30

/** 最大亮度缩放因子（0-255），调低可省电 */
#define MAX_BRIGHTNESS      180

/* ─── 正弦呼吸亮度查找表（64步，0→255→0，一个完整周期）─── */
/* 公式：round(127.5 × (1 − cos(2π × i / 64)))              */
static const uint8_t breathe_lut[64] = {
      0,   0,   2,   5,   9,  15,  21,  29,   /*  0- 7 */
     37,  46,  56,  67,  79,  90, 102, 115,   /*  8-15 */
    127, 140, 152, 164, 176, 187, 198, 208,   /* 16-23 */
    217, 225, 233, 240, 245, 249, 252, 254,   /* 24-31 */
    255, 254, 252, 249, 245, 240, 233, 225,   /* 32-39 */
    217, 208, 198, 187, 176, 164, 152, 140,   /* 40-47 */
    127, 115, 102,  90,  79,  67,  56,  46,   /* 48-55 */
     37,  29,  21,  15,   9,   5,   2,   0,   /* 56-63 */
};

/* ─── 硬件设备句柄（通过 DT nodelabel 获取，编译期绑定）────── */
static const struct device *led_strip_dev =
    DEVICE_DT_GET(DT_NODELABEL(led_strip));

/* ─── 状态变量 ─────────────────────────────────────────────── */
static uint8_t  batt_level  = 100;
static bool     usb_powered = false;
static bool     flash_lit   = false;
static uint8_t  breathe_idx = 0;

static struct k_work_delayable indicator_work;

/* ─── 辅助：向 WS2812 发送一帧 RGB ─────────────────────────── */

static inline uint8_t scale(uint8_t raw)
{
    return (uint8_t)((uint32_t)raw * MAX_BRIGHTNESS / 255U);
}

static void set_led(uint8_t r, uint8_t g, uint8_t b)
{
    struct led_rgb color = { .r = r, .g = g, .b = b };
    int ret = led_strip_update_rgb(led_strip_dev, &color, 1);
    if (ret) {
        LOG_ERR("LED strip update failed: %d", ret);
    }
}

/* ─── 工作项：动画帧更新 ─────────────────────────────────────*/

static void indicator_work_handler(struct k_work *work)
{
    if (usb_powered) {
        if (batt_level >= 100) {
            /* 充满电：绿色常亮 */
            set_led(0, scale(255), 0);
            LOG_DBG("RGB: green solid (fully charged)");
            /* 不再重调度，直到状态改变 */
            return;
        }

        /* 充电中：黄色呼吸灯 */
        uint8_t luma = breathe_lut[breathe_idx];
        breathe_idx  = (breathe_idx + 1U) % 64U;

        /* 暖黄色：R 略强于 G */
        uint8_t r = scale(luma);
        uint8_t g = (uint8_t)((uint32_t)scale(luma) * 200U / 255U);
        set_led(r, g, 0);

        LOG_DBG("RGB: yellow breathe idx=%u luma=%u", breathe_idx, luma);
        k_work_schedule(&indicator_work, K_MSEC(BREATHE_STEP_MS));
        return;
    }

    if (batt_level < LOW_BATT_THRESHOLD) {
        /* 低电量：红色闪烁 */
        flash_lit = !flash_lit;
        set_led(flash_lit ? scale(255) : 0, 0, 0);

        LOG_DBG("RGB: red flash lit=%d (batt=%u%%)", flash_lit, batt_level);
        k_work_schedule(&indicator_work, K_MSEC(FLASH_HALF_MS));
        return;
    }

    /* 正常：熄灭省电 */
    set_led(0, 0, 0);
    LOG_DBG("RGB: off (batt=%u%%, no USB)", batt_level);
}

/* ─── 触发状态刷新 ────────────────────────────────────────── */

static void trigger_update(void)
{
    k_work_cancel_delayable(&indicator_work);
    breathe_idx = 0;
    flash_lit   = false;
    k_work_schedule(&indicator_work, K_NO_WAIT);
}

/* ─── ZMK 事件监听器 ────────────────────────────────────────*/

static int on_battery_state_changed(const zmk_event_t *eh)
{
    const struct zmk_battery_state_changed *ev =
        as_zmk_battery_state_changed(eh);
    if (!ev) {
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
    if (!ev) {
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
    if (!device_is_ready(led_strip_dev)) {
        LOG_ERR("WS2812 LED strip not ready (check SPI3/D1 config)");
        return -ENODEV;
    }

    k_work_init_delayable(&indicator_work, indicator_work_handler);

    /*
     * 延迟 3s 后首次刷新：等待电量采样和 USB 状态初始化完成。
     * 这段延迟不影响蓝牙广播（BT 在 PRE_KERNEL 阶段已启动）。
     */
    k_work_schedule(&indicator_work, K_SECONDS(3));

    LOG_INF("Battery RGB indicator ready (threshold=%d%%)", LOW_BATT_THRESHOLD);
    return 0;
}

SYS_INIT(battery_rgb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(batt_rgb_batt, on_battery_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_batt, zmk_battery_state_changed);

ZMK_LISTENER(batt_rgb_usb, on_usb_conn_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_usb, zmk_usb_conn_state_changed);
