/*
 * battery_rgb_indicator.c
 *
 * 电池状态 RGB LED 指示灯（D1 / P0.06 / WS2812）
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

/** 黄色呼吸灯更新间隔（ms）: 64步 × 30ms ≈ 1.9s 一次完整呼吸 */
#define BREATHE_STEP_MS     30

/** 最大亮度缩放因子（0-255），调低可省电 */
#define MAX_BRIGHTNESS      180

/* ─── 正弦呼吸亮度查找表（64步，一个完整周期 0→255→0）─── */
/* 由 round(127.5 * (1 - cos(2π * i / 64))) 生成                */
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

/* ─── 硬件设备句柄 ─────────────────────────────────────────── */
static const struct device *led_strip_dev =
    DEVICE_DT_GET(DT_NODELABEL(led_strip));

/* ─── 状态变量（仅在 system workqueue 中访问，无竞争） ──────── */
static uint8_t  batt_level  = 100;   /* 当前电量百分比 */
static bool     usb_powered = false; /* USB 已接入（充电中） */

/* 闪烁状态 */
static bool flash_lit = false;

/* 呼吸灯步进索引 */
static uint8_t breathe_idx = 0;

/* 延迟工作项（在 system workqueue 执行，ZMK 事件也在此队列） */
static struct k_work_delayable indicator_work;

/* ─── 辅助函数 ─────────────────────────────────────────────── */

/**
 * @brief 向 WS2812 LED 发送一帧 RGB 颜色
 *
 * @param r  红色分量 0-255
 * @param g  绿色分量 0-255
 * @param b  蓝色分量 0-255
 */
static void set_led_rgb(uint8_t r, uint8_t g, uint8_t b)
{
    if (!device_is_ready(led_strip_dev)) {
        return;
    }
    struct led_rgb color = { .r = r, .g = g, .b = b };
    int ret = led_strip_update_rgb(led_strip_dev, &color, 1);
    if (ret) {
        LOG_ERR("LED strip update failed: %d", ret);
    }
}

/**
 * @brief 将 0-255 亮度值按 MAX_BRIGHTNESS 缩放后输出
 */
static inline uint8_t scale(uint8_t raw)
{
    return (uint8_t)((uint32_t)raw * MAX_BRIGHTNESS / 255U);
}

/* ─── 工作项处理函数 ────────────────────────────────────────── */

static void indicator_work_handler(struct k_work *work)
{
    if (usb_powered) {
        /* ── USB 接入：区分充满 / 充电中 ── */
        if (batt_level >= 100) {
            /* 充满电：绿色常亮，不再重调度 */
            set_led_rgb(0, scale(255), 0);
            LOG_DBG("LED: green solid (fully charged)");
            return;
        } else {
            /* 充电中：黄色呼吸灯 */
            uint8_t luma = breathe_lut[breathe_idx];
            breathe_idx = (breathe_idx + 1) % 64U;

            /* 黄色 = 红 + 绿，此处选用暖黄色（红略强于绿） */
            uint8_t r = scale(luma);
            uint8_t g = (uint8_t)((uint32_t)scale(luma) * 200U / 255U);
            uint8_t b = 0;
            set_led_rgb(r, g, b);

            k_work_schedule(&indicator_work, K_MSEC(BREATHE_STEP_MS));
            return;
        }
    }

    /* ── 未接 USB ── */
    if (batt_level < LOW_BATT_THRESHOLD) {
        /* 低电量：红色闪烁 */
        flash_lit = !flash_lit;
        if (flash_lit) {
            set_led_rgb(scale(255), 0, 0);
        } else {
            set_led_rgb(0, 0, 0);
        }
        LOG_DBG("LED: red flash (batt=%u%%)", batt_level);
        k_work_schedule(&indicator_work, K_MSEC(FLASH_HALF_MS));
        return;
    }

    /* 正常电量，未充电：熄灭 LED 省电 */
    set_led_rgb(0, 0, 0);
    LOG_DBG("LED: off (batt=%u%%, not charging)", batt_level);
}

/* ─── 触发 LED 状态刷新 ─────────────────────────────────────── */

static void trigger_update(void)
{
    /* 取消旧的挂起工作，立刻重新调度 */
    k_work_cancel_delayable(&indicator_work);
    breathe_idx = 0;
    flash_lit   = false;
    k_work_schedule(&indicator_work, K_NO_WAIT);
}

/* ─── ZMK 事件监听器 ────────────────────────────────────────── */

/**
 * 电量变化事件：更新 batt_level，触发 LED 刷新
 */
static int on_battery_state_changed(const zmk_event_t *eh)
{
    const struct zmk_battery_state_changed *ev =
        as_zmk_battery_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    uint8_t new_level = ev->state_of_charge;
    LOG_INF("Battery level: %u%%", new_level);

    if (new_level != batt_level) {
        batt_level = new_level;
        trigger_update();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

/**
 * USB 连接状态变化事件：更新 usb_powered，触发 LED 刷新
 */
static int on_usb_conn_state_changed(const zmk_event_t *eh)
{
    const struct zmk_usb_conn_state_changed *ev =
        as_zmk_usb_conn_state_changed(eh);
    if (ev == NULL) {
        return ZMK_EV_EVENT_BUBBLE;
    }

    bool powered = (ev->conn_state != ZMK_USB_CONN_NONE);
    LOG_INF("USB state: %s (conn_state=%d)",
            powered ? "connected" : "disconnected", ev->conn_state);

    if (powered != usb_powered) {
        usb_powered = powered;
        trigger_update();
    }

    return ZMK_EV_EVENT_BUBBLE;
}

/* ─── 模块初始化 ────────────────────────────────────────────── */

static int battery_rgb_init(void)
{
    if (!device_is_ready(led_strip_dev)) {
        LOG_ERR("WS2812 LED strip (led_strip) not ready, check SPI3 config");
        return -ENODEV;
    }

    k_work_init_delayable(&indicator_work, indicator_work_handler);

    /* 开机延迟 2s 后首次更新（等待电量和 USB 状态初始化完毕） */
    k_work_schedule(&indicator_work, K_SECONDS(2));

    LOG_INF("Battery RGB indicator initialized (threshold=%d%%)",
            LOW_BATT_THRESHOLD);
    return 0;
}

/* 在 APPLICATION 阶段初始化，优先级稍低于 ZMK 核心 */
SYS_INIT(battery_rgb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

/* 注册事件订阅 */
ZMK_LISTENER(batt_rgb_batt, on_battery_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_batt, zmk_battery_state_changed);

ZMK_LISTENER(batt_rgb_usb, on_usb_conn_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_usb, zmk_usb_conn_state_changed);
