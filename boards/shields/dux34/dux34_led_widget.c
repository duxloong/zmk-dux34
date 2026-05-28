/*
 * DUX34 WS2812 LED 电池/充电状态指示 Widget
 * 
 * D1 引脚 (P0.06) 连接单颗 WS2812 RGB LED:
 *   🟡 黄色常亮 = 充电中 (USB 已插入, 电量 < 100%)
 *   🔵 蓝色常亮 = 充满电 (USB 已插入, 电量 = 100%)
 *   🔴 红色常亮 = 低电量 (电量 < 20%, USB 未插入)
 *   ⚫ 熄灭     = 正常使用 (电量充足, 无 USB)
 *
 * 注意：nice!nano v2 的充电 IC STAT 引脚未连接到 GPIO，
 *       因此用 "USB 已插入" 代替 "正在充电" 检测。
 *
 * Copyright (c) 2024 dux34 contributors
 * SPDX-License-Identifier: MIT
 */

#include <zephyr/device.h>
#include <zephyr/init.h>
#include <zephyr/kernel.h>
#include <zephyr/drivers/led_strip.h>
#include <zephyr/logging/log.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/workqueue.h>

#if IS_ENABLED(CONFIG_ZMK_USB)
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* 低电量阈值（百分比） */
#define LOW_BATTERY_THRESHOLD 20

/* LED 亮度（0-255），适当调低以节省电量 */
#define LED_BRIGHTNESS 80

/* 获取 underglow LED strip 设备 */
#define STRIP_CHOSEN DT_CHOSEN(zmk_underglow)

/* 颜色定义 */
static const struct led_rgb COLOR_OFF    = {.r = 0,              .g = 0,              .b = 0};
static const struct led_rgb COLOR_RED    = {.r = LED_BRIGHTNESS, .g = 0,              .b = 0};
static const struct led_rgb COLOR_YELLOW = {.r = LED_BRIGHTNESS, .g = LED_BRIGHTNESS / 2, .b = 0};
static const struct led_rgb COLOR_BLUE   = {.r = 0,              .g = 0,              .b = LED_BRIGHTNESS};

/* 全局状态 */
static uint8_t current_battery_level = 100;
static bool usb_is_powered = false;

/* LED strip 设备 */
static const struct device *led_strip_dev;

/* 工作队列任务 */
static struct k_work led_update_work;

/**
 * 根据当前电池和 USB 状态计算应显示的颜色
 * 优先级（由高到低）：
 *   1. USB 插入 + 电量=100% → 蓝色（充满）
 *   2. USB 插入 + 电量<100% → 黄色（充电中）
 *   3. USB 未插入 + 电量<20% → 红色（低电量）
 *   4. 其他 → 熄灭
 */
static struct led_rgb dux34_calc_led_color(void) {
    if (usb_is_powered) {
        if (current_battery_level >= 100) {
            return COLOR_BLUE;      /* 充满电 */
        } else {
            return COLOR_YELLOW;    /* 充电中 */
        }
    } else {
        if (current_battery_level < LOW_BATTERY_THRESHOLD) {
            return COLOR_RED;       /* 低电量 */
        } else {
            return COLOR_OFF;       /* 正常使用，关灯节省电量 */
        }
    }
}

/**
 * 实际更新 LED 颜色的工作队列处理函数
 * 在低优先级工作队列中执行，避免阻塞事件处理线程
 */
static void dux34_led_update_handler(struct k_work *work) {
    if (!device_is_ready(led_strip_dev)) {
        LOG_ERR("LED strip device not ready");
        return;
    }

    struct led_rgb color = dux34_calc_led_color();
    struct led_rgb pixels[1] = {color};

    int err = led_strip_update_rgb(led_strip_dev, pixels, 1);
    if (err < 0) {
        LOG_ERR("Failed to update LED strip: %d", err);
    } else {
        LOG_DBG("LED updated: R=%d G=%d B=%d (bat=%d%%, usb=%d)",
                color.r, color.g, color.b,
                current_battery_level, usb_is_powered);
    }
}

/**
 * ZMK 事件监听器
 * 响应电池电量变化和 USB 连接状态变化事件
 */
static int dux34_led_event_listener(const zmk_event_t *eh) {
    /* 处理电池电量变化事件 */
    const struct zmk_battery_state_changed *bat_ev = as_zmk_battery_state_changed(eh);
    if (bat_ev) {
        current_battery_level = bat_ev->state_of_charge;
        LOG_DBG("Battery state: %d%%", current_battery_level);
        k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &led_update_work);
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_USB)
    /* 处理 USB 连接状态变化事件（仅在 USB 启用时编译） */
    const struct zmk_usb_conn_state_changed *usb_ev = as_zmk_usb_conn_state_changed(eh);
    if (usb_ev) {
        usb_is_powered = (usb_ev->conn_state != ZMK_USB_CONN_NONE);
        LOG_DBG("USB state: %s", usb_is_powered ? "connected" : "disconnected");
        k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &led_update_work);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif /* IS_ENABLED(CONFIG_ZMK_USB) */

    return ZMK_EV_EVENT_BUBBLE;
}

/* 注册事件监听器（订阅电池事件，始终需要） */
ZMK_LISTENER(dux34_led_widget, dux34_led_event_listener);
ZMK_SUBSCRIPTION(dux34_led_widget, zmk_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_USB)
/* USB 状态事件订阅（仅在 USB 启用时编译） */
ZMK_SUBSCRIPTION(dux34_led_widget, zmk_usb_conn_state_changed);
#endif /* IS_ENABLED(CONFIG_ZMK_USB) */

/**
 * 初始化函数
 * 在应用初始化阶段执行，获取 LED strip 设备并初始化工作队列任务
 */
static int dux34_led_widget_init(void) {
    led_strip_dev = DEVICE_DT_GET(STRIP_CHOSEN);

    if (!device_is_ready(led_strip_dev)) {
        LOG_ERR("LED strip device \"%s\" is not ready", led_strip_dev->name);
        return -ENODEV;
    }

    k_work_init(&led_update_work, dux34_led_update_handler);

#if IS_ENABLED(CONFIG_ZMK_USB)
    /* 初始化时读取当前 USB 状态 */
    usb_is_powered = zmk_usb_is_powered();
#endif

    /* 立即更新一次 LED（等待电池事件覆盖初始默认值） */
    k_work_submit_to_queue(zmk_workqueue_lowprio_work_q(), &led_update_work);

    LOG_INF("DUX34 LED widget initialized (USB=%s)", usb_is_powered ? "on" : "off");
    return 0;
}

/* 在应用初始化阶段启动 widget（与 rgb_underglow 相同的优先级） */
SYS_INIT(dux34_led_widget_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
