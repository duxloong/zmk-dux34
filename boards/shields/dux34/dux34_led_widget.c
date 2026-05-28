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

#if IS_ENABLED(CONFIG_ZMK_USB)
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>
#endif

LOG_MODULE_DECLARE(zmk, CONFIG_ZMK_LOG_LEVEL);

/* 低电量阈值（百分比） */
#define LOW_BATTERY_THRESHOLD 20

/* LED 亮度（0-255），适当调低以节省电量 */
#define LED_BRIGHTNESS 60

/*
 * 使用 DT alias "dux34-led" 直接引用 LED strip，
 * 不依赖 zmk,underglow chosen（避免与 ZMK 框架冲突）
 */
#define STRIP_NODE DT_ALIAS(dux34_led)

BUILD_ASSERT(DT_NODE_EXISTS(STRIP_NODE),
             "dux34-led alias not found in devicetree");

/* 颜色定义 */
static const struct led_rgb COLOR_OFF    = {.r = 0,              .g = 0,              .b = 0};
static const struct led_rgb COLOR_RED    = {.r = LED_BRIGHTNESS, .g = 0,              .b = 0};
static const struct led_rgb COLOR_YELLOW = {.r = LED_BRIGHTNESS, .g = LED_BRIGHTNESS / 2, .b = 0};
static const struct led_rgb COLOR_BLUE   = {.r = 0,              .g = 0,              .b = LED_BRIGHTNESS};

/* 全局状态 */
static uint8_t current_battery_level = 100;
static bool usb_is_powered = false;
static bool strip_ready = false;

/* LED strip 设备（编译期绑定，不会为 NULL） */
static const struct device *const led_strip_dev = DEVICE_DT_GET(STRIP_NODE);

/*
 * 独立工作队列：不占用 ZMK 低优先级工作队列，
 * 避免与电池监控等共用队列时栈溢出。
 */
#define LED_WORK_STACK_SIZE 1024
#define LED_WORK_PRIORITY   10

static K_THREAD_STACK_DEFINE(led_work_stack, LED_WORK_STACK_SIZE);
static struct k_work_q led_work_q;
static struct k_work   led_update_work;

/**
 * 根据当前电池和 USB 状态计算应显示的颜色
 */
static struct led_rgb dux34_calc_led_color(void)
{
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
 * LED 更新工作函数（在独立工作队列中执行）
 */
static void dux34_led_update_handler(struct k_work *work)
{
    if (!strip_ready) {
        return;
    }

    struct led_rgb color  = dux34_calc_led_color();
    struct led_rgb pixels[1] = {color};

    int err = led_strip_update_rgb(led_strip_dev, pixels, 1);
    if (err < 0) {
        LOG_ERR("DUX34: LED strip update failed: %d", err);
    } else {
        LOG_DBG("DUX34: LED R=%d G=%d B=%d (bat=%d%% usb=%d)",
                color.r, color.g, color.b,
                current_battery_level, usb_is_powered);
    }
}

/**
 * ZMK 事件监听器
 */
static int dux34_led_event_listener(const zmk_event_t *eh)
{
    const struct zmk_battery_state_changed *bat_ev =
        as_zmk_battery_state_changed(eh);
    if (bat_ev) {
        current_battery_level = bat_ev->state_of_charge;
        k_work_submit(&led_update_work);
        return ZMK_EV_EVENT_BUBBLE;
    }

#if IS_ENABLED(CONFIG_ZMK_USB)
    const struct zmk_usb_conn_state_changed *usb_ev =
        as_zmk_usb_conn_state_changed(eh);
    if (usb_ev) {
        usb_is_powered = (usb_ev->conn_state != ZMK_USB_CONN_NONE);
        k_work_submit(&led_update_work);
        return ZMK_EV_EVENT_BUBBLE;
    }
#endif

    return ZMK_EV_EVENT_BUBBLE;
}

ZMK_LISTENER(dux34_led_widget, dux34_led_event_listener);
ZMK_SUBSCRIPTION(dux34_led_widget, zmk_battery_state_changed);

#if IS_ENABLED(CONFIG_ZMK_USB)
ZMK_SUBSCRIPTION(dux34_led_widget, zmk_usb_conn_state_changed);
#endif

/**
 * 初始化函数
 */
static int dux34_led_widget_init(void)
{
    if (!device_is_ready(led_strip_dev)) {
        /* LED strip 不可用，优雅退出，不影响键盘功能 */
        LOG_WRN("DUX34: LED strip not ready, widget disabled");
        return 0;   /* 返回 0 避免 Zephyr 打印 init error */
    }

    strip_ready = true;

    /* 启动独立工作队列（专用栈，不与 ZMK 队列共享） */
    k_work_queue_start(&led_work_q, led_work_stack,
                       K_THREAD_STACK_SIZEOF(led_work_stack),
                       LED_WORK_PRIORITY, NULL);
    k_thread_name_set(&led_work_q.thread, "dux34_led");

    k_work_init(&led_update_work, dux34_led_update_handler);

#if IS_ENABLED(CONFIG_ZMK_USB)
    usb_is_powered = zmk_usb_is_powered();
#endif

    /* 初始熄灭 LED，等待第一次电池事件后再显示正确颜色 */
    struct led_rgb off[1] = {COLOR_OFF};
    led_strip_update_rgb(led_strip_dev, off, 1);

    LOG_INF("DUX34: LED widget initialized (USB=%s, bat=%d%%)",
            usb_is_powered ? "on" : "off", current_battery_level);
    return 0;
}

SYS_INIT(dux34_led_widget_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);
