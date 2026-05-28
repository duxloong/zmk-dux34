/*
 * battery_rgb_indicator.c
 *
 * 电池状态 RGB LED 指示灯（D1 / P0.06 / WS2812）
 * 使用独立工作队列进行 SPI 传输，避免阻塞 Zephyr 系统队列，
 * 防止按键失效与蓝牙断开。
 */

#include <zephyr/kernel.h>
#include <zephyr/logging/log.h>
#include <zephyr/device.h>
#include <zephyr/drivers/led_strip.h>

#include <zmk/event_manager.h>
#include <zmk/events/battery_state_changed.h>
#include <zmk/events/usb_conn_state_changed.h>
#include <zmk/usb.h>

LOG_MODULE_REGISTER(battery_rgb, CONFIG_ZMK_LOG_LEVEL);

/* ─── 驱动设备 ───────────────────────────────────────────────*/
static const struct device *led_strip_dev = DEVICE_DT_GET(DT_NODELABEL(led_strip));

/* ─── 用户可调参数 ────────────────────────────────────────────*/
#define LOW_BATT_THRESHOLD  20
#define FLASH_HALF_MS       500
#define BREATHE_STEP_MS     30
#define MAX_BRIGHTNESS      70

/* ─── 颜色设置 ───────────────────────────────────────────────*/
static void set_led(uint8_t r, uint8_t g, uint8_t b)
{
    if (!device_is_ready(led_strip_dev)) return;

    struct led_rgb color = {
        .r = r,
        .g = g,
        .b = b,
    };
    /* 这个调用如果挂起，只会挂起我们的自定义队列，不会影响键盘按键和蓝牙 */
    led_strip_update_rgb(led_strip_dev, &color, 1);
}

static const uint8_t breathe_lut[64] = {
      0,   0,   1,   2,   4,   6,   8,  11,
     15,  18,  22,  26,  31,  35,  40,  45,
     50,  55,  60,  64,  69,  73,  78,  82,
     85,  88,  91,  94,  96,  98,  99, 100,
    100,  99,  98,  96,  94,  91,  88,  85,
     82,  78,  73,  69,  64,  60,  55,  50,
     45,  40,  35,  31,  26,  22,  18,  15,
     11,   8,   6,   4,   2,   1,   0,   0,
};

/* ─── 独立工作队列 ───────────────────────────────────────────*/
static K_THREAD_STACK_DEFINE(indicator_work_q_stack, 1024);
static struct k_work_q indicator_work_q;
static struct k_work_delayable indicator_work;

/* ─── 状态变量 ───────────────────────────────────────────────*/
static uint8_t  batt_level  = 100;
static bool     usb_powered = false;
static bool     flash_lit   = false;
static uint8_t  breathe_idx = 0;

static void indicator_work_handler(struct k_work *work)
{
    if (usb_powered) {
        if (batt_level >= 100) {
            /* 绿色常亮，限制亮度 */
            set_led(0, MAX_BRIGHTNESS, 0);
            return;
        }

        uint8_t luma = breathe_lut[breathe_idx];
        breathe_idx = (breathe_idx + 1U) % 64U;

        /* 黄色呼吸灯，按 luma 缩放 */
        uint8_t r = (uint8_t)((uint32_t)luma * MAX_BRIGHTNESS / 100U);
        uint8_t g = (uint8_t)((uint32_t)luma * MAX_BRIGHTNESS / 100U);
        set_led(r, g, 0);

        k_work_reschedule_for_queue(&indicator_work_q, &indicator_work, K_MSEC(BREATHE_STEP_MS));
        return;
    }

    if (batt_level < LOW_BATT_THRESHOLD) {
        flash_lit = !flash_lit;
        if (flash_lit) {
            set_led(MAX_BRIGHTNESS, 0, 0);
        } else {
            set_led(0, 0, 0);
        }
        k_work_reschedule_for_queue(&indicator_work_q, &indicator_work, K_MSEC(FLASH_HALF_MS));
        return;
    }

    /* 正常状态熄灭 */
    set_led(0, 0, 0);
}

static void trigger_update(void)
{
    k_work_cancel_delayable(&indicator_work);
    breathe_idx = 0;
    flash_lit   = false;
    k_work_reschedule_for_queue(&indicator_work_q, &indicator_work, K_NO_WAIT);
}

/* ─── ZMK 事件监听 ───────────────────────────────────────────*/
static int on_battery_state_changed(const zmk_event_t *eh)
{
    const struct zmk_battery_state_changed *ev = as_zmk_battery_state_changed(eh);
    if (ev && ev->state_of_charge != batt_level) {
        batt_level = ev->state_of_charge;
        trigger_update();
    }
    return ZMK_EV_EVENT_BUBBLE;
}

static int on_usb_conn_state_changed(const zmk_event_t *eh)
{
    const struct zmk_usb_conn_state_changed *ev = as_zmk_usb_conn_state_changed(eh);
    if (ev) {
        bool powered = (ev->conn_state != ZMK_USB_CONN_NONE);
        if (powered != usb_powered) {
            usb_powered = powered;
            trigger_update();
        }
    }
    return ZMK_EV_EVENT_BUBBLE;
}

/* ─── 模块初始化 ─────────────────────────────────────────────*/
static int battery_rgb_init(void)
{
    if (!device_is_ready(led_strip_dev)) {
        LOG_ERR("LED strip not ready");
        return -ENODEV;
    }

    /* 启动专属后台线程 */
    k_work_queue_init(&indicator_work_q);
    k_work_queue_start(&indicator_work_q, indicator_work_q_stack,
                       K_THREAD_STACK_SIZEOF(indicator_work_q_stack),
                       K_LOWEST_APPLICATION_THREAD_PRIO, NULL);

    k_work_init_delayable(&indicator_work, indicator_work_handler);
    
    /* 延迟 3 秒首次执行 */
    k_work_reschedule_for_queue(&indicator_work_q, &indicator_work, K_SECONDS(3));
    return 0;
}

SYS_INIT(battery_rgb_init, APPLICATION, CONFIG_APPLICATION_INIT_PRIORITY);

ZMK_LISTENER(batt_rgb_batt, on_battery_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_batt, zmk_battery_state_changed);

ZMK_LISTENER(batt_rgb_usb, on_usb_conn_state_changed);
ZMK_SUBSCRIPTION(batt_rgb_usb, zmk_usb_conn_state_changed);
