/*
 * ESP32-S3 USB HID Gamepad — 2-joystick generic USB game controller
 * ------------------------------------------------------------
 * This firmware turns an ESP32-S3 into a standard USB HID gamepad.
 * It is not tied to any specific application — Windows/Linux/macOS,
 * games, sims, or any software that reads a generic joystick/gamepad
 * will see this exactly like any off-the-shelf USB controller. The
 * axis layout below is just a sensible default; rebind the physical
 * axes/buttons to whatever function you like inside any given app's
 * own controller settings.
 *
 * Wiring (default pins, change in the CONFIG section below):
 *
 *   Joystick 1 (LEFT stick)
 *     VRx -> GPIO4  (ADC1_CH3)   -> HID axis Z
 *     VRy -> GPIO5  (ADC1_CH4)   -> HID axis Rz
 *     SW  -> GPIO6  (button, active LOW, internal pull-up)
 *
 *   Joystick 2 (RIGHT stick)
 *     VRx -> GPIO7  (ADC1_CH6)   -> HID axis X
 *     VRy -> GPIO8  (ADC1_CH7)   -> HID axis Y
 *     SW  -> GPIO9  (button, active LOW, internal pull-up)
 *
 *   Extra momentary buttons - optional:
 *     GPIO10, GPIO11  (active LOW, internal pull-up)
 *
 *   Both joysticks: VCC -> 3V3, GND -> GND
 *
 * The board enumerates on USB as a standard HID gamepad with:
 *   x, y, z, rz  -> 4 analog axes (int8_t, -127..127)
 *   buttons      -> up to 8 buttons (bitmask)
 *
 * Because this is a plain HID gamepad, it works with any software that
 * accepts a generic controller: drone/flight sims, racing games, retro
 * emulators, DirectInput test tools, custom scripts, etc. Just bind the
 * axes and buttons in that software's own controller/input settings —
 * no drone-specific or game-specific code lives in this firmware.
 *
 * Requires the ESP32-S3's *native USB* port (the one wired to the D+/D-
 * pins, not the separate USB-UART/JTAG port used for flashing on some
 * boards) to be connected to the PC.
 */

#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "tinyusb.h"
#include "class/hid/hid_device.h"

static const char *TAG = "drone_joystick";

/* ----------------------- CONFIG: pins ----------------------- */
#define ADC_UNIT            ADC_UNIT_1

/* Left stick (throttle/yaw) */
#define PIN_L_X             GPIO_NUM_4
#define PIN_L_Y             GPIO_NUM_5
#define ADC_CH_L_X          ADC_CHANNEL_3   /* GPIO4 on ESP32-S3 */
#define ADC_CH_L_Y          ADC_CHANNEL_4   /* GPIO5 on ESP32-S3 */
#define PIN_L_SW            GPIO_NUM_6

/* Right stick (roll/pitch) */
#define PIN_R_X             GPIO_NUM_7
#define PIN_R_Y             GPIO_NUM_8
#define ADC_CH_R_X          ADC_CHANNEL_6   /* GPIO7 on ESP32-S3 */
#define ADC_CH_R_Y          ADC_CHANNEL_7   /* GPIO8 on ESP32-S3 */
#define PIN_R_SW            GPIO_NUM_9

/* Extra buttons (arm/mode switches) */
#define PIN_BTN_3           GPIO_NUM_10
#define PIN_BTN_4           GPIO_NUM_11

/* ----------------------- CONFIG: behavior -------------------- */
#define ADC_MAX              4095
#define DEADZONE             60      /* raw counts of deadzone around center */
#define REPORT_INTERVAL_MS   4       /* ~250 Hz update rate */

/* Auto-calibration: at boot, each axis's center is measured by
 * averaging this many samples while the sticks are untouched, instead
 * of assuming a fixed midpoint. Keep the sticks released while the
 * board boots. */
#define CALIBRATION_SAMPLES     200
#define CALIBRATION_SAMPLE_MS   5    /* ~1 second total per axis set */

/* Per-axis inversion. Flip any of these (0<->1) if a physical push
 * direction produces the opposite HID axis direction than you want -
 * this depends purely on your joystick hardware's wiring polarity,
 * not on any particular application. */
#define INVERT_AXIS_Z        0   /* left stick X  -> HID axis Z  */
#define INVERT_AXIS_RZ        1   /* left stick Y  -> HID axis Rz */
#define INVERT_AXIS_X        0   /* right stick X -> HID axis X  */
#define INVERT_AXIS_Y        0   /* right stick Y -> HID axis Y  */

/* ----------------------- USB HID descriptors ----------------- */
enum {
    REPORT_ID_GAMEPAD = 1,
};

static const uint8_t hid_report_descriptor[] = {
    TUD_HID_REPORT_DESC_GAMEPAD(HID_REPORT_ID(REPORT_ID_GAMEPAD))
};

static const char *hid_string_descriptor[5] = {
    (char[]){0x09, 0x04},              // 0: supported language (English)
    "DIY",                              // 1: Manufacturer
    "ESP32S3 USB Game Controller",      // 2: Product
    "123456",                           // 3: Serial
    "USB Gamepad HID Interface",        // 4: HID interface name
};

static const uint8_t hid_configuration_descriptor[] = {
    TUD_CONFIG_DESCRIPTOR(1, 1, 0, TUD_CONFIG_DESC_LEN + TUD_HID_DESC_LEN,
                           TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    TUD_HID_DESCRIPTOR(0, 4, false, sizeof(hid_report_descriptor), 0x81, 16, 10),
};

static const tusb_desc_device_t hid_device_descriptor = {
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = 0x0200,
    .bDeviceClass       = 0x00,
    .bDeviceSubClass    = 0x00,
    .bDeviceProtocol    = 0x00,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,
    .idVendor           = 0x303A,   /* Espressif VID */
    .idProduct          = 0x8090,
    .bcdDevice          = 0x0100,
    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,
    .bNumConfigurations = 0x01
};

/* ----------------------- TinyUSB HID callbacks ---------------- */
/* Required by TinyUSB even if unused */
uint8_t const *tud_hid_descriptor_report_cb(uint8_t instance)
{
    (void) instance;
    return hid_report_descriptor;
}

uint16_t tud_hid_get_report_cb(uint8_t instance, uint8_t report_id,
                                hid_report_type_t report_type, uint8_t *buffer,
                                uint16_t reqlen)
{
    (void) instance; (void) report_id; (void) report_type;
    (void) buffer; (void) reqlen;
    return 0;
}

void tud_hid_set_report_cb(uint8_t instance, uint8_t report_id,
                            hid_report_type_t report_type, uint8_t const *buffer,
                            uint16_t bufsize)
{
    (void) instance; (void) report_id; (void) report_type;
    (void) buffer; (void) bufsize;
}

/* ----------------------- ADC helpers -------------------------- */
static adc_oneshot_unit_handle_t s_adc_handle;
static adc_cali_handle_t s_cali_handle = NULL;
static bool s_cali_enabled = false;

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT,
    };
    ESP_ERROR_CHECK(adc_oneshot_new_unit(&init_cfg, &s_adc_handle));

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CH_L_X, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CH_L_Y, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CH_R_X, &chan_cfg));
    ESP_ERROR_CHECK(adc_oneshot_config_channel(s_adc_handle, ADC_CH_R_Y, &chan_cfg));

    adc_cali_curve_fitting_config_t cali_cfg = {
        .unit_id  = ADC_UNIT,
        .atten    = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    s_cali_enabled = (adc_cali_create_scheme_curve_fitting(&cali_cfg, &s_cali_handle) == ESP_OK);
}

static int adc_read_raw_ch(adc_channel_t ch)
{
    int raw = 2048;
    adc_oneshot_read(s_adc_handle, ch, &raw);
    return raw;
}

/* ----------------------- Per-axis calibration ------------------ */
static int s_center_lx, s_center_ly, s_center_rx, s_center_ry;

/* Sample each axis repeatedly while assumed at rest and average the
 * readings, so each axis's true center is used instead of a fixed
 * assumed midpoint. Run once at boot, before the sticks are touched. */
static void calibrate_centers(void)
{
    int64_t sum_lx = 0, sum_ly = 0, sum_rx = 0, sum_ry = 0;

    for (int i = 0; i < CALIBRATION_SAMPLES; i++) {
        sum_lx += adc_read_raw_ch(ADC_CH_L_X);
        sum_ly += adc_read_raw_ch(ADC_CH_L_Y);
        sum_rx += adc_read_raw_ch(ADC_CH_R_X);
        sum_ry += adc_read_raw_ch(ADC_CH_R_Y);
        vTaskDelay(pdMS_TO_TICKS(CALIBRATION_SAMPLE_MS));
    }

    s_center_lx = (int)(sum_lx / CALIBRATION_SAMPLES);
    s_center_ly = (int)(sum_ly / CALIBRATION_SAMPLES);
    s_center_rx = (int)(sum_rx / CALIBRATION_SAMPLES);
    s_center_ry = (int)(sum_ry / CALIBRATION_SAMPLES);

    ESP_LOGI(TAG, "Calibrated centers: LX=%d LY=%d RX=%d RY=%d",
             s_center_lx, s_center_ly, s_center_rx, s_center_ry);
}

static void buttons_init(void)
{
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << PIN_L_SW) | (1ULL << PIN_R_SW) |
                        (1ULL << PIN_BTN_3) | (1ULL << PIN_BTN_4),
        .mode         = GPIO_MODE_INPUT,
        .pull_up_en   = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    gpio_config(&io_conf);
}

/* Map a raw 12-bit ADC reading, centered at the given calibrated
 * center, to an int8_t joystick axis value in the range [-127, 127],
 * applying a deadzone. */
static int8_t map_axis(int raw, int center, bool invert)
{
    int centered = raw - center;

    if (centered > -DEADZONE && centered < DEADZONE) {
        centered = 0;
    } else if (centered > 0) {
        centered -= DEADZONE;
    } else {
        centered += DEADZONE;
    }

    /* Use the larger of the two half-spans either side of this axis's
     * own calibrated center, so full deflection still reaches -127/127
     * even if the center isn't exactly in the middle of the ADC range. */
    int span_hi = (ADC_MAX - center) - DEADZONE;
    int span_lo = center - DEADZONE;
    int span = (span_hi > span_lo) ? span_hi : span_lo;
    if (span <= 0) span = 1;

    int32_t val = ((int32_t)centered * 127) / span;
    if (val > 127) val = 127;
    if (val < -127) val = -127;
    if (invert) val = -val;

    return (int8_t)val;
}

/* ----------------------- Main task ----------------------------- */
static void joystick_task(void *arg)
{
    while (1) {
        if (tud_hid_ready()) {
            int raw_lx = adc_read_raw_ch(ADC_CH_L_X);
            int raw_ly = adc_read_raw_ch(ADC_CH_L_Y);
            int raw_rx = adc_read_raw_ch(ADC_CH_R_X);
            int raw_ry = adc_read_raw_ch(ADC_CH_R_Y);

            int8_t axis_z  = map_axis(raw_lx, s_center_lx, INVERT_AXIS_Z);
            int8_t axis_rz = map_axis(raw_ly, s_center_ly, INVERT_AXIS_RZ);
            int8_t axis_x  = map_axis(raw_rx, s_center_rx, INVERT_AXIS_X);
            int8_t axis_y  = map_axis(raw_ry, s_center_ry, INVERT_AXIS_Y);

            uint32_t buttons = 0;
            if (gpio_get_level(PIN_L_SW) == 0) buttons |= (1u << 0);
            if (gpio_get_level(PIN_R_SW) == 0) buttons |= (1u << 1);
            if (gpio_get_level(PIN_BTN_3) == 0) buttons |= (1u << 2);
            if (gpio_get_level(PIN_BTN_4) == 0) buttons |= (1u << 3);

            hid_gamepad_report_t report = {
                .x       = axis_x,
                .y       = axis_y,
                .z       = axis_z,
                .rz      = axis_rz,
                .rx      = 0,
                .ry      = 0,
                .hat     = 0,
                .buttons = buttons,
            };

            tud_hid_n_report(0, REPORT_ID_GAMEPAD, &report, sizeof(report));
        }

        vTaskDelay(pdMS_TO_TICKS(REPORT_INTERVAL_MS));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Initializing ADC + buttons");
    adc_init();
    buttons_init();

    ESP_LOGI(TAG, "Calibrating stick centers - keep sticks released...");
    calibrate_centers();

    ESP_LOGI(TAG, "Installing TinyUSB HID gamepad");
    const tinyusb_config_t tusb_cfg = {
        .port = TINYUSB_PORT_FULL_SPEED_0,
        .task = {
            .size     = 4096,
            .priority = 5,
            .xCoreID  = 0,
        },
        .descriptor = {
            .device             = &hid_device_descriptor,
            .string             = hid_string_descriptor,
            .string_count       = sizeof(hid_string_descriptor) / sizeof(hid_string_descriptor[0]),
            .full_speed_config  = hid_configuration_descriptor,
        },
    };
    ESP_ERROR_CHECK(tinyusb_driver_install(&tusb_cfg));

    ESP_LOGI(TAG, "USB HID gamepad ready. Connect the native USB port to your PC.");

    xTaskCreate(joystick_task, "joystick_task", 4096, NULL, 5, NULL);
}
