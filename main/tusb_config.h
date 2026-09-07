#pragma once

#ifdef __cplusplus
extern "C" {
#endif

/* ------------- Board / MCU / RTOS ------------- */
#define CFG_TUSB_MCU            OPT_MCU_ESP32S3
#define CFG_TUSB_OS             OPT_OS_FREERTOS

#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif

#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN      __attribute__ ((aligned(4)))
#endif

/* ------------- Device mode only --------------- */
#define CFG_TUD_ENABLED         1
#define CFG_TUH_ENABLED         0

#define CFG_TUD_ENDPOINT0_SIZE  64

/* ------------- Device class driver ------------ */
#define CFG_TUD_HID              1
#define CFG_TUD_CDC              0
#define CFG_TUD_MSC              0
#define CFG_TUD_MIDI             0
#define CFG_TUD_VENDOR           0

/* HID buffer size */
#define CFG_TUD_HID_EP_BUFSIZE   16

#ifdef __cplusplus
}
#endif
