/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * TinyUSB device configuration for the SPDIF -> USB UAC1 recording device.
 * Copyright (C) 2026 Weeb Labs
 *
 * IMPORTANT: TinyUSB's built-in audio class driver (audio_device.c) is UAC2-only
 * -- audiod_open() hard-requires bInterfaceProtocol == AUDIO_INT_PROTOCOL_CODE_V2
 * and rejects UAC1 (protocol 0x00). So we set CFG_TUD_AUDIO = 0 and provide our
 * own minimal UAC1 class driver, registered via usbd_app_driver_get_cb() in
 * main.c. (This mirrors the approach proven in the DSPi UAC1 firmware.)
 */

#ifndef _TUSB_CONFIG_H_
#define _TUSB_CONFIG_H_

#ifdef __cplusplus
extern "C" {
#endif

/* -------------------------------------------------------------------------- */
/* Board / MCU                                                                 */
/* -------------------------------------------------------------------------- */
#ifndef CFG_TUSB_MCU
#define CFG_TUSB_MCU              OPT_MCU_RP2040
#endif

#ifndef CFG_TUSB_OS
#define CFG_TUSB_OS               OPT_OS_PICO
#endif

#ifndef BOARD_TUD_RHPORT
#define BOARD_TUD_RHPORT          0
#endif

#ifndef BOARD_TUD_MAX_SPEED
#define BOARD_TUD_MAX_SPEED       OPT_MODE_FULL_SPEED   /* RP2040 USB is full-speed only */
#endif

#define CFG_TUSB_RHPORT0_MODE     (OPT_MODE_DEVICE | BOARD_TUD_MAX_SPEED)

/* Memory placement / alignment required by the RP2040 USB DPRAM */
#ifndef CFG_TUSB_MEM_SECTION
#define CFG_TUSB_MEM_SECTION
#endif
#ifndef CFG_TUSB_MEM_ALIGN
#define CFG_TUSB_MEM_ALIGN        __attribute__((aligned(4)))
#endif

#define CFG_TUD_ENABLED           1
#define CFG_TUD_MAX_SPEED         BOARD_TUD_MAX_SPEED

/* EP0 control endpoint size */
#define CFG_TUD_ENDPOINT0_SIZE    64

/* -------------------------------------------------------------------------- */
/* Class drivers                                                               */
/*   The UAC1 audio function is handled by our own class driver (see main.c),  */
/*   NOT by TinyUSB's built-in (UAC2-only) audio driver.                       */
/* -------------------------------------------------------------------------- */
#define CFG_TUD_CDC               0
#define CFG_TUD_MSC               0
#define CFG_TUD_HID               0
#define CFG_TUD_MIDI              0
#define CFG_TUD_VENDOR            0
#define CFG_TUD_AUDIO             0

/* -------------------------------------------------------------------------- */
/* Audio format parameters (shared by usb_descriptors.c and main.c)            */
/* -------------------------------------------------------------------------- */

/* 2 channels, 24-bit (3 bytes) per sample => 6 bytes per stereo frame. */
#define USBRX_N_CHANNELS                 2
#define USBRX_BYTES_PER_SAMPLE           3
#define USBRX_BYTES_PER_FRAME            (USBRX_N_CHANNELS * USBRX_BYTES_PER_SAMPLE)

/*
 * Maximum isochronous IN packet size. Highest rate is 96 kHz (96 stereo frames
 * / 1 ms); add servo headroom: 100 frames * 6 bytes = 600 bytes, well within
 * the full-speed isochronous limit of 1023 bytes.
 */
#define USBRX_MAX_FRAMES_PER_PACKET      100
#define USBRX_EP_IN_SIZE                 (USBRX_MAX_FRAMES_PER_PACKET * USBRX_BYTES_PER_FRAME)

/* Length (bytes) of the AC+AS audio descriptors (excluding the IAD); the
 * custom class driver's open() returns this span. Kept in sync by a static
 * assert in usb_descriptors.c. */
#define USBRX_AUDIO_DESC_LEN             100

#ifdef __cplusplus
}
#endif

#endif /* _TUSB_CONFIG_H_ */
