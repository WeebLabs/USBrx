/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * SPDIF -> USB UAC1 stereo recording device for the RP2040.
 * Copyright (C) 2026 Weeb Labs
 *
 *   S/PDIF input (GPIO) --[pico_spdif_rx: PIO + DMA]--> internal FIFO (ring buffer)
 *                                                          |
 *                              rate-matching servo (per USB frame)
 *                                                          |
 *                         UAC1 isochronous async IN endpoint --> USB host
 *
 * The S/PDIF receiver recovers the sample rate and fills an internal FIFO. The
 * USB host polls the isochronous IN endpoint once per (1 ms) frame. Each frame
 * we decide how many stereo frames to send: a feed-forward term equal to the
 * actual S/PDIF rate (samples per millisecond) plus a small proportional
 * correction that keeps the FIFO near a target fill level. Varying the packet
 * size frame-by-frame is the implicit feedback that matches the USB data rate
 * to the free-running S/PDIF clock (no resampling).
 *
 * The sample rate is dictated by the S/PDIF source. The host must select the
 * matching rate; if it does not, audio streams at the wrong pitch (by design).
 *
 * TinyUSB's built-in audio driver is UAC2-only, so the audio function is
 * implemented here as a custom UAC1 class driver registered through
 * usbd_app_driver_get_cb() (pattern proven in the DSPi UAC1 firmware).
 */

#include <stdio.h>
#include <string.h>

#include "pico/stdlib.h"
#include "tusb.h"
#include "device/usbd_pvt.h"      /* usbd_class_driver_t, usbd_edpt_* */
#include "class/audio/audio.h"    /* AUDIO_* constants (not pulled in when CFG_TUD_AUDIO=0) */

#include "spdif_rx.h"
#include "tusb_config.h"

/* -------------------------------------------------------------------------- */
/* Configuration                                                               */
/* -------------------------------------------------------------------------- */
#define SPDIF_DATA_PIN        15     /* S/PDIF data input GPIO */
#define SPDIF_LOCK_STATUS_44100_PIN 2      /* S/PDIF lock at 44100 output GPIO */
#define SPDIF_LOCK_STATUS_48000_PIN 3      /* S/PDIF lock at 44100 output GPIO */
#define SPDIF_LOCK_STATUS_88200_PIN 4      /* S/PDIF lock at 44100 output GPIO */
#define SPDIF_LOCK_STATUS_96000_PIN 5      /* S/PDIF lock at 44100 output GPIO */

#define PIN_DCDC_PSM_CTRL     23     /* Pico SMPS mode pin: drive high for PWM (less audio noise) */

#define AUDIO_IN_EP           0x81   /* isochronous IN endpoint address (must match descriptor) */

/* UAC1 control request codes (direction is encoded in bRequest, unlike UAC2). */
#define UAC1_REQ_SET_CUR      0x01
#define UAC1_REQ_GET_CUR      0x81

/* Target number of stereo frames buffered in the S/PDIF FIFO. The receiver's
 * DMA delivers data in blocks of 192 stereo frames, so the working point must
 * comfortably exceed one block. 256 frames keeps latency low (~5.3 ms @ 48 kHz)
 * while leaving ample margin against under/overflow (FIFO holds 1536 frames). */
#define TARGET_FILL_FRAMES    256

/* Proportional servo gain and clamp. The S/PDIF and USB clocks differ only by a
 * few ppm, so only a tiny correction is ever needed in steady state; the clamp
 * also bounds the pull-in rate so the audio is never noticeably time-warped. */
#define SERVO_KP              0.008f   /* (samples/frame) per (frame) of fill error */
#define SERVO_MAX_CORR        2.0f     /* max |correction| in samples/frame */

/* -------------------------------------------------------------------------- */
/* State (all USB callbacks below run in tud_task() context on core 0)         */
/* -------------------------------------------------------------------------- */
static volatile uint32_t g_usb_sample_rate = 48000;   /* host-selected rate (echoed on GET_CUR) */
static uint32_t          g_silence_frames  = 48;      /* silence packet size = host_rate / 1000  */

static spdif_rx_samp_freq_t g_last_sf      = SAMP_FREQ_NONE;
static float                g_nominal_fpf  = 48.0f;   /* nominal stereo frames per USB frame */
static float                g_acc          = 0.0f;    /* fractional-sample accumulator */

static bool g_streaming = false;   /* AS alt setting 1 active */
static bool g_primed    = false;   /* FIFO has reached target at least once */

/* IN packet scratch (RAM). Only touched while no transfer is in flight. */
static uint8_t __attribute__((aligned(4))) g_pkt[USBRX_EP_IN_SIZE];

/* Flags set from S/PDIF IRQ callbacks, consumed (for logging) in main loop. */
static volatile bool g_evt_stable = false;
static volatile bool g_evt_lost   = false;

/* Current lock state */
static bool g_locked = false;


/* -------------------------------------------------------------------------- */
/* Rate-matching servo                                                         */
/* -------------------------------------------------------------------------- */
static void update_nominal_from_spdif(void)
{
    /* Use the measured frequency so the feed-forward includes the source's real
     * ppm offset; the servo then only has to absorb USB-vs-S/PDIF frame jitter. */
    float actual = spdif_rx_get_samp_freq_actual();
    if (actual < 1000.0f) {
        actual = (float) g_usb_sample_rate;   /* fallback before a measurement exists */
    }
    g_nominal_fpf = actual / 1000.0f;
}

static inline uint16_t silence_bytes(void)
{
    return (uint16_t)(g_silence_frames * USBRX_BYTES_PER_FRAME);
}

/*
 * Fill 'buf' with the next isochronous IN packet and return its size in bytes.
 * This is the rate-matching servo; it runs once per USB frame (each time the
 * previous IN packet completes).
 */
static uint16_t fill_audio_packet(uint8_t *buf)
{
    /* No valid S/PDIF signal: emit silence at the host's cadence, stay unprimed. */
    if (spdif_rx_get_state() != SPDIF_RX_STATE_STABLE) {
        g_primed = false;
        g_acc    = 0.0f;
        uint16_t n = silence_bytes();
        memset(buf, 0, n);
        return n;
    }

    /* Refresh the feed-forward rate if the detected S/PDIF rate changed. */
    spdif_rx_samp_freq_t sf = spdif_rx_get_samp_freq();
    if (sf != g_last_sf && sf != SAMP_FREQ_NONE) {
        g_last_sf = sf;
        update_nominal_from_spdif();
        g_acc = 0.0f;
    }

    uint32_t avail = spdif_rx_get_fifo_count() / 2;   /* stereo frames available */

    /* Prime: wait until the FIFO has filled to the target before streaming audio. */
    if (!g_primed) {
        if (avail >= (uint32_t) TARGET_FILL_FRAMES) {
            g_primed = true;
            g_acc    = 0.0f;
        } else {
            uint16_t n = silence_bytes();
            memset(buf, 0, n);
            return n;
        }
    }

    /* ---- Rate-matching servo: nominal feed-forward + clamped P correction ---- */
    int   err  = (int) avail - TARGET_FILL_FRAMES;
    float corr = SERVO_KP * (float) err;
    if (corr >  SERVO_MAX_CORR) corr =  SERVO_MAX_CORR;
    if (corr < -SERVO_MAX_CORR) corr = -SERVO_MAX_CORR;

    g_acc += g_nominal_fpf + corr;
    int n = (int) g_acc;           /* floor; g_acc is always >= 0 */
    g_acc -= (float) n;

    if (n < 0) n = 0;
    if (n > USBRX_MAX_FRAMES_PER_PACKET) n = USBRX_MAX_FRAMES_PER_PACKET;
    if ((uint32_t) n > avail) n = (int) avail;

    if (n == 0) {
        /* Real underrun: keep the stream alive and re-prime. */
        g_primed = false;
        uint16_t z = silence_bytes();
        memset(buf, 0, z);
        return z;
    }

    /* ---- Copy + convert n stereo frames (2n subframe words) to 24-bit LE ---- */
    uint32_t words_needed = (uint32_t) n * 2;
    uint32_t got = 0;
    uint8_t *dst = buf;

    while (got < words_needed) {
        uint32_t *src;
        uint32_t  c = spdif_rx_read_fifo(&src, words_needed - got);
        if (c == 0) break;
        for (uint32_t i = 0; i < c; i++) {
            /* FIFO word: [27:4] = 24-bit signed audio (MSB at bit 27). */
            uint32_t s = (src[i] >> 4) & 0x00FFFFFFu;
            *dst++ = (uint8_t)(s & 0xFF);
            *dst++ = (uint8_t)((s >> 8) & 0xFF);
            *dst++ = (uint8_t)((s >> 16) & 0xFF);
        }
        got += c;
    }

    return (uint16_t)(got * USBRX_BYTES_PER_SAMPLE);
}

/* -------------------------------------------------------------------------- */
/* S/PDIF callbacks (IRQ context: keep short)                                  */
/* -------------------------------------------------------------------------- */
static void on_stable_func(spdif_rx_samp_freq_t samp_freq)
{
    (void) samp_freq;
    g_evt_stable = true;
}

static void on_lost_stable_func(void)
{
    g_evt_lost = true;
}

/* ========================================================================== */
/* Custom UAC1 class driver (TinyUSB's built-in audio driver is UAC2-only)     */
/* ========================================================================== */

static struct {
    uint8_t        ac_itf;       /* AudioControl interface number */
    uint8_t        as_itf;       /* AudioStreaming interface number */
    uint8_t        cur_alt;      /* current AS alternate setting */
    bool           ep_open;      /* iso IN endpoint open */
    const uint8_t *ep_desc;      /* pointer to the iso IN endpoint descriptor */
    uint8_t        pending_cs;   /* deferred SET_CUR control selector */
    uint8_t        pending_len;  /* deferred SET_CUR data length */
} uac1;

static uint8_t uac1_ctrl_buf[8];  /* SET_CUR data-stage scratch */

/* Fill the scratch packet via the servo and queue it on the IN endpoint. */
static void uac1_arm_in(uint8_t rhport)
{
    uint16_t len = fill_audio_packet(g_pkt);
    usbd_edpt_xfer(rhport, AUDIO_IN_EP, g_pkt, len);
}

/* Apply an AudioStreaming alternate setting (0 = idle, 1 = streaming). */
static bool uac1_apply_alt(uint8_t rhport, uint8_t alt)
{
    if (alt > 1) return false;
    if (alt == uac1.cur_alt) return true;   /* idempotent SET_INTERFACE */
    uac1.cur_alt = alt;

    if (alt == 1) {
#ifdef TUP_DCD_EDPT_ISO_ALLOC
        TU_ASSERT(usbd_edpt_iso_activate(rhport, (tusb_desc_endpoint_t const *) uac1.ep_desc));
#else
        TU_ASSERT(usbd_edpt_open(rhport, (tusb_desc_endpoint_t const *) uac1.ep_desc));
#endif
        /* Clear any stale iso EP state (AVAIL/busy) left from a previous
         * alt cycle; stall+clear flushes both the stack and hardware flags. */
        usbd_edpt_stall(rhport, AUDIO_IN_EP);
        usbd_edpt_clear_stall(rhport, AUDIO_IN_EP);

        uac1.ep_open = true;
        g_streaming  = true;
        g_primed     = false;
        g_acc        = 0.0f;

        uac1_arm_in(rhport);   /* queue the first packet */
    } else {
        if (uac1.ep_open) {
            usbd_edpt_close(rhport, AUDIO_IN_EP);
            uac1.ep_open = false;
        }
        g_streaming = false;
    }
    return true;
}

static void uac1_driver_init(void)
{
    memset(&uac1, 0, sizeof(uac1));
}

static bool uac1_driver_deinit(void)
{
    return true;
}

static void uac1_driver_reset(uint8_t rhport)
{
    (void) rhport;
    uac1.cur_alt = 0;
    uac1.ep_open = false;
    g_streaming  = false;
    g_primed     = false;
}

/* Claim the AC + AS interfaces and reserve the iso IN endpoint. */
static uint16_t uac1_driver_open(uint8_t rhport, tusb_desc_interface_t const *itf_desc, uint16_t max_len)
{
    TU_VERIFY(itf_desc->bInterfaceClass == TUSB_CLASS_AUDIO);
    TU_VERIFY(itf_desc->bInterfaceSubClass == AUDIO_SUBCLASS_CONTROL);
    TU_VERIFY(itf_desc->bAlternateSetting == 0);

    uac1.ac_itf = itf_desc->bInterfaceNumber;

    const uint8_t *p   = (const uint8_t *) itf_desc;
    const uint8_t *end = p + max_len;
    uint16_t drv_len = 0;

    /* AC standard interface descriptor */
    drv_len += tu_desc_len(p);
    p       += tu_desc_len(p);

    /* AC class-specific descriptors (header, terminals, ...) */
    while (p < end && tu_desc_type(p) == TUSB_DESC_CS_INTERFACE) {
        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);
    }

    /* AS interfaces (alt 0 + alt 1) and their class-specific + endpoint descriptors */
    while (p < end && tu_desc_type(p) == TUSB_DESC_INTERFACE) {
        tusb_desc_interface_t const *as = (tusb_desc_interface_t const *) p;
        if (as->bInterfaceClass != TUSB_CLASS_AUDIO ||
            as->bInterfaceSubClass != AUDIO_SUBCLASS_STREAMING) {
            break;
        }
        uac1.as_itf = as->bInterfaceNumber;

        drv_len += tu_desc_len(p);
        p       += tu_desc_len(p);

        while (p < end && tu_desc_type(p) != TUSB_DESC_INTERFACE) {
            if (tu_desc_type(p) == TUSB_DESC_ENDPOINT) {
                tusb_desc_endpoint_t const *ep = (tusb_desc_endpoint_t const *) p;
                if (ep->bmAttributes.xfer == TUSB_XFER_ISOCHRONOUS &&
                    tu_edpt_dir(ep->bEndpointAddress) == TUSB_DIR_IN) {
                    uac1.ep_desc = p;   /* remember for iso (de)activation */
#ifdef TUP_DCD_EDPT_ISO_ALLOC
                    usbd_edpt_iso_alloc(rhport, ep->bEndpointAddress, USBRX_EP_IN_SIZE);
#endif
                }
            }
            drv_len += tu_desc_len(p);
            p       += tu_desc_len(p);
        }
    }

    return drv_len;
}

static bool uac1_driver_control_xfer_cb(uint8_t rhport, uint8_t stage, tusb_control_request_t const *req)
{
    if (stage == CONTROL_STAGE_SETUP) {
        /* ----- Standard requests on our interfaces ----- */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_STANDARD) {
            uint8_t itf = TU_U16_LOW(req->wIndex);

            if (req->bRequest == TUSB_REQ_SET_INTERFACE) {
                uint8_t alt = TU_U16_LOW(req->wValue);
                if (itf == uac1.ac_itf) {
                    return (alt == 0) ? tud_control_status(rhport, req) : false;
                }
                if (itf == uac1.as_itf) {
                    return uac1_apply_alt(rhport, alt) ? tud_control_status(rhport, req) : false;
                }
                return false;
            }
            if (req->bRequest == TUSB_REQ_GET_INTERFACE) {
                static uint8_t alt_resp;
                if (itf == uac1.ac_itf)      alt_resp = 0;
                else if (itf == uac1.as_itf) alt_resp = uac1.cur_alt;
                else                         return false;
                return tud_control_xfer(rhport, req, &alt_resp, 1);
            }
            return false;
        }

        /* ----- Class requests (sampling-frequency control on the endpoint) ----- */
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
            req->bmRequestType_bit.recipient == TUSB_REQ_RCPT_ENDPOINT) {

            if (TU_U16_LOW(req->wIndex) != AUDIO_IN_EP) return false;
            uint8_t cs = TU_U16_HIGH(req->wValue);

            if (req->bmRequestType_bit.direction == TUSB_DIR_IN) {
                /* GET_CUR sampling frequency */
                if (req->bRequest == UAC1_REQ_GET_CUR && cs == AUDIO_CS_CTRL_SAM_FREQ) {
                    static uint8_t freq[3];
                    uint32_t f = g_usb_sample_rate;
                    freq[0] = (uint8_t)(f & 0xFF);
                    freq[1] = (uint8_t)((f >> 8) & 0xFF);
                    freq[2] = (uint8_t)((f >> 16) & 0xFF);
                    return tud_control_xfer(rhport, req, freq, 3);
                }
                return false;
            } else {
                /* SET_CUR sampling frequency: defer to the data stage */
                if (req->bRequest == UAC1_REQ_SET_CUR && cs == AUDIO_CS_CTRL_SAM_FREQ) {
                    uint16_t len = req->wLength;
                    if (len == 0 || len > sizeof(uac1_ctrl_buf)) return false;
                    uac1.pending_cs  = cs;
                    uac1.pending_len = (uint8_t) len;
                    return tud_control_xfer(rhport, req, uac1_ctrl_buf, len);
                }
                return false;
            }
        }
        return false;
    }

    if (stage == CONTROL_STAGE_DATA) {
        if (req->bmRequestType_bit.type == TUSB_REQ_TYPE_CLASS &&
            uac1.pending_cs == AUDIO_CS_CTRL_SAM_FREQ) {
            uint32_t f = (uint32_t) uac1_ctrl_buf[0]
                       | ((uint32_t) uac1_ctrl_buf[1] << 8)
                       | ((uint32_t) uac1_ctrl_buf[2] << 16);
            if (f == 44100 || f == 48000 || f == 88200 || f == 96000) {
                g_usb_sample_rate = f;
                g_silence_frames  = f / 1000;
            }
            uac1.pending_cs = 0;
        }
        return true;
    }

    return true;   /* CONTROL_STAGE_ACK */
}

static bool uac1_driver_xfer_cb(uint8_t rhport, uint8_t ep_addr, xfer_result_t result, uint32_t xferred_bytes)
{
    (void) result;
    (void) xferred_bytes;
    if (ep_addr == AUDIO_IN_EP) {
        if (uac1.ep_open) uac1_arm_in(rhport);   /* refill + queue next frame */
        return true;
    }
    return false;
}

static const usbd_class_driver_t uac1_driver = {
    .name            = "usbrx_uac1",
    .init            = uac1_driver_init,
    .deinit          = uac1_driver_deinit,
    .reset           = uac1_driver_reset,
    .open            = uac1_driver_open,
    .control_xfer_cb = uac1_driver_control_xfer_cb,
    .xfer_cb         = uac1_driver_xfer_cb,
    .sof             = NULL,
};

/* TinyUSB calls this weak symbol at tud_init() to discover app class drivers. */
usbd_class_driver_t const *usbd_app_driver_get_cb(uint8_t *driver_count)
{
    *driver_count = 1;
    return &uac1_driver;
}

/* -------------------------------------------------------------------------- */
/* Onboard-LED status indicator (diagnostic; no-op if board has no LED pin)     */
/*   fast blink (100 ms) : running, USB NOT enumerated                          */
/*   slow blink (500 ms) : enumerated/mounted, not streaming                    */
/*   solid on            : host is recording                                   */
/* -------------------------------------------------------------------------- */
static inline void led_init(void)
{
#ifdef PICO_DEFAULT_LED_PIN
    gpio_init(PICO_DEFAULT_LED_PIN);
    gpio_set_dir(PICO_DEFAULT_LED_PIN, GPIO_OUT);
#endif
}

/* -------------------------------------------------------------------------- */
/* GPIO output status indicators                                              */
/* one each for locked to 44.1, 48, 88.2 and 96kHz S/PDIF input               */
/* -------------------------------------------------------------------------- */
static inline void usbrx_gpio_init(void)
{
    gpio_init(SPDIF_LOCK_STATUS_44100_PIN);
    gpio_init(SPDIF_LOCK_STATUS_48000_PIN);
    gpio_init(SPDIF_LOCK_STATUS_88200_PIN);
    gpio_init(SPDIF_LOCK_STATUS_96000_PIN);

    gpio_set_dir(SPDIF_LOCK_STATUS_44100_PIN, GPIO_OUT);
    gpio_set_dir(SPDIF_LOCK_STATUS_48000_PIN, GPIO_OUT);
    gpio_set_dir(SPDIF_LOCK_STATUS_88200_PIN, GPIO_OUT);
    gpio_set_dir(SPDIF_LOCK_STATUS_96000_PIN, GPIO_OUT);
}

static inline void led_set(bool on)
{
#ifdef PICO_DEFAULT_LED_PIN
    gpio_put(PICO_DEFAULT_LED_PIN, on);
#else
    (void) on;
#endif
}

static void led_task(void)
{
    static uint32_t last = 0;
    static bool     on   = false;

    if (g_streaming) {
        led_set(true);
        on = true;
        return;
    }

    uint32_t now      = to_ms_since_boot(get_absolute_time());
    uint32_t interval = tud_mounted() ? 500 : 100;
    if (now - last >= interval) {
        last = now;
        on = !on;
        led_set(on);
    }
}

static void gpio_task(void)
{
    if(g_locked == true)
    {
        spdif_rx_samp_freq_t sf = spdif_rx_get_samp_freq();
        gpio_put(SPDIF_LOCK_STATUS_44100_PIN, sf == SAMP_FREQ_44100 ? 1 : 0);
        gpio_put(SPDIF_LOCK_STATUS_48000_PIN, sf == SAMP_FREQ_48000 ? 1 : 0);
        gpio_put(SPDIF_LOCK_STATUS_88200_PIN, sf == SAMP_FREQ_88200 ? 1 : 0);
        gpio_put(SPDIF_LOCK_STATUS_96000_PIN, sf == SAMP_FREQ_96000 ? 1 : 0);
    }
    else
    {
        gpio_put(SPDIF_LOCK_STATUS_44100_PIN, 0);
        gpio_put(SPDIF_LOCK_STATUS_48000_PIN, 0);
        gpio_put(SPDIF_LOCK_STATUS_88200_PIN, 0);
        gpio_put(SPDIF_LOCK_STATUS_96000_PIN, 0);
    }
}

/* -------------------------------------------------------------------------- */
/* main                                                                        */
/* -------------------------------------------------------------------------- */
int main(void)
{
    /* Keep the default clocks: 125 MHz sys (required by pico_spdif_rx) and the
     * 48 MHz USB PLL. Do NOT call set_sys_clock_* here. */
    stdio_init_all();
    led_init();
    usbrx_gpio_init();

    /* Pico SMPS into PWM mode for lower audio noise (harmless on other boards). */
    gpio_init(PIN_DCDC_PSM_CTRL);
    gpio_set_dir(PIN_DCDC_PSM_CTRL, GPIO_OUT);
    gpio_put(PIN_DCDC_PSM_CTRL, 1);

    printf("\nSPDIF -> USB UAC1 recorder starting (SPDIF on GP%d)\n", SPDIF_DATA_PIN);

    /* Start the USB device stack FIRST so the device always enumerates,
     * independent of whether an S/PDIF signal is present. */
    tusb_init();

    /* Start the S/PDIF receiver (non-blocking; just waits for a signal). */
    spdif_rx_config_t config = {
        .data_pin     = SPDIF_DATA_PIN,
        .pio_sm       = 0,
        .dma_channel0 = 0,
        .dma_channel1 = 1,
        .alarm_pool   = alarm_pool_get_default(),
        .flags        = SPDIF_RX_FLAGS_ALL,
    };
    spdif_rx_start(&config);
    spdif_rx_set_callback_on_stable(on_stable_func);
    spdif_rx_set_callback_on_lost_stable(on_lost_stable_func);

    while (true) {
        tud_task();
        led_task();
        gpio_task();

        if (g_evt_stable) {
            g_evt_stable = false;
            g_locked = true;
            printf("S/PDIF locked: %d Hz (%.1f Hz actual)\n",
                   (int) spdif_rx_get_samp_freq(), (double) spdif_rx_get_samp_freq_actual());
        }
        if (g_evt_lost) {
            g_evt_lost = false;
            g_locked = false;
            printf("S/PDIF signal lost\n");
        }
    }
    return 0;
}
