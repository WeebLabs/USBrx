/* SPDX-License-Identifier: GPL-3.0-or-later */
/*
 * USB descriptors for the SPDIF -> USB UAC1 stereo recording device.
 * Copyright (C) 2026 Weeb Labs
 *
 * Topology (Audio Class 1.0):
 *
 *   [Input Terminal: S/PDIF interface, ID 1] --> [Output Terminal: USB streaming, ID 2]
 *                                                          |
 *                                              AudioStreaming IN endpoint (0x81)
 *
 * Format: Type I PCM, 2 channels, 24-bit, discrete rates 44.1/48/88.2/96 kHz.
 * The IN endpoint is isochronous / asynchronous: the device is the clock master
 * (driven by the incoming S/PDIF stream) and the host adapts to the data rate.
 */

#include "tusb.h"
#include "class/audio/audio.h"   /* AUDIO_* constants (not pulled in when CFG_TUD_AUDIO=0) */
#include "tusb_config.h"

/* ----- Little-endian byte splitters for descriptor tables ----- */
#define U16LE(x)  (uint8_t)((x) & 0xFF), (uint8_t)(((x) >> 8) & 0xFF)
#define U24LE(x)  (uint8_t)((x) & 0xFF), (uint8_t)(((x) >> 8) & 0xFF), (uint8_t)(((x) >> 16) & 0xFF)

/* Interface and endpoint numbers */
#define ITF_NUM_AUDIO_CONTROL    0
#define ITF_NUM_AUDIO_STREAMING  1
#define ITF_NUM_TOTAL            2

#define EPNUM_AUDIO_IN           0x81

/* Audio entity IDs */
#define UAC1_INPUT_TERMINAL_ID   0x01   /* S/PDIF interface */
#define UAC1_OUTPUT_TERMINAL_ID  0x02   /* USB streaming     */

/* USB IDs (test/development values) */
#define USB_VID  0xCAFE
#define USB_PID  0x4011
#define USB_BCD  0x0200

/* -------------------------------------------------------------------------- */
/* Device descriptor                                                           */
/* -------------------------------------------------------------------------- */
tusb_desc_device_t const desc_device =
{
    .bLength            = sizeof(tusb_desc_device_t),
    .bDescriptorType    = TUSB_DESC_DEVICE,
    .bcdUSB             = USB_BCD,
    /* Miscellaneous device + IAD: required so the host (and TinyUSB's
     * process_set_config) treat the AC+AS interfaces as one function bound to
     * our custom UAC1 class driver. */
    .bDeviceClass       = TUSB_CLASS_MISC,
    .bDeviceSubClass    = MISC_SUBCLASS_COMMON,
    .bDeviceProtocol    = MISC_PROTOCOL_IAD,
    .bMaxPacketSize0    = CFG_TUD_ENDPOINT0_SIZE,

    .idVendor           = USB_VID,
    .idProduct          = USB_PID,
    .bcdDevice          = 0x0100,

    .iManufacturer      = 0x01,
    .iProduct           = 0x02,
    .iSerialNumber      = 0x03,

    .bNumConfigurations = 0x01
};

uint8_t const *tud_descriptor_device_cb(void)
{
    return (uint8_t const *) &desc_device;
}

/* -------------------------------------------------------------------------- */
/* Configuration descriptor                                                    */
/* -------------------------------------------------------------------------- */
/* TUD_AUDIO_DESC_IAD_LEN (== 8) comes from class/audio/audio.h */
#define CONFIG_TOTAL_LEN  (TUD_CONFIG_DESC_LEN + TUD_AUDIO_DESC_IAD_LEN + USBRX_AUDIO_DESC_LEN)

/* Class-specific AudioControl interface total length: header + input + output */
#define AC_CS_TOTAL_LEN   (9 + 12 + 9)

uint8_t const desc_configuration[] =
{
    /* Configuration descriptor: 2 interfaces, bus-powered, 100 mA */
    TUD_CONFIG_DESCRIPTOR(1, ITF_NUM_TOTAL, 0, CONFIG_TOTAL_LEN,
                          TUSB_DESC_CONFIG_ATT_REMOTE_WAKEUP, 100),

    /* Interface Association Descriptor: groups the AC + AS interfaces into a
     * single audio function so the host binds both to our class driver. */
    8, TUSB_DESC_INTERFACE_ASSOCIATION,
    ITF_NUM_AUDIO_CONTROL,    /* bFirstInterface */
    2,                        /* bInterfaceCount  (AC + AS) */
    TUSB_CLASS_AUDIO,         /* bFunctionClass */
    AUDIO_SUBCLASS_CONTROL,   /* bFunctionSubClass */
    0x00,                     /* bFunctionProtocol (UAC1) */
    0x00,                     /* iFunction */

    /* ====================== AudioControl interface ====================== */
    /* Standard AC interface descriptor (alt 0, no endpoints) */
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_CONTROL, 0x00, 0x00,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_CONTROL, AUDIO_INT_PROTOCOL_CODE_UNDEF, 0x00,

    /* Class-specific AC interface header descriptor */
    9, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_HEADER,
    U16LE(0x0100),                 /* bcdADC = 1.00 */
    U16LE(AC_CS_TOTAL_LEN),        /* wTotalLength of all CS AC descriptors */
    0x01,                          /* bInCollection: 1 streaming interface */
    ITF_NUM_AUDIO_STREAMING,       /* baInterfaceNr(1) */

    /* Input terminal: S/PDIF interface, stereo */
    12, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_INPUT_TERMINAL,
    UAC1_INPUT_TERMINAL_ID,        /* bTerminalID */
    U16LE(0x0605),                 /* wTerminalType = S/PDIF interface */
    0x00,                          /* bAssocTerminal */
    USBRX_N_CHANNELS,              /* bNrChannels = 2 */
    U16LE(0x0003),                 /* wChannelConfig = Left + Right */
    0x00,                          /* iChannelNames */
    0x00,                          /* iTerminal */

    /* Output terminal: USB streaming, sourced from the input terminal */
    9, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AC_INTERFACE_OUTPUT_TERMINAL,
    UAC1_OUTPUT_TERMINAL_ID,       /* bTerminalID */
    U16LE(AUDIO_TERM_TYPE_USB_STREAMING),
    0x00,                          /* bAssocTerminal */
    UAC1_INPUT_TERMINAL_ID,        /* bSourceID = input terminal */
    0x00,                          /* iTerminal */

    /* ===================== AudioStreaming interface ===================== */
    /* Alt 0: zero-bandwidth (no endpoint) */
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING, 0x00, 0x00,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_UNDEF, 0x00,

    /* Alt 1: operational, 1 isochronous IN endpoint */
    9, TUSB_DESC_INTERFACE,
    ITF_NUM_AUDIO_STREAMING, 0x01, 0x01,
    TUSB_CLASS_AUDIO, AUDIO_SUBCLASS_STREAMING, AUDIO_INT_PROTOCOL_CODE_UNDEF, 0x00,

    /* Class-specific AS general interface descriptor */
    7, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_AS_GENERAL,
    UAC1_OUTPUT_TERMINAL_ID,       /* bTerminalLink = output terminal */
    0x01,                          /* bDelay (frames) */
    U16LE(0x0001),                 /* wFormatTag = PCM */

    /* Type I format descriptor: 2 ch, 3 bytes/subframe, 24-bit, 4 discrete rates */
    20, TUSB_DESC_CS_INTERFACE, AUDIO_CS_AS_INTERFACE_FORMAT_TYPE,
    AUDIO_FORMAT_TYPE_I,
    USBRX_N_CHANNELS,              /* bNrChannels */
    USBRX_BYTES_PER_SAMPLE,        /* bSubframeSize = 3 */
    24,                            /* bBitResolution */
    0x04,                          /* bSamFreqType: 4 discrete frequencies */
    U24LE(44100),
    U24LE(48000),
    U24LE(88200),
    U24LE(96000),

    /* Standard isochronous audio data endpoint descriptor (9 bytes: includes
     * bRefresh + bSynchAddress as required for audio endpoints).
     * bmAttributes 0x05 = isochronous, asynchronous, data endpoint. */
    9, TUSB_DESC_ENDPOINT,
    EPNUM_AUDIO_IN, 0x05,
    U16LE(USBRX_EP_IN_SIZE),       /* wMaxPacketSize */
    0x01,                          /* bInterval = 1 frame */
    0x00,                          /* bRefresh */
    0x00,                          /* bSynchAddress (none) */

    /* Class-specific isochronous audio data endpoint descriptor.
     * bmAttributes bit0 = sampling frequency control is supported. */
    7, TUSB_DESC_CS_ENDPOINT, AUDIO_CS_EP_SUBTYPE_GENERAL,
    0x01,                          /* bmAttributes: sampling freq control */
    0x00,                          /* bLockDelayUnits */
    U16LE(0x0000),                 /* wLockDelay */
};

/* Make sure the hand-counted descriptor length matches the macro. */
TU_VERIFY_STATIC(sizeof(desc_configuration) == CONFIG_TOTAL_LEN,
                 "configuration descriptor length mismatch");
TU_VERIFY_STATIC((CONFIG_TOTAL_LEN - TUD_CONFIG_DESC_LEN - TUD_AUDIO_DESC_IAD_LEN) == USBRX_AUDIO_DESC_LEN,
                 "USBRX_AUDIO_DESC_LEN does not match descriptor table");

uint8_t const *tud_descriptor_configuration_cb(uint8_t index)
{
    (void) index;
    return desc_configuration;
}

/* -------------------------------------------------------------------------- */
/* String descriptors                                                          */
/* -------------------------------------------------------------------------- */
enum { STRID_LANGID = 0, STRID_MANUFACTURER, STRID_PRODUCT, STRID_SERIAL };

char const *string_desc_arr[] =
{
    (const char[]){ 0x09, 0x04 },   /* 0: English (0x0409) */
    "Weeb Labs",                    /* 1: Manufacturer */
    "Weeb Labs USBrx",              /* 2: Product */
    "0001",                         /* 3: Serial */
};

static uint16_t _desc_str[33];

uint16_t const *tud_descriptor_string_cb(uint8_t index, uint16_t langid)
{
    (void) langid;
    size_t chr_count;

    if (index == STRID_LANGID)
    {
        memcpy(&_desc_str[1], string_desc_arr[0], 2);
        chr_count = 1;
    }
    else
    {
        if (index >= sizeof(string_desc_arr) / sizeof(string_desc_arr[0]))
            return NULL;

        const char *str = string_desc_arr[index];

        chr_count = strlen(str);
        if (chr_count > 32) chr_count = 32;

        for (size_t i = 0; i < chr_count; i++)
            _desc_str[1 + i] = str[i];
    }

    /* first byte: length (incl. header), second byte: string type */
    _desc_str[0] = (uint16_t) ((TUSB_DESC_STRING << 8) | (2 * chr_count + 2));

    return _desc_str;
}
