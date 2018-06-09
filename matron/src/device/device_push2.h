#pragma once

#include <cairo.h>
#include <libusb.h>
#include <alsa/asoundlib.h>

#include "device_common.h"

#define PUSH2_LINE    2048
#define PUSH2_HEIGHT  160
#define PUSH2_WIDTH   960
#define PUSH2_DATA_PKT_SZ (PUSH2_LINE * PUSH2_WIDTH)


struct dev_push2 {
    struct dev_common dev;

    bool cuckoo_;
	bool running_;

    // screen
    libusb_device_handle *handle_;
    int iface_;
    int endpointOut_;

    uint8_t *headerPkt_;

    unsigned char *dataPkt_;
    unsigned char *imgBuf_;
    unsigned char *imgBuf2_;


    cairo_surface_t *surfacefb;
    cairo_t *crfb;

	cairo_surface_t *surface;
	cairo_t *cr;

    // midi
    snd_rawmidi_t *handle_in;
    snd_rawmidi_t *handle_out;

    // grid
    uint8_t* grid_state;
    uint8_t* grid_state_buf;
    uint8_t  grid_page;
    bool     midi_mode;
    uint8_t  midi_octave;
};

extern int dev_push2_init(void *self);
extern void dev_push2_deinit(void *self);
extern void* dev_push2_start(void *self);
