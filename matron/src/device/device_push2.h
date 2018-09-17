#pragma once

#include <cairo.h>
#include <libusb.h>
#include <alsa/asoundlib.h>

#include "device_common.h"


#include <lua.h>


#define PUSH2_LINE    2048
#define PUSH2_HEIGHT  160
#define PUSH2_WIDTH   960
#define PUSH2_DATA_PKT_SZ (PUSH2_LINE * PUSH2_WIDTH)

#define PUSH2_DEV_OFFSET 10


/// note: display mode, covers both screen and screen navigation
typedef enum {
    P2DM_NORNS,
    P2DM_NATIVE
} push2_display_mode_t;



// grid events or midi notes from pads
typedef enum  {
    P2PM_GRID,
    P2PM_NOTE
} push2_pad_mode_t;



struct dev_push2 {

    // midi device
    struct dev_common dev_;
    snd_rawmidi_t *handle_in_;
    snd_rawmidi_t *handle_out_;

    bool running_;

    // screen
    push2_display_mode_t display_mode_;

    libusb_device_handle *handle_;
    int iface_;
    int endpointOut_;

    uint8_t *headerPkt_;
    unsigned char *dataPkt_;

    unsigned char *screenBuf_[2];
    cairo_surface_t *screenSurface_[2];
    cairo_t *screen_[2];

    unsigned char *pushBuf_[2];
    cairo_surface_t *pushDisplSurface_[2];
    cairo_t *pushDispl_[2];

    push2_pad_mode_t  pad_mode_;

    // grid
    uint8_t* grid_state_;
    uint8_t* grid_state_buf_;
    uint8_t  grid_page_;

    // midi
    uint8_t  midi_octave_;
    uint8_t  midi_note_state_[128];
    uint8_t  midi_cc_state_[128];
};

extern int dev_push2_init(void *self);
extern void dev_push2_deinit(void *self);
extern void* dev_push2_start(void *self);

extern void dev_push2_event(void* self, uint8_t op);

// grid api
extern int push2_grid_set_led(lua_State *l);
extern int push2_grid_all_led(lua_State *l);
extern int push2_grid_refresh(lua_State *l);
extern int push2_grid_rows(lua_State *l);
extern int push2_grid_cols(lua_State *l);
