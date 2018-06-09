#include <stdio.h>

#include "../events.h"

#include "device.h"
#include "device_push2.h"


#include <stdarg.h>
#include <memory.h>

#include <cairo.h>
#include <cairo-ft.h>

#include "../weaver.h"
#include "../hardware/screen.h"

// lua
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

// fwd decls
int init(void *self );
int deinit(void *self );
int render(void *self );

void dev_push2_midi_read(void *self, uint8_t* msg_buf, uint8_t* msg_pos, uint8_t* msg_len);
ssize_t dev_push2_midi_send(void *self, uint8_t *data, size_t n);
void dev_push2_midi_send_note(void *self, uint8_t note, uint8_t vel);
void dev_push2_midi_send_cc(void *self, uint8_t cc, uint8_t v);

void push2_handle_midi(void* self, union event_data* ev);
void push2_register_lua(void* self);

void dev_push2_grid_state(void* self, uint8_t x, uint8_t y, uint8_t z);
void dev_push2_grid_state_all(void* self, uint8_t z);
void dev_push2_grid_refresh(void* self, bool force);

#define GRID_X 16
#define GRID_Y 8
#define PUSH2_GRID_X 8
#define PUSH2_GRID_Y 8

#define GRID_STATE(b,x,y) *(b + x + (y * GRID_X) )


#define P2_MIDI_NOTE_ON     0x90
#define P2_MIDI_NOTE_OFF    0x80
#define P2_MIDI_POLY_AT     0xA0
#define P2_MIDI_CC          0xB0
#define P2_MIDI_PGM         0xC0
#define P2_MIDI_CHAN_PR     0xD0
#define P2_MIDI_PB          0xE0
#define P2_MIDI_OTHER       0xF0


// midi note/cc numbers
#define P2_NOTE_PAD_START       36
#define P2_NOTE_PAD_END         (P2_NOTE_PAD_START + 63)

#define P2_ENCODER_CC_TEMPO     14
#define P2_ENCODER_CC_SWING     15
#define P2_ENCODER_CC_START     71
#define P2_ENCODER_CC_END       (P2_ENCODER_CC_START + 7)
#define P2_ENCODER_CC_VOLUME    7

#define P2_NOTE_ENCODER_START   0
#define P2_NOTE_ENCODER_END     (P2_NOTE_ENCODER_START + 7)


#define P2_DEV_SELECT_CC_START  102
#define P2_DEV_SELECT_CC_END    (P2_DEV_SELECT_CC_START + 7)

#define P2_TRACK_SELECT_CC_START    20
#define P2_TRACK_SELECT_CC_END      (P2_TRACK_SELECT_CC_START + 7)

#define P2_USER_CC                  59
#define P2_DEVICE_CC                110
#define P2_BROWSE_CC                111

#define P2_OCTAVE_UP 55
#define P2_OCTAVE_DOWN 54
#define P2_PAGE_PREV 62
#define P2_PAGE_NEXT 63
#define P2_LAYOUT 31

#define PAD_NOTE_ON_CLR (int8_t) 127
#define PAD_NOTE_OFF_CLR (int8_t) 0
#define PAD_NOTE_ROOT_CLR (int8_t) 41
#define PAD_NOTE_IN_KEY_CLR (int8_t) 3


static const uint16_t VID = 0x2982, PID = 0x1967;

#define HDR_PKT_SZ 0x10
static uint8_t headerPkt[HDR_PKT_SZ] = { 0xFF, 0xCC, 0xAA, 0x88, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                       };


// TODO...
// push 2 native mode


// Future versions of libusb will use usb_interface instead of interface
// in libusb_config_descriptor => cater for that
#define usb_interface interface

static int pmsg(char const *format, ...) {
    va_list args;
    int r;

    va_start (args, format);
    r = vfprintf(stdout, format, args);
    va_end(args);

    return r;
}

static int perr(char const *format, ...) {
    va_list args;
    int r;

    va_start (args, format);
    r = vfprintf(stderr, format, args);
    va_end(args);

    return r;
}

#define ERR_EXIT(errcode) do { perr("   %s\n", libusb_strerror((enum libusb_error)errcode)); return -1; } while (0)
#define CALL_CHECK(fcall) do { r=fcall; if (r < 0) ERR_EXIT(r); } while (0);

struct dev_push2 *defaultPush2 = NULL;

int dev_push2_init(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    struct dev_common *base = (struct dev_common *) self;

    base->start = &dev_push2_start;
    base->deinit = &dev_push2_deinit;

    // midi
    unsigned int alsa_card;
    unsigned int alsa_dev;
    char *alsa_name;

    sscanf(base->path, "/dev/snd/midiC%uD%u", &alsa_card, &alsa_dev);

    if (asprintf(&alsa_name, "hw:%u,%u", alsa_card, alsa_dev) < 0) {
        fprintf(stderr, "push2: failed to create alsa device name for card %d,%d\n", alsa_card, alsa_dev);
        return -1;
    }

    int mode = SND_RAWMIDI_NONBLOCK;
    if (snd_rawmidi_open(&push2->handle_in, &push2->handle_out, alsa_name, mode) < 0) {
        fprintf(stderr, "push2: failed to open alsa device %s\n", alsa_name);
        return -1;
    }


    // currently hardcode, later noet
    push2->cuckoo_ = true;


    // screen
    push2->dataPkt_ = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));
    push2->imgBuf_ = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));
    push2->imgBuf2_ = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));


    push2->headerPkt_ = headerPkt;
    push2->handle_ = NULL;
    push2->iface_ = 0;
    push2->endpointOut_ = 1;

    memset(push2->imgBuf_, 0, PUSH2_DATA_PKT_SZ);
    memset(push2->imgBuf2_, 0, PUSH2_DATA_PKT_SZ);
    memset(push2->dataPkt_, 0, PUSH2_DATA_PKT_SZ);

    // screen needs to be double buffered
    push2->surfacefb = cairo_image_surface_create_for_data(
                           (unsigned char*) push2->imgBuf_,
                           CAIRO_FORMAT_RGB16_565,
                           PUSH2_WIDTH,
                           PUSH2_HEIGHT,
                           PUSH2_LINE
                       );
    push2->crfb = cairo_create (push2->surfacefb);

    push2->surface = cairo_image_surface_create_for_data(
                         (unsigned char*) push2->imgBuf2_,
                         CAIRO_FORMAT_RGB16_565,
                         PUSH2_WIDTH,
                         PUSH2_HEIGHT,
                         PUSH2_LINE
                     );
    push2->cr = cairo_create (push2->surface);



    cairo_set_operator(push2->crfb, CAIRO_OPERATOR_SOURCE);
    cairo_set_source_surface(push2->crfb, push2->surface, 0, 0);

    if (push2->cuckoo_) {
        screen_cr((void*) push2->cr, (void*) push2->crfb);
        cairo_scale(push2->cr, 2.0f, 2.0f);
    }

    init(self);

    // grid
    push2_register_lua(self);

    if (defaultPush2 == NULL) defaultPush2 = push2;

    push2->midi_octave = 5;
    push2->midi_mode = false;
    push2->grid_page = 0;
    push2->grid_state = calloc(GRID_X * GRID_Y, sizeof(uint8_t));
    push2->grid_state_buf = calloc(GRID_X * GRID_Y, sizeof(uint8_t));
    memset(push2->grid_state, 0, GRID_X * GRID_Y * sizeof(uint8_t));
    memset(push2->grid_state_buf, 0, GRID_X * GRID_Y * sizeof(uint8_t));

    dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? 0x7f : 0x04)));
    dev_push2_midi_send_cc(self, P2_OCTAVE_UP,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? 0x7f : 0x04)));

    dev_push2_midi_send_cc(self, P2_PAGE_PREV, (push2->grid_page > 0) ?  0x7f : 0x04);
    dev_push2_midi_send_cc(self, P2_PAGE_NEXT, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? 0x7f : 0x04);
    dev_push2_midi_send_cc(self, P2_LAYOUT, push2->midi_mode ? 0x7f : 0x04);

    dev_push2_grid_state_all(push2, 0);
    dev_push2_grid_refresh(self, true);

    //loop
    push2->running_ = true;
    return 0;
}

void dev_push2_deinit(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    // midi
    snd_rawmidi_close(push2->handle_in);
    snd_rawmidi_close(push2->handle_out);

    // loop
    push2->running_ = false;
    while (!push2->running_) {
        sleep(1);
    }

    push2->running_ = false;

    // screen
    deinit(self);
    cairo_destroy (push2->cr);
    cairo_surface_destroy (push2->surface);
    cairo_destroy (push2->crfb);
    cairo_surface_destroy (push2->surfacefb);

    if (push2->imgBuf_) {free(push2->imgBuf_); push2->imgBuf_ = NULL;}
    if (push2->imgBuf2_) {free(push2->imgBuf2_); push2->imgBuf2_ = NULL;}
    if (push2->dataPkt_) {free(push2->dataPkt_); push2->grid_state = NULL;}
    if (push2->grid_state) {free(push2->grid_state); push2->dataPkt_ = NULL;}
    if (push2->grid_state_buf) {free(push2->grid_state_buf); push2->grid_state_buf = NULL;}

    if (defaultPush2 == push2) defaultPush2 = NULL;

}

void* dev_push2_start(void *self) {
    perr("Push 2 render loop starting\n");
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    uint8_t msg_buf[256];
    uint8_t msg_pos = 0;
    uint8_t msg_len = 0;

    int count = 0;
    while (push2->running_) {
        dev_push2_midi_read(self, msg_buf, &msg_pos, &msg_len);
        if ((count % 32) ==0) render(self); //~30fps
        usleep(1000); // 1ms
        count++;
    }
    push2->running_ = true;
    perr("Push 2 render loop stopped\n");
    return NULL;
}


int render(void *self ) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    if (push2->handle_ == NULL) return -1;
    int tfrsize = 0;
    int r = 0;

    for (int y = 0; y < PUSH2_HEIGHT; y++) {
        for (int x = 0; x < PUSH2_LINE; x += 4) {
            int pixelOffset = (y * PUSH2_LINE) + x;
            int destinationOffset = (y * PUSH2_LINE) + x;
            // mask 0xFFE7F3E7
            push2->dataPkt_[destinationOffset]   = push2->imgBuf_[pixelOffset]   ^ 0xE7;
            push2->dataPkt_[destinationOffset + 1] = push2->imgBuf_[pixelOffset + 1] ^ 0xF3;
            push2->dataPkt_[destinationOffset + 2] = push2->imgBuf_[pixelOffset + 2] ^ 0xE7;
            push2->dataPkt_[destinationOffset + 3] = push2->imgBuf_[pixelOffset + 3] ^ 0xFF;
        }
    }

    CALL_CHECK(libusb_bulk_transfer(push2->handle_, push2->endpointOut_, push2->headerPkt_, HDR_PKT_SZ, &tfrsize, 1000));
    if (tfrsize != HDR_PKT_SZ) { perr("push2 header packet short %d\n", tfrsize); }
    CALL_CHECK(libusb_bulk_transfer(push2->handle_, push2->endpointOut_, (unsigned char *) push2->dataPkt_, PUSH2_DATA_PKT_SZ, &tfrsize, 1000));
    if (tfrsize != PUSH2_DATA_PKT_SZ) { perr("push2 data packet short %d\n", tfrsize); }

    return 0;
}

void clear(void*self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    memset(push2->imgBuf_, 0, PUSH2_DATA_PKT_SZ);
    cairo_set_source_rgb (push2->cr, 0, 0, 0);
    cairo_paint (push2->cr);
}

int init(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    const struct libusb_version *version;
    version = libusb_get_version();
    pmsg("Push2 Using libusb %d.%d.%d.%d\n", version->major, version->minor, version->micro, version->nano);

    int r = libusb_init(NULL);
    libusb_set_debug(NULL, LIBUSB_LOG_LEVEL_INFO);
    static const uint16_t vid = VID, pid = PID;

    pmsg("Opening Push2 : 0x0%4x 0x%04x\n", vid, pid);

    push2->handle_ = libusb_open_device_with_vid_pid(NULL, vid, pid);

    if (push2->handle_ == NULL) {
        perr("Push 2 open usb open failed.\n");
        return -1;
    }

    CALL_CHECK(libusb_claim_interface(push2->handle_, push2-> iface_));

    return r;
}

int deinit(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    if (push2->handle_ != NULL) {
        if (push2->iface_ != 0) libusb_release_interface(push2->handle_, push2->iface_);
        libusb_close(push2->handle_);
    }
    libusb_exit(NULL);
    return 0;
}

void dev_push2_grid_state(void* self, uint8_t x, uint8_t y, uint8_t z) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    GRID_STATE(push2->grid_state, x, y) = z;
    // *(push2->grid_state + x + (y * GRID_X) ) = z;
}

void dev_push2_grid_state_all(void* self, uint8_t z) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    for (int y = 0; y < GRID_Y; y++) {
        for (int x = 0; x < GRID_X; x++) {
            GRID_STATE(push2->grid_state, x, y) = z;
        }
    }
    dev_push2_grid_refresh(self, false);
}

void dev_push2_grid_refresh(void* self, bool force) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    uint8_t msg[3] = {P2_MIDI_NOTE_ON, P2_NOTE_PAD_START, 0};


    if (!push2->midi_mode) { // grid emulation
        int offset = push2->grid_page * PUSH2_GRID_X;
        int end = offset + PUSH2_GRID_X;

        for (int y = 0; y < GRID_Y; y++) {
            for (int x = offset; x < end; x++) {
                uint8_t z =  GRID_STATE(push2->grid_state, x, y) ;
                uint8_t z1 =  GRID_STATE(push2->grid_state_buf, x, y) ;
                if (z != z1 || force) {
                    GRID_STATE(push2->grid_state_buf, x, y) = z;
                    msg[1] = P2_NOTE_PAD_START + ((PUSH2_GRID_Y - y - 1) * PUSH2_GRID_X) + ( x - offset );
                    msg[2] = (z > 0 ? ( z == 15 ? 122 : 9 + z) : 0);
                    dev_push2_midi_send(push2, msg, 3);
                }
            }
        }
    } else {
        if (force) { //ignore update grid events.
            for (int y = 0; y < PUSH2_GRID_Y; y++ ) {
                const int rowOffset = 5;
                const int scale = 0b101011010101;
                for (int x = 0; x < PUSH2_GRID_X; x++) {
                    int clr = 0;
                    int note_s = (y * rowOffset) + x;
                    int i = note_s %  12;
                    int v = (scale & (1 << ( 11 - i)));
                    clr = (i == 0 ? PAD_NOTE_ROOT_CLR : (v > 0 ? PAD_NOTE_IN_KEY_CLR : PAD_NOTE_OFF_CLR));
                    msg[1] = P2_NOTE_PAD_START + (y * PUSH2_GRID_X)  + x;
                    msg[2] = clr;
                    dev_push2_midi_send(push2, msg, 3);
                }
            }
        }
    }
}

/***
 * grid: set led
 * @function grid_set_led
 * @param dev grid device
 * @param x x
 * @param y y
 * @param z level (0-15)
 */
int push2_grid_set_led(lua_State *l) {
    if (lua_gettop(l) != 4) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    // struct dev_push2 *md = lua_touserdata(l, 1);
    int x = (int) luaL_checkinteger(l, 2) - 1; // convert from 1-base
    int y = (int) luaL_checkinteger(l, 3) - 1; // convert from 1-base
    int z = (int) luaL_checkinteger(l, 4); // don't convert value!
    // dev_monome_set_led(md, x, y, z);
    //perr("push2_grid_set_led %d,%d,%d", x, y, z);
    dev_push2_grid_state(defaultPush2, x, y, z);
    lua_settop(l, 0);
    return 0;
}

/***
 * grid: set all LEDs
 * @function grid_all_led
 * @param dev grid device
 * @param z level (0-15)
 */
int push2_grid_all_led(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    // struct dev_push2 *md = lua_touserdata(l, 1);
    int z = (int) luaL_checkinteger(l, 2); // don't convert value!
    // dev_monome_all_led(md, z);
    //perr("push2_grid_all_led %d", z);
    dev_push2_grid_state_all(defaultPush2, z);
    lua_settop(l, 0);
    return 0;
}

/***
 * grid: refresh
 * @function grid_refresh
 * @param dev grid device
 */
int push2_grid_refresh(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    // struct dev_push2 *md = lua_touserdata(l, 1);
    //perr("push2_grid_refresh");
    dev_push2_grid_refresh(defaultPush2, false);
    lua_settop(l, 0);
    return 0;
}

/***
 * grid: rows
 * @function grid_rows
 * @param dev grid device
 */
int push2_grid_rows(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    lua_pushinteger(l, GRID_Y);
    return 1;
}

/***
 * grid: cold
 * @function grid_cols
 * @param dev grid device
 */
int push2_grid_cols(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    lua_pushinteger(l, GRID_X);
    return 1;
}

void push2_register_lua(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    lua_State* lvm = (lua_State*) luaState();
    if (push2->cuckoo_) {
        lua_register(lvm, "grid_set_led", &push2_grid_set_led);
        lua_register(lvm, "grid_all_led", &push2_grid_all_led);
        lua_register(lvm, "grid_refresh", &push2_grid_refresh);
        lua_register(lvm, "grid_rows", &push2_grid_rows);
        lua_register(lvm, "grid_cols", &push2_grid_cols);
    }
}

void dev_push2_midi_read(void *self, uint8_t* msg_buf, uint8_t* msg_pos, uint8_t* msg_len) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    union event_data *ev;

    ssize_t read = 0;
    uint8_t byte = 0;

    do {
        read = snd_rawmidi_read(push2->handle_in, &byte, 1);

        if (byte >= 0x80) {
            // control byte
            msg_buf[0] = byte;
            *msg_pos = 1;

            switch (byte & 0xf0) {
            case P2_MIDI_NOTE_ON:
            case P2_MIDI_NOTE_OFF:
            case P2_MIDI_POLY_AT:
            case P2_MIDI_CC:
            case P2_MIDI_PB: {
                *msg_len = 3;
                break;
            }
            case P2_MIDI_CHAN_PR:
            case P2_MIDI_PGM: {
                *msg_len = 2;
                break;
            }
            case P2_MIDI_OTHER: {
                switch (byte & 0x0f) {
                case 0x01: // midi time code
                case 0x03: // song selec
                    *msg_len = 2;
                    break;
                case 0x02 :
                    *msg_len = 3;
                    break;
                case 0x07: // sysex end
                    // TODO: properly handle sysex length
                    *msg_len = *msg_pos; // sysex end
                    break;
                case 0x00: // sysex start
                    *msg_len = 0;
                    break;
                case 0x08:
                    *msg_len = 1;
                    break;
                default:
                    *msg_len = 2;
                    break;
                }
                break;
            }
            default: {
                *msg_len = 2;
                break;
            }
            } //case
        } else {
            // data byes
            msg_buf[*msg_pos] = byte;
            *msg_pos = *msg_pos + 1;
        }

        if (*msg_pos == *msg_len) {
            ev = event_data_new(EVENT_MIDI_EVENT);
            ev->midi_event.id = push2->dev.id;
            ev->midi_event.data[0] = msg_buf[0];
            ev->midi_event.data[1] = *msg_len > 1 ? msg_buf[1] : 0;
            ev->midi_event.data[2] = *msg_len > 2 ? msg_buf[2] : 0;
            ev->midi_event.nbytes = *msg_len;
            push2_handle_midi(self, ev);
            *msg_pos = 0;
            *msg_len = 0;
        }
    } while (read > 0);
}

void push2_handle_midi(void* self, union event_data* evin) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    if (!push2->cuckoo_) {
        event_post(evin);
        return;
    }

    static struct timespec encoderthin[8];

    uint8_t type = evin->midi_event.data[0] & 0xF0;
    // uint8_t ch = ev->midi_event.data[0] & 0x0F;
    // determine if midi message is going to be interpretted or just sent on
    switch (type) {
    case P2_MIDI_NOTE_ON:
    case P2_MIDI_NOTE_OFF: {
        int8_t note = evin->midi_event.data[1];
        int8_t data = evin->midi_event.data[2];
        if (note >= P2_NOTE_PAD_START && note <= P2_NOTE_PAD_END) {
            if (!push2->midi_mode) {
                // send grid key event
                int x = ((note - P2_NOTE_PAD_START) % PUSH2_GRID_X  ) + (push2->grid_page * PUSH2_GRID_X);
                int y = PUSH2_GRID_Y - ((note - P2_NOTE_PAD_START) / PUSH2_GRID_X) - 1;
                union event_data *ev = event_data_new(EVENT_GRID_KEY);
                ev->grid_key.id = push2->dev.id;
                ev->grid_key.x = x;
                ev->grid_key.y = y;
                ev->grid_key.state = (type == P2_MIDI_NOTE_ON) && (data > 0);
                // fprintf(stderr, "key %d\t%d\t%d\t%d\n", ev->grid_key.id, ev->grid_key.x, ev->grid_key.y , ev->grid_key.state);
                event_post(ev);
            } else {
                const int tonic = 0;
                const int rowOffset = 5;
                int x = (note - P2_NOTE_PAD_START) % PUSH2_GRID_X;
                int y =  (note - P2_NOTE_PAD_START) / PUSH2_GRID_X;
                int noteout =  note = (push2->midi_octave * 12)  + (y * rowOffset) + x + tonic;
                evin->midi_event.data[1] = noteout;
                // fprintf(stderr, "midi  0x%02x\t%d\t%d\n", evin->midi_event.data[0], evin->midi_event.data[1], evin->midi_event.data[2]);
                event_post(evin);
            }
            return;
        }
        break;
    }
    case P2_MIDI_CC: {
        int8_t cc = evin->midi_event.data[1];
        int8_t data = evin->midi_event.data[2];
        if (cc >= P2_DEV_SELECT_CC_START  && cc <= P2_DEV_SELECT_CC_END) {
            if (cc <= P2_DEV_SELECT_CC_START + 2) {
                int8_t button = cc - P2_DEV_SELECT_CC_START;
                // send norns button evt
                union event_data *ev = event_data_new(EVENT_KEY);
                ev->enc.n = button + 1;
                ev->enc.delta = (data > 0);
                // perr("norns button %d %d\n", ev->enc.n, ev->enc.delta);
                event_post(ev);
                return;
            }
        } else if (cc >= P2_ENCODER_CC_START  && cc <= P2_ENCODER_CC_END) {
            if (cc <= P2_ENCODER_CC_START + 2) {
                // send norns encoder evt
                int8_t enc = cc - P2_ENCODER_CC_START;

                int v = data > 0x40 ? -1 : 1;
                // int v = data > 0x40 ? (0x80 - data) * -1 : data;
                // perr("norns encoder %d %04x %d\n", enc, data, v);

                struct timespec now;
                clock_gettime(CLOCK_MONOTONIC, &now); // get initial time-stamp

                double diffns = (double)(now.tv_sec - encoderthin[enc].tv_sec) * 1.0e9 +
                                (double)(now.tv_nsec - encoderthin[enc].tv_nsec);

                if (diffns > 1000000) {
                    encoderthin[enc] = now;
                    union event_data *ev = event_data_new(EVENT_ENC);
                    ev->enc.n = enc + 1;
                    ev->enc.delta = v;
                    // perr("norns encoder %d %d\n", ev->enc.n, ev->enc.delta);
                    event_post(ev);
                    return;
                }
            }
        } else {
            switch (cc) {
            case P2_PAGE_PREV : {
                if (data && !push2->midi_mode && push2->grid_page > 0)  {
                    push2->grid_page--;
                    dev_push2_midi_send_cc(self, P2_PAGE_PREV, (push2->grid_page > 0) ?  0x7f : 0x04);
                    dev_push2_midi_send_cc(self, P2_PAGE_NEXT, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? 0x7f : 0x04);
                    dev_push2_grid_refresh(self, true);
                }
                break;
            }
            case P2_PAGE_NEXT : {
                if (data && !push2->midi_mode && push2->grid_page <  ((GRID_X / PUSH2_GRID_X) - 1))  {
                    push2->grid_page++;
                    dev_push2_midi_send_cc(self, P2_PAGE_PREV, (push2->grid_page > 0) ?  0x7f : 0x04);
                    dev_push2_midi_send_cc(self, P2_PAGE_NEXT, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? 0x7f : 0x04);
                    dev_push2_grid_refresh(self, true);
                }
                break;
            }
            case P2_OCTAVE_DOWN : {
                if (data && push2->midi_mode) {
                    if (push2->midi_octave > 0) {
                        push2->midi_octave--;
                        dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? 0x7f : 0x04)));
                        dev_push2_midi_send_cc(self, P2_OCTAVE_UP,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? 0x7f : 0x04)));
                    }
                }
                break;
            }
            case P2_OCTAVE_UP : {
                if (data && push2->midi_mode) {
                    if (push2->midi_octave < 7) {
                        push2->midi_octave++;
                        dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? 0x7f : 0x04)));
                        dev_push2_midi_send_cc(self, P2_OCTAVE_UP,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? 0x7f : 0x04)));
                    }
                }
                break;
            }
            case P2_LAYOUT : {
                if (data) {
                    push2->midi_mode = ! push2->midi_mode;
                    dev_push2_midi_send_cc(self, P2_LAYOUT, push2->midi_mode ? 0x7f : 0x04);
                    if (push2->midi_mode) {
                        dev_push2_midi_send_cc(self, P2_PAGE_PREV, 0);
                        dev_push2_midi_send_cc(self, P2_PAGE_NEXT, 0);
                        dev_push2_grid_refresh(self, true);

                    } else {
                        dev_push2_midi_send_cc(self, P2_PAGE_PREV, (push2->grid_page > 0) ?  0x7f : 0x04);
                        dev_push2_midi_send_cc(self, P2_PAGE_NEXT, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? 0x7f : 0x04);
                        dev_push2_grid_refresh(self, true);
                    }
                    dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? 0x7f : 0x04)));
                    dev_push2_midi_send_cc(self, P2_OCTAVE_UP,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? 0x7f : 0x04)));
                }
                break;
            }
            }
        }
    }
    default: {
        ;
    }

    }
}

ssize_t dev_push2_midi_send(void *self, uint8_t *data, size_t n) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    return snd_rawmidi_write(push2->handle_out, data, n);
}


void dev_push2_midi_send_note(void *self, uint8_t note, uint8_t vel) {
    uint8_t msg[3] = {P2_MIDI_NOTE_ON, note, vel};
    dev_push2_midi_send(self, msg, sizeof(msg));
}

void dev_push2_midi_send_cc(void *self, uint8_t cc, uint8_t v) {
    uint8_t msg[3] = {P2_MIDI_CC, cc, v};
    dev_push2_midi_send(self, msg, sizeof(msg));
}
