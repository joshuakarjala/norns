#include <stdio.h>

#include "../events.h"

#include "device.h"
#include "device_push2.h"


#include <stdarg.h>
#include <memory.h>
#include <pthread.h>


#include <cairo.h>
#include <cairo-ft.h>

#include "../weaver.h"
#include "../hardware/screen.h"

// lua
// #include <lua.h>
// #include <lualib.h>
#include <lauxlib.h>

// fwd decls
int init(void *self );
int deinit(void *self );
int render(void *self );

void dev_push2_midi_read(void *self, uint8_t* msg_buf, uint8_t* msg_pos, uint8_t* msg_len);
ssize_t dev_push2_midi_send(void *self, uint8_t *data, size_t n);
void dev_push2_midi_send_note(void *self, uint8_t note, uint8_t vel);
void dev_push2_midi_send_cc(void *self, uint8_t cc, uint8_t v);


void dev_push2_event_send(void* self, uint8_t op);

void push2_handle_midi(void* self, union event_data* ev);
void push2_register_lua(void* self);

void dev_push2_grid_state(void* self, uint8_t x, uint8_t y, uint8_t z);
void dev_push2_grid_state_all(void* self, uint8_t z);
void dev_push2_grid_refresh(void* self, bool force);
void push2d_init();


void dev_push2_cursor_left(void *self);
void dev_push2_cursor_right(void *self); 
void dev_push2_layout(void *self);




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

#define P2_SETUP_CC                  30
#define P2_USER_CC                  59
#define P2_DEVICE_CC                110
#define P2_BROWSE_CC                111



#define P2_CURSOR_RIGHT_CC 45
#define P2_CURSOR_LEFT_CC 44
#define P2_CURSOR_UP_CC 46
#define P2_CURSOR_DOWN_CC 47


#define P2_OCTAVE_UP_CC 55
#define P2_OCTAVE_DOWN_CC 54
#define P2_PAGE_PREV_CC 62
#define P2_PAGE_NEXT_CC 63


#define P2_LAYOUT_CC 31

#define PAD_NOTE_ON_CLR (uint8_t) 127
#define PAD_NOTE_OFF_CLR (uint8_t) 0
#define PAD_NOTE_ROOT_CLR (uint8_t) 41
#define PAD_NOTE_IN_KEY_CLR (uint8_t) 3

#define P2_CLR_W_AVAIL 0x10
#define P2_CLR_W_ON 0x7f


#define P2_OP_LAYOUT 0
#define P2_OP_CURSOR_LEFT 1
#define P2_OP_CURSOR_RIGHT 2


static const uint16_t VID = 0x2982, PID = 0x1967;

#define HDR_PKT_SZ 0x10
static uint8_t headerPkt[HDR_PKT_SZ] = { 0xFF, 0xCC, 0xAA, 0x88, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                       };

// default screen context (e.g. norn hardware)
void* defaultScreen = NULL;
void* defaultScreenFB = NULL;


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


// theoretically required due to render thread, but unlikely
// so removed lock calls for now
pthread_mutex_t push2_midilock;

//// START OF PUBLIC INTERFACE

int dev_push2_init(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    struct dev_common *base = (struct dev_common *) self;

    base->start = &dev_push2_start;
    base->deinit = &dev_push2_deinit;


    pthread_mutex_init(&push2_midilock,0);

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

    for(int i=0;i<128;i++) {
        push2->midi_note_state[i] = 0;
        push2->midi_cc_state[i] = 0;
    }

    push2->cuckoo_ = true;


    push2->dataPkt_ = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));
    memset(push2->dataPkt_, 0, PUSH2_DATA_PKT_SZ);
    push2->headerPkt_ = headerPkt;
    push2->handle_ = NULL;
    push2->iface_ = 0;
    push2->endpointOut_ = 1;

    // screen
    push2->screenBuf_[0] = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));
    push2->screenBuf_[1] = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));
    memset(push2->screenBuf_[0], 0, PUSH2_DATA_PKT_SZ);
    memset(push2->screenBuf_[1], 0, PUSH2_DATA_PKT_SZ);

    // screen needs to be double buffered
    push2->screenSurface_[0] = cairo_image_surface_create_for_data(
                                   (unsigned char*) push2->screenBuf_[0],
                                   CAIRO_FORMAT_RGB16_565,
                                   PUSH2_WIDTH,
                                   PUSH2_HEIGHT,
                                   PUSH2_LINE
                               );
    push2->screen_[0] = cairo_create (push2->screenSurface_[0]);

    push2->screenSurface_[1] = cairo_image_surface_create_for_data(
                                   (unsigned char*) push2->screenBuf_[1],
                                   CAIRO_FORMAT_RGB16_565,
                                   PUSH2_WIDTH,
                                   PUSH2_HEIGHT,
                                   PUSH2_LINE
                               );
    push2->screen_[1] = cairo_create (push2->screenSurface_[1]);
    cairo_scale(push2->screen_[1], 2.0f, 2.0f);

    if(defaultScreen == NULL) {
        // assumes screen has been initialised before push
        // only do once, since push will overwrite
        // idea: potentially this allows us to reset it back to norns display
        screen_context(&defaultScreen, &defaultScreenFB);
    }


    if (push2->cuckoo_) {
        // hijack default norns screen
        screen_cr((void*) push2->screen_[1], (void*) push2->screen_[0]);
    }
    cairo_set_operator(push2->screen_[0], CAIRO_OPERATOR_SOURCE);
    cairo_set_source_surface(push2->screen_[0], push2->screenSurface_[1], 0, 0);

    // push 2 native mode
    push2->pushBuf_[0] = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));
    push2->pushBuf_[1] = calloc(PUSH2_DATA_PKT_SZ, sizeof(unsigned char));
    memset(push2->pushBuf_[0], 0, PUSH2_DATA_PKT_SZ);
    memset(push2->pushBuf_[1], 0, PUSH2_DATA_PKT_SZ);
    // screen needs to be double buffered
    push2->pushDisplSurface_[0] = cairo_image_surface_create_for_data(
                                      (unsigned char*) push2->pushBuf_[0],
                                      CAIRO_FORMAT_RGB16_565,
                                      PUSH2_WIDTH,
                                      PUSH2_HEIGHT,
                                      PUSH2_LINE
                                  );
    push2->pushDispl_[0] = cairo_create (push2->pushDisplSurface_[0]);

    push2->pushDisplSurface_[1] = cairo_image_surface_create_for_data(
                                      (unsigned char*) push2->pushBuf_[1],
                                      CAIRO_FORMAT_RGB16_565,
                                      PUSH2_WIDTH,
                                      PUSH2_HEIGHT,
                                      PUSH2_LINE
                                  );
    push2->pushDispl_[1] = cairo_create (push2->pushDisplSurface_[1]);

    push2d_init(self);
    cairo_set_operator(push2->pushDispl_[0], CAIRO_OPERATOR_SOURCE);
    cairo_set_source_surface(push2->pushDispl_[0], push2->pushDisplSurface_[1], 0, 0);


    init(self);

    // grid
    push2_register_lua(self);

    if (defaultPush2 == NULL) defaultPush2 = push2;

    push2->midi_octave = 1;
    push2->midi_mode = false;
    push2->grid_page = 0;
    push2->grid_state = calloc(GRID_X * GRID_Y, sizeof(uint8_t));
    push2->grid_state_buf = calloc(GRID_X * GRID_Y, sizeof(uint8_t));
    memset(push2->grid_state, 0, GRID_X * GRID_Y * sizeof(uint8_t));
    memset(push2->grid_state_buf, 0, GRID_X * GRID_Y * sizeof(uint8_t));

    dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN_CC,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));
    dev_push2_midi_send_cc(self, P2_OCTAVE_UP_CC,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));

    dev_push2_midi_send_cc(self, P2_CURSOR_LEFT_CC, (push2->grid_page > 0) ?  P2_CLR_W_ON : P2_CLR_W_AVAIL);
    dev_push2_midi_send_cc(self, P2_CURSOR_RIGHT_CC, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
    dev_push2_midi_send_cc(self, P2_LAYOUT_CC, push2->midi_mode ? P2_CLR_W_ON : P2_CLR_W_AVAIL);


    dev_push2_midi_send_cc(self, P2_USER_CC, push2->cuckoo_ ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
    dev_push2_midi_send_cc(self, P2_SETUP_CC, push2->cuckoo_ ? 0 : P2_CLR_W_AVAIL);

    for(int i=P2_DEV_SELECT_CC_START;i<=P2_DEV_SELECT_CC_END;i++) {
        dev_push2_midi_send_cc(self, i, (i> P2_DEV_SELECT_CC_START + 2) ? 0 : P2_CLR_W_AVAIL);
    }

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
    cairo_destroy (push2->screen_[1]);
    cairo_surface_destroy (push2->screenSurface_[1]);
    cairo_destroy (push2->screen_[0]);
    cairo_surface_destroy (push2->screenSurface_[0]);
    cairo_destroy (push2->pushDispl_[1]);
    cairo_surface_destroy (push2->pushDisplSurface_[1]);
    cairo_destroy (push2->pushDispl_[0]);
    cairo_surface_destroy (push2->pushDisplSurface_[0]);

    if (push2->screenBuf_[0]) {free(push2->screenBuf_[0]); push2->screenBuf_[0] = NULL;}
    if (push2->screenBuf_[1]) {free(push2->screenBuf_[1]); push2->screenBuf_[1] = NULL;}
    if (push2->pushBuf_[0]) {free(push2->pushBuf_[0]); push2->pushBuf_[0] = NULL;}
    if (push2->pushBuf_[1]) {free(push2->pushBuf_[1]); push2->pushBuf_[1] = NULL;}
    if (push2->dataPkt_) {free(push2->dataPkt_); push2->grid_state = NULL;}
    if (push2->grid_state) {free(push2->grid_state); push2->dataPkt_ = NULL;}
    if (push2->grid_state_buf) {free(push2->grid_state_buf); push2->grid_state_buf = NULL;}

    if (defaultPush2 == push2) defaultPush2 = NULL;

    pthread_mutex_destroy(&push2_midilock);


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

        if ((count % 32) == 0) render(self); //~30fps
	else if ((count % 16) == 0) {  // how often to refresh leds
            unsigned sendcount = 64; // how many in one go

            uint8_t msg[3] = {P2_MIDI_NOTE_ON, 0, 0};
	    uint8_t n = sizeof(msg);
            msg[0] = P2_MIDI_NOTE_ON;
            for(int i=0;i<128 && sendcount > 0 ;i++ ) {

                // pthread_mutex_lock(&push2_midilock);
                unsigned v = push2->midi_note_state[i];

                if(v < 128 ) {
                    msg[1] = i;
                    msg[2] = v;
                    if(dev_push2_midi_send(self, msg,n)==n ) {
			    push2->midi_note_state[i] = v | 0b10000000;
			    sendcount--;
		    }
		    else {
			    sendcount=0;
		    }
                }
                // pthread_mutex_unlock(&push2_midilock);
            }

            msg[0] = P2_MIDI_CC;
            for(int i=0;i<128 && sendcount > 0 ;i++ ) {

                // pthread_mutex_lock(&push2_midilock);
                unsigned v = push2->midi_cc_state[i];

                if(v < 128 ) {
                    msg[1] = i;
                    msg[2] = v;
                    if(dev_push2_midi_send(self, msg,n)==n ) {
			    push2->midi_cc_state[i] = v | 0b10000000;
			    sendcount--;
		    }
		    else {
			    sendcount=0;
		    }
                }
                // pthread_mutex_unlock(&push2_midilock);
            }
        } // midi send
        usleep(1000); // 1ms
        count++;

    }
    push2->running_ = true;
    perr("Push 2 render loop stopped\n");
    return NULL;
}



void dev_push2_event(void* self, uint8_t op) {
    switch(op) {
        case P2_OP_LAYOUT : {
            dev_push2_layout(self);
            break;
        }
        case P2_OP_CURSOR_LEFT : {
            dev_push2_cursor_left(self);
            break;
        }
        case P2_OP_CURSOR_RIGHT : {
            dev_push2_cursor_right(self);
            break;
        }
        default:
        break;
    }

}

//// END OF PUBLIC INTERFACE


int render(void *self ) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    if (push2->handle_ == NULL) return -1;
    int tfrsize = 0;
    int r = 0;
    unsigned char* scrbuf = push2->screenBuf_[0];
    if (!push2->cuckoo_) scrbuf = push2->pushBuf_[0];

    for (int y = 0; y < PUSH2_HEIGHT; y++) {
        for (int x = 0; x < PUSH2_LINE; x += 4) {
            int pixelOffset = (y * PUSH2_LINE) + x;
            int destinationOffset = (y * PUSH2_LINE) + x;
            // mask 0xFFE7F3E7
            push2->dataPkt_[destinationOffset]   = scrbuf[pixelOffset]   ^ 0xE7;
            push2->dataPkt_[destinationOffset + 1] = scrbuf[pixelOffset + 1] ^ 0xF3;
            push2->dataPkt_[destinationOffset + 2] = scrbuf[pixelOffset + 2] ^ 0xE7;
            push2->dataPkt_[destinationOffset + 3] = scrbuf[pixelOffset + 3] ^ 0xFF;
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

    memset(push2->screenBuf_[0], 0, PUSH2_DATA_PKT_SZ);
    cairo_set_source_rgb (push2->screen_[1], 0, 0, 0);
    cairo_paint (push2->screen_[1]);
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

void dev_push2_event_send(void* self, uint8_t op) {
    union event_data *ev = event_data_new(EVENT_PUSH2_EVENT);
    ev->push2_event.dev = self;
    ev->push2_event.op = op;
    event_post(ev);
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
}

void dev_push2_grid_refresh(void* self, bool force) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;


    if (!push2->midi_mode) { // grid emulation
        int offset = push2->grid_page * PUSH2_GRID_X;
        int end = offset + PUSH2_GRID_X;

        for (int y = 0; y < GRID_Y; y++) {
            for (int x = offset; x < end; x++) {
                uint8_t z =  GRID_STATE(push2->grid_state, x, y) ;
                uint8_t z1 =  GRID_STATE(push2->grid_state_buf, x, y) ;
                if (z != z1 || force) {
                    GRID_STATE(push2->grid_state_buf, x, y) = z;
                    unsigned note = P2_NOTE_PAD_START + ((PUSH2_GRID_Y - y - 1) * PUSH2_GRID_X) + ( x - offset );
                    unsigned vel = (z > 0 ? ( z == 15 ? 122 : 9 + z) : 0);
                    dev_push2_midi_send_note(self,note,vel);
                }
            }
        }
    } else {
        if (force) { //ignore update grid events.
            for (int y = 0; y < PUSH2_GRID_Y; y++ ) {
                const int rowOffset = 5;
                const int scale = 0b101011010101;
                for (int x = 0; x < PUSH2_GRID_X; x++) {
                    int note_s = (y * rowOffset) + x;
                    int i = note_s %  12;
                    int v = (scale & (1 << ( 11 - i)));
                    uint8_t clr = (i == 0 ? PAD_NOTE_ROOT_CLR : (v > 0 ? PAD_NOTE_IN_KEY_CLR : PAD_NOTE_OFF_CLR));
                    uint8_t note = P2_NOTE_PAD_START + (y * PUSH2_GRID_X)  + x;
                    dev_push2_midi_send_note(self,note,clr);
                }
            }
        }
    }
}




//=========================================================================================
// MIDI
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
            ev->midi_event.id = push2->dev.id + PUSH2_DEV_OFFSET;
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

    uint8_t type = evin->midi_event.data[0] & 0xF0;

    if (type == P2_MIDI_CC) {
        int8_t cc = evin->midi_event.data[1];
        int8_t data = evin->midi_event.data[2];
        if (cc == P2_USER_CC && data > 0) {
            push2->cuckoo_ = ! push2->cuckoo_;
            dev_push2_midi_send_cc(self, P2_USER_CC,  push2->cuckoo_ ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
            dev_push2_midi_send_cc(self, P2_SETUP_CC, push2->cuckoo_ ? 0 : P2_CLR_W_AVAIL);
            for(int i=P2_DEV_SELECT_CC_START;i<=P2_DEV_SELECT_CC_END;i++) {
                dev_push2_midi_send_cc(self, i, (i> P2_DEV_SELECT_CC_START + 2 || !push2->cuckoo_) 
                                        ? 0 : P2_CLR_W_AVAIL);
            }
            return;
        }
        if (cc == P2_SETUP_CC) {
            // setup acts as norns button 1
            union event_data *ev = event_data_new(EVENT_KEY);
            ev->enc.n = 1;
            ev->enc.delta = (data > 0);
            // perr("norns button %d %d\n", ev->enc.n, ev->enc.delta);
            event_post(ev);
            return;

        }
    }

    if (!push2->cuckoo_) {
        event_post(evin);
        return;
    }

    static struct timespec encoderthin[8];

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
                ev->grid_key.id = push2->dev.id + PUSH2_DEV_OFFSET;
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
                int noteout = (push2->midi_octave * 12)  + (y * rowOffset) + x + tonic;
                evin->midi_event.data[1] = noteout;
                // fprintf(stderr, "midi  0x%02x\t%d\t%d\n", evin->midi_event.data[0], evin->midi_event.data[1], evin->midi_event.data[2]);
                event_post(evin);

	        uint8_t clr = PAD_NOTE_ON_CLR;
		if(type == P2_MIDI_NOTE_OFF || data == 0) {
                    const int scale = 0b101011010101;
                    int note_s = (y * rowOffset) + x;
                    int i = note_s %  12;
                    int v = (scale & (1 << ( 11 - i)));
	            clr = (i == 0 ? PAD_NOTE_ROOT_CLR : (v > 0 ? PAD_NOTE_IN_KEY_CLR : PAD_NOTE_OFF_CLR));
		}
	        dev_push2_midi_send_note(self,note,clr);
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
            case P2_CURSOR_LEFT_CC : {
                if (data)  {
                    dev_push2_event_send(self,P2_OP_CURSOR_LEFT);
                }
                break;
            }
            case P2_CURSOR_RIGHT_CC : {
                if (data)  {
                    dev_push2_event_send(self,P2_OP_CURSOR_RIGHT);
                }
                break;
            }
            case P2_OCTAVE_DOWN_CC : {
                if (data && push2->midi_mode) {
                    if (push2->midi_octave > 0) {
                        push2->midi_octave--;
                        dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN_CC,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));
                        dev_push2_midi_send_cc(self, P2_OCTAVE_UP_CC,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));
                    }
                }
                break;
            }
            case P2_OCTAVE_UP_CC : {
                if (data && push2->midi_mode) {
                    if (push2->midi_octave < 7) {
                        push2->midi_octave++;
                        dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN_CC,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));
                        dev_push2_midi_send_cc(self, P2_OCTAVE_UP_CC,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));
                    }
                }
                break;
            }
            case P2_LAYOUT_CC : {
                if (data) {
                    dev_push2_event_send(self,P2_OP_LAYOUT);
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

void dev_push2_cursor_left(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    if (!push2->midi_mode && push2->grid_page > 0)  {
        push2->grid_page--;
        dev_push2_midi_send_cc(self, P2_CURSOR_LEFT_CC, (push2->grid_page > 0) ?  P2_CLR_W_ON : P2_CLR_W_AVAIL);
        dev_push2_midi_send_cc(self, P2_CURSOR_RIGHT_CC, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
        dev_push2_grid_refresh(self,true);
    }
}

void dev_push2_cursor_right(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    if (!push2->midi_mode && push2->grid_page <  ((GRID_X / PUSH2_GRID_X) - 1))  {
        push2->grid_page++;
        dev_push2_midi_send_cc(self, P2_CURSOR_LEFT_CC, (push2->grid_page > 0) ?  P2_CLR_W_ON : P2_CLR_W_AVAIL);
        dev_push2_midi_send_cc(self, P2_CURSOR_RIGHT_CC, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
        dev_push2_grid_refresh(self,true);
    }
}


void dev_push2_layout(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    push2->midi_mode = ! push2->midi_mode;
    dev_push2_midi_send_cc(self, P2_LAYOUT_CC, push2->midi_mode ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
    if (push2->midi_mode) {
        dev_push2_midi_send_cc(self, P2_CURSOR_LEFT_CC, 0);
        dev_push2_midi_send_cc(self, P2_CURSOR_RIGHT_CC, 0);
    } else {
        dev_push2_midi_send_cc(self, P2_CURSOR_LEFT_CC, (push2->grid_page > 0) ?  P2_CLR_W_ON : P2_CLR_W_AVAIL);
        dev_push2_midi_send_cc(self, P2_CURSOR_RIGHT_CC, (push2->grid_page < (GRID_X / PUSH2_GRID_X) - 1) ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
    }
    dev_push2_grid_refresh(self,true);
    dev_push2_midi_send_cc(self, P2_OCTAVE_DOWN_CC,    (!push2->midi_mode ? 0x00 : (push2->midi_octave > 0 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));
    dev_push2_midi_send_cc(self, P2_OCTAVE_UP_CC,      (!push2->midi_mode ? 0x00 : (push2->midi_octave < 7 ? P2_CLR_W_ON : P2_CLR_W_AVAIL)));
}

ssize_t dev_push2_midi_send(void *self, uint8_t *data, size_t n) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    if(snd_rawmidi_write(push2->handle_out, data, n) == (int) n) {
	    if(snd_rawmidi_drain(push2->handle_out) >=0) {
		    return n;
	    }
    }
    return -1;
}


void dev_push2_midi_send_note(void *self, uint8_t note, uint8_t vel) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    // pthread_mutex_lock(&push2_midilock);
    if(vel != ( push2->midi_note_state[note] & 0b01111111 )) {
        push2->midi_note_state[note] = vel;
    }
    // pthread_mutex_unlock(&push2_midilock);

}

void dev_push2_midi_send_cc(void *self, uint8_t cc, uint8_t v) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    // pthread_mutex_lock(&push2_midilock);
    if(v != ( push2->midi_cc_state[cc] & 0b01111111)) {
        push2->midi_cc_state[cc] = v;
    }
    // pthread_mutex_unlock(&push2_midilock);
}

//=========================================================================================
// LUA
//=========================================================================================

// Monone grid ============================================================================
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
    struct dev_push2 *push2 = lua_touserdata(l, 1);
    int x = (int) luaL_checkinteger(l, 2) - 1; // convert from 1-base
    int y = (int) luaL_checkinteger(l, 3) - 1; // convert from 1-base
    int z = (int) luaL_checkinteger(l, 4); // don't convert value!
    dev_push2_grid_state(push2, x, y, z);
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
    struct dev_push2 *push2 = lua_touserdata(l, 1);
    int z = (int) luaL_checkinteger(l, 2); // don't convert value!
    //perr("push2_grid_all_led %d", z);
    dev_push2_grid_state_all(push2, z);
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
    struct dev_push2 *push2 = lua_touserdata(l, 1);
    //perr("push2_grid_refresh");
    dev_push2_grid_refresh(push2, false);
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

// Native display



#define NUM_FONTS 14
static char font_path[NUM_FONTS][32];
static cairo_font_face_t *ct[NUM_FONTS];
static FT_Library value;
static FT_Error status;
static FT_Face face[NUM_FONTS];

void push2d_init(void* self) {
    struct dev_push2 *dev = (struct dev_push2 *) self;
    status = FT_Init_FreeType(&value);
    if (status != 0) {
        fprintf(stderr, "ERROR (pish2) freetype init\n");
        return;
    }

    strcpy(font_path[0], "04B_03__.TTF");
    strcpy(font_path[1], "liquid.ttf");
    strcpy(font_path[2], "Roboto-Thin.ttf");
    strcpy(font_path[3], "Roboto-Light.ttf");
    strcpy(font_path[4], "Roboto-Regular.ttf");
    strcpy(font_path[5], "Roboto-Medium.ttf");
    strcpy(font_path[6], "Roboto-Bold.ttf");
    strcpy(font_path[7], "Roboto-Black.ttf");
    strcpy(font_path[8], "Roboto-ThinItalic.ttf");
    strcpy(font_path[9], "Roboto-LightItalic.ttf");
    strcpy(font_path[10], "Roboto-Italic.ttf");
    strcpy(font_path[11], "Roboto-MediumItalic.ttf");
    strcpy(font_path[12], "Roboto-BoldItalic.ttf");
    strcpy(font_path[13], "Roboto-BlackItalic.ttf");

    char filename[256];

    for (int i = 0; i < NUM_FONTS; i++) {
        // FIXME should be path relative to norns/
        snprintf(filename, 256, "%s/norns/resources/%s", getenv("HOME"), font_path[i]);

        status = FT_New_Face(value, filename, 0, &face[i]);
        if (status != 0) {
            fprintf(stderr, "ERROR (push2) font load: %s\n", filename);
            return;
        }
        else {
            ct[i] = cairo_ft_font_face_create_for_ft_face(face[i], 0);
        }
    }



    cairo_set_operator(dev->pushDispl_[1], CAIRO_OPERATOR_CLEAR);
    cairo_paint(dev->pushDispl_[1]);
    cairo_set_operator(dev->pushDispl_[1], CAIRO_OPERATOR_OVER);

    cairo_font_options_t *font_options;
    font_options = cairo_font_options_create();
    cairo_font_options_set_antialias(font_options, CAIRO_ANTIALIAS_SUBPIXEL);

    cairo_set_font_face (dev->pushDispl_[1], ct[0]);
    cairo_set_font_options(dev->pushDispl_[1], font_options);
    cairo_set_font_size(dev->pushDispl_[1], 8.0);
}



int _push2d_font_face(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    int x = (int) luaL_checkinteger(l, 2);
    if ( (x >= 0) && (x < NUM_FONTS) ) {
        cairo_set_font_face(dev->pushDispl_[1], ct[x]);
    }
    lua_settop(l, 0);
    return 0;
}

int _push2d_font_size(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    int z = (int) luaL_checknumber(l, 2);
    cairo_set_font_size(dev->pushDispl_[1], z);
    lua_settop(l, 0);
    return 0;
}


int _push2d_aa(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    int s = (int) luaL_checkinteger(l, 2);
    if (s == 0) {
        cairo_set_antialias(dev->pushDispl_[1], CAIRO_ANTIALIAS_NONE);
    } else {
        cairo_set_antialias(dev->pushDispl_[1], CAIRO_ANTIALIAS_DEFAULT);
    }
    lua_settop(l, 0);
    return 0;
}

int _push2d_colour(lua_State *l) {
    if (lua_gettop(l) != 4) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    double r =  luaL_checknumber(l, 2);
    double g =  luaL_checknumber(l, 3);
    double b =  luaL_checknumber(l, 4);
    // push2 is BGR - not RGB
    cairo_set_source_rgb(dev->pushDispl_[1], b, g, r);
    lua_settop(l, 0);
    return 0;
}

int _push2d_line_width(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    double w = luaL_checknumber(l, 2);
    cairo_set_line_width(dev->pushDispl_[1], w);
    lua_settop(l, 0);
    return 0;
}


int _push2d_line_cap(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    const char *style = luaL_checkstring(l, 2);

    if(strcmp(style, "round") == 0){
      cairo_set_line_cap(dev->pushDispl_[1],CAIRO_LINE_CAP_ROUND);
    }else if(strcmp(style, "square") == 0){
      cairo_set_line_cap(dev->pushDispl_[1],CAIRO_LINE_CAP_SQUARE);
    }else{
      cairo_set_line_cap(dev->pushDispl_[1],CAIRO_LINE_CAP_BUTT);
    }

    lua_settop(l, 0);
    return 0;
}

int _push2d_line_join(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    const char *style = luaL_checkstring(l, 2);

    if(strcmp(style, "round") == 0){
      cairo_set_line_join(dev->pushDispl_[1],CAIRO_LINE_JOIN_ROUND);
    }else if(strcmp(style, "bevel") == 0){
      cairo_set_line_join(dev->pushDispl_[1],CAIRO_LINE_JOIN_BEVEL);
    }else{
      cairo_set_line_join(dev->pushDispl_[1],CAIRO_LINE_JOIN_MITER);
    }

    lua_settop(l, 0);
    return 0;
}

int _push2d_miter_limit(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    struct dev_push2 *dev = lua_touserdata(l, 1);
    double limit = luaL_checknumber(l, 2);
    cairo_set_miter_limit(dev->pushDispl_[1],limit);

    lua_settop(l, 0);
    return 0;
}

int _push2d_move(lua_State *l) {
    if (lua_gettop(l) != 3) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    double x = luaL_checknumber(l, 2);
    double y = luaL_checknumber(l, 3);
    cairo_move_to(dev->pushDispl_[1], x, y);
    lua_settop(l, 0);
    return 0;
}

int _push2d_line(lua_State *l) {
    if (lua_gettop(l) != 3) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    double x = luaL_checknumber(l, 2);
    double y = luaL_checknumber(l, 3);
    cairo_line_to(dev->pushDispl_[1], x, y);
    lua_settop(l, 0);
    return 0;
}

int _push2d_move_rel(lua_State *l) {
    if (lua_gettop(l) != 3) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    double x = luaL_checknumber(l, 2);
    double y = luaL_checknumber(l, 3);
    cairo_rel_move_to(dev->pushDispl_[1], x, y);
    lua_settop(l, 0);
    return 0;
}

int _push2d_line_rel(lua_State *l) {
    if (lua_gettop(l) != 3) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    double x = (int) luaL_checknumber(l, 2);
    double y = (int) luaL_checknumber(l, 3);
    cairo_rel_line_to(dev->pushDispl_[1], x, y);
    lua_settop(l, 0);
    return 0;
}

int _push2d_curve(lua_State *l) {
    if (lua_gettop(l) != 7) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    double x1 = luaL_checknumber(l, 2);
    double y1 = luaL_checknumber(l, 3);
    double x2 = luaL_checknumber(l, 4);
    double y2 = luaL_checknumber(l, 5);
    double x3 = luaL_checknumber(l, 6);
    double y3 = luaL_checknumber(l, 7);
    cairo_curve_to(dev->pushDispl_[1], x1, y1, x2, y2, x3, y3);
    lua_settop(l, 0);
    return 0;
}

int _push2d_curve_rel(lua_State *l) {
    if (lua_gettop(l) != 7) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);
    double x1 = luaL_checknumber(l, 2);
    double y1 = luaL_checknumber(l, 3);
    double x2 = luaL_checknumber(l, 4);
    double y2 = luaL_checknumber(l, 5);
    double x3 = luaL_checknumber(l, 6);
    double y3 = luaL_checknumber(l, 7);
    cairo_rel_curve_to(dev->pushDispl_[1], x1, y1, x2, y2, x3, y3);
    lua_settop(l, 0);
    return 0;
}

int _push2d_arc(lua_State *l) {
    if (lua_gettop(l) != 6) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    double x = luaL_checknumber(l, 2);
    double y = luaL_checknumber(l, 3);
    double r = luaL_checknumber(l, 4);
    double a1 = luaL_checknumber(l, 5);
    double a2 = luaL_checknumber(l, 6);
    cairo_arc(dev->pushDispl_[1], x,  y, r, a1, a2);
    lua_settop(l, 0);
    return 0;
}

int _push2d_rect(lua_State *l) {
    if (lua_gettop(l) != 5) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    double x = luaL_checknumber(l, 2);
    double y = luaL_checknumber(l, 3);
    double w = luaL_checknumber(l, 4);
    double h = luaL_checknumber(l, 5);
    cairo_rectangle(dev->pushDispl_[1], x, y, w, h);
    lua_settop(l, 0);
    return 0;
}

int _push2d_stroke(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    cairo_stroke(dev->pushDispl_[1]);
    lua_settop(l, 0);
    return 0;
}

int _push2d_fill(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    cairo_fill(dev->pushDispl_[1]);
    lua_settop(l, 0);
    return 0;
}

int _push2d_text(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    const char *s = luaL_checkstring(l, 2);
    cairo_show_text(dev->pushDispl_[1], s);
    lua_settop(l, 0);
    return 0;
}

int _push2d_clear(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    cairo_set_operator(dev->pushDispl_[1], CAIRO_OPERATOR_CLEAR);
    cairo_paint(dev->pushDispl_[1]);
    cairo_set_operator(dev->pushDispl_[1], CAIRO_OPERATOR_OVER);
    lua_settop(l, 0);
    return 0;
}

int _push2d_close(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    cairo_close_path(dev->pushDispl_[1]);
    lua_settop(l, 0);
    return 0;
}

int _push2d_extents(lua_State *l) {
    static double text_xy[2];
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }
    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *dev = lua_touserdata(l, 1);

    const char *s = luaL_checkstring(l, 2);
    cairo_text_extents_t extents;
    cairo_text_extents(dev->pushDispl_[1], s, &extents);
    text_xy[0] = extents.width;
    text_xy[1] = extents.height;
    lua_pushinteger(l, text_xy[0]);
    lua_pushinteger(l, text_xy[1]);
    return 2;
}



int _push2d_update(lua_State *l) {
    if (lua_gettop(l) == 0) {
        return luaL_error(l, "wrong number of arguments");
    }

    luaL_checktype(l, 1, LUA_TLIGHTUSERDATA);
    struct dev_push2 *push2 = lua_touserdata(l, 1);

    if (push2 == defaultPush2) {
        defaultPush2->cuckoo_ = false;
        dev_push2_midi_send_cc(push2, P2_USER_CC,  push2->cuckoo_ ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
        dev_push2_midi_send_cc(push2, P2_SETUP_CC, push2->cuckoo_ ? 0 : P2_CLR_W_AVAIL);
    }

    cairo_paint(push2->pushDispl_[0]);
    lua_settop(l, 0);
    return 0;
}

// screen
int _push2_screen_update(lua_State *l) {
    if (lua_gettop(l) != 0) {
        return luaL_error(l, "wrong number of arguments");
    }

    defaultPush2->cuckoo_ = true;
    dev_push2_midi_send_cc(defaultPush2, P2_USER_CC,  defaultPush2->cuckoo_ ? P2_CLR_W_ON : P2_CLR_W_AVAIL);
    dev_push2_midi_send_cc(defaultPush2, P2_SETUP_CC, defaultPush2->cuckoo_ ? 0 : P2_CLR_W_AVAIL);
    for(int i=P2_DEV_SELECT_CC_START;i<=P2_DEV_SELECT_CC_END;i++) {
        dev_push2_midi_send_cc(defaultPush2, i, (i> P2_DEV_SELECT_CC_START + 2) ? 0 : P2_CLR_W_AVAIL);
    }

    screen_update();
    lua_settop(l, 0);
    return 0;
}

//=========================================================================================
// Register lua
void push2_register_lua(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;

    lua_State* lvm = (lua_State*) luaState();
    if (push2->cuckoo_) {
        lua_register(lvm, "s_update", &_push2_screen_update);
    }

    lua_register(lvm, "p2_update", &_push2d_update);
    lua_register(lvm, "p2_font_face", &_push2d_font_face);
    lua_register(lvm, "p2_font_size", &_push2d_font_size);
    lua_register(lvm, "p2_aa", &_push2d_aa);
    lua_register(lvm, "p2_line_cap", &_push2d_line_cap);
    lua_register(lvm, "p2_line_join", &_push2d_line_join);
    lua_register(lvm, "p2_miter_limit", &_push2d_miter_limit);
    lua_register(lvm, "p2_colour", &_push2d_colour);
    lua_register(lvm, "p2_line_width", &_push2d_line_width);
    lua_register(lvm, "p2_move", &_push2d_move);
    lua_register(lvm, "p2_line", &_push2d_line);
    lua_register(lvm, "p2_move_rel", &_push2d_move_rel);
    lua_register(lvm, "p2_line_rel", &_push2d_line_rel);
    lua_register(lvm, "p2_curve", &_push2d_curve);
    lua_register(lvm, "p2_curve_rel", &_push2d_curve_rel);
    lua_register(lvm, "p2_arc", &_push2d_arc);
    lua_register(lvm, "p2_rect", &_push2d_rect);
    lua_register(lvm, "p2_stroke", &_push2d_stroke);
    lua_register(lvm, "p2_fill", &_push2d_fill);
    lua_register(lvm, "p2_text", &_push2d_text);
    lua_register(lvm, "p2_clear", &_push2d_clear);
    lua_register(lvm, "p2_close", &_push2d_close);
    lua_register(lvm, "p2_extents", &_push2d_extents);
}
