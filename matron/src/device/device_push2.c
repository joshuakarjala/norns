#include <stdio.h>

#include "../events.h"

#include "device.h"
#include "device_push2.h"


#include <stdarg.h>
#include <memory.h>

#include <cairo.h>
#include <cairo-ft.h>

#include "../hardware/screen.h"

static const uint16_t VID = 0x2982, PID = 0x1967;

#define HDR_PKT_SZ 0x10
static uint8_t headerPkt[HDR_PKT_SZ] = { 0xFF, 0xCC, 0xAA, 0x88, 0x00, 0x00, 0x00, 0x00,
                                         0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
                                       };


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

int init(void *self );
int deinit(void *self );
int render(void *self );


int dev_push2_init(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    struct dev_common *base = (struct dev_common *) self;

    base->start = &dev_push2_start;
    base->deinit = &dev_push2_deinit;

    push2->headerPkt_ = headerPkt;
    push2->handle_ = NULL;
    push2->iface_ = 0;
    push2->endpointOut_ = 1;

    memset(push2->imgBuf_, 0, PUSH2_DATA_PKT_SZ);
    push2->surface = cairo_image_surface_create_for_data(
                         (unsigned char*) push2->imgBuf_,
                         CAIRO_FORMAT_RGB16_565,
                         PUSH2_WIDTH,
                         PUSH2_HEIGHT,
                         PUSH2_LINE
                     );
    push2->cr = cairo_create (push2->surface);
    memset(push2->dataPkt_, 0, PUSH2_DATA_PKT_SZ);

    screen_cr((void*) push2->cr);
    cairo_scale(push2->cr, 2.0f, 2.0f);
    init(self);
    push2->running_ = true;
    return 0;
}

void dev_push2_deinit(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    push2->running_ = false;
    while (!push2->running_) {
        sleep(1);
    }

    push2->running_ = false;
    deinit(self);
    cairo_destroy (push2->cr);
    cairo_surface_destroy (push2->surface);
    ;
}

void* dev_push2_start(void *self) {
    perr("Push 2 render loop starting\n");
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    while (push2->running_) {
        render(self);
        usleep(1);
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




