#include <stdio.h>

#include "../events.h"

#include "device.h"
#include "device_push2.h"


#include <stdarg.h>
#include <memory.h>

#include <cairo.h>
#include <cairo-ft.h>


// lua
#include <lua.h>
#include <lualib.h>
#include <lauxlib.h>

#include "../weaver.h"

#define NUM_FONTS 14
static char font_path[NUM_FONTS][32];
static cairo_font_face_t *ct[NUM_FONTS];
static FT_Library value;
static FT_Error status;
static FT_Face face[NUM_FONTS];
static double text_xy[2];

static float c[16] =
{   0, 0.066666666666667, 0.13333333333333, 0.2, 0.26666666666667,
    0.33333333333333,
    0.4, 0.46666666666667, 0.53333333333333, 0.6, 0.66666666666667,
    0.73333333333333, 0.8, 0.86666666666667, 0.93333333333333, 1
};


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

void push2RegisterLua(void *self);

static struct dev_push2 *default_push2 = NULL;



// skip this if you don't want every screen module call to perform null checks
#ifndef CHECK_CR
#define CHECK_CR if (default_push2->cr == NULL) {return 0; }
#define CHECK_CRR if (default_push2->cr == NULL) {return 0; }
#endif



int dev_push2_init(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    struct dev_common *base = (struct dev_common *) self;

    base->start = &dev_push2_start;
    base->deinit = &dev_push2_deinit;

    push2->headerPkt_ = headerPkt;
    push2->handle_ = NULL;
    push2->iface_ = 0;
    push2->endpointOut_ = 1;

    push2->surface = cairo_image_surface_create_for_data(
                         (unsigned char*) push2->imgBuf_,
                         CAIRO_FORMAT_RGB16_565,
                         PUSH2_WIDTH,
                         PUSH2_HEIGHT,
                         PUSH2_LINE
                     );
    push2->cr = cairo_create (push2->surface);
    memset(push2->dataPkt_, 0, PUSH2_DATA_PKT_SZ);
    memset(push2->imgBuf_, 0, PUSH2_DATA_PKT_SZ);

    status = FT_Init_FreeType(&value);
    if (status != 0) {
        fprintf(stderr, "ERROR (screen) freetype init\n");
        return 0;
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
        snprintf(filename, 256, "%s/norns/resources/%s", getenv(
                     "HOME"), font_path[i]);

        status = FT_New_Face(value, filename, 0, &face[i]);
        if (status != 0) {
            fprintf(stderr, "ERROR (screen) font load: %s\n", filename);
            return 0;
        }
        else {
            ct[i] = cairo_ft_font_face_create_for_ft_face(face[i], 0);
        }
    }

    cairo_set_operator(push2->cr, CAIRO_OPERATOR_CLEAR);
    cairo_paint(push2->cr);
    cairo_set_operator(push2->cr, CAIRO_OPERATOR_OVER);

    cairo_font_options_t *font_options;
    font_options = cairo_font_options_create();
    cairo_font_options_set_antialias(font_options, CAIRO_ANTIALIAS_SUBPIXEL);

    // default font
    cairo_set_font_face (push2->cr, ct[0]);
    cairo_set_font_options(push2->cr, font_options);
    cairo_set_font_size(push2->cr, 8.0);

    cairo_scale(push2->cr, 5.0f, 5.0f);

    push2RegisterLua(self);

    return 0;
}

void dev_push2_deinit(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    push2->running_ = false;
    while (!push2->running_) {
        sleep(1);
    }
    push2->running_ = false;
    cairo_destroy (push2->cr);
    cairo_surface_destroy (push2->surface);
    ;
}

void* dev_push2_start(void *self) {
    perr("Push 2 render loop starting\n");
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    push2->running_ = true;
    init(self);
    while (push2->running_) {
        render(self);
        usleep(1);
    }
    deinit(self);
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



//------------------------------------------------------------------------------------------------------------

int push2_s_update(lua_State *l) {
    if (lua_gettop(l) != 0) {
        return luaL_error(l, "wrong number of arguments");
    }
    CHECK_CR

    cairo_paint(default_push2->cr);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: set font face
 * @function s_font_face
 */
int push2_s_font_face(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    int i = (int) luaL_checkinteger(l, 1);
    CHECK_CR
    if ( (i >= 0) && (i < NUM_FONTS) ) {
        cairo_set_font_face(default_push2->cr, ct[i]);
    }
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: set font size
 * @function s_font_size
 */
int push2_s_font_size(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    int z = (int) luaL_checkinteger(l, 1);
    CHECK_CR
    cairo_set_font_size(default_push2->cr, z);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: change antialias mode for drawing
 * @function s_aa
 * @tparam integer state, 0=off, 1=on
 */
int push2_s_aa(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    int x = (int) luaL_checkinteger(l, 1);
    CHECK_CR
    if (x == 0) {
        cairo_set_antialias(default_push2->cr, CAIRO_ANTIALIAS_NONE);
    } else {
        cairo_set_antialias(default_push2->cr, CAIRO_ANTIALIAS_DEFAULT);
    }
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: change level (color)
 * @function s_level
 * @tparam integer level, 0 (black) to 15 (white)
 */
int push2_s_level(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    int z = (int) luaL_checkinteger(l, 1);
    CHECK_CR
    cairo_set_source_rgb(default_push2->cr, c[z], c[z], c[z]);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: set line width
 * @function s_line_width
 * @tparam integer width line width
 */
int push2_s_line_width(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    double w = luaL_checknumber(l, 1);
    CHECK_CR
    cairo_set_line_width(default_push2->cr, w);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: move position
 * @function s_move
 * @param x
 * @param y
 */
int push2_s_move(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x = luaL_checknumber(l, 1);
    double y = luaL_checknumber(l, 2);
    CHECK_CR
    cairo_move_to(default_push2->cr, x + 0.5, y + 0.5);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: draw line to position
 * @function s_line
 * @param x
 * @param y
 */
int push2_s_line(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x = luaL_checkinteger(l, 1);
    double y = luaL_checkinteger(l, 2);
    CHECK_CR
    cairo_line_to(default_push2->cr, x + 0.5, y + 0.5);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: move position rel
 * @function s_move_rel
 * @param x
 * @param y
 */
int push2_s_move_rel(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x = luaL_checknumber(l, 1);
    double y = luaL_checknumber(l, 2);
    CHECK_CR
    cairo_rel_move_to(default_push2->cr, x + 0.5, y + 0.5);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: draw line to position rel
 * @function s_line_rel
 * @param x
 * @param y
 */
int push2_s_line_rel(lua_State *l) {
    if (lua_gettop(l) != 2) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x = (int) luaL_checknumber(l, 1);
    double y = (int) luaL_checknumber(l, 2);
    CHECK_CR
    cairo_rel_line_to(default_push2->cr, x + 0.5, y + 0.5);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: draw curve
 * @function s_curve
 * @param x
 * @param y
 */
int push2_s_curve(lua_State *l) {
    if (lua_gettop(l) != 6) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x1 = luaL_checknumber(l, 1);
    double y1 = luaL_checknumber(l, 2);
    double x2 = luaL_checknumber(l, 3);
    double y2 = luaL_checknumber(l, 4);
    double x3 = luaL_checknumber(l, 5);
    double y3 = luaL_checknumber(l, 6);
    CHECK_CR
    cairo_curve_to(default_push2->cr, x1, y1, x2, y2, x3, y3);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: draw curve rel
 * @function s_curve_rel
 * @param x
 * @param y
 */
int push2_s_curve_rel(lua_State *l) {
    if (lua_gettop(l) != 6) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x1 = luaL_checknumber(l, 1);
    double y1 = luaL_checknumber(l, 2);
    double x2 = luaL_checknumber(l, 3);
    double y2 = luaL_checknumber(l, 4);
    double x3 = luaL_checknumber(l, 5);
    double y3 = luaL_checknumber(l, 6);
    CHECK_CR
    cairo_rel_curve_to(default_push2->cr, x1, y1, x2, y2, x3, y3);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: draw arc
 * @function s_arc
 * @param x
 * @param y
 */
int push2_s_arc(lua_State *l) {
    if (lua_gettop(l) != 5) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x = luaL_checknumber(l, 1);
    double y = luaL_checknumber(l, 2);
    double r = luaL_checknumber(l, 3);
    double a1 = luaL_checknumber(l, 4);
    double a2 = luaL_checknumber(l, 5);
    CHECK_CR
    cairo_arc(default_push2->cr, x + 0.5, y + 0.5, r, a1, a2);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: draw rect
 * @function s_rect
 * @param x
 * @param y
 */
int push2_s_rect(lua_State *l) {
    if (lua_gettop(l) != 4) {
        return luaL_error(l, "wrong number of arguments");
    }

    double x = luaL_checknumber(l, 1);
    double y = luaL_checknumber(l, 2);
    double w = luaL_checknumber(l, 3);
    double h = luaL_checknumber(l, 4);
    CHECK_CR
    cairo_rectangle(default_push2->cr, x + 0.5, y + 0.5, w, h);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: stroke, or apply width/color to line(s)
 * @function s_stroke
 */
int push2_s_stroke(lua_State *l) {
    if (lua_gettop(l) != 0) {
        return luaL_error(l, "wrong number of arguments");
    }
    CHECK_CR
    cairo_stroke(default_push2->cr);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: fill path
 * @function s_fill
 */
int push2_s_fill(lua_State *l) {
    if (lua_gettop(l) != 0) {
        return luaL_error(l, "wrong number of arguments");
    }

    CHECK_CR
    cairo_fill(default_push2->cr);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: text
 * @function s_text
 * @tparam string text test to print
 */
int push2_s_text(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    const char *s = luaL_checkstring(l, 1);
    perr("s_text %s", s);
    CHECK_CR
    cairo_show_text(default_push2->cr, s);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: clear to black
 * @function s_clear
 */
int push2_s_clear(lua_State *l) {
    if (lua_gettop(l) != 0) {
        return luaL_error(l, "wrong number of arguments");
    }

    cairo_set_operator(default_push2->cr, CAIRO_OPERATOR_CLEAR);
    cairo_paint(default_push2->cr);
    cairo_set_operator(default_push2->cr, CAIRO_OPERATOR_OVER);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: close path
 * @function s_close
 */
int push2_s_close(lua_State *l) {
    if (lua_gettop(l) != 0) {
        return luaL_error(l, "wrong number of arguments");
    }

    CHECK_CR
    cairo_close_path(default_push2->cr);
    lua_settop(l, 0);
    return 0;
}

/***
 * sdefault_push2->creen: extents
 * @function s_extents
 * @tparam gets x/y displacement of a string
 */
int push2_s_extents(lua_State *l) {
    if (lua_gettop(l) != 1) {
        return luaL_error(l, "wrong number of arguments");
    }

    const char *s = luaL_checkstring(l, 1);
    CHECK_CRR
    cairo_text_extents_t extents;
    cairo_text_extents(default_push2->cr, s, &extents);
    text_xy[0] = extents.width;
    text_xy[1] = extents.height;
    lua_pushinteger(l, text_xy[0]);
    lua_pushinteger(l, text_xy[1]);
    return 2;
}


#undef CHECK_CR
#undef CHECK_CRR


void push2RegisterLua(void *self) {
    struct dev_push2 *push2 = (struct dev_push2 *) self;
    default_push2 = push2;
    lua_State* lvm =  (lua_State*) luaState();
    lua_register(lvm, "s_update", &push2_s_update);
    lua_register(lvm, "s_font_face", &push2_s_font_face);
    lua_register(lvm, "s_font_size", &push2_s_font_size);
    lua_register(lvm, "s_aa", &push2_s_aa);
    lua_register(lvm, "s_level", &push2_s_level);
    lua_register(lvm, "s_line_width", &push2_s_line_width);
    lua_register(lvm, "s_move", &push2_s_move);
    lua_register(lvm, "s_line", &push2_s_line);
    lua_register(lvm, "s_move_rel", &push2_s_move_rel);
    lua_register(lvm, "s_line_rel", &push2_s_line_rel);
    lua_register(lvm, "s_curve", &push2_s_curve);
    lua_register(lvm, "s_curve_rel", &push2_s_curve_rel);
    lua_register(lvm, "s_arc", &push2_s_arc);
    lua_register(lvm, "s_rect", &push2_s_rect);
    lua_register(lvm, "s_stroke", &push2_s_stroke);
    lua_register(lvm, "s_fill", &push2_s_fill);
    lua_register(lvm, "s_text", &push2_s_text);
    lua_register(lvm, "s_clear", &push2_s_clear);
    lua_register(lvm, "s_close", &push2_s_close);
    lua_register(lvm, "s_extents", &push2_s_extents);

}

