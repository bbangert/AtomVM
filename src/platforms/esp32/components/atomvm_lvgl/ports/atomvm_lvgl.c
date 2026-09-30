/*
 * SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
 *
 * LVGL display port driver.
 *
 * Erlang owns a flat set of LVGL objects, addressed by small integer ids that
 * are also their z-order (0 at the bottom), and changes them in batches:
 *
 *   {batch, [Op]} -> ok | busy        applied atomically, in order
 *     {new, Id, box | label | marquee | image | flipbook}  created at z-index Id; a
 *                                      marquee is a label that scrolls round when wider than
 *                                      w, a flipbook an image that cycles through `frames`
 *     {del, Id}
 *     {set, Id, [{Prop, Value}]}      x y w h bg fg text font src sx sy ox oy hidden recolor speed
 *                                      fx fx_dir fx_steps fx_ms fx_row fx_left fx_noise:
 *                                      a label plays a text effect on its own text, see below
 *                                      mx my mdir mdelay mms mease: the object glides in from
 *                                      (x + mx, y + my), or out to there and hides, see below
 *                                      glide: once placed, a new x or y is animated to over
 *                                      that many ms rather than jumped to
 *                                      frames (binary of 16-bit little-endian image ids),
 *                                      frame_ms: a flipbook's pictures and how long each shows
 *     {img, ImgId, rgba8888 | a8, W, H, Bin[, Scale]}
 *                                      Scale enlarges it once, nearest neighbour; a fully
 *                                      opaque RGBA image is kept as RGB565, drawn unblended
 *     {unimg, ImgId}
 *     {font, FontId, uf | raw8x16, Bin}  loaded once; a second load of an id is ignored
 *     {reset}                          deletes every object and image, keeps fonts
 *     {decor, [Spec]}                  replaces the decorations drawn over everything:
 *       {border, Thickness, PixelsPerSecond, [RGB]}  a frame whose colours travel round
 *       {beam, RGB, PeriodMs, Height, Opa}           a line that sweeps down now and then
 *       {glitch, [RGB], MinMs, MaxMs}                 bars that flash at random
 *       {line, X, Y, W, H, [RGB], pulse | flow, PeriodMs, MinPercent}
 *                                     a gradient line that breathes, or slides along
 *   stats -> {InternalFree, DmaFree, InternalLargest, PsramFree, Refreshes}
 *
 * The mailbox handler runs on a VM scheduler thread and never calls LVGL: it
 * parses a batch into PSRAM and queues it. A timer inside esp_lvgl_port's
 * task drains the queue, so LVGL is only ever called from that task. A full
 * queue answers `busy` and applies nothing, so the caller can diff the next
 * frame against what the panel really shows.
 */

#include <sdkconfig.h>

#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_heap_trace.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>
#include <esp_memory_utils.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <lvgl.h>

/* Public in LVGL 9.6, but only declared in a private header. */
void lv_image_cache_drop(const void *src);

#include <atom.h>
#include <context.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <memory.h>
#include <port.h>
#include <portnifloader.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#define TAG "atomvm_lvgl"

/* Not exported by libAtomVM; the in-tree drivers each define it. */
#define PORT_REPLY_SIZE (TUPLE_SIZE(2) + REF_SIZE)

#define LCD_HOST SPI2_HOST
#define DRAW_ROWS 24
#define QUEUE_DEPTH 8
#define MAX_OBJS 512
#define MAX_IMAGES 256
#define MAX_FONTS 8
#define PSRAM_CAPS (MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT)

static const char *const sclk_atom = ATOM_STR("\x4", "sclk");
static const char *const mosi_atom = ATOM_STR("\x4", "mosi");
static const char *const cs_atom = ATOM_STR("\x2", "cs");
static const char *const dc_atom = ATOM_STR("\x2", "dc");
static const char *const reset_atom = ATOM_STR("\x5", "reset");
static const char *const width_atom = ATOM_STR("\x5", "width");
static const char *const height_atom = ATOM_STR("\x6", "height");
static const char *const clock_hz_atom = ATOM_STR("\x8", "clock_hz");
static const char *const swap_xy_atom = ATOM_STR("\x7", "swap_xy");
static const char *const mirror_x_atom = ATOM_STR("\x8", "mirror_x");
static const char *const mirror_y_atom = ATOM_STR("\x8", "mirror_y");
static const char *const invert_atom = ATOM_STR("\x6", "invert");
static const char *const bgr_atom = ATOM_STR("\x3", "bgr");
static const char *const no_display_atom = ATOM_STR("\xA", "no_display");
static const char *const busy_atom = ATOM_STR("\x4", "busy");

enum request
{
    ReqInvalid = 0,
    ReqBatch,
    ReqStats,
    ReqTraceStart,
    ReqTraceDump
};

static const AtomStringIntPair request_table[] = {
    { ATOM_STR("\x5", "batch"), ReqBatch },
    { ATOM_STR("\x5", "stats"), ReqStats },
    { ATOM_STR("\xB", "trace_start"), ReqTraceStart },
    { ATOM_STR("\xA", "trace_dump"), ReqTraceDump },
    SELECT_INT_DEFAULT(ReqInvalid)
};

enum op_kind
{
    OpInvalid = 0,
    OpNew,
    OpDel,
    OpSet,
    OpImg,
    OpUnimg,
    OpFont,
    OpReset,
    OpDecor
};

static const AtomStringIntPair op_table[] = {
    { ATOM_STR("\x3", "new"), OpNew },
    { ATOM_STR("\x3", "del"), OpDel },
    { ATOM_STR("\x3", "set"), OpSet },
    { ATOM_STR("\x3", "img"), OpImg },
    { ATOM_STR("\x5", "unimg"), OpUnimg },
    { ATOM_STR("\x4", "font"), OpFont },
    { ATOM_STR("\x5", "reset"), OpReset },
    { ATOM_STR("\x5", "decor"), OpDecor },
    SELECT_INT_DEFAULT(OpInvalid)
};

enum obj_type
{
    TypeNone = 0,
    TypeBox,
    TypeLabel,
    TypeImage,
    TypeMarquee,
    TypeFlipbook
};

static const AtomStringIntPair type_table[] = {
    { ATOM_STR("\x3", "box"), TypeBox },
    { ATOM_STR("\x5", "label"), TypeLabel },
    { ATOM_STR("\x5", "image"), TypeImage },
    { ATOM_STR("\x7", "marquee"), TypeMarquee },
    { ATOM_STR("\x8", "flipbook"), TypeFlipbook },
    SELECT_INT_DEFAULT(TypeNone)
};

enum prop
{
    PropInvalid = 0,
    PropX,
    PropY,
    PropW,
    PropH,
    PropBg,
    PropFg,
    PropText,
    PropFont,
    PropSrc,
    PropScaleX,
    PropScaleY,
    PropOffsetX,
    PropOffsetY,
    PropHidden,
    PropRecolor,
    PropSpeed,
    PropFx,
    PropFxDir,
    PropFxSteps,
    PropFxMs,
    PropFxRow,
    PropFxLeft,
    PropFxNoise,
    PropMoveX,
    PropMoveY,
    PropMoveDir,
    PropMoveDelay,
    PropMoveMs,
    PropMoveEase,
    PropGlide,
    PropFrames,
    PropFrameMs
};

static const AtomStringIntPair prop_table[] = {
    { ATOM_STR("\x1", "x"), PropX },
    { ATOM_STR("\x1", "y"), PropY },
    { ATOM_STR("\x1", "w"), PropW },
    { ATOM_STR("\x1", "h"), PropH },
    { ATOM_STR("\x2", "bg"), PropBg },
    { ATOM_STR("\x2", "fg"), PropFg },
    { ATOM_STR("\x4", "text"), PropText },
    { ATOM_STR("\x4", "font"), PropFont },
    { ATOM_STR("\x3", "src"), PropSrc },
    { ATOM_STR("\x2", "sx"), PropScaleX },
    { ATOM_STR("\x2", "sy"), PropScaleY },
    { ATOM_STR("\x2", "ox"), PropOffsetX },
    { ATOM_STR("\x2", "oy"), PropOffsetY },
    { ATOM_STR("\x6", "hidden"), PropHidden },
    { ATOM_STR("\x7", "recolor"), PropRecolor },
    { ATOM_STR("\x5", "speed"), PropSpeed },
    { ATOM_STR("\x2", "fx"), PropFx },
    { ATOM_STR("\x6", "fx_dir"), PropFxDir },
    { ATOM_STR("\x8", "fx_steps"), PropFxSteps },
    { ATOM_STR("\x5", "fx_ms"), PropFxMs },
    { ATOM_STR("\x6", "fx_row"), PropFxRow },
    { ATOM_STR("\x7", "fx_left"), PropFxLeft },
    { ATOM_STR("\x8", "fx_noise"), PropFxNoise },
    { ATOM_STR("\x2", "mx"), PropMoveX },
    { ATOM_STR("\x2", "my"), PropMoveY },
    { ATOM_STR("\x4", "mdir"), PropMoveDir },
    { ATOM_STR("\x6", "mdelay"), PropMoveDelay },
    { ATOM_STR("\x3", "mms"), PropMoveMs },
    { ATOM_STR("\x5", "mease"), PropMoveEase },
    { ATOM_STR("\x5", "glide"), PropGlide },
    { ATOM_STR("\x6", "frames"), PropFrames },
    { ATOM_STR("\x8", "frame_ms"), PropFrameMs },
    SELECT_INT_DEFAULT(PropInvalid)
};

enum data_format
{
    FmtInvalid = 0,
    FmtRgba8888,
    FmtA8,
    FmtUf,
    FmtRaw8x16
};

static const AtomStringIntPair format_table[] = {
    { ATOM_STR("\x8", "rgba8888"), FmtRgba8888 },
    { ATOM_STR("\x2", "a8"), FmtA8 },
    { ATOM_STR("\x2", "uf"), FmtUf },
    { ATOM_STR("\x7", "raw8x16"), FmtRaw8x16 },
    SELECT_INT_DEFAULT(FmtInvalid)
};

/* One property; text is a PSRAM copy owned by the op until applied. */
struct prop_value
{
    uint8_t key;
    int32_t value;
    char *text;
};

/* One operation; everything it points at lives in PSRAM and is freed by the LVGL task. */
struct op
{
    uint8_t kind;
    uint8_t type;
    uint8_t format;
    uint16_t id;
    uint16_t w;
    uint16_t h;
    uint8_t scale;
    uint16_t prop_count;
    struct prop_value *props;
    uint8_t *data;
    size_t size;
    struct decor *decor;
};

struct batch
{
    size_t count;
    struct op *ops;
};

/* A font read straight from a .uf file or a raw 8x16 bitmap, kept in PSRAM. */
struct font_data
{
    lv_font_t font;
    uint8_t format;
    const uint8_t *data;
    size_t size;
    uint32_t glyphs;
    uint32_t intervals;
    uint32_t interval_count;
    uint32_t bitmap;
};

static QueueHandle_t queue;
static lv_display_t *display;
static lv_obj_t *objs[MAX_OBJS];
static uint8_t obj_types[MAX_OBJS];
static lv_image_dsc_t *images[MAX_IMAGES];
static struct font_data *fonts[MAX_FONTS];
static volatile uint32_t refreshes;
static volatile uint32_t failed_flushes;
static volatile uint32_t dropped_ops;
static esp_lcd_panel_handle_t panel_handle;
static bool started;

static void *psram_malloc(size_t size)
{
    void *p = heap_caps_malloc(size, PSRAM_CAPS);
    return p != NULL ? p : heap_caps_malloc(size, MALLOC_CAP_DEFAULT);
}

static int get_int_default(term kv, AtomString key, int default_value, GlobalContext *global)
{
    term value = interop_kv_get_value(kv, key, global);
    return term_is_integer(value) ? term_to_int(value) : default_value;
}

static bool get_bool_default(term kv, AtomString key, bool default_value, GlobalContext *global)
{
    term value = interop_kv_get_value(kv, key, global);
    if (value == TRUE_ATOM) {
        return true;
    }
    if (value == FALSE_ATOM) {
        return false;
    }
    return default_value;
}

/* ---------------------------------------------------------------------------
 * Fonts
 * ------------------------------------------------------------------------- */

static uint16_t le16(const uint8_t *p)
{
    return (uint16_t) (p[0] | (p[1] << 8));
}

static uint32_t le32(const uint8_t *p)
{
    return (uint32_t) (p[0] | (p[1] << 8) | (p[2] << 16) | ((uint32_t) p[3] << 24));
}

static uint32_t be32(const uint8_t *p)
{
    return ((uint32_t) p[0] << 24) | (p[1] << 16) | (p[2] << 8) | p[3];
}

/* The 18-byte glyph record for a codepoint, or NULL when the font lacks it. */
static const uint8_t *uf_glyph(const struct font_data *f, uint32_t letter, uint32_t *index_out)
{
    for (uint32_t i = 0; i < f->interval_count; i++) {
        const uint8_t *iv = f->data + f->intervals + i * 12;
        uint32_t first = le32(iv);
        uint32_t last = le32(iv + 4);
        if (letter >= first && letter <= last) {
            uint32_t index = le32(iv + 8) + (letter - first);
            if (f->glyphs + (index + 1) * 18 > f->size) {
                return NULL;
            }
            *index_out = index;
            return f->data + f->glyphs + index * 18;
        }
    }
    return NULL;
}

static bool uf_glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc, uint32_t letter, uint32_t next)
{
    UNUSED(next);
    const struct font_data *f = font->user_data;
    uint32_t index;
    const uint8_t *g = uf_glyph(f, letter, &index);
    if (g == NULL) {
        return false;
    }
    uint16_t width = le16(g);
    uint16_t height = le16(g + 2);
    dsc->adv_w = le16(g + 4);
    dsc->box_w = width;
    dsc->box_h = height;
    dsc->ofs_x = (int16_t) le16(g + 6);
    dsc->ofs_y = (int16_t) le16(g + 8) - (int16_t) height;
    dsc->stride = 0;
    dsc->format = LV_FONT_GLYPH_FORMAT_A4;
    dsc->is_placeholder = false;
    dsc->gid.index = index + 1;
    return true;
}

/* Decodes 4-bit levels, two a byte with the low nibble first, into A8. */
static const void *uf_glyph_bitmap(lv_font_glyph_dsc_t *dsc, lv_draw_buf_t *draw_buf)
{
    const struct font_data *f = dsc->resolved_font->user_data;
    uint32_t index = dsc->gid.index - 1;
    const uint8_t *g = f->data + f->glyphs + index * 18;
    uint16_t width = le16(g);
    uint16_t height = le16(g + 2);
    uint32_t offset = le32(g + 14);
    uint32_t row_bytes = (width + 1) / 2;
    if (width == 0 || height == 0 || f->bitmap + offset + row_bytes * height > f->size) {
        return NULL;
    }

    const uint8_t *in = f->data + f->bitmap + offset;
    uint8_t *out = draw_buf->data;
    uint32_t stride = lv_draw_buf_width_to_stride(width, LV_COLOR_FORMAT_A8);
    for (uint32_t y = 0; y < height; y++) {
        for (uint32_t x = 0; x < width; x++) {
            uint8_t byte = in[y * row_bytes + x / 2];
            uint8_t level = (x & 1) ? (byte >> 4) : (byte & 0xF);
            out[y * stride + x] = level * 17;
        }
    }
    return draw_buf;
}

static bool raw_glyph_dsc(const lv_font_t *font, lv_font_glyph_dsc_t *dsc, uint32_t letter, uint32_t next)
{
    UNUSED(font);
    UNUSED(next);
    if (letter > 255) {
        return false;
    }
    dsc->adv_w = 8;
    dsc->box_w = 8;
    dsc->box_h = 16;
    dsc->ofs_x = 0;
    dsc->ofs_y = 0;
    dsc->stride = 0;
    dsc->format = LV_FONT_GLYPH_FORMAT_A1;
    dsc->is_placeholder = false;
    dsc->gid.index = letter + 1;
    return true;
}

/* One byte a row, most significant bit leftmost, into A8. */
static const void *raw_glyph_bitmap(lv_font_glyph_dsc_t *dsc, lv_draw_buf_t *draw_buf)
{
    const struct font_data *f = dsc->resolved_font->user_data;
    const uint8_t *rows = f->data + (dsc->gid.index - 1) * 16;
    uint8_t *out = draw_buf->data;
    uint32_t stride = lv_draw_buf_width_to_stride(8, LV_COLOR_FORMAT_A8);
    for (int y = 0; y < 16; y++) {
        for (int x = 0; x < 8; x++) {
            out[y * stride + x] = (rows[y] & (0x80 >> x)) ? 0xFF : 0x00;
        }
    }
    return draw_buf;
}

/* LVGL task only. Takes ownership of `data`. */
static void load_font(int id, int format, uint8_t *data, size_t size)
{
    if (id < 0 || id >= MAX_FONTS || fonts[id] != NULL) {
        heap_caps_free(data);
        return;
    }
    struct font_data *f = heap_caps_calloc(1, sizeof(struct font_data), PSRAM_CAPS);
    f->format = format;
    f->data = data;
    f->size = size;
    f->font.user_data = f;
    f->font.static_bitmap = 0;

    if (format == FmtRaw8x16 && size >= 256 * 16) {
        f->font.get_glyph_dsc = raw_glyph_dsc;
        f->font.get_glyph_bitmap = raw_glyph_bitmap;
        f->font.line_height = 16;
        f->font.base_line = 0;
        fonts[id] = f;
        return;
    }

    /* IFF: "FORM" size "uFL0", then records of a 4-byte tag, a big-endian size, 4-byte aligned. */
    uint32_t header = 0;
    for (size_t pos = 12; pos + 8 <= size;) {
        uint32_t len = be32(data + pos + 4);
        uint32_t body = pos + 8;
        if (memcmp(data + pos, "uFH0", 4) == 0) {
            header = body;
        } else if (memcmp(data + pos, "uFI0", 4) == 0) {
            f->intervals = body;
        } else if (memcmp(data + pos, "uFP0", 4) == 0) {
            f->glyphs = body;
        } else if (memcmp(data + pos, "uFB0", 4) == 0) {
            f->bitmap = body;
        }
        pos = body + len;
        pos += (4 - pos % 4) % 4;
    }

    if (format != FmtUf || header == 0 || f->glyphs == 0 || f->intervals == 0 || f->bitmap == 0
        || data[header + 4] != 0) {
        ESP_LOGE(TAG, "font %d: not an uncompressed .uf", id);
        heap_caps_free(data);
        heap_caps_free(f);
        return;
    }

    uint16_t advance_y = le16(data + header + 5);
    uint16_t ascender = le16(data + header + 7);
    f->interval_count = le32(data + header);
    f->font.get_glyph_dsc = uf_glyph_dsc;
    f->font.get_glyph_bitmap = uf_glyph_bitmap;
    f->font.line_height = advance_y;
    f->font.base_line = advance_y - ascender;
    fonts[id] = f;
}

/* ---------------------------------------------------------------------------
 * Images
 * ------------------------------------------------------------------------- */

/*
 * LVGL task only. Takes ownership of `data`. Enlarged `scale` times once here
 * rather than on every frame; RGBA with no transparency becomes RGB565, which
 * LVGL copies straight to the panel, and the rest LVGL's BGRA.
 */
static void load_image(int id, int format, int w, int h, int scale, uint8_t *data, size_t size)
{
    size_t in_bpp = format == FmtA8 ? 1 : 4;
    if (id < 0 || id >= MAX_IMAGES || images[id] != NULL || size < (size_t) w * h * in_bpp) {
        heap_caps_free(data);
        return;
    }
    if (scale < 1) {
        scale = 1;
    }

    bool opaque = format == FmtRgba8888;
    for (size_t i = 3; opaque && i < (size_t) w * h * 4; i += 4) {
        opaque = data[i] == 0xFF;
    }

    int out_w = w * scale;
    int out_h = h * scale;
    size_t out_bpp = format == FmtA8 ? 1 : (opaque ? 2 : 4);
    uint8_t *out = heap_caps_malloc((size_t) out_w * out_h * out_bpp, PSRAM_CAPS);
    if (out == NULL) {
        heap_caps_free(data);
        return;
    }

    for (int y = 0; y < out_h; y++) {
        const uint8_t *row = data + (size_t) (y / scale) * w * in_bpp;
        uint8_t *dst = out + (size_t) y * out_w * out_bpp;
        for (int x = 0; x < out_w; x++) {
            const uint8_t *px = row + (size_t) (x / scale) * in_bpp;
            if (format == FmtA8) {
                dst[x] = px[0];
            } else if (opaque) {
                uint16_t rgb565 = ((px[0] & 0xF8) << 8) | ((px[1] & 0xFC) << 3) | (px[2] >> 3);
                dst[x * 2] = rgb565 & 0xFF;
                dst[x * 2 + 1] = rgb565 >> 8;
            } else {
                dst[x * 4] = px[2];
                dst[x * 4 + 1] = px[1];
                dst[x * 4 + 2] = px[0];
                dst[x * 4 + 3] = px[3];
            }
        }
    }
    heap_caps_free(data);

    lv_image_dsc_t *dsc = heap_caps_calloc(1, sizeof(lv_image_dsc_t), PSRAM_CAPS);
    dsc->header.magic = LV_IMAGE_HEADER_MAGIC;
    dsc->header.cf = format == FmtA8 ? LV_COLOR_FORMAT_A8
        : (opaque ? LV_COLOR_FORMAT_RGB565 : LV_COLOR_FORMAT_ARGB8888);
    dsc->header.w = out_w;
    dsc->header.h = out_h;
    dsc->header.stride = out_w * out_bpp;
    dsc->data_size = (uint32_t) out_w * out_h * out_bpp;
    dsc->data = out;
    images[id] = dsc;
}

static void free_image(int id)
{
    if (id < 0 || id >= MAX_IMAGES || images[id] == NULL) {
        return;
    }
    lv_image_cache_drop(images[id]);
    heap_caps_free((void *) images[id]->data);
    heap_caps_free(images[id]);
    images[id] = NULL;
}

/* ---------------------------------------------------------------------------
 * Objects, LVGL task only
 * ------------------------------------------------------------------------- */

static void plain(lv_obj_t *obj)
{
    lv_obj_remove_style_all(obj);
    lv_obj_set_scrollable(obj, false);
    lv_obj_set_clickable(obj, false);
}

/* ---------------------------------------------------------------------------
 * Text effects, LVGL task only
 *
 * A label can play an effect on its own text, a frame every fx_ms, the same
 * four the Elixir side drew a frame at a time: decrypt, rain, wipe, slide.
 * `in` brings the text in and leaves it whole; `out` runs the effect
 * backwards and leaves its first frame. Cells are codepoints; rows split on
 * newlines, and fx_row numbers the first row so rain can fall across lines.
 * ------------------------------------------------------------------------- */

enum fx_kind
{
    FxNone = 0,
    FxDecrypt,
    FxRain,
    FxWipe,
    FxSlide
};

#define FX_MAX_ROWS 8

struct fx_params
{
    uint8_t kind;
    uint8_t out;
    uint8_t left;
    uint8_t block;
    uint16_t steps;
    uint16_t ms;
    int16_t row;
    bool touched;
};

struct fx
{
    struct fx_params p;
    uint16_t frame;
    uint16_t len;
    uint32_t *target;
    char *text;
    lv_timer_t *timer;
    int id;
};

static struct fx *fxs[MAX_OBJS];
static struct fx_params fx_params[MAX_OBJS];

/* Scrambled cells: shades, blocks, box lines and a few symbols in code page
 * 437 for the built-in font, plain ASCII for the others. */
static const uint32_t fx_block_noise[] = { 0xB0, 0xB1, 0xB2, 0xDB, 0xDC, 0xDF, 0xB3, 0xC4, 0xC5,
    0xCE, 0xBA, 0xCD, 0xF9, 0xFE, '0', '1', '/', '\\', '<', '>', '*', '+', '#', '%' };
static const uint32_t fx_ascii_noise[] = { '0', '1', '/', '\\', '<', '>', '*', '+', '#', '%', '=', '?', '$', '&' };
static const uint32_t fx_block_edge[] = { 0xB2, 0xB1, 0xB0 };
static const uint32_t fx_ascii_edge[] = { '#', '=', '-' };

/* The same hash as Badge.Marquee, so a scramble looks as it did. */
static int32_t fx_hash(int32_t r, int32_t c, int32_t salt)
{
    int32_t x = r * 13 + c * 29 + salt * 7 + 11;
    return (x * x + 3 * x) % 1009;
}

static uint32_t fx_noise(const struct fx *f, int32_t r, int32_t c, int32_t k)
{
    int32_t h = fx_hash(r, c, k);
    return f->p.block ? fx_block_noise[h % (int32_t) (sizeof(fx_block_noise) / 4)]
                      : fx_ascii_noise[h % (int32_t) (sizeof(fx_ascii_noise) / 4)];
}

static uint32_t fx_cell(const struct fx *f, const uint32_t *row, int32_t width, int32_t r, int32_t c, int32_t k)
{
    uint32_t t = row[c];
    int32_t steps = f->p.steps;

    switch (f->p.kind) {
        case FxDecrypt:
            if (t == ' ') {
                return ' ';
            }
            return k > fx_hash(r, c, 0) % 9 ? t : fx_noise(f, r, c, k);

        case FxRain: {
            int32_t head = k - fx_hash(0, c, 1) % 8;
            if (head > r) {
                return t;
            }
            return head == r ? fx_noise(f, r, c, k) : ' ';
        }

        case FxWipe: {
            int32_t edge = (4 * k * width) / 40 - 2;
            if (c < edge) {
                return t;
            }
            if (c <= edge + 2) {
                return f->p.block ? fx_block_edge[c - edge] : fx_ascii_edge[c - edge];
            }
            return ' ';
        }

        case FxSlide: {
            int32_t offset = (5 * (steps - k) * width) / 40;
            if (offset >= width) {
                return ' ';
            }
            if (f->p.left) {
                return c < offset ? ' ' : row[c - offset];
            }
            return c + offset < width ? row[c + offset] : ' ';
        }

        default:
            return t;
    }
}

static char *utf8_put(char *out, uint32_t cp)
{
    if (cp < 0x80) {
        *out++ = (char) cp;
    } else if (cp < 0x800) {
        *out++ = (char) (0xC0 | (cp >> 6));
        *out++ = (char) (0x80 | (cp & 0x3F));
    } else {
        *out++ = (char) (0xE0 | (cp >> 12));
        *out++ = (char) (0x80 | ((cp >> 6) & 0x3F));
        *out++ = (char) (0x80 | (cp & 0x3F));
    }
    return out;
}

/* Draws effect frame k: `in` counts up to the whole text, `out` counts back down. */
static void fx_draw(struct fx *f, int32_t k)
{
    char *out = f->text;
    int32_t r = 0;
    uint16_t start = 0;

    while (start <= f->len) {
        uint16_t end = start;
        while (end < f->len && f->target[end] != '\n') {
            end++;
        }
        int32_t width = end - start;
        for (int32_t c = 0; c < width; c++) {
            out = utf8_put(out, fx_cell(f, f->target + start, width, f->p.row + r, c, k));
        }
        if (end < f->len) {
            *out++ = '\n';
        }
        start = end + 1;
        r++;
    }
    *out = '\0';
    lv_label_set_text(objs[f->id], f->text);
}

static void fx_stop(int id)
{
    struct fx *f = fxs[id];
    if (f == NULL) {
        return;
    }
    lv_timer_delete(f->timer);
    heap_caps_free(f->target);
    heap_caps_free(f->text);
    heap_caps_free(f);
    fxs[id] = NULL;
}

static void fx_tick(lv_timer_t *timer)
{
    struct fx *f = lv_timer_get_user_data(timer);
    f->frame++;

    if (f->frame >= f->p.steps) {
        /* In leaves the text whole; out leaves the effect's first frame. */
        if (f->p.out) {
            fx_draw(f, 0);
        } else {
            char *out = f->text;
            for (uint16_t i = 0; i < f->len; i++) {
                out = utf8_put(out, f->target[i]);
            }
            *out = '\0';
            lv_label_set_text(objs[f->id], f->text);
        }
        fx_stop(f->id);
        return;
    }
    fx_draw(f, f->p.out ? f->p.steps - 1 - f->frame : f->frame);
}

/* Takes the label's current text as the effect's target and starts it. */
static void fx_start(int id)
{
    fx_stop(id);
    struct fx_params p = fx_params[id];
    if (p.kind == FxNone || p.steps == 0 || objs[id] == NULL) {
        return;
    }

    const char *text = lv_label_get_text(objs[id]);
    size_t bytes = strlen(text);
    struct fx *f = heap_caps_calloc(1, sizeof(struct fx), PSRAM_CAPS);
    f->target = heap_caps_calloc(bytes + 1, sizeof(uint32_t), PSRAM_CAPS);
    f->text = heap_caps_calloc(bytes * 3 + 4, 1, PSRAM_CAPS);

    /* UTF-8 to codepoints; anything malformed is taken a byte at a time. */
    const uint8_t *in = (const uint8_t *) text;
    uint16_t n = 0;
    for (size_t i = 0; i < bytes;) {
        uint32_t cp = in[i];
        if ((cp & 0xE0) == 0xC0 && i + 1 < bytes) {
            cp = ((cp & 0x1F) << 6) | (in[i + 1] & 0x3F);
            i += 2;
        } else if ((cp & 0xF0) == 0xE0 && i + 2 < bytes) {
            cp = ((cp & 0x0F) << 12) | ((in[i + 1] & 0x3F) << 6) | (in[i + 2] & 0x3F);
            i += 3;
        } else {
            i++;
        }
        f->target[n++] = cp;
    }

    f->p = p;
    f->len = n;
    f->id = id;
    f->timer = lv_timer_create(fx_tick, p.ms > 0 ? p.ms : 200, f);
    fxs[id] = f;
    fx_draw(f, p.out ? p.steps - 1 : 0);
}

static void set_fx_param(int id, int key, int32_t v)
{
    struct fx_params *p = &fx_params[id];
    switch (key) {
        case PropFx:
            p->kind = v;
            break;
        case PropFxDir:
            p->out = v != 0;
            break;
        case PropFxSteps:
            p->steps = v;
            break;
        case PropFxMs:
            p->ms = v;
            break;
        case PropFxRow:
            p->row = v;
            break;
        case PropFxLeft:
            p->left = v != 0;
            break;
        case PropFxNoise:
            p->block = v != 0;
            break;
        default:
            return;
    }
    p->touched = true;
}

/* ---------------------------------------------------------------------------
 * Motion, LVGL task only
 *
 * An object can glide in from an offset to where it was placed, appearing
 * when its delay runs out, or glide out to an offset after its delay and
 * hide. LVGL's own animations do the moving, at its full frame rate.
 * ------------------------------------------------------------------------- */

struct motion_params
{
    int16_t dx;
    int16_t dy;
    uint8_t leave;
    uint8_t ease;
    uint16_t delay;
    uint16_t ms;
    bool touched;
};

static struct motion_params motions[MAX_OBJS];

static void move_x(void *obj, int32_t v)
{
    lv_obj_set_x(obj, v);
}

static void move_y(void *obj, int32_t v)
{
    lv_obj_set_y(obj, v);
}

static void motion_show(lv_anim_t *a)
{
    lv_obj_set_hidden(a->var, false);
}

static void motion_hide(lv_anim_t *a)
{
    lv_obj_set_hidden(a->var, true);
}

static lv_anim_path_cb_t motion_path(int ease)
{
    switch (ease) {
        case 1:
            return lv_anim_path_ease_out;
        case 2:
            return lv_anim_path_overshoot;
        case 3:
            return lv_anim_path_bounce;
        default:
            return lv_anim_path_linear;
    }
}

/* One animation per axis that moves; the first carries the show or hide. */
static void motion_axis(lv_obj_t *obj, const struct motion_params *m, lv_anim_exec_xcb_t exec,
    int32_t at, int32_t offset, bool first)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, exec);
    lv_anim_set_duration(&a, m->ms > 0 ? m->ms : 200);
    lv_anim_set_delay(&a, m->delay);
    lv_anim_set_path_cb(&a, motion_path(m->ease));
    if (m->leave) {
        lv_anim_set_values(&a, at, at + offset);
        if (first) {
            lv_anim_set_completed_cb(&a, motion_hide);
        }
    } else {
        lv_anim_set_values(&a, at + offset, at);
        lv_anim_set_early_apply(&a, true);
        if (first) {
            lv_anim_set_start_cb(&a, motion_show);
        }
    }
    lv_anim_start(&a);
}

static void motion_start(int id)
{
    lv_obj_t *obj = objs[id];
    const struct motion_params *m = &motions[id];

    lv_anim_delete(obj, move_x);
    lv_anim_delete(obj, move_y);

    int32_t x = lv_obj_get_x_aligned(obj);
    int32_t y = lv_obj_get_y_aligned(obj);

    /* Arriving objects wait out of sight; a leaving one stays until it has gone. */
    if (!m->leave) {
        lv_obj_set_hidden(obj, true);
    }
    bool first = true;
    if (m->dx != 0 || m->dy == 0) {
        motion_axis(obj, m, move_x, x, m->dx, first);
        first = false;
    }
    if (m->dy != 0) {
        motion_axis(obj, m, move_y, y, m->dy, first);
    }
}

static void set_motion_param(int id, int key, int32_t v)
{
    struct motion_params *m = &motions[id];
    switch (key) {
        case PropMoveX:
            m->dx = v;
            break;
        case PropMoveY:
            m->dy = v;
            break;
        case PropMoveDir:
            m->leave = v != 0;
            break;
        case PropMoveDelay:
            m->delay = v;
            break;
        case PropMoveMs:
            m->ms = v;
            break;
        case PropMoveEase:
            m->ease = v;
            break;
        default:
            return;
    }
    m->touched = true;
}

/* ---------------------------------------------------------------------------
 * Glide and flipbooks, LVGL task only
 * ------------------------------------------------------------------------- */

static uint16_t glides[MAX_OBJS];

/* A placed object with a glide time animates to a new x or y; a new one is just put there. */
static void glide_to(int id, lv_anim_exec_xcb_t exec, int32_t from, int32_t to)
{
    lv_anim_delete(objs[id], exec);
    if (glides[id] == 0 || from == to) {
        exec(objs[id], to);
        return;
    }
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, objs[id]);
    lv_anim_set_exec_cb(&a, exec);
    lv_anim_set_values(&a, from, to);
    lv_anim_set_duration(&a, glides[id]);
    lv_anim_set_path_cb(&a, lv_anim_path_ease_out);
    lv_anim_start(&a);
}

#define FLIP_MAX_FRAMES 32

struct flipbook
{
    uint16_t ids[FLIP_MAX_FRAMES];
    uint8_t count;
    uint8_t index;
    uint16_t ms;
    bool touched;
    lv_timer_t *timer;
    int id;
};

static struct flipbook *flips[MAX_OBJS];

static void flip_show(struct flipbook *f)
{
    uint16_t img = f->ids[f->index];
    if (img < MAX_IMAGES && images[img] != NULL) {
        lv_image_set_src(objs[f->id], images[img]);
    }
}

static void flip_tick(lv_timer_t *timer)
{
    struct flipbook *f = lv_timer_get_user_data(timer);
    f->index = (f->index + 1) % f->count;
    flip_show(f);
}

static void flip_stop(int id)
{
    struct flipbook *f = flips[id];
    if (f == NULL) {
        return;
    }
    if (f->timer != NULL) {
        lv_timer_delete(f->timer);
    }
    heap_caps_free(f);
    flips[id] = NULL;
}

static struct flipbook *flip_for(int id)
{
    if (flips[id] == NULL) {
        flips[id] = heap_caps_calloc(1, sizeof(struct flipbook), PSRAM_CAPS);
        flips[id]->id = id;
    }
    return flips[id];
}

/* Restarts from the first frame whenever its frames or pace change. */
static void flip_start(int id)
{
    struct flipbook *f = flips[id];
    f->touched = false;
    if (f->timer != NULL) {
        lv_timer_delete(f->timer);
        f->timer = NULL;
    }
    f->index = 0;
    if (f->count == 0) {
        return;
    }
    flip_show(f);
    if (f->count > 1) {
        f->timer = lv_timer_create(flip_tick, f->ms > 0 ? f->ms : 200, f);
    }
}

static void delete_obj(int id)
{
    if (id >= 0 && id < MAX_OBJS) {
        memset(&motions[id], 0, sizeof(struct motion_params));
        glides[id] = 0;
        flip_stop(id);
    }
    fx_stop(id);
    if (id >= 0 && id < MAX_OBJS) {
        memset(&fx_params[id], 0, sizeof(struct fx_params));
    }
    if (id < 0 || id >= MAX_OBJS || objs[id] == NULL) {
        return;
    }
    lv_obj_delete(objs[id]);
    objs[id] = NULL;
    obj_types[id] = TypeNone;
}

static void create_obj(int id, int type)
{
    if (id < 0 || id >= MAX_OBJS) {
        dropped_ops++;
        return;
    }
    delete_obj(id);

    lv_obj_t *screen = lv_screen_active();
    lv_obj_t *obj;
    switch (type) {
        case TypeBox:
            obj = lv_obj_create(screen);
            plain(obj);
            lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
            break;
        case TypeLabel:
            obj = lv_label_create(screen);
            plain(obj);
            lv_label_set_long_mode(obj, LV_LABEL_LONG_MODE_CLIP);
            break;
        case TypeMarquee:
            obj = lv_label_create(screen);
            plain(obj);
            lv_label_set_long_mode(obj, LV_LABEL_LONG_MODE_SCROLL_CIRCULAR);
            break;
        case TypeImage:
        case TypeFlipbook:
            obj = lv_image_create(screen);
            plain(obj);
            lv_image_set_inner_align(obj, LV_IMAGE_ALIGN_TOP_LEFT);
            lv_image_set_pivot(obj, 0, 0);
            break;
        default:
            dropped_ops++;
            return;
    }

    /* Ids are z-order: created at the end, then moved down to its place. */
    int32_t below = 0;
    for (int i = 0; i < id; i++) {
        if (objs[i] != NULL) {
            below++;
        }
    }
    lv_obj_move_to_index(obj, below);

    objs[id] = obj;
    obj_types[id] = type;
}

static void set_prop(int id, struct prop_value *p)
{
    lv_obj_t *obj = objs[id];
    int type = obj_types[id];
    int32_t v = p->value;

    switch (p->key) {
        case PropX:
            glide_to(id, move_x, lv_obj_get_x_aligned(obj), v);
            break;
        case PropY:
            glide_to(id, move_y, lv_obj_get_y_aligned(obj), v);
            break;
        case PropGlide:
            glides[id] = v > 0 ? v : 0;
            break;
        case PropFrames:
            if (type == TypeFlipbook) {
                struct flipbook *f = flip_for(id);
                f->count = 0;
                for (int32_t i = 0; i + 1 < v && f->count < FLIP_MAX_FRAMES; i += 2) {
                    f->ids[f->count++] = (uint8_t) p->text[i] | ((uint8_t) p->text[i + 1] << 8);
                }
                f->touched = true;
            }
            break;
        case PropFrameMs:
            if (type == TypeFlipbook) {
                flip_for(id)->ms = v;
                flip_for(id)->touched = true;
            }
            break;
        case PropW:
            lv_obj_set_width(obj, v);
            break;
        case PropH:
            lv_obj_set_height(obj, v);
            break;
        case PropBg:
            /* A negative background is none: the item is see-through. */
            if (v < 0) {
                lv_obj_set_style_bg_opa(obj, LV_OPA_TRANSP, 0);
            } else {
                lv_obj_set_style_bg_color(obj, lv_color_hex((uint32_t) v), 0);
                lv_obj_set_style_bg_opa(obj, LV_OPA_COVER, 0);
            }
            break;
        case PropFg:
            lv_obj_set_style_text_color(obj, lv_color_hex((uint32_t) v), 0);
            break;
        case PropText:
            if (type == TypeLabel || type == TypeMarquee) {
                lv_label_set_text(obj, p->text != NULL ? p->text : "");
                if (fx_params[id].kind != FxNone) {
                    fx_params[id].touched = true;
                }
            }
            break;
        case PropFont:
            if (v >= 0 && v < MAX_FONTS && fonts[v] != NULL) {
                lv_obj_set_style_text_font(obj, &fonts[v]->font, 0);
            }
            break;
        case PropSrc:
            if ((type == TypeImage || type == TypeFlipbook) && v >= 0 && v < MAX_IMAGES
                && images[v] != NULL) {
                lv_image_set_src(obj, images[v]);
            }
            break;
        case PropScaleX:
            lv_image_set_scale_x(obj, v);
            break;
        case PropScaleY:
            lv_image_set_scale_y(obj, v);
            break;
        case PropOffsetX:
            lv_image_set_offset_x(obj, v);
            break;
        case PropOffsetY:
            lv_image_set_offset_y(obj, v);
            break;
        case PropHidden:
            lv_obj_set_hidden(obj, v != 0);
            break;
        case PropRecolor:
            if (v < 0) {
                lv_obj_set_style_image_recolor_opa(obj, LV_OPA_TRANSP, 0);
            } else {
                lv_obj_set_style_image_recolor(obj, lv_color_hex((uint32_t) v), 0);
                lv_obj_set_style_image_recolor_opa(obj, LV_OPA_COVER, 0);
            }
            break;
        case PropFx:
        case PropFxDir:
        case PropFxSteps:
        case PropFxMs:
        case PropFxRow:
        case PropFxLeft:
        case PropFxNoise:
            set_fx_param(id, p->key, v);
            break;
        case PropMoveX:
        case PropMoveY:
        case PropMoveDir:
        case PropMoveDelay:
        case PropMoveMs:
        case PropMoveEase:
            set_motion_param(id, p->key, v);
            break;
        case PropSpeed:
            /* Pixels a second, however long the text. */
            lv_obj_set_style_anim_duration(obj, lv_anim_speed(v > 0 ? v : 40), 0);
            break;
        default:
            break;
    }
}

/* ---------------------------------------------------------------------------
 * Decor, LVGL task only
 *
 * Theme decorations on LVGL's top layer, over every page: a border whose
 * colours travel round the panel, a beam that sweeps down it, and glitch
 * bars that flash at random. Only the strips they cover are redrawn.
 * ------------------------------------------------------------------------- */

#define DECOR_MAX_COLOURS 8
#define DECOR_GLITCH_BARS 3
#define DECOR_TICK_MS 40
#define BORDER_SEGMENT 4
#define DECOR_MAX_LINES 4

enum line_mode
{
    LinePulse = 0,
    LineFlow
};

struct decor_line
{
    int16_t x;
    int16_t y;
    int16_t w;
    int16_t h;
    uint8_t mode;
    uint8_t min_pct;
    uint16_t period;
    uint8_t count;
    uint32_t colours[DECOR_MAX_COLOURS];
};

struct decor
{
    bool border;
    uint8_t thickness;
    uint16_t speed;
    uint8_t border_count;
    uint32_t border_colours[DECOR_MAX_COLOURS];

    bool beam;
    uint32_t beam_colour;
    uint16_t beam_period;
    uint8_t beam_height;
    uint8_t beam_opa;

    bool glitch;
    uint8_t glitch_count;
    uint32_t glitch_colours[DECOR_MAX_COLOURS];
    uint16_t glitch_min;
    uint16_t glitch_max;

    uint8_t line_count;
    struct decor_line lines[DECOR_MAX_LINES];
};

static struct decor decor_now;
static lv_obj_t *border_edges[4];
static lv_obj_t *line_objs[DECOR_MAX_LINES];
static lv_obj_t *beam_obj;
static lv_obj_t *glitch_bars[DECOR_GLITCH_BARS];
static lv_timer_t *border_timer;
static lv_timer_t *glitch_timer;
static int32_t border_phase;
static bool glitch_showing;

static int32_t panel_w(void)
{
    return lv_display_get_horizontal_resolution(display);
}

static int32_t panel_h(void)
{
    return lv_display_get_vertical_resolution(display);
}

/* The colour at a distance round the perimeter, the palette spread evenly and looping. */
static lv_color_t border_colour(int32_t pos)
{
    int32_t perimeter = 2 * (panel_w() + panel_h());
    int32_t n = decor_now.border_count;
    if (n <= 1) {
        return lv_color_hex(n == 1 ? decor_now.border_colours[0] : 0xFFFFFF);
    }
    int32_t at = ((pos + border_phase) % perimeter + perimeter) % perimeter;
    int32_t scaled = at * n;
    int32_t i = scaled / perimeter;
    int32_t frac = (scaled % perimeter) * 255 / perimeter;
    uint32_t a = decor_now.border_colours[i];
    uint32_t b = decor_now.border_colours[(i + 1) % n];
    int32_t r = ((a >> 16) & 0xFF) + ((((int32_t) ((b >> 16) & 0xFF)) - ((a >> 16) & 0xFF)) * frac) / 255;
    int32_t g = ((a >> 8) & 0xFF) + ((((int32_t) ((b >> 8) & 0xFF)) - ((a >> 8) & 0xFF)) * frac) / 255;
    int32_t bl = (a & 0xFF) + ((((int32_t) (b & 0xFF)) - (a & 0xFF)) * frac) / 255;
    return lv_color_make(r, g, bl);
}

/* Perimeter position of a point on an edge, clockwise from the top-left corner. */
static int32_t perimeter_pos(int edge, int32_t along)
{
    int32_t w = panel_w();
    int32_t h = panel_h();
    switch (edge) {
        case 0:
            return along;
        case 1:
            return w + along;
        case 2:
            return w + h + (w - along);
        default:
            return 2 * w + h + (h - along);
    }
}

static void border_draw(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target_obj(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    int edge = (int) (intptr_t) lv_event_get_user_data(e);
    lv_area_t box;
    lv_obj_get_coords(obj, &box);

    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_opa = LV_OPA_COVER;

    bool across = edge == 0 || edge == 2;
    int32_t length = across ? lv_area_get_width(&box) : lv_area_get_height(&box);
    int32_t offset = across ? box.x1 : box.y1;

    for (int32_t along = 0; along < length; along += BORDER_SEGMENT) {
        lv_area_t seg = box;
        if (across) {
            seg.x1 = box.x1 + along;
            seg.x2 = LV_MIN(seg.x1 + BORDER_SEGMENT - 1, box.x2);
        } else {
            seg.y1 = box.y1 + along;
            seg.y2 = LV_MIN(seg.y1 + BORDER_SEGMENT - 1, box.y2);
        }
        dsc.bg_color = border_colour(perimeter_pos(edge, offset + along));
        lv_draw_rect(layer, &dsc, &seg);
    }
}

/* Linear through `count` stops over `span`, looping back to the first when `loop`. */
static uint32_t gradient_at(const uint32_t *colours, int32_t count, int32_t at, int32_t span, bool loop)
{
    if (count <= 1 || span <= 0) {
        return count == 1 ? colours[0] : 0xFFFFFF;
    }
    int32_t segments = loop ? count : count - 1;
    int32_t scaled = LV_CLAMP(0, at, span) * segments;
    int32_t i = LV_MIN(scaled / span, segments - 1);
    int32_t frac = ((scaled - i * span) * 255) / span;
    uint32_t a = colours[i];
    uint32_t b = colours[(i + 1) % count];
    uint32_t out = 0;
    for (int shift = 16; shift >= 0; shift -= 8) {
        int32_t ca = (a >> shift) & 0xFF;
        int32_t cb = (b >> shift) & 0xFF;
        out |= (uint32_t) (ca + ((cb - ca) * frac) / 255) << shift;
    }
    return out;
}

static uint32_t dim(uint32_t rgb, int32_t level)
{
    uint32_t out = 0;
    for (int shift = 16; shift >= 0; shift -= 8) {
        out |= (uint32_t) ((((rgb >> shift) & 0xFF) * level) / 255) << shift;
    }
    return out;
}

/* Brightness 0..255 for a pulsing line: a sine between its floor and full, on the shared clock. */
static int32_t pulse_level(const struct decor_line *l)
{
    int32_t period = l->period > 0 ? l->period : 2000;
    int32_t angle = (int32_t) ((lv_tick_get() % period) * 360 / period);
    int32_t wave = (lv_trigo_sin(angle) + 32767) * 255 / 65534;
    int32_t floor = l->min_pct * 255 / 100;
    return floor + ((255 - floor) * wave) / 255;
}

static void line_draw(lv_event_t *e)
{
    lv_obj_t *obj = lv_event_get_target_obj(e);
    lv_layer_t *layer = lv_event_get_layer(e);
    const struct decor_line *l = &decor_now.lines[(int) (intptr_t) lv_event_get_user_data(e)];
    lv_area_t box;
    lv_obj_get_coords(obj, &box);

    lv_draw_rect_dsc_t dsc;
    lv_draw_rect_dsc_init(&dsc);
    dsc.bg_opa = LV_OPA_COVER;

    int32_t width = lv_area_get_width(&box);
    int32_t level = l->mode == LinePulse ? pulse_level(l) : 255;
    int32_t shift = 0;
    if (l->mode == LineFlow && l->period > 0) {
        shift = (int32_t) ((lv_tick_get() % l->period) * width / l->period);
    }

    for (int32_t along = 0; along < width; along += BORDER_SEGMENT) {
        lv_area_t seg = box;
        seg.x1 = box.x1 + along;
        seg.x2 = LV_MIN(seg.x1 + BORDER_SEGMENT - 1, box.x2);
        uint32_t rgb = l->mode == LineFlow
            ? gradient_at(l->colours, l->count, (along + shift) % width, width, true)
            : gradient_at(l->colours, l->count, along, width, false);
        dsc.bg_color = lv_color_hex(dim(rgb, level));
        lv_draw_rect(layer, &dsc, &seg);
    }
}

static void border_tick(lv_timer_t *timer)
{
    UNUSED(timer);
    border_phase += (decor_now.speed * DECOR_TICK_MS) / 1000;
    for (int i = 0; i < 4; i++) {
        if (border_edges[i] != NULL) {
            lv_obj_invalidate(border_edges[i]);
        }
    }
    for (int i = 0; i < DECOR_MAX_LINES; i++) {
        if (line_objs[i] != NULL) {
            lv_obj_invalidate(line_objs[i]);
        }
    }
}

static lv_obj_t *decor_box(void)
{
    lv_obj_t *obj = lv_obj_create(lv_layer_top());
    plain(obj);
    return obj;
}

static void glitch_tick(lv_timer_t *timer)
{
    if (glitch_showing) {
        for (int i = 0; i < DECOR_GLITCH_BARS; i++) {
            lv_obj_set_hidden(glitch_bars[i], true);
        }
        glitch_showing = false;
        lv_timer_set_period(timer, lv_rand(decor_now.glitch_min, decor_now.glitch_max));
        return;
    }

    int32_t w = panel_w();
    int32_t h = panel_h();
    for (int i = 0; i < DECOR_GLITCH_BARS; i++) {
        lv_obj_t *bar = glitch_bars[i];
        int32_t height = lv_rand(1, 4);
        int32_t width = lv_rand(w / 4, w);
        uint32_t colour = decor_now.glitch_colours[lv_rand(0, decor_now.glitch_count - 1)];
        lv_obj_set_pos(bar, lv_rand(0, w - width), lv_rand(0, h - height));
        lv_obj_set_size(bar, width, height);
        lv_obj_set_style_bg_color(bar, lv_color_hex(colour), 0);
        lv_obj_set_hidden(bar, false);
    }
    glitch_showing = true;
    lv_timer_set_period(timer, lv_rand(60, 140));
}

static void decor_clear(void)
{
    if (border_timer != NULL) {
        lv_timer_delete(border_timer);
        border_timer = NULL;
    }
    if (glitch_timer != NULL) {
        lv_timer_delete(glitch_timer);
        glitch_timer = NULL;
    }
    for (int i = 0; i < 4; i++) {
        if (border_edges[i] != NULL) {
            lv_obj_delete(border_edges[i]);
            border_edges[i] = NULL;
        }
    }
    for (int i = 0; i < DECOR_MAX_LINES; i++) {
        if (line_objs[i] != NULL) {
            lv_obj_delete(line_objs[i]);
            line_objs[i] = NULL;
        }
    }
    if (beam_obj != NULL) {
        lv_obj_delete(beam_obj);
        beam_obj = NULL;
    }
    for (int i = 0; i < DECOR_GLITCH_BARS; i++) {
        if (glitch_bars[i] != NULL) {
            lv_obj_delete(glitch_bars[i]);
            glitch_bars[i] = NULL;
        }
    }
    glitch_showing = false;
}

static void set_beam_y(void *obj, int32_t y)
{
    lv_obj_set_y(obj, y);
}

static void decor_apply(struct decor *d)
{
    decor_clear();
    decor_now = *d;
    int32_t w = panel_w();
    int32_t h = panel_h();

    /* Lines first, so the border's corners sit over their ends. */
    for (int i = 0; i < d->line_count; i++) {
        const struct decor_line *l = &d->lines[i];
        lv_obj_t *obj = decor_box();
        lv_obj_set_pos(obj, l->x, l->y);
        lv_obj_set_size(obj, l->w, l->h);
        lv_obj_add_event_cb(obj, line_draw, LV_EVENT_DRAW_MAIN, (void *) (intptr_t) i);
        line_objs[i] = obj;
    }

    if (d->border && d->thickness > 0) {
        int32_t t = d->thickness;
        int32_t geometry[4][4] = {
            { 0, 0, w, t }, { w - t, t, t, h - 2 * t }, { 0, h - t, w, t }, { 0, t, t, h - 2 * t }
        };
        for (int i = 0; i < 4; i++) {
            lv_obj_t *edge = decor_box();
            lv_obj_set_pos(edge, geometry[i][0], geometry[i][1]);
            lv_obj_set_size(edge, geometry[i][2], geometry[i][3]);
            lv_obj_add_event_cb(edge, border_draw, LV_EVENT_DRAW_MAIN, (void *) (intptr_t) i);
            border_edges[i] = edge;
        }
    }

    if ((d->border && d->thickness > 0) || d->line_count > 0) {
        border_timer = lv_timer_create(border_tick, DECOR_TICK_MS, NULL);
    }

    if (d->beam && d->beam_period > 0) {
        beam_obj = decor_box();
        lv_obj_set_size(beam_obj, w, d->beam_height);
        lv_obj_set_style_bg_color(beam_obj, lv_color_hex(d->beam_colour), 0);
        lv_obj_set_style_bg_opa(beam_obj, d->beam_opa, 0);

        /* Sweeps down in a third of the period, then waits out the rest. */
        lv_anim_t a;
        lv_anim_init(&a);
        lv_anim_set_var(&a, beam_obj);
        lv_anim_set_exec_cb(&a, set_beam_y);
        lv_anim_set_values(&a, -d->beam_height, h);
        lv_anim_set_duration(&a, d->beam_period / 3);
        lv_anim_set_repeat_delay(&a, d->beam_period - d->beam_period / 3);
        lv_anim_set_repeat_count(&a, LV_ANIM_REPEAT_INFINITE);
        lv_anim_start(&a);
    }

    if (d->glitch && d->glitch_count > 0 && d->glitch_max >= d->glitch_min) {
        for (int i = 0; i < DECOR_GLITCH_BARS; i++) {
            lv_obj_t *bar = decor_box();
            lv_obj_set_style_bg_opa(bar, LV_OPA_80, 0);
            lv_obj_set_hidden(bar, true);
            glitch_bars[i] = bar;
        }
        glitch_timer = lv_timer_create(glitch_tick, lv_rand(d->glitch_min, d->glitch_max), NULL);
    }
}

static void apply_op(struct op *op)
{
    switch (op->kind) {
        case OpDecor:
            decor_apply(op->decor);
            break;
        case OpReset:
            for (int i = MAX_OBJS - 1; i >= 0; i--) {
                delete_obj(i);
            }
            for (int i = 0; i < MAX_IMAGES; i++) {
                free_image(i);
            }
            break;
        case OpNew:
            create_obj(op->id, op->type);
            break;
        case OpDel:
            delete_obj(op->id);
            break;
        case OpSet:
            if (op->id < MAX_OBJS && objs[op->id] != NULL) {
                for (uint16_t i = 0; i < op->prop_count; i++) {
                    set_prop(op->id, &op->props[i]);
                }
                /* A new text or effect restarts the effect, once every property has landed. */
                if (fx_params[op->id].touched) {
                    fx_params[op->id].touched = false;
                    fx_start(op->id);
                }
                if (motions[op->id].touched) {
                    motions[op->id].touched = false;
                    motion_start(op->id);
                }
                if (flips[op->id] != NULL && flips[op->id]->touched) {
                    flip_start(op->id);
                }
            } else {
                dropped_ops++;
            }
            break;
        case OpImg:
            load_image(op->id, op->format, op->w, op->h, op->scale, op->data, op->size);
            op->data = NULL;
            break;
        case OpUnimg:
            free_image(op->id);
            break;
        case OpFont:
            load_font(op->id, op->format, op->data, op->size);
            op->data = NULL;
            break;
        default:
            dropped_ops++;
            break;
    }
}

static void free_batch(struct batch *b)
{
    for (size_t i = 0; i < b->count; i++) {
        struct op *op = &b->ops[i];
        for (uint16_t j = 0; j < op->prop_count; j++) {
            heap_caps_free(op->props[j].text);
        }
        heap_caps_free(op->props);
        heap_caps_free(op->data);
        heap_caps_free(op->decor);
    }
    heap_caps_free(b->ops);
    heap_caps_free(b);
}

/* Runs inside the LVGL task, so it may touch LVGL. */
static void drain(lv_timer_t *timer)
{
    UNUSED(timer);
    struct batch *b;
    bool applied = false;
    while (xQueueReceive(queue, &b, 0) == pdTRUE) {
        for (size_t i = 0; i < b->count; i++) {
            apply_op(&b->ops[i]);
        }
        free_batch(b);
        applied = true;
    }
    /* Drawn now rather than on the next refresh period, so a key press shows at once. */
    if (applied) {
        lv_refr_now(display);
    }
}

/* ---------------------------------------------------------------------------
 * Parsing, VM scheduler thread: no LVGL here
 * ------------------------------------------------------------------------- */

static size_t list_length(term list)
{
    size_t n = 0;
    while (term_is_nonempty_list(list)) {
        n++;
        list = term_get_list_tail(list);
    }
    return n;
}

static int int_at(term tuple, int index)
{
    if (term_get_tuple_arity(tuple) <= index) {
        return 0;
    }
    term value = term_get_tuple_element(tuple, index);
    if (term_is_integer(value)) {
        return term_to_int(value);
    }
    if (value == TRUE_ATOM) {
        return 1;
    }
    return 0;
}

static uint8_t *copy_binary(term value, size_t *size_out)
{
    if (!term_is_binary(value)) {
        *size_out = 0;
        return NULL;
    }
    size_t size = term_binary_size(value);
    uint8_t *copy = psram_malloc(size > 0 ? size : 1);
    if (copy != NULL) {
        memcpy(copy, term_binary_data(value), size);
    }
    *size_out = size;
    return copy;
}

static char *copy_text(term value)
{
    if (term_is_binary(value)) {
        size_t size = term_binary_size(value);
        char *text = psram_malloc(size + 1);
        memcpy(text, term_binary_data(value), size);
        text[size] = '\0';
        return text;
    }
    int ok;
    char *text = interop_term_to_string(value, &ok);
    return ok ? text : NULL;
}

static bool parse_props(struct op *op, term list, GlobalContext *global)
{
    op->prop_count = list_length(list);
    op->props = heap_caps_calloc(op->prop_count > 0 ? op->prop_count : 1, sizeof(struct prop_value), PSRAM_CAPS);
    if (op->props == NULL) {
        return false;
    }
    for (uint16_t i = 0; i < op->prop_count; i++) {
        term pair = term_get_list_head(list);
        list = term_get_list_tail(list);
        if (!term_is_tuple(pair) || term_get_tuple_arity(pair) != 2) {
            continue;
        }
        struct prop_value *p = &op->props[i];
        p->key = interop_atom_term_select_int(prop_table, term_get_tuple_element(pair, 0), global);
        term value = term_get_tuple_element(pair, 1);
        if (p->key == PropText) {
            p->text = copy_text(value);
        } else if (p->key == PropFrames && term_is_binary(value)) {
            size_t size;
            p->text = (char *) copy_binary(value, &size);
            p->value = (int32_t) size;
        } else if (term_is_integer(value)) {
            p->value = term_to_int(value);
        } else {
            p->value = value == TRUE_ATOM ? 1 : (value == FALSE_ATOM ? 0 : -1);
        }
    }
    return true;
}

static uint8_t parse_colours(term list, uint32_t *out)
{
    uint8_t n = 0;
    while (term_is_nonempty_list(list) && n < DECOR_MAX_COLOURS) {
        term c = term_get_list_head(list);
        if (term_is_integer(c)) {
            out[n++] = (uint32_t) term_to_int(c);
        }
        list = term_get_list_tail(list);
    }
    return n;
}

static bool parse_decor(struct op *op, term specs, GlobalContext *global)
{
    struct decor *d = heap_caps_calloc(1, sizeof(struct decor), PSRAM_CAPS);
    if (d == NULL) {
        return false;
    }
    op->decor = d;
    term border = globalcontext_make_atom(global, ATOM_STR("\x6", "border"));
    term beam = globalcontext_make_atom(global, ATOM_STR("\x4", "beam"));
    term glitch = globalcontext_make_atom(global, ATOM_STR("\x6", "glitch"));
    term line = globalcontext_make_atom(global, ATOM_STR("\x4", "line"));
    term flow = globalcontext_make_atom(global, ATOM_STR("\x4", "flow"));

    while (term_is_nonempty_list(specs)) {
        term spec = term_get_list_head(specs);
        specs = term_get_list_tail(specs);
        if (!term_is_tuple(spec) || term_get_tuple_arity(spec) < 1) {
            continue;
        }
        term kind = term_get_tuple_element(spec, 0);
        int arity = term_get_tuple_arity(spec);
        if (kind == border && arity == 4) {
            d->border = true;
            d->thickness = int_at(spec, 1);
            d->speed = int_at(spec, 2);
            d->border_count = parse_colours(term_get_tuple_element(spec, 3), d->border_colours);
        } else if (kind == beam && arity == 5) {
            d->beam = true;
            d->beam_colour = (uint32_t) int_at(spec, 1);
            d->beam_period = int_at(spec, 2);
            d->beam_height = int_at(spec, 3);
            d->beam_opa = int_at(spec, 4);
        } else if (kind == glitch && arity == 4) {
            d->glitch = true;
            d->glitch_count = parse_colours(term_get_tuple_element(spec, 1), d->glitch_colours);
            d->glitch_min = int_at(spec, 2);
            d->glitch_max = int_at(spec, 3);
        } else if (kind == line && arity == 9 && d->line_count < DECOR_MAX_LINES) {
            struct decor_line *l = &d->lines[d->line_count++];
            l->x = int_at(spec, 1);
            l->y = int_at(spec, 2);
            l->w = int_at(spec, 3);
            l->h = int_at(spec, 4);
            l->count = parse_colours(term_get_tuple_element(spec, 5), l->colours);
            l->mode = term_get_tuple_element(spec, 6) == flow ? LineFlow : LinePulse;
            l->period = int_at(spec, 7);
            l->min_pct = int_at(spec, 8);
        }
    }
    return true;
}

static bool parse_op(struct op *op, term t, GlobalContext *global)
{
    if (!term_is_tuple(t) || term_get_tuple_arity(t) < 1) {
        return false;
    }
    op->kind = interop_atom_term_select_int(op_table, term_get_tuple_element(t, 0), global);
    if (op->kind == OpReset) {
        return true;
    }
    if (op->kind == OpDecor) {
        return term_get_tuple_arity(t) == 2 && parse_decor(op, term_get_tuple_element(t, 1), global);
    }
    if (term_get_tuple_arity(t) < 2) {
        return false;
    }
    op->id = int_at(t, 1);

    switch (op->kind) {
        case OpNew:
            op->type = interop_atom_term_select_int(type_table, term_get_tuple_element(t, 2), global);
            return op->type != TypeNone;
        case OpDel:
        case OpUnimg:
            return true;
        case OpSet:
            return term_get_tuple_arity(t) == 3 && parse_props(op, term_get_tuple_element(t, 2), global);
        case OpImg:
            if (term_get_tuple_arity(t) != 6 && term_get_tuple_arity(t) != 7) {
                return false;
            }
            op->scale = term_get_tuple_arity(t) == 7 ? int_at(t, 6) : 1;
            op->format = interop_atom_term_select_int(format_table, term_get_tuple_element(t, 2), global);
            op->w = int_at(t, 3);
            op->h = int_at(t, 4);
            op->data = copy_binary(term_get_tuple_element(t, 5), &op->size);
            return op->data != NULL && (op->format == FmtRgba8888 || op->format == FmtA8);
        case OpFont:
            if (term_get_tuple_arity(t) != 4) {
                return false;
            }
            op->format = interop_atom_term_select_int(format_table, term_get_tuple_element(t, 2), global);
            op->data = copy_binary(term_get_tuple_element(t, 3), &op->size);
            return op->data != NULL && (op->format == FmtUf || op->format == FmtRaw8x16);
        default:
            return false;
    }
}

static term enqueue_batch(term ops, GlobalContext *global)
{
    if (queue == NULL) {
        return ERROR_ATOM;
    }
    struct batch *b = heap_caps_calloc(1, sizeof(struct batch), PSRAM_CAPS);
    b->count = list_length(ops);
    b->ops = heap_caps_calloc(b->count > 0 ? b->count : 1, sizeof(struct op), PSRAM_CAPS);

    for (size_t i = 0; i < b->count; i++) {
        if (!parse_op(&b->ops[i], term_get_list_head(ops), global)) {
            b->ops[i].kind = OpInvalid;
        }
        ops = term_get_list_tail(ops);
    }

    if (xQueueSend(queue, &b, 0) != pdTRUE) {
        free_batch(b);
        return globalcontext_make_atom(global, busy_atom);
    }
    return OK_ATOM;
}

/* ---------------------------------------------------------------------------
 * Display
 * ------------------------------------------------------------------------- */

/*
 * Replaces esp_lvgl_port's flush. A transfer that cannot be queued never
 * raises the done callback, and LVGL would wait on it forever with its task
 * spinning; so a failed strip is dropped and LVGL told it is finished.
 */
static void flush(lv_display_t *disp, const lv_area_t *area, uint8_t *pixels)
{
    lv_draw_sw_rgb565_swap(pixels, lv_area_get_size(area));

    esp_err_t err = esp_lcd_panel_draw_bitmap(
        panel_handle, area->x1, area->y1, area->x2 + 1, area->y2 + 1, pixels);
    if (err != ESP_OK) {
        failed_flushes++;
        lv_display_flush_ready(disp);
    }
}

static void count_refresh(lv_event_t *event)
{
    UNUSED(event);
    refreshes++;
}

static term stats_term(Context *ctx)
{
    term tuple = term_alloc_tuple(5, &ctx->heap);
    term_put_tuple_element(tuple, 0, term_from_int(heap_caps_get_free_size(MALLOC_CAP_INTERNAL)));
    term_put_tuple_element(tuple, 1, term_from_int(heap_caps_get_free_size(MALLOC_CAP_DMA)));
    term_put_tuple_element(
        tuple, 2, term_from_int(heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL)));
    term_put_tuple_element(tuple, 3, term_from_int(heap_caps_get_free_size(MALLOC_CAP_SPIRAM)));
    term_put_tuple_element(tuple, 4, term_from_int((avm_int_t) refreshes));
    return tuple;
}

/* ---------------------------------------------------------------------------
 * Diagnostics: heap tracing in leaks mode, when built with it. What is still
 * recorded at the dump was allocated after the start and never freed; only
 * internal RAM is reported, grouped by the allocating call chain.
 * ------------------------------------------------------------------------- */

#ifdef CONFIG_HEAP_TRACING_STANDALONE
#define TRACE_RECORDS 3000
#define TRACE_GROUPS 48

static heap_trace_record_t *trace_records;

struct trace_group
{
    void *callers[CONFIG_HEAP_TRACING_STACK_DEPTH];
    size_t bytes;
    int count;
};

static void trace_start(void)
{
    if (trace_records == NULL) {
        trace_records = heap_caps_calloc(TRACE_RECORDS, sizeof(heap_trace_record_t), MALLOC_CAP_SPIRAM);
        ESP_ERROR_CHECK(heap_trace_init_standalone(trace_records, TRACE_RECORDS));
    }
    ESP_ERROR_CHECK(heap_trace_start(HEAP_TRACE_LEAKS));
    ESP_LOGI(TAG, "trace: started, internal_free=%u", heap_caps_get_free_size(MALLOC_CAP_INTERNAL));
}

static void trace_dump(void)
{
    heap_trace_stop();
    size_t count = heap_trace_get_count();
    struct trace_group *groups = heap_caps_calloc(TRACE_GROUPS, sizeof(struct trace_group), MALLOC_CAP_SPIRAM);
    int used = 0;
    size_t internal_bytes = 0;
    int internal_count = 0;

    for (size_t i = 0; i < count; i++) {
        heap_trace_record_t rec;
        if (heap_trace_get(i, &rec) != ESP_OK || !esp_ptr_internal(rec.address)) {
            continue;
        }
        internal_bytes += rec.size;
        internal_count++;

        int g = 0;
        while (g < used
            && memcmp(groups[g].callers, rec.alloced_by, sizeof(groups[g].callers)) != 0) {
            g++;
        }
        if (g == used) {
            if (used == TRACE_GROUPS) {
                continue;
            }
            memcpy(groups[g].callers, rec.alloced_by, sizeof(groups[g].callers));
            used++;
        }
        groups[g].bytes += rec.size;
        groups[g].count++;
    }

    ESP_LOGI(TAG, "trace: %u records, %d internal still allocated, %u bytes, %d call chains, internal_free=%u",
        (unsigned) count, internal_count, (unsigned) internal_bytes, used,
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL));

    for (int g = 0; g < used; g++) {
        printf("TRACE bytes=%u count=%d callers=", (unsigned) groups[g].bytes, groups[g].count);
        for (int d = 0; d < CONFIG_HEAP_TRACING_STACK_DEPTH; d++) {
            printf("0x%08x ", (unsigned) (uintptr_t) groups[g].callers[d]);
        }
        printf("\n");
    }
    heap_caps_free(groups);
}
#else
static void trace_start(void)
{
    ESP_LOGW(TAG, "trace: needs CONFIG_HEAP_TRACING_STANDALONE");
}

static void trace_dump(void)
{
}
#endif


static bool start_display(term opts, GlobalContext *global)
{
    int width = get_int_default(opts, width_atom, 320, global);
    int height = get_int_default(opts, height_atom, 240, global);

    spi_bus_config_t bus = {
        .sclk_io_num = get_int_default(opts, sclk_atom, 5, global),
        .mosi_io_num = get_int_default(opts, mosi_atom, 8, global),
        .miso_io_num = -1,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = width * DRAW_ROWS * sizeof(uint16_t),
    };
    esp_err_t err = spi_bus_initialize(LCD_HOST, &bus, SPI_DMA_CH_AUTO);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize: %s", esp_err_to_name(err));
        return false;
    }

    esp_lcd_panel_io_handle_t io = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = get_int_default(opts, dc_atom, 4, global),
        .cs_gpio_num = get_int_default(opts, cs_atom, 7, global),
        .pclk_hz = get_int_default(opts, clock_hz_atom, 40000000, global),
        .lcd_cmd_bits = 8,
        .lcd_param_bits = 8,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    err = esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t) LCD_HOST, &io_config, &io);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_io_spi: %s", esp_err_to_name(err));
        return false;
    }

    esp_lcd_panel_handle_t panel = NULL;
    esp_lcd_panel_dev_config_t panel_config = {
        .reset_gpio_num = get_int_default(opts, reset_atom, 6, global),
        .rgb_ele_order = get_bool_default(opts, bgr_atom, false, global)
            ? LCD_RGB_ELEMENT_ORDER_BGR
            : LCD_RGB_ELEMENT_ORDER_RGB,
        .bits_per_pixel = 16,
    };
    err = esp_lcd_new_panel_st7789(io, &panel_config, &panel);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "esp_lcd_new_panel_st7789: %s", esp_err_to_name(err));
        return false;
    }
    panel_handle = panel;
    esp_lcd_panel_reset(panel);
    esp_lcd_panel_init(panel);
    esp_lcd_panel_invert_color(panel, get_bool_default(opts, invert_atom, true, global));
    esp_lcd_panel_disp_on_off(panel, true);

    lvgl_port_cfg_t lvgl_config = ESP_LVGL_PORT_INIT_CONFIG();
    /* Internal RAM is what wifi and TLS run short of; the stack can live in PSRAM. */
    lvgl_config.task_stack_caps = MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT;
    err = lvgl_port_init(&lvgl_config);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "lvgl_port_init: %s", esp_err_to_name(err));
        return false;
    }

    lvgl_port_display_cfg_t display_config = {
        .io_handle = io,
        .panel_handle = panel,
        .buffer_size = width * DRAW_ROWS,
        .double_buffer = true,
        .hres = width,
        .vres = height,
        .monochrome = false,
        .color_format = LV_COLOR_FORMAT_RGB565,
        .rotation = {
            .swap_xy = get_bool_default(opts, swap_xy_atom, true, global),
            .mirror_x = get_bool_default(opts, mirror_x_atom, false, global),
            .mirror_y = get_bool_default(opts, mirror_y_atom, true, global),
        },
        .flags = {
            .buff_dma = true,
            .swap_bytes = true,
        },
    };
    display = lvgl_port_add_disp(&display_config);
    if (display == NULL) {
        ESP_LOGE(TAG, "lvgl_port_add_disp failed");
        return false;
    }

    queue = xQueueCreate(QUEUE_DEPTH, sizeof(struct batch *));

    lvgl_port_lock(0);
    lv_display_set_flush_cb(display, flush);
    lv_obj_t *screen = lv_screen_active();
    lv_obj_remove_style_all(screen);
    lv_obj_set_scrollable(screen, false);
    lv_obj_set_style_bg_color(screen, lv_color_hex(0x000000), 0);
    lv_obj_set_style_bg_opa(screen, LV_OPA_COVER, 0);
    lv_display_add_event_cb(display, count_refresh, LV_EVENT_REFR_READY, NULL);
    lv_timer_create(drain, 5, NULL);
    lvgl_port_unlock();

    ESP_LOGI(TAG, "started %dx%d, internal_free=%u dma_free=%u", width, height,
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_DMA));
    return true;
}

static NativeHandlerResult consume_mailbox(Context *ctx)
{
    GlobalContext *global = ctx->global;
    Message *message = mailbox_first(&ctx->mailbox);

    GenMessage gen_message;
    if (UNLIKELY(port_parse_gen_message(message->message, &gen_message) != GenCallMessage)) {
        ESP_LOGW(TAG, "Received a message that is not a call");
        mailbox_remove_message(&ctx->mailbox, &ctx->heap);
        return NativeContinue;
    }

    term req = gen_message.req;
    term req_term = term_is_tuple(req) ? term_get_tuple_element(req, 0) : req;
    int kind = interop_atom_term_select_int(request_table, req_term, global);

    switch (kind) {
        case ReqStats:
            port_ensure_available(ctx, PORT_REPLY_SIZE + TUPLE_SIZE(5));
            port_send_reply(ctx, gen_message.pid, gen_message.ref, stats_term(ctx));
            break;

        case ReqBatch: {
            term reply = BADARG_ATOM;
            if (term_is_tuple(req) && term_get_tuple_arity(req) == 2) {
                term ops = term_get_tuple_element(req, 1);
                if (term_is_list(ops)) {
                    reply = enqueue_batch(ops, global);
                }
            }
            port_ensure_available(ctx, PORT_REPLY_SIZE);
            port_send_reply(ctx, gen_message.pid, gen_message.ref, reply);
            break;
        }

        case ReqTraceStart:
        case ReqTraceDump:
            if (kind == ReqTraceStart) {
                trace_start();
            } else {
                trace_dump();
            }
            port_ensure_available(ctx, PORT_REPLY_SIZE);
            port_send_reply(ctx, gen_message.pid, gen_message.ref, OK_ATOM);
            break;

        default:
            port_ensure_available(ctx, PORT_REPLY_SIZE);
            port_send_reply(ctx, gen_message.pid, gen_message.ref, BADARG_ATOM);
            break;
    }

    mailbox_remove_message(&ctx->mailbox, &ctx->heap);
    return NativeContinue;
}

void atomvm_lvgl_init(GlobalContext *global)
{
    UNUSED(global);
}

Context *atomvm_lvgl_create_port(GlobalContext *global, term opts)
{
    if (started) {
        ESP_LOGE(TAG, "Only one LVGL display is supported");
        return NULL;
    }
    /* Serves only `stats`, to measure memory without LVGL running. */
    if (get_bool_default(opts, no_display_atom, false, global)) {
        ESP_LOGI(TAG, "no_display: stats only");
    } else if (!start_display(opts, global)) {
        return NULL;
    } else {
        started = true;
    }

    Context *ctx = context_new(global);
    ctx->native_handler = consume_mailbox;
    return ctx;
}

REGISTER_PORT_DRIVER(lvgl, atomvm_lvgl_init, NULL, atomvm_lvgl_create_port)
