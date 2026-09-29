/*
 * SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
 *
 * LVGL display port driver.
 *
 * Erlang owns a flat set of LVGL objects, addressed by small integer ids that
 * are also their z-order (0 at the bottom), and changes them in batches:
 *
 *   {batch, [Op]} -> ok | busy        applied atomically, in order
 *     {new, Id, box | label | marquee | image}  created at z-index Id; a marquee
 *                                      is a label that scrolls round when wider than w
 *     {del, Id}
 *     {set, Id, [{Prop, Value}]}      x y w h bg fg text font src sx sy ox oy hidden recolor speed
 *     {img, ImgId, rgba8888 | a8, W, H, Bin}
 *     {unimg, ImgId}
 *     {font, FontId, uf | raw8x16, Bin}  loaded once; a second load of an id is ignored
 *     {reset}                          deletes every object and image, keeps fonts
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
    OpReset
};

static const AtomStringIntPair op_table[] = {
    { ATOM_STR("\x3", "new"), OpNew },
    { ATOM_STR("\x3", "del"), OpDel },
    { ATOM_STR("\x3", "set"), OpSet },
    { ATOM_STR("\x3", "img"), OpImg },
    { ATOM_STR("\x5", "unimg"), OpUnimg },
    { ATOM_STR("\x4", "font"), OpFont },
    { ATOM_STR("\x5", "reset"), OpReset },
    SELECT_INT_DEFAULT(OpInvalid)
};

enum obj_type
{
    TypeNone = 0,
    TypeBox,
    TypeLabel,
    TypeImage,
    TypeMarquee
};

static const AtomStringIntPair type_table[] = {
    { ATOM_STR("\x3", "box"), TypeBox },
    { ATOM_STR("\x5", "label"), TypeLabel },
    { ATOM_STR("\x5", "image"), TypeImage },
    { ATOM_STR("\x7", "marquee"), TypeMarquee },
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
    PropSpeed
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
    uint16_t prop_count;
    struct prop_value *props;
    uint8_t *data;
    size_t size;
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

/* LVGL task only. Takes ownership of `data`; RGBA is reordered to LVGL's BGRA. */
static void load_image(int id, int format, int w, int h, uint8_t *data, size_t size)
{
    size_t bpp = format == FmtA8 ? 1 : 4;
    if (id < 0 || id >= MAX_IMAGES || images[id] != NULL || size < (size_t) w * h * bpp) {
        heap_caps_free(data);
        return;
    }
    if (format == FmtRgba8888) {
        for (size_t i = 0; i < (size_t) w * h * 4; i += 4) {
            uint8_t r = data[i];
            data[i] = data[i + 2];
            data[i + 2] = r;
        }
    }
    lv_image_dsc_t *dsc = heap_caps_calloc(1, sizeof(lv_image_dsc_t), PSRAM_CAPS);
    dsc->header.magic = LV_IMAGE_HEADER_MAGIC;
    dsc->header.cf = format == FmtA8 ? LV_COLOR_FORMAT_A8 : LV_COLOR_FORMAT_ARGB8888;
    dsc->header.w = w;
    dsc->header.h = h;
    dsc->header.stride = w * bpp;
    dsc->data_size = (uint32_t) w * h * bpp;
    dsc->data = data;
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

static void delete_obj(int id)
{
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
            lv_obj_set_x(obj, v);
            break;
        case PropY:
            lv_obj_set_y(obj, v);
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
            }
            break;
        case PropFont:
            if (v >= 0 && v < MAX_FONTS && fonts[v] != NULL) {
                lv_obj_set_style_text_font(obj, &fonts[v]->font, 0);
            }
            break;
        case PropSrc:
            if (type == TypeImage && v >= 0 && v < MAX_IMAGES && images[v] != NULL) {
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
        case PropSpeed:
            /* Pixels a second, however long the text. */
            lv_obj_set_style_anim_duration(obj, lv_anim_speed(v > 0 ? v : 40), 0);
            break;
        default:
            break;
    }
}

static void apply_op(struct op *op)
{
    switch (op->kind) {
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
            } else {
                dropped_ops++;
            }
            break;
        case OpImg:
            load_image(op->id, op->format, op->w, op->h, op->data, op->size);
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
    }
    heap_caps_free(b->ops);
    heap_caps_free(b);
}

/* Runs inside the LVGL task, so it may touch LVGL. */
static void drain(lv_timer_t *timer)
{
    UNUSED(timer);
    struct batch *b;
    while (xQueueReceive(queue, &b, 0) == pdTRUE) {
        for (size_t i = 0; i < b->count; i++) {
            apply_op(&b->ops[i]);
        }
        free_batch(b);
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
        } else if (term_is_integer(value)) {
            p->value = term_to_int(value);
        } else {
            p->value = value == TRUE_ATOM ? 1 : (value == FALSE_ATOM ? 0 : -1);
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
            if (term_get_tuple_arity(t) != 6) {
                return false;
            }
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
