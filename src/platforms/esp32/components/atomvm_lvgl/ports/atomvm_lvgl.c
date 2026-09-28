/*
 * SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
 *
 * LVGL display port driver: a go/no-go spike.
 *
 * The port's mailbox handler runs on a VM scheduler thread and never calls
 * LVGL. It copies each command onto a FreeRTOS queue; an LVGL timer, running
 * inside esp_lvgl_port's task, drains the queue. LVGL is not thread safe, so
 * that task is the only place it is ever called from.
 *
 * Commands (all port calls, answered once queued):
 *   {label, Id, X, Y, Text, RGB, Font}     Font 0 = 14px, 1 = 28px
 *   {marquee, Id, X, Y, W, Text, RGB, Font} a circular-scrolling label
 *   {text, Id, Text}
 *   {bg, RGB}
 *   clear
 *   {bench, Frames}                         full-screen repaints, timed, logged
 *   stats                                   -> {InternalFree, DmaFree, InternalLargest, PsramFree, Refreshes}
 */

#include <sdkconfig.h>

#include <stdlib.h>
#include <string.h>

#include <driver/gpio.h>
#include <driver/spi_master.h>
#include <esp_heap_caps.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>
#include <esp_log.h>
#include <esp_lvgl_port.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>
#include <freertos/queue.h>
#include <lvgl.h>

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
#define MAX_OBJS 32
#define QUEUE_DEPTH 64
#define DRAW_ROWS 12

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

enum lvgl_cmd
{
    LvglInvalidCmd = 0,
    LvglLabelCmd,
    LvglMarqueeCmd,
    LvglTextCmd,
    LvglBgCmd,
    LvglClearCmd,
    LvglBenchCmd,
    LvglStatsCmd
};

static const AtomStringIntPair cmd_table[] = {
    { ATOM_STR("\x5", "label"), LvglLabelCmd },
    { ATOM_STR("\x7", "marquee"), LvglMarqueeCmd },
    { ATOM_STR("\x4", "text"), LvglTextCmd },
    { ATOM_STR("\x2", "bg"), LvglBgCmd },
    { ATOM_STR("\x5", "clear"), LvglClearCmd },
    { ATOM_STR("\x5", "bench"), LvglBenchCmd },
    { ATOM_STR("\x5", "stats"), LvglStatsCmd },
    SELECT_INT_DEFAULT(LvglInvalidCmd)
};

/* A queued command; text is malloc'd here and freed by the LVGL task. */
struct command
{
    enum lvgl_cmd kind;
    int id;
    int x;
    int y;
    int w;
    uint32_t rgb;
    int font;
    int frames;
    char *text;
};

static QueueHandle_t queue;
static lv_display_t *display;
static lv_obj_t *objs[MAX_OBJS];
static volatile uint32_t refreshes;
static bool started;

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

static int int_at(term tuple, int index)
{
    term value = term_get_tuple_element(tuple, index);
    return term_is_integer(value) ? term_to_int(value) : 0;
}

static char *text_at(term tuple, int index)
{
    int ok;
    char *text = interop_term_to_string(term_get_tuple_element(tuple, index), &ok);
    return ok ? text : NULL;
}

static const lv_font_t *font_for(int font)
{
    return font == 1 ? &lv_font_montserrat_28 : &lv_font_montserrat_14;
}

/* LVGL task only. */
static lv_obj_t *label_for(int id)
{
    if (id < 0 || id >= MAX_OBJS) {
        return NULL;
    }
    if (objs[id] == NULL) {
        objs[id] = lv_label_create(lv_screen_active());
    }
    return objs[id];
}

/* LVGL task only: repaints the whole screen `frames` times and logs the rate. */
static void bench(int frames)
{
    lv_obj_t *screen = lv_screen_active();
    int64_t started_us = esp_timer_get_time();

    for (int i = 0; i < frames; i++) {
        lv_obj_set_style_bg_color(screen, lv_color_hex(i % 2 ? 0x16122A : 0x0D0A18), 0);
        lv_obj_invalidate(screen);
        lv_refr_now(display);
    }

    int64_t elapsed_us = esp_timer_get_time() - started_us;
    ESP_LOGI(TAG, "bench: %d full frames in %lld us, %lld us/frame, %lld.%01lld fps", frames,
        elapsed_us, elapsed_us / frames, (1000000LL * frames) / elapsed_us,
        ((10000000LL * frames) / elapsed_us) % 10);
}

/* LVGL task only. */
static void apply(struct command *cmd)
{
    lv_obj_t *obj;

    switch (cmd->kind) {
        case LvglLabelCmd:
        case LvglMarqueeCmd:
            obj = label_for(cmd->id);
            if (obj == NULL) {
                break;
            }
            lv_obj_set_pos(obj, cmd->x, cmd->y);
            lv_obj_set_style_text_color(obj, lv_color_hex(cmd->rgb), 0);
            lv_obj_set_style_text_font(obj, font_for(cmd->font), 0);
            if (cmd->kind == LvglMarqueeCmd) {
                lv_obj_set_width(obj, cmd->w);
                lv_label_set_long_mode(obj, LV_LABEL_LONG_SCROLL_CIRCULAR);
            }
            lv_label_set_text(obj, cmd->text ? cmd->text : "");
            break;

        case LvglTextCmd:
            obj = label_for(cmd->id);
            if (obj != NULL) {
                lv_label_set_text(obj, cmd->text ? cmd->text : "");
            }
            break;

        case LvglBgCmd:
            lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(cmd->rgb), 0);
            lv_obj_set_style_bg_opa(lv_screen_active(), LV_OPA_COVER, 0);
            break;

        case LvglClearCmd:
            lv_obj_clean(lv_screen_active());
            memset(objs, 0, sizeof(objs));
            break;

        case LvglBenchCmd:
            bench(cmd->frames > 0 ? cmd->frames : 30);
            break;

        default:
            break;
    }

    free(cmd->text);
}

/* Runs inside the LVGL task, so it may touch LVGL. */
static void drain(lv_timer_t *timer)
{
    UNUSED(timer);
    struct command cmd;
    while (xQueueReceive(queue, &cmd, 0) == pdTRUE) {
        apply(&cmd);
    }
}

static void count_refresh(lv_event_t *event)
{
    UNUSED(event);
    refreshes++;
}

static void report(lv_timer_t *timer)
{
    UNUSED(timer);
    static uint32_t last;
    uint32_t now = refreshes;
    ESP_LOGI(TAG, "stats: %lu refreshes/5s internal_free=%u dma_free=%u internal_largest=%u psram_free=%u",
        (unsigned long) (now - last), heap_caps_get_free_size(MALLOC_CAP_INTERNAL),
        heap_caps_get_free_size(MALLOC_CAP_DMA),
        heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL),
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM));
    last = now;
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

/* VM scheduler thread: parse and queue, never call LVGL. */
static term enqueue(term req, int kind)
{
    struct command cmd = { .kind = kind };

    switch (kind) {
        case LvglLabelCmd:
            cmd.id = int_at(req, 1);
            cmd.x = int_at(req, 2);
            cmd.y = int_at(req, 3);
            cmd.text = text_at(req, 4);
            cmd.rgb = (uint32_t) int_at(req, 5);
            cmd.font = int_at(req, 6);
            break;
        case LvglMarqueeCmd:
            cmd.id = int_at(req, 1);
            cmd.x = int_at(req, 2);
            cmd.y = int_at(req, 3);
            cmd.w = int_at(req, 4);
            cmd.text = text_at(req, 5);
            cmd.rgb = (uint32_t) int_at(req, 6);
            cmd.font = int_at(req, 7);
            break;
        case LvglTextCmd:
            cmd.id = int_at(req, 1);
            cmd.text = text_at(req, 2);
            break;
        case LvglBgCmd:
            cmd.rgb = (uint32_t) int_at(req, 1);
            break;
        case LvglBenchCmd:
            cmd.frames = int_at(req, 1);
            break;
        default:
            break;
    }

    if (xQueueSend(queue, &cmd, 0) != pdTRUE) {
        free(cmd.text);
        return ERROR_ATOM;
    }
    return OK_ATOM;
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
    term cmd_term = term_is_tuple(req) ? term_get_tuple_element(req, 0) : req;
    int kind = interop_atom_term_select_int(cmd_table, cmd_term, global);

    if (kind == LvglStatsCmd) {
        port_ensure_available(ctx, PORT_REPLY_SIZE + TUPLE_SIZE(5));
        port_send_reply(ctx, gen_message.pid, gen_message.ref, stats_term(ctx));
    } else {
        term reply = kind == LvglInvalidCmd ? BADARG_ATOM : enqueue(req, kind);
        port_ensure_available(ctx, PORT_REPLY_SIZE);
        port_send_reply(ctx, gen_message.pid, gen_message.ref, reply);
    }

    mailbox_remove_message(&ctx->mailbox, &ctx->heap);
    return NativeContinue;
}

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

    queue = xQueueCreate(QUEUE_DEPTH, sizeof(struct command));

    lvgl_port_lock(0);
    lv_obj_set_style_bg_color(lv_screen_active(), lv_color_hex(0x16122A), 0);
    lv_display_add_event_cb(display, count_refresh, LV_EVENT_REFR_READY, NULL);
    lv_timer_create(drain, 10, NULL);
    lv_timer_create(report, 5000, NULL);
    lvgl_port_unlock();

    ESP_LOGI(TAG, "started %dx%d, internal_free=%u dma_free=%u", width, height,
        heap_caps_get_free_size(MALLOC_CAP_INTERNAL), heap_caps_get_free_size(MALLOC_CAP_DMA));
    return true;
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
    if (!start_display(opts, global)) {
        return NULL;
    }
    started = true;

    Context *ctx = context_new(global);
    ctx->native_handler = consume_mailbox;
    return ctx;
}

REGISTER_PORT_DRIVER(lvgl, atomvm_lvgl_init, NULL, atomvm_lvgl_create_port)
