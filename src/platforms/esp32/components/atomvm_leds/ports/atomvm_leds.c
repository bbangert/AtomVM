/*
 * This file is part of AtomVM.
 *
 * SPDX-License-Identifier: Apache-2.0 OR LGPL-2.1-or-later
 */

/*
 * `leds` port driver: a short SK6812 / WS2812 chain animated in C.
 *
 * Opened with open_port({spawn, "leds"}, [{pin, Gpio}, {count, N}]). The chain is
 * driven by RMT and repainted 100 times a second from an esp_timer, so effects
 * run smoothly whatever the VM is doing. Colours are gamma corrected and
 * temporally dithered, so slow fades at low brightness do not step.
 *
 * Requests, each answered ok or badarg:
 *   {effect, Name, [RGB], [{Key, Int}]}  runs an effect; changing only its options
 *                                        keeps it going from where it was. Keys:
 *       period     ms per cycle                                  (default 2000)
 *       brightness 0..100, perceptual                            (40)
 *       spread     0..100, how far apart neighbours are, in %    (25)
 *       direction  0 clockwise, 1 anticlockwise                  (0)
 *       trail      0..100, how long lit LEDs take to fade        (50)
 *       density    0..100, how often random things happen        (30)
 *   {flash, RGB, Ms}                     lights the whole ring over the effect, fading out
 *   {raw, [RGB]}                         shows these colours as given, in chain order
 *   {order, [Index]}                     chain index of each ring position, clockwise
 *
 * Effects: off, solid, breathe, drift, rainbow, chase, bounce, twinkle,
 * heartbeat, candle, aurora, strobe, glitch. Ring positions run clockwise, so
 * chase and bounce travel round the corners; strobe alternates the first and
 * second half of the ring.
 */

#include <sdkconfig.h>

#ifdef CONFIG_AVM_LEDS_ENABLE

#include <math.h>
#include <stdlib.h>
#include <string.h>

#include <driver/rmt_tx.h>
#include <esp_log.h>
#include <esp_random.h>
#include <esp_timer.h>
#include <freertos/FreeRTOS.h>

#include <atom.h>
#include <context.h>
#include <defaultatoms.h>
#include <globalcontext.h>
#include <interop.h>
#include <mailbox.h>
#include <port.h>
#include <portnifloader.h>
#include <term.h>
#include <utils.h>

#include <esp32_sys.h>

#define TAG "atomvm_leds"

/* Not exported by libAtomVM; the in-tree drivers each define it. */
#define PORT_REPLY_SIZE (TUPLE_SIZE(2) + REF_SIZE)

#define MAX_LEDS 16
#define MAX_COLOURS 8
#define FRAME_US 10000
#define RMT_HZ 10000000
#define GAMMA 2.2f
#define TAU 6.2831853f

enum effect
{
    EffectOff = 0,
    EffectSolid,
    EffectBreathe,
    EffectDrift,
    EffectRainbow,
    EffectChase,
    EffectBounce,
    EffectTwinkle,
    EffectHeartbeat,
    EffectCandle,
    EffectAurora,
    EffectStrobe,
    EffectGlitch,
    EffectRaw
};

static const AtomStringIntPair effect_table[] = {
    { ATOM_STR("\x3", "off"), EffectOff },
    { ATOM_STR("\x5", "solid"), EffectSolid },
    { ATOM_STR("\x7", "breathe"), EffectBreathe },
    { ATOM_STR("\x5", "drift"), EffectDrift },
    { ATOM_STR("\x7", "rainbow"), EffectRainbow },
    { ATOM_STR("\x5", "chase"), EffectChase },
    { ATOM_STR("\x6", "bounce"), EffectBounce },
    { ATOM_STR("\x7", "twinkle"), EffectTwinkle },
    { ATOM_STR("\x9", "heartbeat"), EffectHeartbeat },
    { ATOM_STR("\x6", "candle"), EffectCandle },
    { ATOM_STR("\x6", "aurora"), EffectAurora },
    { ATOM_STR("\x6", "strobe"), EffectStrobe },
    { ATOM_STR("\x6", "glitch"), EffectGlitch },
    SELECT_INT_DEFAULT(-1)
};

enum request
{
    ReqEffect,
    ReqFlash,
    ReqRaw,
    ReqOrder
};

static const AtomStringIntPair request_table[] = {
    { ATOM_STR("\x6", "effect"), ReqEffect },
    { ATOM_STR("\x5", "flash"), ReqFlash },
    { ATOM_STR("\x3", "raw"), ReqRaw },
    { ATOM_STR("\x5", "order"), ReqOrder },
    SELECT_INT_DEFAULT(-1)
};

struct rgb
{
    float r;
    float g;
    float b;
};

struct settings
{
    int effect;
    int count;
    struct rgb colours[MAX_COLOURS];
    uint32_t period;
    float brightness;
    float spread;
    bool anticlockwise;
    float trail;
    float density;
    uint32_t raw[MAX_LEDS];
};

/* Written by the VM thread under `lock`, taken up by the timer on its next frame. */
static struct settings wanted;
static bool wanted_fresh;
static struct rgb flash_wanted;
static uint32_t flash_wanted_ms;
static bool flash_fresh;
static uint8_t order_wanted[MAX_LEDS];
static portMUX_TYPE lock = portMUX_INITIALIZER_UNLOCKED;

/* Owned by the timer. */
static struct settings now;
static double phase;
static int64_t last_us;
static struct rgb lit[MAX_LEDS];
static float wander[MAX_LEDS];
static float wander_to[MAX_LEDS];
static float dither[MAX_LEDS][3];
static struct rgb flash_colour;
static float flash_left;
static float flash_total;
static uint8_t order[MAX_LEDS];
static uint8_t frame[MAX_LEDS * 3];

static int led_count;
static bool started;
static rmt_channel_handle_t channel;
static rmt_encoder_handle_t encoder;
static esp_timer_handle_t timer;

static const struct rgb black = { 0, 0, 0 };

static float random_unit(void)
{
    return (float) (esp_random() & 0xFFFF) / 65535.0f;
}

static struct rgb from_int(uint32_t rgb)
{
    struct rgb c = { ((rgb >> 16) & 0xFF) / 255.0f, ((rgb >> 8) & 0xFF) / 255.0f, (rgb & 0xFF) / 255.0f };
    return c;
}

static struct rgb scaled(struct rgb c, float k)
{
    struct rgb out = { c.r * k, c.g * k, c.b * k };
    return out;
}

static struct rgb mixed(struct rgb a, struct rgb b, float t)
{
    struct rgb out = { a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t };
    return out;
}

static struct rgb brighter(struct rgb a, struct rgb b)
{
    struct rgb out = { fmaxf(a.r, b.r), fmaxf(a.g, b.g), fmaxf(a.b, b.b) };
    return out;
}

static float wrap_unit(float x)
{
    return x - floorf(x);
}

static float clamp_unit(float x)
{
    return x < 0 ? 0 : (x > 1 ? 1 : x);
}

/* The palette as a loop: 0..1 runs smoothly through every colour and back to the first. */
static struct rgb palette_at(float t)
{
    if (now.count == 0) {
        return black;
    }
    if (now.count == 1) {
        return now.colours[0];
    }
    float at = wrap_unit(t) * now.count;
    int i = (int) at;
    return mixed(now.colours[i % now.count], now.colours[(i + 1) % now.count], at - i);
}

/* One palette colour, stepping rather than blending. */
static struct rgb palette_nth(long n)
{
    if (now.count == 0) {
        return black;
    }
    long i = n % now.count;
    return now.colours[i < 0 ? i + now.count : i];
}

static struct rgb hue(float t)
{
    float h = wrap_unit(t) * 6.0f;
    float x = 1.0f - fabsf(fmodf(h, 2.0f) - 1.0f);
    struct rgb c = black;
    switch ((int) h) {
        case 0:
            c.r = 1;
            c.g = x;
            break;
        case 1:
            c.r = x;
            c.g = 1;
            break;
        case 2:
            c.g = 1;
            c.b = x;
            break;
        case 3:
            c.g = x;
            c.b = 1;
            break;
        case 4:
            c.r = x;
            c.b = 1;
            break;
        default:
            c.r = 1;
            c.b = x;
            break;
    }
    return c;
}

/* Where ring position `i` sits for an effect, counted in its direction of travel. */
static int along(int i)
{
    return now.anticlockwise ? (led_count - i) % led_count : i;
}

/* Lit LEDs fade by the trail: 0 is gone in 50 ms, 100 lingers about a second and a half. */
static void decay(float dt)
{
    float keep = powf(0.02f, dt / (0.05f + now.trail * 1.5f));
    for (int i = 0; i < led_count; i++) {
        lit[i] = scaled(lit[i], keep);
    }
}

static float heartbeat(float t)
{
    float a = expf(-powf((t - 0.10f) / 0.035f, 2));
    float b = 0.7f * expf(-powf((t - 0.30f) / 0.045f, 2));
    return fmaxf(a, b);
}

/* Each LED's wander value creeps toward a random target, and picks a new one on arrival. */
static void wander_step(float rate)
{
    for (int i = 0; i < led_count; i++) {
        float gap = wander_to[i] - wander[i];
        if (fabsf(gap) < 0.02f) {
            wander_to[i] = random_unit();
        } else {
            wander[i] += gap * clamp_unit(rate);
        }
    }
}

static void render(struct rgb *out, float dt)
{
    float period = now.period > 0 ? now.period / 1000.0f : 2.0f;
    phase += dt / period;
    float cycle = (float) phase;
    float step = now.spread / (float) led_count;
    long lap = (long) floor(phase);

    for (int i = 0; i < led_count; i++) {
        out[i] = black;
    }

    switch (now.effect) {
        case EffectSolid:
            for (int i = 0; i < led_count; i++) {
                out[i] = palette_at(along(i) * step);
            }
            break;

        case EffectBreathe:
            for (int i = 0; i < led_count; i++) {
                float c = cycle - along(i) * step;
                float level = 0.5f - 0.5f * cosf(TAU * wrap_unit(c));
                out[i] = scaled(palette_nth((long) floorf(c)), 0.03f + 0.97f * level);
            }
            break;

        case EffectDrift:
            for (int i = 0; i < led_count; i++) {
                out[i] = palette_at(cycle + along(i) * step);
            }
            break;

        case EffectRainbow:
            for (int i = 0; i < led_count; i++) {
                out[i] = hue(cycle + along(i) * step);
            }
            break;

        case EffectChase: {
            /* A head goes round once a period; the LED it approaches brightens as it nears. */
            float head = wrap_unit(cycle) * led_count;
            float reach = 0.15f + now.trail * 2.0f;
            struct rgb colour = palette_nth(lap);
            for (int i = 0; i < led_count; i++) {
                float back = fmodf(head - along(i) + led_count, (float) led_count);
                float ahead = led_count - back;
                float level = fmaxf(expf(-back / reach), ahead < 1 ? 1 - ahead : 0);
                out[i] = scaled(colour, level);
            }
            break;
        }

        case EffectBounce: {
            float t = wrap_unit(cycle);
            float head = (t < 0.5f ? t * 2 : 2 - t * 2) * (led_count - 1);
            struct rgb colour = palette_nth(lap);
            decay(dt);
            for (int i = 0; i < led_count; i++) {
                float near = clamp_unit(1 - fabsf(along(i) - head));
                lit[i] = brighter(lit[i], scaled(colour, near));
                out[i] = lit[i];
            }
            break;
        }

        case EffectTwinkle: {
            float chance = (0.2f + now.density * 6.0f) * dt / period;
            decay(dt);
            for (int i = 0; i < led_count; i++) {
                if (random_unit() < chance) {
                    lit[i] = palette_nth(esp_random() % (now.count > 0 ? now.count : 1));
                }
                out[i] = lit[i];
            }
            break;
        }

        case EffectHeartbeat:
            for (int i = 0; i < led_count; i++) {
                float c = cycle - along(i) * step * 0.25f;
                float level = fmaxf(heartbeat(wrap_unit(c)), 0.04f);
                out[i] = scaled(palette_at(along(i) * step), level);
            }
            break;

        case EffectCandle:
            /* Density sets how deep the flicker dips; the palette runs from calm to guttering. */
            wander_step(dt * 12.0f / period);
            for (int i = 0; i < led_count; i++) {
                float dip = wander[i] * wander[i];
                float level = 1 - (0.2f + now.density * 0.7f) * dip;
                out[i] = scaled(palette_at(dip * 0.5f), level);
            }
            break;

        case EffectAurora:
            wander_step(dt * 1.5f / period);
            for (int i = 0; i < led_count; i++) {
                float glow = 0.55f + 0.45f * sinf(TAU * (cycle * 0.5f + wander[i]));
                out[i] = scaled(palette_at(wander[i] + along(i) * step), glow);
            }
            break;

        case EffectStrobe: {
            float half = wrap_unit(cycle * 2) * period * 0.5f;
            bool on = half < 0.04f || (half > 0.10f && half < 0.14f);
            bool first = wrap_unit(cycle) < 0.5f;
            for (int i = 0; i < led_count; i++) {
                bool side = along(i) < led_count / 2;
                if (on && side == first) {
                    out[i] = palette_nth(first ? 0 : 1);
                }
            }
            break;
        }

        case EffectGlitch: {
            float chance = (0.5f + now.density * 12.0f) * dt;
            float keep = powf(0.02f, dt / 0.06f);
            struct rgb base = scaled(palette_nth(0), 0.25f);
            for (int i = 0; i < led_count; i++) {
                lit[i] = scaled(lit[i], keep);
                if (random_unit() < chance / led_count) {
                    lit[i] = palette_nth(esp_random() % (now.count > 0 ? now.count : 1));
                }
                out[i] = brighter(base, lit[i]);
            }
            break;
        }

        default:
            break;
    }

    if (flash_left > 0) {
        float level = flash_left / flash_total;
        for (int i = 0; i < led_count; i++) {
            out[i] = mixed(out[i], flash_colour, level * level);
        }
        flash_left -= dt;
    }
}

/* Perceptual colour to chain bytes: gamma, brightness, then dithering carries the remainder. */
static void to_frame(const struct rgb *out)
{
    float cap = now.brightness;
    for (int i = 0; i < led_count; i++) {
        float channels[3] = { out[i].g, out[i].r, out[i].b };
        int at = order[i] * 3;
        for (int c = 0; c < 3; c++) {
            float want = powf(clamp_unit(channels[c]) * cap, GAMMA) * 255.0f + dither[i][c];
            float byte = floorf(want + 0.5f);
            byte = byte < 0 ? 0 : (byte > 255 ? 255 : byte);
            dither[i][c] = want - byte;
            frame[at + c] = (uint8_t) byte;
        }
    }
}

static void raw_frame(void)
{
    for (int i = 0; i < led_count; i++) {
        uint32_t rgb = now.raw[i];
        frame[i * 3] = (rgb >> 8) & 0xFF;
        frame[i * 3 + 1] = (rgb >> 16) & 0xFF;
        frame[i * 3 + 2] = rgb & 0xFF;
    }
}

static void take_wanted(void)
{
    portENTER_CRITICAL(&lock);
    if (wanted_fresh) {
        bool same = wanted.effect == now.effect && wanted.count == now.count
            && memcmp(wanted.colours, now.colours, sizeof(now.colours)) == 0;
        now = wanted;
        wanted_fresh = false;
        if (!same) {
            phase = 0;
            memset(lit, 0, sizeof(lit));
            for (int i = 0; i < led_count; i++) {
                wander[i] = random_unit();
                wander_to[i] = random_unit();
            }
        }
    }
    if (flash_fresh) {
        flash_colour = flash_wanted;
        flash_total = flash_left = flash_wanted_ms / 1000.0f;
        flash_fresh = false;
    }
    memcpy(order, order_wanted, sizeof(order));
    portEXIT_CRITICAL(&lock);
}

static void frame_tick(void *arg)
{
    UNUSED(arg);
    int64_t t = esp_timer_get_time();
    float dt = (t - last_us) / 1e6f;
    last_us = t;
    if (dt <= 0 || dt > 0.1f) {
        dt = FRAME_US / 1e6f;
    }

    take_wanted();

    /* The previous frame is still going out; this one is skipped rather than queued. */
    if (rmt_tx_wait_all_done(channel, 0) != ESP_OK) {
        return;
    }

    if (now.effect == EffectRaw) {
        raw_frame();
    } else {
        struct rgb out[MAX_LEDS];
        render(out, dt);
        to_frame(out);
    }

    rmt_transmit_config_t tx = { .loop_count = 0 };
    rmt_transmit(channel, encoder, frame, led_count * 3, &tx);
}

static bool start_chain(int pin, int count)
{
    rmt_tx_channel_config_t config = {
        .gpio_num = pin,
        .clk_src = RMT_CLK_SRC_DEFAULT,
        .resolution_hz = RMT_HZ,
        .mem_block_symbols = 48,
        .trans_queue_depth = 2,
    };
    if (rmt_new_tx_channel(&config, &channel) != ESP_OK) {
        ESP_LOGE(TAG, "no RMT channel for GPIO %d", pin);
        return false;
    }

    /* SK6812: 0 is 0.3 us high then 0.9 us low, 1 is 0.6 us each, at 10 MHz ticks. */
    rmt_bytes_encoder_config_t bits = {
        .bit0 = { .level0 = 1, .duration0 = 3, .level1 = 0, .duration1 = 9 },
        .bit1 = { .level0 = 1, .duration0 = 6, .level1 = 0, .duration1 = 6 },
        .flags.msb_first = 1,
    };
    if (rmt_new_bytes_encoder(&bits, &encoder) != ESP_OK || rmt_enable(channel) != ESP_OK) {
        ESP_LOGE(TAG, "RMT setup failed");
        return false;
    }

    led_count = count;
    for (int i = 0; i < MAX_LEDS; i++) {
        order[i] = order_wanted[i] = i;
    }
    now.effect = EffectOff;
    now.brightness = 0.4f;
    last_us = esp_timer_get_time();

    esp_timer_create_args_t args = { .callback = frame_tick, .name = "leds" };
    if (esp_timer_create(&args, &timer) != ESP_OK || esp_timer_start_periodic(timer, FRAME_US) != ESP_OK) {
        ESP_LOGE(TAG, "no frame timer");
        return false;
    }

    ESP_LOGI(TAG, "%d LEDs on GPIO %d", count, pin);
    return true;
}

/* ---------------------------------------------------------------------------
 * Requests, on the VM thread
 * ------------------------------------------------------------------------- */

static int kv_int(term kv, AtomString key, int fallback, GlobalContext *global)
{
    term value = interop_kv_get_value(kv, key, global);
    return term_is_integer(value) ? term_to_int(value) : fallback;
}

static float percent(int value)
{
    return (value < 0 ? 0 : (value > 100 ? 100 : value)) / 100.0f;
}

static int parse_colours(term list, struct rgb *out)
{
    int n = 0;
    while (term_is_nonempty_list(list) && n < MAX_COLOURS) {
        term c = term_get_list_head(list);
        if (term_is_integer(c)) {
            out[n++] = from_int((uint32_t) term_to_int(c));
        }
        list = term_get_list_tail(list);
    }
    return n;
}

static term effect_request(term req, GlobalContext *global)
{
    if (!term_is_tuple(req) || term_get_tuple_arity(req) != 4) {
        return BADARG_ATOM;
    }
    int effect = interop_atom_term_select_int(effect_table, term_get_tuple_element(req, 1), global);
    term opts = term_get_tuple_element(req, 3);
    if (effect < 0 || !term_is_list(term_get_tuple_element(req, 2)) || !term_is_list(opts)) {
        return BADARG_ATOM;
    }

    struct settings s;
    memset(&s, 0, sizeof(s));
    s.effect = effect;
    s.count = parse_colours(term_get_tuple_element(req, 2), s.colours);
    s.period = kv_int(opts, ATOM_STR("\x6", "period"), 2000, global);
    s.brightness = percent(kv_int(opts, ATOM_STR("\xA", "brightness"), 40, global));
    s.spread = percent(kv_int(opts, ATOM_STR("\x6", "spread"), 25, global));
    s.anticlockwise = kv_int(opts, ATOM_STR("\x9", "direction"), 0, global) == 1;
    s.trail = percent(kv_int(opts, ATOM_STR("\x5", "trail"), 50, global));
    s.density = percent(kv_int(opts, ATOM_STR("\x7", "density"), 30, global));

    portENTER_CRITICAL(&lock);
    wanted = s;
    wanted_fresh = true;
    portEXIT_CRITICAL(&lock);
    return OK_ATOM;
}

static term flash_request(term req)
{
    if (!term_is_tuple(req) || term_get_tuple_arity(req) != 3 || !term_is_integer(term_get_tuple_element(req, 1))
        || !term_is_integer(term_get_tuple_element(req, 2))) {
        return BADARG_ATOM;
    }
    int ms = term_to_int(term_get_tuple_element(req, 2));
    portENTER_CRITICAL(&lock);
    flash_wanted = from_int((uint32_t) term_to_int(term_get_tuple_element(req, 1)));
    flash_wanted_ms = ms > 0 ? ms : 1;
    flash_fresh = true;
    portEXIT_CRITICAL(&lock);
    return OK_ATOM;
}

static term raw_request(term req)
{
    if (!term_is_tuple(req) || term_get_tuple_arity(req) != 2 || !term_is_list(term_get_tuple_element(req, 1))) {
        return BADARG_ATOM;
    }
    struct settings s;
    memset(&s, 0, sizeof(s));
    s.effect = EffectRaw;
    term list = term_get_tuple_element(req, 1);
    for (int i = 0; i < MAX_LEDS && term_is_nonempty_list(list); i++) {
        term c = term_get_list_head(list);
        s.raw[i] = term_is_integer(c) ? (uint32_t) term_to_int(c) : 0;
        list = term_get_list_tail(list);
    }
    portENTER_CRITICAL(&lock);
    wanted = s;
    wanted_fresh = true;
    portEXIT_CRITICAL(&lock);
    return OK_ATOM;
}

static term order_request(term req)
{
    if (!term_is_tuple(req) || term_get_tuple_arity(req) != 2 || !term_is_list(term_get_tuple_element(req, 1))) {
        return BADARG_ATOM;
    }
    uint8_t next[MAX_LEDS];
    term list = term_get_tuple_element(req, 1);
    for (int i = 0; i < MAX_LEDS; i++) {
        next[i] = i;
    }
    for (int i = 0; i < led_count && term_is_nonempty_list(list); i++) {
        term c = term_get_list_head(list);
        if (!term_is_integer(c) || term_to_int(c) < 0 || term_to_int(c) >= led_count) {
            return BADARG_ATOM;
        }
        next[i] = term_to_int(c);
        list = term_get_list_tail(list);
    }
    portENTER_CRITICAL(&lock);
    memcpy(order_wanted, next, sizeof(order_wanted));
    portEXIT_CRITICAL(&lock);
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
    term kind_term = term_is_tuple(req) && term_get_tuple_arity(req) > 0 ? term_get_tuple_element(req, 0) : req;
    term reply;
    switch (interop_atom_term_select_int(request_table, kind_term, global)) {
        case ReqEffect:
            reply = effect_request(req, global);
            break;
        case ReqFlash:
            reply = flash_request(req);
            break;
        case ReqRaw:
            reply = raw_request(req);
            break;
        case ReqOrder:
            reply = order_request(req);
            break;
        default:
            reply = BADARG_ATOM;
            break;
    }

    port_ensure_available(ctx, PORT_REPLY_SIZE);
    port_send_reply(ctx, gen_message.pid, gen_message.ref, reply);
    mailbox_remove_message(&ctx->mailbox, &ctx->heap);
    return NativeContinue;
}

void atomvm_leds_init(GlobalContext *global)
{
    UNUSED(global);
}

Context *atomvm_leds_create_port(GlobalContext *global, term opts)
{
    if (started) {
        ESP_LOGE(TAG, "Only one LED chain is supported");
        return NULL;
    }
    int pin = kv_int(opts, ATOM_STR("\x3", "pin"), -1, global);
    int count = kv_int(opts, ATOM_STR("\x5", "count"), 0, global);
    if (pin < 0 || count <= 0 || count > MAX_LEDS || !start_chain(pin, count)) {
        return NULL;
    }
    started = true;

    Context *ctx = context_new(global);
    ctx->native_handler = consume_mailbox;
    return ctx;
}

REGISTER_PORT_DRIVER(leds, atomvm_leds_init, NULL, atomvm_leds_create_port)

#endif
