#include "dashboard.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <cstdio>

// Widget handles (created once, updated each poll).
static lv_obj_t* s_dot        = nullptr;  // live heartbeat dot
static lv_obj_t* s_card       = nullptr;  // hero state card
static lv_obj_t* s_state      = nullptr;  // big SAFE/ARMED word
static lv_obj_t* s_link       = nullptr;  // wifi icon + rssi
static lv_obj_t* s_fired      = nullptr;  // "FIRED n/16"
static lv_obj_t* s_last       = nullptr;  // "LAST ch N"
static lv_obj_t* s_bar        = nullptr;  // fired progress bar
static lv_obj_t* s_seq        = nullptr;  // sequence status
static lv_obj_t* s_cellBox[16] = {nullptr};
static lv_obj_t* s_cellNum[16] = {nullptr};

// Palette (dark theme).
#define C_BG         lv_color_hex(0x0a0e1a)
#define C_MUTED      lv_color_hex(0x7888a4)
#define C_SAFE       lv_color_hex(0x18b85c)
#define C_SAFE2      lv_color_hex(0x0a7034)
#define C_ARMED      lv_color_hex(0xe62424)
#define C_ARMED2     lv_color_hex(0x870f0f)
#define C_GREY       lv_color_hex(0x4a5468)
#define C_GREY2      lv_color_hex(0x252c3c)
#define C_AMBER      lv_color_hex(0xe09000)
#define C_AMBER2     lv_color_hex(0x8a5800)
#define C_LINKOK     lv_color_hex(0x46d27a)
#define C_LINKWARN   lv_color_hex(0xe5bb33)
#define C_LINKBAD    lv_color_hex(0xe85555)
#define C_CELLOFF    lv_color_hex(0x161e30)
#define C_CELLOFF_BD lv_color_hex(0x2c3852)
#define C_NUMOFF     lv_color_hex(0x5a6783)
#define C_FIRED      lv_color_hex(0x24d268)
#define C_FIRED_BD   lv_color_hex(0x5bf09a)
#define C_FLASH      lv_color_hex(0xffffff)

// Shared state between dashboard_update (control task, under port lock) and the
// pulse timer (LVGL task, runs only while the port lock is held by the LVGL
// task) — the port mutex serializes the two, so plain statics are safe.
enum PulseMode { PULSE_NONE, PULSE_ARMED, PULSE_ALARM };
static volatile PulseMode g_pulse = PULSE_NONE;
static uint16_t g_fired = 0;          // last-seen fired bitmap
static int      g_flash[16] = {0};    // per-cell "just fired" flash ticks
static uint32_t g_tick = 0;
static const int FLASH_TICKS = 6;     // ~0.7s of white flash (timer @ 120ms)

static int popcount16(uint16_t v) {
    int n = 0;
    for (int i = 0; i < 16; ++i) if ((v >> i) & 1) ++n;
    return n;
}

static void render_cell_settled(int i, bool fired) {
    lv_obj_set_style_bg_color(s_cellBox[i], fired ? C_FIRED : C_CELLOFF, 0);
    lv_obj_set_style_border_color(s_cellBox[i], fired ? C_FIRED_BD : C_CELLOFF_BD, 0);
    lv_obj_set_style_shadow_width(s_cellBox[i], fired ? 10 : 0, 0);
    lv_obj_set_style_shadow_color(s_cellBox[i], C_FIRED, 0);
    lv_obj_set_style_shadow_opa(s_cellBox[i], fired ? LV_OPA_70 : LV_OPA_TRANSP, 0);
    lv_obj_set_style_text_color(s_cellNum[i], fired ? lv_color_black() : C_NUMOFF, 0);
}

// Pulse/flash animation tick (LVGL task). Owns the card glow intensity and the
// transient white cell flashes; steady cell/colour state is set in update().
static void pulse_timer_cb(lv_timer_t* /*t*/) {
    g_tick++;

    // Card glow.
    int opa, width;
    if (g_pulse == PULSE_ARMED) {
        int ph = g_tick % 12, tri = (ph < 6) ? ph : 12 - ph;  // 0..6 triangle
        opa = 70 + tri * 30; width = 14 + tri * 4;            // breathe
    } else if (g_pulse == PULSE_ALARM) {
        bool on = (g_tick / 3) % 2;                           // ~0.7s blink
        opa = on ? 255 : 40; width = on ? 34 : 12;
    } else {
        opa = 110; width = 22;                               // steady
    }
    lv_obj_set_style_shadow_opa(s_card, opa, 0);
    lv_obj_set_style_shadow_width(s_card, width, 0);

    // Transient "just fired" white flashes.
    for (int i = 0; i < 16; ++i) {
        if (g_flash[i] > 0) {
            lv_obj_set_style_bg_color(s_cellBox[i], C_FLASH, 0);
            lv_obj_set_style_border_color(s_cellBox[i], C_FLASH, 0);
            lv_obj_set_style_shadow_width(s_cellBox[i], 14, 0);
            lv_obj_set_style_shadow_color(s_cellBox[i], C_FLASH, 0);
            lv_obj_set_style_shadow_opa(s_cellBox[i], LV_OPA_COVER, 0);
            lv_obj_set_style_text_color(s_cellNum[i], lv_color_black(), 0);
            if (--g_flash[i] == 0) render_cell_settled(i, true);  // settle to green
        }
    }
}

void dashboard_create(void) {
    if (!lvgl_port_lock(0)) return;
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, C_BG, 0);
    lv_obj_set_style_bg_opa(scr, LV_OPA_COVER, 0);
    lv_obj_remove_flag(scr, LV_OBJ_FLAG_SCROLLABLE);

    // --- header: heartbeat dot + title (left), link (right) ---
    s_dot = lv_obj_create(scr);
    lv_obj_set_size(s_dot, 10, 10);
    lv_obj_align(s_dot, LV_ALIGN_TOP_LEFT, 10, 11);
    lv_obj_remove_flag(s_dot, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(s_dot, 5, 0);
    lv_obj_set_style_border_width(s_dot, 0, 0);
    lv_obj_set_style_bg_color(s_dot, C_LINKOK, 0);

    lv_obj_t* title = lv_label_create(scr);
    lv_obj_set_style_text_font(title, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(title, C_MUTED, 0);
    lv_label_set_text(title, "FIRE BOX 0");
    lv_obj_align(title, LV_ALIGN_TOP_LEFT, 28, 8);

    s_link = lv_label_create(scr);
    lv_obj_set_style_text_font(s_link, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_link, C_MUTED, 0);
    lv_label_set_text(s_link, LV_SYMBOL_WIFI "  --");
    lv_obj_align(s_link, LV_ALIGN_TOP_RIGHT, -12, 8);

    // --- hero state card (gradient + colored glow) ---
    s_card = lv_obj_create(scr);
    lv_obj_set_size(s_card, 300, 74);
    lv_obj_align(s_card, LV_ALIGN_TOP_MID, 0, 28);
    lv_obj_remove_flag(s_card, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_radius(s_card, 16, 0);
    lv_obj_set_style_border_width(s_card, 0, 0);
    lv_obj_set_style_pad_all(s_card, 0, 0);
    lv_obj_set_style_bg_grad_dir(s_card, LV_GRAD_DIR_VER, 0);
    lv_obj_set_style_shadow_spread(s_card, 1, 0);

    s_state = lv_label_create(s_card);
    lv_obj_set_style_text_font(s_state, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_state, lv_color_white(), 0);
    lv_label_set_text(s_state, "...");
    lv_obj_center(s_state);

    // --- stats row: FIRED (left) · LAST (center) · SEQ (right) ---
    s_fired = lv_label_create(scr);
    lv_obj_set_style_text_font(s_fired, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_fired, lv_color_white(), 0);
    lv_label_set_text(s_fired, "FIRED  0/16");
    lv_obj_align(s_fired, LV_ALIGN_TOP_LEFT, 12, 108);

    s_last = lv_label_create(scr);
    lv_obj_set_style_text_font(s_last, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_last, C_MUTED, 0);
    lv_label_set_text(s_last, "LAST --");
    lv_obj_align(s_last, LV_ALIGN_TOP_MID, 0, 108);

    s_seq = lv_label_create(scr);
    lv_obj_set_style_text_font(s_seq, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_seq, C_MUTED, 0);
    lv_label_set_text(s_seq, "IDLE");
    lv_obj_align(s_seq, LV_ALIGN_TOP_RIGHT, -12, 108);

    // --- fired progress bar ---
    s_bar = lv_bar_create(scr);
    lv_obj_set_size(s_bar, 296, 8);
    lv_obj_align(s_bar, LV_ALIGN_TOP_MID, 0, 130);
    lv_obj_set_style_radius(s_bar, 4, 0);
    lv_obj_set_style_bg_color(s_bar, C_GREY2, LV_PART_MAIN);
    lv_obj_set_style_bg_color(s_bar, C_FIRED, LV_PART_INDICATOR);
    lv_obj_set_style_radius(s_bar, 4, LV_PART_INDICATOR);
    lv_bar_set_range(s_bar, 0, 16);
    lv_bar_set_value(s_bar, 0, LV_ANIM_OFF);

    // --- fired grid: 2 rows x 8 numbered cells ---
    for (int i = 0; i < 16; ++i) {
        int col = i % 8, row = i / 8;
        lv_obj_t* c = lv_obj_create(scr);
        lv_obj_set_size(c, 33, 32);
        lv_obj_align(c, LV_ALIGN_TOP_LEFT, 10 + col * 37, 150 + row * 42);
        lv_obj_remove_flag(c, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_style_radius(c, 7, 0);
        lv_obj_set_style_pad_all(c, 0, 0);
        lv_obj_set_style_border_width(c, 1, 0);
        lv_obj_set_style_border_color(c, C_CELLOFF_BD, 0);
        lv_obj_set_style_bg_color(c, C_CELLOFF, 0);
        s_cellBox[i] = c;

        lv_obj_t* n = lv_label_create(c);
        lv_obj_set_style_text_font(n, &lv_font_montserrat_14, 0);
        lv_obj_set_style_text_color(n, C_NUMOFF, 0);
        char t[4];
        std::snprintf(t, sizeof(t), "%d", i);
        lv_label_set_text(n, t);
        lv_obj_center(n);
        s_cellNum[i] = n;
    }

    lv_timer_create(pulse_timer_cb, 120, nullptr);  // glow + flash animation
    lvgl_port_unlock();
}

void dashboard_update(const StatusModel& m) {
    if (!lvgl_port_lock(0)) return;

    // --- hero state card: text, gradient, glow colour, pulse mode ---
    const char* word;
    lv_color_t top, bot;
    PulseMode pulse;
    if (!m.controllerReachable)               { word = "NO CTRL"; top = C_GREY;  bot = C_GREY2;  pulse = PULSE_ALARM; }
    else if (!m.boxPresent || !m.boxLinkAlive){ word = "NO LINK"; top = C_AMBER; bot = C_AMBER2; pulse = PULSE_ALARM; }
    else if (m.boxArmed)                       { word = "ARMED";   top = C_ARMED; bot = C_ARMED2; pulse = PULSE_ARMED; }
    else                                       { word = "SAFE";    top = C_SAFE;  bot = C_SAFE2;  pulse = PULSE_NONE;  }
    lv_label_set_text(s_state, word);
    lv_obj_set_style_bg_color(s_card, top, 0);
    lv_obj_set_style_bg_grad_color(s_card, bot, 0);
    lv_obj_set_style_shadow_color(s_card, top, 0);
    g_pulse = pulse;

    // --- heartbeat dot: toggle each successful poll (freezes if polls stop) ---
    static bool hb = false;
    hb = !hb;
    lv_obj_set_style_bg_color(s_dot, hb ? C_LINKOK : C_GREY2, 0);

    // --- link: wifi icon + rssi, coloured by strength ---
    char link[32];
    if (m.controllerReachable && m.boxPresent && m.boxLinkAlive) {
        std::snprintf(link, sizeof(link), LV_SYMBOL_WIFI "  %d dBm", m.rssi);
        lv_color_t lc = (m.rssi >= -60) ? C_LINKOK : (m.rssi >= -75) ? C_LINKWARN : C_LINKBAD;
        lv_obj_set_style_text_color(s_link, lc, 0);
    } else {
        std::snprintf(link, sizeof(link), LV_SYMBOL_WARNING "  LINK LOST");
        lv_obj_set_style_text_color(s_link, C_LINKBAD, 0);
    }
    lv_label_set_text(s_link, link);

    // --- fired count + bar + last-fired ---
    int n = popcount16(m.firedBitmap);
    char fired[20];
    std::snprintf(fired, sizeof(fired), "FIRED  %d/16", n);
    lv_label_set_text(s_fired, fired);
    lv_bar_set_value(s_bar, n, LV_ANIM_OFF);

    char last[16];
    if (m.lastFired >= 0) std::snprintf(last, sizeof(last), "LAST %d", m.lastFired);
    else                  std::snprintf(last, sizeof(last), "LAST --");
    lv_label_set_text(s_last, last);

    // --- sequence ---
    if (m.seqRunning) {
        lv_label_set_text(s_seq, LV_SYMBOL_PLAY "  RUN");
        lv_obj_set_style_text_color(s_seq, C_LINKOK, 0);
    } else {
        lv_label_set_text(s_seq, "IDLE");
        lv_obj_set_style_text_color(s_seq, C_MUTED, 0);
    }

    // --- fired grid: settle each cell, and flash the newly-fired ones ---
    uint16_t newBits = (uint16_t)(m.firedBitmap & ~g_fired);
    for (int i = 0; i < 16; ++i) {
        bool fired = (m.firedBitmap >> i) & 0x1;
        render_cell_settled(i, fired);
        if ((newBits >> i) & 0x1) g_flash[i] = FLASH_TICKS;  // just fired -> flash
    }
    g_fired = m.firedBitmap;

    lvgl_port_unlock();
}
