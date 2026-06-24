#include "dashboard.h"
#include "esp_lvgl_port.h"
#include "lvgl.h"
#include <cstdio>

static lv_obj_t* s_banner = nullptr;   // ARMED/SAFE/NO CONTROLLER/NO BOX LINK
static lv_obj_t* s_boxline = nullptr;  // link + rssi
static lv_obj_t* s_seqline = nullptr;  // sequence
static lv_obj_t* s_cells[16] = {nullptr};

static const lv_color_t C_GREEN = LV_COLOR_MAKE(0x14, 0x80, 0x2a);
static const lv_color_t C_RED   = LV_COLOR_MAKE(0xc0, 0x10, 0x10);
static const lv_color_t C_GREY  = LV_COLOR_MAKE(0x55, 0x55, 0x55);
static const lv_color_t C_AMBER = LV_COLOR_MAKE(0xc8, 0x80, 0x00);
static const lv_color_t C_CELL_OFF = LV_COLOR_MAKE(0x22, 0x22, 0x22);
static const lv_color_t C_CELL_ON  = LV_COLOR_MAKE(0xe0, 0xa0, 0x20);

void dashboard_create(void) {
    if (!lvgl_port_lock(0)) return;
    lv_obj_t* scr = lv_screen_active();
    lv_obj_set_style_bg_color(scr, lv_color_black(), 0);

    // Hero banner (top, full width)
    s_banner = lv_label_create(scr);
    lv_obj_set_style_text_font(s_banner, &lv_font_montserrat_48, 0);
    lv_obj_set_style_text_color(s_banner, lv_color_white(), 0);
    lv_obj_set_style_bg_opa(s_banner, LV_OPA_COVER, 0);
    lv_obj_set_style_pad_all(s_banner, 8, 0);
    lv_obj_set_width(s_banner, 320);
    lv_obj_set_style_text_align(s_banner, LV_TEXT_ALIGN_CENTER, 0);
    lv_obj_align(s_banner, LV_ALIGN_TOP_MID, 0, 0);
    lv_label_set_text(s_banner, "...");

    // Box line
    s_boxline = lv_label_create(scr);
    lv_obj_set_style_text_font(s_boxline, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_boxline, lv_color_white(), 0);
    lv_obj_align(s_boxline, LV_ALIGN_TOP_LEFT, 6, 80);
    lv_label_set_text(s_boxline, "");

    // Sequence line
    s_seqline = lv_label_create(scr);
    lv_obj_set_style_text_font(s_seqline, &lv_font_montserrat_14, 0);
    lv_obj_set_style_text_color(s_seqline, lv_color_white(), 0);
    lv_obj_align(s_seqline, LV_ALIGN_TOP_LEFT, 6, 104);
    lv_label_set_text(s_seqline, "");

    // Fired grid: 2 rows x 8 cells along the bottom
    for (int i = 0; i < 16; ++i) {
        lv_obj_t* c = lv_obj_create(scr);
        lv_obj_set_size(c, 34, 34);
        lv_obj_set_style_radius(c, 4, 0);
        lv_obj_set_style_border_width(c, 1, 0);
        lv_obj_set_style_bg_color(c, C_CELL_OFF, 0);
        int col = i % 8, row = i / 8;
        lv_obj_align(c, LV_ALIGN_TOP_LEFT, 6 + col * 38, 140 + row * 40);
        s_cells[i] = c;
    }
    lvgl_port_unlock();
}

void dashboard_update(const StatusModel& m) {
    if (!lvgl_port_lock(0)) return;

    if (!m.controllerReachable) {
        lv_label_set_text(s_banner, "NO CONTROLLER");
        lv_obj_set_style_bg_color(s_banner, C_GREY, 0);
    } else if (!m.boxPresent || !m.boxLinkAlive) {
        lv_label_set_text(s_banner, "NO BOX LINK");
        lv_obj_set_style_bg_color(s_banner, C_AMBER, 0);
    } else if (m.boxArmed) {
        lv_label_set_text(s_banner, "ARMED");
        lv_obj_set_style_bg_color(s_banner, C_RED, 0);
    } else {
        lv_label_set_text(s_banner, "SAFE");
        lv_obj_set_style_bg_color(s_banner, C_GREEN, 0);
    }

    char line[48];
    if (m.controllerReachable && m.boxLinkAlive)
        std::snprintf(line, sizeof(line), "Box0  LINK OK  %d dBm", m.rssi);
    else
        std::snprintf(line, sizeof(line), "Box0  LINK DOWN");
    lv_label_set_text(s_boxline, line);

    lv_label_set_text(s_seqline, m.seqRunning ? "Sequence: running" : "Sequence: idle");

    for (int i = 0; i < 16; ++i) {
        bool on = (m.firedBitmap >> i) & 0x1;
        lv_obj_set_style_bg_color(s_cells[i], on ? C_CELL_ON : C_CELL_OFF, 0);
    }
    lvgl_port_unlock();
}
