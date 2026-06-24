#include "status_client.h"
#include "status_parse.h"
#include "esp_http_client.h"
#include "esp_timer.h"
#include "esp_log.h"
#include <cstring>
#include <cstdio>

static const char* TAG = "status_client";
static const char* URL = "http://192.168.4.1/api/status";
static const char* EVENTS_URL = "http://192.168.4.1/api/events";

// Accumulate the response body into a fixed buffer.
struct Accum { char buf[512]; int len; };

static esp_err_t on_evt(esp_http_client_event_t* e) {
    if (e->event_id == HTTP_EVENT_ON_DATA) {
        Accum* a = (Accum*)e->user_data;
        int n = e->data_len;
        if (a->len + n > (int)sizeof(a->buf) - 1) n = sizeof(a->buf) - 1 - a->len;
        if (n > 0) { std::memcpy(a->buf + a->len, e->data, n); a->len += n; }
    }
    return ESP_OK;
}

void status_client_poll_once(StatusModel& model) {
    Accum acc{}; acc.len = 0;
    esp_http_client_config_t cfg = {};
    cfg.url = URL;
    cfg.timeout_ms = 800;
    cfg.event_handler = on_evt;
    cfg.user_data = &acc;
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) { model.controllerReachable = false; return; }
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);

    if (err != ESP_OK || status != 200) {
        model.controllerReachable = false;
        ESP_LOGW(TAG, "poll failed (err=%s status=%d)", esp_err_to_name(err), status);
        return;
    }
    acc.buf[acc.len] = 0;
    model.controllerReachable = true;
    parseStatus(acc.buf, model);
    model.lastUpdateMs = (uint32_t)(esp_timer_get_time() / 1000);
    ESP_LOGI(TAG, "armed=%d link=%d rssi=%d fired=0x%04x seq=%d present=%d",
             model.boxArmed, model.boxLinkAlive, model.rssi,
             (unsigned)model.firedBitmap, model.seqRunning, model.boxPresent);
}

struct AccumE { char buf[3300]; int len; };
static esp_err_t on_evt_e(esp_http_client_event_t* e) {
    if (e->event_id == HTTP_EVENT_ON_DATA) {
        AccumE* a = (AccumE*)e->user_data;
        int n = e->data_len;
        if (a->len + n > (int)sizeof(a->buf) - 1) n = sizeof(a->buf) - 1 - a->len;
        if (n > 0) { std::memcpy(a->buf + a->len, e->data, n); a->len += n; }
    }
    return ESP_OK;
}

int status_client_poll_events(uint32_t& sinceSeq, LogEv* out, int maxOut) {
    static AccumE acc;
    acc.len = 0;
    char url[80];
    std::snprintf(url, sizeof(url), "%s?since=%lu", EVENTS_URL, (unsigned long)sinceSeq);
    esp_http_client_config_t cfg = {};
    cfg.url = url; cfg.timeout_ms = 800; cfg.event_handler = on_evt_e; cfg.user_data = &acc;
    esp_http_client_handle_t c = esp_http_client_init(&cfg);
    if (!c) return 0;
    esp_err_t err = esp_http_client_perform(c);
    int status = esp_http_client_get_status_code(c);
    esp_http_client_cleanup(c);
    if (err != ESP_OK || status != 200) return 0;
    acc.buf[acc.len] = 0;
    int n = parseEvents(acc.buf, out, maxOut);
    for (int i = 0; i < n; ++i) if ((uint32_t)out[i].seq > sinceSeq) sinceSeq = (uint32_t)out[i].seq;
    return n;
}
