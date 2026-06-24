/*
 * web_server.cpp — HTTP JSON API for the firework controller.
 *
 * JSON parsing: cJSON is a managed component (not built-in IDF) in v6.0.1, so
 * we use a minimal hand-rolled parser for the two fixed shapes we need:
 *   - {"box":N,"channel":M}
 *   - {"steps":[[t,b,c],...]}
 * Both shapes are flat / singly-nested; sscanf + strstr handles them safely.
 *
 * Concurrency:
 *   All handlers run in the httpd task.  They must NOT call runner.* directly.
 *   Instead they post a UiCommand to g_cmdQueue (non-blocking, 0-tick timeout).
 *   The control-loop task drains g_cmdQueue each tick before runner.tick().
 *   For /api/run: g_runSteps[] is written by the handler then RUN is enqueued;
 *   the loop reads g_runSteps only on the RUN command (SPSC — safe without mutex).
 *   Status snapshot: volatile fields written by loop after tick, read by handler.
 */

#include "web_server.h"
#include "esp_http_server.h"
#include "esp_log.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

static const char* TAG = "web_server";

// ── helpers ────────────────────────────────────────────────────────────────

static esp_err_t send_ok(httpd_req_t* req) {
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, "{\"ok\":true}", HTTPD_RESP_USE_STRLEN);
}

static esp_err_t send_fail(httpd_req_t* req, int http_code, const char* msg) {
    char buf[96];
    snprintf(buf, sizeof(buf), "{\"ok\":false,\"error\":\"%s\"}", msg);
    if (http_code == 503) {
        httpd_resp_set_status(req, "503 Service Unavailable");
    } else {
        httpd_resp_set_status(req, "400 Bad Request");
    }
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

// Read the entire request body (up to 4096 bytes) into a null-terminated
// heap buffer.  Caller must free().  Returns nullptr on error.
static char* read_body(httpd_req_t* req) {
    int total = req->content_len;
    if (total <= 0 || total > 4096) return nullptr;
    char* buf = static_cast<char*>(malloc(total + 1));
    if (!buf) return nullptr;
    int received = 0;
    while (received < total) {
        int r = httpd_req_recv(req, buf + received, total - received);
        if (r <= 0) { free(buf); return nullptr; }
        received += r;
    }
    buf[total] = '\0';
    return buf;
}

// Post a UiCommand to the queue.  Responds 503 if full, 200 ok otherwise.
static esp_err_t enqueue_cmd(const UiCommand& cmd, httpd_req_t* req) {
    if (xQueueSend(g_cmdQueue, &cmd, 0) != pdTRUE) {
        return send_fail(req, 503, "queue full");
    }
    return send_ok(req);
}

// ── Minimal JSON helpers ───────────────────────────────────────────────────
// Only handles the two fixed shapes used by /api/fire and /api/run.

// Parse {"box":N,"channel":M} — returns true and sets *box/*ch on success.
static bool parse_fire_json(const char* s, int* box, int* ch) {
    // Find "box" value
    const char* p = strstr(s, "\"box\"");
    if (!p) return false;
    p += 5; // skip "box"
    while (*p == ' ' || *p == ':') p++;
    if (sscanf(p, "%d", box) != 1) return false;

    // Find "channel" value
    p = strstr(s, "\"channel\"");
    if (!p) return false;
    p += 9; // skip "channel"
    while (*p == ' ' || *p == ':') p++;
    if (sscanf(p, "%d", ch) != 1) return false;

    return true;
}

// Parse {"steps":[[t,b,c],...]} into g_runSteps[].
// Sets *count on success.  Returns false on parse error.
static bool parse_run_json(const char* s, size_t* count) {
    const char* p = strstr(s, "\"steps\"");
    if (!p) return false;
    p += 7; // skip "steps"
    while (*p == ' ' || *p == ':') p++;
    if (*p != '[') return false;
    p++; // skip outer [

    size_t n = 0;
    while (*p && *p != ']') {
        // skip whitespace and commas between steps
        while (*p == ' ' || *p == '\t' || *p == '\n' || *p == '\r' || *p == ',') p++;
        if (*p != '[') break;
        p++; // skip inner [

        long t = 0, b = 0, c = 0;
        char* end;
        t = strtol(p, &end, 10); if (end == p) return false; p = end;
        while (*p == ' ' || *p == ',') p++;
        b = strtol(p, &end, 10); if (end == p) return false; p = end;
        while (*p == ' ' || *p == ',') p++;
        c = strtol(p, &end, 10); if (end == p) return false; p = end;

        // Skip to end of inner array
        while (*p && *p != ']') p++;
        if (*p == ']') p++;

        if (n >= MAX_RUN_STEPS) return false;
        g_runSteps[n].timeMs  = static_cast<uint32_t>(t);
        g_runSteps[n].boxId   = static_cast<uint8_t>(b);
        g_runSteps[n].channel = static_cast<uint8_t>(c);
        n++;
    }
    if (n == 0) return false;
    *count = n;
    return true;
}

// ── GET / — serve embedded single-file webui ──────────────────────────────
// www/index.html.gz is embedded at link time via EMBED_FILES in CMakeLists.txt.
// The browser receives the raw gzip bytes; Content-Encoding: gzip tells it to decompress.
extern const uint8_t index_html_gz_start[] asm("_binary_index_html_gz_start");
extern const uint8_t index_html_gz_end[]   asm("_binary_index_html_gz_end");

static esp_err_t handle_root(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Content-Encoding", "gzip");
    size_t len = static_cast<size_t>(index_html_gz_end - index_html_gz_start);
    return httpd_resp_send(req,
        reinterpret_cast<const char*>(index_html_gz_start),
        static_cast<ssize_t>(len));
}

// ── Simple POST handlers (no body required) ────────────────────────────────

static esp_err_t handle_arm(httpd_req_t* req) {
    UiCommand cmd{}; cmd.type = CmdType::ARM;
    return enqueue_cmd(cmd, req);
}

static esp_err_t handle_disarm(httpd_req_t* req) {
    UiCommand cmd{}; cmd.type = CmdType::DISARM;
    return enqueue_cmd(cmd, req);
}

static esp_err_t handle_estop(httpd_req_t* req) {
    UiCommand cmd{}; cmd.type = CmdType::ESTOP;
    return enqueue_cmd(cmd, req);
}

static esp_err_t handle_stop(httpd_req_t* req) {
    UiCommand cmd{}; cmd.type = CmdType::STOP;
    return enqueue_cmd(cmd, req);
}

static esp_err_t handle_heartbeat(httpd_req_t* req) {
    UiCommand cmd{}; cmd.type = CmdType::HEARTBEAT;
    return enqueue_cmd(cmd, req);
}

// ── POST /api/fire  { "box": N, "channel": M } ────────────────────────────

static esp_err_t handle_fire(httpd_req_t* req) {
    char* body = read_body(req);
    if (!body) return send_fail(req, 400, "bad body");

    int box = 0, ch = 0;
    if (!parse_fire_json(body, &box, &ch)) {
        free(body);
        return send_fail(req, 400, "need {box,channel}");
    }
    free(body);

    UiCommand cmd{};
    cmd.type    = CmdType::FIRE;
    cmd.boxId   = static_cast<uint8_t>(box);
    cmd.channel = static_cast<uint8_t>(ch);
    return enqueue_cmd(cmd, req);
}

// ── POST /api/run  { "steps": [[timeMs, box, channel], ...] } ─────────────

static esp_err_t handle_run(httpd_req_t* req) {
    char* body = read_body(req);
    if (!body) return send_fail(req, 400, "bad body");

    size_t n = 0;
    // Write g_runSteps BEFORE enqueueing RUN — SPSC: loop reads only on RUN cmd.
    if (!parse_run_json(body, &n)) {
        free(body);
        return send_fail(req, 400, "need {steps:[[t,b,ch],...]}");
    }
    free(body);

    g_runCount = n;

    UiCommand cmd{}; cmd.type = CmdType::RUN;
    return enqueue_cmd(cmd, req);
}

// ── GET /api/status ────────────────────────────────────────────────────────

static esp_err_t handle_status(httpd_req_t* req) {
    // Snapshot fields are volatile; reads are atomic on Xtensa for word-sized types.
    bool    armed   = g_status.armed;
    bool    seq     = g_status.seqRunning;
    uint8_t lastBox = g_status.lastFailedBox;

    char buf[420];
    char lastBoxTok[8];
    if (lastBox == 0xFF) snprintf(lastBoxTok, sizeof(lastBoxTok), "null");
    else                 snprintf(lastBoxTok, sizeof(lastBoxTok), "%u", lastBox);

    int n = snprintf(buf, sizeof(buf),
        "{\"armed\":%s,\"seqRunning\":%s,\"lastFailedBox\":%s,\"boxes\":[",
        armed ? "true" : "false", seq ? "true" : "false", lastBoxTok);

    for (int b = 0; b < 2; b++) {
        BoxTelemetry box = g_status.boxes[b];   // snapshot volatile fields once
        size_t rem = (n < (int)sizeof(buf)) ? sizeof(buf) - (size_t)n : 0;
        n += snprintf(buf + n, rem,
            "%s{\"id\":%d,\"linkAlive\":%s,\"rssi\":%d,\"state\":%u,"
            "\"firedBitmap\":%u,\"lastFired\":%d}",
            b ? "," : "", b,
            box.linkAlive ? "true" : "false",
            (int)box.rssi,
            (unsigned)box.state,
            (unsigned)box.firedBitmap,
            (box.lastFiredChannel == 0xFF) ? -1 : (int)box.lastFiredChannel);
    }
    size_t rem2 = (n < (int)sizeof(buf)) ? sizeof(buf) - (size_t)n : 0;
    n += snprintf(buf + n, rem2, "]}");

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, buf, HTTPD_RESP_USE_STRLEN);
}

// ── WebServer::start ────────────────────────────────────────────────────────

esp_err_t WebServer::start(QueueHandle_t cmdQueue, StatusSnapshot* /*snap*/) {
    // Assign the caller-provided queue to the global so the interface contract
    // is honest: a caller passing cmdQueue through this API has it wired up.
    // snap is kept for API clarity; g_status is the same object (&g_status == snap).
    g_cmdQueue = cmdQueue;

    httpd_config_t config    = HTTPD_DEFAULT_CONFIG();
    config.lru_purge_enable  = true;
    config.max_uri_handlers  = 16;  // default is 8; we have 9 routes

    httpd_handle_t server = nullptr;
    esp_err_t ret = httpd_start(&server, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "httpd_start failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Register URI handlers using struct literal syntax compatible with C++ / gnu++17.
    // The IDF toolchain (-std=gnu++17) accepts C99 designated initializers as an extension.
    struct Route { const char* uri; httpd_method_t method; esp_err_t (*handler)(httpd_req_t*); };
    static const Route routes[] = {
        { "/",              HTTP_GET,  handle_root      },
        { "/api/arm",       HTTP_POST, handle_arm       },
        { "/api/disarm",    HTTP_POST, handle_disarm    },
        { "/api/estop",     HTTP_POST, handle_estop     },
        { "/api/stop",      HTTP_POST, handle_stop      },
        { "/api/heartbeat", HTTP_POST, handle_heartbeat },
        { "/api/fire",      HTTP_POST, handle_fire      },
        { "/api/run",       HTTP_POST, handle_run       },
        { "/api/status",    HTTP_GET,  handle_status    },
    };
    for (const auto& r : routes) {
        httpd_uri_t u = {};
        u.uri      = r.uri;
        u.method   = r.method;
        u.handler  = r.handler;
        u.user_ctx = nullptr;
        ESP_ERROR_CHECK(httpd_register_uri_handler(server, &u));
    }

    ESP_LOGI(TAG, "HTTP server started on port %d", config.server_port);
    return ESP_OK;
}
