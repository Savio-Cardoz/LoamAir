#include "transfer.hpp"

#include <cstdio>
#include <cstring>

#include "config.hpp"
#include "esp_check.h"
#include "esp_event.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"

namespace loamair {
namespace {

constexpr char TAG[] = "xfer";

constexpr int kHeapCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
constexpr size_t kTxBatchRecords = 64;
constexpr TickType_t kIdlePoll = pdMS_TO_TICKS(10);

constexpr char kIndexHtml[] =
    "<!doctype html><html><head><meta charset=utf-8><title>LoamAir</title></head>"
    "<body style=\"font-family:sans-serif;max-width:40em;margin:2em auto\">"
    "<h1>LoamAir</h1>"
    "<p>Telemetry buffer for the current session.</p>"
    "<ul>"
    "<li><a href=\"/status\">/status</a> &mdash; record count and capacity (JSON)</li>"
    "<li><a href=\"/data\">/data</a> &mdash; raw little-endian records "
    "(16 bytes: uint32 us + 6&times;int16)</li>"
    "<li><a href=\"/clear\">/clear</a> &mdash; clear the buffer</li>"
    "</ul>"
    "<p>Prefer <code>tools/loamair_client.py</code> to log and plot the stream.</p>"
    "</body></html>";

void init_nvs()
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

}  // namespace

Transfer::~Transfer()
{
    stop();
}

esp_err_t Transfer::start(RingBuffer *ring)
{
    if (running_.load(std::memory_order_acquire)) {
        return ESP_OK;
    }
    if (ring == nullptr || !ring->is_initialised()) {
        return ESP_ERR_INVALID_ARG;
    }
    ring_ = ring;

    ESP_RETURN_ON_ERROR(start_softap(), TAG, "softAP start failed");
    ESP_RETURN_ON_ERROR(register_handlers(), TAG, "HTTP server start failed");

    running_.store(true, std::memory_order_release);
    ESP_LOGI(TAG, "transfer ready: connect to \"" "%s" "\" and open http://192.168.4.1/",
             kApSsid);
    log_heap("after Wi-Fi + HTTP");
    return ESP_OK;
}

esp_err_t Transfer::stop()
{
    running_.store(false, std::memory_order_release);

    if (server_ != nullptr) {
        httpd_stop(server_);
        server_ = nullptr;
    }
    stop_softap();
    return ESP_OK;
}

esp_err_t Transfer::start_softap()
{
    if (wifi_up_) {
        return ESP_OK;
    }

    init_nvs();

    if (esp_netif_init() == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "netif already initialised");
    }
    if (esp_event_loop_create_default() == ESP_ERR_INVALID_STATE) {
        ESP_LOGD(TAG, "default event loop already exists");
    }

    if (netif_ == nullptr) {
        netif_ = esp_netif_create_default_wifi_ap();
        if (netif_ == nullptr) {
            ESP_LOGE(TAG, "failed to create default Wi-Fi AP netif");
            return ESP_FAIL;
        }
    }

    wifi_init_config_t init_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_RETURN_ON_ERROR(esp_wifi_init(&init_cfg), TAG, "esp_wifi_init failed");

    wifi_config_t ap_cfg = {};
    std::memcpy(ap_cfg.ap.ssid, kApSsid, std::strlen(kApSsid));
    ap_cfg.ap.ssid_len = static_cast<uint8_t>(std::strlen(kApSsid));
    ap_cfg.ap.channel = kApChannel;
    ap_cfg.ap.max_connection = kApMaxConn;
    ap_cfg.ap.authmode = WIFI_AUTH_OPEN;

    ESP_RETURN_ON_ERROR(esp_wifi_set_mode(WIFI_MODE_AP), TAG, "esp_wifi_set_mode failed");
    ESP_RETURN_ON_ERROR(esp_wifi_set_config(WIFI_IF_AP, &ap_cfg), TAG,
                        "esp_wifi_set_config failed");
    ESP_RETURN_ON_ERROR(esp_wifi_start(), TAG, "esp_wifi_start failed");

    wifi_up_ = true;
    return ESP_OK;
}

void Transfer::stop_softap()
{
    if (!wifi_up_) {
        return;
    }
    esp_wifi_stop();
    esp_wifi_deinit();
    wifi_up_ = false;

    if (netif_ != nullptr) {
        esp_netif_destroy(netif_);
        netif_ = nullptr;
    }
}

esp_err_t Transfer::register_handlers()
{
    httpd_config_t cfg = HTTPD_DEFAULT_CONFIG();
    cfg.stack_size = 8192;
    cfg.max_uri_handlers = 8;
    cfg.lru_purge_enable = true;
    ESP_RETURN_ON_ERROR(httpd_start(&server_, &cfg), TAG, "httpd_start failed");

    const httpd_uri_t uris[] = {
        {.uri = "/", .method = HTTP_GET, .handler = root_handler, .user_ctx = this},
        {.uri = "/status", .method = HTTP_GET, .handler = status_handler, .user_ctx = this},
        {.uri = "/data", .method = HTTP_GET, .handler = data_handler, .user_ctx = this},
        {.uri = "/clear", .method = HTTP_GET, .handler = clear_handler, .user_ctx = this},
    };
    for (const auto &u : uris) {
        ESP_RETURN_ON_ERROR(httpd_register_uri_handler(server_, &u), TAG,
                            "register %s failed", u.uri);
    }
    return ESP_OK;
}

void Transfer::log_heap(const char *when) const
{
    ESP_LOGI(TAG, "heap %s: free=%u largest=%u", when,
             static_cast<unsigned>(heap_caps_get_free_size(kHeapCaps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(kHeapCaps)));
}

esp_err_t Transfer::root_handler(httpd_req *req)
{
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, kIndexHtml, HTTPD_RESP_USE_STRLEN);
}

esp_err_t Transfer::status_handler(httpd_req *req)
{
    auto *self = static_cast<Transfer *>(req->user_ctx);
    char json[160];
    const int n = std::snprintf(
        json, sizeof(json),
        "{\"capacity\":%u,\"count\":%u,\"sample_rate_hz\":%u,\"record_size\":%u}",
        static_cast<unsigned>(self->ring_->capacity()),
        static_cast<unsigned>(self->ring_->size()),
        static_cast<unsigned>(kSampleRateHz),
        static_cast<unsigned>(sizeof(Record)));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, n);
}

esp_err_t Transfer::data_handler(httpd_req *req)
{
    auto *self = static_cast<Transfer *>(req->user_ctx);
    httpd_resp_set_type(req, "application/octet-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");

    Record batch[kTxBatchRecords];
    while (self->running_.load(std::memory_order_acquire)) {
        const size_t n = self->ring_->read_bulk(batch, kTxBatchRecords);
        if (n == 0) {
            vTaskDelay(kIdlePoll);
            continue;
        }
        const esp_err_t err = httpd_resp_send_chunk(
            req, reinterpret_cast<const char *>(batch), n * sizeof(Record));
        if (err != ESP_OK) {
            ESP_LOGI(TAG, "client disconnected");
            break;
        }
    }
    return httpd_resp_send_chunk(req, nullptr, 0);  // terminate chunked response
}

esp_err_t Transfer::clear_handler(httpd_req *req)
{
    auto *self = static_cast<Transfer *>(req->user_ctx);
    self->ring_->reset();
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_send(req, "OK\n", HTTPD_RESP_USE_STRLEN);
}

}  // namespace loamair
