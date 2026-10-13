#pragma once

#include <atomic>

#include "esp_err.h"
#include "esp_http_server.h"
#include "esp_netif.h"

#include "ring_buffer.hpp"

namespace loamair {

// Brings up a Wi-Fi softAP and a small HTTP server that serves the telemetry
// ring buffer. The device runs an open AP while streaming is active.
class Transfer {
public:
    Transfer() = default;
    ~Transfer();

    Transfer(const Transfer &) = delete;
    Transfer &operator=(const Transfer &) = delete;

    // Starts NVS/netif/Wi-Fi AP and the HTTP server. Safe to call repeatedly.
    esp_err_t start(RingBuffer *ring);
    esp_err_t stop();

    bool running() const { return running_.load(std::memory_order_acquire); }

    static constexpr const char *kApSsid = "LoamAir";
    static constexpr int kApChannel = 1;
    static constexpr int kApMaxConn = 2;

private:
    static esp_err_t root_handler(httpd_req *req);
    static esp_err_t status_handler(httpd_req *req);
    static esp_err_t data_handler(httpd_req *req);
    static esp_err_t clear_handler(httpd_req *req);

    esp_err_t start_softap();
    void stop_softap();
    esp_err_t register_handlers();
    void log_heap(const char *when) const;

    RingBuffer *ring_{nullptr};
    httpd_handle_t server_{nullptr};
    esp_netif_t *netif_{nullptr};
    bool wifi_up_{false};
    std::atomic<bool> running_{false};
};

}  // namespace loamair
