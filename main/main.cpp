#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "config.hpp"
#include "imu.hpp"
#include "ring_buffer.hpp"
#include "transfer.hpp"

static const char *TAG = "loamair";

namespace {

loamair::RingBuffer g_ring;

struct ImuTaskArgs {
    loamair::Imu *imu;
    loamair::RingBuffer *ring;
};

// Largest usable internal-heap block, minus a reserve for Wi-Fi + HTTP.
size_t compute_buffer_bytes()
{
    constexpr int caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    const size_t largest = heap_caps_get_largest_free_block(caps);

    if (largest <= loamair::kHeapReserveBytes + loamair::kMinBufferBytes) {
        return 0;
    }

    size_t target = largest - loamair::kHeapReserveBytes;
    if (target > loamair::kMaxBufferBytes) {
        target = loamair::kMaxBufferBytes;
    }
    target -= target % sizeof(loamair::Record);
    return target;
}

void log_heap(const char *when)
{
    constexpr int caps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
    ESP_LOGI(TAG, "heap %s: free=%u largest=%u", when,
             static_cast<unsigned>(heap_caps_get_free_size(caps)),
             static_cast<unsigned>(heap_caps_get_largest_free_block(caps)));
}

void imu_task(void *arg)
{
    auto *args = static_cast<ImuTaskArgs *>(arg);
    loamair::Imu *imu = args->imu;
    loamair::RingBuffer *ring = args->ring;

    TickType_t last_wake = xTaskGetTickCount();
    const TickType_t period = pdMS_TO_TICKS(loamair::kSamplePeriodMs);

    const int64_t session_start_us = esp_timer_get_time();

    uint32_t samples = 0;
    uint32_t failures = 0;
    uint32_t dropped = 0;

    while (true) {
        loamair::ImuSample s{};
        if (imu->read(s) == ESP_OK) {
            ++samples;

            loamair::Record rec{};
            rec.ts_offset_us =
                static_cast<uint32_t>(esp_timer_get_time() - session_start_us);
            rec.ax = s.ax;
            rec.ay = s.ay;
            rec.az = s.az;
            rec.gx = s.gx;
            rec.gy = s.gy;
            rec.gz = s.gz;

            if (!ring->write(rec)) {
                ++dropped;  // consumer is behind; overwrite policy is drop-newest
            }

            if (samples % loamair::kSampleRateHz == 0) {
                ESP_LOGI(TAG, "ax=%.3f ay=%.3f az=%.3f g | gx=%.1f gy=%.1f gz=%.1f dps | "
                              "ring %u/%u dropped=%lu failures=%lu",
                         s.ax / loamair::Imu::kAccelLsbPerG,
                         s.ay / loamair::Imu::kAccelLsbPerG,
                         s.az / loamair::Imu::kAccelLsbPerG,
                         s.gx / loamair::Imu::kGyroLsbPerDps,
                         s.gy / loamair::Imu::kGyroLsbPerDps,
                         s.gz / loamair::Imu::kGyroLsbPerDps,
                         static_cast<unsigned>(ring->size()),
                         static_cast<unsigned>(ring->capacity()),
                         static_cast<unsigned long>(dropped),
                         static_cast<unsigned long>(failures));
            }
        } else {
            ++failures;
        }
        vTaskDelayUntil(&last_wake, period);
    }
}

}  // namespace

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "LoamAir booting (ESP-IDF %s)", esp_get_idf_version());

    log_heap("before buffer");

    const size_t buffer_bytes = compute_buffer_bytes();
    if (buffer_bytes == 0) {
        ESP_LOGE(TAG, "not enough internal heap to reserve a telemetry buffer");
    } else {
        ESP_ERROR_CHECK(g_ring.init(buffer_bytes));
        const double seconds = static_cast<double>(g_ring.capacity()) / loamair::kSampleRateHz;
        ESP_LOGI(TAG, "ring buffer: %u bytes, %u records (%.1f s @ %u Hz)",
                 static_cast<unsigned>(g_ring.capacity() * sizeof(loamair::Record)),
                 static_cast<unsigned>(g_ring.capacity()), seconds,
                 static_cast<unsigned>(loamair::kSampleRateHz));
    }

    log_heap("after buffer");

    if (!loamair::ring_buffer_selftest()) {
        ESP_LOGE(TAG, "ring buffer selftest FAILED");
    } else {
        ESP_LOGI(TAG, "ring buffer selftest passed");
    }

    static loamair::Imu imu;
    ESP_ERROR_CHECK(imu.init());

    static ImuTaskArgs args{&imu, &g_ring};
    xTaskCreatePinnedToCore(imu_task, "imu", 4096, &args, 5, nullptr, 1);

    // Phase 3: serve immediately. Phase 4 will gate this behind the session
    // state machine (button-triggered TRANSFER).
    static loamair::Transfer transfer;
    ESP_ERROR_CHECK(transfer.start(&g_ring));
}
