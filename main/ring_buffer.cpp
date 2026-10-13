#include "ring_buffer.hpp"

#include "esp_heap_caps.h"
#include "esp_log.h"

namespace loamair {
namespace {
constexpr char TAG[] = "ring";

constexpr int kAllocCaps = MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT;
}  // namespace

RingBuffer::~RingBuffer()
{
    deinit();
}

esp_err_t RingBuffer::init(size_t capacity_bytes)
{
    deinit();

    const size_t records = capacity_bytes / sizeof(Record);
    if (records == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    buf_ = static_cast<Record *>(heap_caps_malloc(records * sizeof(Record), kAllocCaps));
    if (buf_ == nullptr) {
        return ESP_ERR_NO_MEM;
    }

    cap_records_ = records;
    reset();
    return ESP_OK;
}

void RingBuffer::deinit()
{
    if (buf_ != nullptr) {
        heap_caps_free(buf_);
        buf_ = nullptr;
    }
    cap_records_ = 0;
}

size_t RingBuffer::size() const
{
    const size_t head = head_.load(std::memory_order_acquire);
    const size_t tail = tail_.load(std::memory_order_acquire);
    return head - tail;  // unsigned wrap-safe
}

size_t RingBuffer::free() const
{
    return cap_records_ - size();
}

bool RingBuffer::write(const Record &r)
{
    const size_t head = head_.load(std::memory_order_relaxed);
    const size_t tail = tail_.load(std::memory_order_acquire);
    if (head - tail >= cap_records_) {
        return false;  // full
    }

    buf_[head % cap_records_] = r;
    head_.store(head + 1, std::memory_order_release);
    return true;
}

size_t RingBuffer::write_bulk(const Record *r, size_t n)
{
    size_t written = 0;
    while (written < n && write(r[written])) {
        ++written;
    }
    return written;
}

bool RingBuffer::read(Record &r)
{
    const size_t tail = tail_.load(std::memory_order_relaxed);
    const size_t head = head_.load(std::memory_order_acquire);
    if (tail == head) {
        return false;  // empty
    }

    r = buf_[tail % cap_records_];
    tail_.store(tail + 1, std::memory_order_release);
    return true;
}

size_t RingBuffer::read_bulk(Record *out, size_t max)
{
    size_t read = 0;
    while (read < max && this->read(out[read])) {
        ++read;
    }
    return read;
}

void RingBuffer::reset()
{
    head_.store(0, std::memory_order_relaxed);
    tail_.store(0, std::memory_order_relaxed);
}

bool ring_buffer_selftest()
{
    RingBuffer rb;
    constexpr size_t kRecords = 64;
    if (rb.init(kRecords * sizeof(Record)) != ESP_OK) {
        ESP_LOGE(TAG, "selftest: allocation failed");
        return false;
    }

    bool ok = true;
    Record r{};

    // Empty buffer must not read.
    ok = ok && !rb.read(r);

    // Fill, then verify full-rejection and ordering.
    for (size_t i = 0; i < kRecords; ++i) {
        r.ax = static_cast<int16_t>(i);
        ok = ok && rb.write(r);
    }
    ok = ok && (rb.size() == kRecords);
    ok = ok && !rb.write(r);  // full

    for (size_t i = 0; i < kRecords; ++i) {
        ok = ok && rb.read(r) && (r.ax == static_cast<int16_t>(i));
    }
    ok = ok && !rb.read(r);  // drained

    // Wrap: advance the counters past the end and read back.
    for (size_t i = 0; i < 40; ++i) {
        r.ax = static_cast<int16_t>(200 + i);
        ok = ok && rb.write(r);
    }
    for (size_t i = 0; i < 40; ++i) {
        ok = ok && rb.read(r) && (r.ax == static_cast<int16_t>(200 + i));
    }

    ok = ok && (rb.size() == 0);

    rb.deinit();
    return ok;
}

}  // namespace loamair
