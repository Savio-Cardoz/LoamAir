#pragma once

#include <atomic>
#include <cstddef>
#include <cstdint>

#include "esp_err.h"

namespace loamair {

// One 16-byte telemetry record, streamed in this exact byte layout.
// Little-endian: 4-byte session-relative timestamp + 6 raw int16 axes.
struct Record {
    uint32_t ts_offset_us;  // microseconds since session start
    int16_t ax;             // raw accel, 2048 LSB/g
    int16_t ay;
    int16_t az;
    int16_t gx;             // raw gyro, 16.4 LSB/dps
    int16_t gy;
    int16_t gz;
};

static_assert(sizeof(Record) == 16, "Record wire layout must be exactly 16 bytes");

// Single-producer / single-consumer ring buffer over a heap-allocated block.
// The producer (IMU task) calls write(); the consumer (HTTP task) calls read().
// No locks are used; correctness relies on the two counter atomics.
class RingBuffer {
public:
    RingBuffer() = default;
    ~RingBuffer();

    RingBuffer(const RingBuffer &) = delete;
    RingBuffer &operator=(const RingBuffer &) = delete;

    // Allocates enough internal RAM for at least `capacity_bytes` worth of
    // records (rounded down to whole records).
    esp_err_t init(size_t capacity_bytes);
    void deinit();

    bool is_initialised() const { return buf_ != nullptr; }
    size_t capacity() const { return cap_records_; }   // total record slots
    size_t size() const;                               // records available to read
    size_t free() const;                               // record slots remaining

    bool write(const Record &r);                 // producer
    size_t write_bulk(const Record *r, size_t n);

    bool read(Record &r);                        // consumer
    size_t read_bulk(Record *out, size_t max);

    // Only safe to call while the producer and consumer are quiescent.
    void reset();

private:
    Record *buf_{nullptr};
    size_t cap_records_{0};
    std::atomic<size_t> head_{0};  // total records ever written
    std::atomic<size_t> tail_{0};  // total records ever read
};

// Exercises wrap / full / empty / reset on a small temporary buffer.
// Returns true when all assertions pass.
bool ring_buffer_selftest();

}  // namespace loamair
