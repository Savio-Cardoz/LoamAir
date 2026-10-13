# Mountain Biking Airtime Sensor - Prototype Architecture

## 1. System Overview
This document outlines the architecture for the prototype wearable airtime sensor designed specifically for mountain biking. The system captures high-frequency inertial movement, detects jump events using a multi-sensor state machine, caches telemetry in the ESP32's volatile RAM, and serves the data over a local Wi-Fi Access Point upon user request.

---

## 2. Hardware Bill of Materials (BoM)
*   **Microcontroller:** ESP32-WROOM Development Board (Dual-core, integrated Wi-Fi/BLE, ample SRAM).
*   **Inertial Measurement Unit (IMU):** GY-87 10DoF Module, utilizing **only** the MPU6050 6-axis accelerometer & gyroscope via I2C (HMC5883L and BMP180 disabled/ignored to maximize I2C bus speed).
*   **Power Source:** Standard USB Power Bank (5V stepping down via onboard regulator for prototype phase).
*   **User Interaction:** External hardware push-button connected to a GPIO pin (configured with internal pull-up and interrupt) to toggle Wi-Fi transmission mode.

---

## 3. System State Machine & Data Flow

```
[IDLE / WAITING] 
       │
       ▼ (Motion / Pre-Takeoff Detected)
[LOGGING STATE (RAM Circular Buffer)]
       │
       ▼ (Button Pressed / Post-Run Trigger)
[WIFI AP + HTTP SERVER MODE] ──> [Smartphone Client Download]
```

### State Definitions:
1. **Idle State:** 
   * ESP32 polls the MPU6050 via I2C at 100 Hz.
   * Wi-Fi radio is completely disabled to conserve power.
   * Monitors for takeoff criteria (pitch rotation rate combined with dropping acceleration).
2. **Logging State:**
   * Active during the descent/jump run.
   * Serializes IMU packets ($16 \text{ bytes}$ per sample: 4-byte timestamp + 6 axes of int16_t data).
   * Writes packets into a pre-allocated static RAM circular buffer.
3. **Transfer State:**
   * Triggered by the external push-button interrupt.
   * Spawns an ESP32 Wi-Fi Access Point (AP) and starts a lightweight HTTP web server.
   * Streams the buffered session data directly to a smartphone browser or client app, then clears the buffer upon confirmation.

---

## 4. Memory & Buffer Specifications

*   **Sampling Rate:** 100 Hz ($100 \text{ samples/sec}$).
*   **Packet Structure:** 
    *   Timestamp: `uint32_t` (4 bytes)
    *   Accel X, Y, Z: `int16_t` (6 bytes)
    *   Gyro X, Y, Z: `int16_t` (6 bytes)
    *   *Total Packet Size:* 16 bytes.
*   **Throughput:** $100 \text{ Hz} \times 16 \text{ bytes} = 1.6 \text{ KB/sec}$.
*   **RAM Allocation Limit:** 
    *   Assuming $\approx 150 \text{ KB}$ of safe free heap allocated for the circular buffer:
    *   $\frac{150,000 \text{ bytes}}{1,600 \text{ bytes/sec}} \approx \textbf{93 seconds}$ of continuous high-frequency logging capacity per session.

---

## 5. Firmware Execution Guidelines

1. **I2C Optimization:** Run the I2C bus at **400 kHz** to ensure read cycles from the MPU6050 complete well within the 10ms sampling window.
2. **Memory Safety:** Use statically allocated arrays or FreeRTOS queues rather than dynamic heap allocations during active logging to avoid memory fragmentation.
3. **Safety Notice:** Because RAM is volatile, maintain a stable connection to the power bank; any power loss will clear the current session buffer.