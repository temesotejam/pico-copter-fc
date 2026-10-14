#ifndef PICO_COPTER_DSHOT_PROTOCOL_HPP
#define PICO_COPTER_DSHOT_PROTOCOL_HPP

#include <stdint.h>
#include <cmath>

namespace dshot {
constexpr uint32_t kBitRate = 300000;
constexpr uint32_t kPioCyclesPerBit = 8;
constexpr uint32_t kPioClockHz = kBitRate * kPioCyclesPerBit;
constexpr uint16_t kStop = 0;
constexpr uint16_t kMinimumThrottle = 48;
constexpr uint16_t kMaximumThrottle = 2047;

// Unidirectional DShot: telemetry bit 0, normal (non-inverted) CRC.
// 0: motor stop; 1..47: reserved ESC commands; 48..2047: throttle.
inline uint16_t frame(uint16_t throttle, bool telemetry = false) {
    if (throttle > kMaximumThrottle) throttle = kMaximumThrottle;
    const uint16_t payload = static_cast<uint16_t>((throttle << 1) | (telemetry ? 1u : 0u));
    const uint16_t checksum = static_cast<uint16_t>((payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0fu);
    return static_cast<uint16_t>((payload << 4) | checksum);
}

// Preserve the old PWM output's approximate normalized ESC range:
// the old duty 0..1 mapped to 1100..1900 us (assuming ESC 1000..2000 us).
// A positive duty is a running command; zero/negative/nonfinite stop.
// This is an approximation, NOT guaranteed to match ESC PWM motor RPM.
inline uint16_t throttle_from_legacy_duty(float duty) {
    if (!std::isfinite(duty) || duty <= 0.0f) return kStop;
    if (duty > 1.0f) duty = 1.0f;
    const float normalized = 0.10f + 0.80f * duty;
    const float value = static_cast<float>(kMinimumThrottle)
                      + static_cast<float>(kMaximumThrottle - kMinimumThrottle) * normalized;
    uint16_t command = static_cast<uint16_t>(value + 0.5f);
    if (command < kMinimumThrottle) command = kMinimumThrottle;
    if (command > kMaximumThrottle) command = kMaximumThrottle;
    return command;
}

// PIO OUT shifts left with autopull threshold 16. Put the 16 packet bits at
// the most significant end of the 32-bit OSR, MSB first.
inline uint32_t pio_word(uint16_t command) {
    return static_cast<uint32_t>(frame(command)) << 16;
}
} // namespace dshot

#endif
