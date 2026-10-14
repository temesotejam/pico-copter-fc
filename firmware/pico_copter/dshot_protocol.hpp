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

inline uint16_t bidirectional_frame(uint16_t throttle) {
    if (throttle > kMaximumThrottle) throttle = kMaximumThrottle;
    const uint16_t payload = static_cast<uint16_t>(throttle << 1);
    const uint16_t checksum = static_cast<uint16_t>(
        ~(payload ^ (payload >> 4) ^ (payload >> 8)) & 0x0fu);
    return static_cast<uint16_t>((payload << 4) | checksum);
}

// The PIO library sends the inverted bitstream, MSB first.
inline uint32_t bidirectional_pio_word(uint16_t throttle) {
    return static_cast<uint32_t>(~bidirectional_frame(throttle) & 0xffffu) << 16;
}

// In bidirectional DShot, telemetry arrives on the motor signal wire.
// The outgoing frame uses the INVERTED CRC nibble (unlike one-way DShot).

// Use the entire running-throttle range of DShot300. The EXISTING mixer
// clamps each running motor's duty to 0.10..0.95 in control.cpp. Map those
// endpoints to DShot 48..2047 without changing PID, mixing, or arm/stop rules.
// Zero/negative/nonfinite duty is a motor STOP (DShot 0), never a command 1..47.
// This deliberately changes motor speed response relative to prior PWM/DShot
// versions; check behavior WITHOUT propellers before attempting a flight.
constexpr float kLegacyMinimumRunningDuty = 0.10f;
constexpr float kLegacyMaximumRunningDuty = 0.95f;
inline uint16_t throttle_from_legacy_duty(float duty) {
    if (!std::isfinite(duty) || duty <= 0.0f) return kStop;
    if (duty <= kLegacyMinimumRunningDuty) return kMinimumThrottle;
    if (duty >= kLegacyMaximumRunningDuty) return kMaximumThrottle;

    const float position = (duty - kLegacyMinimumRunningDuty) /
        (kLegacyMaximumRunningDuty - kLegacyMinimumRunningDuty);
    const float value = static_cast<float>(kMinimumThrottle) +
        static_cast<float>(kMaximumThrottle - kMinimumThrottle) * position;
    return static_cast<uint16_t>(value + 0.5f);
}

// PIO OUT shifts left with autopull threshold 16. Put the 16 packet bits at
// the most significant end of the 32-bit OSR, MSB first.
inline uint32_t pio_word(uint16_t command) {
    return static_cast<uint32_t>(frame(command)) << 16;
}
} // namespace dshot

#endif
