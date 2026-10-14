#include "dshot_protocol.hpp"
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>

int main() {
    using namespace dshot;
    static_assert(kPioClockHz == 2400000, "DShot300 PIO clock");
    static_assert(kPioCyclesPerBit == 8, "Correct 1/0 duty widths");
    assert(frame(0) == 0);
    assert(frame(48) == 0x0606);
    assert(frame(2047) == 0xFFEE);
    assert(frame(2047, true) == 0xFFFF); // Verify CRC includes telemetry bit.
    for (unsigned throttle = 0; throttle <= 2047; ++throttle) {
        uint16_t f = frame(static_cast<uint16_t>(throttle));
        uint16_t payload = f >> 4;
        assert(((payload ^ (payload >> 4) ^ (payload >> 8)) & 15) == (f & 15));
        assert((f >> 5) == throttle);
        assert((f & 16) == 0); // Telemetry bit remains zero in normal operation.
    }
    assert(throttle_from_legacy_duty(0.0f) == kStop);
    assert(throttle_from_legacy_duty(-1.0f) == kStop);
    assert(throttle_from_legacy_duty(NAN) == kStop);
    assert(throttle_from_legacy_duty(INFINITY) == kStop);
    // Every running DShot value 48..2047 must be reachable from the
    // existing motor mixer's 0.10..0.95 duty interval.
    assert(throttle_from_legacy_duty(kLegacyMinimumRunningDuty) == kMinimumThrottle);
    assert(throttle_from_legacy_duty(kLegacyMaximumRunningDuty) == kMaximumThrottle);
    assert(throttle_from_legacy_duty(0.001f) == kMinimumThrottle);
    assert(throttle_from_legacy_duty(1.0f) == kMaximumThrottle);
    assert(throttle_from_legacy_duty(10.0f) == kMaximumThrottle);
    assert(throttle_from_legacy_duty(0.525f) >= 1047);
    assert(throttle_from_legacy_duty(0.525f) <= 1048);

    uint16_t previous = 0;
    for (int i = 0; i <= 10000; ++i) {
        const float duty = kLegacyMinimumRunningDuty +
            (kLegacyMaximumRunningDuty - kLegacyMinimumRunningDuty) * i / 10000.0f;
        const uint16_t command = throttle_from_legacy_duty(duty);
        assert(command >= kMinimumThrottle && command <= kMaximumThrottle);
        assert(command >= previous);
        previous = command;
    }
    // Test all 2000 running values, including boundaries, for gaps.
    for (uint16_t cmd = kMinimumThrottle; cmd <= kMaximumThrottle; ++cmd) {
        const float position = static_cast<float>(cmd - kMinimumThrottle) /
            static_cast<float>(kMaximumThrottle - kMinimumThrottle);
        const float duty = kLegacyMinimumRunningDuty +
            (kLegacyMaximumRunningDuty - kLegacyMinimumRunningDuty) * position;
        assert(throttle_from_legacy_duty(duty) == cmd);
    }
    assert(pio_word(kStop) == 0);
    assert(pio_word(48) == 0x06060000u);
    std::puts("DShot300 encoding, CRC, stop and throttle mapping passed");
}
