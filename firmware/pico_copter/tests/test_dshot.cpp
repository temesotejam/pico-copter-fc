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
    assert(throttle_from_legacy_duty(0.1f) >= kMinimumThrottle);
    assert(throttle_from_legacy_duty(0.95f) < kMaximumThrottle);
    assert(throttle_from_legacy_duty(0.95f) > throttle_from_legacy_duty(0.10f));
    assert(throttle_from_legacy_duty(10.0f) <= kMaximumThrottle);
    assert(pio_word(kStop) == 0);
    assert(pio_word(48) == 0x06060000u);
    std::puts("DShot300 encoding, CRC, stop and throttle mapping passed");
}
