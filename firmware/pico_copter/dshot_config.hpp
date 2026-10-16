#ifndef PICO_COPTER_DSHOT_CONFIG_HPP
#define PICO_COPTER_DSHOT_CONFIG_HPP

#include <stdint.h>

// ============================================================================
// DShot communication rate: change JUST THIS VALUE and rebuild the Pico UF2.
//   150 = DShot150; 300 = DShot300 (RP2040 hardware-tested);
//   600 = DShot600; 1200 = DShot1200 (higher rates need ESC/oscilloscope test).
// This remains BIDIRECTIONAL DShot at every speed. It does NOT change the
// 400 Hz motor update period or PID/EKF control loops.
//
// A build can optionally override this with -DDSHOT_RATE_KBPS=600.
// ============================================================================
#ifndef DSHOT_RATE_KBPS
#define DSHOT_RATE_KBPS 300
#endif

static_assert(DSHOT_RATE_KBPS == 150 || DSHOT_RATE_KBPS == 300 ||
              DSHOT_RATE_KBPS == 600 || DSHOT_RATE_KBPS == 1200,
              "Unsupported DShot speed: choose 150, 300, 600, or 1200.");

namespace dshot {
constexpr uint32_t kRateKbps = DSHOT_RATE_KBPS;
constexpr uint32_t kBitRate = kRateKbps * 1000u;

// The proven PIO TX loop takes 40 PIO instructions/cycles per 1 DShot bit.
// Changing this value alone would change the waveform and RX sampling.
// Keep it at 40: PIO CLKDIV, TX, and RX all scale with kBitRate.
constexpr uint32_t kPioCyclesPerBit = 40;
constexpr uint32_t kPioClockHz = kBitRate * kPioCyclesPerBit;

// Host sends this loop count into the second PIO TX word. PIO executes
// roughly one iteration per clock; integer values for all four speeds.
// At DShot300 the count remains EXACTLY 300 (previous verified firmware).
constexpr uint32_t kTurnaroundUs = 25;
constexpr uint32_t kTurnaroundCycles =
    kTurnaroundUs * kPioClockHz / 1000000u;

static_assert(kPioClockHz >= 6000000u && kPioClockHz <= 48000000u,
              "PIO clock outside tested DShot configuration limits.");
static_assert(kTurnaroundCycles == kRateKbps,
              "Unexpected PIO turnaround calculation.");

// Scale the PIO reply-start search to roughly 43 us at every DShot speed.
// The existing PIO program contains SET Y,7. At the proven DShot300
// setting, kRxWaitOuterY is 7 and its instruction is left unchanged.
// At other speeds, patch only that SET instruction in PIO memory BEFORE
// starting state machines (150:3, 300:7, 600:15, 1200:31).
constexpr uint32_t kRxWaitOuterY = (8u * kRateKbps / 300u) - 1u;
static_assert(kRxWaitOuterY <= 31u, "RX wait loop must fit 5-bit PIO SET");


} // namespace dshot
#endif
