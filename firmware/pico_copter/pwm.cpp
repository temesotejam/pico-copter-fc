/*
 * Bidirectional DShot300 / eRPM on Raspberry Pi Pico (RP2040).
 * Direct connection to ESC signal, without a one-way level shifter.
 *
 * The 400 Hz IRQ computes motor commands, then queues bidirectional
 * DShot300 frames. The previous cycle's eRPM replies are read WITHOUT
 * waiting for the ESC. Telemetry is USB diagnostics only, never feedback
 * for PID, EKF or the motor mixer.
 *
 * PIO TX/RX timing is identical to the verified 2026-10-16 RP2040
 * baseline. Missing, malformed or delayed ESC replies never affect motor
 * commands. This is always BIDIRECTIONAL DShot (not a protocol fallback).
 *
 * TX/RX PIO program adapted from simonwunderlich/pio-dshot (MIT):
 * https://github.com/simonwunderlich/pio-dshot
 */
#include "pwm.hpp"
#include "dshot_protocol.hpp"
#include "dshot_bidir_decode.hpp"
#include "dshot_tx.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

uint8_t ESC_calib = 0; // DShot does not require PWM throttle calibration.

namespace {
constexpr uint16_t PWM_WRAP = 3124;
// Compute the PWM-slice divider from the actual system clock so the
// 400 Hz control timer stays unchanged on Pico (and builds for Pico 2).
constexpr uint32_t PWM_FREQUENCY_HZ = 400;
constexpr uint16_t SERVO_INITIAL_LEVEL = 625;
constexpr uint MOTOR_COUNT = 4;
// IMPORTANT: channel index order is FR, FL, RR, RL throughout.
constexpr uint MOTOR_PINS[MOTOR_COUNT] = {
    PWM_PIN_FR, PWM_PIN_FL, PWM_PIN_RR, PWM_PIN_RL
};
// Bidirectional PIO is 40 cycles/bit (NOT the older one-way PIO's 8).
constexpr uint32_t PIO_CYCLES_PER_BIT = 40;
constexpr uint32_t RX_TURNAROUND_CYCLES = 25u * dshot::kBitRate
                                              * PIO_CYCLES_PER_BIT / 1000000u;
constexpr uint32_t ERPM_STALE_US = 250000; // Telemetry display only.
static_assert(RX_TURNAROUND_CYCLES == 300, "Wrong DShot300 turnaround");
static_assert(400 * 2500 == 1000000, "400 Hz scheduling invariant");

PIO dshot_pio = pio0;
uint sm_for_motor[MOTOR_COUNT]{};
bool dshot_ready = false;
float requested_duty[MOTOR_COUNT]{};
bool motor_sent[MOTOR_COUNT]{};

struct MotorStats {
    uint32_t erpm = 0;
    uint32_t last_good_us = 0;
    uint32_t tx = 0;
    uint32_t rx_ok = 0;
    uint32_t rx_timeout = 0;
    uint32_t rx_bad_gcr = 0;
    uint32_t rx_bad_crc = 0;
    uint32_t rx_other = 0;
    uint32_t rx_discarded = 0; // Unpaired/excess responses drained from FIFO.
    uint32_t tx_busy = 0;
    uint16_t last_command = 0;
    uint16_t last_frame = 0;
};
MotorStats telemetry[MOTOR_COUNT]{};

void init_bidir_dshot() {
    const uint offset = pio_add_program(dshot_pio, &dshot_tx_program);
    const float divisor = static_cast<float>(clock_get_hz(clk_sys)) /
                          static_cast<float>(dshot::kBitRate * PIO_CYCLES_PER_BIT);
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        const uint sm = pio_claim_unused_sm(dshot_pio, true);
        const uint pin = MOTOR_PINS[i];
        sm_for_motor[i] = sm;
        pio_sm_config cfg = dshot_tx_program_get_default_config(offset);
        sm_config_set_out_pins(&cfg, pin, 1);
        sm_config_set_set_pins(&cfg, pin, 1);
        sm_config_set_in_pins(&cfg, pin);
        sm_config_set_jmp_pin(&cfg, pin);
        sm_config_set_out_shift(&cfg, false, false, 32); // Left shift; manual pull
        sm_config_set_in_shift(&cfg, false, false, 32);
        sm_config_set_clkdiv(&cfg, divisor);
        pio_gpio_init(dshot_pio, pin);
        gpio_set_pulls(pin, true, false);
        pio_sm_init(dshot_pio, sm, offset, &cfg);
        pio_sm_set_pins(dshot_pio, sm, 1u); // Bidir line idle HIGH.
        pio_sm_set_consecutive_pindirs(dshot_pio, sm, pin, 1, true);
    }
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        pio_sm_set_enabled(dshot_pio, sm_for_motor[i], true);
    }
    dshot_ready = true;
}

// This runs once per 400 Hz tick, never blocking on a missing reply.
// Drain every motor's RX FIFO, including channels whose previous TX was
// skipped. Otherwise a late/unpaired response could remain indefinitely
// and cause start_next_exchange() to skip ALL subsequent commands.
void collect_previous_telemetry() {
    const uint32_t now_us = time_us_32();
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        const uint sm = sm_for_motor[i];
        MotorStats& stat = telemetry[i];
        const bool reply_expected = motor_sent[i];
        motor_sent[i] = false;
        bool response_collected = false;

        // A PIO RX FIFO holds at most four words. Any additional response
        // is stale; never allow it to block a future motor command.
        for (uint j = 0; j < 4u && !pio_sm_is_rx_fifo_empty(dshot_pio, sm); ++j) {
            const uint32_t response = pio_sm_get(dshot_pio, sm);
            if (!reply_expected || response_collected) {
                ++stat.rx_discarded;
                continue;
            }
            response_collected = true;
            const dshot::ErpmResult result = dshot::decode_erpm(response);
            switch (result.result) {
            case dshot::RxResult::Erpm:
                stat.erpm = result.erpm;
                stat.last_good_us = now_us;
                ++stat.rx_ok;
                break;
            case dshot::RxResult::NoResponse: ++stat.rx_timeout; break;
            case dshot::RxResult::BadGcr: ++stat.rx_bad_gcr; break;
            case dshot::RxResult::BadChecksum: ++stat.rx_bad_crc; break;
            case dshot::RxResult::OtherTelemetry: ++stat.rx_other; break;
            }
        }
        if (reply_expected && !response_collected)
            ++stat.rx_timeout; // ESC missing / not powered / no telemetry
    }
}

void start_next_exchange() {
    if (!dshot_ready) return;
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        const uint sm = sm_for_motor[i];
        // Ignore and drain any unpaired late RX data before queuing TX.
        // DShot replies are optional and cannot hold up the output path.
        for (uint j = 0; j < 4u && !pio_sm_is_rx_fifo_empty(dshot_pio, sm); ++j) {
            (void)pio_sm_get(dshot_pio, sm);
            ++telemetry[i].rx_discarded;
        }
        if (!pio_sm_is_tx_fifo_empty(dshot_pio, sm)) {
            // A genuinely stalled TX state machine is recorded separately.
            // Never block the ISR waiting for it.
            ++telemetry[i].tx_busy;
            continue;
        }
        const uint16_t throttle = dshot::throttle_from_legacy_duty(requested_duty[i]);
        pio_sm_put(dshot_pio, sm, dshot::bidirectional_pio_word(throttle));
        pio_sm_put(dshot_pio, sm, RX_TURNAROUND_CYCLES);
        ++telemetry[i].tx;
        telemetry[i].last_command = throttle;
        telemetry[i].last_frame = dshot::bidirectional_frame(throttle);
        motor_sent[i] = true;
    }
}

void pwm_wrap_irq_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN));
    // Existing 400Hz motor and SBUS/failsafe calculations run unchanged.
    MAINLOOP();
    // Read the previous cycle's RX FIFO only; do not wait for the ESC.
    collect_previous_telemetry();
    start_next_exchange();
}
} // namespace

DShotTelemetrySnapshot dshot_telemetry_snapshot(void) {
    DShotTelemetrySnapshot result{};
    const uint32_t saved = save_and_disable_interrupts();
    const uint32_t now_us = time_us_32();
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        const MotorStats& s = telemetry[i];
        auto& dest = result.motors[i];
        dest.erpm = s.erpm;
        dest.tx = s.tx;
        dest.rx_ok = s.rx_ok;
        dest.rx_timeout = s.rx_timeout;
        dest.rx_bad_gcr = s.rx_bad_gcr;
        dest.rx_bad_crc = s.rx_bad_crc;
        dest.rx_other = s.rx_other;
        dest.rx_discarded = s.rx_discarded;
        dest.tx_busy = s.tx_busy;
        dest.requested_duty = requested_duty[i];
        dest.last_command = s.last_command;
        dest.last_frame = s.last_frame;
        dest.fresh = s.rx_ok != 0 &&
                     static_cast<uint32_t>(now_us - s.last_good_us) <= ERPM_STALE_US;
    }
    restore_interrupts(saved);
    return result;
}

void pwm_init() {
    static_assert(PWM_PIN_FR != PWM_PIN_FL && PWM_PIN_FR != PWM_PIN_RR
        && PWM_PIN_FR != PWM_PIN_RL && PWM_PIN_FL != PWM_PIN_RR
        && PWM_PIN_FL != PWM_PIN_RL && PWM_PIN_RR != PWM_PIN_RL,
        "DShot motor GPIOs must be unique");
    static_assert(PWM_PIN_SERVO != PWM_PIN_FR && PWM_PIN_SERVO != PWM_PIN_FL
        && PWM_PIN_SERVO != PWM_PIN_RR && PWM_PIN_SERVO != PWM_PIN_RL,
        "Servo GPIO must be distinct from DShot GPIOs");

    // Preserve servo PWM. Preserve PWM slice 7's 400 Hz wrap IRQ,
    // even though GPIO14 now routes to PIO, not the PWM peripheral.
    gpio_set_function(PWM_PIN_SERVO, GPIO_FUNC_PWM);
    const uint servo_slice = pwm_gpio_to_slice_num(PWM_PIN_SERVO);
    const uint mainloop_slice = pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN);
    if (servo_slice == mainloop_slice) panic("Servo/control PWM slice collision");
    const float pwm_clkdiv = static_cast<float>(clock_get_hz(clk_sys)) /
                             (static_cast<float>(PWM_WRAP + 1u) * PWM_FREQUENCY_HZ);
    if (pwm_clkdiv < 1.0f || pwm_clkdiv >= 256.0f) {
        panic("Unsupported clock for 400 Hz PWM timebase");
    }
    pwm_set_wrap(servo_slice, PWM_WRAP);
    pwm_set_clkdiv(servo_slice, pwm_clkdiv);
    pwm_set_gpio_level(PWM_PIN_SERVO, SERVO_INITIAL_LEVEL);

    init_bidir_dshot();

    pwm_set_wrap(mainloop_slice, PWM_WRAP);
    pwm_set_clkdiv(mainloop_slice, pwm_clkdiv);
    pwm_clear_irq(mainloop_slice);
    pwm_set_irq_enabled(mainloop_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_set_enabled(servo_slice, true);
    pwm_set_enabled(mainloop_slice, true);

    // Preserve legacy initialization delay and output stop during disarm.
    sleep_ms(3000);
    pwm_set_gpio_level(PWM_PIN_SERVO, SERVO_INITIAL_LEVEL);
    sleep_ms(1000);
}

// Existing interface. These update 4 desired throttle commands only; actual
// DShot TX/RX happens together after MAINLOOP() every 2.5 ms.
void set_duty_fr(float duty) { requested_duty[0] = duty; }
void set_duty_fl(float duty) { requested_duty[1] = duty; }
void set_duty_rr(float duty) { requested_duty[2] = duty; }
void set_duty_rl(float duty) { requested_duty[3] = duty; }
