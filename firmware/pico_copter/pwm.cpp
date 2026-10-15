/*
 * One-way DShot300 on RP2040 / RP2350: PIO sends one normal (not
 * inverted) DShot packet per motor after each 400 Hz control tick.
 * NO bidirectional DShot, NO RX/turnaround, NO eRPM reading.
 *
 * Retains the original GPIO pinout, servo PWM, 400 Hz timer, and all
 * flight control / arm / failsafe / mixer behavior. No ESC is needed to
 * complete a frame. STOP is DShot command 0 at every disarmed tick.
 */
#include "pwm.hpp"
#include "dshot_protocol.hpp"
#include "dshot_tx.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"
#include "hardware/sync.h"

uint8_t ESC_calib = 0; // No PWM ESC calibration for DShot.

namespace {
constexpr uint16_t PWM_WRAP = 3124;
constexpr uint32_t PWM_FREQUENCY_HZ = 400;
constexpr uint16_t SERVO_INITIAL_LEVEL = 625;
constexpr uint MOTOR_COUNT = 4;
constexpr uint MOTOR_PINS[MOTOR_COUNT] = {
    PWM_PIN_FR, PWM_PIN_FL, PWM_PIN_RR, PWM_PIN_RL
};
static_assert(dshot::kBitRate == 300000 &&
              dshot::kPioCyclesPerBit == 8,
              "One-way DShot300 PIO requires 8 cycles per bit");
static_assert(400 * 2500 == 1000000, "Preserve 400 Hz control");

PIO dshot_pio = pio0;
uint motor_sm[MOTOR_COUNT]{};
bool dshot_ready = false;
float requested_duty[MOTOR_COUNT]{};

struct MotorTxStats {
    uint32_t tx = 0;
    uint32_t tx_busy = 0;
    uint16_t last_command = 0;
    uint16_t last_frame = 0;
};
MotorTxStats tx_stats[MOTOR_COUNT]{};

void init_oneway_dshot() {
    const uint offset = pio_add_program(dshot_pio, &dshot_tx_program);
    const float divisor = static_cast<float>(clock_get_hz(clk_sys)) /
                          static_cast<float>(dshot::kPioClockHz);
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        const uint sm = pio_claim_unused_sm(dshot_pio, true);
        const uint pin = MOTOR_PINS[i];
        motor_sm[i] = sm;
        pio_sm_config cfg = dshot_tx_program_get_default_config(offset);
        sm_config_set_sideset_pins(&cfg, pin);
        sm_config_set_out_shift(&cfg, false, true, 16); // MSB first, autopull 16 bits
        sm_config_set_fifo_join(&cfg, PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&cfg, divisor);
        pio_gpio_init(dshot_pio, pin);
        pio_sm_init(dshot_pio, sm, offset, &cfg);
        // Idle LOW is driven by PIO side-set while waiting for the TX FIFO.
        pio_sm_set_consecutive_pindirs(dshot_pio, sm, pin, 1, true);
    }
    for (uint i = 0; i < MOTOR_COUNT; ++i)
        pio_sm_set_enabled(dshot_pio, motor_sm[i], true);
    dshot_ready = true;
}

void send_next_frames() {
    if (!dshot_ready) return;
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        const uint sm = motor_sm[i];
        if (!pio_sm_is_tx_fifo_empty(dshot_pio, sm)) {
            // Never block the control ISR; drop stale queued frames so a
            // STOP instruction is not delayed behind old motor commands.
            pio_sm_clear_fifos(dshot_pio, sm);
            ++tx_stats[i].tx_busy;
        }
        const uint16_t command =
            dshot::throttle_from_legacy_duty(requested_duty[i]);
        const uint16_t packet = dshot::frame(command, false);
        pio_sm_put(dshot_pio, sm, dshot::pio_word(command));
        ++tx_stats[i].tx;
        tx_stats[i].last_command = command;
        tx_stats[i].last_frame = packet;
    }
}

void pwm_wrap_irq_handler() {
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN));
    MAINLOOP();
    send_next_frames(); // TX only, no ESC feedback wait.
}
} // namespace

DShotTxSnapshot dshot_tx_snapshot() {
    DShotTxSnapshot snapshot{};
    const uint32_t saved = save_and_disable_interrupts();
    for (uint i = 0; i < MOTOR_COUNT; ++i) {
        auto& m = snapshot.motors[i];
        m.tx = tx_stats[i].tx;
        m.tx_busy = tx_stats[i].tx_busy;
        m.requested_duty = requested_duty[i];
        m.last_command = tx_stats[i].last_command;
        m.last_frame = tx_stats[i].last_frame;
    }
    restore_interrupts(saved);
    return snapshot;
}

void pwm_init() {
    static_assert(PWM_PIN_FR != PWM_PIN_FL && PWM_PIN_FR != PWM_PIN_RR &&
                  PWM_PIN_FR != PWM_PIN_RL && PWM_PIN_FL != PWM_PIN_RR &&
                  PWM_PIN_FL != PWM_PIN_RL && PWM_PIN_RR != PWM_PIN_RL,
                  "Each DShot motor requires a distinct GPIO");
    static_assert(PWM_PIN_SERVO != PWM_PIN_FR && PWM_PIN_SERVO != PWM_PIN_FL &&
                  PWM_PIN_SERVO != PWM_PIN_RR && PWM_PIN_SERVO != PWM_PIN_RL,
                  "Servo PWM must not share motor GPIO");

    gpio_set_function(PWM_PIN_SERVO, GPIO_FUNC_PWM);
    const uint servo_slice = pwm_gpio_to_slice_num(PWM_PIN_SERVO);
    const uint mainloop_slice = pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN);
    if (servo_slice == mainloop_slice) panic("Servo/control slice collision");

    // Compute divider for 125 MHz RP2040 and 150 MHz RP2350 independently.
    const float pwm_clkdiv = static_cast<float>(clock_get_hz(clk_sys)) /
                             (static_cast<float>(PWM_WRAP + 1u) * PWM_FREQUENCY_HZ);
    if (pwm_clkdiv < 1.0f || pwm_clkdiv >= 256.0f)
        panic("Unsupported system clock for 400 Hz");

    pwm_set_wrap(servo_slice, PWM_WRAP);
    pwm_set_clkdiv(servo_slice, pwm_clkdiv);
    pwm_set_gpio_level(PWM_PIN_SERVO, SERVO_INITIAL_LEVEL);

    init_oneway_dshot();

    // PWM slice 7 remains a pure 400 Hz IRQ timer, not a GPIO14 output.
    pwm_set_wrap(mainloop_slice, PWM_WRAP);
    pwm_set_clkdiv(mainloop_slice, pwm_clkdiv);
    pwm_clear_irq(mainloop_slice);
    pwm_set_irq_enabled(mainloop_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);
    pwm_set_enabled(servo_slice, true);
    pwm_set_enabled(mainloop_slice, true);

    sleep_ms(3000);
    pwm_set_gpio_level(PWM_PIN_SERVO, SERVO_INITIAL_LEVEL);
    sleep_ms(1000);
}

void set_duty_fr(float duty) { requested_duty[0] = duty; }
void set_duty_fl(float duty) { requested_duty[1] = duty; }
void set_duty_rr(float duty) { requested_duty[2] = duty; }
void set_duty_rl(float duty) { requested_duty[3] = duty; }
