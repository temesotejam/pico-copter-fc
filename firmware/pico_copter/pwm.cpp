/*
 * pwm.cpp - PIO DShot300 for four ESCs, hardware PWM for servo and
 * a free-running PWM slice 7 wrap interrupt for the ORIGINAL 400 Hz loop.
 * The timer no longer drives GPIO14 (GPIO14 is a PIO data output).
 */
#include "pwm.hpp"
#include "dshot_protocol.hpp"
#include "dshot_tx.pio.h"
#include "hardware/pio.h"
#include "hardware/clocks.h"

uint8_t ESC_calib=0; // Legacy flag; ESC PWM calibration is not used by DShot.

namespace {
constexpr uint16_t PWM_WRAP=3124;
constexpr float PWM_CLKDIV=100.0f;
constexpr uint16_t SERVO_INITIAL_LEVEL=625;
constexpr uint MOTOR_COUNT=4;
constexpr uint MOTOR_PINS[MOTOR_COUNT]={
    PWM_PIN_FR, PWM_PIN_FL, PWM_PIN_RR, PWM_PIN_RL
};
PIO dshot_pio=pio0;
uint dshot_sm[MOTOR_COUNT]{};
bool dshot_initialized=false;
volatile uint32_t dshot_fifo_overruns=0;

static void init_dshot_output() {
    const uint offset=pio_add_program(dshot_pio,&dshot_tx_program);
    const float clkdiv=static_cast<float>(clock_get_hz(clk_sys))
                       / static_cast<float>(dshot::kPioClockHz);
    for (uint i=0;i<MOTOR_COUNT;++i) {
        const uint sm=pio_claim_unused_sm(dshot_pio,true);
        dshot_sm[i]=sm;
        const uint pin=MOTOR_PINS[i];
        pio_sm_config cfg=dshot_tx_program_get_default_config(offset);
        sm_config_set_sideset_pins(&cfg,pin);
        sm_config_set_out_shift(&cfg,false,true,16); // MSB first, 16-bit autopull
        sm_config_set_fifo_join(&cfg,PIO_FIFO_JOIN_TX);
        sm_config_set_clkdiv(&cfg,clkdiv);
        pio_gpio_init(dshot_pio,pin);
        pio_sm_init(dshot_pio,sm,offset,&cfg);
        pio_sm_set_consecutive_pindirs(dshot_pio,sm,pin,1,true);
    }
    // Enable only after every GPIO and state machine has been initialized.
    for (uint i=0;i<MOTOR_COUNT;++i) {
        pio_sm_set_enabled(dshot_pio,dshot_sm[i],true);
    }
    dshot_initialized=true;
}

static inline void send_dshot(uint channel,float duty) {
    if (!dshot_initialized) return;
    const uint sm=dshot_sm[channel];
    const uint16_t command=dshot::throttle_from_legacy_duty(duty);
    // Avoid IRQ blocking and prevent stale queued motor commands.
    // At 400 Hz this FIFO should always be empty (DShot300 takes 53 us).
    if (!pio_sm_is_tx_fifo_empty(dshot_pio,sm)) {
        pio_sm_clear_fifos(dshot_pio,sm);
        ++dshot_fifo_overruns;
    }
    pio_sm_put(dshot_pio,sm,dshot::pio_word(command));
}

static void pwm_wrap_irq_handler() {
    // Preserve the exact 400 Hz timer source (PWM slice of GPIO14), even
    // though the actual motor signal GPIO14 is muxed to PIO0.
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN));
    MAINLOOP();
}
} // namespace

void pwm_init() {
    static_assert(PWM_PIN_FR != PWM_PIN_FL && PWM_PIN_FR != PWM_PIN_RR
                  && PWM_PIN_FR != PWM_PIN_RL && PWM_PIN_FL != PWM_PIN_RR
                  && PWM_PIN_FL != PWM_PIN_RL && PWM_PIN_RR != PWM_PIN_RL,
                  "DShot motor GPIO assignments must be unique");
    static_assert(PWM_PIN_SERVO != PWM_PIN_FR && PWM_PIN_SERVO != PWM_PIN_FL
                  && PWM_PIN_SERVO != PWM_PIN_RR && PWM_PIN_SERVO != PWM_PIN_RL,
                  "Servo PWM pin cannot be a DShot pin");

    // Servo is the only GPIO that actually uses PWM output function.
    gpio_set_function(PWM_PIN_SERVO,GPIO_FUNC_PWM);
    const uint servo_slice=pwm_gpio_to_slice_num(PWM_PIN_SERVO);
    const uint mainloop_slice=pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN);
    if (servo_slice==mainloop_slice) {
        panic("Servo and 400 Hz timebase must use distinct PWM slices");
    }
    pwm_set_wrap(servo_slice,PWM_WRAP);
    pwm_set_clkdiv(servo_slice,PWM_CLKDIV);
    pwm_set_gpio_level(PWM_PIN_SERVO,SERVO_INITIAL_LEVEL);

    // DShot owns all four motor signal pins. No extra CPU timer or DMA.
    init_dshot_output();

    // A free-running PWM slice remains the original 2.5 ms control timer.
    // No GPIO has to be configured in PWM mode for this slice to run.
    pwm_set_wrap(mainloop_slice,PWM_WRAP);
    pwm_set_clkdiv(mainloop_slice,PWM_CLKDIV);
    pwm_clear_irq(mainloop_slice);
    pwm_set_irq_enabled(mainloop_slice,true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP,pwm_wrap_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP,true);
    pwm_set_enabled(servo_slice,true);
    pwm_set_enabled(mainloop_slice,true);

    // The 400 Hz ISR emits DShot stop=0 while the controller is not armed.
    // Preserve the previous startup delays; DShot needs no ESC calibration.
    sleep_ms(3000);
    pwm_set_gpio_level(PWM_PIN_SERVO,SERVO_INITIAL_LEVEL);
    sleep_ms(1000);
}

void set_duty_fr(float duty) { send_dshot(0,duty); }
void set_duty_fl(float duty) { send_dshot(1,duty); }
void set_duty_rr(float duty) { send_dshot(2,duty); }
void set_duty_rl(float duty) { send_dshot(3,duty); }
