/*
 * pwm.cpp
*/
#include "pwm.hpp"

//グローバル変数
uint8_t ESC_calib=0;

namespace {
constexpr uint16_t PWM_WRAP = 3124;
constexpr float PWM_CLKDIV = 100.0f;
constexpr uint16_t SERVO_INITIAL_LEVEL = 625;

const uint PWM_OUTPUT_PINS[] = {
    PWM_PIN_RL,
    PWM_PIN_RR,
    PWM_PIN_FL,
    PWM_PIN_FR,
    PWM_PIN_SERVO,
};

const uint MOTOR_PWM_PINS[] = {
    PWM_PIN_RL,
    PWM_PIN_RR,
    PWM_PIN_FL,
    PWM_PIN_FR,
};

bool same_pwm_output(uint gpio_a, uint gpio_b)
{
    return pwm_gpio_to_slice_num(gpio_a) == pwm_gpio_to_slice_num(gpio_b)
        && pwm_gpio_to_channel(gpio_a) == pwm_gpio_to_channel(gpio_b);
}

void validate_pwm_pin_assignments()
{
    const size_t pin_count = sizeof(PWM_OUTPUT_PINS) / sizeof(PWM_OUTPUT_PINS[0]);

    for (size_t i = 0; i < pin_count; ++i)
    {
        for (size_t j = i + 1; j < pin_count; ++j)
        {
            if (same_pwm_output(PWM_OUTPUT_PINS[i], PWM_OUTPUT_PINS[j]))
            {
                panic("PWM GPIO %u and GPIO %u map to the same PWM slice/channel",
                      PWM_OUTPUT_PINS[i], PWM_OUTPUT_PINS[j]);
            }
        }
    }
}

void configure_pwm_pin(uint gpio)
{
    gpio_set_function(gpio, GPIO_FUNC_PWM);

    const uint slice_num = pwm_gpio_to_slice_num(gpio);
    pwm_set_wrap(slice_num, PWM_WRAP);
    pwm_set_clkdiv(slice_num, PWM_CLKDIV);
}

void enable_pwm_pin(uint gpio)
{
    pwm_set_enabled(pwm_gpio_to_slice_num(gpio), true);
}

void set_all_motor_levels(uint16_t level)
{
    const size_t motor_count = sizeof(MOTOR_PWM_PINS) / sizeof(MOTOR_PWM_PINS[0]);
    for (size_t i = 0; i < motor_count; ++i)
    {
        pwm_set_gpio_level(MOTOR_PWM_PINS[i], level);
    }
}

uint16_t motor_duty_to_level(float duty)
{
    if (duty > 0.97f) duty = 0.97f;
    if (duty < 0.01f) duty = 0.01f;

    float level = (float)(DUTYMAX - DUTYMIN) * duty + DUTYMIN;
    if (level > DUTYMAX) level = DUTYMAX;
    if (level < DUTYMIN) level = DUTYMIN;

    return (uint16_t)level;
}

void pwm_wrap_irq_handler()
{
    // Keep PWM-specific IRQ handling in this module. The control loop does not
    // need to know which slice is used as its 400 Hz timing source.
    pwm_clear_irq(pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN));
    MAINLOOP();
}
} // namespace

void pwm_init()
{
    validate_pwm_pin_assignments();

    // Configure every PWM GPIO from its pin number. Slice/channel selection is
    // intentionally not hard-coded so future pin changes only require editing
    // PWM_PIN_* in pwm.hpp.
    const size_t pin_count = sizeof(PWM_OUTPUT_PINS) / sizeof(PWM_OUTPUT_PINS[0]);
    for (size_t i = 0; i < pin_count; ++i)
    {
        configure_pwm_pin(PWM_OUTPUT_PINS[i]);
    }

    // Current setting:
    // T = (wrap + 1) * clkdiv / sysclock
    //   = 3125 * 100 / 125 MHz = 2.5 ms = 400 Hz
    const uint mainloop_slice = pwm_gpio_to_slice_num(PWM_MAINLOOP_PIN);
    pwm_clear_irq(mainloop_slice);
    pwm_set_irq_enabled(mainloop_slice, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, pwm_wrap_irq_handler);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // ESC initial/calibration level.
    set_all_motor_levels(ESC_calib == 1 ? DUTYMAX : DUTYMIN);

    // Enable all slices used by the configured GPIOs. Calling this more than
    // once for two pins on the same slice is harmless.
    for (size_t i = 0; i < pin_count; ++i)
    {
        enable_pwm_pin(PWM_OUTPUT_PINS[i]);
    }

    sleep_ms(3000);

    set_all_motor_levels(DUTYMIN);
    pwm_set_gpio_level(PWM_PIN_SERVO, SERVO_INITIAL_LEVEL);

    sleep_ms(1000);
}

void set_duty_fr(float duty)
{
    pwm_set_gpio_level(PWM_PIN_FR, motor_duty_to_level(duty));
}

void set_duty_fl(float duty)
{
    pwm_set_gpio_level(PWM_PIN_FL, motor_duty_to_level(duty));
}

void set_duty_rr(float duty)
{
    pwm_set_gpio_level(PWM_PIN_RR, motor_duty_to_level(duty));
}

void set_duty_rl(float duty)
{
    pwm_set_gpio_level(PWM_PIN_RL, motor_duty_to_level(duty));
}
