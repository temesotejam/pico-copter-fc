/*
 * pwm.hpp - DShot300 on four motor pins; conventional PWM on servo GPIO6.
 * One-way (standard polarity, CRC), no ESC telemetry on motor wires.
 */
#ifndef PWM_HPP
#define PWM_HPP
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico_copter.hpp"
#include "control.hpp"

// Existing hardware allocation and 400 Hz PWM-slice timebase.
#define DUTYMIN 1375
#define DUTYMAX 2375
constexpr uint PWM_PIN_RL    = 2;
constexpr uint PWM_PIN_RR    = 3;
constexpr uint PWM_PIN_FL    = 14;
constexpr uint PWM_PIN_FR    = 15;
constexpr uint PWM_PIN_SERVO = 6;
constexpr uint PWM_MAINLOOP_PIN = PWM_PIN_FL;

extern uint8_t ESC_calib;
void pwm_init();
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

// Read-only USB diagnostics, not motor feedback.
// Last successfully queued packet (not measured at the ESC).
struct DShotMotorTxStatus {
    uint32_t tx;
    uint32_t tx_busy;
    float requested_duty;
    uint16_t last_command;  // 0=STOP, 48..2047=running
    uint16_t last_frame;    // non-inverted standard DShot frame, telemetry bit=0
};
struct DShotTxSnapshot {
    DShotMotorTxStatus motors[4]; // FR, FL, RR, RL
};
DShotTxSnapshot dshot_tx_snapshot();

#endif
