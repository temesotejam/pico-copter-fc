/*
 * pwm.hpp
 * Header file for PWM function
*/
#ifndef PWM_HPP
#define PWM_HPP
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/pwm.h"
#include "hardware/irq.h"
#include "pico_copter.hpp"
#include "control.hpp"

//#define DUTYMIN 1250
//#define DUTYMAX 2500
#define DUTYMIN 1375
#define DUTYMAX 2375

// Actual motor signals use PIO DShot300; the servo remains PWM.
// PWM_MAINLOOP_PIN identifies the internal PWM slice 7 timebase only.
// Keep these GPIO numbers identical to the current I-26 control board.
constexpr uint PWM_PIN_RL    = 2;
constexpr uint PWM_PIN_RR    = 3;
constexpr uint PWM_PIN_FL    = 14;
constexpr uint PWM_PIN_FR    = 15;
constexpr uint PWM_PIN_SERVO = 6;

// The main 400 Hz control loop is driven by the PWM wrap IRQ of this output.
constexpr uint PWM_MAINLOOP_PIN = PWM_PIN_FL;

//グローバル変数
extern uint8_t ESC_calib;

//関数の宣言
void pwm_init();
void set_duty_fr(float duty);
void set_duty_fl(float duty);
void set_duty_rr(float duty);
void set_duty_rl(float duty);

#endif
