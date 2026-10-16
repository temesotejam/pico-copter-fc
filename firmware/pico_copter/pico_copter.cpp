// comment
#include "pico_copter.hpp"
#include "pico/stdio_usb.h"

//グローバル変数
volatile uint8_t Arm_flag=0;
semaphore_t sem;

int main(void)
{
  int start_wait=5;
  
  gpio_init(LED_PIN);
  gpio_set_dir(LED_PIN, GPIO_OUT);
  
  //Initialize stdio for Pico
  stdio_init_all();
  
  //Initialize LSM9DS1
  imu_mag_init();
  
  //Initialize Radio
  radio_init();
  
  //Initialize Variavle
  variable_init();
  
  //Initilize Control
  control_init();
  
  //Initialize synchronized RAM flight recorder before enabling 400 Hz IRQ.
  flight_log_init();

  //Initialize PWM
  //Start 400Hz Interval
  ESC_calib=0;
  pwm_init();

  while(start_wait)
  {
    start_wait--;
    printf("#Please wait %d[s]\r",start_wait); 
    sleep_ms(1000);
  }
  printf("\n");
 
  //マルチコア関連の設定
  sem_init(&sem, 0, 1);
  multicore_launch_core1(angle_control);  

  Arm_flag=1;
  
  bool last_reported_failsafe = true;
  uint32_t last_diagnostic_ms = to_ms_since_boot(get_absolute_time());
  while(1)
  {
    const bool fs = rc_failsafe_latched;
    if (fs != last_reported_failsafe) {
      last_reported_failsafe = fs;
      printf("#RC %s (motors %s)\n", fs ? "FAILSAFE" : "GOOD",
             fs ? "INHIBITED" : "DISARMED UNTIL RE-ARM");
    }
    // Diagnostic snapshots are taken on the ordinary main thread only.
    // Never print or wait for the USB port inside the 400 Hz motor-control IRQ.
    // Keep USB output away from the CH5 flight-log export.
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    const uint32_t period_ms = Arm_flag == 2 ? 1000u : 500u;
    if (static_cast<uint32_t>(now_ms - last_diagnostic_ms) >= period_ms) {
      last_diagnostic_ms = now_ms;
      if (Arm_flag != 4 && Logoutputflag == 0 && stdio_usb_connected()) {
        const RadioDebugSnapshot radio = radio_debug_snapshot();
        const ControlDebugSnapshot ctrl = control_debug_snapshot();
        const DShotTelemetrySnapshot dshot = dshot_telemetry_snapshot();

        printf("#DBG STATE t_ms=%lu arm=%u lock=%u rc_latch=%u "
               "record=%u overg=%u\n",
               static_cast<unsigned long>(now_ms),
               static_cast<unsigned int>(ctrl.arm),
               static_cast<unsigned int>(ctrl.lock_mode),
               ctrl.failsafe_latched ? 1u : 0u,
               static_cast<unsigned int>(ctrl.logflag),
               static_cast<unsigned int>(ctrl.over_g));
        // CH1..CH6 values are the last valid SBUS frame, not a promise that
        // these channels are still live. Check healthy= and age_ms= too.
        printf("#DBG SBUS ch1=%u ch2=%u ch3=%u ch4=%u ch5=%u ch6=%u "
               "healthy=%u age_ms=%ld flags=0x%02x good=%lu fs_events=%lu\n",
               static_cast<unsigned int>(radio.channels[0]),
               static_cast<unsigned int>(radio.channels[1]),
               static_cast<unsigned int>(radio.channels[2]),
               static_cast<unsigned int>(radio.channels[3]),
               static_cast<unsigned int>(radio.channels[4]),
               static_cast<unsigned int>(radio.channels[5]),
               radio.healthy ? 1u : 0u,
               radio.received ? static_cast<long>(radio.age_ms) : -1L,
               static_cast<unsigned int>(radio.flags),
               static_cast<unsigned long>(radio.good_frames),
               static_cast<unsigned long>(radio.failsafe_events));
        // Mixer values may be stale while disarmed; the DShot cmd below is
        // the LAST ACTUALLY QUEUED frame and is the signal-level reference.
        printf("#DBG MIX active=%u T=%.3f P=%.3f Q=%.3f R=%.3f "
               "FR=%.3f FL=%.3f RR=%.3f RL=%.3f\n",
               (ctrl.arm == 2 && ctrl.lock_mode == 2
                   && !ctrl.failsafe_latched) ? 1u : 0u,
               static_cast<double>(ctrl.t_ref),
               static_cast<double>(ctrl.p_com),
               static_cast<double>(ctrl.q_com),
               static_cast<double>(ctrl.r_com),
               static_cast<double>(ctrl.fr_duty),
               static_cast<double>(ctrl.fl_duty),
               static_cast<double>(ctrl.rr_duty),
               static_cast<double>(ctrl.rl_duty));

        const char* names[4] = {"FR", "FL", "RR", "RL"};
        for (int i = 0; i < 4; ++i) {
          const DShotMotorTelemetry& m = dshot.motors[i];
          const uint32_t success_percent = m.tx
              ? static_cast<uint32_t>((static_cast<uint64_t>(m.rx_ok) * 100u) / m.tx)
              : 0u;
          printf("#DShot300 %s mode=bidir duty=%.3f cmd=%u pkt=0x%04x "
                 "eRPM=%ld valid=%u rx=%lu/%lu (%lu%%) "
                 "timeout=%lu gcr=%lu crc=%lu other=%lu stray=%lu busy=%lu\n",
                 names[i], static_cast<double>(m.requested_duty),
                 static_cast<unsigned int>(m.last_command),
                 static_cast<unsigned int>(m.last_frame),
                 m.fresh ? static_cast<long>(m.erpm) : -1L,
                 m.fresh ? 1u : 0u,
                 static_cast<unsigned long>(m.rx_ok),
                 static_cast<unsigned long>(m.tx),
                 static_cast<unsigned long>(success_percent),
                 static_cast<unsigned long>(m.rx_timeout),
                 static_cast<unsigned long>(m.rx_bad_gcr),
                 static_cast<unsigned long>(m.rx_bad_crc),
                 static_cast<unsigned long>(m.rx_other),
                 static_cast<unsigned long>(m.rx_discarded),
                 static_cast<unsigned long>(m.tx_busy));
        }
      }
    }
    tight_loop_contents();
    while (Logoutputflag==1){
      log_output();
    }
  }

  return 0;
}
