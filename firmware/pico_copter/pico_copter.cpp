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
  uint32_t last_erpm_report_ms = to_ms_since_boot(get_absolute_time());
  while(1)
  {
    const bool fs = rc_failsafe_latched;
    if (fs != last_reported_failsafe) {
      last_reported_failsafe = fs;
      printf("#RC %s (motors %s)\n", fs ? "FAILSAFE" : "GOOD",
             fs ? "INHIBITED" : "DISARMED UNTIL RE-ARM");
    }
    //printf("Arm_flag:%d LockMode:%d\n",Arm_flag, LockMode);
    // Diagnostic only: print from core-0 main, never in the 400 Hz IRQ.
    // eRPM is not used by PID, EKF, motor mixing, or CH5 ring logging.
    const uint32_t now_ms = to_ms_since_boot(get_absolute_time());
    if (static_cast<uint32_t>(now_ms - last_erpm_report_ms) >= 1000) {
      last_erpm_report_ms = now_ms;
      if (Arm_flag == 2 && Logoutputflag == 0 && stdio_usb_connected()) {
        const DShotTelemetrySnapshot snapshot = dshot_telemetry_snapshot();
        const char* name[4] = {"FR", "FL", "RR", "RL"};
        for (int i = 0; i < 4; ++i) {
          const DShotMotorTelemetry& m = snapshot.motors[i];
          const uint32_t success_percent = m.tx ?
            static_cast<uint32_t>((static_cast<uint64_t>(m.rx_ok) * 100u) / m.tx) : 0;
          printf("#DShot300 %s eRPM=%ld valid=%u rx=%lu/%lu (%lu%%) "
                 "timeout=%lu gcr=%lu crc=%lu other=%lu busy=%lu\n",
                 name[i], m.fresh ? static_cast<long>(m.erpm) : -1L,
                 m.fresh ? 1u : 0u,
                 static_cast<unsigned long>(m.rx_ok),
                 static_cast<unsigned long>(m.tx),
                 static_cast<unsigned long>(success_percent),
                 static_cast<unsigned long>(m.rx_timeout),
                 static_cast<unsigned long>(m.rx_bad_gcr),
                 static_cast<unsigned long>(m.rx_bad_crc),
                 static_cast<unsigned long>(m.rx_other),
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
