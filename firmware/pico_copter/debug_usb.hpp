#ifndef PICO_COPTER_DEBUG_USB_HPP
#define PICO_COPTER_DEBUG_USB_HPP

// All ordinary USB CDC diagnostics are implemented in debug_usb.cpp.
// Called ONLY on core 0's main thread: never call from the 400 Hz IRQ or
// from angle_control() on core 1.
//
// Flight-log sample export is NOT debug output and remains in control.cpp,
// so its 100 Hz x 38 field data stream and timing are unchanged.
namespace debug_usb {

// These switches affect ONLY USB human-readable diagnostics, not CH5 data.
constexpr bool kEnablePeriodicDiagnostics = true;
constexpr bool kEnableRcStateMessages = true;

void startup_countdown(int seconds_remaining);
void startup_complete();
void sensor_not_found();
void poll();

} // namespace debug_usb
#endif
