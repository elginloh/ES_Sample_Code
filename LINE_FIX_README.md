# Line following and power-cycle fix

## What changed
- Temporary loss of the line continues the low-speed alternating pivot search instead of exhausting two sweeps, stopping forever, and latching a critical `LINE_RECOVERY` fault.
- Line loss no longer reverses (`LF_BACKUP_MM` 0). The old 100 mm back-up compared the signed odometer, which counts down while reversing, so the car reversed until it happened to cross the line again.
- Each sweep pivots to 45° (`LF_SEARCH_DEG`; was 90°, which swung too far) from the heading where the line was lost: first towards the side it was last seen, then 45° the other way, repeating. The angle is measured with the IMU heading (`s4_get_imu_status()`); without a valid heading the sweeps are timed (`LF_SEARCH_T1_MS` 2300 ≈ 45°), and a measured sweep also ends at 1.5× its timed length. The LSM303DLHC heading is magnetic: if the motors bend it, set `LF_SEARCH_USE_IMU` 0 in `line_follow.h`. The console prints `line: sweep ...` and `line: found at N deg` to check the angles.
- When the left or centre IR sensor sees black again, the controller emits `LINE_FOUND`, clears the line-loss/recovery status and resumes normal steering. The right IR sensor is reserved for barcodes and junctions; it is not a line-following sensor.
- Unsuccessful initial IR calibration is retried every 200 ms rather than immediately causing a critical calibration fault. Startup still requires the centre sensor over black and the left sensor over white, with stable levels before the 8-second INIT timeout.
- All other critical safety conditions (e.g. collision, IMU, motor watchdog, ultrasonic safety) continue to stop motors. The car will not drive when those conditions persist.

## Build a production UF2 (NOT TEST=1)
In `build_make`, with the ARM cross compiler and Pico dependencies installed, run:

    make clean
    make -j8 TEST=0

The expected production image is `build_make/mtk3pico_smp1_usb_cdc_wifi.uf2` on the default build configuration. This environment did not have `arm-none-eabi-gcc`, so the production UF2 could not be built here.

## Flash once, then reboot normally
Hold BOOTSEL while plugging in the Pico W, copy the UF2 to the RPI-RP2 drive, and allow the board to reboot. The firmware is stored in onboard flash and should survive unplugging/replugging the USB power supply. **Do not hold BOOTSEL on normal power-up.** If it repeatedly boots to RPI-RP2 or loses its firmware, check that you actually copied the finished production UF2, power and cabling, and whether the flash process completed. A physical on/off switch should cut/reapply operating power; if it is connected to a GPIO input instead, the firmware must explicitly read that pin.

## Test safely
Lift the wheels off the ground for the first boot. Position the CENTRE IR sensor over the black track with LEFT over white; wait for `line: IR calibrated` and `motion: mode INIT -> NORMAL`. Confirm line pickup; briefly move the line away, then place it back. Watch USB serial diagnostics for any `motion: mode ... -> SAFE_STOP (faults 0x...)`: that means a different critical fault is active and needs diagnosis; do not disable safety interlocks to conceal it.

## Host tests
S3-U25, S3-U27 and INT-14d now assert the non-latching, no-reverse search (they used to expect a back-up and a bounded search ending in SAFE_STOP). S3-U42..U46 cover the IMU-measured sweeps (`LF_SEARCH_DEG`), the timed fallback, the safety limit and the turn-in after a find. `make -C app_program/subsystem2/host_test`: 0 failed checks.
