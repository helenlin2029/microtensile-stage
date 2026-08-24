#!/usr/bin/env python3
"""
step_test.py - replicate a function generator on the STEP pin.

This is the bridge between "it worked with a signal generator" and
"it works from code". It produces exactly what your generator produced:
a 3.3 V square wave at 50% duty, at a frequency you set.

Nothing else is touched. EN and the MS pins stay however you have them
wired unless you fill in their pin numbers below.

    python3 step_test.py

WIRING CHECK BEFORE RUNNING
---------------------------
The single most common reason a stage works on a bench generator and
then fails from a microcontroller is a missing ground.

Your 12 V battery, the driver, and the Pi must share a common GND.
The generator's ground clip used to provide that link. Now the Pi has
to. Verify continuity between Pi header pin 6 (GND), the driver's GND
pad, and the battery negative terminal. If VIO is already fed from the
Pi's 3.3 V you probably have this, but measure it rather than assume.
"""

import threading
import time

from gpiozero import DigitalOutputDevice, PWMOutputDevice

# ---------------- pins (BCM numbering) ----------------
STEP_PIN = 26
DIR_PIN  = 16

# Set to None for anything still hardwired. If EN is strapped to GND,
# leave EN_PIN as None - driving a GPIO into that strap shorts the pin.
EN_PIN   = None      # None = assumed hardwired to GND (always enabled)
MS1_PIN  = None      # None = leave floating/strapped as-is
MS2_PIN  = None

# ---------------- signal ----------------
# From the Keysight screenshot: SQU, 1.000 kHz, 50% duty,
# High +3.300 V / Low +0.000 V.
FREQ_HZ  = 1000.0
DUTY     = 0.5

# How often 'go' reports displacement while moving. At 1 kHz a step lands
# every 1 ms, so printing every single one would be 1000 lines a second -
# unreadable and slow enough to disturb the timing. This prints roughly
# every PRINT_EVERY_S seconds instead. At low frequencies (a few Hz) that
# works out to every single step, which is what you want down there.
PRINT_EVERY_S = 0.1

# The sweep panel visible on that screen was an idle display, not an
# engaged sweep - the motor was driven by a plain fixed 1 kHz square
# wave, started cold. The 'ramp' command below is kept only as a
# fallback: if the Pi at a fixed 1 kHz stalls where the generator did
# not, ramping in will tell you the difference is torque/acceleration
# rather than wiring.
SWEEP_START_HZ = 100.0
SWEEP_STOP_HZ  = 1000.0
SWEEP_TIME_S   = 1.0

# Dead-man timer: 'run' auto-stops after this many seconds so a forgotten
# or unattended run cannot drive the carriage into a hard stop. At 1 kHz
# and 1/8 microstepping this is roughly 2 mm of travel - already well past
# the 240 um span of the tensile test. Set to None to disable.
MAX_RUN_S = 3.0

# ---------------- mechanics (for the readout only) ----------------
FULL_STEPS_PER_REV = 200
MICROSTEPS         = 8       # floating MS pins on a TMC2209 = 1/8
LEAD_UM            = 1000.0  # verify against your lead screw


def main():
    step = PWMOutputDevice(STEP_PIN, frequency=FREQ_HZ, initial_value=0.0)
    dirp = DigitalOutputDevice(DIR_PIN, initial_value=False)

    en = ms1 = ms2 = None
    if EN_PIN is not None:
        en = DigitalOutputDevice(EN_PIN, active_high=False, initial_value=True)
    if MS1_PIN is not None:
        ms1 = DigitalOutputDevice(MS1_PIN, initial_value=False)
    if MS2_PIN is not None:
        ms2 = DigitalOutputDevice(MS2_PIN, initial_value=False)

    freq = FREQ_HZ
    running = False
    deadman = [None]     # holds the auto-stop timer, if armed

    def report():
        ss = LEAD_UM / (FULL_STEPS_PER_REV * MICROSTEPS)
        print(f"  {freq:.0f} Hz x {ss:.4f} um/step = {freq*ss:.1f} um/s "
              f"({freq/(FULL_STEPS_PER_REV*MICROSTEPS)*60:.1f} rpm)")

    print("STEP pin square-wave test")
    print(f"  STEP = GPIO{STEP_PIN}, DIR = GPIO{DIR_PIN}")
    print(f"  EN   = {'GPIO%d' % EN_PIN if EN_PIN else 'hardwired (untouched)'}")
    print(f"  MS   = {'driven' if MS1_PIN else 'hardwired/floating (untouched)'}")
    report()
    print("""
Commands:
  run           start the square wave
  stop          stop it
  up / down     direction
  f 250         set frequency in Hz
  go 3          run for 3 seconds, then stop
  ramp          soft-start 100 Hz -> 1 kHz over 1 s (fallback only)
  quit
""")

    try:
        while True:
            try:
                cmd = input("> ").strip().lower().split()
            except (EOFError, KeyboardInterrupt):
                break
            if not cmd:
                continue

            try:
                if cmd[0] in ("quit", "exit"):
                    break
                elif cmd[0] == "run":
                    if en:
                        en.on()
                    if deadman[0]:
                        deadman[0].cancel()
                    step.value = DUTY
                    running = True
                    if MAX_RUN_S:
                        def _cutoff():
                            step.value = 0.0
                            print(f"\n  [auto-stopped after {MAX_RUN_S} s]")
                        deadman[0] = threading.Timer(MAX_RUN_S, _cutoff)
                        deadman[0].daemon = True
                        deadman[0].start()
                        print(f"  running (auto-stop in {MAX_RUN_S} s)")
                    else:
                        print("  running")
                elif cmd[0] == "stop":
                    if deadman[0]:
                        deadman[0].cancel()
                    step.value = 0.0
                    running = False
                    print("  stopped")
                elif cmd[0] == "up":
                    was = running
                    step.value = 0.0
                    time.sleep(0.01)
                    dirp.value = True
                    time.sleep(0.01)
                    if was:
                        step.value = DUTY
                    print("  direction: up")
                elif cmd[0] == "down":
                    was = running
                    step.value = 0.0
                    time.sleep(0.01)
                    dirp.value = False
                    time.sleep(0.01)
                    if was:
                        step.value = DUTY
                    print("  direction: down")
                elif cmd[0] == "ramp":
                    if en:
                        en.on()
                    print(f"  {SWEEP_START_HZ:.0f} -> {SWEEP_STOP_HZ:.0f} Hz "
                          f"over {SWEEP_TIME_S} s")
                    t0 = time.perf_counter()
                    step.frequency = SWEEP_START_HZ
                    step.value = DUTY
                    try:
                        while True:
                            frac = (time.perf_counter() - t0) / SWEEP_TIME_S
                            if frac >= 1.0:
                                break
                            step.frequency = (SWEEP_START_HZ + frac *
                                              (SWEEP_STOP_HZ - SWEEP_START_HZ))
                            time.sleep(0.005)
                        step.frequency = SWEEP_STOP_HZ
                        freq = SWEEP_STOP_HZ
                        print("  holding at top of ramp - 'stop' to end")
                        running = True
                    except KeyboardInterrupt:
                        step.value = 0.0
                        running = False
                        print("\n  aborted")
                elif cmd[0] == "f":
                    freq = float(cmd[1])
                    step.frequency = freq
                    report()
                elif cmd[0] == "go":
                    secs = float(cmd[1])
                    if en:
                        en.on()
                    if deadman[0]:
                        deadman[0].cancel()

                    ss = LEAD_UM / (FULL_STEPS_PER_REV * MICROSTEPS)
                    sign = 1 if dirp.value else -1
                    # print every this many steps - at least 1, so slow
                    # frequencies report each individual step
                    per_print = max(1, int(round(freq * PRINT_EVERY_S)))

                    print(f"  running {secs} s at {freq:.0f} Hz "
                          f"({ss:.4f} um/step)")
                    step.frequency = freq
                    t0 = time.perf_counter()
                    step.value = DUTY
                    next_mark = per_print
                    try:
                        while True:
                            el = time.perf_counter() - t0
                            if el >= secs:
                                break
                            done = int(el * freq)
                            while done >= next_mark:
                                print(f"    {sign*next_mark*ss:10.3f} um"
                                      f"   ({next_mark} steps)")
                                next_mark += per_print
                            time.sleep(0.002)
                    except KeyboardInterrupt:
                        print("\n  aborted")
                    finally:
                        step.value = 0.0
                        running = False

                    el = min(time.perf_counter() - t0, secs)
                    total = int(round(el * freq))
                    print(f"  TOTAL {sign*total*ss:.3f} um "
                          f"({total} steps in {el:.3f} s)")
                else:
                    print("  ?")
            except (IndexError, ValueError):
                print("  bad argument")
    finally:
        if deadman[0]:
            deadman[0].cancel()
        step.value = 0.0
        step.close()
        dirp.close()
        if en:
            en.off()
            en.close()
        for p in (ms1, ms2):
            if p:
                p.close()
        print("Stopped, pins released.")


if __name__ == "__main__":
    main()