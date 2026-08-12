#!/usr/bin/env python3
"""
TMC2209 + NEMA 11 stepper test on a Raspberry Pi 4.

Wiring (BCM pin numbers -- change below if you used different pins):
    GPIO 20 -> STEP
    GPIO 21 -> DIR
    GPIO 16 -> EN   (active LOW: LOW = motor energised)
    Pi 3V3   -> VIO / VDD on the driver
    Pi GND   -> driver GND  AND  motor-supply GND (common ground is required)
    12V PSU  -> VM / VMOT + its GND

Install deps (Raspberry Pi OS Bookworm or newer):
    sudo apt install python3-gpiozero python3-lgpio

Run:
    python3 test_stepper.py
"""

from gpiozero import DigitalOutputDevice
from time import sleep, perf_counter

# ---------------------------------------------------------------- config ---
STEP_PIN = 20
DIR_PIN = 21
EN_PIN = 16

FULL_STEPS_PER_REV = 200   # 1.8 deg/step motor. Use 400 if yours is 0.9 deg.
MICROSTEPS = 8             # TMC2209 standalone default when MS1 = MS2 = LOW
STEPS_PER_REV = FULL_STEPS_PER_REV * MICROSTEPS

# ------------------------------------------------------------------ setup ---
step = DigitalOutputDevice(STEP_PIN, initial_value=False)
direction = DigitalOutputDevice(DIR_PIN, initial_value=False)
# initial_value=True -> EN pin HIGH -> driver disabled until we explicitly enable
enable = DigitalOutputDevice(EN_PIN, initial_value=True)


def _wait(seconds):
    """sleep() can't resolve sub-millisecond delays reliably, so busy-wait
    for the short ones. Burns CPU, which is fine for a bench test."""
    if seconds > 0.002:
        sleep(seconds)
    else:
        end = perf_counter() + seconds
        while perf_counter() < end:
            pass


def move(revolutions, clockwise=True, rpm=30):
    """Spin the shaft a given number of revolutions."""
    direction.value = bool(clockwise)
    _wait(0.00001)  # DIR setup time before the first STEP edge

    pulses = int(revolutions * STEPS_PER_REV)
    half_period = 30.0 / (rpm * STEPS_PER_REV)  # (60 / rpm / steps) / 2

    for _ in range(pulses):
        step.on()
        _wait(half_period)
        step.off()
        _wait(half_period)


# ------------------------------------------------------------------- test ---
if __name__ == "__main__":
    try:
        print("Enabling driver -- the motor should stiffen and hold position.")
        enable.off()          # EN LOW = enabled
        sleep(2)

        print("One revolution clockwise...")
        move(1, clockwise=True, rpm=30)
        sleep(1)

        print("One revolution counter-clockwise...")
        move(1, clockwise=False, rpm=30)
        sleep(1)

        print("Five revolutions clockwise, faster...")
        move(5, clockwise=True, rpm=90)

        print("Done.")

    except KeyboardInterrupt:
        print("\nInterrupted.")

    finally:
        # Always release the motor so the driver and coils stop heating up
        enable.on()
        step.off()
        print("Driver disabled.")