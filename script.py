``#!/usr/bin/env python3
"""
Python port of tensile_stage_strain_controlled_3_5_26.ino, updated for:
  - TMC2208 stepper driver (MS1/MS2 wired directly to VIO -- confirmed via
    BigTreeTech's documented truth table to give 1/16 microstepping.
    EN is wired to Pi GPIO22, NOT GND -- see EN control note below.)
  - Adafruit HX711 load cell amplifier (SCK on GPIO6, DATA on GPIO5)

Command set and motor/fatigue logic are kept identical to the original
sketch. HX711 support (tare/calibrate/force) is new functionality that
did not exist in the original Arduino code.

EN control: EN was originally hardwired to GND (always enabled), which
kept continuous holding current through the coils at all times -- a
contributing factor in the failure analysis of the original Pi 4 (see
project history: transients from continuously-energized coils sharing
a ground path with the Pi killed GPIO17 and the 3.3V rail). EN is now
wired to GPIO22 and actively toggled: LOW (enabled) for the full duration
of any move AND for fatigue's 10-second dwell between reversals (holding
force preserved where the test relies on it), HIGH (disabled, no current)
when idle waiting for commands. Requires the physical EN-to-GPIO22 jumper
to be in place -- if EN is still tied to GND, this code runs harmlessly
but the driver stays always-enabled regardless.
"""

import time
import RPi.GPIO as GPIO

# -------------------------------------------------------
# Stepper driver (TMC2208) pins
# EN on GPIO22: active-low (LOW = enabled, HIGH = disabled), per
# BigTreeTech's documentation. Driver starts DISABLED at script launch.
# MS1/MS2 are wired directly to VIO (not to the Pi) -- this fixes
# microstep resolution at 1/16 per BigTreeTech's documented truth table.
# -------------------------------------------------------
enP = 22
stpP = 17
dirP = 27

# -------------------------------------------------------
# HX711 load cell amplifier pins
# -------------------------------------------------------
HX711_SCK = 6
HX711_DATA = 5
HX711_READS_TARE = 15       # samples averaged for a tare reading
HX711_READS_CALIBRATION = 15  # samples averaged for a calibration reading
HX711_READS_FORCE = 3       # samples averaged for a live force reading
G = 9.80665                 # N per kg, for grams -> Newtons conversion

# The lowest possible stepsize at 1/16 microstepping is 0.3125um.
# If changing microstepping, change this value
stepsize = 0.3125  # um
strainrate = 4  # um/s

# This delay time is calculated from the stepsize and strain rate.
# In the original Arduino code this value (in ms) could not exceed 16383
# and had to be swapped to delayMicroseconds() below 1ms. time.sleep()
# in Python has no such restriction, so delaytime is used directly
# (converted to seconds) regardless of magnitude.
# Use a multiplication factor of 1000 for delay command (milliseconds),
# or 1000000 for delayMicroseconds (microseconds) -- kept from original
# comment for reference; Python sleeps in seconds.
delaytime = stepsize / strainrate * 1000 / 2  # ms, same formula as .ino

# This initializes the current displacement and sets it to zero
disp = 0.0

# This initializes the step variable and sets it to zero
j = 0  # unused, kept for fidelity with original sketch

# This initializes the direction variable. 1 for pull, -1 for push
dir = 1

# HX711 state: set by tare() and calibrate() commands. Both start unset --
# force readings are refused until calibrate() has been run at least once.
tare_offset = 0.0
calibration_factor = None  # counts per gram

initialLength = 4000.0  # um - unstretched gauge length of sample
returnLength = 4142.0   # um - target return stretched length of sample
finalLength = 4240.0    # um - target stretched length of sample
fatigueCycles = 500     # number of full pull-push cycles


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(enP, GPIO.OUT)
    GPIO.output(enP, GPIO.HIGH)  # start DISABLED -- no coil current until a command runs

    GPIO.setup(stpP, GPIO.OUT)
    GPIO.setup(dirP, GPIO.OUT)
    GPIO.output(dirP, GPIO.LOW)
    # MS1/MS2 not set up here -- not connected to the Pi (fixed by board wiring).

    GPIO.setup(HX711_SCK, GPIO.OUT)
    GPIO.setup(HX711_DATA, GPIO.IN)
    GPIO.output(HX711_SCK, GPIO.LOW)

    print("Welcome to the Tensile Stage VB.5, Strain-Controlled Fatigue Version")
    print("Type help to view available commands")
    print("The default motor direction is pull")
    print("The displacement has been zeroed")
    print(f"The current step size is (um): {stepsize}")
    print(f"The current strain rate is (um/s): {strainrate}")
    print("Load cell is NOT calibrated yet. Run 'tare' then 'calibrate' before using 'force'.")
    print("Driver is DISABLED at idle -- automatically enabled during moves and fatigue.")


# -------------------------------------------------------
# EN control: LOW = enabled (current flowing), HIGH = disabled (no current)
# Verify with a multimeter: EN-to-GND should read ~3.3V at idle,
# ~0V during a commanded move.
# -------------------------------------------------------
def enable_driver():
    GPIO.output(enP, GPIO.LOW)


def disable_driver():
    GPIO.output(enP, GPIO.HIGH)


# -------------------------------------------------------
# HELPER: move motor a relative distance in um in the given direction
# direction: 1 = pull, -1 = push
# -------------------------------------------------------
def moveDistance(distanceUm, direction):
    global disp, dir

    if direction == 1:
        GPIO.output(dirP, GPIO.LOW)
    else:
        GPIO.output(dirP, GPIO.HIGH)
    dir = direction

    steps = int(distanceUm / stepsize)  # truncation, same as Arduino (long) cast
    sleep_s = delaytime / 1000.0  # ms -> s
    for i in range(steps):
        GPIO.output(stpP, GPIO.HIGH)
        time.sleep(sleep_s)
        GPIO.output(stpP, GPIO.LOW)
        time.sleep(sleep_s)
        disp = disp + (dir * stepsize)
        print(disp)


# -------------------------------------------------------
# HELPER: move motor to an absolute displacement target in um
# -------------------------------------------------------
def moveToPosition(targetUm):
    delta = targetUm - disp
    if delta > 0:
        moveDistance(delta, 1)   # pull
    elif delta < 0:
        moveDistance(-delta, -1)  # push
    # if delta == 0, already at target


# -------------------------------------------------------
# HX711: low-level raw read (bit-banged, no external library)
# Returns one signed 24-bit reading, channel A, gain 128 (default).
# -------------------------------------------------------
def hx711_read_raw():
    # DATA goes LOW when a new conversion is ready. This blocks until then --
    # at the default 10 Hz rate that's up to ~100ms; faster if RATE is wired high.
    while GPIO.input(HX711_DATA) == 1:
        pass

    count = 0
    for _ in range(24):
        GPIO.output(HX711_SCK, GPIO.HIGH)
        count = count << 1
        GPIO.output(HX711_SCK, GPIO.LOW)
        if GPIO.input(HX711_DATA):
            count += 1

    # 25th pulse selects channel A, gain 128 for the *next* reading
    GPIO.output(HX711_SCK, GPIO.HIGH)
    GPIO.output(HX711_SCK, GPIO.LOW)

    # convert unsigned 24-bit value to signed (two's complement)
    if count & 0x800000:
        count -= 0x1000000

    return count


def hx711_read_average(num_readings):
    total = 0
    for _ in range(num_readings):
        total += hx711_read_raw()
    return total / num_readings


# -------------------------------------------------------
# Load cell zero point. Re-run at the start of each session/sample --
# this is a snapshot, not a fixed hardware property.
# -------------------------------------------------------
def tare():
    global tare_offset
    print("Taring -- make sure nothing is on the load cell.")
    print("Press Enter when ready.")
    input()
    tare_offset = hx711_read_average(HX711_READS_TARE)
    print(f"Tare complete. Offset = {tare_offset:.1f} counts")


# -------------------------------------------------------
# Determines counts-per-gram for THIS load cell + HX711 pair, using a
# known weight. Only needs to be re-run if the hardware or gain changes --
# unlike tare, this value should stay valid across sessions.
# -------------------------------------------------------
def calibrate():
    global calibration_factor
    print("--- Calibration ---")
    print("Make sure the load cell has NO weight on it, then press Enter.")
    input()
    zero_reading = hx711_read_average(HX711_READS_CALIBRATION)

    known_mass = None
    while known_mass is None:
        try:
            val = input("Place a known weight on the load cell and enter its mass in grams: ").strip()
            known_mass = float(val)
        except ValueError:
            print("Please enter a number.")

    print("Reading...")
    loaded_reading = hx711_read_average(HX711_READS_CALIBRATION)

    delta = loaded_reading - zero_reading
    if delta == 0:
        print("ERROR: no change detected between zero and loaded readings. Calibration failed.")
        return

    calibration_factor = delta / known_mass
    print(f"Calibration complete. Factor = {calibration_factor:.4f} counts/gram")
    print("Note this number down. To skip this step next time, hardcode it by")
    print("setting calibration_factor directly near the top of the script.")


def read_force_grams():
    if calibration_factor is None:
        print("Not calibrated yet. Run 'calibrate' first.")
        return None
    raw = hx711_read_average(HX711_READS_FORCE)
    return (raw - tare_offset) / calibration_factor


def force_cmd():
    grams = read_force_grams()
    if grams is None:
        return
    newtons = (grams / 1000.0) * G
    print(f"Force: {grams:.2f} g  ({newtons:.4f} N)")


def help_cmd():
    print("pull: changes motor direction to pull")
    print("push: changes motor direction to push")
    print("step: moves 1 step in the current motor direction")
    print("1, 5, 10, 50, 100, 250, 500, 1000, or 3000: moves that many microns in the current motor direction")
    print("zero: sets the current global displacement to 0")
    print("fatigue: runs strain-controlled fatigue cycling.")
    print("The current position is used as the baseline.")
    print("The motor cycles (finalLength - initialLength) um above it.")
    print("tare: zeroes the load cell baseline (run with no weight on the cell)")
    print("calibrate: determines the load cell's counts-per-gram using a known weight")
    print("force: prints the current force reading in grams and Newtons")
    print("Note: driver is only enabled during moves/fatigue; idle = disabled (no coil current)")


def movestep():
    moveDistance(stepsize, dir)


def push():
    global dir
    GPIO.output(dirP, GPIO.HIGH)
    dir = -1
    print("The motor direction is now: push")


def pull():
    global dir
    GPIO.output(dirP, GPIO.LOW)
    dir = 1
    print("The motor direction is now: pull")


def zero():
    global disp
    print("Are you sure you want to zero the displacement? (y/n)")
    check = input().strip()
    if check == "y":
        disp = 0.0
        print("Displacement has been zeroed")
    elif check == "n":
        print("Displacement has not been changed")
    else:
        print("Something went wrong.")


def fatigue():
    global dir

    # Validate parameters
    if finalLength <= initialLength:
        print("ERROR: finalLength must be greater than initialLength. Check top-of-code parameters.")
        return

    # Capture current position as the fatigue baseline
    baseline = disp
    elongation = finalLength - initialLength  # um
    peakPos = baseline + elongation
    returnPos = baseline + (returnLength - initialLength)
    percentStrain = elongation / initialLength * 100.0  # noqa: F841 (unused, kept for fidelity)

    for i in range(fatigueCycles):
        print(f"Cycle: {i + 1} / {fatigueCycles}")

        # Pull to peak (max elongation)
        moveToPosition(peakPos)

        # Push back to chosen position
        moveToPosition(returnPos)

        # Stay in the return position for 10 seconds
        time.sleep(10)

    # Restore direction to pull and confirm completion
    GPIO.output(dirP, GPIO.LOW)
    dir = 1

    print("--- Fatigue Test Complete ---")


def loop():
    while True:
        command = input().strip()

        if command == "push":
            push()
        elif command == "pull":
            pull()
        elif command == "zero":
            zero()
        elif command == "step":
            enable_driver()
            movestep()
            disable_driver()
        elif command == "1":
            enable_driver()
            moveDistance(1, dir)
            disable_driver()
        elif command == "5":
            enable_driver()
            moveDistance(5, dir)
            disable_driver()
        elif command == "10":
            enable_driver()
            moveDistance(10, dir)
            disable_driver()
        elif command == "50":
            enable_driver()
            moveDistance(50, dir)
            disable_driver()
        elif command == "100":
            enable_driver()
            moveDistance(100, dir)
            disable_driver()
        elif command == "250":
            enable_driver()
            moveDistance(250, dir)
            disable_driver()
        elif command == "500":
            enable_driver()
            moveDistance(500, dir)
            disable_driver()
        elif command == "1000":
            enable_driver()
            moveDistance(1000, dir)
            disable_driver()
        elif command == "3000":
            enable_driver()
            moveDistance(3000, dir)
            disable_driver()
        elif command == "help":
            help_cmd()
        elif command == "fatigue":
            # Enabled for the ENTIRE fatigue run, including the 10s dwells --
            # holding force preserved where the test relies on it.
            enable_driver()
            fatigue()
            disable_driver()
        elif command == "tare":
            tare()
        elif command == "calibrate":
            calibrate()
        elif command == "force":
            force_cmd()
        else:
            print("I'm sorry, I don't understand that command. Please type help to view available commands")


if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        pass
    finally:
        try:
            disable_driver()
        except Exception:
            pass
        GPIO.cleanup()