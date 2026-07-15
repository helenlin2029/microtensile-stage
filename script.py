#!/usr/bin/env python3
"""
Direct Python port of tensile_stage_strain_controlled_3_5_26.ino
This code is used for the tensile stage with the load cell and smaller
stepper motor with integrated lead screw.

Ported for Raspberry Pi (RPi.GPIO). Command set, math, and control flow
are kept identical to the original Arduino sketch. No functional changes.
"""

import time
import RPi.GPIO as GPIO


enaP = 2
stpP = 3
dirP = 4
ms1 = 5
ms2 = 6

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

initialLength = 4000.0  # um - unstretched gauge length of sample
returnLength = 4142.0   # um - target return stretched length of sample
finalLength = 4240.0    # um - target stretched length of sample
fatigueCycles = 500     # number of full pull-push cycles


def setup():
    GPIO.setmode(GPIO.BCM)
    GPIO.setwarnings(False)

    GPIO.setup(enaP, GPIO.OUT)
    GPIO.setup(stpP, GPIO.OUT)
    GPIO.setup(dirP, GPIO.OUT)
    GPIO.setup(ms1, GPIO.OUT)
    GPIO.setup(ms2, GPIO.OUT)

    GPIO.output(dirP, GPIO.LOW)

    # Microstepping: (H,H) = 1/16
    GPIO.output(ms1, GPIO.HIGH)
    GPIO.output(ms2, GPIO.HIGH)

    print("Welcome to the Tensile Stage VB.5, Strain-Controlled Fatigue Version")
    print("Type help to view available commands")
    print("The default motor direction is pull")
    print("The displacement has been zeroed")
    print(f"The current step size is (um): {stepsize}")
    print(f"The current strain rate is (um/s): {strainrate}")


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


def help_cmd():
    print("pull: changes motor direction to pull")
    print("push: changes motor direction to push")
    print("step: moves 1 step in the current motor direction")
    print("1, 5, 10, 50, 100, 250, 500, 1000, or 3000: moves that many microns in the current motor direction")
    print("zero: sets the current global displacement to 0")
    print("fatigue: runs strain-controlled fatigue cycling.")
    print("The current position is used as the baseline.")
    print("The motor cycles (finalLength - initialLength) um above it.")


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
            movestep()
        elif command == "1":
            moveDistance(1, dir)
        elif command == "5":
            moveDistance(5, dir)
        elif command == "10":
            moveDistance(10, dir)
        elif command == "50":
            moveDistance(50, dir)
        elif command == "100":
            moveDistance(100, dir)
        elif command == "250":
            moveDistance(250, dir)
        elif command == "500":
            moveDistance(500, dir)
        elif command == "1000":
            moveDistance(1000, dir)
        elif command == "3000":
            moveDistance(3000, dir)
        elif command == "help":
            help_cmd()
        elif command == "fatigue":
            fatigue()
        else:
            print("I'm sorry, I don't understand that command. Please type help to view available commands")


if __name__ == "__main__":
    try:
        setup()
        loop()
    except KeyboardInterrupt:
        pass
    finally:
        GPIO.cleanup()