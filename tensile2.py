#!/usr/bin/env python3
"""
tensile_stage.py - Raspberry Pi 4 + TMC2209 + NEMA 11

Full port of tensile_stage_strain_controlled_3_5_26.ino, built on the
motion engine validated in step_test.py.

TWO SPEEDS, DELIBERATELY
------------------------
  JOG   - 1 kHz, the setting confirmed working with the bench generator.
          Used for manual repositioning: fast, ~625 um/s. Driven by the
          hardware PWM peripheral.

  TEST  - STRAIN_RATE_UM_S (default 4 um/s = 6.4 Hz at 1/8 microstepping).
          Used for the fatigue test and any controlled move. Software
          timed, because exact step COUNTS matter here and PWM can only
          be stopped to within a fraction of a cycle.

Both emit the same 3.3 V, 50% duty square wave. The only difference is
frequency and who times the edges. At 6.4 Hz software timing is far more
accurate than it needs to be; at 1 kHz the peripheral is better.

Run:  python3 tensile_stage.py
Quit: 'quit', or Ctrl-C (stops motion, releases pins)
"""

import atexit
import csv
import os
import time
from datetime import datetime

from gpiozero import DigitalOutputDevice, PWMOutputDevice

# =====================================================================
# PINS (BCM numbering - BCM26 is physical pin 37, BCM16 is physical 36)
# =====================================================================
PIN_STEP = 26
PIN_DIR  = 16
PIN_EN   = None    # None = EN strapped to GND in hardware. Driving a GPIO
                   # into an existing strap shorts the pin.
PIN_MS1  = None    # None = leave MS pins as wired. Floating on a TMC2209
PIN_MS2  = None    # means 1/8 - set MICROSTEPS to match.

# =====================================================================
# MECHANICS  -- both of these are still ASSUMPTIONS until measured.
# Run 'calibrate' and check them before trusting any strain number.
# =====================================================================
FULL_STEPS_PER_REV = 200
MICROSTEPS         = 8        # floating MS pins on a TMC2209 = 1/8
LEAD_UM            = 1000.0   # linear travel per motor revolution

STEP_SIZE_UM = LEAD_UM / (FULL_STEPS_PER_REV * MICROSTEPS)

MICROSTEP_TABLE = {8: (0, 0), 32: (1, 0), 64: (0, 1), 16: (1, 1)}

DIR_PULL_LEVEL = 0     # DIR level that stretches. Flip if reversed.
PULSE_DUTY     = 0.5   # 50% square wave, as validated
DIR_SETUP_S    = 1e-3

# =====================================================================
# SPEEDS
# =====================================================================
JOG_FREQ_HZ      = 1000.0   # manual repositioning
STRAIN_RATE_UM_S = 4.0      # controlled test rate

# =====================================================================
# TEST PARAMETERS
# =====================================================================
INITIAL_LENGTH_UM = 4000.0
RETURN_LENGTH_UM  = 4142.0   # 3.55 % strain
FINAL_LENGTH_UM   = 4240.0   # 6.00 % strain
FATIGUE_CYCLES    = 500
DWELL_S           = 10.0

SOFT_LIMIT_MIN_UM = -500.0
SOFT_LIMIT_MAX_UM = 5000.0

LOG_DIR      = os.path.expanduser("~/tensile_logs")
LOG_EVERY_UM = 1.0


class TensileStage:
    def __init__(self):
        self.step_size_um = STEP_SIZE_UM

        # PWM device doubles as a plain output: value 0/1 for bit-banging,
        # value=DUTY with a frequency for the hardware square wave.
        self.step = PWMOutputDevice(PIN_STEP, frequency=JOG_FREQ_HZ,
                                    initial_value=0.0)
        self.dirp = DigitalOutputDevice(PIN_DIR,
                                        initial_value=bool(DIR_PULL_LEVEL))

        self.en = None
        if PIN_EN is not None:
            self.en = DigitalOutputDevice(PIN_EN, active_high=False,
                                          initial_value=False)

        self.ms1 = self.ms2 = None
        if PIN_MS1 is not None:
            ms1, ms2 = MICROSTEP_TABLE.get(MICROSTEPS, (0, 0))
            self.ms1 = DigitalOutputDevice(PIN_MS1, initial_value=bool(ms1))
            self.ms2 = DigitalOutputDevice(PIN_MS2, initial_value=bool(ms2))

        self.pos_steps = 0        # integer count - no float drift
        self.direction = 1
        self.rate = STRAIN_RATE_UM_S
        self.log = None
        self.log_writer = None
        self.cycle = 0
        self.abort = False

        atexit.register(self.shutdown)

    # ---------------- position ----------------
    @property
    def disp(self):
        return self.pos_steps * self.step_size_um

    @property
    def strain_pct(self):
        return self.disp / INITIAL_LENGTH_UM * 100.0

    def zero(self):
        self.pos_steps = 0

    # ---------------- driver ----------------
    def enable(self):
        if self.en:
            self.en.on()
            time.sleep(1e-3)

    def disable(self):
        if self.en:
            self.en.off()

    def set_direction(self, d):
        self.direction = 1 if d >= 0 else -1
        level = DIR_PULL_LEVEL if self.direction == 1 else 1 - DIR_PULL_LEVEL
        self.step.value = 0.0
        time.sleep(DIR_SETUP_S)
        self.dirp.value = bool(level)
        time.sleep(DIR_SETUP_S)

    # ---------------- motion engines ----------------
    def _pulse_pwm(self, n_steps, freq, direction):
        """Hardware PWM. Fast and jitter-free, but the stop lands within a
        fraction of a cycle, so the emitted count is n_steps +/- 1."""
        duration = n_steps / freq
        self.step.frequency = freq
        self.step.value = PULSE_DUTY
        t_end = time.perf_counter() + duration
        try:
            while time.perf_counter() < t_end:
                if self.abort:
                    break
                time.sleep(min(0.02, max(0, t_end - time.perf_counter())))
        finally:
            self.step.value = 0.0
        elapsed = duration - max(0.0, t_end - time.perf_counter())
        emitted = int(round(elapsed * freq))
        self.pos_steps += direction * min(emitted, n_steps)

    def _pulse_timed(self, n_steps, freq, direction):
        """Software-timed 50% square wave. Same waveform as the PWM path,
        but every edge is counted, so the step total is exact. Deadline
        scheduled, so timing errors don't accumulate."""
        period = 1.0 / freq
        high = period * PULSE_DUTY
        steps_per_log = max(1, int(LOG_EVERY_UM / self.step_size_um))
        next_t = time.perf_counter()

        for i in range(n_steps):
            if self.abort:
                break
            self.step.value = 1.0
            fall = time.perf_counter() + high
            slack = fall - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
            self.step.value = 0.0

            self.pos_steps += direction
            if i % steps_per_log == 0:
                self.write_log()

            next_t += period
            slack = next_t - time.perf_counter()
            if slack > 0:
                time.sleep(slack)
        self.step.value = 0.0

    def move_steps(self, n_steps, direction, freq=None, exact=True):
        if n_steps <= 0:
            return
        freq = freq or (self.rate / self.step_size_um)

        target_um = (self.pos_steps + direction * n_steps) * self.step_size_um
        if not (SOFT_LIMIT_MIN_UM <= target_um <= SOFT_LIMIT_MAX_UM):
            print(f"REFUSED: target {target_um:.1f} um outside soft limits "
                  f"[{SOFT_LIMIT_MIN_UM:.0f}, {SOFT_LIMIT_MAX_UM:.0f}]")
            return

        self.abort = False
        self.set_direction(direction)
        self.enable()
        try:
            if exact:
                self._pulse_timed(n_steps, freq, direction)
            else:
                self._pulse_pwm(n_steps, freq, direction)
        except KeyboardInterrupt:
            self.step.value = 0.0
            print(f"\n*** ABORTED at {self.disp:.2f} um ***")
            raise
        finally:
            self.step.value = 0.0
        self.write_log()

    def move_distance(self, um, direction, jog=False):
        n = int(round(abs(um) / self.step_size_um))
        self.move_steps(n, direction,
                        freq=JOG_FREQ_HZ if jog else None,
                        exact=not jog)

    def move_to(self, target_um, jog=False):
        """Absolute move, rounded to the nearest whole microstep."""
        delta = int(round(target_um / self.step_size_um)) - self.pos_steps
        if delta:
            self.move_steps(abs(delta), 1 if delta > 0 else -1,
                            freq=JOG_FREQ_HZ if jog else None,
                            exact=not jog)

    # ---------------- logging ----------------
    def open_log(self, tag="run"):
        os.makedirs(LOG_DIR, exist_ok=True)
        path = os.path.join(
            LOG_DIR, f"{tag}_{datetime.now():%Y%m%d_%H%M%S}.csv")
        self.log = open(path, "w", newline="")
        self.log_writer = csv.writer(self.log)
        self.log_writer.writerow(
            ["unix_time", "iso_time", "cycle", "steps", "disp_um",
             "strain_pct"])
        print(f"Logging to {path}")
        return path

    def write_log(self):
        if not self.log_writer:
            return
        now = time.time()
        self.log_writer.writerow([
            f"{now:.4f}",
            datetime.fromtimestamp(now).isoformat(timespec="milliseconds"),
            self.cycle, self.pos_steps,
            f"{self.disp:.4f}", f"{self.strain_pct:.4f}"])

    def close_log(self):
        if self.log:
            self.log.flush()
            self.log.close()
            self.log = self.log_writer = None

    # ---------------- experiment ----------------
    def fatigue(self):
        if FINAL_LENGTH_UM <= INITIAL_LENGTH_UM:
            print("ERROR: FINAL_LENGTH_UM must exceed INITIAL_LENGTH_UM.")
            return

        baseline   = self.disp
        elongation = FINAL_LENGTH_UM - INITIAL_LENGTH_UM
        peak_pos   = baseline + elongation
        return_pos = baseline + (RETURN_LENGTH_UM - INITIAL_LENGTH_UM)
        span       = peak_pos - return_pos
        est_s = (FATIGUE_CYCLES * (2 * span / self.rate + DWELL_S)
                 + elongation / self.rate)

        print(f"\n  baseline   : {baseline:.1f} um (treated as 0 % strain)")
        print(f"  peak       : {peak_pos:.1f} um  "
              f"({elongation/INITIAL_LENGTH_UM*100:.2f} % strain)")
        print(f"  return     : {return_pos:.1f} um  "
              f"({(RETURN_LENGTH_UM-INITIAL_LENGTH_UM)/INITIAL_LENGTH_UM*100:.2f} % strain)")
        print(f"  rate       : {self.rate:.2f} um/s "
              f"({self.rate/self.step_size_um:.1f} Hz)")
        print(f"  cycles     : {FATIGUE_CYCLES}")
        print(f"  est. time  : {est_s/3600:.2f} h")
        print("\n  Strain values assume a 1/%d microstep and a %.0f um lead."
              % (MICROSTEPS, LEAD_UM))
        print("  If you have not run 'calibrate', they are unverified.")
        if input("Proceed? (y/n) ").strip().lower() != "y":
            print("Cancelled.")
            return

        self.open_log("fatigue")
        t0 = time.time()
        try:
            for i in range(FATIGUE_CYCLES):
                self.cycle = i + 1
                el = time.time() - t0
                print(f"  cycle {i+1}/{FATIGUE_CYCLES}  "
                      f"({(i+1)/FATIGUE_CYCLES*100:5.1f}%)  "
                      f"elapsed {el/3600:.2f} h", flush=True)
                self.move_to(peak_pos)
                self.move_to(return_pos)
                t_end = time.time() + DWELL_S
                while time.time() < t_end:
                    self.write_log()
                    time.sleep(0.25)
        except KeyboardInterrupt:
            print(f"\n  stopped during cycle {self.cycle}")
        finally:
            self.cycle = 0
            self.close_log()
            self.step.value = 0.0
            print("--- fatigue test ended ---")

    def calibrate(self):
        """One motor revolution. Measured travel = your true lead."""
        steps = FULL_STEPS_PER_REV * MICROSTEPS
        print(f"\n  {steps} pulses = 1 revolution at 1/{MICROSTEPS}")
        print(f"  expected travel: {steps*self.step_size_um:.1f} um "
              f"(if the lead really is {LEAD_UM:.0f} um)")
        print("  Put an indicator on the carriage. Time the shaft too:")
        print(f"  at {JOG_FREQ_HZ:.0f} Hz this should take "
              f"{steps/JOG_FREQ_HZ:.2f} s.")
        if input("Proceed? (y/n) ").strip().lower() != "y":
            return
        t0 = time.perf_counter()
        self.move_steps(steps, self.direction, freq=JOG_FREQ_HZ, exact=False)
        print(f"  done in {time.perf_counter()-t0:.2f} s")
        print("  Measured travel differing from expected means LEAD_UM or")
        print("  MICROSTEPS is wrong - fix before any fatigue run.")

    def shutdown(self):
        try:
            self.step.value = 0.0
            self.disable()
            self.close_log()
            self.step.close()
            self.dirp.close()
        except Exception:
            pass


HELP = """
  pull / push     set direction (pull = stretch)
  step            one microstep, current direction
  N               jog N um at 1 kHz, current direction   e.g.  50
  move N          move N um at the TEST rate             e.g.  move 20
  goto N          absolute move to N um, at test rate
  jog N           absolute move to N um, at 1 kHz
  zero            set current position as zero
  rate N          set test strain rate, um/s
  fatigue         run the strain-controlled fatigue test
  calibrate       one revolution - verify travel per step
  status          position, strain, step size, speeds
  log / nolog     start / stop CSV logging of manual moves
  help / quit
"""


def main():
    stage = TensileStage()
    jog_um_s = JOG_FREQ_HZ * stage.step_size_um
    test_hz = STRAIN_RATE_UM_S / stage.step_size_um

    print("Tensile Stage - Raspberry Pi / TMC2209")
    print(f"  STEP GPIO{PIN_STEP} (pin 37), DIR GPIO{PIN_DIR} (pin 36)")
    print(f"  EN {'GPIO%d' % PIN_EN if PIN_EN else 'hardwired'}, "
          f"MS {'driven' if PIN_MS1 else 'hardwired/floating'}")
    print(f"  step size : {stage.step_size_um:.4f} um "
          f"(1/{MICROSTEPS}, {LEAD_UM:.0f} um/rev)")
    print(f"  jog       : {JOG_FREQ_HZ:.0f} Hz = {jog_um_s:.0f} um/s")
    print(f"  test rate : {STRAIN_RATE_UM_S:.2f} um/s = {test_hz:.1f} Hz")
    print("\n  Step size is unverified until you run 'calibrate'.")
    print("  Type 'help' for commands.\n")

    while True:
        try:
            parts = input("> ").strip().lower().split()
        except (EOFError, KeyboardInterrupt):
            break
        if not parts:
            continue
        head = parts[0]

        try:
            if head in ("quit", "exit"):
                break
            elif head == "help":
                print(HELP)
            elif head == "pull":
                stage.set_direction(1);  print("  direction: pull")
            elif head == "push":
                stage.set_direction(-1); print("  direction: push")
            elif head == "step":
                stage.move_steps(1, stage.direction)
                print(f"  {stage.disp:.3f} um")
            elif head == "move":
                stage.move_distance(float(parts[1]), stage.direction)
                print(f"  {stage.disp:.3f} um ({stage.strain_pct:.3f} %)")
            elif head == "goto":
                stage.move_to(float(parts[1]))
                print(f"  {stage.disp:.3f} um ({stage.strain_pct:.3f} %)")
            elif head == "jog":
                stage.move_to(float(parts[1]), jog=True)
                print(f"  {stage.disp:.3f} um")
            elif head == "zero":
                if input("  zero displacement? (y/n) ").strip().lower() == "y":
                    stage.zero(); print("  zeroed")
            elif head == "rate":
                stage.rate = float(parts[1])
                print(f"  test rate {stage.rate} um/s "
                      f"({stage.rate/stage.step_size_um:.1f} Hz)")
            elif head == "status":
                print(f"  position  : {stage.disp:.3f} um "
                      f"({stage.pos_steps} steps)")
                print(f"  strain    : {stage.strain_pct:.3f} %")
                print(f"  direction : "
                      f"{'pull' if stage.direction == 1 else 'push'}")
                print(f"  step size : {stage.step_size_um:.4f} um")
                print(f"  test rate : {stage.rate:.2f} um/s")
            elif head == "log":
                stage.open_log("manual")
            elif head == "nolog":
                stage.close_log(); print("  logging stopped")
            elif head == "fatigue":
                stage.fatigue()
            elif head == "calibrate":
                stage.calibrate()
            else:
                stage.move_distance(float(head), stage.direction, jog=True)
                print(f"  {stage.disp:.3f} um")
        except KeyboardInterrupt:
            print("  (aborted)")
        except (ValueError, IndexError):
            print("  ? - type 'help'")

    stage.shutdown()
    print("Pins released.")


if __name__ == "__main__":
    main()