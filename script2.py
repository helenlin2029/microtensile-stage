#!/usr/bin/env python3
"""
Minimal TMC2209 single-wire UART driver.

Lets you set run/hold current, microstepping, and chopper mode in software
instead of via the VREF trimmer and MS1/MS2 pins.

WIRING (Pi 4 -> TMC2209):
    GPIO14 (TXD) --[1k]-- PDN_UART
    GPIO15 (RXD) ---------PDN_UART      (tie both to the same pin)
    GND ------------------GND

The 1k resistor is required: the Pi's TX and the driver both drive that
line, and the resistor keeps them from fighting. The Pi hears its own
transmission echoed back, which this code discards.

PI SETUP:
    sudo raspi-config -> Interface -> Serial:
        login shell over serial = NO
        serial hardware enabled = YES
    Add to /boot/firmware/config.txt:   dtoverlay=disable-bt
    Reboot. Use /dev/serial0.

IMPORTANT: in UART mode MS1/MS2 become the slave ADDRESS pins, not
microstep select. Tie both LOW for address 0 (this driver's default).

Register bit assignments are from the TMC2209 datasheet rev 1.09.
Verify against your own copy before trusting a production run.
"""

import time

import serial

# --- register addresses ---
REG_GCONF       = 0x00
REG_GSTAT       = 0x01
REG_IFCNT       = 0x02
REG_IHOLD_IRUN  = 0x10
REG_TPOWERDOWN  = 0x11
REG_TSTEP       = 0x12
REG_TPWMTHRS    = 0x13
REG_CHOPCONF    = 0x6C
REG_DRV_STATUS  = 0x6F
REG_PWMCONF     = 0x70

# MRES field (CHOPCONF bits 27:24) -> microsteps
MRES = {256: 0, 128: 1, 64: 2, 32: 3, 16: 4, 8: 5, 4: 6, 2: 7, 1: 8}


class TMC2209Error(Exception):
    pass


class TMC2209:
    def __init__(self, port="/dev/serial0", baud=115200, addr=0,
                 r_sense=0.11, verbose=True):
        self.addr = addr
        self.r_sense = r_sense
        self.verbose = verbose
        self.ser = serial.Serial(port, baud, timeout=0.2)
        time.sleep(0.05)
        self.ser.reset_input_buffer()

        # Verify we can actually talk to the chip before anything moves.
        gconf = self.read_reg(REG_GCONF)
        if gconf is None:
            raise TMC2209Error(
                f"No reply from TMC2209 at address {addr} on {port}. "
                "Check the 1k resistor, PDN_UART wiring, MS1/MS2 address "
                "pins, and that the serial console is disabled.")

        # pdn_disable=1 (bit6): required to use UART at all
        # mstep_reg_select=1 (bit7): microsteps come from CHOPCONF, not pins
        # I_scale_analog=0 (bit0): ignore VREF trimmer, use internal reference
        #   so current is fully determined by software
        # en_spreadcycle=0 (bit2): StealthChop, much quieter at low speed
        gconf |= (1 << 6) | (1 << 7)
        gconf &= ~(1 << 0)
        gconf &= ~(1 << 2)
        self.write_reg(REG_GCONF, gconf)

        self.write_reg(REG_TPOWERDOWN, 20)   # ~0.3 s before standstill drop

    # ------------------------------------------------------------------
    # low-level datagram handling
    # ------------------------------------------------------------------
    @staticmethod
    def _crc8(data):
        crc = 0
        for byte in data:
            b = byte
            for _ in range(8):
                if (crc >> 7) ^ (b & 0x01):
                    crc = ((crc << 1) ^ 0x07) & 0xFF
                else:
                    crc = (crc << 1) & 0xFF
                b >>= 1
        return crc

    def write_reg(self, reg, value):
        value &= 0xFFFFFFFF
        dg = [0x05, self.addr, reg | 0x80,
              (value >> 24) & 0xFF, (value >> 16) & 0xFF,
              (value >> 8) & 0xFF, value & 0xFF]
        dg.append(self._crc8(dg))
        self.ser.reset_input_buffer()
        self.ser.write(bytes(dg))
        self.ser.flush()
        time.sleep(0.002)
        self.ser.reset_input_buffer()   # discard our own echo

    def read_reg(self, reg):
        dg = [0x05, self.addr, reg]
        dg.append(self._crc8(dg))
        self.ser.reset_input_buffer()
        self.ser.write(bytes(dg))
        self.ser.flush()

        # 4 bytes of echo, then an 8-byte reply
        raw = self.ser.read(12)
        if len(raw) < 12:
            return None
        reply = raw[4:]
        if reply[0] != 0x05 or self._crc8(list(reply[:7])) != reply[7]:
            return None
        return (reply[3] << 24) | (reply[4] << 16) | (reply[5] << 8) | reply[6]

    # ------------------------------------------------------------------
    # current
    # ------------------------------------------------------------------
    def _cs_for(self, ma, vsense):
        """Current scale 0-31 for a target RMS current in mA.
        I_RMS = (CS+1)/32 * V_fs/(R_sense+0.02) / sqrt(2)"""
        v_fs = 0.180 if vsense else 0.325
        i_max = v_fs / (self.r_sense + 0.02) / (2 ** 0.5) * 1000.0
        cs = round(ma / i_max * 32.0) - 1
        return max(0, min(31, cs)), i_max

    def set_current(self, run_ma, hold_ma=None, hold_delay=8):
        """Set run and hold current in mA RMS. Returns the actual achieved
        run current, which is quantised to 32 levels."""
        if hold_ma is None:
            hold_ma = run_ma * 0.5

        # Prefer the low-sensitivity range for small motors: it puts a
        # 0.5 A motor near the middle of the 32-step scale instead of the
        # bottom, so the quantisation error is much smaller.
        vsense = 1
        cs_run, i_max = self._cs_for(run_ma, vsense)
        if cs_run >= 31:
            vsense = 0
            cs_run, i_max = self._cs_for(run_ma, vsense)
        cs_hold, _ = self._cs_for(hold_ma, vsense)

        chop = self.read_reg(REG_CHOPCONF)
        if chop is None:
            raise TMC2209Error("Lost UART contact while setting current.")
        chop = (chop | (1 << 17)) if vsense else (chop & ~(1 << 17))
        self.write_reg(REG_CHOPCONF, chop)

        self.write_reg(REG_IHOLD_IRUN,
                       (cs_hold & 0x1F) |
                       ((cs_run & 0x1F) << 8) |
                       ((hold_delay & 0x0F) << 16))

        actual_run = (cs_run + 1) / 32.0 * i_max
        actual_hold = (cs_hold + 1) / 32.0 * i_max
        if self.verbose:
            print(f"  current: run {actual_run:.0f} mA (CS={cs_run}), "
                  f"hold {actual_hold:.0f} mA (CS={cs_hold}), vsense={vsense}")
        return actual_run, actual_hold

    # ------------------------------------------------------------------
    # microstepping
    # ------------------------------------------------------------------
    def set_microsteps(self, n, interpolate=True):
        if n not in MRES:
            raise ValueError(f"Invalid microstep setting: {n}")
        chop = self.read_reg(REG_CHOPCONF)
        if chop is None:
            raise TMC2209Error("Lost UART contact while setting microsteps.")
        chop &= ~(0x0F << 24)
        chop |= (MRES[n] << 24)
        chop = (chop | (1 << 28)) if interpolate else (chop & ~(1 << 28))
        if (chop & 0x0F) == 0:      # TOFF=0 means the driver is off
            chop |= 0x03
        self.write_reg(REG_CHOPCONF, chop)
        if self.verbose:
            print(f"  microstepping: 1/{n}"
                  f"{' (interpolated to 1/256)' if interpolate else ''}")

    def get_microsteps(self):
        chop = self.read_reg(REG_CHOPCONF)
        if chop is None:
            return None
        mres = (chop >> 24) & 0x0F
        for k, v in MRES.items():
            if v == mres:
                return k
        return None

    def set_stealthchop(self, on=True):
        g = self.read_reg(REG_GCONF)
        g = (g & ~(1 << 2)) if on else (g | (1 << 2))
        self.write_reg(REG_GCONF, g)

    # ------------------------------------------------------------------
    # diagnostics
    # ------------------------------------------------------------------
    def status(self):
        s = self.read_reg(REG_DRV_STATUS)
        if s is None:
            return {"error": "no reply"}
        return {
            "overtemp_warning": bool(s & (1 << 0)),
            "overtemp_shutdown": bool(s & (1 << 1)),
            "short_to_gnd_A": bool(s & (1 << 2)),
            "short_to_gnd_B": bool(s & (1 << 3)),
            "open_load_A": bool(s & (1 << 4)),
            "open_load_B": bool(s & (1 << 5)),
            "standstill": bool(s & (1 << 31)),
            "cs_actual": (s >> 16) & 0x1F,
        }

    def close(self):
        try:
            self.ser.close()
        except Exception:
            pass