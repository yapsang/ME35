"""
Line follower - Robo ESP32 + 3x VEML6040 + 2x DC motors (MicroPython)

Layout (top view, robot drives UP the page):

                   [CENTER]              <- ahead of the sides, over the line: colour markers ONLY
            [LEFT]   line   [RIGHT]      <- straddle the line: the ONLY sensors used to steer

Every startup:
  1. Colour training  - centre sensor learns black, red, green, blue, white
  2. Sweep calibration - side sensors learn line vs floor
  3. Drive

First time ever: set RUN_MOTOR_TEST = True to check wheel direction, then False.
Prompts accept ENTER in the Thonny shell OR a press of the onboard button.
"""

from machine import Pin, PWM, SoftI2C
import time
import math
import sys
import select

try:
    import json
except ImportError:
    import ujson as json

# =====================================================================
# CONFIG
# =====================================================================

# --- Pins (from partner's wiring) ---
# All VEML6040s share the same fixed I2C address (0x10), so each one is
# on its own Grove port / its own pair of pins. (scl, sda)
LEFT_I2C   = (32, 33)
CENTER_I2C = (22, 21)
RIGHT_I2C  = (16, 17)

# Each motor channel = two pins into the onboard driver (A, B)
RIGHT_MOTOR_PINS  = (14, 27)   # Motor 2 = LEFT wheel
LEFT_MOTOR_PINS = (12, 13)   # Motor 1 = RIGHT wheel
LEFT_REVERSED  = False        # flip if that wheel spins backwards
RIGHT_REVERSED = False
PWM_FREQ = 20000
MIN_DUTY = 0.30               # fraction of full power below which motors stall

# Onboard button used instead of ENTER when running without a laptop.
# Set to the GPIO of one of the board's programmable buttons, or None.
BUTTON_PIN = None

RUN_MOTOR_TEST = False        # True = just spin wheels to check direction

# --- Sensor ---
SENSOR_INTEGRATION_MS = 40    # VEML6040 minimum; loop runs ~20 Hz
LINE_IS_DARK = True           # dark line on light floor? False for light line on dark floor

# --- Colour training ---
TRAIN_COLORS = ("black", "red", "green", "blue", "white")
MARKER_COLORS = ("red", "green", "blue")   # these trigger on_marker(); black/white are references only
SAMPLES_PER_COLOR = 10
BRIGHTNESS_WEIGHT = 1.0       # how much brightness counts vs hue (needed to tell black from white)
SAVE_FILE = "colors.json"
OFFER_REUSE = True            # at startup: press to retrain, or wait to reuse last training
REUSE_WAIT_S = 4

# --- Control (tune these on the track) ---
BASE_SPEED   = 0.30           # -1..1
KP           = 0.80          # reacts to how far off the line you are now
KI           = 0.00           # corrects a steady drift that builds up over time (keep small)
KD           = 0.2           # damps wobble by reacting to how fast the error changes
I_LIMIT      = 0.30           # max size of the KI term, stops it building up too far

# --- Marker detection ---
MARKER_MAX_DIST = 0.25        # reject matches further than this from every trained colour
MARKER_CONFIRM  = 3           # consecutive reads needed
MARKER_COOLDOWN_MS = 1500     # ignore the same tape right after acting on it

# =====================================================================
# INPUT (ENTER in console or onboard button)
# =====================================================================

button = Pin(BUTTON_PIN, Pin.IN, Pin.PULL_UP) if BUTTON_PIN is not None else None
_stdin_poll = select.poll()
_stdin_poll.register(sys.stdin, select.POLLIN)


def read_console():
    """Read everything typed, including the extra \\r or \\n Thonny sends,
    so one ENTER press = exactly one input."""
    chars = []
    while _stdin_poll.poll(40):          # keep reading until 40 ms of silence
        chars.append(sys.stdin.read(1))
    return "".join(chars).strip().lower()


def flush_console():
    while _stdin_poll.poll(0):
        sys.stdin.read(1)


def wait_go(msg, timeout_s=None):
    """Wait for ENTER or button press. Returns False if timeout_s passes first."""
    flush_console()
    print(msg + ("  [ENTER or button]" if button else "  [ENTER]"))
    start = time.ticks_ms()
    while True:
        if button is not None and button.value() == 0:
            time.sleep_ms(30)
            while button.value() == 0:
                time.sleep_ms(10)
            return True
        if _stdin_poll.poll(20):
            read_console()
            return True
        if timeout_s is not None and time.ticks_diff(time.ticks_ms(), start) > timeout_s * 1000:
            return False

# =====================================================================
# MOTORS
# =====================================================================

class Motor:
    def __init__(self, pins, reversed_=False):
        self.a = PWM(Pin(pins[0]), freq=PWM_FREQ)
        self.b = PWM(Pin(pins[1]), freq=PWM_FREQ)
        self.reversed = reversed_
        self.stop()

    def set(self, speed):
        """speed in -1..1"""
        speed = max(-1.0, min(1.0, speed))
        if self.reversed:
            speed = -speed
        mag = abs(speed)
        if mag < 0.02:
            self.stop()
            return
        duty = int((MIN_DUTY + mag * (1.0 - MIN_DUTY)) * 65535)
        if speed > 0:
            self.a.duty_u16(duty); self.b.duty_u16(0)
        else:
            self.a.duty_u16(0); self.b.duty_u16(duty)

    def stop(self):
        self.a.duty_u16(0); self.b.duty_u16(0)


left_motor = Motor(LEFT_MOTOR_PINS, LEFT_REVERSED)
right_motor = Motor(RIGHT_MOTOR_PINS, RIGHT_REVERSED)


def drive(l, r):
    left_motor.set(l)
    right_motor.set(r)


def stop():
    left_motor.stop()
    right_motor.stop()


def motor_test():
    print("Motor test: LEFT forward, RIGHT forward, then both reverse.")
    for name, l, r in (("LEFT fwd", 0.5, 0), ("RIGHT fwd", 0, 0.5), ("BOTH rev", -0.5, -0.5)):
        print("  ", name)
        drive(l, r); time.sleep(1.5); stop(); time.sleep(0.5)
    print("Done. If a wheel went the wrong way, flip *_REVERSED.")

# =====================================================================
# SENSORS
# =====================================================================

class VEML6040:
    """Minimal driver (replaces the veml6040 library).
    Forced mode: each trigger_measurement() starts ONE fresh 40 ms reading."""
    ADDR = 0x10
    CONF = 0x02            # IT = 40 ms (bits 6:4 = 000), AF = 1 (forced mode), SD = 0 (on)
    TRIG = 0x04

    def __init__(self, i2c):
        self.i2c = i2c
        self._conf(self.CONF)

    def _conf(self, value):
        self.i2c.writeto_mem(self.ADDR, 0x00, bytes([value, 0x00]))

    def _word(self, reg):
        d = self.i2c.readfrom_mem(self.ADDR, reg, 2)
        return d[0] | (d[1] << 8)

    def trigger_measurement(self):
        self._conf(self.CONF | self.TRIG)

    def read_rgbw(self):
        return (self._word(0x08), self._word(0x09), self._word(0x0A), self._word(0x0B))


def make_sensor(name, pins):
    bus = SoftI2C(scl=Pin(pins[0]), sda=Pin(pins[1]), freq=100000)
    found = bus.scan()
    if 0x10 not in found:
        raise RuntimeError("%s VEML6040 not found on pins %s (scan: %s)"
                           % (name, pins, [hex(a) for a in found]))
    return VEML6040(bus)


left_s   = make_sensor("LEFT", LEFT_I2C)
right_s  = make_sensor("RIGHT", RIGHT_I2C)
center_s = make_sensor("CENTER", CENTER_I2C)
SENSORS = (left_s, right_s, center_s)


def read_all():
    """Trigger all three at once, wait one integration period, read all (~45 ms)."""
    for s in SENSORS:
        s.trigger_measurement()
    time.sleep_ms(SENSOR_INTEGRATION_MS + 10)
    return [s.read_rgbw() for s in SENSORS]


def read_center():
    center_s.trigger_measurement()
    time.sleep_ms(SENSOR_INTEGRATION_MS + 10)
    return center_s.read_rgbw()

# =====================================================================
# COLOUR TRAINING (centre sensor)
# =====================================================================
# Features = hue ratios (r/w, g/w, b/w) + brightness (w / white's w).
# Ratios alone can't separate black from white (both are "grey"),
# so brightness is included.

color_means = {}     # colour -> (r, g, b, w) raw averages
color_refs = {}      # colour -> feature tuple
white_w = 1.0


def features(r, g, b, w):
    sw = w if w > 0 else 1e-6
    return (r / sw, g / sw, b / sw, BRIGHTNESS_WEIGHT * w / white_w)


def build_refs():
    global white_w, color_refs
    white_w = max(color_means.get("white", (0, 0, 0, 1))[3],
                  max(m[3] for m in color_means.values()), 1)
    color_refs = {c: features(*m) for c, m in color_means.items()}


def train_colors():
    global color_means
    color_means = {}
    print("\n" + "=" * 50)
    print("COLOUR TRAINING (centre sensor)")
    print("=" * 50)
    for color in TRAIN_COLORS:
        wait_go("Put the CENTRE sensor over [%s]" % color.upper())
        sums = [0.0, 0.0, 0.0, 0.0]
        seen = set()
        for i in range(SAMPLES_PER_COLOR):
            rgbw = read_center()
            seen.add(rgbw)
            if max(rgbw) >= 65000:
                print("  [!] saturated - raise the sensor a little")
            for j in range(4):
                sums[j] += rgbw[j]
        if len(seen) == 1:
            print("  [!] all samples identical - sensor may not be re-measuring")
        color_means[color] = tuple(s / SAMPLES_PER_COLOR for s in sums)
        print("  %s mean RGBW: %s" % (color, tuple(int(v) for v in color_means[color])))
    build_refs()
    try:
        with open(SAVE_FILE, "w") as f:
            json.dump(color_means, f)
        print("Saved training to", SAVE_FILE)
    except OSError:
        print("[!] couldn't save training")
    test_colors()


def load_colors():
    global color_means
    try:
        with open(SAVE_FILE) as f:
            color_means = {c: tuple(v) for c, v in json.load(f).items()}
        build_refs()
        return True
    except (OSError, ValueError):
        return False


def classify_center(rgbw):
    """Nearest trained colour, or None if nothing is close enough."""
    if not color_refs:
        return None
    f = features(*rgbw)
    best, best_d = None, 1e9
    for c, ref in color_refs.items():
        d = math.sqrt(sum((a - b) ** 2 for a, b in zip(f, ref)))
        if d < best_d:
            best, best_d = c, d
    return best if best_d < MARKER_MAX_DIST else None


def test_colors():
    print("\nTEST: hold the centre sensor over anything; press to classify.")
    print("Type 'd' + ENTER (or wait 10 s) when done.")
    while True:
        print("  ...[ENTER/button = test, 'd' = done]")
        start = time.ticks_ms()
        got = None
        while time.ticks_diff(time.ticks_ms(), start) < 10000:
            if button is not None and button.value() == 0:
                time.sleep_ms(30)
                while button.value() == 0:
                    time.sleep_ms(10)
                got = ""
                break
            if _stdin_poll.poll(20):
                got = read_console()
                break
        if got is None or got == "d":
            return
        rgbw = read_center()
        print("  -> %s   raw %s" % (classify_center(rgbw), tuple(int(v) for v in rgbw)))

# =====================================================================
# SIDE-SENSOR CALIBRATION (sweep)
# =====================================================================

cal_lo = [65535.0] * 3
cal_hi = [0.0] * 3


def calibrate(seconds=4):
    """Side sensors only - the centre sensor is used just for colour markers."""
    print("\nCALIBRATION: slowly slide the robot side-to-side across the line")
    print("so the LEFT and RIGHT sensors each pass over both line and floor.")
    wait_go("Ready to sweep?")
    t_end = time.ticks_add(time.ticks_ms(), seconds * 1000)
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        readings = read_all()
        for i in (0, 1):
            w = readings[i][3]
            cal_lo[i] = min(cal_lo[i], w)
            cal_hi[i] = max(cal_hi[i], w)
    for i, n in ((0, "LEFT"), (1, "RIGHT")):
        print("  %s  W range: %d .. %d" % (n, cal_lo[i], cal_hi[i]))
        if cal_hi[i] - cal_lo[i] < 200:
            print("  [!] %s barely changed - check height/alignment" % n)


def line_amount(i, w):
    """0 = floor, 1 = line, using the calibration range."""
    span = cal_hi[i] - cal_lo[i]
    if span < 1:
        return 0.0
    x = (w - cal_lo[i]) / span
    x = max(0.0, min(1.0, x))
    return 1.0 - x if LINE_IS_DARK else x

# =====================================================================
# MARKER ACTIONS - edit for your assignment
# =====================================================================

def on_marker(color):
    print("MARKER:", color)
    if color == "red":
        stop(); time.sleep(2)
    elif color == "green":
        stop(); time.sleep(1)
    elif color == "blue":
        stop(); time.sleep(1)

# =====================================================================
# MAIN LOOP
# =====================================================================

def run():
    last_error = 0.0
    integral = 0.0
    last_t = time.ticks_ms()
    streak_color, streak = None, 0
    marker_block_until = time.ticks_ms()

    while True:
        (lr, lg, lb, lw), (rr, rg, rb, rw), center = read_all()
        now = time.ticks_ms()
        dt = max(1, time.ticks_diff(now, last_t)) / 1000
        last_t = now

        # Only LEFT and RIGHT steer. Centre is only for colour markers.
        L = line_amount(0, lw)
        R = line_amount(1, rw)

        # ---- markers (centre sensor) ----
        m = classify_center(center)
        if m in MARKER_COLORS and time.ticks_diff(now, marker_block_until) > 0:
            streak = streak + 1 if m == streak_color else 1
            streak_color = m
            if streak >= MARKER_CONFIRM:
                on_marker(m)
                integral = 0.0
                streak_color, streak = None, 0
                marker_block_until = time.ticks_add(time.ticks_ms(), MARKER_COOLDOWN_MS)
                last_t = time.ticks_ms()
                continue
        else:
            streak_color, streak = None, 0

        # ---- PID steering ----
        # Both sides on floor -> error 0 -> drive straight (line is between them).
        # Line under RIGHT sensor -> error > 0 -> turn right, and vice versa.
        error = R - L

        if KI > 0:
            integral += error * dt
            integral = max(-I_LIMIT / KI, min(I_LIMIT / KI, integral))  # anti-windup
        derivative = (error - last_error) / dt
        last_error = error

        turn = KP * error + KI * integral + KD * derivative

        drive(BASE_SPEED + turn, BASE_SPEED - turn)

# =====================================================================
# STARTUP
# =====================================================================

try:
    if RUN_MOTOR_TEST:
        motor_test()
    else:
        retrain = True
        if OFFER_REUSE and load_colors():
            print("\nFound saved colour training.")
            retrain = wait_go("Press within %d s to RETRAIN (otherwise reuse)" % REUSE_WAIT_S,
                              timeout_s=REUSE_WAIT_S)
        if retrain:
            train_colors()
        else:
            print("Reusing saved colours:", list(color_means))
        calibrate()
        wait_go("Put the robot on the line - GO?")
        run()
except KeyboardInterrupt:
    print("Stopped.")
finally:
    stop()