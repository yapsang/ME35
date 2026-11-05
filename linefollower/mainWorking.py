"""
Line follower - Robo ESP32 + 3x VEML6040 + 2x DC motors (MicroPython)

Colour classification (centre sensor), v4:
  The VEML6040's R, G, B channels are NOT equally sensitive and read a small
  non-zero value even on black. So raw "hue ratios" make black/white/grey look
  coloured (usually green). Fix: two-point calibration per channel.

      reflectance_c = (raw_c - black_c) / (white_c - black_c)     c = R, G, B

  Black becomes (0, 0, 0), white becomes (1, 1, 1), and the tape colours land
  in between. The tapes are fairly dark (they reflect only ~5-40% of what white
  does), so their ABSOLUTE reflectance sits close to black. Their HUE does not:
      hue = (R, G, B) / (R + G + B)      (using the calibrated values)
  A reading counts as a marker only if
    1. its brightness is in the marker range (not black, not the white floor)
    2. its hue is close to ONE trained marker hue
    3. it is clearly closer to that marker than to the other markers or to grey

Layout (top view, robot drives UP the page):

                    [CENTER]              <- colour markers + "is the line under me?"
             [LEFT]   line   [RIGHT]      <- straddle the line: the ONLY sensors used to steer

Marker actions:
  RED   -> spin 360 clockwise, then find the line again
  GREEN -> stop and play a jingle on the onboard speaker
  BLUE  -> spin 360 counter-clockwise (opposite of red), then find the line again

NeoPixels: yellow = setting up, GREEN = ready (press button 35 to go), off = driving.

Every startup:
  1. Colour training & line calibration (can be loaded from JSON)
  2. Choice to reuse or skip line calibration
  3. Drive

First time ever: set RUN_MOTOR_TEST = True to check wheel direction, then False.
Prompts accept ENTER in the Thonny shell OR a press of the onboard button (GPIO 35).

IMPORTANT: train the colours with the centre sensor at the SAME HEIGHT it has
on the robot while driving.
"""

from machine import Pin, PWM, SoftI2C
from neopixel import NeoPixel
import network
import espnow
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
RIGHT_MOTOR_PINS  = (14, 27)    # Motor 2 = LEFT wheel
LEFT_MOTOR_PINS = (12, 13)    # Motor 1 = RIGHT wheel
LEFT_REVERSED  = False        # flip if that wheel spins backwards
RIGHT_REVERSED = False
PWM_FREQ = 20000
MIN_DUTY = 0.30               # fraction of full power below which motors stall

# Onboard button. GPIO35 is input-only and has NO internal pull-up
# (the board has an external one), so it is set up without PULL_UP below.
BUTTON_PIN = 35

# --- NeoPixels and speaker ---
NEOPIXEL_PIN = 15
NUM_PIXELS   = 2              # change to however many pixels your board has
BUZZER_PIN   = 23

RUN_MOTOR_TEST = False        # True = just spin wheels to check direction

# --- ESP-NOW: every detected marker colour is sent to this board ---
# MAC of the RECEIVER (the receiver prints it as "As bytes:"). None = don't send.
PEER_MAC = b'\x20\x50\x0d\x2b\x1b\xf8'    # CHANGE THIS

# --- Marker spin ---
SPIN_SPEED          = 0.5     # wheel speed while spinning
SPIN_TIME_S         = 1.6     # time for roughly 360 degrees: TUNE THIS on your robot
FIND_LINE_TIMEOUT_S = 3       # max extra time to keep turning while looking for the line

# --- Sensor ---
SENSOR_INTEGRATION_MS = 40    # VEML6040 minimum
I2C_FREQ = 400000             # VEML6040 supports 400 kHz; drop to 100000 if you get I2C errors
LINE_IS_DARK = True           # dark line on light floor? False for light line on dark floor

# --- Colour training (centre sensor) ---
# "black" and "white" MUST be in the list (they calibrate the channels).
# White is also the floor, so no separate floor class is needed.
TRAIN_COLORS = ("black", "white", "red", "green", "blue")
MARKER_COLORS = ("red", "green", "blue")    # only these trigger on_marker()
SAMPLES_PER_COLOR = 15
SAVE_FILE = "colors_reflect.json"
OFFER_REUSE = True            # at startup: press to retrain, or wait to reuse last training
REUSE_WAIT_S = 4

# --- Marker detection ---
MARKER_MAX_DIST    = 0.15     # max HUE distance to the matched marker (hue = R,G,B / (R+G+B))
MARKER_MARGIN      = 0.85     # best distance must be <= this x second-best (1.0 disables)
MIN_BRIGHT_FRACTION = 0.5     # ignore readings dimmer than this x the dimmest marker sample (black)
MAX_BRIGHT_FACTOR   = 1.6     # ignore readings brighter than this x the brightest marker sample (white floor)
MARKER_CONFIRM      = 2       # reads needed (misses allowed in between, see below)
MARKER_MISS_ALLOWED = 1       # non-matching reads tolerated while confirming
MARKER_COOLDOWN_MS  = 1500    # ignore the same tape right after acting on it

# --- Control (tune these on the track) ---
BASE_SPEED   = 0.15            # -1..1
KP           = 0.8            # reacts to how far off the line you are now
KI           = 0.00           # corrects a steady drift that builds up over time (keep small)
KD           = 0.2            # damps wobble by reacting to how fast the error changes
I_LIMIT      = 0.30           # max size of the KI term, stops it building up too far
LOST_STEER   = 0.6            # error used to steer back when the line is lost

# =====================================================================
# NEOPIXELS & SPEAKER
# =====================================================================

pixels = NeoPixel(Pin(NEOPIXEL_PIN), NUM_PIXELS)


def set_pixels(r, g, b):
    for i in range(NUM_PIXELS):
        pixels[i] = (r, g, b)
    pixels.write()


buzzer = PWM(Pin(BUZZER_PIN), freq=1000, duty=0)

JINGLE = ((523, 120), (659, 120), (784, 120), (1047, 300))   # (Hz, ms): C E G high-C


def play_jingle():
    for freq, ms in JINGLE:
        buzzer.freq(freq)
        buzzer.duty_u16(32768)
        time.sleep_ms(ms)
        buzzer.duty_u16(0)
        time.sleep_ms(30)

# =====================================================================
# ESP-NOW (send detected colour to another board)
# =====================================================================

espnow_ok = False
espnow_link = None
if PEER_MAC is not None:
    try:
        sta = network.WLAN(network.WLAN.IF_STA)
        sta.active(True)
        sta.disconnect()
        espnow_link = espnow.ESPNow()
        espnow_link.active(True)
        espnow_link.add_peer(PEER_MAC)
        espnow_ok = True
    except Exception as ex:
        print("[!] ESP-NOW setup failed:", ex)


def send_marker(color):
    """Send {"state": "marker", "payload": "<colour>"} to PEER_MAC."""
    if not espnow_ok:
        return
    try:
        msg = json.dumps({"state": "marker", "payload": color})
        ok = espnow_link.send(PEER_MAC, msg)
        print("ESP-NOW sent", msg, "ok" if ok else "(no ack)")
    except Exception as ex:
        print("[!] ESP-NOW send failed:", ex)

# =====================================================================
# INPUT (ENTER in console or onboard button)
# =====================================================================

button = Pin(BUTTON_PIN, Pin.IN) if BUTTON_PIN is not None else None
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
    CONF = 0x02              # IT = 40 ms (bits 6:4 = 000), AF = 1 (forced mode), SD = 0 (on)
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
    bus = SoftI2C(scl=Pin(pins[0]), sda=Pin(pins[1]), freq=I2C_FREQ)
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
    """Trigger all three at once, wait one integration period, read all."""
    for s in SENSORS:
        s.trigger_measurement()
    time.sleep_ms(SENSOR_INTEGRATION_MS + 10)
    return [s.read_rgbw() for s in SENSORS]


def read_center():
    center_s.trigger_measurement()
    time.sleep_ms(SENSOR_INTEGRATION_MS + 10)
    return center_s.read_rgbw()

# =====================================================================
# STORAGE & CALIBRATION SAVING
# =====================================================================

color_samples = {}   # colour -> list of raw (r, g, b, w) readings
color_refs = {}      # colour -> average reflectance (r, g, b)
dark = [0.0, 0.0, 0.0]    # per-channel raw value on black
span = [1.0, 1.0, 1.0]    # per-channel (white - black)
marker_hues = {}          # marker colour -> average hue (r, g, b fractions summing to 1)
min_bright = 0.0          # readings dimmer than this are ignored (black)
max_bright = 99.0         # readings brighter than this are ignored (white floor)
NEUTRAL = (1 / 3, 1 / 3, 1 / 3)   # hue of anything grey (black, white, floor)

cal_lo = [65535.0] * 3
cal_hi = [0.0] * 3
NAMES = ("LEFT", "RIGHT", "CENTER")


def save_calibration_data():
    """Save both color training and side-sensor calibration bounds to JSON."""
    data = {
        "colors": color_samples,
        "cal_lo": cal_lo,
        "cal_hi": cal_hi
    }
    try:
        with open(SAVE_FILE, "w") as f:
            json.dump(data, f)
        print("Saved training and calibration to", SAVE_FILE)
    except OSError:
        print("[!] couldn't save configuration file")


def load_calibration_data():
    """Load color samples and line calibration bounds from JSON."""
    global color_samples, cal_lo, cal_hi
    try:
        with open(SAVE_FILE) as f:
            data = json.load(f)

        # Backward compatibility check if file only contained colors dictionary
        if "colors" in data:
            raw_colors = data["colors"]
            cal_lo = [float(v) for v in data.get("cal_lo", [65535.0] * 3)]
            cal_hi = [float(v) for v in data.get("cal_hi", [0.0] * 3)]
        else:
            raw_colors = data

        color_samples = {c: [tuple(s) for s in rows] for c, rows in raw_colors.items()}

        if (set(color_samples) != set(TRAIN_COLORS)
                or any(len(rows) == 0 for rows in color_samples.values())):
            return False

        build_refs()
        return True
    except (OSError, ValueError, TypeError):
        return False


def hue_of(refl):
    """(hue, brightness) of a calibrated reflectance. hue is None if there is no light."""
    s = refl[0] + refl[1] + refl[2]
    if s <= 0:
        return None, 0.0
    return (refl[0] / s, refl[1] / s, refl[2] / s), s


def reflectance(r, g, b):
    """Raw channels -> 0 (black) .. ~1 (white), each channel calibrated separately."""
    return ((r - dark[0]) / span[0],
            (g - dark[1]) / span[1],
            (b - dark[2]) / span[2])


def dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2 + (a[2] - b[2]) ** 2)


def build_refs():
    """Set the black/white calibration, then one average reflectance per class."""
    global color_refs, dark, span, marker_hues, min_bright, max_bright

    def mean_rgb(rows):
        n = len(rows)
        return [sum(s[j] for s in rows) / n for j in range(3)]

    black = mean_rgb(color_samples["black"])
    white = mean_rgb(color_samples["white"])
    dark = black
    span = [max(white[j] - black[j], 1.0) for j in range(3)]

    color_refs = {}
    for c, rows in color_samples.items():
        refl = [reflectance(s[0], s[1], s[2]) for s in rows]
        n = len(refl)
        color_refs[c] = (sum(x[0] for x in refl) / n,
                         sum(x[1] for x in refl) / n,
                         sum(x[2] for x in refl) / n)

    print("Per-channel black (R,G,B): %s   white: %s" % (
        tuple(int(v) for v in black), tuple(int(v) for v in white)))
    print("Class reflectance (R,G,B), 0 = black, 1 = white:")
    for c, ref in color_refs.items():
        print("  %-6s (%.2f, %.2f, %.2f)" % (c, ref[0], ref[1], ref[2]))

    # Hue (brightness-independent) of each marker, from its individual samples
    marker_hues = {}
    brights = []
    for c in MARKER_COLORS:
        hs = []
        for s in color_samples[c]:
            h, b = hue_of(reflectance(s[0], s[1], s[2]))
            if h is not None:
                hs.append(h)
                brights.append(b)
        n = max(len(hs), 1)
        marker_hues[c] = (sum(h[0] for h in hs) / n,
                          sum(h[1] for h in hs) / n,
                          sum(h[2] for h in hs) / n)

    min_bright = MIN_BRIGHT_FRACTION * min(brights)
    max_bright = MAX_BRIGHT_FACTOR * max(brights)

    print("Marker hue (R,G,B fractions):")
    for c, h in marker_hues.items():
        print("  %-6s (%.2f, %.2f, %.2f)   distance to grey %.2f" % (c, h[0], h[1], h[2], dist(h, NEUTRAL)))
    print("Brightness gate: %.2f <= R+G+B <= %.2f" % (min_bright, max_bright))


def train_colors():
    global color_samples
    color_samples = {}
    print("\n" + "=" * 50)
    print("COLOUR TRAINING (centre sensor)")
    print("=" * 50)
    print("Hold the sensor at its DRIVING HEIGHT.")
    print("Use the same black tape and the same floor you will drive on.")
    print("While sampling, nudge it a little so the average covers variation.")
    for color in TRAIN_COLORS:
        wait_go("Put the CENTRE sensor over [%s]" % color.upper())
        rows = []
        for i in range(SAMPLES_PER_COLOR):
            rgbw = read_center()
            rows.append(rgbw)
            if max(rgbw) >= 65000:
                print("  [!] saturated - raise the sensor a little")
        if len(set(rows)) == 1:
            print("  [!] all samples identical - sensor may not be re-measuring")
        color_samples[color] = rows
        mean = [sum(r[j] for r in rows) / len(rows) for j in range(4)]
        print("  %s: %d samples, mean RGBW: %s" % (color, len(rows), tuple(int(v) for v in mean)))
    build_refs()
    save_calibration_data()
    test_colors()


def classify_all(rgbw):
    """Calibrated reflectance, brightness, hue, and hue distances to each marker and to grey."""
    refl = reflectance(rgbw[0], rgbw[1], rgbw[2])
    h, bright = hue_of(refl)
    if h is None:
        return refl, 0.0, None, []
    ranked = sorted([(dist(h, ref), c) for c, ref in marker_hues.items()]
                    + [(dist(h, NEUTRAL), "grey")])
    return refl, bright, h, ranked


def classify_center(rgbw):
    """'red' / 'green' / 'blue', or None for anything else (black, white, floor, unknown)."""
    if not marker_hues:
        return None
    refl, bright, h, ranked = classify_all(rgbw)
    if h is None or bright < min_bright:
        return None                      # too dark: black / shadow (hue is just noise)
    if bright > max_bright:
        return None                      # too bright: the white floor
    best_d, best = ranked[0]
    if best == "grey":
        return None                      # greyer than any marker
    if best_d > MARKER_MAX_DIST:
        return None                      # not close enough to the trained tape
    if best_d > MARKER_MARGIN * ranked[1][0]:
        return None                      # too close to call between two classes
    return best


def explain_center(rgbw):
    """Diagnostic: show the calibrated reading and the distance to every class."""
    refl, bright, h, ranked = classify_all(rgbw)
    print("  reflectance (%.2f, %.2f, %.2f)   brightness %.2f (need %.2f .. %.2f)"
          % (refl[0], refl[1], refl[2], bright, min_bright, max_bright))
    if h is not None:
        print("  hue (%.2f, %.2f, %.2f)   distances: %s"
              % (h[0], h[1], h[2], [(c, round(d, 3)) for d, c in ranked]))


def test_colors():
    print("\nTEST: hold the centre sensor over red/green/blue AND over black,")
    print("white and floor - the last three should all say None.")
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
        explain_center(rgbw)

# =====================================================================
# SENSOR CALIBRATION (sweep) - all three sensors learn line vs floor
# =====================================================================

def calibrate(seconds=10):
    """Slide the robot across the line so every sensor sees both line and floor."""
    global cal_lo, cal_hi
    print("\nCALIBRATION: slowly slide the robot side-to-side across the line")
    print("so the LEFT, RIGHT and CENTER sensors each pass over both line and floor.")
    print("Sweep for %d seconds. Go slowly and cover a bit past each edge." % seconds)
    wait_go("Ready to sweep?")

    # Reset so repeated calibrations don't keep stale values
    for i in range(3):
        cal_lo[i] = 65535.0
        cal_hi[i] = 0.0

    # Countdown so you can get your hands in position
    for n in (3, 2, 1):
        print("  starting in %d..." % n)
        time.sleep(1)
    print("  SWEEP NOW")

    t_start = time.ticks_ms()
    t_end = time.ticks_add(t_start, seconds * 1000)
    next_report = 1
    while time.ticks_diff(t_end, time.ticks_ms()) > 0:
        readings = read_all()
        for i in (0, 1, 2):
            w = readings[i][3]
            cal_lo[i] = min(cal_lo[i], w)
            cal_hi[i] = max(cal_hi[i], w)

        # Progress line once per second
        elapsed = time.ticks_diff(time.ticks_ms(), t_start) // 1000
        if elapsed >= next_report:
            next_report = elapsed + 1
            print("  %2ds  L %d..%d   R %d..%d   C %d..%d" % (
                elapsed, cal_lo[0], cal_hi[0], cal_lo[1], cal_hi[1], cal_lo[2], cal_hi[2]))

    print("  Done.")
    ok = True
    for i in (0, 1, 2):
        sp = cal_hi[i] - cal_lo[i]
        print("  %-6s W range: %d .. %d  (span %d)" % (NAMES[i], cal_lo[i], cal_hi[i], sp))
        if sp < 200:
            print("  [!] %s barely changed - check height/alignment" % NAMES[i])
            ok = False
    if not ok:
        if wait_go("Calibration looks weak. Press to REDO, or wait 5 s to continue",
                   timeout_s=5):
            calibrate(seconds)
    else:
        save_calibration_data()


def line_amount(i, w):
    """0 = floor, 1 = line, using the calibration range."""
    sp = cal_hi[i] - cal_lo[i]
    if sp < 1:
        return 0.0
    x = (w - cal_lo[i]) / sp
    x = max(0.0, min(1.0, x))
    return 1.0 - x if LINE_IS_DARK else x

# =====================================================================
# MARKER ACTIONS
# =====================================================================

def spin_and_find_line(direction):
    """direction +1 = clockwise, -1 = counter-clockwise."""
    stop(); time.sleep_ms(200)
    drive(SPIN_SPEED * direction, -SPIN_SPEED * direction)
    time.sleep(SPIN_TIME_S)                      # roughly one full turn
    # keep turning the same way until the centre sensor sees the line again
    t0 = time.ticks_ms()
    while (line_amount(2, read_center()[3]) < 0.5
           and time.ticks_diff(time.ticks_ms(), t0) < FIND_LINE_TIMEOUT_S * 1000):
        pass
    stop(); time.sleep_ms(200)


def on_marker(color):
    print("MARKER:", color)
    send_marker(color)                           # tell the other board first
    if color == "red":
        spin_and_find_line(+1)
    elif color == "green":
        stop()
        play_jingle()
    elif color == "blue":
        spin_and_find_line(-1)                   # opposite direction to red

# =====================================================================
# MAIN LOOP
# =====================================================================

def run():
    last_dir = 1
    last_error = 0.0
    integral = 0.0
    last_t = time.ticks_ms()
    streak_color, streak, misses = None, 0, 0
    marker_block_until = time.ticks_ms()

    while True:
        (lr, lg, lb, lw), (rr, rg, rb, rw), center = read_all()
        now = time.ticks_ms()
        dt = max(1, time.ticks_diff(now, last_t)) / 1000
        last_t = now

        # Only LEFT and RIGHT steer.
        L = line_amount(0, lw)
        R = line_amount(1, rw)
        C = line_amount(2, center[3])      # centre: line under the middle?

        # ---- markers (centre sensor): red / green / blue / None ----
        m = classify_center(center)
        if time.ticks_diff(now, marker_block_until) > 0:
            if m is not None:
                if m == streak_color:
                    streak += 1
                else:
                    streak_color, streak = m, 1
                misses = 0
                if streak >= MARKER_CONFIRM:
                    on_marker(m)
                    integral = 0.0
                    last_error = 0.0
                    streak_color, streak, misses = None, 0, 0
                    marker_block_until = time.ticks_add(time.ticks_ms(), MARKER_COOLDOWN_MS)
                    last_t = time.ticks_ms()
                    continue
            elif streak_color is not None:
                misses += 1
                if misses > MARKER_MISS_ALLOWED:
                    streak_color, streak, misses = None, 0, 0
        else:
            streak_color, streak, misses = None, 0, 0

        # Is the line under the centre sensor? (centre brightness, or marker tape on the line)
        on_line = (C > 0.5) or (m is not None)
        lost = (not on_line) and L < 0.1 and R < 0.1

        # ---- PID steering ----
        # Line under RIGHT sensor -> error > 0 -> turn right, and vice versa.
        error = R - L
        if abs(error) > 0.1:
            last_dir = 1 if error > 0 else -1      # remember which side it went
        if lost:
            error = last_dir * LOST_STEER          # steer back toward where it was last seen

        if KI > 0:
            integral += error * dt
            integral = max(-I_LIMIT / KI, min(I_LIMIT / KI, integral))  # anti-windup
        derivative = (error - last_error) / dt
        last_error = error

        turn = KP * error + KI * integral + KD * derivative
        turn = max(-BASE_SPEED, min(BASE_SPEED, turn))   # never reverse a wheel
        drive(BASE_SPEED + turn, BASE_SPEED - turn)

# =====================================================================
# STARTUP
# =====================================================================

try:
    set_pixels(40, 30, 0)                        # yellow = setting up

    if RUN_MOTOR_TEST:
        motor_test()
    else:
        retrain_colors = True
        redo_calibration = True

        if OFFER_REUSE and load_calibration_data():
            print("\nFound saved training and sensor calibration.")
            # Give choice to retrain colors
            retrain_colors = wait_go("Press within %d s to RETRAIN colours (otherwise reuse)" % REUSE_WAIT_S,
                                     timeout_s=REUSE_WAIT_S)
            if not retrain_colors:
                print("Reusing saved colours:", list(color_samples))
                # Give separate choice to reuse or redo line sweep calibration
                redo_calibration = wait_go("Press within %d s to REDO line calibration (otherwise reuse)" % REUSE_WAIT_S,
                                           timeout_s=REUSE_WAIT_S)

        if retrain_colors:
            train_colors()

        if redo_calibration:
            calibrate()
        else:
            print("Reusing saved line calibration spans: L %d..%d, R %d..%d, C %d..%d" % (
                cal_lo[0], cal_hi[0], cal_lo[1], cal_hi[1], cal_lo[2], cal_hi[2]))

        set_pixels(0, 60, 0)                     # GREEN = ready, waiting for button 35
        wait_go("Put the robot on the line - press button to GO")
        set_pixels(0, 0, 0)                      # off while driving
        run()
except KeyboardInterrupt:
    print("Stopped.")
finally:
    stop()
    buzzer.duty_u16(0)
    set_pixels(0, 0, 0)
