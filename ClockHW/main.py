# main.py  -  Tide Clock for Cytron ROBO ESP32 (MicroPython)
#
# Save this file on the board as  main.py  so it runs automatically at power-up.
#
# ---------------------------------------------------------------------------
# WIRING (ROBO ESP32)
#   Servo        -> servo header, D4 row   (S = signal, + = red, - = brown/black)
#   Red LED      -> D25 -> 330 ohm -> LED long leg (+) ... short leg (-) -> GND   (AM/PM)
#   Blue LED top -> D26 -> 330 ohm -> LED + ... LED - -> GND                       (blinks = HIGH tide)
#   Blue LED bot -> D33 -> 330 ohm -> LED + ... LED - -> GND                       (blinks = LOW tide)
#   Button       -> one leg to D32, other leg to GND  (no resistor, internal pull-up used)
#
#   Grove 3 gives you GND / 3V3 / D26 / D25, Grove 5 gives GND / 3V3 / D33 / D32.
#   D25, D26, D33 also have onboard status LEDs, so you'll see them mirror your LEDs.
#   Want to skip wiring the button? Set PIN_BUTTON = 34 to use the onboard D34 button.
#
# BEHAVIOUR
#   Button cycles:  CLOCK -> NEXT HIGH TIDE -> NEXT LOW TIDE -> CLOCK ...
#   CLOCK mode : both blue LEDs solid
#   HIGH tide  : top blue blinks, bottom blue solid
#   LOW tide   : bottom blue blinks, top blue solid
#   Red LED    : ON = PM, OFF = AM (for whatever time the servo is showing)
#   Servo dial : matches the printed face - 12 on the LEFT, 6 at the TOP, 12 on the RIGHT.
#                Pointer sweeps left -> over the top -> right across 12 hours, then glides
#                back to the left 12 at noon and midnight (red LED tells you AM vs PM).
#
# FIRST-TIME SETUP: set CALIBRATE = True, upload, and follow the Shell prompts to
# line the surfboard pointer up with the printed dial. Then set it back to False.
#   Booting    : both blue LEDs blink together while connecting / syncing
#   Error      : red LED flashes fast, then returns to clock mode
# ---------------------------------------------------------------------------

import network
import time
import gc
import machine
from machine import Pin, PWM
import urequests

# ------------------------------- CONFIG ------------------------------------
SSID = "tufts_eecs"
PASSWORD = "foundedin1883"

TIMEZONE = "America/New_York"
TIME_URL = "https://timeapi.io/api/time/current/zone?timeZone=" + TIMEZONE   # free, no key
DATE_HEADER_URL = "https://www.google.com/generate_204"   # backup: any server's "Date" header
NOAA_STATION = "8443970"        # Boston, MA  (find others at tidesandcurrents.noaa.gov)

PIN_SERVO = 4
PIN_RED = 25
PIN_BLUE_TOP = 26
PIN_BLUE_BOTTOM = 33
PIN_BUTTON = 32                 # 34 = onboard button

# Dial calibration (see CALIBRATE below).
#   DIAL_LEFT_US  = pulse that points the pointer at the LEFT 12  (start of the dial)
#   DIAL_RIGHT_US = pulse that points the pointer at the RIGHT 12 (end of the dial)
# Servo shaft faces you through the plate, so a standard SG90/MG90 turns
# counter-clockwise as the pulse gets longer: long pulse = left, short pulse = right.
# If your pointer runs backwards, just swap these two numbers.
DIAL_LEFT_US = 2500
DIAL_RIGHT_US = 500
SERVO_SPEED_DEG_S = 120         # how fast the pointer glides between positions
CALIBRATE = False               # True = run the pointer-alignment routine instead of the clock

BLINK_MS = 500
DEBOUNCE_MS = 250
RESYNC_MS = 60 * 60 * 1000      # re-sync clock every hour (also catches DST changes)
# ---------------------------------------------------------------------------

MODE_CLOCK, MODE_HIGH, MODE_LOW = 0, 1, 2
MODE_NAMES = ("CLOCK", "HIGH TIDE", "LOW TIDE")

# MicroPython on ESP32 may use a 2000 epoch instead of 1970 - detect it
EPOCH_OFFSET = 0 if time.gmtime(0)[0] == 1970 else 946684800

# ------------------------------- HARDWARE ----------------------------------
red = Pin(PIN_RED, Pin.OUT, value=0)
blue_top = Pin(PIN_BLUE_TOP, Pin.OUT, value=0)
blue_bot = Pin(PIN_BLUE_BOTTOM, Pin.OUT, value=0)

if PIN_BUTTON >= 34:            # GPIO34-39 have no internal pull-up (onboard button has its own)
    btn = Pin(PIN_BUTTON, Pin.IN)
else:
    btn = Pin(PIN_BUTTON, Pin.IN, Pin.PULL_UP)

servo = PWM(Pin(PIN_SERVO), freq=50)


def write_us(us):
    servo.duty_u16(int(us * 65535 / 20000))   # 20 ms period at 50 Hz


def dial_to_us(pos):
    # pos = 0 deg (left 12) ... 90 deg (6, top) ... 180 deg (right 12), as printed on the face
    pos = max(0, min(180, pos))
    return DIAL_LEFT_US + (DIAL_RIGHT_US - DIAL_LEFT_US) * pos / 180


current_pos = None


def set_angle(pos, smooth=True):
    # Glide to a dial position instead of snapping, so the pointer (and the
    # glued-on surfboard) doesn't whip across the face at noon/midnight.
    global current_pos
    pos = max(0, min(180, pos))
    if current_pos is None or not smooth:
        write_us(dial_to_us(pos))
        current_pos = pos
        return
    step = 1 if pos > current_pos else -1
    delay = int(1000 / SERVO_SPEED_DEG_S)
    p = current_pos
    while abs(pos - p) > 1:
        p += step
        write_us(dial_to_us(p))
        time.sleep_ms(delay)
    write_us(dial_to_us(pos))
    current_pos = pos


def time_to_angle(hour, minute):
    # 12 hours across the 180 deg face = 15 deg per hour, 12 o'clock at the left end
    return ((hour % 12) + minute / 60) * 15


def calibrate():
    """Walk the pointer around the printed dial so you can align it."""
    print("\n=== DIAL CALIBRATION ===")
    print("1. Take the pointer off the horn.")
    print("2. Watch where the shaft goes, then press the pointer on pointing at the LEFT 12.")
    points = [(0, "LEFT 12"), (45, "3"), (90, "6 (top)"), (135, "9"), (180, "RIGHT 12")]
    while True:
        for pos, name in points:
            set_angle(pos)
            print("Pointer should be on %-9s (%4d us)" % (name, dial_to_us(pos)))
            time.sleep(3)
        print("If the ends are short or overshoot, adjust DIAL_LEFT_US / DIAL_RIGHT_US")
        print("by ~50 us at a time. If it runs backwards, swap them. Ctrl-C to stop.\n")


def booting_leds(on):
    blue_top.value(on)
    blue_bot.value(on)


def flash_error(times=8):
    for _ in range(times):
        red.value(1)
        time.sleep_ms(100)
        red.value(0)
        time.sleep_ms(100)


# ------------------------------- BUTTON ------------------------------------
btn_flag = False
last_press = 0


def on_press(pin):
    # Keep the interrupt tiny - just set a flag, the main loop does the work
    global btn_flag, last_press
    now = time.ticks_ms()
    if time.ticks_diff(now, last_press) > DEBOUNCE_MS and pin.value() == 0:
        last_press = now
        btn_flag = True


btn.irq(trigger=Pin.IRQ_FALLING, handler=on_press)

# ------------------------------- WIFI --------------------------------------
wlan = network.WLAN(network.STA_IF)


def reset_wifi():
    # Full radio reset - clears the "Wifi Internal State Error" condition
    try:
        wlan.disconnect()
    except Exception:
        pass
    wlan.active(False)
    time.sleep_ms(500)
    wlan.active(True)
    try:
        wlan.config(pm=wlan.PM_NONE)        # disable power-save (causes random drops)
    except Exception:
        pass


def connect_wifi(timeout_s=20):
    if not wlan.active():
        reset_wifi()
    if wlan.isconnected():
        return True
    print("Connecting to WiFi...")
    try:
        wlan.connect(SSID, PASSWORD)
    except OSError as e:
        # Happens if a previous attempt is still half-open - reset and retry once
        print("WiFi state error, resetting radio:", e)
        reset_wifi()
        wlan.connect(SSID, PASSWORD)
    start = time.time()
    blink = 0
    while not wlan.isconnected():
        blink ^= 1
        booting_leds(blink)
        time.sleep_ms(250)
        if time.time() - start > timeout_s:
            print("WiFi timeout")
            reset_wifi()                    # leave the radio in a clean state
            return False
    print("Connected! IP:", wlan.ifconfig()[0])
    return True


def get_json(url, tries=3):
    """GET a URL and return parsed JSON, retrying on flaky connections."""
    last_err = None
    for attempt in range(tries):
        if not connect_wifi():
            last_err = OSError("no WiFi")
            continue
        gc.collect()
        r = None
        status = None
        try:
            r = urequests.get(url, headers={"Connection": "close"})
            status = r.status_code
            if status != 200:
                raise OSError("HTTP %d" % status)
            return r.json()
        except Exception as e:
            last_err = e
            print("GET failed (try %d/%d): %s" % (attempt + 1, tries, e))
            if status is not None and 400 <= status < 500:
                break                       # 4xx (e.g. 401): retrying won't help
            time.sleep(2)
        finally:
            if r:
                r.close()
            gc.collect()
    raise last_err


# ------------------------------- TIME --------------------------------------
def nth_sunday(year, month, n):
    t = time.mktime((year, month, 1, 0, 0, 0, 0, 0))
    wd = time.gmtime(t)[6]                  # Monday = 0 ... Sunday = 6
    return 1 + (6 - wd) % 7 + 7 * (n - 1)


def eastern_offset(utc_secs):
    # US Eastern: DST from 2nd Sunday of March 07:00 UTC to 1st Sunday of Nov 06:00 UTC
    y = time.gmtime(utc_secs)[0]
    start = time.mktime((y, 3, nth_sunday(y, 3, 2), 7, 0, 0, 0, 0))
    end = time.mktime((y, 11, nth_sunday(y, 11, 1), 6, 0, 0, 0, 0))
    return -4 * 3600 if start <= utc_secs < end else -5 * 3600


def set_rtc_local(local_secs):
    t = time.gmtime(local_secs)
    machine.RTC().datetime((t[0], t[1], t[2], t[6], t[3], t[4], t[5], 0))


MONTHS = ("Jan", "Feb", "Mar", "Apr", "May", "Jun",
          "Jul", "Aug", "Sep", "Oct", "Nov", "Dec")


def set_rtc_from_local_fields(y, mo, d, h, mi, s):
    local_secs = time.mktime((y, mo, d, h, mi, s, 0, 0))
    set_rtc_local(local_secs)


def time_from_timeapi_io():
    # Response has local-time fields: year, month, day, hour, minute, seconds
    data = get_json(TIME_URL)
    set_rtc_from_local_fields(data["year"], data["month"], data["day"],
                              data["hour"], data["minute"], data["seconds"])


def time_from_date_header():
    # Every HTTPS server sends a "Date: Mon, 21 Sep 2026 14:05:33 GMT" header.
    # Uses normal web traffic, so it works even when the network blocks NTP.
    if not connect_wifi():
        raise OSError("no WiFi")
    gc.collect()
    r = urequests.get(DATE_HEADER_URL)
    try:
        date_str = None
        for k, v in r.headers.items():
            if k.lower() == "date":
                date_str = v
    finally:
        r.close()
        gc.collect()
    if not date_str:
        raise ValueError("no Date header")
    p = date_str.split()                    # ['Mon,', '21', 'Sep', '2026', '14:05:33', 'GMT']
    hh, mm, ss = [int(x) for x in p[4].split(":")]
    utc = time.mktime((int(p[3]), MONTHS.index(p[2]) + 1, int(p[1]), hh, mm, ss, 0, 0))
    set_rtc_local(utc + eastern_offset(utc))


def time_from_ntp():
    if not connect_wifi():
        raise OSError("no WiFi")
    import ntptime
    ntptime.settime()                       # sets RTC to UTC
    utc = time.time()
    set_rtc_local(utc + eastern_offset(utc))


def sync_time():
    """Try each time source in order until one works."""
    sources = (("timeapi.io", time_from_timeapi_io),
               ("HTTP Date header", time_from_date_header),
               ("NTP", time_from_ntp))
    for name, fn in sources:
        try:
            fn()
            print("Time synced (%s):" % name, time.localtime())
            return True
        except Exception as e:
            print("%s failed: %s" % (name, e))
    return False


# ------------------------------- TIDES -------------------------------------
def fetch_tides():
    """Returns {'H': (hour, minute), 'L': (hour, minute)} for the next high/low tide."""
    y, m, d = time.localtime()[:3]
    url = ("https://api.tidesandcurrents.noaa.gov/api/prod/datagetter"
           "?product=predictions&application=robo_tide_clock"
           "&begin_date=%04d%02d%02d&range=48&datum=MLLW"
           "&station=%s&time_zone=lst_ldt&units=english"
           "&interval=hilo&format=json") % (y, m, d, NOAA_STATION)
    data = get_json(url)

    now = time.time()
    nxt = {"H": None, "L": None}
    for p in data["predictions"]:
        ts = p["t"]                         # e.g. "2026-09-21 14:32" (local time)
        hh, mm = int(ts[11:13]), int(ts[14:16])
        t = time.mktime((int(ts[0:4]), int(ts[5:7]), int(ts[8:10]), hh, mm, 0, 0, 0))
        kind = p["type"][0]                 # "H" or "L"
        if t > now and nxt[kind] is None:
            nxt[kind] = (hh, mm)
    gc.collect()
    print("Next tides:", nxt)
    if nxt["H"] is None or nxt["L"] is None:
        raise ValueError("tide data incomplete")
    return nxt


# ------------------------------- STARTUP -----------------------------------
set_angle(0, smooth=False)      # park on the left 12 while booting
if CALIBRATE:
    calibrate()
while not sync_time():
    flash_error(4)
    time.sleep(5)
booting_leds(1)

# ------------------------------- MAIN LOOP ---------------------------------
mode = MODE_CLOCK
tides = None
blink_state = 0
last_blink = time.ticks_ms()
last_sync = time.ticks_ms()
last_angle = None

while True:
    now_ms = time.ticks_ms()

    # --- button: advance mode ---
    if btn_flag:
        btn_flag = False
        mode = (mode + 1) % 3
        print("Mode:", MODE_NAMES[mode])
        if mode == MODE_HIGH:
            booting_leds(1)
            try:
                tides = fetch_tides()       # one fetch covers both high and low
            except Exception as e:
                print("Tide fetch failed:", e)
                flash_error()
                mode = MODE_CLOCK
        blink_state = 0
        last_blink = time.ticks_ms()

    # --- what time are we showing? ---
    if mode == MODE_CLOCK:
        hour, minute = time.localtime()[3:5]
    elif mode == MODE_HIGH:
        hour, minute = tides["H"]
    else:
        hour, minute = tides["L"]

    # --- red LED: PM on, AM off ---
    red.value(1 if hour >= 12 else 0)

    # --- servo: only move when the angle changes ---
    angle = time_to_angle(hour, minute)
    if angle != last_angle:
        set_angle(angle)
        last_angle = angle

    # --- blue LEDs ---
    if time.ticks_diff(now_ms, last_blink) >= BLINK_MS:
        blink_state ^= 1
        last_blink = now_ms

    if mode == MODE_CLOCK:
        blue_top.value(1)
        blue_bot.value(1)
    elif mode == MODE_HIGH:
        blue_top.value(blink_state)
        blue_bot.value(1)
    else:
        blue_top.value(1)
        blue_bot.value(blink_state)

    # --- periodic re-sync (only in clock mode so it never interrupts a tide display) ---
    if mode == MODE_CLOCK and time.ticks_diff(now_ms, last_sync) > RESYNC_MS:
        sync_time()
        last_sync = time.ticks_ms()

    time.sleep_ms(20)
