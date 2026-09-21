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
#   Servo dial : 0 deg = 12:00, 180 deg = just before 12:00 (12-hour semicircle).
#                Snaps back to 0 at noon and midnight.
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
TIME_URL = "http://worldtimeapi.org/api/timezone/" + TIMEZONE
NOAA_STATION = "8443970"        # Boston, MA  (find others at tidesandcurrents.noaa.gov)

PIN_SERVO = 4
PIN_RED = 25
PIN_BLUE_TOP = 26
PIN_BLUE_BOTTOM = 33
PIN_BUTTON = 32                 # 34 = onboard button

SERVO_MIN_US = 500              # pulse width at 0 deg   - tune for your servo
SERVO_MAX_US = 2500             # pulse width at 180 deg - tune for your servo
SERVO_REVERSE = False           # flip if the pointer sweeps the wrong way

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


def set_angle(angle):
    angle = max(0, min(180, angle))
    if SERVO_REVERSE:
        angle = 180 - angle
    us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * angle / 180
    servo.duty_u16(int(us * 65535 / 20000))   # 20 ms period at 50 Hz


def time_to_angle(hour, minute):
    # 12 hours across 180 degrees = 15 degrees per hour
    return ((hour % 12) + minute / 60) * 15


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


def connect_wifi(timeout_s=20):
    wlan.active(True)
    if wlan.isconnected():
        return True
    print("Connecting to WiFi...")
    wlan.connect(SSID, PASSWORD)
    start = time.time()
    blink = 0
    while not wlan.isconnected():
        blink ^= 1
        booting_leds(blink)
        time.sleep_ms(250)
        if time.time() - start > timeout_s:
            print("WiFi timeout")
            return False
    print("Connected! IP:", wlan.ifconfig()[0])
    return True


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


def sync_time():
    """Primary: worldtimeapi.org. Fallback: NTP + US Eastern DST rule."""
    if not connect_wifi():
        return False
    gc.collect()
    try:
        r = urequests.get(TIME_URL)
        try:
            data = r.json()
        finally:
            r.close()
        utc = data["unixtime"] - EPOCH_OFFSET
        offset = data["raw_offset"] + data["dst_offset"]
        set_rtc_local(utc + offset)
        print("Time synced (worldtimeapi):", time.localtime())
        return True
    except Exception as e:
        print("worldtimeapi failed:", e, "- trying NTP")
    try:
        import ntptime
        ntptime.settime()                   # sets RTC to UTC
        utc = time.time()
        set_rtc_local(utc + eastern_offset(utc))
        print("Time synced (NTP):", time.localtime())
        return True
    except Exception as e:
        print("NTP failed:", e)
        return False


# ------------------------------- TIDES -------------------------------------
def fetch_tides():
    """Returns {'H': (hour, minute), 'L': (hour, minute)} for the next high/low tide."""
    if not connect_wifi():
        raise OSError("no WiFi")
    y, m, d = time.localtime()[:3]
    url = ("https://api.tidesandcurrents.noaa.gov/api/prod/datagetter"
           "?product=predictions&application=robo_tide_clock"
           "&begin_date=%04d%02d%02d&range=48&datum=MLLW"
           "&station=%s&time_zone=lst_ldt&units=english"
           "&interval=hilo&format=json") % (y, m, d, NOAA_STATION)
    gc.collect()
    r = urequests.get(url)
    try:
        data = r.json()
    finally:
        r.close()

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
set_angle(0)
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
    