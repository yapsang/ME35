# =====================================================================
#  ACE'S PHONE DOWN FLIGHT SIM  -  main program (MicroPython / Thonny)
#  Cytron Robo ESP32
#
#  Boot:  slowly home the rack on the back switch and sample the room light
#         with the colour sensor (no phone on it yet)
#  Then:  title screen -> phone down -> tutorial -> press fire -> game
#  Lift the phone at any time and Ace refuses to fly.
#
#  Save to the board as main.py so it runs on power-up.
# =====================================================================
from machine import Pin, PWM, ADC, SPI, SoftSPI, I2C, Timer
import framebuf, time, math, random

# ---------------- PINS ----------------
JOY_X_PIN   = 33    # joystick X  (Grove 5 yellow)
JOY_Y_PIN   = 32    # joystick Y  (Grove 5 white) - also the FIRE click (reads 4095)
ENC_A_PIN   = 26    # encoder A   (Grove 3 yellow)
ENC_B_PIN   = 25    # encoder B   (Grove 3 white)
MOTOR_IN1   = 12    # M1A terminal
MOTOR_IN2   = 13    # M1B terminal
SERVO_PIN   = 4     # servo on the D4 servo port
HOME_SW_PIN = 5     # momentary switch at the BACK of the rack: D5 signal -> GND
SW_PRESSED  = 0     # value when pressed: 0 if wired COM+NO to GND, 1 if wired to NC
COLOR_SCL   = 22    # VEML6040 colour sensor in Grove 2 (board's I2C port: D22 = SCL)
COLOR_SDA   = 21    #                                          (D21 = SDA)
OLED_SCK    = 17    # OLED on header pins
OLED_MOSI   = 16    # "SDA" on the OLED = SPI data
OLED_CS     = 18    # moved off D21 (Grove 2 is the colour sensor now)
OLED_DC     = 19    # moved off D22
OLED_RES    = 2
SPI_MISO    = 36    # unused input-only pin (SPI needs one)
FLIP        = True  # OLED is mounted upside down
BUZZER_PIN  = 23    # onboard piezo buzzer (the board's mute switch silences it)
# Onboard (free for later): NeoPixels D15, buttons D34/D35

# ---------------- PLANE: LEFT / RIGHT (servo + parallelogram) ----------------
ARM_L_MM     = 40.0   # arm length, pin to pin
MAX_SIDE_MM  = 35.0   # max side travel (must stay below ARM_L_MM)
SERVO_CENTER = 90     # trim so arms are vertical
SERVO_DIR    = -1     # servo mounted backwards
SIDE_SPEED   = 60.0   # mm/s at full stick (higher = faster)
SERVO_SLEW   = 8.0    # max servo change per loop, degrees

# ---------------- PLANE: FORWARD / BACK (rack + encoder) ----------------
RACK_SPEED   = 0.8    # fraction of full rack travel per second at full stick
KP           = 600    # PWM per unit of position error (1.0 = full rack length)
MIN_PWM      = 70     # smallest PWM that actually moves the rack
MAX_PWM      = 200    # speed cap (255 = full)
DEADBAND     = 0.015  # close enough (1.5 % of travel) - stop, don't buzz
END_MARGIN   = 0.04   # stay this far from each end while playing
MOTOR_DIR    = 1      # flip to -1 if homing drives AWAY from the switch
JOY_Y_DIR    = -1     # flip to 1 if forward/back on the stick feels reversed
MOTOR_RAMP   = 25     # max PWM change per loop (soft start/stop)
HOME_PWM     = 90     # SLOW homing speed - raise a little if it stalls before the switch
HOME_RAMP_MS = 500    # ease into the homing speed (no jerk off the start)
HOME_TIMEOUT_MS = 15000
RACK_TRAVEL_COUNTS = 2000  # <-- SET THIS: encoder counts from the switch to the front end
COUNTS_PER_REV = 0       # optional: set it to print positions in mm
PINION_PD_MM   = 21.0

# ---------------- PHONE DOCK (VEML6040 colour sensor, white channel) ----------------
COVER_RATIO   = 0.5   # phone DOWN when light drops below 50 % of the room level
UNCOVER_RATIO = 0.65  # phone UP again above 65 % (gap stops flicker)
PHONE_DEBOUNCE_MS = 400

# ---------------- GAME ----------------
LIVES        = 3
INVULN_MS    = 1500   # safe time after getting hit (jet blinks)
GAMEOVER_MS  = 3000   # game-over message shows at least this long

# ---------------- SOUND ----------------
SOUND_ON = True
VOLUME   = 20000      # 0-32768 (piezo is loudest around 32768)

# ---------------- JOYSTICK ----------------
FIRE_THRESH = 4000
JOY_DEAD    = 150
JOY_SAMPLES = 4
JOY_ALPHA   = 0.25    # low-pass filter: lower = smoother
EXPO        = 0.4     # finer control near center


# =====================================================================
#  DRIVERS
# =====================================================================
class QuadEncoder:
    """Hardware counter if available (MicroPython 1.26+), else interrupts."""
    _TABLE = (0, 1, -1, 0, -1, 0, 0, 1, 1, 0, 0, -1, 0, -1, 1, 0)

    def __init__(self, pin_a, pin_b):
        self.hw = None
        try:
            from machine import Encoder
            self.hw = Encoder(0, Pin(pin_a, Pin.IN, Pin.PULL_UP),
                              Pin(pin_b, Pin.IN, Pin.PULL_UP), phases=4)
            print("Encoder: hardware counter")
        except Exception:
            self.a = Pin(pin_a, Pin.IN, Pin.PULL_UP)
            self.b = Pin(pin_b, Pin.IN, Pin.PULL_UP)
            self._count = 0
            self._last = (self.a.value() << 1) | self.b.value()
            self.a.irq(self._cb, Pin.IRQ_RISING | Pin.IRQ_FALLING)
            self.b.irq(self._cb, Pin.IRQ_RISING | Pin.IRQ_FALLING)
            print("Encoder: software interrupts")

    def _cb(self, pin):
        s = (self.a.value() << 1) | self.b.value()
        self._count += QuadEncoder._TABLE[(self._last << 2) | s]
        self._last = s

    def count(self):
        return self.hw.value() if self.hw else self._count

    def zero(self):
        if self.hw:
            self.hw.value(0)
        else:
            self._count = 0


class SH1106(framebuf.FrameBuffer):
    """1.3 inch 128x64 SPI OLED"""
    def __init__(self, spi, dc, res, cs, w=128, h=64):
        self.spi, self.dc, self.res, self.cs = spi, dc, res, cs
        self.w, self.h, self.pages = w, h, h // 8
        self.buf = bytearray(w * self.pages)
        super().__init__(self.buf, w, h, framebuf.MONO_VLSB)
        for p in (dc, res, cs):
            p.init(Pin.OUT, value=1)
        self.res(0); time.sleep_ms(10); self.res(1); time.sleep_ms(10)
        for c in (0xAE, 0xD5, 0x80, 0xA8, 0x3F, 0xD3, 0x00, 0x40, 0xAD, 0x8B,
                  0xA0 if FLIP else 0xA1, 0xC0 if FLIP else 0xC8,
                  0xDA, 0x12, 0x81, 0xCF, 0xD9, 0x1F, 0xDB, 0x40, 0xA4, 0xA6, 0xAF):
            self.cmd(c)
        self.fill(0)
        self.show()

    def cmd(self, c):
        self.cs(1); self.dc(0); self.cs(0)
        self.spi.write(bytearray([c]))
        self.cs(1)

    def invert(self, on):
        self.cmd(0xA7 if on else 0xA6)

    def show(self):
        for page in range(self.pages):
            self.cmd(0xB0 | page); self.cmd(0x02); self.cmd(0x10)
            self.cs(1); self.dc(1); self.cs(0)
            self.spi.write(memoryview(self.buf)[page * self.w:(page + 1) * self.w])
            self.cs(1)


# =====================================================================
#  HARDWARE SETUP
# =====================================================================
joy_x = ADC(Pin(JOY_X_PIN)); joy_x.atten(ADC.ATTN_11DB)
joy_y = ADC(Pin(JOY_Y_PIN)); joy_y.atten(ADC.ATTN_11DB)
VEML_ADDR = 0x10
i2c = I2C(0, scl=Pin(COLOR_SCL), sda=Pin(COLOR_SDA), freq=100000)
color_ok = VEML_ADDR in i2c.scan()
if color_ok:
    i2c.writeto_mem(VEML_ADDR, 0x00, b"\x00\x00")   # enable, auto mode, 40 ms integration
    print("Colour sensor: OK")
_last_light = 0


def read_light():
    """VEML6040 WHITE channel: high = open room, low = phone covering it."""
    global _last_light
    try:
        d = i2c.readfrom_mem(VEML_ADDR, 0x0B, 2)
        _last_light = d[0] | (d[1] << 8)
    except OSError:
        pass                         # keep the last good value on a glitch
    return _last_light
home_sw = Pin(HOME_SW_PIN, Pin.IN, Pin.PULL_UP)        # pressed = 0
m1 = PWM(Pin(MOTOR_IN1), freq=1000, duty_u16=0)
m2 = PWM(Pin(MOTOR_IN2), freq=1000, duty_u16=0)
servo = PWM(Pin(SERVO_PIN), freq=50)
enc = QuadEncoder(ENC_A_PIN, ENC_B_PIN)

try:
    spi = SPI(1, baudrate=8_000_000, polarity=0, phase=0,
              sck=Pin(OLED_SCK), mosi=Pin(OLED_MOSI), miso=Pin(SPI_MISO))
except Exception:
    spi = SoftSPI(baudrate=4_000_000, polarity=0, phase=0,
                  sck=Pin(OLED_SCK), mosi=Pin(OLED_MOSI), miso=Pin(SPI_MISO))
oled = SH1106(spi, dc=Pin(OLED_DC), res=Pin(OLED_RES), cs=Pin(OLED_CS))


# =====================================================================
#  SOUND - plays in the background on a 10 ms timer, so the game never waits.
#  Each sound is a list of (frequency Hz, duration ms); frequency 0 = silence.
#  (Never use exactly 1000 Hz or 50 Hz - those share timers with the motor/servo.)
# =====================================================================
SFX_BOOT     = [(1319, 60), (1568, 60), (2093, 120)]
SFX_CHIRP    = [(1760, 50), (2637, 80)]                   # phone down / ready
SFX_PHONE_UP = [(220, 180), (0, 60), (196, 300)]          # Ace sulks
SFX_START    = [(523, 90), (659, 90), (784, 90), (1047, 180)]
SFX_SHOOT    = [(2400, 20), (1800, 20), (1300, 25)]       # pew
SFX_BOOM     = [(330, 30), (220, 40), (147, 60)]          # enemy destroyed
SFX_OUCH     = [(880, 80), (659, 80), (440, 160)]         # lost a life
SFX_GAMEOVER = [(784, 220), (740, 220), (698, 220), (659, 600)]   # sad trombone
SFX_ERROR    = [(150, 400), (0, 100), (150, 400)]

buzzer = PWM(Pin(BUZZER_PIN), freq=2000, duty_u16=0)
_sfx = []          # remaining notes: [freq, ms_left]
_sfx_prio = 0


def _start_note(n):
    if n[0] > 0:
        buzzer.freq(n[0])
        buzzer.duty_u16(VOLUME)
    else:
        buzzer.duty_u16(0)


def _sfx_tick(_):
    global _sfx_prio
    if not _sfx:
        return
    _sfx[0][1] -= 10
    if _sfx[0][1] <= 0:
        _sfx.pop(0)
        if _sfx:
            _start_note(_sfx[0])
        else:
            buzzer.duty_u16(0)
            _sfx_prio = 0


def play(notes, prio=0):
    """Start a sound. A higher-priority sound isn't cut off by a lower one."""
    global _sfx_prio
    if not SOUND_ON or (_sfx and prio < _sfx_prio):
        return
    _sfx[:] = [[f, d] for f, d in notes]
    _sfx_prio = prio
    _start_note(_sfx[0])


sfx_timer = Timer(0)
sfx_timer.init(period=10, mode=Timer.PERIODIC, callback=_sfx_tick)


# =====================================================================
#  HELPERS
# =====================================================================
def clamp(v, lo, hi):
    return max(lo, min(hi, v))


def set_motor(pwm):
    """pwm: -255..255 (negative = toward the home switch)"""
    pwm = clamp(int(pwm * MOTOR_DIR), -255, 255)
    duty = abs(pwm) * 257
    if pwm > 0:
        m1.duty_u16(duty); m2.duty_u16(0)
    elif pwm < 0:
        m1.duty_u16(0); m2.duty_u16(duty)
    else:
        m1.duty_u16(0); m2.duty_u16(0)


def servo_write(angle):
    angle = clamp(angle, 0, 180)
    us = 500 + 1900 * angle / 180
    servo.duty_u16(int(us / 20000 * 65535))


def read_avg(adc, n=JOY_SAMPLES):
    total = 0
    for _ in range(n):
        total += adc.read()
    return total // n


def expo(v):
    return (1 - EXPO) * v + EXPO * v * v * v


def approach(cur, target, step):
    if target > cur + step:
        return cur + step
    if target < cur - step:
        return cur - step
    return target


def switch_pressed():
    return home_sw.value() == SW_PRESSED


class Axis:
    """Joystick axis that learns its own range as you push it around."""
    def __init__(self, center):
        self.c, self.lo, self.hi = center, center - 400, center + 400

    def norm(self, raw):
        if raw < FIRE_THRESH:
            self.lo = min(self.lo, raw)
            self.hi = max(self.hi, raw)
        d = raw - self.c
        if abs(d) < JOY_DEAD:
            return 0.0
        span = (self.hi - self.c) if d > 0 else (self.c - self.lo)
        return clamp(d / span, -1.0, 1.0)


def sprite(rows):
    data = bytearray(int(r.replace('#', '1').replace('.', '0'), 2) for r in rows)
    return framebuf.FrameBuffer(data, 8, 8, framebuf.MONO_HLSB)


JET = sprite(["...##...", "...##...", "..####..", ".######.",
              "########", "##.##.##", "...##...", "..####.."])
ENEMY = sprite(["#......#", ".#.##.#.", "..####..", ".######.",
                "##.##.##", ".######.", "..#..#..", ".#....#."])


def center_text(s, y, c=1):
    oled.text(s, (128 - len(s) * 8) // 2, y, c)


def flash(times=3, ms=120):
    for _ in range(times):
        oled.invert(True); time.sleep_ms(ms)
        oled.invert(False); time.sleep_ms(ms)


def calib_screen(line1, line2=""):
    oled.fill(0)
    oled.rect(0, 0, 128, 64, 1)
    center_text("CALIBRATING", 6)
    oled.hline(16, 16, 96, 1)
    center_text(line1, 24)
    center_text(line2, 34)
    center_text("phone OFF", 46)
    center_text("the sensor", 55)
    oled.show()


def fault(msg1, msg2):
    """Stop everything and show what went wrong."""
    set_motor(0)
    print("FAULT:", msg1, msg2)
    play(SFX_ERROR, 9)
    oled.fill(0)
    oled.rect(0, 0, 128, 64, 1)
    center_text("SETUP PROBLEM", 8)
    center_text(msg1, 28)
    center_text(msg2, 40)
    oled.show()
    while True:
        time.sleep_ms(500)


# =====================================================================
#  BOOT CALIBRATION
# =====================================================================
light_samples = []


def sample_light():
    light_samples.append(read_light())


def drive_until(pwm, stop_fn, timeout_ms):
    """Ease up to 'pwm', then drive until stop_fn() is True, the rack stalls, or timeout.
    Returns 'stop', 'stall' or 'timeout'. Samples room light while moving."""
    t0 = time.ticks_ms()
    last_c, last_t = enc.count(), t0
    while True:
        if stop_fn():
            set_motor(0)
            return "stop"
        now = time.ticks_ms()
        ramp = clamp(time.ticks_diff(now, t0) / HOME_RAMP_MS, 0.3, 1.0)
        set_motor(pwm * ramp)                            # gentle start
        sample_light()
        if time.ticks_diff(now, t0) > timeout_ms:
            set_motor(0)
            return "timeout"
        if time.ticks_diff(now, last_t) > 300:          # stall check every 0.3 s
            c = enc.count()
            print("  homing: count=%d switch_raw=%d" % (c, home_sw.value()))
            if time.ticks_diff(now, t0) > HOME_RAMP_MS + 600 and abs(c - last_c) < 2:
                set_motor(0)
                return "stall"
            last_c, last_t = c, now
        time.sleep_ms(10)


set_motor(0)
servo_write(SERVO_CENTER)
if not color_ok:
    fault("colour sensor", "check Grove 2")

oled.fill(0)
center_text("A C E", 20)
oled.blit(JET, 60, 34)
oled.show()
play(SFX_BOOT)
time.sleep_ms(600)

# Joystick center - hands off during boot
calib_screen("hands off", "the stick")
sx = sy = 0
for _ in range(50):
    sx += joy_x.read(); sy += joy_y.read()
    sample_light()
    time.sleep_ms(5)
axis_x = Axis(sx // 50)
axis_y = Axis(sy // 50)
print("Joystick center X=%d Y=%d" % (axis_x.c, axis_y.c))

print("Switch raw value at boot: %d (expect %d when NOT pressed)" % (home_sw.value(), 1 - SW_PRESSED))

# 1) If we start ON the switch, back off a little first
if switch_pressed():
    calib_screen("backing off", "switch")
    drive_until(HOME_PWM, lambda: not switch_pressed(), 2000)
    drive_until(HOME_PWM, lambda: False, 250)

# 2) Slowly drive back until the switch clicks = HOME (position 0)
calib_screen("finding home", "(slowly)")
c_start = enc.count()
result = drive_until(-HOME_PWM, switch_pressed, HOME_TIMEOUT_MS)
if result == "stall":
    fault("rack stalled", "raise HOME_PWM")
if result != "stop":
    fault("no switch hit", "flip MOTOR_DIR?")
moved = enc.count() - c_start
if abs(moved) < 20:
    fault("encoder not", "counting")
ENC_SIGN = -1 if moved > 0 else 1       # make "forward" = positive position
time.sleep_ms(150)
enc.zero()
print("Homed. Encoder sign:", ENC_SIGN)
play(SFX_CHIRP)


def pos_counts():
    return ENC_SIGN * enc.count()


# 3) Forward distance is hard-coded
travel = RACK_TRAVEL_COUNTS
print("Rack travel: %d counts (hard-coded)" % travel)
if COUNTS_PER_REV > 0:
    print("  = %.1f mm" % (travel / COUNTS_PER_REV * math.pi * PINION_PD_MM))


def frac_pos():
    return pos_counts() / travel


# 4) Park the plane in the middle
calib_screen("centering", "")
t0 = time.ticks_ms()
while time.ticks_diff(time.ticks_ms(), t0) < 10000:
    err = 0.5 - frac_pos()
    if abs(err) < DEADBAND:
        break
    sample_light()
    set_motor(HOME_PWM if err > 0 else -HOME_PWM)
    time.sleep_ms(10)
set_motor(0)

# 5) Room light reference (phone was off the sensor the whole time)
ambient = sum(light_samples) / len(light_samples)
cover_th = ambient * COVER_RATIO
uncover_th = ambient * UNCOVER_RATIO
print("Room light %d -> phone DOWN below %d, UP above %d" % (ambient, cover_th, uncover_th))
if ambient < 50:
    print("WARNING: room is very dark - the phone sensor may not tell the difference")


# =====================================================================
#  GAME
# =====================================================================
def new_game():
    global stars, enemies, bullets, score, last_fire, lives, hit_time
    stars = [[random.randint(0, 127), random.randint(10, 63), random.randint(1, 2)]
             for _ in range(18)]
    enemies = [[random.randint(0, 119), random.randint(-60, -8)] for _ in range(3)]
    bullets = []
    score = 0
    last_fire = 0
    lives = LIVES
    hit_time = -INVULN_MS


HEART = sprite(["........", ".##.##..", "#######.", "#######.",
                ".#####..", "..###...", "...#....", "........"])


def draw_gameover(blink, ready):
    oled.fill(0)
    center_text("GREAT FLYING!", 0)
    oled.hline(16, 9, 96, 1)
    center_text("Now keep the", 13)
    center_text("phone down", 22)
    center_text("and STUDY!", 31)
    center_text("score %d" % score, 43)
    if ready and blink:
        center_text("FIRE=fly again", 55)
    oled.show()


def draw_title(blink):
    oled.fill(0)
    oled.rect(0, 0, 128, 64, 1)
    center_text("ACE'S", 4)
    center_text("PHONE DOWN", 14)
    center_text("FLIGHT SIM", 24)
    oled.blit(JET, 112, 4)
    oled.blit(JET, 8, 4)
    if blink:
        center_text("place phone", 40)
        center_text("down to start!", 50)
    oled.show()


def draw_tutorial(blink):
    oled.fill(0)
    center_text("HOW TO FLY", 0)
    oled.hline(0, 9, 128, 1)
    oled.text("UP/DN  fwd/back", 2, 13, 1)
    oled.text("L/R    bank", 2, 23, 1)
    oled.text("CLICK  shoot", 2, 33, 1)
    center_text("keep phone down", 43)
    if blink:
        oled.fill_rect(0, 53, 128, 11, 1)
        center_text("FIRE TO PLAY!", 55, 0)
    oled.show()


def draw_paused(light):
    oled.fill(0)
    oled.rect(0, 0, 128, 64, 1)
    center_text("ACE WON'T FLY", 6)
    center_text(":(", 20)
    center_text("put your phone", 34)
    center_text("back down", 44)
    oled.text("%d/%d" % (light, cover_th), 4, 55, 1)
    oled.show()


def draw_game(jet_x, jet_y, fire, now):
    """Returns True when the last life is lost."""
    global bullets, score, last_fire, lives, hit_time
    safe = time.ticks_diff(now, hit_time) < INVULN_MS
    if fire and time.ticks_diff(now, last_fire) > 200:
        bullets.append([jet_x + 3, jet_y - 2])
        last_fire = now
        play(SFX_SHOOT, 0)

    oled.fill(0)
    for st in stars:
        st[1] += st[2]
        if st[1] > 63:
            st[0], st[1] = random.randint(0, 127), 10
        oled.pixel(st[0], st[1], 1)

    for b in bullets:
        b[1] -= 4
        oled.vline(b[0], b[1], 3, 1)
    bullets = [b for b in bullets if b[1] > 10]

    speed = 1 + score // 15                # enemies speed up as you score
    for e in enemies:
        e[1] += speed
        # Enemy rams the jet -> lose a life
        if not safe and abs(e[0] - jet_x) < 7 and abs(e[1] - jet_y) < 7:
            lives -= 1
            hit_time = now
            if lives > 0:
                play(SFX_OUCH, 2)
            safe = True
            e[0], e[1] = random.randint(0, 119), random.randint(-40, -8)
            flash(1, 80)
            continue
        hit = None
        for b in bullets:
            if e[0] - 1 <= b[0] <= e[0] + 8 and e[1] <= b[1] <= e[1] + 8:
                hit = b
                break
        if hit:
            bullets.remove(hit)
            score += 1
            play(SFX_BOOM, 1)
        if hit or e[1] > 63:
            e[0], e[1] = random.randint(0, 119), random.randint(-40, -8)
        if e[1] > 2:
            oled.blit(ENEMY, e[0], e[1])

    if not safe or (now // 100) % 2:       # jet blinks while invulnerable
        oled.blit(JET, jet_x, jet_y)
    oled.fill_rect(0, 0, 128, 9, 0)
    oled.text("SCORE %d" % score, 0, 0, 1)
    for i in range(lives):
        oled.blit(HEART, 120 - i * 9, 0)
    oled.hline(0, 9, 128, 1)
    oled.show()
    return lives <= 0


# =====================================================================
#  MAIN LOOP
# =====================================================================
state = "TITLE"
gameover_t = 0
flash(3)
new_game()

phone_down = False
phone_t = time.ticks_ms()
fire_prev = False
filt_x, filt_y = float(axis_x.c), float(axis_y.c)
side_mm = 0.0
servo_angle = float(SERVO_CENTER)
target = frac_pos()
pwm_out = 0.0
last_loop = last_print = time.ticks_ms()

try:
    while True:
        t = time.ticks_ms()
        dt = time.ticks_diff(t, last_loop) / 1000
        last_loop = t
        blink = (t // 450) % 2 == 0

        # ---------- inputs ----------
        raw_x = read_avg(joy_x)
        raw_y = read_avg(joy_y)
        fire = raw_y > FIRE_THRESH
        fire_edge = fire and not fire_prev
        fire_prev = fire
        filt_x += JOY_ALPHA * (raw_x - filt_x)
        if not fire:
            filt_y += JOY_ALPHA * (raw_y - filt_y)

        # ---------- phone dock (hysteresis + debounce) ----------
        light = read_light()
        want = light < (uncover_th if phone_down else cover_th)
        if want == phone_down:
            phone_t = t
        elif time.ticks_diff(t, phone_t) > PHONE_DEBOUNCE_MS:
            phone_down = want
            print("PHONE DOWN" if phone_down else "PHONE UP")

        # ---------- state machine ----------
        if state == "TITLE" and phone_down:
            state = "TUTORIAL"
            play(SFX_CHIRP, 1)
            flash(2)
        elif state == "TUTORIAL":
            if not phone_down:
                state = "TITLE"
                play(SFX_PHONE_UP, 1)
            elif fire_edge:
                state = "PLAY"
                new_game()
                play(SFX_START, 2)
                flash(2)
                fire_prev = True          # don't fire a bullet on the start click
        elif state == "PLAY" and not phone_down:
            state = "PAUSED"
            play(SFX_PHONE_UP, 2)
        elif state == "PAUSED" and phone_down:
            state = "PLAY"
            play(SFX_CHIRP, 2)
        elif state == "GAMEOVER" and time.ticks_diff(t, gameover_t) > GAMEOVER_MS and fire_edge:
            if phone_down:
                state = "PLAY"
                new_game()
                play(SFX_START, 2)
                flash(2)
                fire_prev = True
            else:
                state = "TITLE"

        # ---------- plane control (only while playing) ----------
        pwm = 0
        if state == "PLAY":
            # Stick = speed: push to move, let go to HOLD
            stick_x = expo(axis_x.norm(int(filt_x)))
            stick_y = 0.0 if fire else JOY_Y_DIR * expo(axis_y.norm(int(filt_y)))

            # Left/right: inverse kinematics  theta = asin(x / L)
            side_mm = clamp(side_mm + stick_x * SIDE_SPEED * dt, -MAX_SIDE_MM, MAX_SIDE_MM)
            theta = math.degrees(math.asin(side_mm / ARM_L_MM))
            servo_angle = approach(servo_angle, SERVO_CENTER + SERVO_DIR * theta, SERVO_SLEW)

            # Forward/back: P-control on the encoder
            if switch_pressed():
                enc.zero()                # free re-zero every time we touch home
            target = clamp(target + stick_y * RACK_SPEED * dt, END_MARGIN, 1 - END_MARGIN)
            err = target - frac_pos()
            if abs(err) > DEADBAND:
                mag = clamp(abs(KP * err), MIN_PWM, MAX_PWM)
                pwm = mag if err > 0 else -mag
            if (switch_pressed() and pwm < 0) or (frac_pos() >= 1.0 and pwm > 0):
                pwm = 0                   # hard safety at both ends

        servo_write(servo_angle)          # servo always holds its last angle
        pwm_out = approach(pwm_out, pwm, MOTOR_RAMP) if state == "PLAY" else 0.0
        set_motor(pwm_out)

        # ---------- screen ----------
        if state == "TITLE":
            draw_title(blink)
        elif state == "TUTORIAL":
            draw_tutorial(blink)
        elif state == "PAUSED":
            draw_paused(light)
        elif state == "GAMEOVER":
            draw_gameover(blink, time.ticks_diff(t, gameover_t) > GAMEOVER_MS)
        else:
            # The jet on screen mirrors the REAL plane
            jet_x = int((side_mm + MAX_SIDE_MM) / (2 * MAX_SIDE_MM) * 120)
            jet_y = int(56 - clamp(frac_pos(), 0, 1) * 44)    # forward = up the screen
            if draw_game(jet_x, jet_y, fire, t):
                state = "GAMEOVER"
                gameover_t = t
                play(SFX_GAMEOVER, 3)
                flash(4)
                print("GAME OVER - score", score)

        # ---------- debug ----------
        if time.ticks_diff(t, last_print) > 250:
            last_print = t
            print("%-8s X=%4d Y=%4d fire=%d | pos=%5d/%d tgt=%.2f pwm=%4d | servo=%3d | light=%4d sw=%d"
                  % (state, raw_x, raw_y, fire, pos_counts(), travel, target,
                     int(pwm_out), servo_angle, light, switch_pressed()))

except KeyboardInterrupt:
    pass
finally:
    set_motor(0)
    sfx_timer.deinit()
    buzzer.duty_u16(0)
    oled.fill(0)
    center_text("ACE OFFLINE", 28)
    oled.show()
    print("Stopped - motor off")