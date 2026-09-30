"""
2-DOF planar arm: two hand-turned DC motor encoders drive the X and Y
position of the pointer tip ("wrist"). Inverse kinematics converts (x, y)
into shoulder (servo 1) and elbow (servo 2) angles.

Hardware: ROBO ESP32, 2x Miuzel MG90S servos (D4 = shoulder, D5 = elbow),
          2x DC motors with DFRobot Encoder v2.0 used as input knobs.

Coordinate frame (top view of the arm plane):
    Origin = shoulder servo shaft (servo 1)
    +x     = the direction arm 1 points when servo 1 is at SHOULDER_OFFSET deg
    +y     = 90 deg counter-clockwise from +x
All lengths are in millimetres, all angles in degrees unless noted.
"""

from machine import Pin, PWM
import math
import time

# =====================================================================
# GEOMETRY  -- measure these on the CAD / real parts (pivot to pivot)
# =====================================================================
ARM1_LENGTH = 80.0   # shoulder shaft -> elbow shaft (mm)
ARM2_LENGTH = 70.0   # elbow shaft   -> pointer tip  (mm)

# =====================================================================
# PINS
# =====================================================================
SERVO1_PIN = 4            # shoulder servo (D4)
SERVO2_PIN = 5            # elbow servo    (D5)
ENC_X_PINS = (39, 32)     # encoder driving X  (A, B)  -- from encoder.py
ENC_Y_PINS = (25, 26)     # encoder driving Y  (A, B)  -- TODO: confirm Grove pins

# =====================================================================
# SERVO CALIBRATION
# =====================================================================
SERVO_MIN_US = 500        # pulse width at 0 deg   (MG90S ~500 us)
SERVO_MAX_US = 2500       # pulse width at 180 deg (MG90S ~2500 us)

# Servo reading when the joint angle is zero:
#   shoulder: arm 1 points along +x
#   elbow:    arm 2 is straight in line with arm 1
SHOULDER_OFFSET = 0.0
ELBOW_OFFSET = 90.0
# +1 if increasing the servo angle rotates the joint counter-clockwise
# (viewed from above), -1 if clockwise. Flip these if the arm moves backwards.
SHOULDER_DIR = 1
ELBOW_DIR = 1

# Starting pose (servo degrees). The start (x, y) is computed from this,
# so it is always reachable.
HOME_SHOULDER = 90.0
HOME_ELBOW = 135.0

# =====================================================================
# INPUT FEEL
# =====================================================================
MM_PER_COUNT_X = 0.25     # tip travel per encoder count. Negative = reverse knob
MM_PER_COUNT_Y = 0.25
MAX_STEP_DEG = 15.0       # reject solutions that would jump a joint more than this
LOOP_MS = 20              # 50 Hz update, matches the servo PWM frame
PRINT_MS = 250            # status print interval


# =====================================================================
# ENCODER (quadrature, counts every edge on A and B = 4x resolution)
# =====================================================================
class QuadEncoder:
    # index = (old_state << 2) | new_state, state = (A << 1) | B
    _TABLE = (0, -1, 1, 0,
              1, 0, 0, -1,
              -1, 0, 0, 1,
              0, 1, -1, 0)

    def __init__(self, pin_a, pin_b):
        # Note: GPIO 34-39 have no internal pull-ups. The DFRobot encoder
        # drives its outputs, so none are needed here.
        self.a = Pin(pin_a, Pin.IN)
        self.b = Pin(pin_b, Pin.IN)
        self.count = 0
        self._state = (self.a.value() << 1) | self.b.value()
        trig = Pin.IRQ_RISING | Pin.IRQ_FALLING
        self.a.irq(handler=self._isr, trigger=trig)
        self.b.irq(handler=self._isr, trigger=trig)

    def _isr(self, _pin):
        s = (self.a.value() << 1) | self.b.value()
        self.count += QuadEncoder._TABLE[(self._state << 2) | s]
        self._state = s

    def read(self):
        return self.count


# =====================================================================
# SERVO
# =====================================================================
class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50, duty_u16=0)
        self.angle = None

    def write(self, deg):
        deg = max(0.0, min(180.0, deg))
        us = SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0
        self.pwm.duty_ns(int(us * 1000))
        self.angle = deg

    def off(self):
        self.pwm.duty_u16(0)


# =====================================================================
# KINEMATICS
# =====================================================================
L1 = ARM1_LENGTH
L2 = ARM2_LENGTH


def servo_to_joint(s1, s2):
    """Servo degrees -> joint angles in radians."""
    q1 = math.radians((s1 - SHOULDER_OFFSET) * SHOULDER_DIR)
    q2 = math.radians((s2 - ELBOW_OFFSET) * ELBOW_DIR)
    return q1, q2


def joint_to_servo(q1, q2):
    """Joint angles in radians -> servo degrees (shoulder wrapped to +-180)."""
    s1 = SHOULDER_OFFSET + SHOULDER_DIR * math.degrees(q1)
    s2 = ELBOW_OFFSET + ELBOW_DIR * math.degrees(q2)
    s1 = ((s1 + 180.0) % 360.0) - 180.0
    return s1, s2


def forward(s1, s2):
    """Servo angles -> pointer tip (x, y) in mm."""
    q1, q2 = servo_to_joint(s1, s2)
    x = L1 * math.cos(q1) + L2 * math.cos(q1 + q2)
    y = L1 * math.sin(q1) + L2 * math.sin(q1 + q2)
    return x, y


def inverse(x, y, prev):
    """
    Pointer tip (x, y) -> (shoulder_deg, elbow_deg), or None if unreachable.
    Tries both elbow solutions and keeps the one that stays inside the
    servo range and is closest to the current pose (prev).
    """
    c = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if c < -1.0 or c > 1.0:
        return None                       # outside the reachable ring

    best = None
    best_cost = MAX_STEP_DEG * 2
    for sign in (1, -1):                  # elbow-left / elbow-right
        q2 = sign * math.acos(c)
        q1 = math.atan2(y, x) - math.atan2(L2 * math.sin(q2), L1 + L2 * math.cos(q2))
        s1, s2 = joint_to_servo(q1, q2)
        if not (-0.5 <= s1 <= 180.5 and -0.5 <= s2 <= 180.5):
            continue                      # servo can't physically get there
        d1 = abs(s1 - prev[0])
        d2 = abs(s2 - prev[1])
        if d1 > MAX_STEP_DEG or d2 > MAX_STEP_DEG:
            continue                      # would snap / flip the elbow
        if d1 + d2 < best_cost:
            best_cost = d1 + d2
            best = (s1, s2)
    return best


# =====================================================================
# MAIN
# =====================================================================
def main():
    shoulder = Servo(SERVO1_PIN)
    elbow = Servo(SERVO2_PIN)
    enc_x = QuadEncoder(*ENC_X_PINS)
    enc_y = QuadEncoder(*ENC_Y_PINS)

    shoulder.write(HOME_SHOULDER)
    elbow.write(HOME_ELBOW)
    tx, ty = forward(HOME_SHOULDER, HOME_ELBOW)
    print("Home tip: x=%.1f  y=%.1f mm" % (tx, ty))
    time.sleep_ms(500)

    last_cx = enc_x.read()
    last_cy = enc_y.read()
    last_print = time.ticks_ms()

    try:
        while True:
            cx = enc_x.read()
            cy = enc_y.read()
            dx = (cx - last_cx) * MM_PER_COUNT_X
            dy = (cy - last_cy) * MM_PER_COUNT_Y
            last_cx, last_cy = cx, cy

            if dx or dy:
                prev = (shoulder.angle, elbow.angle)
                # Try the full move first. If it's blocked, try each axis
                # alone so hitting the edge in X doesn't freeze Y (and vice versa).
                for nx, ny in ((tx + dx, ty + dy), (tx + dx, ty), (tx, ty + dy)):
                    sol = inverse(nx, ny, prev)
                    if sol:
                        tx, ty = nx, ny
                        shoulder.write(sol[0])
                        elbow.write(sol[1])
                        break
                # If nothing worked, the tip stays put: turning the knob
                # further into the edge of the workspace does nothing.

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                print("tip x=%6.1f y=%6.1f | shoulder=%5.1f elbow=%5.1f | enc %d %d"
                      % (tx, ty, shoulder.angle, elbow.angle, cx, cy))

            time.sleep_ms(LOOP_MS)
    except KeyboardInterrupt:
        shoulder.off()
        elbow.off()
        print("Stopped")


main()

