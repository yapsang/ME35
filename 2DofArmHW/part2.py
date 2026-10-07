"""
Homework 5 Part 2: the 2-DOF arm draws a 5-point star with an LED on its tip.

Steps (matching the assignment):
  1. Shape: a 5-point star drawn in one stroke, the pencil way: each line
     goes to every second tip, so the lines cross in the middle.
  2. Points: the 5 tips sit on a circle, 72 deg apart (x = cx + r cos a,
     y = cy + r sin a). Extra points are filled in along each line so the
     tip moves in straight lines.
  3. Angles: inverse kinematics turns every (x, y) into a pair of servo
     angles, stored in the ANGLES array.
  4. Draw: step the servos through ANGLES with the LED on.

Hardware: ROBO ESP32, 2x Miuzel MG90S servos (D5 = base/shoulder, D4 = elbow),
          LED on GPIO 13.

Coordinate frame (top view):
    Origin = shoulder servo shaft
    +x     = arm 1 direction when servo 1 is at 0 deg
    +y     = arm 1 direction when servo 1 is at 90 deg (straight up at home)
All lengths in mm.
"""

from machine import Pin, PWM
import math
import time

# =====================================================================
# GEOMETRY  -- pivot to pivot
# =====================================================================
L1 = 100.0   # shoulder shaft -> elbow shaft (mm)
L2 = 100.0   # elbow shaft   -> LED          (mm)

# =====================================================================
# PINS
# =====================================================================
SERVO1_PIN = 5            # servo 1, base / shoulder (D5)
SERVO2_PIN = 4            # servo 2, elbow           (D4)
LED_PIN = 13              # LED on the end of the arm

# =====================================================================
# SERVO CALIBRATION
# =====================================================================
SERVO_MIN_US = 500        # pulse width at 0 deg   (MG90S ~500 us)
SERVO_MAX_US = 2500       # pulse width at 180 deg (MG90S ~2500 us)

# Servo reading when the joint angle is zero:
#   shoulder: arm 1 points along +x
#   elbow:    arm 2 is straight in line with arm 1
# Elbow horn is mounted so 180 = arm 2 straight, 0 = folded back toward
# the base (lowering servo 2 bends arm 2 to the right / clockwise).
SHOULDER_OFFSET = 0.0
ELBOW_OFFSET = 180.0
# Flip to -1 if a joint moves the opposite way to what's expected
SHOULDER_DIR = 1
ELBOW_DIR = 1

# Starting pose (reset every run): arm 1 straight up, arm 2 straight
HOME_SERVO1 = 90.0
HOME_SERVO2 = 180.0

# =====================================================================
# STAR
# =====================================================================
# With the elbow folding a full 180 deg, the LED can reach anywhere from
# the shoulder out to L1 + L2 = 200 mm. Because the elbow only bends to
# the right, the roomiest area is up and to the right of the base.
# The star tips sit on the same circle as the circle drawing, so both
# servos stay well away from their end stops
# (shoulder ~84-148 deg, elbow ~34-119 deg).
STAR_CENTER = (50.0, 105.0)   # (x, y) mm
STAR_RADIUS = 60.0            # centre -> tip (mm), star is ~114 mm wide
POINTS_PER_LINE = 30          # points along each of the 5 lines
TIP_ORDER = [0, 2, 4, 1, 3, 0]  # every second tip = pencil star
ELBOW_SIGN = -1               # -1 = elbow bends right (matches the horn mount)

# =====================================================================
# DRAWING
# =====================================================================
STEP_MS = 40              # time between points (bigger = slower, smoother)
CORNER_MS = 200           # extra pause at each tip so the points come out sharp
MOVE_STEPS = 50           # steps for the LED-off move from home to the start
DRAW_LOOPS = 0            # times to draw the star, 0 = forever
PAUSE_MS = 1000           # LED-off pause between stars


# =====================================================================
# SERVO
# =====================================================================
class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50, duty_u16=0)
        self.angle = None
        self._last_us = None

    def write(self, deg):
        deg = max(0.0, min(180.0, deg))
        us = int(round(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0))
        if us != self._last_us:           # don't resend the same pulse (jitter)
            self.pwm.duty_ns(us * 1000)
            self._last_us = us
        self.angle = deg

    def off(self):
        self.pwm.duty_u16(0)


# =====================================================================
# 3x3 MATRICES (homogeneous 2D transforms) -- plain Python, no numpy
# =====================================================================
def mat_mul(A, B):
    return [[sum(A[i][k] * B[k][j] for k in range(3)) for j in range(3)]
            for i in range(3)]


def rot(theta):
    """R(theta): rotate about the origin."""
    c, s = math.cos(theta), math.sin(theta)
    return [[c, -s, 0.0],
            [s,  c, 0.0],
            [0.0, 0.0, 1.0]]


def trans_x(d):
    """T(d): translate by d along the x-axis."""
    return [[1.0, 0.0, d],
            [0.0, 1.0, 0.0],
            [0.0, 0.0, 1.0]]


# =====================================================================
# FORWARD KINEMATICS
# =====================================================================
def forward_matrix(t1, t2):
    """
    Homogeneous transformations, applied to the point at the origin in order:
        T2: translate l2 along x
        R2: rotate by theta2
        T1: translate l1 along x
        R1: rotate by theta1
    Applied in that order, the full matrix is M = R1 * T1 * R2 * T2,
    and the tip is M * [0, 0, 1].
    """
    M = mat_mul(rot(t1), mat_mul(trans_x(L1), mat_mul(rot(t2), trans_x(L2))))
    return M[0][2], M[1][2]               # last column = M * [0, 0, 1]


def forward_trig(t1, t2):
    """Trigonometric method (triangles): same answer as forward_matrix."""
    x = L1 * math.cos(t1) + L2 * math.cos(t1 + t2)
    y = L1 * math.sin(t1) + L2 * math.sin(t1 + t2)
    return x, y


# =====================================================================
# INVERSE KINEMATICS (teacher's equations)
# =====================================================================
def inverse(x, y, elbow_sign=ELBOW_SIGN):
    """
    (x, y) -> (theta1, theta2) in radians, or None if out of reach.
        cos t2 = (x^2 + y^2 - l1^2 - l2^2) / (2 l1 l2)
        sin t2 = +-sqrt(1 - cos^2 t2)      (sign picks elbow up / down)
        t2     = atan2(sin t2, cos t2)
        t1     = atan2(y, x) - atan2(l2 sin t2, l1 + l2 cos t2)
    """
    c2 = (x * x + y * y - L1 * L1 - L2 * L2) / (2.0 * L1 * L2)
    if abs(c2) > 1.0:                     # |l1 - l2| <= r <= l1 + l2 fails
        return None
    s2 = elbow_sign * math.sqrt(1.0 - c2 * c2)
    t2 = math.atan2(s2, c2)
    t1 = math.atan2(y, x) - math.atan2(L2 * s2, L1 + L2 * c2)
    return t1, t2


def joints_to_servos(t1, t2):
    s1 = SHOULDER_OFFSET + SHOULDER_DIR * math.degrees(t1)
    s2 = ELBOW_OFFSET + ELBOW_DIR * math.degrees(t2)
    return s1, s2


def servos_to_joints(s1, s2):
    t1 = math.radians((s1 - SHOULDER_OFFSET) * SHOULDER_DIR)
    t2 = math.radians((s2 - ELBOW_OFFSET) * ELBOW_DIR)
    return t1, t2


# =====================================================================
# STEP 2: STAR POINTS
# =====================================================================
def star_tips():
    """5 tips on a circle, 72 deg apart, starting at the top."""
    cx, cy = STAR_CENTER
    return [(cx + STAR_RADIUS * math.cos(math.radians(90 + 72 * i)),
             cy + STAR_RADIUS * math.sin(math.radians(90 + 72 * i)))
            for i in range(5)]


def star_points():
    """Straight lines tip -> every second tip, back to the start.
    Returns the points and the indices where a tip (corner) is reached."""
    tips = star_tips()
    pts, corners = [], []
    for a, b in zip(TIP_ORDER, TIP_ORDER[1:]):
        (x0, y0), (x1, y1) = tips[a], tips[b]
        corners.append(len(pts))
        for k in range(POINTS_PER_LINE):
            f = k / POINTS_PER_LINE
            pts.append((x0 + (x1 - x0) * f, y0 + (y1 - y0) * f))
    corners.append(len(pts))
    pts.append(tips[TIP_ORDER[-1]])       # close the star
    return pts, corners


# =====================================================================
# STEP 3: ANGLE ARRAY
# =====================================================================
def build_angles(points):
    angles = []
    for (x, y) in points:
        sol = inverse(x, y)
        if sol is None:
            raise ValueError("(%.1f, %.1f) is out of reach" % (x, y))
        s1, s2 = joints_to_servos(*sol)
        if not (0.0 <= s1 <= 180.0 and 0.0 <= s2 <= 180.0):
            raise ValueError("(%.1f, %.1f) needs servos %.1f, %.1f (outside 0-180)"
                             % (x, y, s1, s2))
        angles.append((s1, s2))
    return angles


def check_angles(points, angles):
    """Run each angle pair back through forward kinematics; return worst error (mm)."""
    worst = 0.0
    for (x, y), (s1, s2) in zip(points, angles):
        fx, fy = forward_matrix(*servos_to_joints(s1, s2))
        worst = max(worst, math.sqrt((fx - x) ** 2 + (fy - y) ** 2))
    return worst


# =====================================================================
# STEP 4: DRAW
# =====================================================================
def move_to(servo1, servo2, s1, s2, steps):
    """Glide both servos to (s1, s2) over a number of steps."""
    a1, a2 = servo1.angle, servo2.angle
    for i in range(1, steps + 1):
        f = i / steps
        servo1.write(a1 + (s1 - a1) * f)
        servo2.write(a2 + (s2 - a2) * f)
        time.sleep_ms(STEP_MS)


def draw(servo1, servo2, led, angles, corners=()):
    led.value(1)
    for i, (s1, s2) in enumerate(angles):
        servo1.write(s1)
        servo2.write(s2)
        time.sleep_ms(STEP_MS)
        if i in corners:
            time.sleep_ms(CORNER_MS)
    led.value(0)


# =====================================================================
# MAIN
# =====================================================================
def main():
    servo1 = Servo(SERVO1_PIN)
    servo2 = Servo(SERVO2_PIN)
    led = Pin(LED_PIN, Pin.OUT, value=0)

    # Reset to the midpoint every run
    servo1.write(HOME_SERVO1)
    servo2.write(HOME_SERVO2)
    time.sleep_ms(500)
    print("set")
    hx, hy = forward_matrix(*servos_to_joints(HOME_SERVO1, HOME_SERVO2))
    print("Home: servo1=%.1f servo2=%.1f | tip x=%.1f y=%.1f mm"
          % (HOME_SERVO1, HOME_SERVO2, hx, hy))

    # Steps 2 + 3: points and angle array
    points, corners = star_points()
    angles = build_angles(points)
    print("Star: %d points, worst FK error %.4f mm"
          % (len(points), check_angles(points, angles)))
    print("ANGLES = [")
    for (x, y), (s1, s2) in zip(points, angles):
        print("    (%6.2f, %6.2f),   # x=%6.1f y=%6.1f" % (s1, s2, x, y))
    print("]")

    # Step 4: draw
    try:
        n = 0
        while DRAW_LOOPS == 0 or n < DRAW_LOOPS:
            move_to(servo1, servo2, angles[0][0], angles[0][1], MOVE_STEPS)
            time.sleep_ms(300)            # let the servos settle on the start
            draw(servo1, servo2, led, angles, corners)
            n += 1
            print("star %d done" % n)
            time.sleep_ms(PAUSE_MS)
        move_to(servo1, servo2, HOME_SERVO1, HOME_SERVO2, MOVE_STEPS)
    except KeyboardInterrupt:
        pass
    led.value(0)
    servo1.off()
    servo2.off()
    print("Stopped")


main()