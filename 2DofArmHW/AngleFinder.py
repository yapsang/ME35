"""
2-DOF planar arm: type two joint angles in the serial console and the arm
moves there.

    Shoulder servo -> D4
    Elbow servo    -> D5

Hardware: ROBO ESP32, 2x Miuzel MG90S servos.

Input (at the "> " prompt):
    45 30        shoulder = 45 deg, elbow = 30 deg   (comma also works: 45,30)
    s 120 60     raw SERVO angles instead of joint angles
    home         return to the home pose
    q            stop and release the servos

Joint angles (same convention as the forward kinematics):
    shoulder: angle of arm 1 measured from +x
    elbow:    angle of arm 2 relative to arm 1 (0 = straight in line)
"""

from machine import Pin, PWM
import math
import time

# =====================================================================
# GEOMETRY  -- pivot to pivot
# =====================================================================
ARM1_LENGTH = 100.0   # shoulder shaft -> elbow shaft (mm)
ARM2_LENGTH = 100.0   # elbow shaft   -> pointer tip  (mm)

# =====================================================================
# PINS
# =====================================================================
SERVO1_PIN = 4            # shoulder servo (D4)
SERVO2_PIN = 5            # elbow servo    (D5)

# =====================================================================
# SERVO CALIBRATION
# =====================================================================
SERVO_MIN_US = 500        # pulse width at 0 deg   (MG90S ~500 us)
SERVO_MAX_US = 2500       # pulse width at 180 deg (MG90S ~2500 us)
SERVO_MIN_DEG = 0.0       # software travel limits
SERVO_MAX_DEG = 180.0

HOME_SERVO1 = 90.0
HOME_SERVO2 = 90.0

# Joint angle -> servo angle:  servo = joint + OFFSET
#   SHOULDER_OFFSET: servo 1 reading when arm 1 points along +x
#   ELBOW_OFFSET:    servo 2 reading when arm 2 is straight in line with arm 1
SHOULDER_OFFSET = 0.0
ELBOW_OFFSET = 90.0

# =====================================================================
# MOTION
# =====================================================================
MOVE_SPEED_DEG_S = 90.0   # speed of the joint that has farthest to go;
                          # the other is scaled so both arrive together
LOOP_MS = 20              # 50 Hz, matches the servo PWM frame


# =====================================================================
# SERVO
# =====================================================================
class Servo:
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50, duty_u16=0)
        self.angle = None

    @staticmethod
    def _to_us(deg):
        return int(round(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0))

    def write(self, deg):
        deg = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, deg))
        self.angle = deg
        self.pwm.duty_ns(self._to_us(deg) * 1000)

    def off(self):
        self.pwm.duty_u16(0)


# =====================================================================
# KINEMATICS
# =====================================================================
def joint_to_servo(q1, q2):
    return q1 + SHOULDER_OFFSET, q2 + ELBOW_OFFSET


def servo_to_joint(s1, s2):
    return s1 - SHOULDER_OFFSET, s2 - ELBOW_OFFSET


def tip_position(s1, s2):
    q1, q2 = servo_to_joint(s1, s2)
    q1 = math.radians(q1)
    q2 = math.radians(q2)
    x = ARM1_LENGTH * math.cos(q1) + ARM2_LENGTH * math.cos(q1 + q2)
    y = ARM1_LENGTH * math.sin(q1) + ARM2_LENGTH * math.sin(q1 + q2)
    return x, y


def in_range(deg):
    return SERVO_MIN_DEG <= deg <= SERVO_MAX_DEG


# =====================================================================
# MOVE (straight-line interpolation in joint space, both finish together)
# =====================================================================
def move_to(servo1, servo2, s1_target, s2_target):
    s1_start, s2_start = servo1.angle, servo2.angle
    d1 = s1_target - s1_start
    d2 = s2_target - s2_start
    dist = max(abs(d1), abs(d2))
    if dist < 0.01:
        return

    duration_ms = dist / MOVE_SPEED_DEG_S * 1000.0
    t0 = time.ticks_ms()
    while True:
        t = time.ticks_diff(time.ticks_ms(), t0) / duration_ms
        if t >= 1.0:
            break
        # smoothstep easing: gentle start and stop
        e = t * t * (3.0 - 2.0 * t)
        servo1.write(s1_start + d1 * e)
        servo2.write(s2_start + d2 * e)
        time.sleep_ms(LOOP_MS)

    servo1.write(s1_target)
    servo2.write(s2_target)


def report(servo1, servo2):
    q1, q2 = servo_to_joint(servo1.angle, servo2.angle)
    x, y = tip_position(servo1.angle, servo2.angle)
    print("joints: shoulder=%.1f elbow=%.1f | servos: %.1f %.1f | tip x=%.1f y=%.1f mm"
          % (q1, q2, servo1.angle, servo2.angle, x, y))


# =====================================================================
# MAIN
# =====================================================================
def parse_two(parts):
    if len(parts) != 2:
        raise ValueError
    return float(parts[0]), float(parts[1])


def main():
    servo1 = Servo(SERVO1_PIN)
    servo2 = Servo(SERVO2_PIN)

    servo1.write(HOME_SERVO1)
    servo2.write(HOME_SERVO2)
    time.sleep_ms(500)
    print("Home.")
    report(servo1, servo2)

    q1_lo, q2_lo = servo_to_joint(SERVO_MIN_DEG, SERVO_MIN_DEG)
    q1_hi, q2_hi = servo_to_joint(SERVO_MAX_DEG, SERVO_MAX_DEG)
    print("Enter: <shoulder> <elbow>   (shoulder %.0f..%.0f, elbow %.0f..%.0f)"
          % (q1_lo, q1_hi, q2_lo, q2_hi))
    print("       s <servo1> <servo2>  |  home  |  q")

    try:
        while True:
            line = input("> ").strip().lower().replace(",", " ")
            if not line:
                continue
            if line in ("q", "quit", "exit"):
                break
            if line == "home":
                move_to(servo1, servo2, HOME_SERVO1, HOME_SERVO2)
                report(servo1, servo2)
                continue

            parts = line.split()
            try:
                if parts[0] == "s":
                    s1, s2 = parse_two(parts[1:])
                else:
                    s1, s2 = joint_to_servo(*parse_two(parts))
            except ValueError:
                print("Couldn't read that. Example: 45 30")
                continue

            if not (in_range(s1) and in_range(s2)):
                print("Out of range: servos would be %.1f, %.1f (limits %.0f-%.0f)"
                      % (s1, s2, SERVO_MIN_DEG, SERVO_MAX_DEG))
                continue

            move_to(servo1, servo2, s1, s2)
            report(servo1, servo2)
    except KeyboardInterrupt:
        pass

    servo1.off()
    servo2.off()
    print("Stopped")


main()