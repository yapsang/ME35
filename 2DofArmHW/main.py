"""
2-DOF planar arm: each hand-turned DC motor encoder sets the angle of one servo.
    Encoder 1 -> servo 1 (shoulder, D4)
    Encoder 2 -> servo 2 (elbow,    D5)

Hardware: ROBO ESP32, 2x Miuzel MG90S servos, 2x DC motors with
          DFRobot Encoder v2.0 used as input knobs (motors are not powered).

The tip (x, y) is calculated from the two servo angles and printed for
reference. It is not used for control.
"""

from machine import Pin, PWM
import math
import time

# =====================================================================
# GEOMETRY  -- pivot to pivot, used only for the printed tip position
# =====================================================================
ARM1_LENGTH = 100.0   # shoulder shaft -> elbow shaft (mm)
ARM2_LENGTH = 100.0   # elbow shaft   -> pointer tip  (mm)

# =====================================================================
# PINS
# =====================================================================
SERVO1_PIN = 4            # shoulder servo (D4)
SERVO2_PIN = 5            # elbow servo    (D5)
ENC1_PINS = (39, 32)      # encoder driving servo 1 (A, B)
ENC2_PINS = (25, 26)      # encoder driving servo 2 (A, B)

# =====================================================================
# SERVO CALIBRATION
# =====================================================================
SERVO_MIN_US = 500        # pulse width at 0 deg   (MG90S ~500 us)
SERVO_MAX_US = 2500       # pulse width at 180 deg (MG90S ~2500 us)
SERVO_MIN_DEG = 0.0       # software travel limits
SERVO_MAX_DEG = 180.0

# Starting pose: both servos at their midpoint (reset every run).
HOME_SERVO1 = 90.0
HOME_SERVO2 = 90.0

# Used only for the printed tip position:
#   servo 1 reading when arm 1 points along +x
#   servo 2 reading when arm 2 is straight in line with arm 1
SHOULDER_OFFSET = 0.0
ELBOW_OFFSET = 90.0

# =====================================================================
# INPUT FEEL
# =====================================================================
# Servo degrees per encoder count. The encoder is on the motor shaft,
# before the gearbox, so it gives many counts per output turn: keep this
# small. Make one negative to reverse that knob.
DEG_PER_COUNT_1 = 0.05
DEG_PER_COUNT_2 = 0.05
LOOP_MS = 20              # 50 Hz update, matches the servo PWM frame

# Anti-jitter
SMOOTHING = 0.3           # 0-1: fraction of the way to the target each loop
                          # (lower = smoother but laggier, 1 = no smoothing)
DEADBAND_US = 8           # only send a new pulse if it changed by >= this
                          # many microseconds (~0.7 deg). Raise if still jittery.
PRINT_MS = 250            # status print interval


# =====================================================================
# ENCODER (quadrature, counts every edge on A and B = 4x resolution)
# CW: 00 -> 10 -> 11 -> 01 counts up, CCW counts down.
# =====================================================================
class QuadEncoder:
    # index = (old_state << 2) | new_state, state = (A << 1) | B
    _TABLE = (0, -1, 1, 0,
              1, 0, 0, -1,
              -1, 0, 0, 1,
              0, 1, -1, 0)

    def __init__(self, pin_a, pin_b):
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
    """
    angle   = target set by the encoder (what the knob asks for)
    _pos    = smoothed position actually sent to the servo
    The PWM is only rewritten when the pulse changes by DEADBAND_US or more,
    so tiny encoder wobbles and float noise don't reach the servo.
    """
    def __init__(self, pin):
        self.pwm = PWM(Pin(pin), freq=50, duty_u16=0)
        self.angle = None
        self._pos = None
        self._last_us = None

    @staticmethod
    def _to_us(deg):
        return int(round(SERVO_MIN_US + (SERVO_MAX_US - SERVO_MIN_US) * deg / 180.0))

    def _send(self, us):
        self.pwm.duty_ns(us * 1000)
        self._last_us = us

    def write(self, deg):
        """Jump straight to an angle (used for the startup reset)."""
        deg = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, deg))
        self.angle = deg
        self._pos = deg
        self._send(self._to_us(deg))

    def set_target(self, deg):
        self.angle = max(SERVO_MIN_DEG, min(SERVO_MAX_DEG, deg))

    def update(self):
        """Call every loop: ease toward the target, write only real changes."""
        self._pos += (self.angle - self._pos) * SMOOTHING
        us = self._to_us(self._pos)
        target_us = self._to_us(self.angle)
        # Write if the change is big enough, or if we've arrived at the target
        # (so the servo settles exactly where the knob says, not 8 us short).
        if abs(us - self._last_us) >= DEADBAND_US or (us == target_us and us != self._last_us):
            self._send(us)

    def off(self):
        self.pwm.duty_u16(0)


# =====================================================================
# TIP POSITION (forward kinematics, for display only)
# tip = R(q1)*[L1, 0] + R(q1 + q2)*[L2, 0]
# =====================================================================
def tip_position(s1, s2):
    q1 = math.radians(s1 - SHOULDER_OFFSET)
    q2 = math.radians(s2 - ELBOW_OFFSET)
    x = ARM1_LENGTH * math.cos(q1) + ARM2_LENGTH * math.cos(q1 + q2)
    y = ARM1_LENGTH * math.sin(q1) + ARM2_LENGTH * math.sin(q1 + q2)
    return x, y


# =====================================================================
# MAIN
# =====================================================================
def main():
    servo1 = Servo(SERVO1_PIN)
    servo2 = Servo(SERVO2_PIN)
    enc1 = QuadEncoder(*ENC1_PINS)
    enc2 = QuadEncoder(*ENC2_PINS)

    # Reset to the midpoint every run
    servo1.write(HOME_SERVO1)
    servo2.write(HOME_SERVO2)
    time.sleep_ms(500)
    print("set")
    tx, ty = tip_position(servo1.angle, servo2.angle)
    print("Home: servo1=%.1f  servo2=%.1f | tip x=%.1f y=%.1f mm"
          % (servo1.angle, servo2.angle, tx, ty))

    last_c1 = enc1.read()
    last_c2 = enc2.read()
    last_print = time.ticks_ms()

    try:
        while True:
            c1 = enc1.read()
            c2 = enc2.read()
            d1 = (c1 - last_c1) * DEG_PER_COUNT_1
            d2 = (c2 - last_c2) * DEG_PER_COUNT_2
            last_c1, last_c2 = c1, c2

            # Each knob nudges its own servo's target. Targets clamp to 0-180,
            # so turning past a limit does nothing and reversing responds
            # immediately.
            if d1:
                servo1.set_target(servo1.angle + d1)
            if d2:
                servo2.set_target(servo2.angle + d2)
            servo1.update()
            servo2.update()

            now = time.ticks_ms()
            if time.ticks_diff(now, last_print) >= PRINT_MS:
                last_print = now
                tx, ty = tip_position(servo1.angle, servo2.angle)
                print("servo1=%5.1f servo2=%5.1f | tip x=%6.1f y=%6.1f | enc %d %d"
                      % (servo1.angle, servo2.angle, tx, ty, c1, c2))

            time.sleep_ms(LOOP_MS)
    except KeyboardInterrupt:
        servo1.off()
        servo2.off()
        print("Stopped")


main()