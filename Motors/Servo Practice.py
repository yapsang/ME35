from machine import PWM, Pin
import time

servo = PWM(Pin(4), freq=50, duty_u16=0)