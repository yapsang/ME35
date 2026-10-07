from machine import Pin, PWM
s1 = PWM(Pin(5), freq=50); s2 = PWM(Pin(4), freq=50)
us = lambda d: int((500 + 2000*d/180) * 1000)