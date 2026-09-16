from machine import Pin
from time import ticks_ms, ticks_diff
import neopixel #importing the library
import time

btn = Pin(34, Pin.IN, Pin.PULL_UP) 
DEBOUNCE_MS = 20
lights = neopixel.NeoPixel(Pin(15), 2) # 0 is the Pin for neopixel and 4 is the number of lights

while True:
    if btn.value() == 0:
        time.sleep_ms(DEBOUNCE_MS)
        if btn.value() == 0:
            while btn.value() == 0:
                lights[0] = (20,0,20)
                lights[1] = (20,0,20)
                lights.write()
    lights[0] = (0,0,0)
    lights[1] = (0,0,0)
    lights.write()
                
                