from machine import Pin
from time import ticks_ms, ticks_diff
import neopixel #importing the library
from machine import Pin # another way of importing a library


btn = Pin(34, Pin.IN, Pin.PULL_UP)
DEBOUNCE_MS = 200
last_press = 0

lights = neopixel.NeoPixel(Pin(15), 2) # 0 is the Pin for neopixel and 4 is the number of lights

lights.write()


def button_handler(pin):
    global last_press
    now = ticks_ms()
    if ticks_diff(now, last_press) > DEBOUNCE_MS:
        if (pin.value() == 0):
            last_press = now
            lights[0] = (20,0,20)
            lights[1] = (20,0,20)
            lights.write()

btn.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)