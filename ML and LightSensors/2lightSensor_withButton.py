from machine import ADC, Pin
from time import ticks_diff, ticks_ms

lightsensor = ADC(Pin(36))

btn = Pin(34, Pin.IN, Pin.PULL_UP) 
DEBOUNCE_MS = 200
last_press = 0

btn2 = Pin(35, Pin.IN, Pin.PULL_UP)
values = list()


pressed_flag = False
pressed_flag2 = False

def button_handler(pin):
    global last_press
    global pressed_flag
    now = ticks_ms()
    if ticks_diff(now, last_press) > DEBOUNCE_MS:
        if (pin.value() == 0):
            last_press = now
            pressed_flag = True

def button2_handler(pin):
    global last_press2
    global pressed_flag2
    now = ticks_ms()
    if ticks_diff(now, last_press) > DEBOUNCE_MS:
        if (pin.value() == 0):
            last_press2 = now
            pressed_flag2 = True

btn.irq(trigger=Pin.IRQ_FALLING, handler=button_handler)
btn2.irq(trigger=Pin.IRQ_FALLING, handler=button2_handler)

while True:
    if pressed_flag:
        value = lightsensor.read_u16()
        print(value)
        values.append(value)
        pressed_flag = False
        
    if pressed_flag2:
        print(values)
        pressed_flag2 = False
    
