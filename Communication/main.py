import bluetooth
import time
from machine import Pin

button_a = Pin(34, Pin.IN)

NAME = b"ESP_HUGO" # you can choose your name
ID_low = 0xFF    # change these to your company ID - 2 bytes
ID_high = 0xFF

ble = bluetooth.BLE()
ble.active(True)

last_state = None

while True:
    state = button_a.value()
    if state != last_state:
        
        # expect to see this format
        # number of bytes, type and data when dealing with BLE
        
        name_field = bytes([len(NAME) + 1, 0x09]) + NAME
        mfg_field = bytes([4, 0xFF, ID_low, ID_high, state]) # 0xFF says it contains data
        #advertise the data once every 100ms. until you stop the code 
        ble.gap_advertise(100_000, name_field + mfg_field, connectable=False)
        last_state = state

    time.sleep_ms(20)
