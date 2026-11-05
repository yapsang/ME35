import network
import espnow
import time
import json
from machine import Pin, PWM
import neopixel

SHOW_MS = 1500                      # how long the colour stays lit
ROBOT_MAC = b'\x20\x50\x0d\x29\x65\x70'

np = neopixel.NeoPixel(Pin(15), 2)
buzzer = PWM(Pin(23), freq=1000, duty=0)

COLORS = {"red": (40, 0, 0), "green": (0, 40, 0), "blue": (0, 0, 40)}


def set_pixels(c):
    for i in range(len(np)):
        np[i] = c
    np.write()


set_pixels((0, 0, 0))

# Wi-Fi interface must be active for ESP-NOW (no connecting needed)
sta = network.WLAN(network.WLAN.IF_STA)
sta.active(True)
sta.disconnect()
print("My MAC as bytes:", sta.config("mac"))

e = espnow.ESPNow()
e.active(True)

message = b''
macadd = b''
new_message = False


def recv_cb(esp):
    global message, macadd, new_message
    while True:                          # drain everything waiting in the buffer
        mac, msg = esp.irecv(0)
        if mac is None:
            return
        end = msg.find(b'}')             # JSON ends at the first closing brace
        message = bytes(msg[:end + 1]) if end >= 0 else b''
        macadd = bytes(mac)              # copy: irecv reuses its buffers
        new_message = True


e.irq(recv_cb)

lit_until = None                         # when to turn the pixels off

while True:
    if new_message:
        new_message = False

        if macadd != ROBOT_MAC:
            print("Ignored message from", ":".join("%02x" % b for b in macadd))
            continue

        try:
            data = json.loads(message)
            state = data["state"]
            payload = data["payload"]
        except (ValueError, KeyError):
            print("Bad message:", message)
            continue

        print(state, payload, "from", ":".join("%02x" % b for b in macadd))

        if state == "marker":
            set_pixels(COLORS.get(payload, (20, 20, 20)))   # white-ish if unknown colour
            lit_until = time.ticks_add(time.ticks_ms(), SHOW_MS)
        elif state == "color":
            set_pixels(tuple(payload))
            lit_until = time.ticks_add(time.ticks_ms(), SHOW_MS)

    # turn the pixels off once the time is up
    if lit_until is not None and time.ticks_diff(time.ticks_ms(), lit_until) >= 0:
        set_pixels((0, 0, 0))
        lit_until = None

    time.sleep_ms(20)
