import network
import espnow
import time
import json

import neopixel
from machine import Pin
np = neopixel.NeoPixel(Pin(15),2)


command = {"state": "color", "payload": []}


# A WLAN interface must be active to send()/recv()
sta = network.WLAN(network.WLAN.IF_STA)
sta.active(True)
sta.disconnect()   # Because ESP8266 auto-connects to last Access Point

e = espnow.ESPNow()
e.active(True)

peer = b'\x10\x52\x1c\x75\xb2\xa8'  #CHANGE THIS
command["payload"] = [10,10,10] #CHANGE THIS


e.add_peer(peer)      # Must add_peer() before send()

mac_addresses = []
message = b''
macadd = b''
new_message = False

def recv_cb(esp):
    global message, macadd, new_message
    if esp.any():
        mac, msg = esp.irecv(0)  # Read immediate message buffer
        if mac is not None:
            #print(mac, msg)
            message = msg
            macadd = mac
            new_message = True
            
            
e.irq(recv_cb)

while True:
    if new_message:
        print(message, macadd)
        state = json.loads(message)["state"]
        payload = json.loads(message)["payload"]
        if state == "normal":
            pass
        elif state == "color":
            np[0] = payload
            np.write()
        elif state == "buzzer":
            #do something with buzzer
            print("tweeet")
        print(payload)
        time.sleep(0.1)
        e.send(peer, json.dumps(command)) 
        new_message = False
    time.sleep(1)
    
