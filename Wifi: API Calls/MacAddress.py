import network
wlan = network.WLAN()
mac = wlan.config("mac")
print(mac)

import ubinascii
mac_readable = ubinascii.hexlify(mac,":").decode()
print(mac_readable)