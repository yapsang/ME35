import network
import ubinascii

wlan = network.WLAN()
mac = wlan.config("mac")


mac_escaped = "".join(f"\\x{b:02x}" for b in mac)
print(f"b'{mac_escaped}'")


mac_readable = ubinascii.hexlify(mac)
print(mac_readable)