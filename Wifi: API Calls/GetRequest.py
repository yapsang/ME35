import urequests
DATE_URL = "https://worldtimeapi.org/api/timezone/America/New_York"
reply = urequests.get(DATE_URL)
print(reply.json()['datetime'])
