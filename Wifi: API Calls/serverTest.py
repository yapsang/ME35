import network
import socket
import machine
import gc

led = machine.Pin(2, machine.Pin.OUT)
led.off()

WIFI_SSID = "tufts_eecs"
WIFI_PASSWORD = "foundedin1883"

wlan = network.WLAN(network.STA_IF)
wlan.active(True)
wlan.connect(WIFI_SSID, WIFI_PASSWORD)

print("Connecting to Wi-Fi...", end="")
while not wlan.isconnected():
    time.sleep(0.1) #just wait till it connects
    
print("\nConnected! Network config:", wlan.ifconfig())

def get_html():
    return f"""<!DOCTYPE html>
<html>
<head>
    <title>ESP32 Web Server</title>
</head>
<body>
    <h1>ESP32 Control Panel</h1>
    <form method="POST" action="/">
        <button name="led" value="on" type="submit">Turn ON</button>
        <button name="led" value="off" type="submit">Turn OFF</button>
    </form>
</body>
</html>
"""

server_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
server_socket.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
server_socket.bind(('', 80))
server_socket.listen(5)

print("Open the IP address above in your browser.")

while True:
    try:
        gc.collect()
        
        conn, addr = server_socket.accept()
        print(f"\nConnection received from: {addr}")
        
        # Read the raw request (up to 2048 bytes)
        request = conn.recv(2048).decode('utf-8')
        
        # Split headers to find the request line
        lines = request.split('\r\n')
        if not lines or len(lines[0]) == 0:
            conn.close()
            continue
            
        request_line = lines[0]
        method, path, _ = request_line.split(' ')
        print(f"Method: {method}, Path: {path}")

        # Handle POST Request
        if method == "POST":
            # Extract POST body (it comes after the empty line '\r\n\r\n')
            parts = request.split('\r\n\r\n')
            if len(parts) > 1:
                body = parts[1]
                print(f"POST Payload: {body}")
                
                # Check the form values
                if "led=on" in body:
                    led.on()
                elif "led=off" in body:
                    led.off()

            # Redirect back to home after POST to prevent duplicate submissions on refresh
            response_headers = "HTTP/1.1 303 See Other\r\nLocation: /\r\n\r\n"
            conn.send(response_headers)

        # Handle GET Request
        else:
            response_body = get_html()
            response_headers = (
                "HTTP/1.1 200 OK\r\n"
                "Content-Type: text/html\r\n"
                f"Content-Length: {len(response_body)}\r\n"
                "Connection: close\r\n\r\n"
            )
            conn.send(response_headers)
            conn.send(response_body)

        conn.close()
        
    except Exception as e:
        print(f"Error encountered: {e}")
        if 'conn' in locals():
            conn.close()

