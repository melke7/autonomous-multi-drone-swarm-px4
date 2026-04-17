import socket, threading, time

PX4_PORTS = [14550, 14581, 14582, 14583, 14584, 14585, 14586, 14587]
AGG_PORT = 14600

agg = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)

def proxy_drone(px4_port):
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.settimeout(0.5)
    px4_addr = ('127.0.0.1', px4_port)
    print(f"Connecting to PX4 on port {px4_port}")
    while True:
        try:
            sock.sendto(b'\x00', px4_addr)
            data, _ = sock.recvfrom(65535)
            agg.sendto(data, ('127.0.0.1', AGG_PORT))
        except socket.timeout:
            pass
        time.sleep(0.05)

for p in PX4_PORTS:
    threading.Thread(target=proxy_drone, args=(p,), daemon=True).start()

print(f"All drones -> port {AGG_PORT}")
print("Ctrl+C to stop")
try:
    while True:
        time.sleep(1)
except KeyboardInterrupt:
    pass
