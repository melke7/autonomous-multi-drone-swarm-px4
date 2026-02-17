#!/usr/bin/env python3

import threading
import time
from pymavlink import mavutil


PORTS = list(range(14540, 14547))   # 14540 .. 14546
TIMEOUT = 10                         # saniye


def test_port(port, results):
    try:
        conn = mavutil.mavlink_connection(f'udp:127.0.0.1:{port}')
        hb = conn.wait_heartbeat(timeout=TIMEOUT)
        if hb:
            results[port] = f"OK  sysid={conn.target_system} compid={conn.target_component}"
        else:
            results[port] = "TIMEOUT - heartbeat gelmedi"
    except Exception as e:
        results[port] = f"ERROR: {e}"


if __name__ == "__main__":
    results = {}
    threads = []

    print(f"[DIAGNOSE] {len(PORTS)} portu {TIMEOUT}s timeout ile test ediliyor...\n")

    for port in PORTS:
        t = threading.Thread(target=test_port, args=(port, results))
        t.start()
        threads.append(t)

    for t in threads:
        t.join()

    print("=" * 50)
    for port in PORTS:
        drone_num = port - 14540   # sys_id (0-indexed)
        status = results.get(port, "?")
        marker = "" if "TIMEOUT" in status or "ERROR" in status else ""
        print(f"  {marker}  Port {port}  (sys_id={drone_num})  ->  {status}")
    print("=" * 50)
