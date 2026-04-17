import time
from pymavlink import mavutil
import sys

def main():
    master = mavutil.mavlink_connection('udpout:127.0.0.1:14581', source_system=255, source_component=190)
    for _ in range(5):
        master.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
        time.sleep(0.1)
    
    hb = master.wait_heartbeat(timeout=3)
    if not hb:
        print("No heartbeat")
        sys.exit(1)
        
    master.mav.request_data_stream_send(master.target_system, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
    
    start = time.time()
    while time.time() - start < 3:
        msg = master.recv_match(type='VFR_HUD', blocking=True, timeout=1)
        if msg:
            print(f"VFR_HUD: alt={msg.alt}, throttle={msg.throttle}")
        msg2 = master.recv_match(type='ATTITUDE', blocking=False)
        if msg2:
            print(f"ATTITUDE: {msg2.pitch}")

main()
