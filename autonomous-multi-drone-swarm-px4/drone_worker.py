import time
from pymavlink import mavutil


class DroneWorker:

    def __init__(self, sys_id, port, target):
        self.sys_id = sys_id
        self.port = port
        self.target = target    # (lat, lon, alt) — takeoff = RTL dönüş noktası
        self.master = None

    def log(self, msg):
        print(f"[Drone {self.sys_id}] {msg}", flush=True)

    def connect(self, timeout=30):
        self.log(f"Connecting on port {self.port}")
        # udpout: = GCS modu (PX4'e paket gönder, cevap al)
        # udp:    = bind modu (PX4 zaten portu tutuyor → heartbeat gelmiyor)
        self.master = mavutil.mavlink_connection(
            f'udpout:127.0.0.1:{self.port}',
            source_system=255,
            source_component=190
        )
        # PX4'in bizi fark etmesi için önce heartbeat gönder
        for _ in range(10):
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            import time as _t; _t.sleep(0.1)
        hb = self.master.wait_heartbeat(timeout=timeout)
        if not hb:
            raise RuntimeError(f"No heartbeat on port {self.port} after {timeout}s")
        self.log(f"Connected — sysid={self.master.target_system} "
                 f"compid={self.master.target_component}")

    def send_setpoint(self, lat, lon, alt):
        self.master.mav.set_position_target_global_int_send(
            0,
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT,
            0b0000111111111000,
            int(lat * 1e7),
            int(lon * 1e7),
            alt,
            0, 0, 0,
            0, 0, 0,
            0, 0
        )

    def wait_for_position(self, timeout=60):
        self.log("Waiting for EKF / valid global position...")
        start = time.time()
        while time.time() - start < timeout:
            msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
            if msg and msg.lat != 0 and msg.lon != 0:
                self.log(f"Position acquired: {msg.lat / 1e7}, {msg.lon / 1e7}")
                return True
        raise RuntimeError("Timeout waiting for global position")

    def start_offboard(self):
        """Setpoint stream başlat, sonra OFFBOARD mod iste."""
        self.wait_for_position()
        self.log("Streaming setpoints for OFFBOARD.")
        for _ in range(20):     # ~2 sn — PX4 en az 2 sn stream ister
            self.send_setpoint(*self.target)
            time.sleep(0.1)

        # OFFBOARD'a geçene kadar tekrar tekrar dene
        for attempt in range(15):
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                1,
                6,      # PX4 OFFBOARD custom mode
                0, 0, 0, 0, 0
            )
            self.log(f"OFFBOARD mode requested (Attempt {attempt+1})")
            
            # Cevap bekle ve stream et
            entered = False
            for _ in range(10):
                self.send_setpoint(*self.target)
                msg = self.master.recv_match(type='HEARTBEAT', blocking=True, timeout=0.2)
                if msg and (msg.custom_mode == 393216 or getattr(msg, 'custom_mode', 0) > 0): 
                    entered = True
                    break
            
            if entered:
                self.log("OFFBOARD mode confirmed!")
                return
        
        self.log("WARNING: Failed to enter OFFBOARD mode!")

    def arm(self, retries=10):
        """ARM iste; her denemede setpoint stream canlı tut."""
        for attempt in range(1, retries + 1):
            self.log(f"Arm attempt {attempt}/{retries}.")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )
            
            # Beklerken 10Hz stream etmeye devam et (OFFBOARD düşmesin diye)
            start_wait = time.time()
            success = False
            while time.time() - start_wait < 4:
                self.send_setpoint(*self.target)
                ack = self.master.recv_match(type='COMMAND_ACK', blocking=False)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.log("Armed ")
                        return
                    else:
                        self.log(f"Arm rejected (result: {ack.result})")
                        break # Rejected, don't wait full 4 seconds
                time.sleep(0.1)
                
            self.log("Arm failed, recovering OFFBOARD mode and retrying...")
            # Eğer timeout olduysa veya reddedildiyse, OFFBOARD düşmüş olabilir. Tekrar bağlan.
            self.start_offboard()
            
        self.log("WARNING: arm could not be confirmed, continuing.")

    def get_position(self):
        # MAVLink kuyruğundaki son konumu al (blocking yapmadan, OFFBOARD stream'i yavaşlatmamak için)
        msg = None
        while True:
            m = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=False)
            if not m:
                break
            msg = m
            
        if msg:
            self._last_pos = (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
            
        return getattr(self, '_last_pos', None)

    def distance_to_target(self, tlat, tlon, talt):
        pos = self.get_position()
        if not pos:
            return float('inf')
        dlat = (pos[0] - tlat) * 111320
        dlon = (pos[1] - tlon) * 111320 * 0.7
        dalt = pos[2] - talt
        return (dlat**2 + dlon**2 + dalt**2) ** 0.5

    def goto_waypoint(self, lat, lon, alt, tolerance=2.0, max_iter=300):
        self.log(f"Goto ({lat:.7f}, {lon:.7f}, {alt}m)")
        dist = float('inf')
        for i in range(max_iter):
            self.send_setpoint(lat, lon, alt)
            time.sleep(0.1)
            dist = self.distance_to_target(lat, lon, alt)
            if dist < tolerance:
                self.log(f"Waypoint reached (dist={dist:.2f}m) ")
                return
            if i % 50 == 0:
                self.log(f"  dist={dist:.2f}m")
        self.log(f"Timeout! dist={dist:.2f}m")

    def return_to_launch(self):
        """RTL komutu gönder, yere inene kadar bekle."""
        self.log("RTL commanded")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.log("Waiting for landing...")
        for _ in range(120):    # max 120 sn
            msg = self.master.recv_match(
                type='EXTENDED_SYS_STATE', blocking=True, timeout=1
            )
            if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                self.log("Landed at launch point")
                return
            time.sleep(1)
        self.log("RTL timeout (drone may still be descending)")

    def run(self, waypoints=None):
        self.connect()
        self.start_offboard()
        self.arm()

        self.log(f"Flying to takeoff alt: {self.target}")
        self.goto_waypoint(*self.target)

        if waypoints:
            self.log(f"Mission: {len(waypoints)} waypoints")
            for idx, wp in enumerate(waypoints):
                self.log(f"WP {idx+1}/{len(waypoints)}")
                self.goto_waypoint(*wp)
                time.sleep(1)
        else:
            self.log("No waypoints — hovering briefly")
            time.sleep(2)

        self.return_to_launch()
        self.log("Mission completed")
