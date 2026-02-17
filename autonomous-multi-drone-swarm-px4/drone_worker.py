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
        self.master = mavutil.mavlink_connection(f'udp:127.0.0.1:{self.port}')
        hb = self.master.wait_heartbeat(timeout=timeout)
        if not hb:
            raise RuntimeError(f"No heartbeat on port {self.port} after {timeout}s")
        # sys_id'yi PX4'ten gelen gerçek değerden al
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

    def start_offboard(self):
        """Setpoint stream başlat, sonra OFFBOARD mod iste."""
        self.log("Streaming setpoints for OFFBOARD.")
        for _ in range(30):     # ~3 sn — PX4 en az 2 sn stream ister
            self.send_setpoint(*self.target)
            time.sleep(0.1)

        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_DO_SET_MODE,
            0,
            1,
            6,      # PX4 OFFBOARD custom mode
            0, 0, 0, 0, 0
        )
        self.log("OFFBOARD mode requested")
        time.sleep(2)

    def arm(self, retries=5):
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
            ack = self.master.recv_match(
                type='COMMAND_ACK', blocking=True, timeout=3
            )
            if ack and ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                self.log("Armed ")
                return
            result_str = str(ack.result) if ack else "timeout"
            self.log(f"Arm failed ({result_str}), retrying")
            # OFFBOARD mod canlı kalsın
            for _ in range(20):
                self.send_setpoint(*self.target)
                time.sleep(0.1)
        self.log("WARNING: arm could not be confirmed, continuing.")

    def get_position(self):
        msg = self.master.recv_match(
            type='GLOBAL_POSITION_INT', blocking=True, timeout=1
        )
        if msg:
            return (msg.lat / 1e7, msg.lon / 1e7, msg.relative_alt / 1000.0)
        return None

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
