import time
from pymavlink import mavutil


class DroneWorker:

    def __init__(self, sys_id, port, target):
        self.sys_id = sys_id
        self.port = port
        self.target = target    # (lat, lon, alt)
        self.master = None

    def log(self, msg):
        print(f"[Drone {self.sys_id}] {msg}", flush=True)

    def connect(self, timeout=30):
        self.log(f"Port {self.port} üzerinden bağlanılıyor (udpout)...")
        self.master = mavutil.mavlink_connection(
            f'udpout:127.0.0.1:{self.port}',
            source_system=255,
            source_component=190
        )

        # Bridge / PX4'ün GCS'i tanıması için heartbeat gönder
        for _ in range(10):
            self.master.mav.heartbeat_send(
                mavutil.mavlink.MAV_TYPE_GCS,
                mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                0, 0, 0
            )
            time.sleep(0.1)

        hb = self.master.wait_heartbeat(timeout=timeout)
        if not hb:
            raise RuntimeError(f"Port {self.port} üzerinde heartbeat alınamadı!")

        self.log(f"Bağlantı başarılı — sysid={self.master.target_system} "
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

    def start_offboard(self, attempts=15):
        """
        Setpoint akışı başlatır ve PX4'ün gerçekten OFFBOARD moduna
        girdiğini heartbeat'ten doğrular.
        PX4 OFFBOARD custom_mode = 393216 (0x60000)
        """
        self.log("OFFBOARD için setpoint akışı başlatılıyor...")

        # PX4'ün mod değiştirmesi için en az 2 sn kesintisiz setpoint gerekli
        for _ in range(30):
            self.send_setpoint(*self.target)
            time.sleep(0.1)

        for attempt in range(1, attempts + 1):
            # OFFBOARD mod komutu gönder
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_DO_SET_MODE,
                0,
                1,      # MAV_MODE_FLAG_CUSTOM_MODE_ENABLED
                6,      # PX4 OFFBOARD custom mode
                0, 0, 0, 0, 0
            )
            self.log(f"OFFBOARD komutu gönderildi (deneme {attempt}/{attempts})")

            # Komut sonrası setpoint gönderirken heartbeat'ten modu doğrula
            for _ in range(20):
                self.send_setpoint(*self.target)
                hb = self.master.recv_match(type='HEARTBEAT', blocking=False)
                if hb and hb.custom_mode == 393216:
                    self.log("OFFBOARD modu onaylandı!")
                    return
                time.sleep(0.1)

            self.log(f"OFFBOARD henüz aktif değil, tekrar deneniyor...")

        self.log("UYARI: OFFBOARD modu onaylanamadı, devam ediliyor.")

    def arm(self, retries=5):
        """
        Motorları ARM eder. ARM beklerken setpoint akışını kesmez
        (OFFBOARD düşmesin diye).
        """
        for attempt in range(1, retries + 1):
            self.log(f"ARM denemesi {attempt}/{retries}...")
            self.master.mav.command_long_send(
                self.master.target_system,
                self.master.target_component,
                mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM,
                0,
                1, 0, 0, 0, 0, 0, 0
            )

            # ACK beklerken 10 Hz setpoint gönder (OFFBOARD canlı kalsın)
            start = time.time()
            while time.time() - start < 4.0:
                self.send_setpoint(*self.target)
                ack = self.master.recv_match(type='COMMAND_ACK', blocking=False)
                if ack and ack.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM:
                    if ack.result == mavutil.mavlink.MAV_RESULT_ACCEPTED:
                        self.log("Motorlar aktif (ARMED).")
                        return
                    else:
                        self.log(f"ARM reddedildi (result={ack.result}), tekrar denenecek...")
                        break
                time.sleep(0.1)

            # ARM başarısızsa setpoint göndererek bekle (OFFBOARD düşmesin)
            self.log("Setpoint akışı sürdürülüyor...")
            for _ in range(20):
                self.send_setpoint(*self.target)
                time.sleep(0.1)

        self.log("UYARI: ARM onayı alınamadı, göreve devam ediliyor.")

    def get_position(self):
        msg = self.master.recv_match(type='GLOBAL_POSITION_INT', blocking=True, timeout=1)
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
        self.log(f"Hedefe gidiliyor: ({lat:.7f}, {lon:.7f}, {alt}m)")
        dist = float('inf')
        for i in range(max_iter):
            self.send_setpoint(lat, lon, alt)
            time.sleep(0.1)
            dist = self.distance_to_target(lat, lon, alt)
            if dist < tolerance:
                self.log(f"Hedefe varıldı (Mesafe: {dist:.2f}m)")
                return
            if i % 50 == 0:
                self.log(f"  Kalan mesafe: {dist:.2f}m")
        self.log(f"Zaman aşımı! Hedefe tam varılamadı: {dist:.2f}m")

    def return_to_launch(self):
        self.log("RTL (Eve Dön) komutu verildi.")
        self.master.mav.command_long_send(
            self.master.target_system,
            self.master.target_component,
            mavutil.mavlink.MAV_CMD_NAV_RETURN_TO_LAUNCH,
            0,
            0, 0, 0, 0, 0, 0, 0
        )
        self.log("İniş bekleniyor...")
        for _ in range(120):
            msg = self.master.recv_match(
                type='EXTENDED_SYS_STATE', blocking=True, timeout=1
            )
            if msg and msg.landed_state == mavutil.mavlink.MAV_LANDED_STATE_ON_GROUND:
                self.log("Drone yere indi.")
                return
            time.sleep(1)
        self.log("RTL zaman aşımı, drone hala iniyor olabilir.")

    def run(self, waypoints=None):
        self.connect()
        self.start_offboard()
        self.arm()

        self.log(f"Kalkış yüksekliğine çıkılıyor: {self.target[2]}m")
        self.goto_waypoint(*self.target)

        if waypoints:
            self.log(f"Görev başlatılıyor: {len(waypoints)} waypoint")
            for idx, wp in enumerate(waypoints):
                self.log(f"WP {idx+1}/{len(waypoints)} gidiliyor...")
                self.goto_waypoint(*wp)
                time.sleep(1)
        else:
            self.log("Waypoint yok, kısa süre hover.")
            time.sleep(2)

        self.return_to_launch()
        self.log("Görev tamamlandı.")
