# 🚁 Sistem Bağlantı Kılavuzu

## Port Haritası

```
multiple_run_baylands.sh -m iris -n 7

  Drone N  →  MAVLink UDP portu: 14560 + N
  ─────────────────────────────────────────
  Drone 1  →  14561   sysid=2
  Drone 2  →  14562   sysid=3
  Drone 3  →  14563   sysid=4
  Drone 4  →  14564   sysid=5
  Drone 5  →  14565   sysid=6
  Drone 6  →  14566   sysid=7
  Drone 7  →  14567   sysid=8

  master.py bağlantı portları: 14541–14547  (offboard kontrol)
```

---

## Akış Şeması

```
PX4 SITL × 7
   ↓ UDP 14561–14567 (telemetri)
   ↓ UDP 14541–14547 (offboard kontrol ← master.py)
telemetry_bridge.py
   ↓ WebSocket ws://localhost:4000/v1/mavlink
drone-command-center.html
```

---

## Adım Adım Çalıştırma

### Terminal 1 — Gazebo + PX4 SITL

```bash
cd ~/PX4-Autopilot   # veya px4 kurulu dizin
source ./Tools/simulation/gazebo-classic/multiple_run_baylands.sh -m iris -n 7
```

> Gazebo penceresi açılır, 7 drone spawn olur. **QGroundControl açmana gerek yok.**
> Ancak açık olması zarar vermez (opsiyonel debug için).

---

### Terminal 2 — Telemetri Köprüsü (YENİ)

```bash
cd ~/Downloads/autonomous-multi-drone-swarm-px4

# İlk kullanımda bağımlılıkları yükle:
pip install pymavlink websockets

# Köprüyü başlat:
python3 telemetry_bridge.py
```

Çıktı şöyle görünmeli:
```
[Bridge] WebSocket sunucu başlatıldı: ws://0.0.0.0:4000/v1/mavlink
[Drone@14561] Connecting...
[Drone@14562] Connecting...
...
[Drone@14561] Connected — sysid=2
[Drone@14562] Connected — sysid=3
```

---

### Terminal 3 — Drone Kontrol (isteğe bağlı)

```bash
cd ~/Downloads/autonomous-multi-drone-swarm-px4
python3 master.py
```

> Bu script drone'ları arm edip waypoint'lere uçurur.
> Telemetri bridge'den **bağımsız** çalışır — ikisi aynı anda olabilir.

---

### Tarayıcı — Arayüz

```
drone-command-center.html dosyasını tarayıcıda aç

WebSocket URL alanı:  ws://localhost:4000/v1/mavlink  (varsayılan)
▶ CONNECT butonuna bas
```

7 drone'un kartları sol panelde, telemetri sağ panelde görünür.

---

## Sorun Giderme

| Sorun | Çözüm |
|-------|-------|
| `[Drone@14561] No heartbeat` | PX4 henüz başlamamış, 10-15sn bekle |
| Arayüzde drone görünmüyor | `telemetry_bridge.py` çalışıyor mu? Port numarası doğru mu? |
| `master.py` bağlanamıyor | `multiple_run_baylands.sh` tam başladı mı? `netstat -ul \| grep 1454` ile portları kontrol et |
| WebSocket ERROR | Tarayıcı konsolunu aç (F12), `ws://` URL'ini kontrol et |

---

## Port Doğrulama Komutu

```bash
# PX4 UDP portlarının açık olup olmadığını kontrol et:
ss -ulnp | grep -E '1456[1-7]|1454[1-7]'
```
