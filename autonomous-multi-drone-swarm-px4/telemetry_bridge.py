#!/usr/bin/env python3
"""
telemetry_bridge.py — MAVLink Router + WebSocket Köprüsü

Mimari:
  PX4 SITL (14581-14587)
       ↕  udpout (GCS bağlantısı)
  [Bu Bridge]
       ├─ WebSocket ws://localhost:4000   → drone-command-center.html
       └─ UDP 14541-14547 (local)        → master.py / drone_worker.py

master.py artık 14541-14547'e bağlanır (eski portlar korundu).
Bridge, master.py ile PX4 arasında mesajları yönlendirir.

Kullanım:
  pip install pymavlink websockets
  python3 telemetry_bridge.py

  Ayrı terminalde:
  python3 master.py
"""

import asyncio
import json
import socket
import threading
import time
import sys

try:
    from pymavlink import mavutil
except ImportError:
    print("pymavlink yüklü değil: pip install pymavlink")
    sys.exit(1)

try:
    import websockets
except ImportError:
    print("websockets yüklü değil: pip install websockets")
    sys.exit(1)

# ─── CONFIG ──────────────────────────────────────────────────────────────────
WS_HOST  = "0.0.0.0"
WS_PORT  = 4000

# PX4 → Bridge (GCS portları, lsof ile doğrulandı)
PX4_PORTS = list(range(14581, 14588))   # 14581..14587

# Bridge → master.py (local proxy portları, master.py bu portlara bağlanır)
PROXY_PORTS = list(range(14541, 14548)) # 14541..14547

RECONNECT_DELAY = 5

TELEM_MSGS = {
    'GLOBAL_POSITION_INT', 'SYS_STATUS', 'VFR_HUD',
    'ATTITUDE', 'HEARTBEAT', 'ATTITUDE_TARGET', 'BATTERY_STATUS',
}

# ─── GLOBALS ─────────────────────────────────────────────────────────────────
_loop    = None
_queue   = None
_clients = set()

# Drone bağlantıları: {px4_port: mavutil_connection}
_drone_conns = {}

# ─── WEBSOCKET ────────────────────────────────────────────────────────────────
async def ws_handler(websocket, path=None):
    """websockets ≥10 sadece websocket argümanı geçirir; path=None default ile her iki sürüm de çalışır."""
    _clients.add(websocket)
    print(f"[WS] Bağlandı: {getattr(websocket,'remote_address','?')}")
    try:
        async for _ in websocket:
            pass
    except Exception:
        pass
    finally:
        _clients.discard(websocket)
        print(f"[WS] Ayrıldı: {getattr(websocket,'remote_address','?')}")


async def broadcast_worker():
    while True:
        msg = await _queue.get()
        if _clients:
            data = json.dumps(msg)
            dead = set()
            for ws in list(_clients):
                try:
                    await ws.send(data)
                except Exception:
                    dead.add(ws)
            _clients.difference_update(dead)
        _queue.task_done()


def push_msg(msg_json):
    if _loop and _queue:
        asyncio.run_coroutine_threadsafe(_queue.put(msg_json), _loop)


# ─── MAVLink JSON ─────────────────────────────────────────────────────────────
def mav_to_json(msg, system_id):
    t = msg.get_type()
    d = msg.to_dict()
    d.pop('mavpackettype', None)
    return {"header": {"system_id": system_id, "component_id": 1, "message_id": 0},
            "message": {"type": t, **d}}


# ─── PX4 LISTENER + ROUTER ───────────────────────────────────────────────────
def drone_listener(px4_port, proxy_port):
    """
    PX4'e (px4_port) bağlanır, telemetriyi WebSocket'e iletir.
    master.py'den gelen komutları (proxy_port) PX4'e yönlendirir.
    """
    # master.py için local UDP proxy soketi (UDPin - bind)
    proxy_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    proxy_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    proxy_sock.settimeout(0.05)
    try:
        proxy_sock.bind(('127.0.0.1', proxy_port))
        print(f"[Proxy] Port {proxy_port} açıldı → Drone@{px4_port}")
    except OSError as e:
        print(f"[Proxy] Port {proxy_port} açılamadı: {e}")

    master_addr = None  # master.py'nin adresi (ilk paketten öğrenilir)

    while True:
        try:
            print(f"[Drone@{px4_port}] Bağlanıyor...")
            mav = mavutil.mavlink_connection(
                f'udpout:127.0.0.1:{px4_port}',
                source_system=255,
                source_component=190
            )

            # PX4'e tanıt
            for _ in range(10):
                mav.mav.heartbeat_send(
                    mavutil.mavlink.MAV_TYPE_GCS,
                    mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                time.sleep(0.1)

            hb = mav.wait_heartbeat(timeout=15)
            if not hb:
                print(f"[Drone@{px4_port}] Heartbeat yok, {RECONNECT_DELAY}s sonra...")
                time.sleep(RECONNECT_DELAY)
                continue

            sysid = mav.target_system
            print(f"[Drone@{px4_port}] Bağlandı — sysid={sysid}  (proxy→master: {proxy_port})")
            _drone_conns[px4_port] = mav

            # Telemetri akışı iste
            try:
                mav.mav.request_data_stream_send(
                    sysid, 1, mavutil.mavlink.MAV_DATA_STREAM_ALL, 10, 1)
            except Exception:
                pass

            last_hb = time.time()

            while True:
                # 1) master.py'den gelen komutları PX4'e ilet
                try:
                    raw, addr = proxy_sock.recvfrom(65535)
                    master_addr = addr
                    # Paketi PX4'e gönder (raw MAVLink bytes)
                    try:
                        mav.write(raw)
                    except Exception:
                        pass
                except socket.timeout:
                    pass

                # 2) PX4'ten gelen telemetriyi oku
                msg = mav.recv_match(
                    type=list(TELEM_MSGS | {'COMMAND_ACK', 'COMMAND_LONG',
                                            'SET_POSITION_TARGET_GLOBAL_INT',
                                            'EXTENDED_SYS_STATE'}),
                    blocking=False
                )
                if msg:
                    src = msg.get_srcSystem() or sysid
                    msg_type = msg.get_type()

                    # Telemetri → WebSocket
                    if msg_type in TELEM_MSGS:
                        push_msg(mav_to_json(msg, src))

                    # Her mesajı master.py'ye de gönder
                    if master_addr:
                        try:
                            proxy_sock.sendto(msg.get_msgbuf(), master_addr)
                        except Exception:
                            pass
                else:
                    time.sleep(0.005)

                # Heartbeat gönder (PX4 bağlantısını canlı tut)
                if time.time() - last_hb > 1.0:
                    try:
                        mav.mav.heartbeat_send(
                            mavutil.mavlink.MAV_TYPE_GCS,
                            mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
                    except Exception:
                        break
                    last_hb = time.time()

        except Exception as e:
            print(f"[Drone@{px4_port}] Hata: {e} — {RECONNECT_DELAY}s sonra yeniden")
            _drone_conns.pop(px4_port, None)
            time.sleep(RECONNECT_DELAY)


# ─── MAIN ─────────────────────────────────────────────────────────────────────
async def main():
    global _loop, _queue
    _loop  = asyncio.get_running_loop()
    _queue = asyncio.Queue()

    # websockets ≥10'da serve() signature değişti; her iki sürümle uyumlu çağır
    try:
        server = await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    except TypeError:
        server = await websockets.serve(ws_handler, WS_HOST, WS_PORT)
    print(f"[Bridge] WebSocket: ws://localhost:{WS_PORT}  (operation.html → ws://localhost:{WS_PORT}/v1/mavlink da çalışır)")
    print(f"[Bridge] PX4 portları: {PX4_PORTS}")
    print(f"[Bridge] master.py proxy portları: {PROXY_PORTS}")
    print("-" * 60)

    for px4_port, proxy_port in zip(PX4_PORTS, PROXY_PORTS):
        t = threading.Thread(
            target=drone_listener,
            args=(px4_port, proxy_port),
            daemon=True, name=f"mav-{px4_port}"
        )
        t.start()

    asyncio.ensure_future(broadcast_worker())
    await server.wait_closed()


if __name__ == "__main__":
    print("=" * 60)
    print("  DRONE TELEMETRY BRIDGE + MAVLINK ROUTER")
    print("=" * 60)
    print()
    print("Sıra:")
    print("  1) multiple_run_baylands.sh -m iris -n 7")
    print("  2) python3 telemetry_bridge.py   ← (bu script)")
    print("  3) python3 master.py             ← (artık çakışmaz)")
    print("  4) drone-command-center.html → otomatik bağlanır")
    print()
    try:
        asyncio.run(main())
    except KeyboardInterrupt:
        print("\n[Bridge] Kapatılıyor...")
