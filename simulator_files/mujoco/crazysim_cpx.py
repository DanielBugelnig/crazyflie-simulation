#!/usr/bin/env python3
"""
CrazySim CPX Bridge — emulates the AI-deck ESP32 WiFi interface.

This script acts as an interpreter between CPX/TCP clients (e.g. cflib
fpv.py, aideck_streamer.py) and the CrazySim SITL environment:

  ┌─────────────┐         ┌──────────────┐         ┌──────────────┐
  │  fpv.py     │  CPX    │              │  CRTP   │  crazysim.py │
  │  (cflib)    │◄──TCP──►│ crazysim_cpx │◄──UDP──►│  cflib port  │
  │             │  :5000  │   (ESP32)    │  :19850 │  (passthru)  │
  └─────────────┘         │              │         └──────────────┘
                          │              │  raw     ┌──────────────┐
                          │              │◄──TCP───│  crazysim.py │
                          │              │  :5100  │  camera render│
                          └──────────────┘         └──────────────┘

CPX functions handled:
  CRTP (3) : bidirectional — unwrap/wrap CRTP packets, forward via UDP
  APP  (5) : camera frames — receive raw frames, wrap in CPX, stream out

Usage:
    python3 crazysim_cpx.py [options]

    # defaults: CPX TCP on :5000, cflib UDP on :19850, frames from :5100
    python3 crazysim_cpx.py
    python3 crazysim_cpx.py --cpx-port 5000 --cflib-port 19850 --frame-port 5100
"""

import argparse
import os
import random
import socket
import struct
import threading
import time

# ---------------------------------------------------------------------------
# CPX protocol constants  (matches cflib/cpx and AI-deck firmware)
# ---------------------------------------------------------------------------
CPX_T_STM32 = 1
CPX_T_ESP32 = 2
CPX_T_HOST  = 3
CPX_T_GAP8  = 4

CPX_F_SYSTEM    = 1
CPX_F_CONSOLE   = 2
CPX_F_CRTP      = 3
CPX_F_WIFI_CTRL = 4
CPX_F_APP       = 5

CPX_HEADER_SIZE = 2   # routing byte + function byte
CPX_MTU         = 1022  # WiFi CPX max payload

CPX_IMG_MAGIC = 0xBC
# Gazebo frames get a 9-byte APP packet [magic u8][stamp_us u64] ahead of the image header:
# the simulation time the frame was rendered. Clients that only look for the 11-byte image
# header skip it.
CPX_STAMP_MAGIC = 0xBD

# Frame chunks from the simulator. Gazebo: [magic u16][seq u16][total u16][width u16]
# [height u16][stamp_us u64][wx f32][wy f32][wz f32] (body rates in rad/s).
# MuJoCo: [seq u16][total u16][width u16][height u16]. Little endian.
FRAME_MAGIC = 0xCF01
FRAME_HEADER = struct.Struct('<HHHHHQfff')
FRAME_HEADER_OLD = struct.Struct('<HHHH')

PROFILE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'gazebo', 'noise_profiles')


def load_profile(name):
    """Noise profile given as a path or as a name in simulator_files/gazebo/noise_profiles."""
    import yaml
    for path in (name, os.path.join(PROFILE_DIR, name), os.path.join(PROFILE_DIR, name + '.yaml')):
        if os.path.isfile(path):
            with open(path) as f:
                return yaml.safe_load(f) or {}
    raise FileNotFoundError(f"noise profile '{name}' not found")


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def cpx_build(source: int, dest: int, function: int,
              data: bytes, last: bool = True) -> bytes:
    """Build a CPX wire packet: [length:u16][routing:1][function:1][data]."""
    route = ((source & 0x7) << 3) | (dest & 0x7)
    if last:
        route |= 0x40
    func = function & 0x3F  # version = 0
    payload = struct.pack('<BB', route, func) + data
    return struct.pack('<H', len(payload)) + payload


def cpx_parse_header(header_4: bytes):
    """Parse 4-byte CPX wire header → (payload_len, src, dst, last, func)."""
    length, route, func_byte = struct.unpack('<HBB', header_4)
    dst  = route & 0x07
    src  = (route >> 3) & 0x07
    last = bool(route & 0x40)
    func = func_byte & 0x3F
    return length - CPX_HEADER_SIZE, src, dst, last, func


def tcp_recv_exact(sock: socket.socket, n: int) -> bytes:
    """Receive exactly n bytes; raises ConnectionError on disconnect."""
    buf = bytearray()
    while len(buf) < n:
        chunk = sock.recv(n - len(buf))
        if not chunk:
            raise ConnectionError('peer disconnected')
        buf.extend(chunk)
    return bytes(buf)


# ---------------------------------------------------------------------------
class CrazySimCPX:
    """ESP32 emulator — CPX TCP server + CRTP/UDP bridge + camera framer."""

    def __init__(self, cpx_port: int, cflib_port: int, frame_port: int,
                 host: str = '127.0.0.1', profile: str = '', cf_id: int = 0):
        self._cpx_port = cpx_port
        self._cflib_host = host
        self._cflib_port = cflib_port
        self._frame_port = frame_port
        self._host = host

        # CPX TCP client (single client, like the real AI-deck)
        self._cpx_client: socket.socket | None = None
        self._cpx_lock = threading.Lock()

        # CRTP UDP socket — connects to crazysim cflib passthrough port
        self._crtp_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self._crtp_sock.settimeout(0.5)
        self._crtp_addr = (host, cflib_port)
        # Set once a client sends CRTP; image-only clients never register on the cflib
        # port, so another cflib connection (e.g. udp://) can keep flying the drone
        self._crtp_registered = threading.Event()

        self._running = False

        # AI-deck camera model and WiFi link, from the camera section of the profile
        self._camera_model = None
        self._link = {'stream_fps': None, 'latency_s': 0.0, 'latency_jitter_s': 0.0, 'drop_prob': 0.0}
        self._last_frame_time = 0.0
        self._pending = []          # (send_time, width, height, pixels, stamp_us), ordered
        self._pending_cv = threading.Condition()
        camera = (load_profile(profile).get('camera') or {}) if profile else {}
        if camera.get('enabled', False):
            import numpy as np
            from camera_model import DEFAULTS, CameraModel
            self._np = np
            seed = int(load_profile(profile).get('seed', 0)) + cf_id + 2000
            self._camera_model = CameraModel({k: camera[k] for k in DEFAULTS if k in camera}, seed)
            self._link.update({k: camera[k] for k in self._link if camera.get(k) is not None})
            self._rng = random.Random(seed)
            self._warned_fov = False
            print(f'[cpx] Camera model from {profile}, link {self._link}')

    def run(self):
        self._running = True

        # Start CRTP UDP → CPX TCP forwarder (firmware responses → client)
        threading.Thread(target=self._crtp_to_cpx_loop, daemon=True).start()

        # Start camera frame receiver → CPX TCP streamer
        threading.Thread(target=self._frame_to_cpx_loop, daemon=True).start()

        # TCP server for CPX clients (blocking accept loop)
        srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        if hasattr(socket, 'SO_REUSEPORT'):
            srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
        srv.bind(('0.0.0.0', self._cpx_port))
        srv.listen(1)
        print(f'[cpx] CPX TCP server listening on tcp://0.0.0.0:{self._cpx_port}')
        print(f'[cpx] CRTP bridge → udp://{self._cflib_host}:{self._cflib_port}')
        print(f'[cpx] Camera frames ← tcp://{self._host}:{self._frame_port}')

        try:
            while self._running:
                srv.settimeout(1.0)
                try:
                    conn, addr = srv.accept()
                except socket.timeout:
                    continue
                conn.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                print(f'[cpx] Client connected from {addr}')

                with self._cpx_lock:
                    if self._cpx_client is not None:
                        try:
                            self._cpx_client.close()
                        except OSError:
                            pass
                    self._cpx_client = conn

                # Handle this client's incoming CPX packets (blocking)
                self._handle_cpx_client(conn)

                with self._cpx_lock:
                    self._cpx_client = None
                self._crtp_registered.clear()
                print(f'[cpx] Client disconnected')

        except KeyboardInterrupt:
            pass
        finally:
            self._running = False
            srv.close()
            self._crtp_sock.close()

    # ------------------------------------------------------------------
    # CPX TCP → CRTP UDP  (client sends CRTP commands)
    # ------------------------------------------------------------------
    def _handle_cpx_client(self, conn: socket.socket):
        """Read CPX packets from TCP client, dispatch by function."""
        try:
            while self._running:
                hdr = tcp_recv_exact(conn, 4)
                data_len, src, dst, last, func = cpx_parse_header(hdr)
                data = tcp_recv_exact(conn, data_len) if data_len > 0 else b''

                if func == CPX_F_CRTP:
                    if not self._crtp_registered.is_set():
                        print(f'[cpx] Sending cflib handshake to {self._crtp_addr}')
                        self._crtp_sock.sendto(bytes([0xFF]), self._crtp_addr)
                        self._crtp_registered.set()
                    # Forward raw CRTP data to firmware via cflib UDP port
                    self._crtp_sock.sendto(data, self._crtp_addr)

                elif func == CPX_F_SYSTEM:
                    pass  # bridge enable etc. — not needed in sim

                elif func == CPX_F_WIFI_CTRL:
                    pass  # WiFi setup — not needed in sim

        except (ConnectionError, OSError):
            pass

    # ------------------------------------------------------------------
    # CRTP UDP → CPX TCP  (firmware responses back to client)
    # ------------------------------------------------------------------
    def _crtp_to_cpx_loop(self):
        """Forward CRTP UDP responses from firmware → CPX TCP client."""
        while self._running:
            # Nothing to receive until a client has registered us on the cflib port
            if not self._crtp_registered.wait(timeout=0.5):
                continue

            try:
                data, _ = self._crtp_sock.recvfrom(1024)
            except socket.timeout:
                continue
            except OSError:
                break

            # Skip cflib passthrough handshake echoes
            if len(data) == 1 and data[0] == 0xFF:
                continue

            with self._cpx_lock:
                client = self._cpx_client
            if client is None:
                continue

            pkt = cpx_build(CPX_T_STM32, CPX_T_HOST, CPX_F_CRTP,
                            data, last=True)
            try:
                client.sendall(pkt)
            except (BrokenPipeError, ConnectionResetError, OSError):
                pass

    # ------------------------------------------------------------------
    # Camera frames (UDP) → CPX APP packets (TCP)
    # ------------------------------------------------------------------
    def _frame_to_cpx_loop(self):
        """Receive UDP frame chunks from crazysim, reassemble, wrap in CPX."""
        frame_sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        frame_sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        frame_sock.bind(('127.0.0.1', self._frame_port))
        frame_sock.settimeout(0.5)
        print(f'[cpx] Camera frame receiver on udp://127.0.0.1:{self._frame_port}')
        threading.Thread(target=self._delayed_sender_loop, daemon=True).start()

        chunks = {}  # seq → data
        expected_total = 0

        while self._running:
            try:
                pkt, _ = frame_sock.recvfrom(65535)
            except socket.timeout:
                continue
            except OSError:
                break

            if len(pkt) >= FRAME_HEADER.size and struct.unpack_from('<H', pkt)[0] == FRAME_MAGIC:
                _, seq, total, width, height, stamp_us, wx, wy, wz = FRAME_HEADER.unpack_from(pkt)
                chunk_data = pkt[FRAME_HEADER.size:]
                rates = (wx, wy, wz)
                frame_stamp_us = stamp_us
            elif len(pkt) >= FRAME_HEADER_OLD.size:
                seq, total, width, height = FRAME_HEADER_OLD.unpack_from(pkt)
                chunk_data = pkt[FRAME_HEADER_OLD.size:]
                rates = None
                frame_stamp_us = None
            else:
                continue

            # New frame starting
            if seq == 0:
                chunks = {}
                expected_total = total
            chunks[seq] = chunk_data

            # Frame complete?
            if len(chunks) < expected_total:
                continue

            # Reassemble
            pixels = b''.join(chunks[i] for i in range(expected_total))
            chunks = {}
            self._on_frame(width, height, pixels, rates, frame_stamp_us)

        frame_sock.close()

    def _on_frame(self, width, height, pixels, rates, stamp_us=None):
        """Camera model, then the WiFi link: frame rate, drops, latency."""
        if self._camera_model is None:
            self._send_frame(width, height, pixels, stamp_us)
            return
        link = self._link
        now = time.monotonic()
        if link['stream_fps'] and now - self._last_frame_time < 1.0 / link['stream_fps']:
            return
        self._last_frame_time = now
        if self._rng.random() < link['drop_prob']:
            return
        frame = self._np.frombuffer(pixels, self._np.uint8).reshape(height, width)
        out = self._camera_model.process(frame, rates)       # may be smaller than the render
        height, width = out.shape
        pixels = out.tobytes()
        if self._camera_model.distortion_outside > 0 and not self._warned_fov:
            self._warned_fov = True
            print(f'[cpx] Lens distortion: {self._camera_model.distortion_outside:.0%} of the pixels see '
                  f'rays outside the rendered image (black); raise camera.horizontal_fov')
        delay = max(0.0, link['latency_s'] + self._rng.gauss(0.0, link['latency_jitter_s']))
        if delay == 0.0:
            self._send_frame(width, height, pixels, stamp_us)
            return
        with self._pending_cv:
            # frames keep their order, like on a TCP stream
            send_time = max(now + delay, self._pending[-1][0] if self._pending else 0.0)
            self._pending.append((send_time, width, height, pixels, stamp_us))
            self._pending_cv.notify()

    def _delayed_sender_loop(self):
        while self._running:
            with self._pending_cv:
                if not self._pending:
                    self._pending_cv.wait(0.5)
                    continue
                wait = self._pending[0][0] - time.monotonic()
                if wait > 0:
                    self._pending_cv.wait(wait)
                    continue
                _, width, height, pixels, stamp_us = self._pending.pop(0)
            self._send_frame(width, height, pixels, stamp_us)

    def _send_frame(self, width, height, pixels, stamp_us=None):
        size = len(pixels)
        with self._cpx_lock:
            client = self._cpx_client
        if client is None:
            return

        # Build CPX APP image header (11 bytes)
        img_header = struct.pack('<BHHBBI',
                                CPX_IMG_MAGIC,
                                width, height,
                                8, 0, size)
        header_pkt = cpx_build(CPX_T_GAP8, CPX_T_HOST,
                              CPX_F_APP, img_header, last=False)
        if stamp_us is not None:
            header_pkt = cpx_build(CPX_T_GAP8, CPX_T_HOST, CPX_F_APP,
                                   struct.pack('<BQ', CPX_STAMP_MAGIC, stamp_us), last=False) + header_pkt

        # Split pixel data into CPX MTU-sized chunks
        data_pkts = bytearray()
        offset = 0
        while offset < size:
            chunk = pixels[offset:offset + CPX_MTU]
            offset += len(chunk)
            is_last = offset >= size
            data_pkts.extend(cpx_build(CPX_T_GAP8, CPX_T_HOST,
                                       CPX_F_APP, chunk, last=is_last))

        try:
            client.sendall(header_pkt + bytes(data_pkts))
        except (BrokenPipeError, ConnectionResetError, OSError):
            pass


# ---------------------------------------------------------------------------
def main():
    p = argparse.ArgumentParser(
        description='CrazySim CPX Bridge — AI-deck ESP32 emulator',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=__doc__)
    p.add_argument('--cpx-port', type=int, default=5050,
                   help='TCP port for CPX clients like fpv.py (default: 5050)')
    p.add_argument('--cflib-port', type=int, default=19850,
                   help='UDP port of crazysim cflib passthrough (default: 19850)')
    p.add_argument('--frame-port', type=int, default=5200,
                   help='UDP port for camera frames from crazysim (default: 5200)')
    p.add_argument('--host', default='127.0.0.1',
                   help='Host where crazysim.py is running (default: 127.0.0.1)')
    p.add_argument('--profile', default='',
                   help='noise profile with a camera section (path or name in gazebo/noise_profiles)')
    p.add_argument('--cf-id', type=int, default=0, help='drone number, for the per-drone seed')
    args = p.parse_args()

    CrazySimCPX(
        cpx_port=args.cpx_port,
        cflib_port=args.cflib_port,
        frame_port=args.frame_port,
        host=args.host,
        profile=args.profile,
        cf_id=args.cf_id,
    ).run()


if __name__ == '__main__':
    main()
